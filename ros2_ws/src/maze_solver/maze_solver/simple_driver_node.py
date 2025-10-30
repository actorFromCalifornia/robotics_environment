import copy
import math
from array import array
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from visualization_msgs.msg import Marker, MarkerArray


# no yaw/odom needed in pure FTG mode


class SimpleDriver(Node):
    def __init__(self):
        super().__init__('simple_driver')

        # Topics/loop
        self.declare_parameter('topic', 'cmd_vel')
        self.declare_parameter('rate_hz', 15.0)

        # Скоростной профиль (FTG)
        self.declare_parameter('v_max', 0.40)
        self.declare_parameter('v_min', 0.05)
        self.declare_parameter('front_stop', 0.10)
        self.declare_parameter('front_slow', 0.20)
        self.declare_parameter('side_clip', 10.0)  # для диагностики секторами

        # Sector geometry (radians)
        self.declare_parameter('front_half_width', math.radians(8.0))
        self.declare_parameter('side_half_width', math.radians(8.0))
        self.declare_parameter('left_center', math.radians(+45.0))
        self.declare_parameter('right_center', math.radians(-45.0))

        # Без FSM — никаких порогов/таймингов состояний

        # FTG parameters (используются и в CORRIDOR, и в TURN_FTG)
        # k_steer: коэффициент поворота, w = k_steer * угол_цели (рад/с на рад)
        self.declare_parameter('k_steer', 1.0)
        # w_max: ограничение по модулю угловой скорости |w|
        self.declare_parameter('w_max', 1.0)
        # range_clip: обрезка дальних значений лидара (м); шумные/бесконечные → 0.0
        self.declare_parameter('range_clip', 4.0)
        # w_safe: безопасная ширина (ширина робота + отступ), м
        self.declare_parameter('w_safe', 0.3)
        # bubble_r0: учитывать минимумы ближе этого радиуса (м)
        self.declare_parameter('bubble_r0', 0.3)
        # min_gap_beams: минимальная длина (в лучах) непрерывного «просвета», чтобы считаться валидным проходом
        self.declare_parameter('min_gap_beams', 20)
        # front_window_beams: число лучей вокруг 0° для оценки фронтального зазора (регулировка скорости)
        self.declare_parameter('front_window_beams', 5)
        # turn_slowdown_gain: дополнительное замедление от величины угла поворота (0..1)
        self.declare_parameter('turn_slowdown_gain', 2.0)
        # polar morphological smoothing (скругление препятствий)
        self.declare_parameter('enable_polar_morph', True)
        self.declare_parameter('morph_radius', 0.1)
        self.declare_parameter('morph_stride', 2)

        # FSM: dead-end detection and turn-around
        self.declare_parameter('dead_end_front', 0.33)
        self.declare_parameter('dead_end_side', 0.3)
        self.declare_parameter('turn_w', 0.6)
        self.declare_parameter('turn_min_time', 0.5)
        self.declare_parameter('turn_timeout', 6.0)

        topic = self.get_parameter('topic').value
        hz = float(self.get_parameter('rate_hz').value)

        # Publishers/subscribers
        self.pub = self.create_publisher(Twist, topic, 10)
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.dbg_diag_pub = self.create_publisher(DiagnosticArray, '/debug/diagnostics', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/debug/markers', 10)

        # Buffers/state
        self.last_scan = None
        self.last_log = 0.0
        self.state = 'GO'  # GO, TURN
        self.state_enter_ts = 0.0
        self.turn_sign = +1

        period = 1.0 / max(1.0, hz)
        self.timer = self.create_timer(period, self.tick)
        self.get_logger().info(f"SimpleDriver FTG → publish '{topic}' at {hz} Hz")

    # ----------------- ROS callbacks -----------------
    def on_scan(self, msg: LaserScan):
        self.last_scan = msg

    # ----------------- Helpers (диагностика секторами) -----------------
    def _sector_min(self, scan: LaserScan, center: float, half_width: float):
        # Robust minimum in sector [center ± half_width] using 20% quantile
        a_min = scan.angle_min
        a_inc = scan.angle_increment
        n = len(scan.ranges)
        if n == 0 or a_inc == 0.0:
            return None

        lo = max(a_min, center - half_width)
        hi = min(scan.angle_max, center + half_width)
        i0 = int(max(0, math.floor((lo - a_min) / a_inc)))
        i1 = int(min(n - 1, math.ceil((hi - a_min) / a_inc)))
        if i1 <= i0:
            return None

        vals = []
        rmin = scan.range_min if scan.range_min > 0 else 0.0
        rmax = scan.range_max if scan.range_max > 0 else float('inf')
        side_clip = float(self.get_parameter('side_clip').value)
        rclip = min(side_clip, rmax)
        for i in range(i0, i1 + 1):
            r = scan.ranges[i]
            if r != r or r <= 0.0 or math.isinf(r):
                continue
            if r < rmin:
                continue
            vals.append(min(r, rclip))
        if not vals:
            return None
        vals.sort()
        idx = max(0, int(0.2 * (len(vals) - 1)))
        return vals[idx]

    # Нет открытия/угла — без FSM

    def _publish_diag(self, v: float, w: float,
                       d_front, d_left, d_right,
                       left_open: bool, right_open: bool,
                       state: str, mode: str):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus()
        st.level = DiagnosticStatus.OK
        st.name = 'simple_driver'
        st.message = state if mode is None else f"{state}:{mode}"
        def kv(k, v):
            kvp = KeyValue(); kvp.key = k; kvp.value = str(v); return kvp
        st.values.extend([
            kv('v', f"{v:.3f}"),
            kv('w', f"{w:.3f}"),
            kv('front', 'nan' if d_front is None else f"{float(d_front):.3f}"),
            kv('left', 'nan' if d_left is None else f"{float(d_left):.3f}"),
            kv('right', 'nan' if d_right is None else f"{float(d_right):.3f}"),
            kv('left_open', bool(left_open)),
            kv('right_open', bool(right_open)),
        ])
        # Добавим FTG-диагностику, если доступна
        try:
            fi = getattr(self, 'ftg_info', None)
            if fi is not None:
                w_safe = float(self.get_parameter('w_safe').value)
                r0 = float(self.get_parameter('bubble_r0').value)

                def compact(arr):
                    return '[' + ','.join(f"{float(x):.2f}" for x in arr) + ']'
                st.values.extend([
                    kv('ftg_best_idx', 'nan' if fi.get('best_idx') is None else int(fi['best_idx'])),
                    kv('ftg_best_deg', 'nan' if fi.get('best_deg') is None else f"{float(fi['best_deg']):.2f}"),
                    kv('ftg_front', 'nan' if fi.get('front') is None else f"{float(fi['front']):.3f}"),
                    kv('ftg_gap', 'none' if fi.get('gap') is None else str(tuple(fi['gap']))),
                    kv('ftg_mask_seeds', int(fi.get('mask_seeds') or 0)),
                    kv('ftg_w_safe', f"{w_safe:.3f}"),
                    kv('ftg_r0', f"{r0:.3f}"),
                    kv('ftg_morph_radius', float(self.get_parameter('morph_radius').value)),
                    kv('ftg_rng_pre', 'none' if fi.get('rng_pre') is None else compact(fi['rng_pre'])),
                    kv('ftg_rng_masked', 'none' if fi.get('rng_masked') is None else compact(fi['rng_masked'])),
                    kv('ftg_rng_morph', 'none' if fi.get('rng_morph') is None else compact(fi['rng_morph'])),
                ])
        except Exception:
            pass
        arr.status.append(st)
        self.dbg_diag_pub.publish(arr)

    def _publish_markers(self, scan: LaserScan, best_idx, best_ang, rng_masked, rng_morph, gap):
        if self.marker_pub is None:
            return

        frame_id = 'base_link'
        stamp = self.get_clock().now().to_msg()
        if scan is not None:
            frame_id = scan.header.frame_id or frame_id
            stamp = scan.header.stamp

        markers = MarkerArray()

        if scan is None or best_ang is None or best_idx is None:
            clear = Marker()
            clear.header.frame_id = frame_id
            clear.header.stamp = stamp
            clear.ns = 'ftg'
            clear.id = 0
            clear.action = Marker.DELETEALL
            markers.markers.append(clear)
            self.marker_pub.publish(markers)
            return

        angle_min = float(scan.angle_min)
        angle_max = float(scan.angle_max)
        angle_inc = float(scan.angle_increment)
        length_limit = scan.range_max if scan.range_max and scan.range_max > 0.0 else 3.0
        length_limit = max(0.5, min(length_limit, 6.0))

        best_len = length_limit
        if rng_morph is not None and 0 <= best_idx < len(rng_morph):
            rng_val = float(rng_morph[best_idx])
            if rng_val > 0.0:
                best_len = max(0.3, min(rng_val, length_limit))

        def make_point(angle: float, radius: float) -> Point:
            return Point(x=radius * math.cos(angle),
                         y=radius * math.sin(angle),
                         z=0.0)

        origin = Point()

        best_marker = Marker()
        best_marker.header.frame_id = frame_id
        best_marker.header.stamp = stamp
        best_marker.ns = 'ftg'
        best_marker.id = 1
        best_marker.type = Marker.LINE_LIST
        best_marker.action = Marker.ADD
        best_marker.scale.x = 0.02
        best_marker.color.r = 1.0
        best_marker.color.g = 0.1
        best_marker.color.b = 0.1
        best_marker.color.a = 1.0
        best_marker.points.extend([origin, make_point(best_ang, best_len)])
        markers.markers.append(best_marker)

        boundary_marker = Marker()
        boundary_marker.header.frame_id = frame_id
        boundary_marker.header.stamp = stamp
        boundary_marker.ns = 'ftg'
        boundary_marker.id = 2
        boundary_marker.type = Marker.LINE_LIST
        boundary_marker.action = Marker.ADD
        boundary_marker.scale.x = 0.01
        boundary_marker.color.r = 0.2
        boundary_marker.color.g = 0.6
        boundary_marker.color.b = 1.0
        boundary_marker.color.a = 0.8
        boundary_marker.points.extend([
            origin,
            make_point(angle_min, length_limit),
            origin,
            make_point(angle_max, length_limit),
        ])
        markers.markers.append(boundary_marker)

        if gap is not None:
            g0, g1 = gap
            if g0 is not None and g1 is not None and g0 >= 0 and g1 >= g0:
                gap_marker = Marker()
                gap_marker.header.frame_id = frame_id
                gap_marker.header.stamp = stamp
                gap_marker.ns = 'ftg'
                gap_marker.id = 3
                gap_marker.type = Marker.LINE_LIST
                gap_marker.action = Marker.ADD
                gap_marker.scale.x = 0.01
                gap_marker.color.r = 0.1
                gap_marker.color.g = 1.0
                gap_marker.color.b = 0.1
                gap_marker.color.a = 0.7
                start_angle = angle_min + g0 * angle_inc
                end_angle = angle_min + g1 * angle_inc
                gap_marker.points.extend([
                    origin,
                    make_point(start_angle, length_limit),
                    origin,
                    make_point(end_angle, length_limit),
                ])
                markers.markers.append(gap_marker)

        if rng_morph is not None:
            stride = max(1, int(self.get_parameter('morph_stride').value))
            strip_marker = Marker()
            strip_marker.header.frame_id = frame_id
            strip_marker.header.stamp = stamp
            strip_marker.ns = 'ftg'
            strip_marker.id = 4
            strip_marker.type = Marker.LINE_STRIP
            strip_marker.action = Marker.ADD
            strip_marker.scale.x = 0.01
            strip_marker.color.r = 1.0
            strip_marker.color.g = 0.8
            strip_marker.color.b = 0.2
            strip_marker.color.a = 0.7
            for idx in range(0, len(rng_morph), stride):
                r_val = rng_morph[idx]
                if r_val <= 0.0:
                    continue
                ang = angle_min + idx * angle_inc
                strip_marker.points.append(make_point(ang, min(r_val, length_limit)))
            markers.markers.append(strip_marker)

        self.marker_pub.publish(markers)

    # ----------------- Helpers (FTG) -----------------
    def _preprocess(self, scan: LaserScan):
        n = len(scan.ranges)
        if n == 0 or scan.angle_increment == 0.0:
            return None
        rclip = float(self.get_parameter('range_clip').value)
        rng = []
        for v in scan.ranges:
            if v != v or v <= 0.0 or math.isinf(v):
                rng.append(0.0)
            else:
                v = max(scan.range_min if scan.range_min > 0 else 0.0,
                        min(v, min(rclip, scan.range_max if scan.range_max > 0 else v)))
                rng.append(v)
        return rng

    def _apply_polar_morph(self, rng, scan: LaserScan):
        if not bool(self.get_parameter('enable_polar_morph').value):
            return list(rng)

        radius = float(self.get_parameter('morph_radius').value)
        if radius <= 0.0:
            return list(rng)

        n = len(rng)
        if n == 0:
            return list(rng)

        angle_inc = abs(float(scan.angle_increment))
        if angle_inc <= 0.0:
            return list(rng)

        # Минимальная дистанция, используемая для оценки окна (коридор)
        d_min = None
        for v in rng:
            if v > 0.0:
                if d_min is None or v < d_min:
                    d_min = v
        if d_min is None or d_min <= 0.0:
            return list(rng)

        delta_angle = math.atan(max(0.0, radius) / max(1e-6, d_min))
        if delta_angle <= 0.0:
            return list(rng)
        window = max(1, int(math.ceil(delta_angle / angle_inc)))

        out = [0.0] * n
        for i in range(n):
            best = None
            for di in range(-window, window + 1):
                j = i + di
                if j < 0 or j >= n:
                    continue
                rj = rng[j]
                if rj <= 0.0:
                    continue
                delta = abs(di) * angle_inc
                cos_delta = math.cos(delta)
                proj = rj * cos_delta - radius
                if proj <= 0.0:
                    proj = 0.0
                if best is None or proj < best:
                    best = proj
            out[i] = max(0.05, best) if best is not None else 0.0

        return out

    def _build_radial_mask(self, rng, scan: LaserScan):
        n = len(rng)
        if n == 0:
            return list(rng), 0

        angle_inc = abs(float(scan.angle_increment))
        if angle_inc <= 0.0:
            return list(rng), 0

        w_safe = float(self.get_parameter('w_safe').value)
        if w_safe <= 0.0:
            return list(rng), 0

        half_width = 0.5 * w_safe
        bubble_limit = float(self.get_parameter('bubble_r0').value)

        masked = list(rng)
        mask = [False] * n
        seeds = 0

        for i, r in enumerate(rng):
            if r <= 0.0:
                continue
            if bubble_limit > 0.0 and r > bubble_limit:
                continue

            seeds += 1
            ratio = half_width / max(1e-6, r)
            if ratio >= 1.0:
                half_angle = math.pi / 4.0
            else:
                half_angle = math.asin(ratio)
            window = max(0, int(math.ceil(half_angle / angle_inc)))

            start = max(0, i - window)
            end = min(n - 1, i + window)
            for j in range(start, end + 1):
                mask[j] = True

        for idx in range(n):
            if mask[idx]:
                masked[idx] = 0

        return masked, seeds

    def _largest_gap(self, rng_mask, rng_morph, min_len):
        # Учитываем только индексы, где и rng_mask > 0, и rng_morph > 0
        best = (-1, -1)
        cur_start = None
        n = min(len(rng_mask), len(rng_morph))
        for i in range(n):
            open_i = (rng_mask[i] * rng_morph[i]) > 0.0
            if open_i:
                if cur_start is None:
                    cur_start = i
            else:
                if cur_start is not None:
                    if i - cur_start > best[1] - best[0]:
                        best = (cur_start, i - 1)
                    cur_start = None
        if cur_start is not None:
            if n - cur_start > best[1] - best[0]:
                best = (cur_start, n - 1)
        if best[0] == -1 or best[1] - best[0] + 1 < min_len:
            return None
        return best

    def _front_clearance(self, arr, scan: LaserScan):
        n = len(arr)
        if n == 0:
            return 0.0
        center = int(round((0.0 - scan.angle_min) / scan.angle_increment))
        w = int(self.get_parameter('front_window_beams').value)
        i0 = max(0, center - w)
        i1 = min(n - 1, center + w)
        vals = [v for v in arr[i0:i1 + 1] if v > 0.0]
        if not vals:
            return 0.0
        return min(vals)

    def _ftg_control(self, scan: LaserScan):
        # Приводим сырые значения дальностей к диапазону, с которым удобно работать:
        # заполняем мусорные и бесконечные измерения нулями, обрезаем дальние значения.
        rng_pre = self._preprocess(scan)
        if rng_pre is None:
            return 0.0, 0.0, None, None, 0.0, None, 0, None, None, None
        rng_mask, seeds_cnt = self._build_radial_mask(rng_pre, scan)
        rng_morph = self._apply_polar_morph(rng_pre, scan)
        # В обнулённом массиве ищем максимально длинный просвет, в который можно ехать.
        gap = self._largest_gap(rng_mask, rng_morph, int(self.get_parameter('min_gap_beams').value))
        if gap is None:
            return 0.0, 0.0, None, None, 0.0, None, seeds_cnt, rng_pre, rng_mask, rng_morph
        g0, g1 = gap
        # Среди лучей внутри выбранного просвета ищем тот, где дальность максимальна — туда «смотрит» робот.
        best_idx = max(range(g0, g1 + 1), key=lambda i: rng_morph[i])
        best_ang = scan.angle_min + best_idx * scan.angle_increment
        # Преобразуем целевой угол в угловую скорость: пропорциональный регулятор с насыщением.
        k_steer = float(self.get_parameter('k_steer').value)
        w_maxp = float(self.get_parameter('w_max').value)
        w = max(-w_maxp, min(w_maxp, k_steer * best_ang))
        # Линейная скорость зависит от фронтального зазора: чем ближе препятствие, тем сильнее замедляемся.
        v_max = float(self.get_parameter('v_max').value)
        v_min = float(self.get_parameter('v_min').value)
        front_stop = float(self.get_parameter('front_stop').value)
        front_slow = float(self.get_parameter('front_slow').value)
        front = self._front_clearance(rng_pre, scan)
        if front <= front_stop:
            v = 0.0
        elif front < front_slow:
            v = v_min + (v_max - v_min) * ((front - front_stop) / max(1e-6, (front_slow - front_stop)))
        else:
            v = v_max
        # Дополнительное замедление на крутых поворотах: остаётся минимум 20% от базовой скорости.
        slow_gain = float(self.get_parameter('turn_slowdown_gain').value)
        span = max(1e-6, (scan.angle_max - scan.angle_min))
        v *= max(0.2, 1.0 - slow_gain * min(1.0, abs(best_ang) / span))
        return v, w, best_idx, best_ang, front, (g0, g1), seeds_cnt, rng_pre, rng_mask, rng_morph

    # ----------------- Main control loop (pure FTG) -----------------
    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        scan_raw = self.last_scan

        # Parameters snapshot
        v_max = float(self.get_parameter('v_max').value)
        v_min = float(self.get_parameter('v_min').value)
        front_stop = float(self.get_parameter('front_stop').value)
        front_slow = float(self.get_parameter('front_slow').value)

        cmd = Twist()
        if scan_raw is None:
            cmd.linear.x = 0.05
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            # Diagnostics (no data)
            self._publish_diag(cmd.linear.x, cmd.angular.z, None, None, None, False, False, 'FTG', 'init')
            return

        rng_filtered = self._preprocess(scan_raw)
        if rng_filtered is None:
            cmd.linear.x = 0.05
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            self._publish_diag(cmd.linear.x, cmd.angular.z, None, None, None, False, False, 'FTG', 'bad_scan')
            return

        scan = copy.deepcopy(scan_raw)
        scan.ranges = array('f', rng_filtered)

        # Distances from sectors
        fhw = float(self.get_parameter('front_half_width').value)
        shw = float(self.get_parameter('side_half_width').value)
        left_c = float(self.get_parameter('left_center').value)
        right_c = float(self.get_parameter('right_center').value)
        d_front = self._sector_min(scan, 0.0, fhw)
        d_left = self._sector_min(scan, left_c, shw)
        d_right = self._sector_min(scan, right_c, shw)

        # Управление FTG (базовая оценка, используется и в FSM)
        v_ftg, w_ftg, best_idx, best_ang, front, gap, seeds_cnt, rng_pre, rng_masked, rng_morph = self._ftg_control(scan)
        self.ftg_info = {
            'best_idx': best_idx,
            'best_deg': math.degrees(best_ang) if best_ang is not None else None,
            'front': front,
            'gap': gap,
            'mask_seeds': seeds_cnt,
            'rng_pre': rng_pre,
            'rng_masked': rng_masked,
            'rng_morph': rng_morph,
        }
        self._publish_markers(scan, best_idx, best_ang, rng_masked, rng_morph, gap)

        # FSM transitions/outputs
        dead_front_th = float(self.get_parameter('dead_end_front').value)
        dead_side_th = float(self.get_parameter('dead_end_side').value)
        is_dead = (
            (d_front is not None and d_front <= dead_front_th) and
            (d_left is not None and d_left <= dead_side_th) and
            (d_right is not None and d_right <= dead_side_th)
        )

        # Default outputs from FTG (GO behavior)
        v_out = v_ftg
        w_out = w_ftg
        mode = 'go'

        if self.state == 'GO':
            if is_dead:
                # Enter TURN state
                self.state = 'TURN'
                self.state_enter_ts = now
                # Выбираем направление разворота: туда, где чуть больше место
                if (d_left or 0.0) > (d_right or 0.0):
                    self.turn_sign = +1  # влево (CCW)
                else:
                    self.turn_sign = -1  # вправо (CW)
            else:
                # Остаёмся в GO, едем по FTG
                pass

        if self.state == 'TURN':
            mode = 'turn'
            w_turn = float(self.get_parameter('turn_w').value)
            v_out = 0.0
            w_out = self.turn_sign * abs(w_turn)

            min_time = float(self.get_parameter('turn_min_time').value)
            timeout = float(self.get_parameter('turn_timeout').value)
            turn_time = now - self.state_enter_ts

            # Условие выхода: есть валидный gap (как в FTG) и прошло минимальное время
            if gap is not None:
                self.state = 'GO'
                self.state_enter_ts = now
                mode = 'go'
                v_out = v_ftg
                w_out = w_ftg
            # elif turn_time >= timeout:
            #     # Failsafe: переключимся на FTG даже без gap
            #     self.state = 'GO'
            #     self.state_enter_ts = now
            #     mode = 'go'
            #     v_out = v_ftg
            #     w_out = w_ftg

        # Clamp/publish
        w_out = max(-1.0, min(1.0, w_out))
        v_out = max(-0.3, min(v_max, v_out))
        cmd.linear.x = float(v_out)
        cmd.angular.z = float(w_out)
        self.pub.publish(cmd)

        # Diagnostics
        self._publish_diag(v_out, w_out, d_front, d_left, d_right, False, False, 'FTG', mode)

        # Logging
        if now - self.last_log > 2.0:
            def fmt(x):
                return f"{x:.2f}" if x is not None else 'nan'
            self.get_logger().info(
                f"FTG[{self.state}/{mode}]: v={v_out:.2f} w={w_out:.2f} front={fmt(d_front)} left={fmt(d_left)} right={fmt(d_right)} best_idx={self.ftg_info.get('best_idx')} best={self.ftg_info.get('best_deg')}° gap={self.ftg_info.get('gap')}")
            self.last_log = now


def main():
    import sys, traceback
    try:
        rclpy.init()
        n = SimpleDriver()
        rclpy.spin(n)
        n.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"[simple_driver] ERROR: {e}", file=sys.stderr, flush=True)
        traceback.print_exc()
        raise


if __name__ == '__main__':
    main()
