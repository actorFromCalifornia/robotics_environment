import json
import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    import serial
    from serial import SerialException
except ModuleNotFoundError as exc:  # pragma: no cover
    serial = None  # type: ignore
    SerialException = Exception  # type: ignore
    raise


class WaveshareMotorDriver(Node):
    """Bridge cmd_vel to the Waveshare sub-controller via JSON-over-serial."""

    def __init__(self) -> None:
        super().__init__('waveshare_motor_driver')

        self.declare_parameter('command_topic', 'cmd_vel')
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('serial_timeout', 0.1)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 3.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('watchdog_period', 0.1)
        self.declare_parameter('reconnect_period', 1.0)
        self.declare_parameter('log_incoming', False)

        topic = self.get_parameter('command_topic').value
        self.subscription = self.create_subscription(
            Twist,
            topic,
            self._on_cmd_vel,
            10,
        )

        self.serial_lock = threading.Lock()
        self.serial_handle: Optional[serial.Serial] = None  # type: ignore[assignment]
        self.reader_thread: Optional[threading.Thread] = None
        self.reader_stop = threading.Event()
        self.last_cmd_time = self.get_clock().now()
        self.last_sent: Optional[Tuple[float, float, float]] = None
        self.zero_sent = False

        watchdog_period = float(self.get_parameter('watchdog_period').value)
        reconnect_period = float(self.get_parameter('reconnect_period').value)
        self.watchdog_timer = self.create_timer(watchdog_period, self._watchdog_tick)
        self.reconnect_timer = self.create_timer(reconnect_period, self._ensure_serial)

        self._ensure_serial(initial=True)
        self.get_logger().info(f"Subscribed to '{topic}' for velocity commands")

    # ------------------------------------------------------------------
    def _ensure_serial(self, initial: bool = False) -> None:
        if self.serial_handle and self.serial_handle.is_open:
            return

        port = str(self.get_parameter('port').value)
        baudrate = int(self.get_parameter('baudrate').value)
        timeout = float(self.get_parameter('serial_timeout').value)
        try:
            handle = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)  # type: ignore[arg-type]
        except SerialException as exc:
            if initial:
                self.get_logger().error(f"Failed to open serial port {port}: {exc}")
            else:
                self.get_logger().debug(f"Serial reconnect failed on {port}: {exc}")
            return

        self.serial_handle = handle
        self._start_reader_thread()
        self.get_logger().info(f"Serial port {port} opened at {baudrate} baud")

        if bool(self.get_parameter('auto_start').value):
            self._send_start(True)
            time.sleep(0.05)
        self._send_velocity(0.0, 0.0, 0.0, force=True)

    def _start_reader_thread(self) -> None:
        if self.reader_thread and self.reader_thread.is_alive():
            return

        self.reader_stop.clear()
        thread = threading.Thread(target=self._read_loop, name='waveshare_serial_reader', daemon=True)
        self.reader_thread = thread
        thread.start()

    def _read_loop(self) -> None:
        while not self.reader_stop.is_set():
            handle = self.serial_handle
            if not handle:
                time.sleep(0.1)
                continue
            try:
                raw = handle.readline()
            except SerialException as exc:
                self.get_logger().warning(f"Serial read error: {exc}")
                self._handle_serial_failure()
                continue

            if not raw:
                continue

            if bool(self.get_parameter('log_incoming').value):
                try:
                    decoded = raw.decode('utf-8', errors='ignore').strip()
                except Exception:
                    decoded = str(raw)
                if decoded:
                    self.get_logger().info(f"Controller: {decoded}")

    def _handle_serial_failure(self) -> None:
        with self.serial_lock:
            if self.serial_handle:
                try:
                    self.serial_handle.close()
                except Exception:
                    pass
                self.serial_handle = None
        self.reader_stop.set()
        self.reader_thread = None

    # ------------------------------------------------------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        max_linear = float(self.get_parameter('max_linear').value)
        max_angular = float(self.get_parameter('max_angular').value)

        x = float(msg.linear.x)
        z = float(msg.angular.z)
        y = float(msg.linear.y)

        if not math.isfinite(x):
            x = 0.0
        if not math.isfinite(z):
            z = 0.0
        if not math.isfinite(y):
            y = 0.0

        if abs(y) < 1e-6:
            y = 0.0

        if max_linear > 0.0:
            x = max(-max_linear, min(max_linear, x))
        if max_angular > 0.0:
            z = max(-max_angular, min(max_angular, z))

        self.last_cmd_time = self.get_clock().now()
        self.zero_sent = False
        self._send_velocity(x, y, z)

    def _watchdog_tick(self) -> None:
        timeout_sec = float(self.get_parameter('cmd_timeout').value)
        if timeout_sec <= 0:
            return

        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed >= timeout_sec and not self.zero_sent:
            self.get_logger().warn('cmd_vel timeout reached, stopping motors')
            self._send_velocity(0.0, 0.0, 0.0, force=True)
            self.zero_sent = True

    def _send_velocity(self, x: float, y: float, z: float, force: bool = False) -> None:
        if not force and self.last_sent == (x, y, z):
            return

        payload = {"T": 13, "X": float(x), "Z": float(z)}
        if y != 0.0:
            payload['Y'] = float(y)

        self._send_payload(payload)
        self.last_sent = (x, y, z)

    def _send_start(self, enabled: bool) -> None:
        payload = {"T": 131, "cmd": 1 if enabled else 0}
        self._send_payload(payload, quiet=not enabled)

    def _send_payload(self, payload: dict, quiet: bool = False) -> None:
        handle = self.serial_handle
        if not handle or not handle.is_open:
            if not quiet:
                self.get_logger().error('Serial port is not open, cannot send command')
            return

        message = json.dumps(payload, separators=(',', ':')) + '\r\n'
        data = message.encode('ascii', errors='ignore')
        try:
            with self.serial_lock:
                handle.write(data)
        except SerialException as exc:
            self.get_logger().error(f"Serial write error: {exc}")
            self._handle_serial_failure()

    # ------------------------------------------------------------------
    def destroy_node(self) -> bool:
        try:
            self._send_velocity(0.0, 0.0, 0.0, force=True)
            if bool(self.get_parameter('auto_start').value):
                self._send_start(False)
        finally:
            self.reader_stop.set()
            if self.reader_thread and self.reader_thread.is_alive():
                self.reader_thread.join(timeout=0.5)
            if self.serial_handle:
                try:
                    self.serial_handle.close()
                except Exception:
                    pass
                self.serial_handle = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaveshareMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':  # pragma: no cover
    main()
