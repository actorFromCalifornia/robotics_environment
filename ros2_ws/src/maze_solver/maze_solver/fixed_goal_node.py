import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.duration import Duration
import math
import random
from tf_transformations import quaternion_from_euler

class FixedGoalSetter(Node):
    def __init__(self) -> None:
        super().__init__('fixed_goal_setter')
        
        
        # Параметры с значениями по умолчанию
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 2.0)
        self.declare_parameter('goal_yaw', 0.0)  # в радианах
        self.declare_parameter('max_attempts', 3)
        self.declare_parameter('retry_radius', 0.1)  # 10 см
        
        # Получение параметров
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_yaw = self.get_parameter('goal_yaw').value
        self.max_attempts = self.get_parameter('max_attempts').value
        self.retry_radius = self.get_parameter('retry_radius').value
        
        self.get_logger().info(f'Задана цель: x={self.goal_x}, y={self.goal_y}, yaw={self.goal_yaw}')
        
        # Клиент для действия навигации
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # Переменные состояния
        self.attempt_count = 0
        self.current_goal_x = self.goal_x
        self.current_goal_y = self.goal_y
        self.current_goal_yaw = self.goal_yaw
        self.retry_timer = None
        
        # Таймер для отправки первой цели после инициализации
        self.initial_timer = self.create_timer(10.0, self.initial_goal_callback)
    
    def initial_goal_callback(self):
        """Отправляет первую цель после инициализации"""
        self.initial_timer.cancel()
        self.send_goal(self.current_goal_x, self.current_goal_y, self.current_goal_yaw)
    
    def generate_random_offset(self):
        """Генерирует случайное смещение в радиусе retry_radius"""
        angle = random.uniform(0, 2 * math.pi)
        distance = random.uniform(0, self.retry_radius)
        
        offset_x = distance * math.cos(angle)
        offset_y = distance * math.sin(angle)
        offset_yaw = random.uniform(-0.17, 0.17)  # ~10 градусов
        
        return offset_x, offset_y, offset_yaw
    
    def send_goal(self, x, y, yaw):
        """Отправляет цель навигации"""
        if self.attempt_count >= self.max_attempts:
            self.get_logger().error(f'Достигнуто максимальное количество попыток ({self.max_attempts}). Прекращаю попытки.')
            return
        
        self.attempt_count += 1
        
        # Преобразуем эйлеровы углы в кватернион
        quaternion = quaternion_from_euler(0, 0, yaw)
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = quaternion[0]
        goal_msg.pose.pose.orientation.y = quaternion[1]
        goal_msg.pose.pose.orientation.z = quaternion[2]
        goal_msg.pose.pose.orientation.w = quaternion[3]
        
        self.get_logger().info(f'Попытка {self.attempt_count}: отправляю цель x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Обработчик ответа на цель"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Цель отклонена навигатором')
            self.retry_with_offset()
            return
        
        self.get_logger().info('Цель принята, робот начал движение')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Обработчик результата выполнения цели"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info('✅ Цель успешно достигнута!')
            self.retry_with_offset()
        else:
            self.get_logger().warn(f'❌ Не удалось достичь цели. Статус: {status}')
            self.retry_with_offset()
    
    def feedback_callback(self, feedback_msg):
        """Обработчик обратной связи о прогрессе"""
        feedback = feedback_msg.feedback
        # Можно добавить логирование прогресса при необходимости
        # self.get_logger().info(f'Текущее расстояние до цели: {feedback.distance_remaining:.2f} м')
    
    def delayed_retry_callback(self):
        """Колбэк для отложенной повторной попытки"""
        if self.retry_timer:
            self.retry_timer.cancel()
            self.retry_timer = None
        self.send_goal(self.current_goal_x, self.current_goal_y, self.current_goal_yaw)
    
    def retry_with_offset(self):
        """Повторная попытка со случайным смещением"""
        if self.attempt_count < self.max_attempts:
            offset_x, offset_y, offset_yaw = self.generate_random_offset()
            
            self.current_goal_x = self.goal_x + offset_x
            self.current_goal_y = self.goal_y + offset_y
            self.current_goal_yaw = self.goal_yaw + offset_yaw
            
            self.get_logger().info(f'Случайное смещение: Δx={offset_x:.2f}, Δy={offset_y:.2f}, Δyaw={offset_yaw:.2f}')
            
            # Создаем таймер для отложенной отправки (1 секунда)
            if self.retry_timer:
                self.retry_timer.cancel()
            self.retry_timer = self.create_timer(1.0, self.delayed_retry_callback)
        else:
            self.get_logger().error('❌ Все попытки достичь цель исчерпаны')

def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedGoalSetter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
