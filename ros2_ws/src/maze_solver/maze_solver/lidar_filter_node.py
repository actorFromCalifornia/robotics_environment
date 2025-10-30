#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')
        
        # Параметры с простыми типами
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('angle_min1', 0.0)    # градусы
        self.declare_parameter('angle_max1', 135.0)  # градусы
        self.declare_parameter('angle_min2', 225.0)  # градусы
        self.declare_parameter('angle_max2', 360.0)  # градусы
        self.declare_parameter('angle_min3', 160.0)  # градусы
        self.declare_parameter('angle_max3', 200.0)  # градусы
        
        # Получаем параметры
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        angle_min1 = self.get_parameter('angle_min1').get_parameter_value().double_value
        angle_max1 = self.get_parameter('angle_max1').get_parameter_value().double_value
        angle_min2 = self.get_parameter('angle_min2').get_parameter_value().double_value
        angle_max2 = self.get_parameter('angle_max2').get_parameter_value().double_value
        angle_min3 = self.get_parameter('angle_min3').get_parameter_value().double_value
        angle_max3 = self.get_parameter('angle_max3').get_parameter_value().double_value
        
        # Создаем угловые диапазоны в радианах
        self.angle_ranges = [
            {'min': math.radians(angle_min1), 'max': math.radians(angle_max1)},
            {'min': math.radians(angle_min2), 'max': math.radians(angle_max2)},
            {'min': math.radians(angle_min3), 'max': math.radians(angle_max3)}
        ]
        
        self.get_logger().info(f'Фильтрация угловых диапазонов:')
        self.get_logger().info(f'  {angle_min1}° - {angle_max1}°')
        self.get_logger().info(f'  {angle_min2}° - {angle_max2}°')
        self.get_logger().info(f'  {angle_min3}° - {angle_max3}°')
        
        # Подписчики и издатели
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            LaserScan,
            output_topic,
            10
        )
        
        self.get_logger().info(f'Lidar filter node запущен: {input_topic} -> {output_topic}')

    def angle_in_range(self, angle):
        """Проверяет, находится ли угол в одном из разрешенных диапазонов"""
        for range_obj in self.angle_ranges:
            if range_obj['min'] <= angle <= range_obj['max']:
                return True
        return False

    def scan_callback(self, msg):
        """Обработка входящих данных LaserScan"""
        try:
            # Создаем новый message
            filtered_scan = LaserScan()
            
            # Копируем заголовок и основные параметры
            filtered_scan.header = msg.header
            filtered_scan.range_min = msg.range_min
            filtered_scan.range_max = msg.range_max
            filtered_scan.angle_min = msg.angle_min
            filtered_scan.angle_max = msg.angle_max
            filtered_scan.angle_increment = msg.angle_increment
            filtered_scan.scan_time = msg.scan_time
            filtered_scan.time_increment = msg.time_increment
            
            # Рассчитываем индексы для фильтрации
            filtered_ranges = []
            filtered_intensities = []
            
            current_angle = msg.angle_min
            
            for i in range(len(msg.ranges)):
                # Нормализуем угол в диапазон [0, 2π]
                normalized_angle = current_angle
                while normalized_angle < 0:
                    normalized_angle += 2 * math.pi
                while normalized_angle >= 2 * math.pi:
                    normalized_angle -= 2 * math.pi
                
                # Проверяем, находится ли угол в разрешенном диапазоне
                if self.angle_in_range(normalized_angle):
                    filtered_ranges.append(msg.ranges[i])
                    if i < len(msg.intensities):
                        filtered_intensities.append(msg.intensities[i])
                else:
                    # Если угол не в диапазоне, устанавливаем 0 или range_max
                    filtered_ranges.append(float('inf'))  # или msg.range_max
                    if i < len(msg.intensities):
                        filtered_intensities.append(0.0)
                
                current_angle += msg.angle_increment
            
            # Устанавливаем отфильтрованные данные
            filtered_scan.ranges = filtered_ranges
            
            if len(filtered_intensities) > 0:
                filtered_scan.intensities = filtered_intensities
            
            # Публикуем отфильтрованное сообщение
            self.publisher.publish(filtered_scan)
            
        except Exception as e:
            self.get_logger().error(f'Ошибка при фильтрации данных: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    node = LidarFilterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
