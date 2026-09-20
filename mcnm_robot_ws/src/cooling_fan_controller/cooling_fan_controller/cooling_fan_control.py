import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import glob

class FanController(Node):
    def __init__(self):
        super().__init__('fan_controller')
        self.publisher = self.create_publisher(Int8, 'fan_pwm', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)  # 每2秒发一次
    
    def get_max_temp(self):
        # 读取所有thermal_zone,返回最高温度
        max_temp = 0
        for zone in glob.glob('/sys/devices/virtual/thermal/thermal_zone*/temp'):
            try:
                with open(zone) as f:
                    temp = int(f.read().strip()) / 1000
                    max_temp = max(max_temp, temp)
            except:
                pass
        return max_temp
    
    def temp_to_pwm(self, temp):
        # 温度到PWM映射
        if temp < 40:
            return 60
        elif temp < 50:
            return 60 + int((temp - 40) * 1.0)  # 60~70
        elif temp < 60:
            return 70 + int((temp - 50) * 0.5)  # 70~75
        elif temp < 70:
            return 75 + int((temp - 60) * 0.5)  # 75~80
        else:
            return 80
    
    def timer_callback(self):
        temp = self.get_max_temp()
        pwm = self.temp_to_pwm(temp)
        
        msg = Int8()
        msg.data = pwm
        self.publisher.publish(msg)
        
        self.get_logger().info(f'Temp: {temp:.1f}°C → Fan PWM: {pwm}%')

def main():
    rclpy.init()
    node = FanController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()