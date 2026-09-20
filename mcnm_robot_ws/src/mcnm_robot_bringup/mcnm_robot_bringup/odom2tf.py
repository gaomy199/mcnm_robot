import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class Odom2TF(Node):
    def __init__(self):
        super().__init__('odom2tf')
        # TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        # 订阅 odom，使用 SensorData QoS（best_effort，与下位机匹配）
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info('odom2tf node started, subscribing to /odom')

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        # 时间戳和坐标系名称沿用 odom 消息
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id          # odom
        t.child_frame_id = msg.child_frame_id            # base_footprint
        # 平移
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        # 旋转
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = Odom2TF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():           # 只在还没 shutdown 的情况下 shutdown
            rclpy.shutdown()

if __name__ == '__main__':
    main()