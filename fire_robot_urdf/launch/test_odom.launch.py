import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros
from transformations import quaternion_from_euler
import math
from zlac8015d import ZLAC8015D  # 這是你提供的ZLAC8015D驅動程式

class MotorControllerNode(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.controller = ZLAC8015D(port="/dev/ttyUSB0")
        
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.timer = self.create_timer(0.1, self.update_state)
        
        self.left_wheel_joint_name = "left_wheel_joint"
        self.right_wheel_joint_name = "right_wheel_joint"
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.previous_left_tick, self.previous_right_tick = self.controller.get_wheels_tick()

    def update_state(self):
        # 獲取當前輪子的轉速
        l_vel, r_vel = self.controller.get_linear_velocities()
        
        # 計算里程計信息
        dt = 0.1
        vx = (l_vel + r_vel) / 2.0
        vth = (r_vel - l_vel) / (2 * self.controller.R_Wheel)
        
        delta_x = vx * math.cos(self.th) * dt
        delta_y = vx * math.sin(self.th) * dt
        delta_th = vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # 創建並發布JointState消息
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [self.left_wheel_joint_name, self.right_wheel_joint_name]
        joint_state.position = [self.controller.get_wheels_tick()[0], self.controller.get_wheels_tick()[1]]
        
        self.joint_state_pub.publish(joint_state)
        
        # 創建並發布Odometry消息
        odom_quat = quaternion_from_euler(0, 0, self.th)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*odom_quat)
        
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        
        self.odom_pub.publish(odom)
        
        # 發布TF變換
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(*odom_quat)
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
