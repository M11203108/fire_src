#!/usr/bin/env python3
from rclpy.node import Node
from zlac8015d import ZLAC8015D
from geometry_msgs.msg import Twist , Quaternion , TransformStamped
from tf_transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from pymodbus.exceptions import ModbusIOException
from tf2_ros import TransformBroadcaster

import time
import numpy as np
import math
import rclpy

WHEEL_DIAMETER = 180 /1000      # 馬達直徑（mm）
WHEEL_SPACING = 650 / 1000      # 馬達前後軸距（mm）
WHEEL_AXLESPACING = 661 / 1000  # 馬達左右軸距（mm）

class ReadOdomVelToDis(Node):
    def __init__(self):
        super().__init__('odom_vel_to_dis')

        
        # 宣告並獲取參數值
        self.declare_parameter('usart_port_name_0', '/dev/ttyUSB0')
        self.declare_parameter('usart_port_name_1', '/dev/ttyUSB1')
        self.declare_parameter('serial_baud_rate', 115200)
        self.declare_parameter('robot_frame_id', 'base_footprint')
        self.declare_parameter('odom_frame_id', 'odom_combined')
        self.declare_parameter('cmd_vel', 'cmd_vel')

        self.usart_port_name_0 = self.get_parameter('usart_port_name_0').value
        self.usart_port_name_1 = self.get_parameter('usart_port_name_1').value
        self.serial_baud_rate = self.get_parameter('serial_baud_rate').value
        self.robot_frame_id = self.get_parameter('robot_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel').value

        # 訂閱 cmd_vel 話題，並設置回調函數
        self.subscription = self.create_subscription(Twist, self.cmd_vel_topic, self.move_callback, 10)
        self.subscription  # 防止被垃圾回收

        # 創建 odom 話題的發佈器
        #self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.publisher_ = self.create_publisher(Odometry, 'odom_combined', 10)

        # 使用從參數中獲取的端口名稱來初始化控制器
        self.motorAB = ZLAC8015D.Controller(port=self.usart_port_name_0)
        self.motorCD = ZLAC8015D.Controller(port=self.usart_port_name_1)

        self.cpr = 16385  #每转脉冲数，示例值
        self.travel_in_one_rev = 0.09 * 2 * math.pi

        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("TransformBroadcaster initialized")
        
        # 初始化 Vx, Vy, Vz
        self.Vx = 0.0
        self.Vy = 0.0
        self.Vz = 0.0

        # 新增的計時器，每0.1秒（即10Hz）調用一次Vel_to_Dis函數
        self.timer_period = 0.1  # 秒
        self.timer = self.create_timer(self.timer_period, self.Vel_to_Dis)
        
        # 設置加速度和減速度時間
        self.a_time = 500
        self.b_time = 1000
        self.motorAB.set_accel_time(self.a_time, self.a_time)
        self.motorAB.set_decel_time(self.b_time, self.b_time)
        self.motorCD.set_accel_time(self.a_time, self.a_time)
        self.motorCD.set_decel_time(self.b_time, self.b_time)
    
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_time = time.time()

        # 設置馬達模式
        self.motorAB.set_mode(3)
        self.motorCD.set_mode(3)

        # 启用电机
        self.motorAB.enable_motor()
        self.motorCD.enable_motor()       
    
    def get_Vel_From_Encoder(self): #取得速度

        try:
            self.MOTOR_A_Encoder, self.MOTOR_B_Encoder = self.motorAB.get_linear_velocities()
            self.MOTOR_C_Encoder, self.MOTOR_D_Encoder = self.motorCD.get_linear_velocities()
        except ModbusIOException as e:
            self.get_logger().error(f"Failed to read motor velocities: {e}")
            return
        

        self.MOTOR_A_Encoder = -self.MOTOR_A_Encoder#馬達安裝方向相反故改為負號#輪編號不同需更改    
        self.MOTOR_B_Encoder = -self.MOTOR_B_Encoder#輪編號不同需更改
        self.MOTOR_C_Encoder = -self.MOTOR_C_Encoder#輪編號不同需更改
        self.MOTOR_D_Encoder = -self.MOTOR_D_Encoder#輪編號不同需更改

        self.Vx = (self.MOTOR_A_Encoder + self.MOTOR_B_Encoder + self.MOTOR_C_Encoder + self.MOTOR_D_Encoder) / 4
        self.Vy = (self.MOTOR_A_Encoder - self.MOTOR_B_Encoder - self.MOTOR_C_Encoder + self.MOTOR_D_Encoder) / 4
        self.Vz = (-self.MOTOR_A_Encoder + self.MOTOR_B_Encoder - self.MOTOR_C_Encoder + self.MOTOR_D_Encoder) / (2 * (WHEEL_AXLESPACING + WHEEL_SPACING))

    def Vel_to_Dis(self):
        self.get_Vel_From_Encoder()
        current_time = time.time()
        dt = current_time - self.prev_time

        if self.Vx == 0 and self.Vy == 0 and self.Vz == 0:
            delta_x = 0
            delta_y = 0
            delta_th = 0
        
        else:
            delta_x = self.Vx * dt
            delta_y = self.Vy * dt
            delta_th = self.Vz * dt

        self.x += delta_x * math.cos(self.theta) - delta_y * math.sin(self.theta)
        self.y += delta_x * math.sin(self.theta) + delta_y * math.cos(self.theta)
        self.theta += delta_th
        
        self.prev_time = current_time

        print(f"Current position -> x: {self.x}, y: {self.y}, theta: {self.theta}")

        self.publish_odometry()

        self.publish_tf()

    def publish_odometry(self):
        
       # 计算四元数
        quat = quaternion_from_euler(0, 0, self.theta)

        # 创建并发布Odometry消息
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom_combined"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # 将四元数的各个分量分别赋值给Quaternion对象
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        odom_msg.twist.twist.linear.x = self.Vx
        odom_msg.twist.twist.linear.y = self.Vy
        odom_msg.twist.twist.angular.z = self.Vz

        self.publisher_.publish(odom_msg)

    def publish_tf(self):

        quat = quaternion_from_euler(0, 0, self.theta)
        # 发布TF变换
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom_combined'
        t.child_frame_id = self.robot_frame_id

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

    def move_callback(self, msg: Twist):
        # 在這裡更新你的速度值，然後可以調用 `Vel_to_Dis()` 計算里程並發布
        vx = -msg.linear.x
        vy = msg.linear.y
        vz = msg.angular.z
        self.set_speeds(vx, vy, vz)        
        
    def set_speeds(self, vx, vy, vz):        
        vz_linear = float(vz * ((WHEEL_SPACING + WHEEL_AXLESPACING) / 2))
        speed_A = int(vx - vy - vz_linear)
        speed_B = int(vx + vy + vz_linear)
        speed_D = int(vx + vy - vz_linear)
        speed_C = int(vx - vy + vz_linear)

        print("speed_A", speed_A, ", speed_B:", speed_B, ", speed_C:", speed_C, ", speed_D", speed_D)

        cmdsAB = [speed_A, -speed_B]
        cmdsCD = [speed_C, -speed_D]

        self.motorAB.set_rpm(int(cmdsAB[0]), int(cmdsAB[1]))
        self.motorCD.set_rpm(int(cmdsCD[0]), int(cmdsCD[1]))


    def on_shutdown(self):
        self.motorAB.disable_motor()
        self.motorCD.disable_motor()

def main(args=None):
    rclpy.init(args=args)
    node = ReadOdomVelToDis()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()