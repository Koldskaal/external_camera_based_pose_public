#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import struct
import math
from scipy.spatial.transform import Rotation as R


class PoseSenderNode(Node):
    def __init__(self):
        super().__init__('pose_sender_node')

        # Set up the TCP connection parameters
        self.tcp_ip = '172.25.16.1'   # IP address of the TCP server
        self.tcp_port = 5006        # Port of the TCP server
        self.get_logger().info("HEY")
        # Connect to the TCP server
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5)
            self.socket.connect((self.tcp_ip, self.tcp_port))
            self.get_logger().info(f"Connected to Pose server at {self.tcp_ip}:{self.tcp_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to server: {e}")
            self.socket = None

        self.timer = self.create_timer(0.05, self.update_callback)
        self.pose_publisher = self.create_publisher(Odometry,'/img_pose',10)
        self.get_logger().info("Pose Sender Node initialized")
        
    def pose_maker(self,x,y,theta):
        pose = Odometry()
        current_time = self.get_clock().now()
        pose.pose.pose.position.x = float(x) /100
        pose.pose.pose.position.y = float(y)/100
        pose.pose.pose.position.z = 0.0
        pose.header.stamp = current_time.to_msg()
        pose.header.frame_id = "odom"
        # Convert theta to a quaternion
        theta = -math.radians(theta-90)
        q = R.from_euler('xyz',[0, 0, theta]).as_quat()
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]
        small_cov = 0.01
        pose.pose.covariance = [0.0]*36
        for i in range(6):
            pose.pose.covariance[i*7] = small_cov
        return pose
    def update_callback(self):
        if self.socket == None:
            pose = self.pose_maker(0,0,0)
            # self.get_logger().info("aASDASDASDASDASD")

            # self.pose_publisher.publish(pose)
            return
            
        # Code you want to execute at each update
        data = self.socket.recv(6)
        x, y, theta = struct.unpack("!hhh", data)
        
        pose = self.pose_maker(x,y,theta)
        
        self.pose_publisher.publish(pose)
    
    def destroy_node(self):
        # Clean up resources before shutting down
        if self.socket:
            self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseSenderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()