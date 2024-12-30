#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped
from std_msgs.msg import String  # Change this if using a different message type
from nav_msgs.msg import Odometry
import struct


class TcpSenderNode(Node):
    def __init__(self):
        super().__init__('tcp_sender_node')

        # Set up the TCP connection parameters
        self.tcp_ip = '169.254.122.152'   # IP address of the TCP server
        self.tcp_port = 5001        # Port of the TCP server
        self.buffer_size = 1024     # Maximum data packet size
        self.get_logger().info("HEY")
        # Connect to the TCP server
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.tcp_ip, self.tcp_port))
            self.get_logger().info(f"Connected to TCP server at {self.tcp_ip}:{self.tcp_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to server: {e}")
            self.socket = None

        # Set up the subscriber to receive data from a ROS 2 topic
        self.cmd_vel_sub = self.create_subscription(
            Twist,             # Message type
            '/cmd_vel',   # Topic name to subscribe to
            self.send_to_tcp,   # Callback function
            10                  # QoS depth
        )
        self.get_logger().info("TCP Sender Node initialized and subscribed to /tcp_data_topic")
    
    def send_to_tcp(self, msg):
        if self.socket:
            try:
                # Convert the message to a string and encode it to bytes
                # print(msg)
                data = [msg.linear.x,msg.angular.z]

                print(data)
                data = struct.pack("!ff",msg.linear.x,msg.angular.z)
                # Send data to the TCP server
                self.socket.sendall(data)
                self.get_logger().info(f"Sent data to TCP server: {[msg.linear.x,msg.angular.z]}")
            except Exception as e:
                self.get_logger().error(f"Failed to send data: {e}")
                # Optionally close the socket and set it to None
                self.socket.close()
                self.socket = None
    def init_pose(self,msg:PoseWithCovarianceStamped):
        self.initial_pose = msg.pose.pose
    def destroy_node(self):
        # Clean up resources before shutting down
        if self.socket:
            self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TcpSenderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
