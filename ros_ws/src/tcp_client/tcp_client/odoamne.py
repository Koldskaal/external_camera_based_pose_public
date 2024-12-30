import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
import math

class TwistToOdom(Node):
    def __init__(self):
        super().__init__('twist_to_odom')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        self.filtered_odom = self.create_subscription(Odometry, '/odometry/filtered', self.publish_transform, 10)

        self.odom_publisher = self.create_publisher(Odometry, '/nav_msgs/Odometry', 10)
        self.init_pos_sub = self.create_subscription(PoseWithCovarianceStamped,"/initialpose",self.init_pose,10)
        self.broadcaster = TransformBroadcaster(self)

        # self.init_pose()
        self.previous_time = self.get_clock().now()
        
        # Initialize variables to keep track of position and orientation
    def publish_transform(self,msg:Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = msg.pose.pose.orientation.z
        current_time = self.get_clock().now()
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation = msg.pose.pose.orientation

        self.broadcaster.sendTransform(transform)
        # self.get_logger().info(f"Pub odom: = x={self.x:.2f} y={self.y:.2f} theta={self.theta:.2f}")
    def creat_odom(self, vx:float, vy:float, vtheta:float):
        # Create the odometry message
        current_time = self.get_clock().now()
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        
        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Convert theta to a quaternion
        q = R.from_euler('xyz',[0, 0, self.theta]).as_quat()
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        small_cov = 0.01
        # Set velocity
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.z = vtheta
        odom_msg.pose.covariance = [0.0]*36
        for i in range(6):
            odom_msg.pose.covariance[i*7] = small_cov
        odom_msg.twist.covariance = odom_msg.pose.covariance
        # Publish the odometry message
        self.odom_publisher.publish(odom_msg)
        
        # Update the previous time
        self.previous_time = current_time
        return odom_msg
    def init_pose(self,msg:PoseWithCovarianceStamped):
        self.initial_pose = msg.pose.pose
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        r = msg.pose.pose.orientation
        r = R.from_quat([r.x,r.y,r.z,r.w])

        r = r.as_euler('xyz')
        self.theta = r[2]
        self.publish_transform(self.creat_odom(0.0,0.0,0.0))

    def twist_callback(self, twist_msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds / 1e9  # Convert to seconds
        
        # Calculate the position and orientation based on velocity and time
        vx = twist_msg.linear.x
        vy = twist_msg.linear.y
        vtheta = twist_msg.angular.z
        
        self.x += vx * math.cos(self.theta) * dt
        self.y += vx * math.sin(self.theta) * dt
        self.theta += vtheta * dt

        self.creat_odom(vx,vy,vtheta)
        # self.publish_transform(self.creat_odom(vx,vy,vtheta))

def main(args=None):
    rclpy.init(args=args)
    twist_to_odom_node = TwistToOdom()
    rclpy.spin(twist_to_odom_node)
    twist_to_odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()