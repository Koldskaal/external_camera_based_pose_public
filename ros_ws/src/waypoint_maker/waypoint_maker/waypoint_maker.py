import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from scipy.spatial.transform import Rotation as R
import math
import random
import time

waypoints = [
  {"x": 30.0, "y":30.0, "theta":0.0},
  {"x": 160.0, "y":30.0, "theta":0.0},
  {"x": 160.0, "y":120.0, "theta":0.0},
  {"x": 30.0, "y":120.0, "theta":0.0},
]

class Nav2Client(Node):
    def __init__(self):
        super().__init__('nav2_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.poses = []
        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = waypoint["x"]/100
            pose.pose.position.y = waypoint["y"]/100
            pose.pose.position.z = 0.0
            
            theta = math.radians(random.randrange(0, 360))
            q = R.from_euler('xyz',[0, 0, theta]).as_quat()
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            
            self.poses.append(pose)
        
        self.current_index = 0
        
        self.send_goal(self.poses[self.current_index])

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            # feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Current progress: {feedback_msg.feedback}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.current_index = (self.current_index + 1) % len(self.poses)
        time.sleep(2)
        self.send_goal(self.poses[self.current_index])



def main(args=None):
    rclpy.init(args=args)
    node = Nav2Client()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
