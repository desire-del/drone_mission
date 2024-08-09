from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from squaternion import Quaternion
from gazebo_msgs.msg import ModelState
import time
import rclpy

GOAL_REACHED_DIST = 0.3
COLLISION_DIST = 0.35
TIME_DELTA = 0.2

last_odom = None

class GazeboEnv(Node):
    def __init__(self):
        super().__init__('env')
        print('Hi from GazeboEnv.')

        self.odom_x = 0
        self.odom_y = 0
        self.odom_z = 0

        self.goal_x = 20.0
        self.goal_y = 20.0
        self.goal_z = 30.0

        self.set_self_state = ModelState()
        self.set_self_state.model_name = 'iris'
        self.set_self_state.pose.position.x = 0.0
        self.set_self_state.pose.position.y = 0.0
        self.set_self_state.pose.position.z = 0.800
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 0.0
        # Set up the ROS publishers and subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.unpause = self.create_client(Empty, '/unpause_physics')
        self.pause = self.create_client(Empty, '/pause_physics')
        self.reset_proxy = self.create_client(Empty, '/reset_world')
        self.req = Empty.Request()

        self.publisher = self.create_publisher(MarkerArray, 'goal_point', 3)
        self.publisher2 = self.create_publisher(MarkerArray, "linear_velocity", 1)
        self.publisher3 = self.create_publisher(MarkerArray, "angular_velocity", 1)

    def step(self, action):
        global env_data
        target = False

        # Publish the robot action

        vel_cmd = Twist()
        vel_cmd.linear.x ,vel_cmd.linear.y, vel_cmd.linear.z = action[0]
        vel_cmd.angular.x, vel_cmd.angular.y, vel_cmd.angular.z = action[1]
        self.cmd_vel_pub.publish(vel_cmd)
        self.publish_markers(action)

        while not self.unpause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        try:
            self.unpause.call_async(Empty.Request())
        except:
            print("/unpause_physics service call failed")

        # propagate state for TIME_DELTA seconds
        time.sleep(TIME_DELTA)

        while not self.pause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        try:
            pass
            self.pause.call_async(Empty.Request())
        except (rclpy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")
        
        done, collision, min_laser = self.observe_collision(env_data)
        v_state = []
        v_state[:] = env_data[:]
        laser_state = [v_state]

        # Calculate robot heading from odometry data
        self.odom_x = last_odom.pose.pose.position.x
        self.odom_y = last_odom.pose.pose.position.y
        quaternion = Quaternion(
            last_odom.pose.pose.orientation.w,
            last_odom.pose.pose.orientation.x,
            last_odom.pose.pose.orientation.y,
            last_odom.pose.pose.orientation.z,
        )
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)

def main():
    print('Hi from drone_mission.')

if __name__ == '__main__':
    main()
