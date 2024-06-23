import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi

class RobotSquarePathNode(Node):
    def __init__(self):
        super().__init__('robot_square_path_node')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.update_timer = self.create_timer(0.01, self.follow_square_path)  # Timer frequency

        self.lin_vel = 0.5    # Linear velocity in meters per second
        self.ang_vel = 0.5    # Angular velocity in radians per second
        self.state_start = self.get_clock().now().nanoseconds / 1e9  # Start time in seconds
        self.time_elapsed = 0
        self.current_state = 'FORWARD'  # Initial state

    def follow_square_path(self):
        now = self.get_clock().now().nanoseconds / 1e9  # Current time in seconds
        self.time_elapsed = now - self.state_start
        print(f"State: {self.current_state}, Time Elapsed: {self.time_elapsed:.6f}")  # Debug information

        if self.current_state == 'FORWARD':
            if self.time_elapsed < 4:
                self.move_forward()
            else:
                self.halt_robot()
                self.state_start = self.get_clock().now().nanoseconds / 1e9
                self.time_elapsed = 0  # Reset elapsed time
                self.current_state = 'ROTATE'

        elif self.current_state == 'ROTATE':
            # Duration of the turn to achieve 90 degrees (π/2 radians), with a buffer
            turn_time = (pi / 2) / self.ang_vel           
            if self.time_elapsed < turn_time:
                self.rotate()
            else:
                print(f"Turn completed in: {self.time_elapsed:.6f} seconds")  # Debug information
                self.halt_robot()
                self.state_start = self.get_clock().now().nanoseconds / 1e9
                self.time_elapsed = 0  # Reset elapsed time
                self.current_state = 'FORWARD'

    def move_forward(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.lin_vel
        twist_msg.angular.z = 0.0
        self.vel_pub.publish(twist_msg)

    def rotate(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = self.ang_vel
        self.vel_pub.publish(twist_msg)

    def halt_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotSquarePathNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
