import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtleLetter(Node):
    def __init__(self, letter):
        super().__init__('turtle_letter')
        self.letter = letter.upper()

        # ROS interfaces
        self.pose = None
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Waypoints for letters
        self.targets = self.get_waypoints(self.letter)
        self.current_target_index = 0
        self.state = "align"  # "align" or "move"

        # Control gains
        self.k_linear = 1.5
        self.k_angular = 4.0
        self.dist_tol = 0.05
        self.ang_tol = 0.05

    def get_waypoints(self, letter):
        """Define waypoints for each letter (N and C)."""
        if letter == "N":
            return [
                (5.5, 8.0),  # Up
                (7.0, 5.5),  # Diagonal Right-Down
                (7.0, 8.0),  # Up
            ]
        elif letter == "C":
            return [
                (3.0, 5.5),  # Left
                (3.0, 2.5),  # Down
                (5.5, 2.5)   # Right 
	    ]
        else:
            self.get_logger().error("Unsupported letter, defaulting to C")
            return [(3.0, 5.5), (3.0, 2.5), (5.5, 2.5)]

    def pose_callback(self, msg):
        self.pose = msg

    def control_loop(self):
        if self.pose is None or self.current_target_index >= len(self.targets):
            return

        target_x, target_y = self.targets[self.current_target_index]
        dx = target_x - self.pose.x
        dy = target_y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)

        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.pose.theta
        # Normalize
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        twist = Twist()

        if self.state == "align":
            if abs(angle_error) > self.ang_tol:
                twist.angular.z = self.k_angular * angle_error
            else:
                # alignment done → switch to moving
                self.state = "move"

        elif self.state == "move":
            if distance > self.dist_tol:
                twist.linear.x = self.k_linear * distance
                twist.angular.z = self.k_angular * angle_error
            else:
                # reached target → stop and go to next
                self.current_target_index += 1
                self.state = "align"
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: ros2 run <pkg> draw_letters_pose <N|C>")
        return

    letter = sys.argv[1]
    node = TurtleLetter(letter)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
