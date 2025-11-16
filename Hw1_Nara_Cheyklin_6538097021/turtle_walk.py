import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtleWalk(Node):
    def __init__(self):
        super().__init__('turtle_walk')

        # Publisher to turtlesim velocity topic
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Choose shape: "circle" or "triangle"
        self.declare_parameter("shape", "circle")
        self.shape = self.get_parameter("shape").get_parameter_value().string_value

        # Timer for publishing
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Triangle parameters
        self.side_length = 2.0      # meters per side
        self.forward_speed = 1.0    # m/s
        self.turn_speed = 1.5       # rad/s
        self.turn_angle = 2 * math.pi / 3  # 120 degrees in radians

        # State variables for triangle
        self.state = "forward"
        self.distance_traveled = 0.0
        self.angle_turned = 0.0
        self.side_count = 0

    def timer_callback(self):
        msg = Twist()

        if self.shape == "circle":
            # Circle motion: forward + angular velocity
            msg.linear.x = 1.0
            msg.angular.z = 1.0

        elif self.shape == "triangle":
            if self.state == "forward":
                msg.linear.x = self.forward_speed
                msg.angular.z = 0.0
                self.distance_traveled += self.forward_speed * self.timer_period

                if self.distance_traveled >= self.side_length:
                    self.state = "turn"
                    self.distance_traveled = 0.0
                    self.angle_turned = 0.0

            elif self.state == "turn":
                msg.linear.x = 0.0
                msg.angular.z = self.turn_speed
                self.angle_turned += self.turn_speed * self.timer_period

                if self.angle_turned >= self.turn_angle:
                    self.side_count += 1
                    if self.side_count >= 3:
                        self.get_logger().info("Triangle complete!")
                        msg.linear.x = 0.0
                        msg.angular.z = 0.0
                        self.publisher_.publish(msg)
                        rclpy.shutdown()
                        return
                    self.state = "forward"
                    self.distance_traveled = 0.0

        # Publish velocity command
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleWalk()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

