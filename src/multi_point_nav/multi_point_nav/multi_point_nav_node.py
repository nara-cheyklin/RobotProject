#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return (0.0, 0.0, qz, qw)


class MultiPointNav(Node):
    def __init__(self):
        super().__init__('multi_point_nav')

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._sub_points = self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_point_callback, 10
        )

        self._sub_cmd = self.create_subscription(
            String, '/gesture_command', self.command_callback, 10
        )

        self.points = []
        self.goal_poses = []
        self.current_goal_idx = 0
        self.goal_handle = None

        self.goals_prepared = False
        self.started = False
        self.paused = False
        self.stopped = False

        # ✅ EDIT yaw for each of the 3 waypoints here (degrees)
        yaw_degrees = [0.0, 90.0, 180.0]
        self.yaws = [math.radians(y) for y in yaw_degrees]

        self.get_logger().info(
            "MultiPointNav ready.\n"
            "- Click 3 points in RViz using 'Publish Point'\n"
            "- Commands: start(1 finger or i), pause(2), continue(4), stop(5 or b)"
        )

    # ---------- RViz clicks ----------
    def clicked_point_callback(self, msg: PointStamped):
        if self.goals_prepared:
            self.get_logger().warn("Already have 3 points; ignoring extra clicks.")
            return

        self.points.append(msg)
        self.get_logger().info(
            f"Collected point {len(self.points)}: ({msg.point.x:.2f}, {msg.point.y:.2f}) frame={msg.header.frame_id}"
        )

        if len(self.points) == 3:
            self.prepare_goals()

    def prepare_goals(self):
        self.get_logger().info("Waiting for Nav2 action server 'navigate_to_pose'...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("navigate_to_pose action server not available.")
            return

        self.goal_poses = []
        for i, pt in enumerate(self.points):
            pose = PoseStamped()
            pose.header.frame_id = pt.header.frame_id  # usually 'map'
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = pt.point.x
            pose.pose.position.y = pt.point.y
            pose.pose.position.z = 0.0

            yaw = self.yaws[i] if i < len(self.yaws) else self.yaws[-1]
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            self.goal_poses.append(pose)

            self.get_logger().info(
                f"Prepared goal {i+1}: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}), yaw={yaw:.2f} rad"
            )

        self.goals_prepared = True
        self.started = False
        self.paused = False
        self.stopped = False
        self.current_goal_idx = 0
        self.goal_handle = None

        self.get_logger().info(
            "✅ Goals prepared.\n"
            "Show 1 finger or press 'i' in gesture window to START.\n"
            "2 fingers PAUSE, 4 fingers CONTINUE, 5 fingers or 'b' STOP."
        )

    # ---------- Commands ----------
    def command_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f"Command: {cmd}")

        if cmd == "start":
            if not self.goals_prepared:
                self.get_logger().warn("Start ignored: no goals yet (click 3 points first).")
                return
            self.stopped = False
            self.paused = False
            self.started = True
            if self.goal_handle is None:
                self.send_current_goal()

        elif cmd == "pause":
            if self.goal_handle is None:
                self.get_logger().info("Pause ignored: no active goal.")
                return
            if self.paused:
                self.get_logger().info("Already paused.")
                return
            self.paused = True
            self.started = False
            self.get_logger().info("Pausing: canceling current goal.")
            self.goal_handle.cancel_goal_async().add_done_callback(lambda f: self._after_cancel("pause", f))

        elif cmd == "continue":
            if not self.goals_prepared:
                self.get_logger().warn("Continue ignored: no goals yet.")
                return
            if not self.paused:
                self.get_logger().info("Continue ignored: not paused.")
                return
            self.paused = False
            self.stopped = False
            self.started = True
            if self.goal_handle is None:
                self.get_logger().info("Continuing: resending current goal.")
                self.send_current_goal()

        elif cmd == "stop":
            self.stopped = True
            self.started = False
            self.paused = False
            if self.goal_handle is not None:
                self.get_logger().info("Stopping: canceling current goal.")
                self.goal_handle.cancel_goal_async().add_done_callback(lambda f: self._after_cancel("stop", f))
            else:
                self.get_logger().info("Stopped (no active goal).")

    # ---------- Nav2 goal sending ----------
    def send_current_goal(self):
        if not self.goals_prepared:
            return
        if self.stopped or self.paused or not self.started:
            return
        if self.current_goal_idx >= len(self.goal_poses):
            self.get_logger().info("🎉 All goals completed.")
            return

        pose = self.goal_poses[self.current_goal_idx]
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(
            f"Sending goal {self.current_goal_idx+1}/{len(self.goal_poses)}: "
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
        )

        self._client.send_goal_async(goal).add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn(f"Goal {self.current_goal_idx+1} rejected. Skipping.")
            self.goal_handle = None
            self.current_goal_idx += 1
            self.send_current_goal()
            return

        self.goal_handle = gh
        self.get_logger().info(f"Goal {self.current_goal_idx+1} accepted.")
        gh.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        _ = future.result().result
        self.get_logger().info(f"Goal {self.current_goal_idx+1} finished.")
        self.goal_handle = None

        if self.stopped or self.paused or not self.started:
            self.get_logger().info("Not running; will not send next goal.")
            return

        self.current_goal_idx += 1
        self.send_current_goal()

    def _after_cancel(self, reason: str, future):
        try:
            _ = future.result()
        except Exception as e:
            self.get_logger().error(f"Cancel error ({reason}): {e}")
        self.goal_handle = None
        if reason == "pause":
            self.get_logger().info("Paused. Show 4 fingers to CONTINUE.")
        elif reason == "stop":
            self.get_logger().info("Stopped.")


def main():
    rclpy.init()
    node = MultiPointNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
