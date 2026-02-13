#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import mediapipe as mp


class HandGestureNode(Node):
    def __init__(self):
        super().__init__('hand_gesture_node')

        self.pub = self.create_publisher(String, '/gesture_command', 10)
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils

        self.last_command = ""

        self.get_logger().info("HandGestureNode listening on /image_raw/compressed")

    def image_callback(self, msg: CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        results = self.hands.process(frame_rgb)
        command = ""

        if results.multi_hand_landmarks:
            hand = results.multi_hand_landmarks[0]
            finger_count = self.count_fingers(hand, frame.shape)

            if finger_count == 1:
                command = "start"
            elif finger_count == 2:
                command = "pause"
            elif finger_count == 4:
                command = "continue"
            elif finger_count == 5:
                command = "stop"

            self.mp_draw.draw_landmarks(frame, hand, self.mp_hands.HAND_CONNECTIONS)
            cv2.putText(frame, f"Fingers: {finger_count}", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        # Keyboard overrides in VMware window
        key = cv2.waitKey(1) & 0xFF
        if key == ord('i'):
            command = "start"
        elif key == ord('b'):
            command = "stop"
        elif key == ord('q'):
            rclpy.shutdown()
            return

        if command and command != self.last_command:
            out = String()
            out.data = command
            self.pub.publish(out)
            self.get_logger().info(f"Published: {command}")
            self.last_command = command
        elif not command:
            self.last_command = ""

        cv2.imshow("Gesture (robot camera stream)", frame)

    def count_fingers(self, hand_landmarks, img_shape):
    """
    Robust finger counting:
    - Detects whether the hand is oriented 'upright' or 'upside-down' in the image.
    - Uses that to decide whether tip.y < pip.y means 'up' or the opposite.
    - Thumb uses left/right hand detection if available; otherwise fallback.
    """
        h, w, _ = img_shape
        lm = hand_landmarks.landmark

        def px(i):
            return (lm[i].x * w, lm[i].y * h)

        # Key landmarks
        wrist_y = px(0)[1]
        middle_mcp_y = px(9)[1]   # middle finger MCP

        # If wrist is lower (bigger y) than MCP, fingers likely point upward in image
        # (because y increases downward)
        hand_upright = wrist_y > middle_mcp_y

        # Choose comparator for "finger is up"
        # Upright: tip above pip => tip_y < pip_y
        # Upside-down: tip below pip => tip_y > pip_y
        def finger_up(tip_id, pip_id):
            tip_y = px(tip_id)[1]
            pip_y = px(pip_id)[1]
            return tip_y < pip_y if hand_upright else tip_y > pip_y

        count = 0

        # Index, Middle, Ring, Pinky
        if finger_up(8, 6):   count += 1
        if finger_up(12, 10): count += 1
        if finger_up(16, 14): count += 1
        if finger_up(20, 18): count += 1

    # Thumb: tricky because it's sideways. Use x-axis, but direction depends on hand.
    # We'll infer hand side by comparing index MCP (5) vs pinky MCP (17):
    # If index_mcp_x < pinky_mcp_x => likely right hand in a normal (non-mirrored) view.
        index_mcp_x = px(5)[0]
        pinky_mcp_x = px(17)[0]
        right_hand_like = index_mcp_x < pinky_mcp_x

        thumb_tip_x = px(4)[0]
        thumb_ip_x  = px(3)[0]

    # If hand is mirrored, right_hand_like may flip; but this heuristic works surprisingly well.
        if right_hand_like:
            thumb_up = thumb_tip_x > thumb_ip_x
        else:
            thumb_up = thumb_tip_x < thumb_ip_x

        count += 1 if thumb_up else 0

        return count


def main():
    rclpy.init()
    node = HandGestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
