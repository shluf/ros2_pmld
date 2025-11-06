#!/usr/bin/env python3

import cv2
import mediapipe as mp
import numpy as np
import copy
from pathlib import Path
from threading import Lock
from collections import deque

from .keypoint_classifier import KeyPointClassifier
from .point_history_classifier import PointHistoryClassifier

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


# -------------------------------------------------------
# Main gesture recognition logic
# -------------------------------------------------------
class GestureRecognition:
    def __init__(self, model_path='model', debug=True):
        self.debug = debug
        self.model_path = Path(model_path)

        # mediapipe hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )

        # frame management
        self.current_frame = None
        self.frame_lock = Lock()

        # classifiers
        kp_model = self.model_path / 'keypoint_classifier' / 'keypoint_classifier.tflite'
        ph_model = self.model_path / 'point_history_classifier' / 'point_history_classifier.tflite'
        self.keypoint_classifier = KeyPointClassifier(str(kp_model))
        self.point_history_classifier = PointHistoryClassifier(str(ph_model))

        # point history
        self.point_history = deque(maxlen=16)

        # current results
        self.current_hand_sign = None
        self.current_finger_gesture = None
        self.current_hand_position = None

        # fps
        self.fps_counter = 0
        self.fps_start_time = cv2.getTickCount()

    def update_frame(self, cv_image):
        with self.frame_lock:
            self.current_frame = cv_image

    def process(self):
        with self.frame_lock:
            if self.current_frame is None:
                return None, None, None
            image = self.current_frame.copy()

        # image = cv2.flip(image, 1)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        results = self.hands.process(image_rgb)
        image_rgb.flags.writeable = True

        self.current_hand_sign = None
        self.current_finger_gesture = None
        self.current_hand_position = None

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                landmark_list = self._calc_landmark_list(image, hand_landmarks)
                pre_landmark = self._pre_process_landmark(landmark_list)
                self.current_hand_position = self._get_hand_center(landmark_list, image.shape)
                hand_sign_id = self.keypoint_classifier(pre_landmark)
                self.current_hand_sign = hand_sign_id

                index_finger_tip = landmark_list[8]
                self.point_history.append(index_finger_tip)

                if len(self.point_history) == self.point_history.maxlen:
                    pre_point_hist = self._pre_process_point_history(list(self.point_history))
                    finger_gesture_id = self.point_history_classifier(pre_point_hist)
                    self.current_finger_gesture = finger_gesture_id
        else:
            self.point_history.clear()

        if self.debug:
            self._debug_display(image)
        return self.current_hand_sign, self.current_finger_gesture, self.current_hand_position

    # ---------- helpers ----------
    def _calc_landmark_list(self, image, landmarks):
        image_width, image_height = image.shape[1], image.shape[0]
        landmark_point = []
        for landmark in landmarks.landmark:
            landmark_x = min(int(landmark.x * image_width), image_width - 1)
            landmark_y = min(int(landmark.y * image_height), image_height - 1)
            landmark_point.append([landmark_x, landmark_y])
        return landmark_point

    def _pre_process_landmark(self, landmark_list):
        temp = copy.deepcopy(landmark_list)
        base_x, base_y = temp[0]
        temp = [[x - base_x, y - base_y] for x, y in temp]
        temp = np.array(temp).flatten()
        max_value = np.max(np.abs(temp)) or 1.0
        temp = temp / max_value
        return temp.tolist()

    def _pre_process_point_history(self, point_history):
        temp = copy.deepcopy(point_history)
        base_x, base_y = temp[0]
        temp = [[x - base_x, y - base_y] for x, y in temp]
        temp = np.array(temp).flatten()
        max_value = np.max(np.abs(temp)) or 1.0
        temp = temp / max_value
        return temp.tolist()

    def _get_hand_center(self, landmark_list, image_shape):
        x = landmark_list[0][0] / image_shape[1]
        y = landmark_list[0][1] / image_shape[0]
        return (x, y)

    def _debug_display(self, image):
        fps_end = cv2.getTickCount()
        time_diff = (fps_end - self.fps_start_time) / cv2.getTickFrequency()
        fps = 1.0 / time_diff if time_diff > 0 else 0
        self.fps_start_time = fps_end
        cv2.putText(image, f'FPS:{fps:.1f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        if self.current_hand_sign is not None:
            cv2.putText(image, f'Hand:{self.current_hand_sign}', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        if self.current_finger_gesture is not None:
            cv2.putText(image, f'Gesture:{self.current_finger_gesture}', (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        cv2.imshow("GestureRecognition", image)
        cv2.waitKey(1)

    def release(self):
        cv2.destroyAllWindows()


# -------------------------------------------------------
# ROS2 Node wrapper
# -------------------------------------------------------
class GestureRecognitionNode(Node):
    def __init__(self):
        super().__init__('gesture_recognition_node')
        self.publisher = self.create_publisher(Int32MultiArray, '/gesture_result', 10)

        model_dir = str(Path(__file__).resolve().parent / 'model')
        self.recognizer = GestureRecognition(model_path=model_dir, debug=False)

        # use webcam if no ros image topic
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('GestureRecognitionNode started.')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        self.recognizer.update_frame(frame)
        hand_sign, finger_gesture, pos = self.recognizer.process()

        msg = Int32MultiArray()
        msg.data = [
            hand_sign if hand_sign is not None else -1,
            finger_gesture if finger_gesture is not None else -1,
            int(pos[0]*100) if pos else -1,
            int(pos[1]*100) if pos else -1
        ]
        self.publisher.publish(msg)

    def destroy_node(self):
        self.recognizer.release()
        self.cap.release()
        super().destroy_node()


# -------------------------------------------------------
# entry point
# -------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = GestureRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down gesture node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()