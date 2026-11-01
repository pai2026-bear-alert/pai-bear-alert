#!/usr/bin/env python3

"""
Bear Detector Node

Reads a video file, runs YOLO (COCO pretrained) on each frame, and publishes
whether a bear is present on the '/bear_alert' topic (std_msgs/Bool).

To avoid reacting to single-frame false positives, the alert turns on only
after `on_frames` consecutive detections. It turns off when no bear has been
seen for `off_seconds` of video time, so the behavior does not depend on how
fast inference runs on the CPU.

Example:
    ros2 run bear_alert bear_detector --ros-args -p video_path:=/home/ubuntu/videos/bear.mp4
"""

import time

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ultralytics import YOLO

BEAR_CLASS_ID = 21  # 'bear' in the COCO dataset
WINDOW_NAME = 'bear_detector'


class BearDetector(Node):
    """ROS2 node that detects bears in a video and publishes an alert flag."""

    def __init__(self):
        super().__init__('bear_detector')

        self.declare_parameter('video_path', '/home/ubuntu/videos/bear.mp4')
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('conf', 0.4)
        self.declare_parameter('imgsz', 416)        # inference size (smaller is faster on CPU)
        self.declare_parameter('rate', 10.0)        # max detection rate [Hz]
        self.declare_parameter('on_frames', 2)      # consecutive hits to turn alert on
        self.declare_parameter('off_seconds', 1.0)  # no bear for this long (video time) turns alert off
        self.declare_parameter('show', True)        # show annotated window
        self.declare_parameter('loop', False)       # replay video when it ends

        self.video_path = self.get_parameter('video_path').value
        self.conf = self.get_parameter('conf').value
        self.imgsz = self.get_parameter('imgsz').value
        self.on_frames = self.get_parameter('on_frames').value
        self.off_seconds = self.get_parameter('off_seconds').value
        self.show = self.get_parameter('show').value
        self.loop = self.get_parameter('loop').value
        rate = self.get_parameter('rate').value

        self.model = YOLO(self.get_parameter('model').value)
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            raise RuntimeError(f'Cannot open video: {self.video_path}')
        self.video_fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0

        self.publisher = self.create_publisher(Bool, 'bear_alert', 10)

        self.alert = False
        self.hit_count = 0
        self.last_seen = None    # video time [s] when a bear was last detected
        self.last_frame = None   # last annotated frame (without status label)
        self.frame_index = 0
        self.start_time = time.monotonic()

        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(f'Bear Detector started: {self.video_path}')

    def read_frame(self):
        """Read the frame that matches the elapsed time, so playback stays at real speed."""
        target = int((time.monotonic() - self.start_time) * self.video_fps)
        while self.frame_index < target:
            if not self.cap.grab():
                return None
            self.frame_index += 1
        ok, frame = self.cap.read()
        if not ok:
            return None
        self.frame_index += 1
        return frame

    def on_timer(self):
        frame = self.read_frame()
        if frame is None:
            if self.loop:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self.frame_index = 0
                self.start_time = time.monotonic()
                return
            self.get_logger().info('Video finished')
            self.alert = False
            self.publish(False)
            self.show_frame()
            self.timer.cancel()
            return

        video_time = self.frame_index / self.video_fps
        t0 = time.monotonic()
        result = self.model(frame, classes=[BEAR_CLASS_ID], conf=self.conf,
                            imgsz=self.imgsz, verbose=False)[0]
        infer_ms = (time.monotonic() - t0) * 1000
        detected = len(result.boxes) > 0

        if detected:
            self.hit_count += 1
            self.last_seen = video_time
        else:
            self.hit_count = 0

        if not self.alert and self.hit_count >= self.on_frames:
            self.alert = True
            self.get_logger().warn(f'BEAR DETECTED -> alert ON (t={video_time:.1f}s)')
        elif self.alert and not detected and video_time - self.last_seen >= self.off_seconds:
            self.alert = False
            self.get_logger().info(f'Bear gone -> alert OFF (t={video_time:.1f}s)')

        self.publish(self.alert)
        self.get_logger().info(
            f't={video_time:.1f}s infer={infer_ms:.0f}ms detected={detected}',
            throttle_duration_sec=1.0)

        self.last_frame = result.plot()
        self.show_frame()

    def show_frame(self):
        """Show the last annotated frame with the current alert status."""
        if not self.show or self.last_frame is None:
            return
        annotated = self.last_frame.copy()
        label = 'BEAR ALERT!' if self.alert else 'monitoring...'
        color = (0, 0, 255) if self.alert else (0, 200, 0)
        cv2.putText(annotated, label, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 3)
        cv2.imshow(WINDOW_NAME, annotated)
        cv2.waitKey(1)
        # Closing the window with the X button stops the node
        if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
            self.get_logger().info('Window closed -> stopping')
            raise SystemExit

    def publish(self, value):
        msg = Bool()
        msg.data = value
        self.publisher.publish(msg)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = BearDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # Tell the warning node that the alert is off before exiting
        try:
            node.publish(False)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
