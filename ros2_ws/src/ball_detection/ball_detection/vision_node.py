import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


# ─────────────────────────────────────────────────────────────
# TUNING PARAMETERS
# ─────────────────────────────────────────────────────────────

# HSV range for a standard football (white/black or orange)
# Switch to ORANGE_* if using an orange training ball
HSV_LOWER = np.array([0,   0,   200])   # white ball
HSV_UPPER = np.array([180, 40,  255])

# Orange ball (comment out above and use these instead):
# HSV_LOWER = np.array([5,  150, 150])
# HSV_UPPER = np.array([25, 255, 255])

MIN_RADIUS      = 15     # px  — ignore tiny detections
MAX_RADIUS      = 200    # px  — ignore if too large (noise)
MIN_AREA        = 500    # px² — minimum contour area

# Camera intrinsics — measure your actual setup
CAMERA_HEIGHT_M = 0.30   # metres above ground
HFOV_DEG        = 62.0   # horizontal field of view (Iriun default ~62°)
BALL_DIAMETER_M = 0.22   # standard football diameter

# Latency logging
LATENCY_LOG_INTERVAL = 30  # log latency every N frames


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        self.bridge = CvBridge()

        # ── Choose source: 'camera' or 'topic' ─────────────────
        # 'camera'  → opens /dev/video0 directly (Iriun via usbipd)
        # 'topic'   → subscribes to /image_raw (if another node publishes)
        self.source = 'camera'
        self.cap    = None

        if self.source == 'camera':
            self._open_camera()
        else:
            self.create_subscription(Image, '/image_raw',
                                     self.image_callback, 1)

        # ── Publishers ─────────────────────────────────────────
        # x = lateral offset in metres (+ = right of centre)
        # y = vertical offset in metres (+ = above hip, usually negative)
        # z = forward distance estimate from apparent ball size
        self.ball_pub = self.create_publisher(Point, '/ball_position', 1)

        # Debug image with detection overlay → view in RViz ImageDisplay
        self.debug_pub = self.create_publisher(Image, '/vision/debug', 1)

        # True when ball is visible
        self.detected_pub = self.create_publisher(Bool, '/ball_detected', 1)

        # ── Timer for camera capture loop (30 Hz) ──────────────
        if self.source == 'camera':
            self.create_timer(1.0 / 30.0, self.capture_and_process)

        # Latency tracking
        self.frame_count    = 0
        self.latency_total  = 0.0

        self.get_logger().info('Vision node started')

    # ──────────────────────────────────────────────────────────
    def _open_camera(self):
        for index in range(4):
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS,           30)
                self.cap = cap
                self.get_logger().info(f'Camera opened at index {index}')
                return
        self.get_logger().error(
            'No camera found. Is Iriun connected and usbipd attached?')

    # ──────────────────────────────────────────────────────────
    def capture_and_process(self):
        if self.cap is None or not self.cap.isOpened():
            return
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame', throttle_duration_sec=2)
            return
        self.process_frame(frame)

    # ──────────────────────────────────────────────────────────
    def image_callback(self, msg: Image):
        """Used when source='topic'."""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_frame(frame)

    # ──────────────────────────────────────────────────────────
    def process_frame(self, frame):
        t_start = time.time()
        self.frame_count += 1

        h, w = frame.shape[:2]
        cx   = w // 2   # image centre x
        cy   = h // 2   # image centre y

        # ── HSV detection ──────────────────────────────────────
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask    = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

        # Clean up mask
        mask = cv2.erode( mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # ── Find contours ──────────────────────────────────────
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        debug    = frame.copy()

        if contours:
            # Pick the largest contour
            c    = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > MIN_AREA:
                ((bx, by), radius) = cv2.minEnclosingCircle(c)
                bx, by, radius = int(bx), int(by), int(radius)

                if MIN_RADIUS < radius < MAX_RADIUS:
                    detected = True

                    # ── Pixel → metres ─────────────────────────
                    # Lateral offset using HFOV
                    fov_rad   = np.deg2rad(HFOV_DEG)
                    px_per_m  = w / (2.0 * CAMERA_HEIGHT_M * np.tan(fov_rad / 2.0))

                    x_px_off  = bx - cx   # pixels from centre
                    x_metres  = x_px_off / px_per_m

                    # Vertical: camera is at fixed height, ball on ground
                    y_metres  = -CAMERA_HEIGHT_M   # foot target is always on ground

                    # Distance estimate from apparent radius
                    # z = (real_diameter / 2) * focal_length / radius_px
                    focal_px  = (w / 2.0) / np.tan(fov_rad / 2.0)
                    z_metres  = (BALL_DIAMETER_M / 2.0) * focal_px / radius

                    # Publish ball position
                    pt      = Point()
                    pt.x    = float(x_metres)
                    pt.y    = float(y_metres)
                    pt.z    = float(z_metres)
                    self.ball_pub.publish(pt)

                    # ── Latency logging ────────────────────────
                    latency = (time.time() - t_start) * 1000   # ms
                    self.latency_total += latency
                    if self.frame_count % LATENCY_LOG_INTERVAL == 0:
                        avg = self.latency_total / self.frame_count
                        self.get_logger().info(
                            f'Ball: x={x_metres:+.3f}m  z={z_metres:.3f}m  '
                            f'r={radius}px  latency={latency:.1f}ms  avg={avg:.1f}ms'
                        )

                    # ── Debug overlay ──────────────────────────
                    cv2.circle(debug, (bx, by), radius, (0, 255, 0), 2)
                    cv2.circle(debug, (bx, by), 4,      (0, 0, 255), -1)
                    cv2.putText(debug,
                        f'x={x_metres:+.2f}m z={z_metres:.2f}m',
                        (bx - 60, by - radius - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Centre crosshair
        cv2.line(debug, (cx - 20, cy), (cx + 20, cy), (255, 255, 0), 1)
        cv2.line(debug, (cx, cy - 20), (cx, cy + 20), (255, 255, 0), 1)
        cv2.putText(debug, f'State: {"BALL" if detected else "SEARCHING"}',
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0) if detected else (0, 0, 255), 1)

        # Publish detected flag
        flag      = Bool()
        flag.data = detected
        self.detected_pub.publish(flag)

        # Publish debug image
        try:
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
        except Exception as e:
            self.get_logger().warn(f'Debug publish failed: {e}',
                                   throttle_duration_sec=5)

    # ──────────────────────────────────────────────────────────
    def destroy_node(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


# ──────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
