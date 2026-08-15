import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import time
import math
import numpy as np
from ultralytics import YOLO

class StereoYoloServer(Node):
    def __init__(self):
        super().__init__('yolodepth_server')
        self.bridge = CvBridge()

        # Load the YOLO segmentation model
        # Make sure 'yolov8n-seg.pt' is in your mararos/models directory or adjust path
        package_share_directory = get_package_share_directory('mararos')
        model_path = os.path.join(package_share_directory, 'models', 'yolov8n-seg.pt')
        self.model = YOLO(model_path)
        
        # Dual camera subscriptions
        self.sub_top = self.create_subscription(Image, '/camera_top/image_raw', self.top_callback, 10)
        self.sub_bot = self.create_subscription(Image, '/camera_bottom/image_raw', self.bot_callback, 10)
        self.sub_cmd = self.create_subscription(String, '/system_command', self.command_callback, 10)
        self.pub_coords = self.create_publisher(Point, '/vision_coords', 10)
        
        # Soft-sync variables
        self.latest_top = None
        self.latest_bot = None

        # State and target variables
        self.target_object = None
        self.active = False
        
        # Locking mechanisms
        self.LOCK_TIME = 2.0  
        self.STABILITY_RADIUS = 20  
        self.SEARCH_TIMEOUT = 10.0
        
        self.lock_start_time = None
        self.last_stable_coords = None
        self.search_start_time = None
        
        # Stereo / Depth Variables
        self.OFFSET_X = -48  
        self.OFFSET_Y = -6  
        self.B = math.sqrt((2.60**2) + (9.30**2)) 
        self.alpha = 56.6  
        # Note: 1280 used here to maintain exact math from original stereoyolo script
        self.f_pixel = (1280 * 0.5) / math.tan(self.alpha * 0.5 * math.pi / 180)
        
        self.smoothed_depth_m = 0.0
        self.smoothing_factor = 0.2

        self.get_logger().info("Stereo YOLO Server Standby...")

    def command_callback(self, msg):
        if msg.data.startswith("FIND:"):
            self.target_object = msg.data.split(":")[1].strip().lower()
            self.active = True
            self.search_start_time = time.time()
            self.lock_start_time = None
            self.last_stable_coords = None
            self.smoothed_depth_m = 0.0
            self.get_logger().info(f"Stereo YOLO Activated. Searching for: '{self.target_object}'")

    def top_callback(self, msg):
        self.latest_top = msg
        self.process_frames()

    def bot_callback(self, msg):
        self.latest_bot = msg
        self.process_frames()

    def crop_to_square(self, frame):
        height, width = frame.shape[:2]
        min_dim = min(height, width)
        start_x = (width // 2) - (min_dim // 2)
        start_y = (height // 2) - (min_dim // 2)
        return frame[start_y : start_y + min_dim, start_x : start_x + min_dim]

    def apply_clahe(self, frame):
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        cl = clahe.apply(l)
        limg = cv2.merge((cl,a,b))
        return cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)

    def get_yolo_center(self, frame):
        # Run segmentation model
        results = self.model(frame, verbose=False, device="cuda:0")
        
        best_contour = None
        max_area = 0
        center = None

        for r in results:
            if r.masks is not None and r.boxes is not None:
                for box, contour_points in zip(r.boxes, r.masks.xy):
                    class_id = int(box.cls[0])
                    class_name = self.model.names[class_id].lower()

                    if self.target_object and (self.target_object in class_name or class_name in self.target_object):
                        contour = np.array(contour_points, dtype=np.int32)
                        area = cv2.contourArea(contour)
                        if area > max_area:
                            max_area = area
                            best_contour = contour
                            
        if best_contour is not None:
            M = cv2.moments(best_contour)
            if M["m00"] != 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            else:
                center = (0, 0)
                
            cv2.drawContours(frame, [best_contour], -1, (0, 255, 0), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            cv2.putText(frame, self.target_object, (center[0] - 20, center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return center, frame

    def find_depth(self, center_top, center_bottom):
        x_top, y_top = center_top
        x_bottom, y_bottom = center_bottom

        d_x = (x_top - x_bottom) - self.OFFSET_X
        d_y = (y_top - y_bottom) - self.OFFSET_Y
        
        disparity = math.sqrt(d_x**2 + d_y**2)

        if disparity != 0:
            zDepth = (self.B * self.f_pixel) / disparity
        else:
            zDepth = 0

        return abs(zDepth)

    def process_frames(self):
        # Only process if active and both frames are received
        if not self.active or self.latest_top is None or self.latest_bot is None:
            return

        # Pop frames to consume them
        msg_top = self.latest_top
        msg_bot = self.latest_bot
        self.latest_top = None
        self.latest_bot = None
        
        # Convert to cv2
        frame_top = self.bridge.imgmsg_to_cv2(msg_top, desired_encoding='bgr8')
        frame_bot = self.bridge.imgmsg_to_cv2(msg_bot, desired_encoding='bgr8')

        # Preprocess
        frame_bot = cv2.flip(frame_bot, 1)
        frame_top = cv2.flip(frame_top, 0)
        frame_top = self.crop_to_square(frame_top)
        frame_bot = self.crop_to_square(frame_bot)
        frame_top = self.apply_clahe(frame_top)
        frame_bot = self.apply_clahe(frame_bot)

        # Get centers
        center_top, display_top = self.get_yolo_center(frame_top)
        center_bot, display_bot = self.get_yolo_center(frame_bot)

        # Timeout Logic
        if center_top is None or center_bot is None:
            elapsed = time.time() - self.search_start_time
            cv2.putText(display_top, f"SEARCHING... TIMEOUT IN {self.SEARCH_TIMEOUT - elapsed:.1f}s", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(display_bot, "TRACKING LOST", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            if elapsed > self.SEARCH_TIMEOUT:
                self.get_logger().info("Timeout. Object not found.")
                # Publish an invalid point to let the Brain know we failed
                self.pub_coords.publish(Point(x=-1.0, y=-1.0, z=-1.0)) 
                self.active = False
                cv2.destroyAllWindows()
                return
        else:
            # We found the target in both cameras, reset search timer
            self.search_start_time = time.time()

            # 1. Calculate Stereo Depth
            depth_cm = self.find_depth(center_top, center_bot)
            raw_depth_m = depth_cm / 100.0

            if self.smoothed_depth_m == 0.0:
                self.smoothed_depth_m = raw_depth_m
            else:
                self.smoothed_depth_m = (self.smoothing_factor * raw_depth_m) + ((1 - self.smoothing_factor) * self.smoothed_depth_m)

            # 2. Locking / Stability Logic (using top camera coordinates as truth)
            cx, cy = center_top
            if self.last_stable_coords is None:
                self.last_stable_coords = (cx, cy)
                self.lock_start_time = time.time()
            else:
                drift = math.hypot(cx - self.last_stable_coords[0], cy - self.last_stable_coords[1])

                if drift <= self.STABILITY_RADIUS:
                    elapsed_time = time.time() - self.lock_start_time
                    bar_width = int(400 * (elapsed_time / self.LOCK_TIME))
                    
                    # Draw locking UI on Top Camera
                    cv2.rectangle(display_top, (50, 400), (50 + bar_width, 430), (0, 255, 0), -1)

                    if elapsed_time >= self.LOCK_TIME:
                        self.get_logger().info(f"Locked at (x:{cx}, y:{cy}) | Depth: {self.smoothed_depth_m:.3f}m")
                        
                        # PUBLISH Z AS DEPTH
                        self.pub_coords.publish(Point(x=float(cx), y=float(cy), z=float(self.smoothed_depth_m)))
                        self.active = False
                        cv2.destroyAllWindows()
                        return
                else:
                    self.last_stable_coords = (cx, cy)
                    self.lock_start_time = time.time()

            # Visual Overlays
            cv2.putText(display_top, "TRACKING", (75, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124, 252, 0), 2)
            cv2.putText(display_top, f"Dist: {self.smoothed_depth_m:.3f} m", (250, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124, 252, 0), 2)
            cv2.putText(display_bot, "TRACKING", (75, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124, 252, 0), 2)

        # Show Output
        cv2.imshow("Vision Subsystem - Top", display_top)
        cv2.imshow("Vision Subsystem - Bottom", display_bot)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = StereoYoloServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()