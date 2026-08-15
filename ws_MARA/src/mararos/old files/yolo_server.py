import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import time
import math
from ultralytics import YOLO

class YoloServer(Node):
    def __init__(self):
        super().__init__('yolo_server')
        self.bridge = CvBridge()

        package_share_directory = get_package_share_directory('mararos')
        model_path = os.path.join(package_share_directory, 'models', 'yolo11n.pt')
        self.model = YOLO(model_path)
        
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.sub_cmd = self.create_subscription(String, '/system_command', self.command_callback, 10)
        self.pub_coords = self.create_publisher(Point, '/vision_coords', 10)
        
        self.target_object = None
        self.active = False
        
        self.LOCK_TIME = 2.0  
        self.STABILITY_RADIUS = 20  
        self.SEARCH_TIMEOUT = 10.0
        
        self.lock_start_time = None
        self.last_stable_coords = None
        self.search_start_time = None
        
        self.get_logger().info("YOLO Server Standby...")

    def command_callback(self, msg):
        if msg.data.startswith("FIND:"):
            self.target_object = msg.data.split(":")[1]
            self.active = True
            self.search_start_time = time.time()
            self.lock_start_time = None
            self.last_stable_coords = None
            self.get_logger().info(f"YOLO Activated. Searching for: '{self.target_object}'")

    def image_callback(self, msg):
        if not self.active: return
        
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, conf=0.7, device="cuda:0")
        display = results[0].plot()

        target_found_this_frame = False
        best_coords_this_frame = None
        best_y_bottom = 0  
        best_area = 0
        LEVEL_THRESHOLD = 40  

        for box in results[0].boxes:
            class_id = int(box.cls[0].item())
            class_name = results[0].names[class_id]

            if self.target_object and (self.target_object in class_name or class_name in self.target_object):
                target_found_this_frame = True
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                area = (x2 - x1) * (y2 - y1)
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                
                if best_coords_this_frame is None or (abs(y2 - best_y_bottom) < LEVEL_THRESHOLD and area > best_area) or y2 > best_y_bottom:
                    best_y_bottom, best_area, best_coords_this_frame = y2, area, (cx, cy)

        if not target_found_this_frame and self.last_stable_coords is None:
            elapsed = time.time() - self.search_start_time
            cv2.putText(display, f"SEARCHING... TIMEOUT IN {self.SEARCH_TIMEOUT - elapsed:.1f}s", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            if elapsed > self.SEARCH_TIMEOUT:
                self.get_logger().info("Timeout. Object not found.")
                # Publish an invalid point to let the Brain know we failed
                self.pub_coords.publish(Point(x=-1.0, y=-1.0, z=-1.0)) 
                self.active = False
                cv2.destroyAllWindows()
                return
        else:
            self.search_start_time = time.time()

        if target_found_this_frame and best_coords_this_frame:
            cx, cy = best_coords_this_frame

            if self.last_stable_coords is None:
                self.last_stable_coords = (cx, cy)
                self.lock_start_time = time.time()
            else:
                drift = math.hypot(cx - self.last_stable_coords[0], cy - self.last_stable_coords[1])

                if drift <= self.STABILITY_RADIUS:
                    elapsed_time = time.time() - self.lock_start_time
                    bar_width = int(400 * (elapsed_time / self.LOCK_TIME))
                    cv2.rectangle(display, (50, 400), (50 + bar_width, 430), (0, 255, 0), -1)

                    if elapsed_time >= self.LOCK_TIME:
                        self.get_logger().info(f"Target locked at {best_coords_this_frame}")
                        self.pub_coords.publish(Point(x=float(cx), y=float(cy), z=0.0))
                        self.active = False
                        cv2.destroyAllWindows()
                        return
                else:
                    self.last_stable_coords = (cx, cy)
                    self.lock_start_time = time.time()

            cv2.circle(display, (cx, cy), 8, (0, 0, 255), -1)

        cv2.imshow("Vision Subsystem", display)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()