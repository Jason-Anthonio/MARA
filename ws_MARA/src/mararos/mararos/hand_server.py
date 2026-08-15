import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import time

class HandServer(Node):
    def __init__(self):
        super().__init__('hand_server')
        self.bridge = CvBridge()
        
        # UPDATED: Now subscribes to the Top Camera specifically
        self.sub_img = self.create_subscription(Image, '/camera_top/image_raw', self.image_callback, 10)
        self.sub_cmd = self.create_subscription(String, '/system_command', self.command_callback, 10)
        self.pub_coords = self.create_publisher(Point, '/hand_coords', 10)
        
        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        
        self.active = False
        self.fist_start_time = None
        self.hover_coords = None
        self.CONFIRM_TIME = 2.0
        self.get_logger().info("Hand Tracking Server Standby (Using Top Camera)...")

    def command_callback(self, msg):
        if msg.data == "TRACK_HAND":
            self.active = True
            self.fist_start_time = None
            self.get_logger().info("Hand Tracking Activated.")

    def image_callback(self, msg):
        if not self.active: return
        
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img = cv2.flip(img, 1)
        h, w, c = img.shape
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)

        status_text = "WAITING FOR POINTING POSE"
        status_color = (0, 0, 255)

        if results.multi_hand_landmarks:
            for hand_lms in results.multi_hand_landmarks:
                index_up = hand_lms.landmark[8].y < hand_lms.landmark[6].y
                index_down = hand_lms.landmark[8].y > hand_lms.landmark[6].y
                middle_down = hand_lms.landmark[12].y > hand_lms.landmark[10].y
                ring_down = hand_lms.landmark[16].y > hand_lms.landmark[14].y
                pinky_down = hand_lms.landmark[20].y > hand_lms.landmark[18].y

                is_pointing = index_up and middle_down and ring_down and pinky_down
                is_fist = index_down and middle_down and ring_down and pinky_down

                if is_pointing:
                    self.fist_start_time = None
                    x_min, y_min, x_max, y_max = w, h, 0, 0
                    depth_z = hand_lms.landmark[0].z

                    for lm in hand_lms.landmark:
                        cx, cy = int(lm.x * w), int(lm.y * h)
                        x_min, x_max = min(x_min, cx), max(x_max, cx)
                        y_min, y_max = min(y_min, cy), max(y_max, cy)

                    center_x = int((x_min + x_max) / 2)
                    center_y = int((y_min + y_max) / 2)
                    self.hover_coords = (center_x, center_y, depth_z)

                    self.mp_draw.draw_landmarks(img, hand_lms, self.mp_hands.HAND_CONNECTIONS)
                    cv2.rectangle(img, (x_min-20, y_min-20), (x_max+20, y_max+20), (0, 255, 0), 2)
                    status_text = "TARGETING - MAKE A FIST TO LOCK"
                    status_color = (0, 255, 255)

                elif is_fist and self.hover_coords:
                    if self.fist_start_time is None:
                        self.fist_start_time = time.time()
                    
                    elapsed_time = time.time() - self.fist_start_time
                    bar_width = int(400 * (elapsed_time / self.CONFIRM_TIME))
                    cv2.rectangle(img, (50, 400), (50 + bar_width, 430), (0, 255, 0), -1)
                    status_text = f"LOCKING IN... {elapsed_time:.1f}s"
                    status_color = (0, 255, 0)

                    if elapsed_time >= self.CONFIRM_TIME:
                        self.get_logger().info(f"Locked at {self.hover_coords}")
                        # Publish result and turn off tracking
                        pt = Point(x=float(self.hover_coords[0]), y=float(self.hover_coords[1]), z=float(self.hover_coords[2]))
                        self.pub_coords.publish(pt)
                        self.active = False
                        cv2.destroyWindow("Pose Sensitive Tracker (Top Camera)")
                        return
                else:
                    self.fist_start_time = None

        cv2.putText(img, f"STATUS: {status_text}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        cv2.imshow("Pose Sensitive Tracker (Top Camera)", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HandServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()