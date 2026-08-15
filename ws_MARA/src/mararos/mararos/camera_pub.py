import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPub(Node):
    def __init__(self):
        super().__init__('camera_pub')
        
        # 1. Declare parameters (defaults to top camera)
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('camera_name', 'camera_top')
        
        # 2. Retrieve parameters
        video_device = self.get_parameter('video_device').get_parameter_value().string_value
        camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        
        topic_name = f'/{camera_name}/image_raw'
        
        self.publisher_ = self.create_publisher(Image, topic_name, 10)
        self.timer = self.create_timer(0.033, self.timer_callback) # ~30 FPS
        
        # Handle integer indices (e.g., "0") vs string paths (e.g., "/dev/v4l/...")
        if video_device.isdigit():
            self.cap = cv2.VideoCapture(int(video_device))
        else:
            self.cap = cv2.VideoCapture(video_device)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera on {video_device}!")
            return
        
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) 
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.bridge = CvBridge()
        self.get_logger().info(f"Camera Active: Broadcasting {video_device} to {topic_name}")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(img_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()