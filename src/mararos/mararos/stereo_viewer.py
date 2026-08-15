import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class StereoViewerNode(Node):
    def __init__(self):
        super().__init__('stereo_viewer')
        self.bridge = CvBridge()

        # Subscribe to both camera topics
        self.sub_top = self.create_subscription(Image, '/camera_top/image_raw', self.top_callback, 10)
        self.sub_bot = self.create_subscription(Image, '/camera_bottom/image_raw', self.bot_callback, 10)

        # Placeholders for the latest frames
        self.latest_top = None
        self.latest_bot = None

        self.get_logger().info("Stereo Viewer Active: Waiting for camera feeds...")

    def top_callback(self, msg):
        self.latest_top = msg
        self.render_frames()

    def bot_callback(self, msg):
        self.latest_bot = msg
        self.render_frames()

    def render_frames(self):
        # Only proceed if we have received a frame from both cameras
        if self.latest_top is None or self.latest_bot is None:
            return

        # Pop the frames so we wait for fresh ones next cycle
        msg_top = self.latest_top
        msg_bot = self.latest_bot
        self.latest_top = None
        self.latest_bot = None

        try:
            # Convert ROS2 Image messages to OpenCV format
            frame_top = self.bridge.imgmsg_to_cv2(msg_top, desired_encoding='bgr8')
            frame_bot = self.bridge.imgmsg_to_cv2(msg_bot, desired_encoding='bgr8')

            # The bottom camera was flipped in your stereo script, so we flip it here too for consistency
            frame_bot = cv2.flip(frame_bot, 1)
            frame_top = cv2.flip(frame_top, 0)

            # Add labels to the frames
            cv2.putText(frame_top, "TOP CAMERA", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame_bot, "BOTTOM CAMERA", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Stack the images vertically (Top on top, Bottom on bottom)
            # Make sure they are the same width before stacking. 
            # If resolutions differ, you would need to resize one of them here.
            stacked_frames = np.vstack((frame_top, frame_bot))

            # Resize the stacked window if it's too large for a standard monitor
            display_height = 800
            scale = display_height / stacked_frames.shape[0]
            display_width = int(stacked_frames.shape[1] * scale)
            resized_display = cv2.resize(stacked_frames, (display_width, display_height))

            # Display the combined feed
            cv2.imshow("Stereo Camera Viewer", resized_display)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing frames: {e}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StereoViewerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()