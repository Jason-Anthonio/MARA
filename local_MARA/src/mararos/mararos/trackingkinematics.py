import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np
from math import cos, sin

# --- Helper Functions (Standardized Math) ---
def normalize_angle(angle_deg):
    return (angle_deg + 360) % 360

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
 
        # Arm Link Lengths (cm)
        self.a1 = 14.0  
        self.a2 = 14.0  
        self.a3 = 22.5  

        # State Variable for the YOLO Depth Camera
        self.vision_depth_m = 0.0  
        self.camera_x = 0.0
        self.camera_y = 0.0

        # Subscriptions
        self.get_logger().info("IK Node initialized. Listening for vision coordinates...")
        self.vision_sub = self.create_subscription(
            Point, 
            '/vision_coords', 
            self.vision_callback, 
            10
        )

        self.angle_sub = self.create_subscription(
            Point, 
            '/angle_states', #make the pubslisher first for remembering angle positions
            self.angle_callback, 
            10
        )
        
        # (Optional) You can set up a publisher here to send the final t1, t2, t3 
        # to your Raspberry Pi Serial transmitter node later.
        # self.angle_pub = self.create_publisher(Float32MultiArray, '/arm_angles', 10)

    def angle_callback(self, msg):
        """Triggered whenever the arm publishes its current joint angles."""
        b, a, d, c = msg.b, msg.a, msg.d, msg.c #FIGURE THIS OUT FOR VISION CALLBACK PX PY

    def vision_callback(self, msg):
        """Triggered whenever yolodepth_server.py publishes a new target."""
        # -1.0 is the timeout/failure flag you set in the YOLO server
        if msg.z == -1.0:
            self.get_logger().warn("YOLO tracking lost or timed out. Holding position.")
            return

        # 1. Store the incoming vision data into its own independent variables
        self.camera_x = msg.x #this is used for centering
        self.camera_y = msg.y #this is used for centering
        self.vision_depth_m = msg.z  # Extracted from the Point.z field

        self.get_logger().info(f"Target Acquired -> Cam X: {self.camera_x}, Cam Y: {self.camera_y}, Depth: {self.vision_depth_m:.3f}m")

        # 2. Coordinate Transformation (Placeholder)
        # Here is where you will eventually translate vision_depth_m into the arm's px and py.
        # For now, we will use your test coordinates to keep the IK logic running.

        px = cos() * self.vision_depth_m * 100  # Convert meters to centimeters
        py = sin() * self.vision_depth_m * 100  # Convert meters to

        px = px + 2.0 
        py = py + 2.0
        desired_phi = 270

        # 3. Calculate Kinematics
        try:
            t1, t2, t3, actual_phi = self.calculate_inverse_kinematics(self.a1, self.a2, self.a3, px, py, desired_phi)
            
            # Map to physical hardware angles
            phys_t1 = t1
            phys_t2 = 90.0 - t2
            phys_t3 = 270.0 - t3
            
            self.get_logger().info(f"Calculated Joint Angles -> J1: {phys_t1:.2f}°, J2: {phys_t2:.2f}°, J3: {phys_t3:.2f}°")
            
            # TODO: Publish these physical angles to your serial node 
            
        except ValueError as e:
            self.get_logger().error(f"IK FATAL ERROR: {e}")

    # --- Core Kinematics Methods ---
    def get_ik_angles(self, a1, a2, a3, px, py, phi_deg):
        phi = np.radians(phi_deg)
        wx = px - a3 * np.cos(phi)
        wy = py - a3 * np.sin(phi)
        
        dist_sq = wx**2 + wy**2
        if dist_sq > (a1 + a2)**2 + 1e-5 or dist_sq < (a1 - a2)**2 - 1e-5:
            return None 

        cos_theta2 = np.clip((wx**2 + wy**2 - a1**2 - a2**2) / (2 * a1 * a2), -1.0, 1.0)
        theta2 = np.arccos(cos_theta2)
        theta1 = np.arctan2(wy, wx) - np.arctan2(a2 * np.sin(theta2), a1 + a2 * np.cos(theta2))
        theta3 = phi - theta1 - theta2

        return normalize_angle(np.degrees(theta1)), normalize_angle(np.degrees(theta2)), normalize_angle(np.degrees(theta3))

    def check_joint_limits(self, t1, t2, t3):
        if not (40 <= t1 <= 140): return False
        if not (30 <= t2 <= 150): return False
        if not (70 <= t1 <= 110): return False  # Note: You used t3 in your original script, but checked t1 here. Make sure limits match hardware!
        return True

    def calculate_inverse_kinematics(self, a1, a2, a3, px, py, desired_phi_deg):
        # Exact angle check
        angles = self.get_ik_angles(a1, a2, a3, px, py, desired_phi_deg)
        if angles is not None and self.check_joint_limits(*angles):
            return angles[0], angles[1], angles[2], desired_phi_deg

        self.get_logger().warn("Target violates limits/geometry. Sweeping for valid configuration...")
        
        # Sweep outward
        for offset in range(1, 181):
            for sign in [1, -1]:
                phi_test = (desired_phi_deg + (offset * sign)) % 360
                angles_test = self.get_ik_angles(a1, a2, a3, px, py, phi_test)
                if angles_test is not None and self.check_joint_limits(*angles_test):
                    return angles_test[0], angles_test[1], angles_test[2], phi_test

        raise ValueError(f"Target ({px}, {py}) cannot be reached safely.")

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    
    # Notice: Matplotlib plotting is removed. 
    # ROS2 callbacks run in real-time loops, and plt.show() blocks the thread, 
    # which will freeze your robot's subscription to the camera data.
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()