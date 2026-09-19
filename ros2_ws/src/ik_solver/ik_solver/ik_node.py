import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import math


class IKSolverNode(Node):

    def __init__(self):
        super().__init__('ik_solver')

        self.L1 = 0.075   # thigh: 75 mm
        self.L2 = 0.079   # shin:  79 mm

        self.subscription = self.create_subscription(
            Point,
            '/ball_position',
            self.ball_callback,
            1   
        )

        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.angle_pub = self.create_publisher(
            Float32MultiArray,
            '/joint_angles',
            1   
        )

        self.get_logger().info(
            f'IK solver started. L1={self.L1*1000:.0f}mm  L2={self.L2*1000:.0f}mm'
        )
        self.get_logger().info('Waiting for /ball_position...')

    def forward_kinematics(self, theta1, theta2):

        x = (
            self.L1 * math.cos(theta1)
            + self.L2 * math.cos(theta1 + theta2)
        )

        y = ( 
            self.L1 * math.sin(theta1)
            + self.L2 * math.sin(theta1 + theta2)
        )

        return x, y

    def ball_callback(self, msg):
        x = msg.x   # forward distance in metres
        y = msg.y   # vertical distance (negative = below hip)

        self.get_logger().info(f'Ball at x={x:.3f}m  y={y:.3f}m')

        # Solve IK
        result = self.solve_ik(x, y)
        if result is None:
            return

        theta1_rad, theta2_rad = result
        theta1_deg = math.degrees(theta1_rad)
        theta2_deg = math.degrees(theta2_rad)
        
        fk_x, fk_y = self.forward_kinematics(theta1_rad, theta2_rad)
        fk_error = math.sqrt((fk_x - x)**2 + (fk_y - y)**2)
        if fk_error > 0.001: 
            self.get_logger().warn(
                f'FK validation failed: error={fk_error*1000:.2f}mm '
                f'target=({x:.3f},{y:.3f}) got=({fk_x:.3f},{fk_y:.3f})'
            )


        self.get_logger().info(
            f'IK solved → hip={theta1_deg:.1f}°  knee={theta2_deg:.1f}°'
            f'FK error={fk_error*1000:.2f}mm'
        )

        # Publish to RViz
        self.publish_joint_states(theta1_rad, theta2_rad)

        # Publish 6 servo angles (converted to 0–180 servo range)
        self.publish_servo_angles(theta1_deg, theta2_deg)

    def solve_ik(self, x, y):
        """
        Geometric IK using law of cosines.

        Coordinate convention (robot frame):
          x = forward (direction of kick)
          y = upward (negative = below the hip)

        Returns (theta1_rad, theta2_rad) or None if target unreachable.
          theta1 = hip angle from horizontal
          theta2 = knee angle relative to thigh (always <= 0, i.e. bent)
        """
        L1, L2 = self.L1, self.L2

        D = math.sqrt(x**2 + y**2)

        max_reach = L1 + L2
        min_reach = abs(L1 - L2)

        if D > max_reach:
            self.get_logger().warn(
                f'Target too far: D={D*1000:.1f}mm, max={max_reach*1000:.1f}mm'
            )
            return None

        if D < min_reach:
            self.get_logger().warn(
                f'Target too close: D={D*1000:.1f}mm, min={min_reach*1000:.1f}mm'
            )
            return None

        # knee angle
        cos_t2 = (D**2 - L1**2 - L2**2) / (2.0 * L1 * L2)
        cos_t2 = max(-1.0, min(1.0, cos_t2))   # clamp for floating point safety
        theta2 = -math.acos(cos_t2)             # negative => knee bends backward

        # hip angle
        alpha  = math.atan2(y, x)
        beta   = math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
        theta1 = alpha - beta

        return theta1, theta2
    

    def publish_joint_states(self, theta1_rad, theta2_rad):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = ['hip_joint', 'knee_joint']
        msg.position = [theta1_rad, theta2_rad]
        self.joint_pub.publish(msg)

    def publish_servo_angles(self, hip_deg, knee_deg):
        
        # Clamp to a safe range, ESP32 adds neutral on top
        def clamp(deg):
            return max(-90.0, min(90.0, deg))

        l_hip   = clamp(hip_deg)
        l_knee  = clamp(knee_deg)
        l_ankle = 0.0  
        r_hip   = 0.0   
        r_knee  = 0.0
        r_ankle = 0.0

        msg = Float32MultiArray()
        msg.data = [l_hip, l_knee, l_ankle, r_hip, r_knee, r_ankle]
        self.angle_pub.publish(msg)

        self.get_logger().debug(
            f'Raw IK → L:[{l_hip:.1f}, {l_knee:.1f}, {l_ankle:.1f}]  '
            f'R:[{r_hip:.1f}, {r_knee:.1f}, {r_ankle:.1f}]'
        )

def main(args=None):
    rclpy.init(args=args)
    node = IKSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
