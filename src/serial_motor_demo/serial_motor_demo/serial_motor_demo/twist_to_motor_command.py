import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from serial_motor_demo_msgs.msg import MotorCommand
import math

class TwistToMotorCommand(Node):

    def __init__(self):
        super().__init__('twist_to_motor_command')

        # Parameters
        self.declare_parameter('wheel_base', 0.2)  # Distance between wheels in meters (ADJUST THIS FOR YOUR ROBOT!)
        self.declare_parameter('max_pwm', 255.0)    # Maximum PWM value
        self.declare_parameter('max_speed_mps', 0.5) # Maximum linear speed in m/s (Adjust for desired top speed)
        self.declare_parameter('max_turn_rps', 1.0) # Maximum turning speed in rad/s (Adjust for desired turning speed)

        # Get parameter values
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.max_pwm = self.get_parameter('max_pwm').get_parameter_value().double_value
        self.max_speed_mps = self.get_parameter('max_speed_mps').get_parameter_value().double_value
        self.max_turn_rps = self.get_parameter('max_turn_rps').get_parameter_value().double_value

        # Publisher for MotorCommand
        self.motor_command_pub_ = self.create_publisher(MotorCommand, 'motor_command', 10)

        # Subscriber for Twist commands
        self.twist_sub_ = self.create_subscription(
            Twist,
            'cmd_vel',  # Subscribe to the standard teleop topic
            self.twist_callback,
            10)

        self.get_logger().info('Twist to MotorCommand bridge started.')
        self.get_logger().info(f'Wheel Base: {self.wheel_base} m')
        self.get_logger().info(f'Max PWM: {self.max_pwm}')
        self.get_logger().info(f'Max Linear Speed: {self.max_speed_mps} m/s')
        self.get_logger().info(f'Max Angular Speed: {self.max_turn_rps} rad/s')


    def twist_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # --- Basic Differential Drive Inverse Kinematics ---
        # Calculate target wheel speeds in m/s (simplified model)
        # This assumes wheel radius is handled by the scaling below
        left_target_mps = linear_x - (angular_z * self.wheel_base / 2.0)
        right_target_mps = linear_x + (angular_z * self.wheel_base / 2.0)

        # --- Scaling to PWM ---
        # Scale the m/s speeds to PWM values (-max_pwm to +max_pwm)
        # We need to map the maximum possible speeds to the maximum PWM
        
        # Simple linear scaling based on max configured .speeds
        # More complex scaling might be needed depending on motor response
        
        # Scale linear component
        pwm_linear_scale = self.max_pwm / self.max_speed_mps if self.max_speed_mps != 0 else 0
        
        # Scale angular component (consider its contribution to wheel speed)
        # Max wheel speed difference due to rotation = max_turn_rps * wheel_base / 2
        max_rot_wheel_speed = self.max_turn_rps * self.wheel_base / 2.0 if self.wheel_base != 0 else 0
        pwm_angular_scale = self.max_pwm / max_rot_wheel_speed if max_rot_wheel_speed != 0 else 0
        
        # Calculate PWM based on a mix of linear and angular demands
        # This is a basic approach; tuning might be required
        
        # Calculate speeds assuming max_speed_mps corresponds to max_pwm forward/backward
        left_pwm = linear_x * pwm_linear_scale - angular_z * self.wheel_base / 2.0 * pwm_angular_scale
        right_pwm = linear_x * pwm_linear_scale + angular_z * self.wheel_base / 2.0 * pwm_angular_scale

        # Clamp PWM values to the maximum range
        left_pwm = max(min(left_pwm, self.max_pwm), -self.max_pwm)
        right_pwm = max(min(right_pwm, self.max_pwm), -self.max_pwm)

        # Create and publish MotorCommand message
        motor_command_msg = MotorCommand()
        motor_command_msg.is_pwm = True  # We are sending PWM commands
        motor_command_msg.mot_1_req_rad_sec = float(left_pwm)  # Using the field for PWM value
        motor_command_msg.mot_2_req_rad_sec = float(right_pwm) # Using the field for PWM value

        self.motor_command_pub_.publish(motor_command_msg)


def main(args=None):
    rclpy.init(args=args)
    twist_to_motor_node = TwistToMotorCommand()
    rclpy.spin(twist_to_motor_node)
    twist_to_motor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

