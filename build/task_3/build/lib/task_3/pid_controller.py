import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

class PID_control(Node):
    def __init__(self):
        super().__init__('pid_speed_controller')

        self.overshoot = 0.15    # 15%
        self.settling_time = 30  # seconds
        self.target_dist = 0.35 # meters


        self.damping_ratio = (-1 * (math.log(self.overshoot))) / math.sqrt(math.pi**2 + (math.log(self.overshoot))**2)
        self.natural_frequency = 4 / (self.damping_ratio * self.settling_time)

        self.kp = self.natural_frequency**2 * self.target_dist
        self.ki = 0.1
        self.kd = 2 * self.damping_ratio * self.natural_frequency * self.target_dist
       
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info("PID Distance Controller Node Started")
        
        
        
    def listener_callback(self, msg):
        # Get forward-facing distance
        forward_idx = len(msg.ranges) // 2
        distance = msg.ranges[forward_idx]

        # Compute error
        error = distance - self.target_dist
        current_time = time.time()
        dt = current_time - self.prev_time if current_time - self.prev_time > 0 else 1e-6

        # PID terms
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        control = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # Limit control signal
        control = max(min(control, 0.15), -0.15)

        # Publish velocity
        twist = Twist()
        twist.linear.x = control
        self.publisher.publish(twist)

        # Logging
        self.get_logger().info(f"Distance: {distance:.3f}, Error: {error:.3f}, Control: {control:.3f}")

        # Update state
        self.prev_error = error
        self.prev_time = current_time


def main(args=None):
    rclpy.init(args=args)
    pid_speed_controller = PID_control()
    rclpy.spin(pid_speed_controller)
    pid_speed_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()