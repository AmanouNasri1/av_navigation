#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
import sys, termios, tty, select, os

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(AckermannDrive, '/cmd_ackermann', 10)

        self.speed = 0.0          # current speed
        self.steering_angle = 0.0 # current steering angle
        self.speed_step = 0.5     # m/s increment per keypress
        self.steer_step = 0.1     # rad increment per keypress
        self.max_speed = 80.0
        self.max_steer = 0.5      # about 30 degrees

        self.get_logger().info("Keyboard teleop started — use W/S/A/D, Space=stop, Q=quit")

        # Non-blocking keyboard
        self.settings = termios.tcgetattr(sys.stdin)

        # Timer to republish drive commands at 10 Hz
        self.create_timer(0.1, self.publish_command)

    def publish_command(self):
        msg = AckermannDrive()
        msg.speed = self.speed
        msg.steering_angle = self.steering_angle
        self.publisher_.publish(msg)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key().lower()

                if key == 'w':
                    self.speed = min(self.speed + self.speed_step, self.max_speed)
                elif key == 's':
                    self.speed = max(self.speed - self.speed_step, -self.max_speed/2)
                elif key == 'a':
                    self.steering_angle = max(self.steering_angle - self.steer_step, -self.max_steer)
                elif key == 'd':
                    self.steering_angle = min(self.steering_angle + self.steer_step, self.max_steer)
                elif key == ' ':
                    self.speed = 0.0
                    self.steering_angle = 0.0
                elif key == 'q':
                    self.get_logger().info('Exiting teleop.')
                    break

                rclpy.spin_once(self, timeout_sec=0.01)
        finally:
            self.speed = 0.0
            self.steering_angle = 0.0
            self.publish_command()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()