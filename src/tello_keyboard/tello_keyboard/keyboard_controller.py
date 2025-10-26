#!/usr/bin/env python3

import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        
        # Parameter
        self.declare_parameter('namespace', 'drone1')
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('yaw_speed', 0.5)
        
        ns = self.get_parameter('namespace').value
        self.speed = self.get_parameter('speed').value
        self.yaw_speed = self.get_parameter('yaw_speed').value
        
        # Publisher
        topic = f'/{ns}/cmd_vel' if ns else '/cmd_vel'
        self.publisher = self.create_publisher(Twist, topic, 10)
        
        # Service client untuk tello_action
        service_name = f'/{ns}/tello_action' if ns else '/tello_action'
        self.tello_client = self.create_client(TelloAction, service_name)
        
        # Timer untuk publish secara periodik
        self.timer = self.create_timer(0.05, self.publish_twist)  # 20 Hz
        
        # State
        self.twist = Twist()
        self.running = True
        self.namespace = ns
        self.last_key = None  # For flip confirmation
        
        self.get_logger().info(f'Keyboard controller started on topic: {topic}')
        self.get_logger().info(f'Service client: {service_name}')
        self.get_logger().info(f'Speed: {self.speed}, Yaw speed: {self.yaw_speed}')
        self.print_instructions()
        
    def print_instructions(self):
        print("\n" + "="*50)
        print("TELLO KEYBOARD CONTROLLER")
        print("="*50)
        print("MOVEMENT:")
        print("  W/S : Maju/Mundur")
        print("  A/D : Kiri/Kanan")
        print("  Q/E : Yaw kiri/kanan")
        print("  I/K : Naik/Turun")
        print("  SPACE : Stop/Hover")
        print("")
        print("COMMANDS:")
        print("  T : Takeoff")
        print("  L : Land")
        print("  H : Emergency stop")
        print("")
        print("FLIPS (tekan 2x untuk konfirmasi):")
        print("  F : Flip forward")
        print("  B : Flip backward")
        print("  R : Flip right")
        print("  V : Flip left")
        print("")
        print("SPEED CONTROL:")
        print("  + : Increase speed")
        print("  - : Decrease speed")
        print("")
        print("ESC : Keluar")
        print("="*50 + "\n")
        
    def publish_twist(self):
        """Publish Twist message secara periodik"""
        if self.running:
            self.publisher.publish(self.twist)
    
    def update_twist(self, key):
        """Update twist berdasarkan input keyboard"""
        # Reset semua ke 0 dulu
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.z = 0.0
        
        # Update berdasarkan key
        if key == 'w':
            self.twist.linear.x = self.speed
            # self.get_logger().info('Maju')
        elif key == 's':
            self.twist.linear.x = -self.speed
            # self.get_logger().info('Mundur')
        elif key == 'a':
            self.twist.linear.y = self.speed
            # self.get_logger().info('Kiri')
        elif key == 'd':
            self.twist.linear.y = -self.speed
            # self.get_logger().info('Kanan')
        elif key == 'i':
            self.twist.linear.z = self.speed
            # self.get_logger().info('Naik')
        elif key == 'k':
            self.twist.linear.z = -self.speed
            # self.get_logger().info('Turun')
        elif key == 'q':
            self.twist.angular.z = self.yaw_speed
            # self.get_logger().info('Yaw kiri')
        elif key == 'e':
            self.twist.angular.z = -self.yaw_speed
            # self.get_logger().info('Yaw kanan')
        elif key == 't':
            # self.get_logger().info('Takeoff command...')
            self.call_tello_action('takeoff')
        elif key == 'l':
            # self.get_logger().info('Land command...')
            self.call_tello_action('land')
        elif key == 'h':
            # self.get_logger().warn('EMERGENCY STOP!')
            self.call_tello_action('emergency')
        elif key == 'f':
            # Flip forward (requires double press)
            if self.last_key == 'f':
                # self.get_logger().info('Flip forward!')
                self.call_tello_action('flip f')
                self.last_key = None
            else:
                # self.get_logger().info('Press F again to confirm flip forward')
                self.last_key = 'f'
        elif key == 'b':
            # Flip backward
            if self.last_key == 'b':
                # self.get_logger().info('Flip backward!')
                self.call_tello_action('flip b')
                self.last_key = None
            else:
                # self.get_logger().info('Press B again to confirm flip backward')
                self.last_key = 'b'
        elif key == 'r':
            # Flip right
            if self.last_key == 'r':
                # self.get_logger().info('Flip right!')
                self.call_tello_action('flip r')
                self.last_key = None
            else:
                # self.get_logger().info('Press R again to confirm flip right')
                self.last_key = 'r'
        elif key == 'v':
            # Flip left
            if self.last_key == 'v':
                # self.get_logger().info('Flip left!')
                self.call_tello_action('flip l')
                self.last_key = None
            else:
                # self.get_logger().info('Press V again to confirm flip left')
                self.last_key = 'v'
        elif key == '+' or key == '=':
            self.speed = min(1.0, self.speed + 0.1)
            self.yaw_speed = min(1.0, self.yaw_speed + 0.1)
            # self.get_logger().info(f'Speed increased: {self.speed:.1f}')
        elif key == '-' or key == '_':
            self.speed = max(0.1, self.speed - 0.1)
            self.yaw_speed = max(0.1, self.yaw_speed - 0.1)
            # self.get_logger().info(f'Speed decreased: {self.speed:.1f}')
        elif key == ' ':
            # Spacebar = stop/hover
            #self.get_logger().info('Stop/Hover')
            self.last_key = None
        else:
            # Reset last_key for non-flip keys
            if key not in ['f', 'b', 'r', 'v']:
                self.last_key = None
    
    def call_tello_action(self, command):
        """Call tello_action service"""
        if not self.tello_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'Service /{self.namespace}/tello_action not available!')
            self.get_logger().warn('Pastikan tello_driver atau simulator dengan TelloPlugin sudah berjalan')
            return
        
        request = TelloAction.Request()
        request.cmd = command
        
        future = self.tello_client.call_async(request)
        future.add_done_callback(lambda f: self.service_response_callback(f, command))
    
    def service_response_callback(self, future, command):
        try:
            response = future.result()
            if response.rc == TelloAction.Response.OK:
                self.get_logger().info(f'✓ Command "{command}" sent successfully')
            elif response.rc == TelloAction.Response.ERROR_NOT_CONNECTED:
                self.get_logger().error(f'✗ Command "{command}" failed: Not connected to drone')
            elif response.rc == TelloAction.Response.ERROR_BUSY:
                self.get_logger().warn(f'⚠ Command "{command}" failed: Drone busy with another command')
            else:
                self.get_logger().error(f'✗ Command "{command}" failed with code: {response.rc}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
        
    def stop(self):
        self.running = False
        self.twist = Twist()  # Reset to zero
        self.publisher.publish(self.twist)


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main(args=None):
    rclpy.init(args=args)
    controller = KeyboardController()
    
    # Thread untuk ROS spin
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()
    
    try:
        while controller.running:
            key = get_key()
            
            # ESC atau Ctrl+C untuk keluar
            if key == '\x1b' or key == '\x03':
                print("\nKeluar...")
                break
            
            controller.update_twist(key.lower())
            
    except KeyboardInterrupt:
        print("\nKeyboard Interrupt")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        controller.stop()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
