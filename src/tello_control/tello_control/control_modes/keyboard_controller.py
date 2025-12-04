#!/usr/bin/env python3

import sys
import tty
import termios
import threading
import select
import time
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
        
        # Track pressed keys untuk smooth movement
        self.active_keys = set()
        
        # Key timeout - jika tidak ada input dalam waktu ini, stop
        self.key_timeout = 2.0  # 2000ms
        self.last_key_time = 0.0
        
        # Key repeat tracking untuk stabilitas
        self.key_last_seen = {}  # key -> timestamp terakhir ditekan
        self.key_release_delay = 0.15  # delay sebelum key dianggap dilepas (150ms)
        
        self.get_logger().info(f'Keyboard controller started on topic: {topic}')
        self.get_logger().info(f'Service client: {service_name}')
        self.get_logger().info(f'Speed: {self.speed}, Yaw speed: {self.yaw_speed}')
        self.print_instructions()
        
    def print_instructions(self):
        print("\n" + "="*50)
        print("TELLO KEYBOARD CONTROLLER")
        print("="*50)
        print("MOVEMENT (tahan untuk bergerak terus):")
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
        if not self.running:
            return
            
        current_time = time.time()
        
        # Check key release dengan delay untuk stabilitas
        # Ini mencegah flicker saat keyboard auto-repeat
        keys_to_remove = []
        for key in list(self.active_keys):
            if key in self.key_last_seen:
                time_since_last = current_time - self.key_last_seen[key]
                if time_since_last > self.key_release_delay:
                    keys_to_remove.append(key)
        
        if keys_to_remove:
            for key in keys_to_remove:
                self.active_keys.discard(key)
                if key in self.key_last_seen:
                    del self.key_last_seen[key]
            self.update_twist_from_active_keys()
            
        # Check timeout - jika tidak ada input dalam key_timeout, stop movement
        if self.last_key_time > 0 and (current_time - self.last_key_time) > self.key_timeout:
            # Timeout - clear active keys dan stop
            if self.active_keys:
                self.active_keys.clear()
                self.key_last_seen.clear()
                self.update_twist_from_active_keys()
        
        self.publisher.publish(self.twist)
    
    def update_twist_from_active_keys(self):
        """Update twist berdasarkan active keys yang sedang ditekan"""
        # Reset twist
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.z = 0.0
        
        # Apply velocity berdasarkan active keys
        if 'w' in self.active_keys:
            self.twist.linear.x += self.speed
        if 's' in self.active_keys:
            self.twist.linear.x -= self.speed
        if 'a' in self.active_keys:
            self.twist.linear.y += self.speed
        if 'd' in self.active_keys:
            self.twist.linear.y -= self.speed
        if 'i' in self.active_keys:
            self.twist.linear.z += self.speed
        if 'k' in self.active_keys:
            self.twist.linear.z -= self.speed
        if 'q' in self.active_keys:
            self.twist.angular.z += self.yaw_speed
        if 'e' in self.active_keys:
            self.twist.angular.z -= self.yaw_speed
    
    def process_key(self, key):
        """Process keyboard input"""
        key = key.lower()
        current_time = time.time()
        self.last_key_time = current_time
        
        # Movement keys - add/update in active_keys
        movement_keys = {'w', 's', 'a', 'd', 'i', 'k', 'q', 'e'}
        
        if key in movement_keys:
            self.active_keys.add(key)
            self.key_last_seen[key] = current_time  # Update timestamp
            self.update_twist_from_active_keys()
            # Reset flip confirmation
            if key not in ['f', 'b', 'r', 'v']:
                self.last_key = None
                
        elif key == ' ':
            # Spacebar = stop/hover - clear all movement
            self.active_keys.clear()
            self.key_last_seen.clear()
            self.update_twist_from_active_keys()
            self.last_key = None
            self.get_logger().info('Stop/Hover')
            
        elif key == 't':
            self.call_tello_action('takeoff')
            self.last_key = None
            
        elif key == 'l':
            self.call_tello_action('land')
            self.last_key = None
            
        elif key == 'h':
            self.active_keys.clear()
            self.key_last_seen.clear()
            self.update_twist_from_active_keys()
            self.call_tello_action('emergency')
            self.get_logger().warn('EMERGENCY STOP!')
            self.last_key = None
            
        elif key == 'f':
            if self.last_key == 'f':
                self.call_tello_action('flip f')
                self.last_key = None
            else:
                self.get_logger().info('Press F again to confirm flip forward')
                self.last_key = 'f'
                
        elif key == 'b':
            if self.last_key == 'b':
                self.call_tello_action('flip b')
                self.last_key = None
            else:
                self.get_logger().info('Press B again to confirm flip backward')
                self.last_key = 'b'
                
        elif key == 'r':
            if self.last_key == 'r':
                self.call_tello_action('flip r')
                self.last_key = None
            else:
                self.get_logger().info('Press R again to confirm flip right')
                self.last_key = 'r'
                
        elif key == 'v':
            if self.last_key == 'v':
                self.call_tello_action('flip l')
                self.last_key = None
            else:
                self.get_logger().info('Press V again to confirm flip left')
                self.last_key = 'v'
                
        elif key == '+' or key == '=':
            self.speed = min(1.0, self.speed + 0.1)
            self.yaw_speed = min(1.0, self.yaw_speed + 0.1)
            self.get_logger().info(f'Speed increased: {self.speed:.1f}')
            # Update twist dengan speed baru
            self.update_twist_from_active_keys()
            
        elif key == '-' or key == '_':
            self.speed = max(0.1, self.speed - 0.1)
            self.yaw_speed = max(0.1, self.yaw_speed - 0.1)
            self.get_logger().info(f'Speed decreased: {self.speed:.1f}')
            # Update twist dengan speed baru
            self.update_twist_from_active_keys()
    
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
        self.active_keys.clear()
        self.key_last_seen.clear()
        self.twist = Twist()  # Reset to zero
        self.publisher.publish(self.twist)


def get_key_nonblocking(timeout=0.1):
    """Non-blocking keyboard read dengan timeout"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        # Use select untuk non-blocking read dengan timeout
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            ch = sys.stdin.read(1)
            return ch
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    controller = KeyboardController()
    
    # Thread untuk ROS spin
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()
    
    try:
        while controller.running:
            # Non-blocking read dengan timeout pendek
            key = get_key_nonblocking(timeout=0.05)
            
            if key is not None:
                # ESC atau Ctrl+C untuk keluar
                if key == '\x1b' or key == '\x03':
                    print("\nKeluar...")
                    break
                
                controller.process_key(key)
            
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
