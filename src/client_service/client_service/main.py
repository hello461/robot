#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
from  example_interfaces.msg import Float32 
from std_msgs.msg import Float32MultiArray 
import numpy as np
import time

class LowBandwidthEncoder(Node):
    def __init__(self):
        super().__init__('low_bandwidth_encoder')
        
        # ==================== PARAMETERS ====================
        
        # Compression mode: 'ultra_low', 'low', 'medium', 'high'
        self.declare_parameter('bandwidth_mode', 'low')
        
        # Advanced parameters
        self.declare_parameter('target_fps', 15)  # Lower FPS = less bandwidth
        self.declare_parameter('adaptive_quality', True)  # Auto adjust quality
        self.declare_parameter('motion_detection', True)  # Only send when motion
        self.declare_parameter('grayscale', False)  # Grayscale = 33% less bandwidth
        
        self.bandwidth_mode = self.get_parameter('bandwidth_mode').value
        self.target_fps = self.get_parameter('target_fps').value
        self.adaptive_quality = self.get_parameter('adaptive_quality').value
        self.motion_detection = self.get_parameter('motion_detection').value
        self.use_grayscale = self.get_parameter('grayscale').value
        
        # Load mode presets
        self.load_bandwidth_preset()
        
        self.bridge = CvBridge()
        
        # ==================== SUBSCRIBERS & PUBLISHERS ====================
        
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.image_publisher = self.create_publisher(
            CompressedImage,
            '/camera/color/compressed',
            10
        )
        
        self.status_subscription = self.create_subscription(
            Float32,
            '/go2_status',
            self.battery_callback,
            10
        )
        
        self.status_publisher = self.create_publisher(
            Float32MultiArray,
            '/status',
            10
        )
        
        # ==================== STATE VARIABLES ====================
        
        self.frame_count = 0
        self.last_frame_time = time.time()
        self.frame_interval = 1.0 / self.target_fps
        
        # Motion detection
        self.previous_frame = None
        self.motion_threshold = 30  # Pixel difference threshold
        
        # Adaptive quality
        self.current_quality = self.jpeg_quality
        self.bandwidth_history = []
        self.max_history = 10
        
        # Statistics
        self.total_bytes_sent = 0
        self.frames_sent = 0
        self.frames_skipped = 0
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🔥 LOW BANDWIDTH VIDEO ENCODER 🔥')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Mode: {self.bandwidth_mode.upper()}')
        self.get_logger().info(f'  Target FPS: {self.target_fps}')
        self.get_logger().info(f'  Resolution: {self.width}x{self.height}')
        self.get_logger().info(f'  JPEG Quality: {self.jpeg_quality}%')
        self.get_logger().info(f'  Estimated Bandwidth: {self.estimated_bandwidth}')
        self.get_logger().info(f'  Adaptive Quality: {"ON" if self.adaptive_quality else "OFF"}')
        self.get_logger().info(f'  Motion Detection: {"ON" if self.motion_detection else "OFF"}')
        self.get_logger().info(f'  Grayscale: {"ON" if self.use_grayscale else "OFF"}')
        self.get_logger().info('=' * 60)
    
    def load_bandwidth_preset(self):
        """Load bandwidth optimization presets"""
        
        presets = {
            'ultra_low': {
                'width': 320,
                'height': 240,
                'jpeg_quality': 50,
                'estimated_bandwidth': '~150-250 KB/s (1-2 Mbps)',
                'description': 'Extreme compression for very slow connections'
            },
            'low': {
                'width': 480,
                'height': 360,
                'jpeg_quality': 60,
                'estimated_bandwidth': '~300-500 KB/s (2-4 Mbps)',
                'description': 'Good for 3G/4G mobile or slow WiFi'
            },
            'medium': {
                'width': 640,
                'height': 480,
                'jpeg_quality': 70,
                'estimated_bandwidth': '~600-900 KB/s (5-7 Mbps)',
                'description': 'Balanced quality and bandwidth'
            },
            'high': {
                'width': 800,
                'height': 600,
                'jpeg_quality': 80,
                'estimated_bandwidth': '~1-1.5 MB/s (8-12 Mbps)',
                'description': 'High quality for good connections'
            }
        }
        
        if self.bandwidth_mode not in presets:
            self.get_logger().warn(f'Unknown mode "{self.bandwidth_mode}", using "low"')
            self.bandwidth_mode = 'low'
        
        preset = presets[self.bandwidth_mode]
        self.width = preset['width']
        self.height = preset['height']
        self.jpeg_quality = preset['jpeg_quality']
        self.estimated_bandwidth = preset['estimated_bandwidth']
    
    def image_callback(self, msg):
        """Process and compress image with bandwidth optimization"""
        
        try:
            # FPS limiting
            current_time = time.time()
            time_since_last = current_time - self.last_frame_time
            
            if time_since_last < self.frame_interval:
                # Skip frame to maintain target FPS
                return
            
            self.last_frame_time = current_time
            
            # Convert to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize for bandwidth reduction
            if (cv_image.shape[1] != self.width or 
                cv_image.shape[0] != self.height):
                cv_image = cv2.resize(
                    cv_image, 
                    (self.width, self.height),
                    interpolation=cv2.INTER_AREA  # Better for downscaling
                )
            
            # Motion detection (skip frame if no motion)
            if self.motion_detection and self.previous_frame is not None:
                if not self.detect_motion(cv_image):
                    self.frames_skipped += 1
                    return
            
            self.previous_frame = cv_image.copy()
            
            # Grayscale conversion (reduces bandwidth by ~33%)
            if self.use_grayscale:
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            
            # Adaptive quality adjustment
            if self.adaptive_quality:
                self.adjust_quality()
            
            # JPEG compression with optimization
            encode_params = [
                int(cv2.IMWRITE_JPEG_QUALITY), self.current_quality,
                int(cv2.IMWRITE_JPEG_OPTIMIZE), 1,
                int(cv2.IMWRITE_JPEG_PROGRESSIVE), 0  # Disable for smaller size
            ]
            
            success, encoded_image = cv2.imencode('.jpg', cv_image, encode_params)
            
            if success:
                # Create compressed message
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = 'jpeg'
                compressed_msg.data = encoded_image.tobytes()
                
                # Publish
                self.image_publisher.publish(compressed_msg)
                
                # Statistics
                frame_size = len(compressed_msg.data)
                self.total_bytes_sent += frame_size
                self.frames_sent += 1
                self.frame_count += 1
                
                # Track bandwidth
                if self.adaptive_quality:
                    self.bandwidth_history.append(frame_size)
                    if len(self.bandwidth_history) > self.max_history:
                        self.bandwidth_history.pop(0)
                
                # Log every 100 frames
                if self.frame_count % 100 == 0:
                    avg_frame_size = self.total_bytes_sent / self.frames_sent / 1024
                    bandwidth_kbps = (self.total_bytes_sent * 8) / (self.frame_count * self.frame_interval) / 1000
                    skip_rate = (self.frames_skipped / self.frame_count) * 100 if self.frame_count > 0 else 0
                    
                    self.get_logger().info(
                        f'📊 Frame {self.frame_count}: '
                        f'Avg {avg_frame_size:.1f} KB/frame, '
                        f'BW: {bandwidth_kbps:.1f} kbps, '
                        f'Quality: {self.current_quality}%, '
                        f'Skip: {skip_rate:.1f}%'
                    )
            
        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {e}')
    
    def detect_motion(self, current_frame):
        """Detect motion between frames"""
        if self.previous_frame is None:
            return True
        
        try:
            # Convert to grayscale for comparison
            gray_current = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
            gray_previous = cv2.cvtColor(self.previous_frame, cv2.COLOR_BGR2GRAY)
            
            # Calculate difference
            diff = cv2.absdiff(gray_current, gray_previous)
            
            # Threshold
            _, thresh = cv2.threshold(diff, self.motion_threshold, 255, cv2.THRESH_BINARY)
            
            # Count non-zero pixels (motion pixels)
            motion_pixels = cv2.countNonZero(thresh)
            total_pixels = thresh.shape[0] * thresh.shape[1]
            
            # If more than 1% pixels changed, consider it motion
            motion_ratio = motion_pixels / total_pixels
            
            return motion_ratio > 0.01
            
        except Exception as e:
            self.get_logger().error(f'Motion detection error: {e}')
            return True  # Send frame on error
    
    def adjust_quality(self):
        """Adaptively adjust JPEG quality based on bandwidth usage"""
        if len(self.bandwidth_history) < 5:
            return
        
        # Calculate average frame size
        avg_size = sum(self.bandwidth_history) / len(self.bandwidth_history)
        
        # Target frame size based on mode
        target_sizes = {
            'ultra_low': 15 * 1024,   # 15 KB
            'low': 30 * 1024,          # 30 KB
            'medium': 60 * 1024,       # 60 KB
            'high': 100 * 1024         # 100 KB
        }
        
        target_size = target_sizes.get(self.bandwidth_mode, 30 * 1024)
        
        # Adjust quality
        if avg_size > target_size * 1.2:  # 20% over target
            self.current_quality = max(30, self.current_quality - 5)
        elif avg_size < target_size * 0.8:  # 20% under target
            self.current_quality = min(self.jpeg_quality + 10, self.current_quality + 5)
        
        # Clamp quality
        self.current_quality = max(30, min(95, self.current_quality))
    
    def battery_callback(self, msg):
        """Process battery and temperature status"""
        try:
            battery_voltage = float(msg.data)
            
            driver_temp = 0.0
            motor_temp = 0.0
            try:
                if len(msg.actuator_states) > 2:
                    driver_temp = float(msg.actuator_states[2].driver_temperature)
                    motor_temp = float(msg.actuator_states[2].motor_temperature)
            except:
                pass
            
            status_array = Float32MultiArray()
            status_array.data = [battery_voltage, driver_temp, motor_temp]
            
            self.status_publisher.publish(status_array)
            
        except Exception as e:
            self.get_logger().error(f'Battery callback error: {e}')
    
    def destroy_node(self):
        """Cleanup and show statistics"""
        if self.frames_sent > 0:
            avg_frame_size = self.total_bytes_sent / self.frames_sent / 1024
            total_mb = self.total_bytes_sent / (1024 * 1024)
            avg_bandwidth_kbps = (self.total_bytes_sent * 8) / (self.frame_count * self.frame_interval) / 1000
            skip_rate = (self.frames_skipped / self.frame_count) * 100 if self.frame_count > 0 else 0
            
            self.get_logger().info('=' * 60)
            self.get_logger().info('📊 LOW BANDWIDTH ENCODER STATISTICS')
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'  Mode: {self.bandwidth_mode.upper()}')
            self.get_logger().info(f'  Total frames processed: {self.frame_count}')
            self.get_logger().info(f'  Frames sent: {self.frames_sent}')
            self.get_logger().info(f'  Frames skipped: {self.frames_skipped} ({skip_rate:.1f}%)')
            self.get_logger().info(f'  Avg frame size: {avg_frame_size:.1f} KB')
            self.get_logger().info(f'  Total data sent: {total_mb:.1f} MB')
            self.get_logger().info(f'  Avg bandwidth: {avg_bandwidth_kbps:.1f} kbps')
            self.get_logger().info(f'  Data saved vs uncompressed: ~{(1 - total_mb / (self.frame_count * 0.5)) * 100:.0f}%')
            self.get_logger().info('=' * 60)
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = LowBandwidthEncoder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()