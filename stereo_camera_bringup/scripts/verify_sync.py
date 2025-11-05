#!/usr/bin/env python3
"""
Verify stereo camera synchronization by comparing timestamps
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
import time


class SyncVerifier(Node):
    def __init__(self):
        super().__init__('sync_verifier')
        
        # Subscribers with time synchronizer
        self.left_sub = Subscriber(self, Image, '/stereo/left/image_raw')
        self.right_sub = Subscriber(self, Image, '/stereo/right/image_raw')
        
        self.sync = ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub],
            queue_size=10,
            slop=0.01  # 10ms tolerance
        )
        self.sync.registerCallback(self.sync_callback)
        
        self.frame_count = 0
        self.max_skew = 0.0
        self.total_skew = 0.0
        self.start_time = time.time()
        
        self.get_logger().info('Stereo sync verifier started')
        self.get_logger().info('Will analyze first 100 frames...')
    
    def sync_callback(self, left_msg, right_msg):
        self.frame_count += 1
        
        # Calculate timestamp difference
        left_time = left_msg.header.stamp.sec + left_msg.header.stamp.nanosec * 1e-9
        right_time = right_msg.header.stamp.sec + right_msg.header.stamp.nanosec * 1e-9
        skew = abs(left_time - right_time) * 1000.0  # Convert to ms
        
        self.total_skew += skew
        if skew > self.max_skew:
            self.max_skew = skew
        
        if self.frame_count % 10 == 0:
            avg_skew = self.total_skew / self.frame_count
            elapsed = time.time() - self.start_time
            fps = self.frame_count / elapsed if elapsed > 0 else 0
            
            self.get_logger().info(
                f'Frame {self.frame_count}: '
                f'Current skew: {skew:.3f} ms, '
                f'Average: {avg_skew:.3f} ms, '
                f'Max: {self.max_skew:.3f} ms, '
                f'FPS: {fps:.1f}'
            )
        
        if self.frame_count >= 100:
            avg_skew = self.total_skew / self.frame_count
            elapsed = time.time() - self.start_time
            fps = self.frame_count / elapsed if elapsed > 0 else 0
            
            self.get_logger().info('\n' + '='*60)
            self.get_logger().info('SYNCHRONIZATION VERIFICATION RESULTS')
            self.get_logger().info('='*60)
            self.get_logger().info(f'Total frames analyzed: {self.frame_count}')
            self.get_logger().info(f'Average timestamp skew: {avg_skew:.3f} ms')
            self.get_logger().info(f'Maximum timestamp skew: {self.max_skew:.3f} ms')
            self.get_logger().info(f'Average framerate: {fps:.1f} FPS')
            
            if self.max_skew < 1.0:
                self.get_logger().info('✓ PASS: Synchronization is excellent (< 1 ms)')
            elif self.max_skew < 5.0:
                self.get_logger().warn('⚠ WARNING: Synchronization acceptable but not ideal (< 5 ms)')
            else:
                self.get_logger().error('✗ FAIL: Synchronization poor (> 5 ms)')
            
            self.get_logger().info('='*60)
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SyncVerifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
