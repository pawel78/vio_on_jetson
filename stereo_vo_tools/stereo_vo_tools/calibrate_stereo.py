#!/usr/bin/env python3
"""
Stereo camera calibration using OpenCV
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import argparse
from pathlib import Path


class StereoCalibrator(Node):
    def __init__(self, args):
        super().__init__('stereo_calibrator')
        
        self.args = args
        self.bridge = CvBridge()
        
        # Calibration pattern
        if args.pattern == 'chessboard':
            self.pattern_size = tuple(map(int, args.size.split('x')))
            self.square_size = args.square
        else:
            self.get_logger().error('Only chessboard pattern supported currently')
            return
        
        # Storage for calibration images
        self.left_images = []
        self.right_images = []
        self.image_points_left = []
        self.image_points_right = []
        
        # Prepare object points
        objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.pattern_size[0], 
                                0:self.pattern_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        self.objpoints = []
        
        # Subscribers
        self.left_sub = self.create_subscription(
            Image, args.left_topic, self.left_callback, 10)
        self.right_sub = self.create_subscription(
            Image, args.right_topic, self.right_callback, 10)
        
        self.left_image = None
        self.right_image = None
        
        # Timer for processing
        self.timer = self.create_timer(0.5, self.process_images)
        
        self.get_logger().info(f'Stereo calibrator started')
        self.get_logger().info(f'Pattern: {args.pattern} {self.pattern_size}')
        self.get_logger().info(f'Target images: {args.num_images}')
        self.get_logger().info('Press SPACE to capture, ESC to finish, Q to quit')
    
    def left_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def right_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def process_images(self):
        if self.left_image is None or self.right_image is None:
            return
        
        # Find chessboard corners
        left_gray = cv2.cvtColor(self.left_image, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(self.right_image, cv2.COLOR_BGR2GRAY)
        
        ret_left, corners_left = cv2.findChessboardCorners(
            left_gray, self.pattern_size, None)
        ret_right, corners_right = cv2.findChessboardCorners(
            right_gray, self.pattern_size, None)
        
        # Draw corners
        vis_left = self.left_image.copy()
        vis_right = self.right_image.copy()
        
        if ret_left:
            cv2.drawChessboardCorners(vis_left, self.pattern_size, corners_left, ret_left)
        if ret_right:
            cv2.drawChessboardCorners(vis_right, self.pattern_size, corners_right, ret_right)
        
        # Display
        cv2.imshow('Left Camera', vis_left)
        cv2.imshow('Right Camera', vis_right)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord(' ') and ret_left and ret_right:
            # Capture this pair
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_left_refined = cv2.cornerSubPix(
                left_gray, corners_left, (11, 11), (-1, -1), criteria)
            corners_right_refined = cv2.cornerSubPix(
                right_gray, corners_right, (11, 11), (-1, -1), criteria)
            
            self.image_points_left.append(corners_left_refined)
            self.image_points_right.append(corners_right_refined)
            self.objpoints.append(self.objpoints[0] if self.objpoints else 
                                 np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32))
            
            self.get_logger().info(f'Captured {len(self.image_points_left)}/{self.args.num_images}')
            
            if len(self.image_points_left) >= self.args.num_images:
                self.get_logger().info('Sufficient images collected, calibrating...')
                self.calibrate()
                return
        
        elif key == 27:  # ESC
            if len(self.image_points_left) >= 5:
                self.get_logger().info('Calibrating with collected images...')
                self.calibrate()
            else:
                self.get_logger().warn('Not enough images for calibration')
        
        elif key == ord('q'):
            self.get_logger().info('Quit without calibration')
            rclpy.shutdown()
    
    def calibrate(self):
        self.get_logger().info('Running stereo calibration...')
        
        # Monocular calibrations
        h, w = self.left_image.shape[:2]
        
        ret_left, K_left, D_left, rvecs_left, tvecs_left = cv2.calibrateCamera(
            self.objpoints, self.image_points_left, (w, h), None, None)
        
        ret_right, K_right, D_right, rvecs_right, tvecs_right = cv2.calibrateCamera(
            self.objpoints, self.image_points_right, (w, h), None, None)
        
        self.get_logger().info(f'Left RMS error: {ret_left:.4f}')
        self.get_logger().info(f'Right RMS error: {ret_right:.4f}')
        
        # Stereo calibration
        flags = cv2.CALIB_FIX_INTRINSIC
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
        
        ret_stereo, K_left, D_left, K_right, D_right, R, T, E, F = cv2.stereoCalibrate(
            self.objpoints, self.image_points_left, self.image_points_right,
            K_left, D_left, K_right, D_right, (w, h),
            criteria=criteria, flags=flags)
        
        self.get_logger().info(f'Stereo RMS error: {ret_stereo:.4f}')
        self.get_logger().info(f'Baseline: {np.linalg.norm(T):.4f} m')
        
        # Stereo rectification
        R1, R2, P1, P2, Q, roi_left, roi_right = cv2.stereoRectify(
            K_left, D_left, K_right, D_right, (w, h), R, T, alpha=0)
        
        # Save calibration
        self.save_calibration(K_left, D_left, R1, P1, 'left')
        self.save_calibration(K_right, D_right, R2, P2, 'right')
        
        self.get_logger().info('Calibration complete!')
        cv2.destroyAllWindows()
        rclpy.shutdown()
    
    def save_calibration(self, K, D, R, P, side):
        h, w = self.left_image.shape[:2]
        
        calib_data = {
            'image_width': w,
            'image_height': h,
            'camera_name': f'{side}_camera',
            'camera_matrix': {
                'rows': 3,
                'cols': 3,
                'data': K.flatten().tolist()
            },
            'distortion_model': 'plumb_bob',
            'distortion_coefficients': {
                'rows': 1,
                'cols': 5,
                'data': D.flatten().tolist()
            },
            'rectification_matrix': {
                'rows': 3,
                'cols': 3,
                'data': R.flatten().tolist()
            },
            'projection_matrix': {
                'rows': 3,
                'cols': 4,
                'data': P.flatten().tolist()
            }
        }
        
        output_file = Path(self.args.output) / f'camera_{side}.yaml'
        output_file.parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_file, 'w') as f:
            yaml.dump(calib_data, f, default_flow_style=False)
        
        self.get_logger().info(f'Saved {side} calibration to {output_file}')


def main(args=None):
    parser = argparse.ArgumentParser(description='Stereo camera calibration')
    parser.add_argument('--left', dest='left_topic', default='/stereo/left/image_raw',
                        help='Left camera topic')
    parser.add_argument('--right', dest='right_topic', default='/stereo/right/image_raw',
                        help='Right camera topic')
    parser.add_argument('--pattern', default='chessboard',
                        help='Calibration pattern type')
    parser.add_argument('--size', default='9x6',
                        help='Pattern size (e.g., 9x6)')
    parser.add_argument('--square', type=float, default=0.025,
                        help='Square size in meters')
    parser.add_argument('--num-images', type=int, default=20,
                        help='Number of calibration images to collect')
    parser.add_argument('--output', default='.',
                        help='Output directory for calibration files')
    
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)
    node = StereoCalibrator(parsed_args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
