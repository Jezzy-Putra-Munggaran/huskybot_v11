#!/usr/bin/env python3
"""
Automatic Camera-LiDAR Calibration Node for ULTRA ACCURATE Sensor Fusion
Author: Jezzy Putra Munggaran
Date: August 5, 2025
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import sensor_msgs_py.point_cloud2 as pc2
from scipy.optimize import minimize
import yaml
import os

class AutoCalibrationNode(Node):
    def __init__(self):
        super().__init__('auto_calibration_node')
        
        self.bridge = CvBridge()
        
        # ✅ Calibration parameters
        self.declare_parameter('camera_topic', '/camera_front/image_raw')
        self.declare_parameter('lidar_topic', '/velodyne_points')
        self.declare_parameter('output_file', '/tmp/camera_lidar_calibration.yaml')
        
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        
        # ✅ Data storage
        self.latest_image = None
        self.latest_pointcloud = None
        self.image_lock = threading.Lock()
        self.pointcloud_lock = threading.Lock()
        
        # ✅ Calibration state
        self.calibration_active = False
        self.calibration_data = []
        
        # ✅ Setup connections
        self.setup_connections()
        
        self.get_logger().info("🚀 AUTO CALIBRATION NODE STARTED!")
        self.get_logger().info("📋 Instructions:")
        self.get_logger().info("   1. Place calibration target (checkerboard/AprilTag) in view")
        self.get_logger().info("   2. Call /start_calibration service")
        self.get_logger().info("   3. Move target to different positions")
        self.get_logger().info("   4. Call /save_calibration service")

    def setup_connections(self):
        """Setup ROS2 connections"""
        try:
            # Subscribers
            self.camera_sub = self.create_subscription(
                Image, self.camera_topic, self.camera_callback, 10)
            self.lidar_sub = self.create_subscription(
                PointCloud2, self.lidar_topic, self.lidar_callback, 10)
            
            # Publishers
            self.calibration_viz_pub = self.create_publisher(
                Image, '/calibration_visualization', 10)
            
            # Services
            from std_srvs.srv import Trigger
            self.start_calib_srv = self.create_service(
                Trigger, 'start_calibration', self.start_calibration_callback)
            self.save_calib_srv = self.create_service(
                Trigger, 'save_calibration', self.save_calibration_callback)
            self.capture_sample_srv = self.create_service(
                Trigger, 'capture_calibration_sample', self.capture_sample_callback)
            
            self.get_logger().info("📡 Connections setup complete!")
            
        except Exception as e:
            self.get_logger().error(f"❌ Connection setup failed: {e}")

    def camera_callback(self, msg):
        """Camera image callback"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.image_lock:
                self.latest_image = cv_image.copy()
        except Exception as e:
            self.get_logger().error(f"❌ Camera callback error: {e}")

    def lidar_callback(self, msg):
        """LiDAR point cloud callback"""
        try:
            with self.pointcloud_lock:
                self.latest_pointcloud = msg
        except Exception as e:
            self.get_logger().error(f"❌ LiDAR callback error: {e}")

    def start_calibration_callback(self, request, response):
        """Start calibration process"""
        try:
            self.calibration_active = True
            self.calibration_data = []
            
            # Start visualization thread
            self.viz_thread = threading.Thread(target=self.visualization_loop, daemon=True)
            self.viz_thread.start()
            
            response.success = True
            response.message = "Calibration started! Move calibration target and call capture_calibration_sample service."
            self.get_logger().info("✅ Calibration started!")
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to start calibration: {e}"
            self.get_logger().error(f"❌ Start calibration failed: {e}")
        
        return response

    def capture_sample_callback(self, request, response):
        """Capture calibration sample"""
        try:
            if not self.calibration_active:
                response.success = False
                response.message = "Calibration not active. Call start_calibration first."
                return response
            
            # Get current data
            with self.image_lock:
                if self.latest_image is None:
                    response.success = False
                    response.message = "No camera image available"
                    return response
                image = self.latest_image.copy()
            
            with self.pointcloud_lock:
                if self.latest_pointcloud is None:
                    response.success = False
                    response.message = "No LiDAR data available"
                    return response
                pointcloud = self.latest_pointcloud
            
            # Detect calibration target
            target_detected, image_points, target_corners_3d = self.detect_calibration_target(image)
            
            if not target_detected:
                response.success = False
                response.message = "Calibration target not detected in image"
                return response
            
            # Extract corresponding LiDAR points
            lidar_points = self.extract_lidar_points(pointcloud)
            target_lidar_points = self.find_target_in_lidar(lidar_points, target_corners_3d)
            
            if target_lidar_points is None:
                response.success = False
                response.message = "Cannot find calibration target in LiDAR data"
                return response
            
            # Store calibration sample
            sample = {
                'image_points': image_points,
                'target_corners_3d': target_corners_3d,
                'lidar_points': target_lidar_points,
                'timestamp': time.time()
            }
            
            self.calibration_data.append(sample)
            
            response.success = True
            response.message = f"Calibration sample {len(self.calibration_data)} captured successfully!"
            self.get_logger().info(f"✅ Calibration sample {len(self.calibration_data)} captured!")
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to capture sample: {e}"
            self.get_logger().error(f"❌ Capture sample failed: {e}")
        
        return response

    def save_calibration_callback(self, request, response):
        """Save calibration results"""
        try:
            if len(self.calibration_data) < 5:
                response.success = False
                response.message = f"Need at least 5 samples, got {len(self.calibration_data)}"
                return response
            
            # Perform calibration optimization
            self.get_logger().info("🔄 Computing optimal calibration parameters...")
            calibration_result = self.compute_calibration()
            
            if calibration_result is None:
                response.success = False
                response.message = "Calibration optimization failed"
                return response
            
            # Save results
            self.save_calibration_results(calibration_result)
            
            self.calibration_active = False
            
            response.success = True
            response.message = f"Calibration completed and saved to {self.output_file}"
            self.get_logger().info(f"✅ Calibration saved to {self.output_file}")
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to save calibration: {e}"
            self.get_logger().error(f"❌ Save calibration failed: {e}")
        
        return response

    def detect_calibration_target(self, image):
        """Detect calibration target (checkerboard) in image"""
        try:
            # Convert to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Checkerboard parameters
            checkerboard_size = (9, 6)  # Internal corners
            square_size = 0.03  # 3cm squares
            
            # Find checkerboard corners
            found, corners = cv2.findChessboardCorners(
                gray, checkerboard_size,
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
            
            if found:
                # Refine corner positions
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                
                # Generate 3D coordinates of checkerboard corners
                objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
                objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
                objp *= square_size
                
                return True, corners.reshape(-1, 2), objp
            
            return False, None, None
            
        except Exception as e:
            self.get_logger().error(f"❌ Target detection failed: {e}")
            return False, None, None

    def extract_lidar_points(self, pointcloud):
        """Extract LiDAR points from point cloud message"""
        try:
            points = []
            for point in pc2.read_points(pointcloud, field_names=("x", "y", "z"), skip_nans=True):
                points.append([point[0], point[1], point[2]])
            return np.array(points)
        except Exception as e:
            self.get_logger().error(f"❌ LiDAR point extraction failed: {e}")
            return np.array([])

    def find_target_in_lidar(self, lidar_points, target_corners_3d):
        """Find calibration target plane in LiDAR points"""
        try:
            if len(lidar_points) == 0:
                return None
            
            # For initial calibration, assume target is approximately 2-5 meters in front
            # Filter points in reasonable range
            front_mask = ((lidar_points[:, 0] > 1.0) & (lidar_points[:, 0] < 6.0) &
                         (np.abs(lidar_points[:, 1]) < 2.0) &
                         (lidar_points[:, 2] > -0.5) & (lidar_points[:, 2] < 2.0))
            
            candidate_points = lidar_points[front_mask]
            
            if len(candidate_points) < 100:
                return None
            
            # Use RANSAC to find dominant plane (calibration target)
            from sklearn.linear_model import RANSACRegressor
            
            # Fit plane: z = ax + by + c
            X = candidate_points[:, :2]  # x, y coordinates
            y = candidate_points[:, 2]   # z coordinates
            
            ransac = RANSACRegressor(random_state=42, residual_threshold=0.02)
            ransac.fit(X, y)
            
            # Get inlier points (points on the plane)
            inlier_mask = ransac.inlier_mask_
            plane_points = candidate_points[inlier_mask]
            
            if len(plane_points) < 50:
                return None
            
            # Return center and bounds of detected plane
            center = np.mean(plane_points, axis=0)
            
            return {
                'center': center,
                'points': plane_points,
                'plane_params': [ransac.estimator_.coef_[0], ransac.estimator_.coef_[1], ransac.estimator_.intercept_]
            }
            
        except Exception as e:
            self.get_logger().error(f"❌ Target finding in LiDAR failed: {e}")
            return None

    def compute_calibration(self):
        """Compute optimal camera-LiDAR calibration parameters"""
        try:
            # Initial guess for extrinsic parameters
            # [rotation_x, rotation_y, rotation_z, translation_x, translation_y, translation_z]
            initial_params = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            
            # Camera intrinsic parameters (should be calibrated separately)
            camera_matrix = np.array([
                [1920.0, 0.0, 960.0],
                [0.0, 1920.0, 540.0],
                [0.0, 0.0, 1.0]
            ], dtype=np.float32)
            
            dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
            
            # Optimization function
            def calibration_error(params):
                total_error = 0.0
                
                for sample in self.calibration_data:
                    # Get transformation matrix from parameters
                    R = self.rodrigues_to_rotation_matrix(params[:3])
                    t = params[3:6].reshape(3, 1)
                    
                    # Transform target center to camera coordinates
                    if sample['lidar_points'] is not None:
                        lidar_center = sample['lidar_points']['center'].reshape(3, 1)
                        camera_coords = R @ lidar_center + t
                        
                        # Project to image
                        if camera_coords[2, 0] > 0:  # In front of camera
                            image_proj = camera_matrix @ camera_coords
                            image_proj = image_proj[:2, 0] / image_proj[2, 0]
                            
                            # Compare with detected image center
                            detected_center = np.mean(sample['image_points'], axis=0)
                            error = np.linalg.norm(image_proj - detected_center)
                            total_error += error
                        else:
                            total_error += 1000.0  # Penalty for behind camera
                
                return total_error
            
            # Optimize
            result = minimize(calibration_error, initial_params, method='Nelder-Mead',
                            options={'maxiter': 1000, 'disp': True})
            
            if result.success:
                # Convert optimized parameters to matrices
                R = self.rodrigues_to_rotation_matrix(result.x[:3])
                t = result.x[3:6]
                
                calibration_result = {
                    'rotation_matrix': R.tolist(),
                    'translation_vector': t.tolist(),
                    'camera_matrix': camera_matrix.tolist(),
                    'distortion_coefficients': dist_coeffs.tolist(),
                    'optimization_error': result.fun,
                    'samples_used': len(self.calibration_data),
                    'timestamp': time.time()
                }
                
                return calibration_result
            else:
                self.get_logger().error("❌ Calibration optimization failed")
                return None
                
        except Exception as e:
            self.get_logger().error(f"❌ Calibration computation failed: {e}")
            return None

    def rodrigues_to_rotation_matrix(self, rvec):
        """Convert Rodrigues vector to rotation matrix"""
        theta = np.linalg.norm(rvec)
        if theta == 0:
            return np.eye(3)
        
        k = rvec / theta
        K = np.array([
            [0, -k[2], k[1]],
            [k[2], 0, -k[0]],
            [-k[1], k[0], 0]
        ])
        
        R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)
        return R

    def save_calibration_results(self, calibration_result):
        """Save calibration results to YAML file"""
        try:
            # Ensure output directory exists
            os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
            
            with open(self.output_file, 'w') as f:
                yaml.dump(calibration_result, f, default_flow_style=False)
            
            self.get_logger().info(f"✅ Calibration results saved to {self.output_file}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to save calibration: {e}")

    def visualization_loop(self):
        """Visualization loop for calibration process"""
        while self.calibration_active:
            try:
                with self.image_lock:
                    if self.latest_image is None:
                        time.sleep(0.1)
                        continue
                    image = self.latest_image.copy()
                
                # Detect calibration target
                target_detected, corners, _ = self.detect_calibration_target(image)
                
                # Create visualization
                viz_image = image.copy()
                
                if target_detected:
                    # Draw detected corners
                    cv2.drawChessboardCorners(viz_image, (9, 6), corners, target_detected)
                    
                    # Status text
                    status_text = f"TARGET DETECTED | Samples: {len(self.calibration_data)}"
                    cv2.putText(viz_image, status_text, (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                else:
                    status_text = f"SEARCHING FOR TARGET | Samples: {len(self.calibration_data)}"
                    cv2.putText(viz_image, status_text, (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                
                # Instructions
                instructions = [
                    "1. Position checkerboard in view",
                    "2. Call 'capture_calibration_sample' service",
                    "3. Move to different positions",
                    "4. Capture at least 5 samples",
                    "5. Call 'save_calibration' service"
                ]
                
                for i, instruction in enumerate(instructions):
                    cv2.putText(viz_image, instruction, (10, 70 + i * 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # Publish visualization
                viz_msg = self.bridge.cv2_to_imgmsg(viz_image, 'bgr8')
                self.calibration_viz_pub.publish(viz_msg)
                
                time.sleep(0.1)  # 10 FPS
                
            except Exception as e:
                self.get_logger().error(f"❌ Visualization error: {e}")
                time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = AutoCalibrationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Shutting down AUTO CALIBRATION...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
