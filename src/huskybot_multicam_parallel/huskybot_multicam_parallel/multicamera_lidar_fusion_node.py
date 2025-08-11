#!/usr/bin/env python3
"""
Multi-Camera LiDAR Fusion Node for ULTRA ACCURATE 360° Sensor Fusion
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
from ultralytics import YOLO
import sensor_msgs_py.point_cloud2 as pc2
from collections import defaultdict
import json
from .opencv_cuda_accelerator import OpenCVCudaAccelerator

class MultiCameraLiDARFusionNode(Node):
    def __init__(self):
        super().__init__('multicamera_lidar_fusion_node')
        
        self.bridge = CvBridge()
        
        # ✅ ULTRA ACCURATE Camera configurations with precise angles
        self.camera_configs = [
            {
                'name': 'camera_front',
                'topic': '/camera_front/image_raw',
                'real_name': 'REAR CAMERA',
                'angle': 180,  # Looking backward
                'position': (0, 0),
                'fov_h': 120,  # Horizontal FOV
                'fov_v': 90    # Vertical FOV
            },
            {
                'name': 'camera_front_left',
                'topic': '/camera_front_left/image_raw',
                'real_name': 'LEFT REAR CAMERA',
                'angle': 240,  # Looking left-backward
                'position': (0, 1),
                'fov_h': 120,
                'fov_v': 90
            },
            {
                'name': 'camera_left',
                'topic': '/camera_left/image_raw',
                'real_name': 'LEFT FRONT CAMERA',
                'angle': 300,  # Looking left-forward
                'position': (0, 2),
                'fov_h': 120,
                'fov_v': 90
            },
            {
                'name': 'camera_rear',
                'topic': '/camera_rear/image_raw',
                'real_name': 'FRONT CAMERA',
                'angle': 0,    # Looking forward
                'position': (1, 0),
                'fov_h': 120,
                'fov_v': 90
            },
            {
                'name': 'camera_rear_right',
                'topic': '/camera_rear_right/image_raw',
                'real_name': 'RIGHT FRONT CAMERA',
                'angle': 60,   # Looking right-forward
                'position': (1, 1),
                'fov_h': 120,
                'fov_v': 90
            },
            {
                'name': 'camera_right',
                'topic': '/camera_right/image_raw',
                'real_name': 'RIGHT REAR CAMERA',
                'angle': 120,  # Looking right-backward
                'position': (1, 2),
                'fov_h': 120,
                'fov_v': 90
            }
        ]
        
        # ✅ Global data storage with thread safety
        self.latest_images = {}
        self.latest_pointcloud = None
        self.image_locks = {}
        self.pointcloud_lock = threading.Lock()
        self.global_detections = []
        self.global_lock = threading.Lock()
        
        # ✅ Setup for each camera
        for config in self.camera_configs:
            self.latest_images[config['name']] = None
            self.image_locks[config['name']] = threading.Lock()
        
        # ✅ Setup YOLO
        self.setup_yolo()
        
        # ✅ Setup CUDA Accelerator
        self.cuda_accelerator = OpenCVCudaAccelerator(use_cuda=True)
        self.get_logger().info(f"🚀 CUDA Accelerator initialized for multi-camera fusion")
        
        # ✅ Setup calibration matrices for each camera
        self.setup_all_calibrations()
        
        # ✅ Setup connections
        self.setup_connections()
        
        # ✅ Setup fusion processing
        self.setup_global_fusion()
        
        self.get_logger().info("🚀 MULTI-CAMERA LIDAR FUSION NODE STARTED!")

    def setup_yolo(self):
        """Setup YOLO model for object detection"""
        try:
            import torch
            import os
            
            # Check CUDA availability
            self.use_cuda = torch.cuda.is_available()
            device_info = "GPU (CUDA)" if self.use_cuda else "CPU"
            self.get_logger().info(f"🔧 Device: {device_info}")
            
            model_paths = [
                "/home/kmp-orin/jezzy/huskybot_v11/yolo11m-seg.pt" if not self.use_cuda else "/home/kmp-orin/jezzy/huskybot_v11/yolo11m-seg.engine",
                "/home/kmp-orin/jezzy/huskybot_v11/yolo11m-seg.pt",
                "yolo11m-seg.pt",
                "yolo11n-seg.pt"
            ]
            
            self.yolo_model = None
            for model_path in model_paths:
                try:
                    if os.path.exists(model_path) or not model_path.startswith('/'):
                        self.get_logger().info(f"🔄 Loading model: {model_path}")
                        self.yolo_model = YOLO(model_path)
                        
                        # Test model
                        test_img = np.zeros((640, 640, 3), dtype=np.uint8)
                        results = self.yolo_model.predict(test_img, verbose=False, task='segment', device='cpu' if not self.use_cuda else 0)
                        
                        self.get_logger().info(f"✅ SUCCESS! Model loaded: {model_path}")
                        break
                except Exception as e:
                    self.get_logger().warning(f"⚠️ Failed to load {model_path}: {e}")
                    continue
            
            if self.yolo_model is None:
                raise Exception("No YOLO model could be loaded")
                
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load YOLO model: {e}")
            self.yolo_model = None

    def setup_all_calibrations(self):
        """Setup calibration matrices for all cameras"""
        self.calibrations = {}
        
        # ✅ Standard camera intrinsic parameters (same for all cameras)
        self.camera_matrix = np.array([
            [1920.0, 0.0, 960.0],
            [0.0, 1920.0, 540.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        
        for i, config in enumerate(self.camera_configs):
            camera_angle_rad = np.radians(config['angle'])
            cos_a = np.cos(camera_angle_rad)
            sin_a = np.sin(camera_angle_rad)
            
            # ✅ ULTRA PRECISE transformation matrix for each camera
            R_lidar_to_camera = np.array([
                [cos_a, -sin_a, 0.0],
                [sin_a, cos_a, 0.0],
                [0.0, 0.0, 1.0]
            ], dtype=np.float32)
            
            camera_radius = 0.3  # 30cm from center
            t_lidar_to_camera = np.array([
                camera_radius * cos_a,
                camera_radius * sin_a,
                0.0
            ], dtype=np.float32).reshape(3, 1)
            
            T_lidar_to_camera = np.hstack([R_lidar_to_camera, t_lidar_to_camera])
            
            self.calibrations[config['name']] = {
                'T_matrix': T_lidar_to_camera,
                'angle': config['angle'],
                'fov_h': config['fov_h'],
                'fov_v': config['fov_v']
            }
        
        self.get_logger().info("📐 All camera calibrations setup completed!")

    def setup_connections(self):
        """Setup all ROS2 connections"""
        try:
            # ✅ Camera subscribers
            self.camera_subs = {}
            for config in self.camera_configs:
                self.camera_subs[config['name']] = self.create_subscription(
                    Image, config['topic'],
                    lambda msg, name=config['name']: self.camera_callback(msg, name),
                    10
                )
            
            # ✅ LiDAR subscriber
            self.lidar_sub = self.create_subscription(
                PointCloud2, '/velodyne_points', self.lidar_callback, 10)
            
            # ✅ Publishers for each camera's fused result
            self.fused_pubs = {}
            for config in self.camera_configs:
                self.fused_pubs[config['name']] = self.create_publisher(
                    Image, f"/{config['name']}_fused", 10)
            
            # ✅ Global fusion result publisher
            self.global_fusion_pub = self.create_publisher(
                Image, '/global_fusion_result', 10)
            
            self.get_logger().info(f"📡 Setup complete: {len(self.camera_configs)} cameras + LiDAR")
            
        except Exception as e:
            self.get_logger().error(f"❌ Connection setup failed: {e}")

    def camera_callback(self, msg, camera_name):
        """Camera callback for each camera"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.image_locks[camera_name]:
                self.latest_images[camera_name] = cv_image.copy()
        except Exception as e:
            self.get_logger().error(f"❌ Camera callback error for {camera_name}: {e}")

    def lidar_callback(self, msg):
        """LiDAR point cloud callback"""
        try:
            with self.pointcloud_lock:
                self.latest_pointcloud = msg
        except Exception as e:
            self.get_logger().error(f"❌ LiDAR callback error: {e}")

    def setup_global_fusion(self):
        """Setup global fusion processing"""
        self.fusion_active = True
        self.global_fusion_thread = threading.Thread(target=self.global_fusion_loop, daemon=True)
        self.global_fusion_thread.start()
        self.get_logger().info("✅ Global fusion processing started!")

    def project_lidar_to_camera(self, points_3d, camera_name):
        """Project LiDAR points to specific camera with ULTRA ACCURACY"""
        if len(points_3d) == 0 or camera_name not in self.calibrations:
            return np.array([]), np.array([])
        
        calibration = self.calibrations[camera_name]
        T_matrix = calibration['T_matrix']
        
        # Transform to camera coordinate system
        points_3d_homogeneous = np.hstack([points_3d, np.ones((points_3d.shape[0], 1))])
        points_camera = points_3d_homogeneous @ T_matrix.T
        
        # Filter points in front of camera
        valid_mask = points_camera[:, 2] > 0.1
        points_camera_valid = points_camera[valid_mask]
        
        if len(points_camera_valid) == 0:
            return np.array([]), np.array([])
        
        # Project to image plane
        points_2d_homogeneous = points_camera_valid @ self.camera_matrix.T
        points_2d = points_2d_homogeneous[:, :2] / points_2d_homogeneous[:, 2:3]
        
        # Filter within image bounds
        image_mask = ((points_2d[:, 0] >= 0) & (points_2d[:, 0] < 1920) &
                     (points_2d[:, 1] >= 0) & (points_2d[:, 1] < 1080))
        
        return points_2d[image_mask], points_camera_valid[image_mask, 2]

    def get_lidar_distance_in_bbox(self, bbox, points_2d, depths):
        """Get ULTRA ACCURATE distance from LiDAR within bounding box"""
        if len(points_2d) == 0:
            return None, 0
        
        x1, y1, x2, y2 = bbox
        
        # Find points within bounding box
        in_bbox_mask = ((points_2d[:, 0] >= x1) & (points_2d[:, 0] <= x2) &
                       (points_2d[:, 1] >= y1) & (points_2d[:, 1] <= y2))
        
        bbox_depths = depths[in_bbox_mask]
        
        if len(bbox_depths) == 0:
            return None, 0
        
        # Use median for robustness
        accurate_distance = float(np.median(bbox_depths))
        return accurate_distance, len(bbox_depths)

    def calculate_global_coordinates(self, distance, center_x, center_y, camera_angle, image_width, image_height, fov_h, fov_v):
        """Calculate ULTRA ACCURATE global 3D coordinates"""
        # Horizontal angle calculation
        angle_offset = ((center_x / image_width) - 0.5) * fov_h
        global_angle = (camera_angle + angle_offset) % 360
        
        # Vertical angle calculation
        vertical_offset = ((center_y / image_height) - 0.5) * fov_v
        
        # Global coordinates
        coord_x = distance * np.cos(np.radians(global_angle))
        coord_y = distance * np.sin(np.radians(global_angle))
        coord_z = 1.5 + distance * np.tan(np.radians(vertical_offset))  # 1.5m base height
        
        return coord_x, coord_y, coord_z, global_angle

    def process_camera_detections(self, camera_name, image, points_3d):
        """Process detections for one camera with ULTRA ACCURATE fusion"""
        if self.yolo_model is None:
            return []
        
        config = next(c for c in self.camera_configs if c['name'] == camera_name)
        
        # YOLO detection
        processing_height = 640
        height, width = image.shape[:2]
        if height > processing_height:
            scale = processing_height / height
            new_width = int(width * scale)
            new_height = int(height * scale)
            image_resized = cv2.resize(image, (new_width, new_height))
        else:
            image_resized = image
            scale = 1.0
        
        results = self.yolo_model.predict(
            image_resized, conf=0.25, iou=0.45, verbose=False, task='segment', device='cpu' if not self.use_cuda else 0)
        
        if not results or len(results) == 0:
            return []
        
        # Project LiDAR points to this camera
        points_2d, depths = self.project_lidar_to_camera(points_3d, camera_name)
        
        detections = []
        result = results[0]
        
        if hasattr(result, 'boxes') and result.boxes is not None:
            boxes = result.boxes.xyxy.cpu().numpy()
            scores = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy()
            names = result.names if hasattr(result, 'names') else {}
            
            for i, (box, score, cls_id) in enumerate(zip(boxes, scores, classes)):
                # Scale back to original image
                x1 = int(box[0] / scale)
                y1 = int(box[1] / scale)
                x2 = int(box[2] / scale)
                y2 = int(box[3] / scale)
                
                # Constrain to image bounds
                x1 = max(0, min(width, x1))
                y1 = max(0, min(height, y1))
                x2 = max(0, min(width, x2))
                y2 = max(0, min(height, y2))
                
                class_name = names.get(int(cls_id), f"class_{int(cls_id)}")
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                # ✅ ULTRA ACCURATE LiDAR-based distance
                lidar_distance, point_count = self.get_lidar_distance_in_bbox(
                    (x1, y1, x2, y2), points_2d, depths)
                
                if lidar_distance is not None:
                    distance = lidar_distance
                    accuracy_source = f"LIDAR_FUSED_{point_count}_pts"
                else:
                    # Fallback to visual estimation
                    bbox_area = (x2 - x1) * (y2 - y1)
                    distance = self.calculate_visual_distance(class_name, bbox_area, width, height)
                    accuracy_source = "VISUAL_ESTIMATION"
                
                # ✅ ULTRA ACCURATE global coordinates
                coord_x, coord_y, coord_z, global_angle = self.calculate_global_coordinates(
                    distance, center_x, center_y, config['angle'], width, height,
                    config['fov_h'], config['fov_v'])
                
                coord_z = max(0.0, min(3.0, coord_z))  # Reasonable height limits
                
                detection = {
                    'camera_name': camera_name,
                    'camera_real_name': config['real_name'],
                    'class': class_name,
                    'confidence': float(score),
                    'bbox': (x1, y1, x2, y2),
                    'distance': distance,
                    'global_x': coord_x,
                    'global_y': coord_y,
                    'global_z': coord_z,
                    'global_angle': global_angle,
                    'accuracy_source': accuracy_source,
                    'color': self.get_distinct_color(int(cls_id)),
                    'timestamp': time.time()
                }
                
                detections.append(detection)
                
                # ✅ ULTRA ACCURATE logging
                log_msg = (
                    f"🎯 FUSION360° {config['real_name']} | "
                    f"Class: {class_name} | "
                    f"Conf: {score:.2f} | "
                    f"Dist: {distance:.2f}m | "
                    f"Global: ({coord_x:.2f}, {coord_y:.2f}, {coord_z:.2f}) | "
                    f"Angle: {global_angle:.1f}° | "
                    f"Source: {accuracy_source}"
                )
                self.get_logger().info(log_msg)
        
        return detections

    def calculate_visual_distance(self, class_name, bbox_area, frame_width, frame_height):
        """Visual distance estimation fallback"""
        object_sizes = {
            'person': 1.7, 'bicycle': 1.8, 'car': 4.5, 'motorcycle': 2.0,
            'bus': 12.0, 'truck': 8.0, 'boat': 6.0, 'traffic light': 1.0,
            'stop sign': 0.6, 'bench': 1.5, 'cat': 0.5, 'dog': 0.6
        }
        
        real_size = object_sizes.get(class_name, 1.0)
        frame_area = frame_width * frame_height
        relative_size = bbox_area / frame_area
        
        if relative_size > 0:
            focal_length = 900
            distance = (real_size * focal_length) / np.sqrt(bbox_area)
            return max(0.5, min(50.0, distance))
        return 5.0

    def merge_global_detections(self, all_detections):
        """Merge detections from all cameras to eliminate duplicates"""
        if len(all_detections) == 0:
            return []
        
        # Group detections by position proximity
        merged_detections = []
        used_indices = set()
        
        for i, det1 in enumerate(all_detections):
            if i in used_indices:
                continue
            
            # Find nearby detections (same object from different cameras)
            similar_detections = [det1]
            used_indices.add(i)
            
            for j, det2 in enumerate(all_detections[i+1:], i+1):
                if j in used_indices:
                    continue
                
                # Check if detections are of same object
                position_distance = np.sqrt(
                    (det1['global_x'] - det2['global_x'])**2 +
                    (det1['global_y'] - det2['global_y'])**2
                )
                
                if (position_distance < 2.0 and  # Within 2 meters
                    det1['class'] == det2['class'] and  # Same class
                    abs(det1['confidence'] - det2['confidence']) < 0.3):  # Similar confidence
                    
                    similar_detections.append(det2)
                    used_indices.add(j)
            
            # Merge similar detections
            if len(similar_detections) > 1:
                # Use detection with highest confidence or LiDAR data
                best_detection = max(similar_detections, 
                                   key=lambda x: (x['accuracy_source'].startswith('LIDAR'), x['confidence']))
                
                # Average positions for better accuracy
                avg_x = np.mean([d['global_x'] for d in similar_detections])
                avg_y = np.mean([d['global_y'] for d in similar_detections])
                avg_z = np.mean([d['global_z'] for d in similar_detections])
                
                best_detection.update({
                    'global_x': avg_x,
                    'global_y': avg_y,
                    'global_z': avg_z,
                    'merged_from': len(similar_detections)
                })
                
                merged_detections.append(best_detection)
            else:
                merged_detections.append(det1)
        
        return merged_detections

    def global_fusion_loop(self):
        """Main global fusion processing loop"""
        while self.fusion_active:
            try:
                # ✅ Check LiDAR data availability
                with self.pointcloud_lock:
                    if self.latest_pointcloud is None:
                        time.sleep(0.1)
                        continue
                    pointcloud = self.latest_pointcloud
                
                # ✅ Extract LiDAR points
                points_3d = []
                for point in pc2.read_points(pointcloud, field_names=("x", "y", "z"), skip_nans=True):
                    points_3d.append([point[0], point[1], point[2]])
                points_3d = np.array(points_3d)
                
                if len(points_3d) == 0:
                    time.sleep(0.1)
                    continue
                
                # ✅ Process each camera
                all_detections = []
                individual_results = {}
                
                for config in self.camera_configs:
                    camera_name = config['name']
                    
                    with self.image_locks[camera_name]:
                        if self.latest_images[camera_name] is None:
                            continue
                        image = self.latest_images[camera_name].copy()
                    
                    # Process this camera
                    camera_detections = self.process_camera_detections(camera_name, image, points_3d)
                    all_detections.extend(camera_detections)
                    
                    # Create individual visualization
                    points_2d, depths = self.project_lidar_to_camera(points_3d, camera_name)
                    individual_vis = self.create_individual_visualization(
                        image, camera_detections, points_2d, depths, config)
                    individual_results[camera_name] = individual_vis
                
                # ✅ Merge global detections
                merged_detections = self.merge_global_detections(all_detections)
                
                # ✅ Update global state
                with self.global_lock:
                    self.global_detections = merged_detections
                
                # ✅ Publish individual camera results
                for camera_name, vis_image in individual_results.items():
                    if camera_name in self.fused_pubs:
                        msg = self.bridge.cv2_to_imgmsg(vis_image, 'bgr8')
                        self.fused_pubs[camera_name].publish(msg)
                
                # ✅ Create and publish global visualization
                global_vis = self.create_global_visualization(merged_detections, points_3d)
                global_msg = self.bridge.cv2_to_imgmsg(global_vis, 'bgr8')
                self.global_fusion_pub.publish(global_msg)
                
                # ✅ Status logging
                lidar_points = len(points_3d)
                total_detections = len(all_detections)
                merged_count = len(merged_detections)
                
                status_msg = (
                    f"🌐 GLOBAL FUSION STATUS | "
                    f"LiDAR Points: {lidar_points} | "
                    f"Raw Detections: {total_detections} | "
                    f"Merged Objects: {merged_count} | "
                    f"Active Cameras: {len(individual_results)}"
                )
                self.get_logger().info(status_msg)
                
                time.sleep(0.05)  # 20 FPS
                
            except Exception as e:
                self.get_logger().error(f"❌ Global fusion error: {e}")
                time.sleep(0.1)

    def create_individual_visualization(self, image, detections, points_2d, depths, config):
        """Create visualization for individual camera"""
        vis_image = image.copy()
        
        # Draw LiDAR points
        if len(points_2d) > 0:
            for point, depth in zip(points_2d, depths):
                x, y = int(point[0]), int(point[1])
                if 0 <= x < vis_image.shape[1] and 0 <= y < vis_image.shape[0]:
                    color_intensity = max(0, min(255, int(255 * (1.0 - depth / 20.0))))
                    cv2.circle(vis_image, (x, y), 1, (0, color_intensity, 255 - color_intensity), -1)
        
        # Draw detections
        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            color = detection['color']
            
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), color, 3)
            
            info_lines = [
                f"{detection['class']} ({detection['confidence']:.2f})",
                f"Dist: {detection['distance']:.2f}m",
                f"Global: ({detection['global_x']:.1f}, {detection['global_y']:.1f}, {detection['global_z']:.1f})",
                f"Angle: {detection['global_angle']:.0f}°",
                f"Src: {detection['accuracy_source']}"
            ]
            
            # Draw info background and text
            font_scale = 0.7
            font_thickness = 2
            line_height = 25
            
            max_width = max([cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)[0][0] 
                           for line in info_lines])
            
            info_height = len(info_lines) * line_height + 10
            
            # Create overlay for transparent background
            overlay = vis_image.copy()
            cv2.rectangle(overlay, (x1, y1 - info_height - 5), 
                         (x1 + max_width + 20, y1), (0, 0, 0), -1)
            
            # Apply transparency (alpha blending) - 🚀 CUDA-accelerated
            alpha = 0.7  # 70% opacity
            try:
                vis_image = self.cuda_accelerator.add_weighted(overlay, alpha, vis_image, 1 - alpha, 0)
            except:
                cv2.addWeighted(overlay, alpha, vis_image, 1 - alpha, 0, vis_image)
            
            # Draw border on top (stays opaque)
            cv2.rectangle(vis_image, (x1, y1 - info_height - 5), 
                         (x1 + max_width + 20, y1), color, 2)
            
            for i, line in enumerate(info_lines):
                text_y = y1 - info_height + (i + 1) * line_height
                cv2.putText(vis_image, line, (x1 + 10, text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), font_thickness)
        
        # Camera status
        status = f"FUSION: {config['real_name']} | LiDAR: {len(points_2d)} pts | Objects: {len(detections)}"
        cv2.putText(vis_image, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 2)
        
        return vis_image

    def create_global_visualization(self, detections, points_3d):
        """Create global bird's-eye view visualization"""
        # Create bird's-eye view canvas
        canvas_size = 800
        canvas = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8)
        
        # Scale: 20 meters = canvas_size pixels
        scale = canvas_size / 40.0  # 40m x 40m view
        center = canvas_size // 2
        
        # Draw coordinate grid
        for i in range(-20, 21, 5):
            y_line = int(center + i * scale)
            x_line = int(center + i * scale)
            cv2.line(canvas, (0, y_line), (canvas_size, y_line), (50, 50, 50), 1)
            cv2.line(canvas, (x_line, 0), (x_line, canvas_size), (50, 50, 50), 1)
        
        # Draw axes
        cv2.line(canvas, (center, 0), (center, canvas_size), (100, 100, 100), 2)
        cv2.line(canvas, (0, center), (canvas_size, center), (100, 100, 100), 2)
        
        # Draw robot position
        cv2.circle(canvas, (center, center), 10, (255, 255, 255), -1)
        cv2.putText(canvas, "ROBOT", (center-30, center-20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        
        # Draw LiDAR points
        if len(points_3d) > 0:
            for point in points_3d[::10]:  # Sample every 10th point
                x, y = point[0], point[1]
                if abs(x) < 20 and abs(y) < 20:  # Within view range
                    px = int(center + x * scale)
                    py = int(center - y * scale)  # Flip Y for proper orientation
                    if 0 <= px < canvas_size and 0 <= py < canvas_size:
                        cv2.circle(canvas, (px, py), 1, (100, 100, 100), -1)
        
        # Draw detections
        for detection in detections:
            x, y = detection['global_x'], detection['global_y']
            if abs(x) < 20 and abs(y) < 20:
                px = int(center + x * scale)
                py = int(center - y * scale)
                
                color = detection['color']
                cv2.circle(canvas, (px, py), 8, color, -1)
                cv2.circle(canvas, (px, py), 12, color, 2)
                
                # Object info
                info_text = f"{detection['class']} ({detection['distance']:.1f}m)"
                cv2.putText(canvas, info_text, (px + 15, py), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Global status
        status_lines = [
            "GLOBAL LIDAR-CAMERA FUSION",
            f"Objects: {len(detections)}",
            f"LiDAR Points: {len(points_3d)}",
            "Scale: 40m x 40m"
        ]
        
        for i, line in enumerate(status_lines):
            cv2.putText(canvas, line, (10, 30 + i * 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
        
        return canvas

    def get_distinct_color(self, class_id):
        """Get distinct bright color for each class"""
        colors = [
            (255, 100, 100), (100, 255, 100), (100, 100, 255), (255, 255, 100),
            (255, 100, 255), (100, 255, 255), (200, 100, 200), (255, 200, 100),
            (100, 200, 100), (200, 200, 100), (100, 100, 200), (200, 100, 100)
        ]
        return colors[class_id % len(colors)]

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = MultiCameraLiDARFusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Shutting down MULTI-CAMERA LIDAR FUSION...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if node:
            node.fusion_active = False
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
