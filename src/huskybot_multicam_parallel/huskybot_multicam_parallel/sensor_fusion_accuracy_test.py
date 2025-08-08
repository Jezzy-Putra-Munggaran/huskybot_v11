#!/usr/bin/env python3
"""
ULTRA ACCURATE SENSOR FUSION ACCURACY TEST
LiDAR-Camera Fusion Precision Validation
Author: Jezzy Putra Munggaran
Date: August 5, 2025
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
import numpy as np
import cv2
from cv_bridge import CvBridge
import json
import time
import math
from scipy.spatial.distance import euclidean
import threading

class SensorFusionAccuracyTest(Node):
    def __init__(self):
        super().__init__('sensor_fusion_accuracy_test')
        
        self.get_logger().info("🎯 ULTRA ACCURATE SENSOR FUSION ACCURACY TEST")
        self.get_logger().info("=" * 60)
        
        # ✅ Initialize components
        self.bridge = CvBridge()
        self.test_results = []
        self.test_start_time = time.time()
        
        # ✅ Test parameters
        self.expected_accuracy = 0.004  # 4mm (VLP32C resolution)
        self.test_duration = 300  # 5 minutes
        self.sample_count = 0
        self.accuracy_samples = []
        
        # ✅ Ground truth reference points (manually measured)
        self.ground_truth_points = [
            {"name": "Front Wall", "distance": 5.0, "x": 5.0, "y": 0.0, "z": 0.0},
            {"name": "Left Wall", "distance": 3.0, "x": 0.0, "y": 3.0, "z": 0.0},
            {"name": "Right Wall", "distance": 3.0, "x": 0.0, "y": -3.0, "z": 0.0},
            {"name": "Corner Object", "distance": 4.24, "x": 3.0, "y": 3.0, "z": 0.5},
        ]
        
        # ✅ Subscribers
        self.global_fusion_sub = self.create_subscription(
            String,
            '/global_fusion_result',
            self.fusion_result_callback,
            10
        )
        
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.lidar_callback,
            10
        )
        
        # ✅ Publishers
        self.test_result_pub = self.create_publisher(
            String,
            '/sensor_fusion_test_results',
            10
        )
        
        # ✅ Timer for periodic testing
        self.test_timer = self.create_timer(10.0, self.periodic_accuracy_test)
        
        # ✅ Final test timer
        self.final_timer = self.create_timer(self.test_duration, self.final_test_report)
        
        self.get_logger().info("✅ Accuracy test initialized")
        self.get_logger().info(f"📊 Test duration: {self.test_duration} seconds")
        self.get_logger().info(f"🎯 Expected accuracy: ±{self.expected_accuracy}m")
        
    def fusion_result_callback(self, msg):
        """Process fusion results for accuracy testing"""
        try:
            fusion_data = json.loads(msg.data)
            self.analyze_fusion_accuracy(fusion_data)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error processing fusion result: {e}")
    
    def lidar_callback(self, msg):
        """Process raw LiDAR data for comparison"""
        self.sample_count += 1
        
        if self.sample_count % 100 == 0:
            self.get_logger().info(f"📡 LiDAR samples processed: {self.sample_count}")
    
    def analyze_fusion_accuracy(self, fusion_data):
        """Analyze accuracy of fusion results against ground truth"""
        try:
            if 'detections' not in fusion_data:
                return
                
            for detection in fusion_data['detections']:
                # Extract 3D coordinates
                x = detection.get('x_3d', 0.0)
                y = detection.get('y_3d', 0.0) 
                z = detection.get('z_3d', 0.0)
                distance = detection.get('distance', 0.0)
                
                # Find closest ground truth point
                closest_gt = self.find_closest_ground_truth(x, y, z)
                
                if closest_gt:
                    # Calculate accuracy metrics
                    distance_error = abs(distance - closest_gt['distance'])
                    position_error = euclidean([x, y, z], 
                                             [closest_gt['x'], closest_gt['y'], closest_gt['z']])
                    
                    accuracy_sample = {
                        'timestamp': time.time(),
                        'ground_truth': closest_gt,
                        'measured': {'x': x, 'y': y, 'z': z, 'distance': distance},
                        'distance_error': distance_error,
                        'position_error': position_error,
                        'accuracy_percentage': max(0, 100 - (distance_error / closest_gt['distance'] * 100))
                    }
                    
                    self.accuracy_samples.append(accuracy_sample)
                    
                    # Log if high accuracy achieved
                    if distance_error < self.expected_accuracy:
                        self.get_logger().info(
                            f"🎯 HIGH ACCURACY: {closest_gt['name']} - "
                            f"Error: {distance_error:.4f}m (Target: ±{self.expected_accuracy}m)"
                        )
                    
        except Exception as e:
            self.get_logger().error(f"❌ Error analyzing fusion accuracy: {e}")
    
    def find_closest_ground_truth(self, x, y, z):
        """Find the closest ground truth point to measured coordinates"""
        min_distance = float('inf')
        closest_point = None
        
        for gt_point in self.ground_truth_points:
            distance = euclidean([x, y, z], [gt_point['x'], gt_point['y'], gt_point['z']])
            
            if distance < min_distance and distance < 1.0:  # Within 1m tolerance
                min_distance = distance
                closest_point = gt_point
                
        return closest_point
    
    def periodic_accuracy_test(self):
        """Periodic accuracy assessment"""
        if not self.accuracy_samples:
            self.get_logger().info("⏳ Waiting for fusion data...")
            return
            
        # Calculate current statistics
        recent_samples = self.accuracy_samples[-50:]  # Last 50 samples
        
        if len(recent_samples) < 5:
            return
            
        distance_errors = [s['distance_error'] for s in recent_samples]
        position_errors = [s['position_error'] for s in recent_samples]
        accuracy_percentages = [s['accuracy_percentage'] for s in recent_samples]
        
        # Statistics
        avg_distance_error = np.mean(distance_errors)
        max_distance_error = np.max(distance_errors)
        min_distance_error = np.min(distance_errors)
        std_distance_error = np.std(distance_errors)
        
        avg_position_error = np.mean(position_errors)
        avg_accuracy = np.mean(accuracy_percentages)
        
        # Ultra accurate threshold check
        ultra_accurate_count = sum(1 for e in distance_errors if e < self.expected_accuracy)
        ultra_accurate_percentage = (ultra_accurate_count / len(distance_errors)) * 100
        
        self.get_logger().info("📊 PERIODIC ACCURACY REPORT")
        self.get_logger().info(f"   📏 Avg Distance Error: {avg_distance_error:.4f}m")
        self.get_logger().info(f"   📏 Max Distance Error: {max_distance_error:.4f}m")
        self.get_logger().info(f"   📏 Min Distance Error: {min_distance_error:.4f}m")
        self.get_logger().info(f"   📏 Std Distance Error: {std_distance_error:.4f}m")
        self.get_logger().info(f"   📍 Avg Position Error: {avg_position_error:.4f}m")
        self.get_logger().info(f"   🎯 Average Accuracy: {avg_accuracy:.1f}%")
        self.get_logger().info(f"   🔥 Ultra Accurate Samples: {ultra_accurate_percentage:.1f}%")
        
        # Publish results
        test_result = {
            'timestamp': time.time(),
            'samples_count': len(recent_samples),
            'avg_distance_error': avg_distance_error,
            'max_distance_error': max_distance_error,
            'min_distance_error': min_distance_error,
            'std_distance_error': std_distance_error,
            'avg_position_error': avg_position_error,
            'avg_accuracy_percentage': avg_accuracy,
            'ultra_accurate_percentage': ultra_accurate_percentage,
            'expected_accuracy': self.expected_accuracy
        }
        
        self.test_result_pub.publish(String(data=json.dumps(test_result)))
    
    def final_test_report(self):
        """Generate final comprehensive test report"""
        self.get_logger().info("🏁 GENERATING FINAL TEST REPORT...")
        
        if not self.accuracy_samples:
            self.get_logger().error("❌ No accuracy samples collected!")
            return
            
        # Calculate comprehensive statistics
        distance_errors = [s['distance_error'] for s in self.accuracy_samples]
        position_errors = [s['position_error'] for s in self.accuracy_samples]
        accuracy_percentages = [s['accuracy_percentage'] for s in self.accuracy_samples]
        
        # Overall statistics
        total_samples = len(self.accuracy_samples)
        avg_distance_error = np.mean(distance_errors)
        median_distance_error = np.median(distance_errors)
        max_distance_error = np.max(distance_errors)
        min_distance_error = np.min(distance_errors)
        std_distance_error = np.std(distance_errors)
        
        avg_position_error = np.mean(position_errors)
        avg_accuracy = np.mean(accuracy_percentages)
        
        # Ultra accurate performance
        ultra_accurate_count = sum(1 for e in distance_errors if e < self.expected_accuracy)
        ultra_accurate_percentage = (ultra_accurate_count / total_samples) * 100
        
        # Performance by ground truth points
        gt_performance = {}
        for gt_point in self.ground_truth_points:
            gt_samples = [s for s in self.accuracy_samples if s['ground_truth']['name'] == gt_point['name']]
            if gt_samples:
                gt_errors = [s['distance_error'] for s in gt_samples]
                gt_performance[gt_point['name']] = {
                    'sample_count': len(gt_samples),
                    'avg_error': np.mean(gt_errors),
                    'max_error': np.max(gt_errors),
                    'min_error': np.min(gt_errors)
                }
        
        # Final report
        self.get_logger().info("=" * 80)
        self.get_logger().info("🎯 ULTRA ACCURATE SENSOR FUSION - FINAL TEST REPORT")
        self.get_logger().info("=" * 80)
        self.get_logger().info(f"🕒 Test Duration: {time.time() - self.test_start_time:.1f} seconds")
        self.get_logger().info(f"📊 Total Samples: {total_samples}")
        self.get_logger().info("")
        self.get_logger().info("📏 DISTANCE ACCURACY METRICS:")
        self.get_logger().info(f"   Average Error: {avg_distance_error:.4f}m")
        self.get_logger().info(f"   Median Error: {median_distance_error:.4f}m")
        self.get_logger().info(f"   Maximum Error: {max_distance_error:.4f}m")
        self.get_logger().info(f"   Minimum Error: {min_distance_error:.4f}m")
        self.get_logger().info(f"   Standard Deviation: {std_distance_error:.4f}m")
        self.get_logger().info("")
        self.get_logger().info("📍 POSITION ACCURACY:")
        self.get_logger().info(f"   Average Position Error: {avg_position_error:.4f}m")
        self.get_logger().info("")
        self.get_logger().info("🎯 PERFORMANCE SUMMARY:")
        self.get_logger().info(f"   Overall Accuracy: {avg_accuracy:.1f}%")
        self.get_logger().info(f"   Ultra Accurate Samples: {ultra_accurate_percentage:.1f}%")
        self.get_logger().info(f"   Target Accuracy (±{self.expected_accuracy}m): {'✅ ACHIEVED' if ultra_accurate_percentage > 80 else '❌ NOT ACHIEVED'}")
        self.get_logger().info("")
        
        # Performance by reference points
        self.get_logger().info("📊 PERFORMANCE BY REFERENCE POINTS:")
        for name, perf in gt_performance.items():
            self.get_logger().info(f"   {name}:")
            self.get_logger().info(f"     Samples: {perf['sample_count']}")
            self.get_logger().info(f"     Avg Error: {perf['avg_error']:.4f}m")
            self.get_logger().info(f"     Max Error: {perf['max_error']:.4f}m")
            self.get_logger().info(f"     Min Error: {perf['min_error']:.4f}m")
        
        self.get_logger().info("")
        self.get_logger().info("=" * 80)
        
        # Final assessment
        if ultra_accurate_percentage > 90:
            self.get_logger().info("🔥 EXCELLENT: Ultra accurate sensor fusion achieved!")
        elif ultra_accurate_percentage > 80:
            self.get_logger().info("✅ GOOD: High accuracy sensor fusion achieved!")
        elif ultra_accurate_percentage > 60:
            self.get_logger().info("⚠️  MODERATE: Decent accuracy, calibration recommended")
        else:
            self.get_logger().info("❌ POOR: Low accuracy, system requires recalibration")
        
        self.get_logger().info("=" * 80)
        
        # Stop the node
        self.get_logger().info("🏁 Test completed. Shutting down...")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    accuracy_test = SensorFusionAccuracyTest()
    
    try:
        rclpy.spin(accuracy_test)
    except KeyboardInterrupt:
        pass
    finally:
        accuracy_test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
