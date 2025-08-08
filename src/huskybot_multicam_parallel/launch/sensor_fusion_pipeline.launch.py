#!/usr/bin/env python3
"""
ULTRA ACCURATE Sensor Fusion Launch File
LiDAR-Camera Fusion for Maximum Accuracy Distance and Coordinate Estimation
Author: Jezzy Putra Munggaran
Date: August 5, 2025
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    # ✅ Launch arguments
    declare_use_fusion = DeclareLaunchArgument(
        'use_fusion', default_value='true',
        description='Use LiDAR-Camera sensor fusion')
    
    declare_use_global_fusion = DeclareLaunchArgument(
        'use_global_fusion', default_value='true',
        description='Use multi-camera global fusion')
    
    declare_calibration_file = DeclareLaunchArgument(
        'calibration_file', default_value='/tmp/camera_lidar_calibration.yaml',
        description='Camera-LiDAR calibration file path')
    
    # ✅ Camera configurations for fusion
    camera_configs = [
        {
            'name': 'camera_front',
            'topic': '/camera_front/image_raw',
            'real_name': 'REAR CAMERA',
            'idx': 0
        },
        {
            'name': 'camera_front_left',
            'topic': '/camera_front_left/image_raw',
            'real_name': 'LEFT REAR CAMERA',
            'idx': 1
        },
        {
            'name': 'camera_left',
            'topic': '/camera_left/image_raw',
            'real_name': 'LEFT FRONT CAMERA',
            'idx': 2
        },
        {
            'name': 'camera_rear',
            'topic': '/camera_rear/image_raw',
            'real_name': 'FRONT CAMERA',
            'idx': 3
        },
        {
            'name': 'camera_rear_right',
            'topic': '/camera_rear_right/image_raw',
            'real_name': 'RIGHT FRONT CAMERA',
            'idx': 4
        },
        {
            'name': 'camera_right',  
            'topic': '/camera_right/image_raw',
            'real_name': 'RIGHT REAR CAMERA',
            'idx': 5
        }
    ]
    
    launch_actions = []
    
    # ✅ 1. ULTRA POWER OPTIMIZATION
    launch_actions.append(
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔧 ULTRA POWER OPTIMIZATION FOR SENSOR FUSION..." && '
                 'sudo nvpmodel -m 0 && '
                 'sudo jetson_clocks --fan && '
                 'sudo sh -c "echo 255 > /sys/class/hwmon/hwmon0/pwm1" && '
                 'echo "✅ GPU/CPU performance maximized for sensor fusion!"'],
            output='screen'
        )
    )
    
    # ✅ 2. Start VLP32C LiDAR with optimized settings
    launch_actions.append(
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🔧 Starting ULTRA ACCURATE VLP32C LiDAR..." && '
                         'ros2 run velodyne_driver velodyne_driver_node --ros-args '
                         '-p model:=VLP32C -p rpm:=600.0 -p port:=2368 -p device_ip:=192.168.1.201 '
                         '-p frame_id:=velodyne &'],
                    output='screen'
                )
            ]
        )
    )
    
    # ✅ 3. Start LiDAR point cloud conversion
    launch_actions.append(
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='velodyne_pointcloud',
                    executable='velodyne_transform_node',
                    name='velodyne_transform',
                    output='screen',
                    parameters=[
                        {'model': 'VLP32C'},
                        {'calibration': '/home/kmp-orin/jezzy/huskybot_v11/src/velodyne/velodyne_pointcloud/params/VeloView-VLP-32C.yaml'},
                        {'min_range': 0.5},
                        {'max_range': 100.0},
                        {'organize_cloud': False}
                    ],
                    remappings=[
                        ('velodyne_packets', '/velodyne_packets'),
                        ('velodyne_points', '/velodyne_points')
                    ]
                )
            ]
        )
    )
    
    # ✅ 4. Start Multi-Camera Global Fusion (Primary System)
    launch_actions.append(
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='huskybot_multicam_parallel',
                    executable='multicamera_lidar_fusion_node',
                    name='multicamera_lidar_fusion',
                    output='screen',
                    parameters=[
                        {'use_sim_time': False}
                    ],
                    remappings=[
                        ('velodyne_points', '/velodyne_points')
                    ]
                )
            ]
        )
    )
    
    # ✅ 5. Individual Camera-LiDAR Fusion Nodes (Backup/Detail)
    base_delay = 10.0
    for i, config in enumerate(camera_configs):
        launch_actions.append(
            TimerAction(
                period=base_delay + i * 1.0,  # Stagger starts
                actions=[
                    Node(
                        package='huskybot_multicam_parallel',
                        executable='lidar_camera_fusion_node',
                        name=f'{config["name"]}_lidar_fusion',
                        output='screen',
                        parameters=[
                            {'camera_name': config['name']},
                            {'camera_topic': config['topic']},
                            {'lidar_topic': '/velodyne_points'},
                            {'camera_real_name': config['real_name']},
                            {'camera_idx': config['idx']},
                            {'use_sim_time': False}
                        ]
                    )
                ]
            )
        )
    
    # ✅ 6. Auto-Calibration Node (On-demand)
    launch_actions.append(
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='huskybot_multicam_parallel',
                    executable='auto_calibration_node',
                    name='auto_calibration',
                    output='screen',
                    parameters=[
                        {'camera_topic': '/camera_front/image_raw'},
                        {'lidar_topic': '/velodyne_points'},
                        {'output_file': LaunchConfiguration('calibration_file')},
                        {'use_sim_time': False}
                    ]
                )
            ]
        )
    )
    
    # ✅ 7. Sensor Fusion Visualization & Monitor
    launch_actions.append(
        TimerAction(
            period=25.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🎯 SENSOR FUSION STATUS MONITOR:" && '
                         'sleep 5 && '
                         'echo "📡 Active Topics:" && '
                         'ros2 topic list | grep -E "(fused|velodyne_points|calibration)" && '
                         'echo "📊 Fusion Node Status:" && '
                         'ros2 node list | grep -E "(fusion|calibration)" && '
                         'echo "🔥 System Performance:" && '
                         'nvidia-smi --query-gpu=utilization.gpu,memory.used,temperature.gpu --format=csv,noheader,nounits 2>/dev/null || echo "GPU: Not available" && '
                         'echo "🎯 ULTRA ACCURATE SENSOR FUSION ACTIVE!"'],
                    output='screen'
                )
            ]
        )
    )
    
    # ✅ 8. RViz2 for Sensor Fusion Visualization
    launch_actions.append(
        TimerAction(
            period=30.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "🚀 Starting RViz2 for Sensor Fusion Visualization..." && '
                         'export DISPLAY=:0 && '
                         'rviz2 -d /opt/ros/humble/share/rviz_common/default_plugins/robot_model.rviz &'],
                    output='screen'
                )
            ]
        )
    )
    
    # ✅ 9. Performance Optimization for Sensor Fusion
    launch_actions.append(
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'echo "🔧 Optimizing system for ULTRA ACCURATE sensor fusion..." && '
                 'export CUDA_VISIBLE_DEVICES=0 && '
                 'export CUDA_LAUNCH_BLOCKING=0 && '
                 'export CUDA_DEVICE_ORDER=PCI_BUS_ID && '
                 'export NVIDIA_TF32_OVERRIDE=0 && '
                 'export CUDA_CACHE_MAXSIZE=2147483648 && '
                 'export CUDA_CACHE_DISABLE=0 && '
                 'export OMP_NUM_THREADS=8 && '
                 'export ROS_LOCALHOST_ONLY=0 && '
                 'echo "✅ System optimized for maximum sensor fusion accuracy!"'],
            output='screen'
        )
    )
    
    # ✅ 10. Usage Instructions
    launch_actions.append(
        TimerAction(
            period=35.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'echo "" && '
                         'echo "🎯========== ULTRA ACCURATE SENSOR FUSION READY! ==========" && '
                         'echo "📋 AVAILABLE SERVICES:" && '
                         'echo "   ros2 service call /start_calibration std_srvs/srv/Trigger" && '
                         'echo "   ros2 service call /capture_calibration_sample std_srvs/srv/Trigger" && '
                         'echo "   ros2 service call /save_calibration std_srvs/srv/Trigger" && '
                         'echo "" && '
                         'echo "📡 FUSION TOPICS:" && '
                         'echo "   /global_fusion_result - Global multi-camera fusion" && '
                         'echo "   /camera_*_fused - Individual camera fusion results" && '
                         'echo "   /calibration_visualization - Calibration process" && '
                         'echo "" && '
                         'echo "🔍 MONITORING:" && '
                         'echo "   ros2 topic hz /velodyne_points" && '
                         'echo "   ros2 topic hz /global_fusion_result" && '
                         'echo "   ros2 node info /multicamera_lidar_fusion" && '
                         'echo "==========================================================" && '
                         'echo ""'],
                    output='screen'
                )
            ]
        )
    )
    
    return LaunchDescription([
        declare_use_fusion,
        declare_use_global_fusion,
        declare_calibration_file
    ] + launch_actions)
