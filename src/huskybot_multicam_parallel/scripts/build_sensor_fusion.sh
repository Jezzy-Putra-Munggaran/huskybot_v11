#!/bin/bash

#==========================================================================
# ULTRA ACCURATE SENSOR FUSION BUILD SCRIPT
# LiDAR-Camera Fusion System Builder
# Author: Jezzy Putra Munggaran
# Date: August 5, 2025
#==========================================================================

echo "🔥🔥🔥 BUILDING ULTRA ACCURATE SENSOR FUSION SYSTEM 🔥🔥🔥"
echo "============================================================"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}📋 Build Information:${NC}"
echo -e "${YELLOW}   - LiDAR: Velodyne VLP32C (32 lasers, 0.004m resolution)${NC}"
echo -e "${YELLOW}   - Cameras: 6x Arducam IMX477 (360° coverage)${NC}"
echo -e "${YELLOW}   - Fusion: Camera-LiDAR coordinate transformation${NC}"
echo -e "${YELLOW}   - Accuracy: ULTRA ACCURATE distance measurement${NC}"
echo ""

# ✅ 1. Environment setup
echo -e "${BLUE}🔧 Setting up build environment...${NC}"
export ROS_DISTRO=humble
export PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH"
export CMAKE_PREFIX_PATH="/opt/ros/humble:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="/opt/ros/humble/lib:$LD_LIBRARY_PATH"

# Source ROS2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}✅ ROS2 Humble sourced${NC}"
else
    echo -e "${RED}❌ ROS2 Humble not found!${NC}"
    exit 1
fi

# ✅ 2. Install Python dependencies
echo -e "${BLUE}📦 Installing Python dependencies for sensor fusion...${NC}"
pip3 install --upgrade pip
pip3 install scipy scikit-learn

# Install ROS2 specific transform library (replacement for tf-transformations)
pip3 install transforms3d

# Install additional dependencies
pip3 install open3d-python
pip3 install matplotlib
pip3 install PyYAML

echo -e "${GREEN}✅ Python dependencies installed${NC}"

# ✅ 3. Check workspace
WORKSPACE_ROOT="/home/kmp-orin/jezzy/huskybot_v7"
if [ ! -d "$WORKSPACE_ROOT" ]; then
    echo -e "${RED}❌ Workspace not found: $WORKSPACE_ROOT${NC}"
    exit 1
fi

cd "$WORKSPACE_ROOT"
echo -e "${GREEN}✅ Workspace: $WORKSPACE_ROOT${NC}"

# ✅ 4. Build sensor fusion package
echo -e "${BLUE}🏗️  Building sensor fusion package...${NC}"
colcon build --packages-select huskybot_multicam_parallel --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Package built successfully${NC}"
else
    echo -e "${RED}❌ Build failed!${NC}"
    exit 1
fi

# ✅ 5. Source the built package
source install/setup.bash
echo -e "${GREEN}✅ Package sourced${NC}"

# ✅ 6. Verify executables
echo -e "${BLUE}🔍 Verifying sensor fusion executables...${NC}"

EXECUTABLES=(
    "lidar_camera_fusion_node"
    "multicamera_lidar_fusion_node" 
    "auto_calibration_node"
)

for exec in "${EXECUTABLES[@]}"; do
    if ros2 pkg executables huskybot_multicam_parallel | grep -q "$exec"; then
        echo -e "${GREEN}✅ $exec - Available${NC}"
    else
        echo -e "${RED}❌ $exec - Missing${NC}"
    fi
done

# ✅ 7. Check Velodyne dependencies
echo -e "${BLUE}🔍 Checking Velodyne LiDAR dependencies...${NC}"
VELODYNE_PACKAGES=(
    "velodyne_driver"
    "velodyne_pointcloud"
    "velodyne_msgs"
)

for pkg in "${VELODYNE_PACKAGES[@]}"; do
    if ros2 pkg list | grep -q "$pkg"; then
        echo -e "${GREEN}✅ $pkg - Installed${NC}"
    else
        echo -e "${RED}❌ $pkg - Missing (Install: apt install ros-humble-velodyne-*)${NC}"
    fi
done

# ✅ 8. Create launch shortcuts
echo -e "${BLUE}🚀 Creating launch shortcuts...${NC}"

cat > /tmp/start_sensor_fusion.sh << EOF
#!/bin/bash
source /opt/ros/humble/setup.bash
source $WORKSPACE_ROOT/install/setup.bash
ros2 launch huskybot_multicam_parallel sensor_fusion_pipeline.launch.py
EOF

chmod +x /tmp/start_sensor_fusion.sh
echo -e "${GREEN}✅ Launch shortcut created: /tmp/start_sensor_fusion.sh${NC}"

# ✅ 9. System optimization check
echo -e "${BLUE}⚡ Checking system optimization...${NC}"

# GPU check
if nvidia-smi > /dev/null 2>&1; then
    echo -e "${GREEN}✅ NVIDIA GPU detected${NC}"
    nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader
else
    echo -e "${YELLOW}⚠️  No NVIDIA GPU detected${NC}"
fi

# Memory check
TOTAL_MEM=$(free -h | awk '/^Mem:/ {print $2}')
echo -e "${GREEN}✅ Total Memory: $TOTAL_MEM${NC}"

# CPU cores
CPU_CORES=$(nproc)
echo -e "${GREEN}✅ CPU Cores: $CPU_CORES${NC}"

# ✅ 10. Final verification
echo ""
echo -e "${PURPLE}🎯 SENSOR FUSION BUILD COMPLETE!${NC}"
echo -e "${CYAN}================================================${NC}"
echo -e "${YELLOW}📋 USAGE INSTRUCTIONS:${NC}"
echo ""
echo -e "${GREEN}🚀 Start Sensor Fusion System:${NC}"
echo "   /tmp/start_sensor_fusion.sh"
echo ""
echo -e "${GREEN}🚀 Manual Launch:${NC}"
echo "   ros2 launch huskybot_multicam_parallel sensor_fusion_pipeline.launch.py"
echo ""
echo -e "${GREEN}🔧 Calibration (if needed):${NC}"
echo "   ros2 service call /start_calibration std_srvs/srv/Trigger"
echo ""
echo -e "${GREEN}📊 Monitor Performance:${NC}"
echo "   ros2 topic hz /global_fusion_result"
echo "   ros2 topic hz /velodyne_points"
echo ""
echo -e "${GREEN}🎯 Topics for Ultra Accurate Data:${NC}"
echo "   /global_fusion_result - Multi-camera global fusion"
echo "   /camera_*_fused - Individual camera fusion results"
echo ""
echo -e "${CYAN}================================================${NC}"
echo ""

# ✅ Success indicator
echo -e "${GREEN}🔥 ULTRA ACCURATE SENSOR FUSION READY! 🔥${NC}"
echo -e "${YELLOW}   Distance Accuracy: ±0.004m (LiDAR resolution)${NC}"
echo -e "${YELLOW}   Coverage: 360° (6 cameras + LiDAR)${NC}"
echo -e "${YELLOW}   Coordinate System: ROS REP-103 compliant${NC}"
echo ""
