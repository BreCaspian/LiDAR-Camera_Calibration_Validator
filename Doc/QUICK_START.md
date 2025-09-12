# 🚀 快速开始指南 | Quick Start Guide

本指南将帮助您在5分钟内启动并运行LiDAR-Camera Calibration Validator。

This guide will help you get LiDAR-Camera Calibration Validator up and running in 5 minutes.

## 📋 前提条件 | Prerequisites

### 系统要求 | System Requirements
- Ubuntu 20.04 LTS
- ROS Noetic
- 至少4GB RAM | At least 4GB RAM
- 支持OpenGL的显卡 | Graphics card with OpenGL support

### 硬件要求 | Hardware Requirements
- LiDAR传感器 | LiDAR sensor (Velodyne, Ouster, Livox等)
- 相机 | Camera (USB, GigE, 或ROS兼容相机)
- 已完成的LiDAR-Camera联合标定 | Completed LiDAR-Camera calibration

## ⚡ 一键安装 | One-Click Installation

### 方法1: 使用脚本安装 | Method 1: Script Installation

```bash
# 下载并运行安装脚本 | Download and run installation script
wget https://raw.githubusercontent.com/BreCaspian/LiDAR-Camera_Calibration_Validator/main/scripts/quick_start.sh
chmod +x quick_start.sh
./quick_start.sh
```

### 方法2: 手动安装 | Method 2: Manual Installation

```bash
# 1. 创建工作空间 | Create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 2. 克隆项目 | Clone repository
git clone https://github.com/BreCaspian/LiDAR-Camera_Calibration_Validator.git lidar_cam_validator

# 3. 安装依赖 | Install dependencies
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. 编译 | Build
catkin build lidar_cam_validator

# 5. 设置环境 | Setup environment
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## 🔧 配置 | Configuration

### 1. 准备标定文件 | Prepare Calibration File

将您的标定文件复制到配置目录：
Copy your calibration file to the config directory:

```bash
cp your_calibration.yaml ~/catkin_ws/src/lidar_cam_validator/config/my_calibration.yaml
```

### 2. 修改话题配置 | Modify Topic Configuration

编辑配置文件：
Edit the configuration file:

```bash
nano ~/catkin_ws/src/lidar_cam_validator/config/settings.yaml
```

更新话题名称：
Update topic names:

```yaml
# 话题配置 | Topic Configuration
image_topic: "/your_camera/image_raw"      # 您的相机话题 | Your camera topic
cloud_topic: "/your_lidar/points"         # 您的LiDAR话题 | Your LiDAR topic

# 标定文件路径 | Calibration file path
calibration_file: "$(find lidar_cam_validator)/config/my_calibration.yaml"
```

## 🚀 启动验证器 | Launch Validator

### 基础启动 | Basic Launch

```bash
# 启动验证器 | Launch validator
roslaunch lidar_cam_validator validator.launch
```

### 自定义参数启动 | Launch with Custom Parameters

```bash
# 指定标定文件和话题 | Specify calibration file and topics
roslaunch lidar_cam_validator validator.launch \
    calibration_file:=/path/to/your/calibration.yaml \
    image_topic:=/your_camera/image_raw \
    cloud_topic:=/your_lidar/points
```

### 无GUI启动 | Launch without GUI

```bash
# 仅后台运行，不显示GUI | Run in background without GUI
roslaunch lidar_cam_validator validator.launch gui:=false
```

## 🎛️ 实时参数调节 | Real-time Parameter Adjustment

启动参数配置界面：
Launch parameter configuration interface:

```bash
# 打开动态参数配置 | Open dynamic reconfigure
rosrun rqt_reconfigure rqt_reconfigure
```

### 主要参数 | Key Parameters

#### 可视化参数 | Visualization Parameters
- **point_size**: 点的大小 (1-8) | Point size (1-8)
- **min_depth**: 最小深度 (0.1-5.0m) | Minimum depth (0.1-5.0m)
- **max_depth**: 最大深度 (5.0-200.0m) | Maximum depth (5.0-200.0m)
- **show_statistics**: 显示统计信息 | Show statistics overlay
- **show_depth_colorbar**: 显示深度颜色条 | Show depth colorbar

#### 性能参数 | Performance Parameters
- **enable_downsampling**: 启用降采样 | Enable downsampling
- **max_points**: 最大处理点数 | Maximum points to process
- **filter_by_distance**: 距离过滤 | Distance filtering

## 📊 查看结果 | View Results

### 1. 可视化窗口 | Visualization Window

验证器会显示一个窗口，包含：
The validator displays a window containing:

- 🎯 **投影点云**: 彩色编码的深度信息 | Projected point cloud with color-coded depth
- 📊 **统计信息**: 投影比例、边缘重合度等 | Statistics: projection ratio, edge overlap, etc.
- 🎨 **深度颜色条**: 深度值对应的颜色映射 | Depth colorbar showing color mapping

### 2. ROS话题输出 | ROS Topic Output

```bash
# 查看融合图像 | View fused image
rosrun image_view image_view image:=/validator/fused_image

# 查看验证信息 | View validation info
rostopic echo /validator/validation_info
```

### 3. 保存结果 | Save Results

```bash
# 保存融合图像 | Save fused image
rosrun image_view extract_images _sec_per_frame:=0.1 image:=/validator/fused_image
```

## 📈 评价指标解读 | Metrics Interpretation

### 投影比例 | Projection Ratio
- **优秀**: > 85% | **Excellent**: > 85%
- **良好**: 70-85% | **Good**: 70-85%
- **一般**: 50-70% | **Fair**: 50-70%
- **较差**: < 50% | **Poor**: < 50%

### 边缘重合度 | Edge Overlap Score
- **优秀**: > 0.8 | **Excellent**: > 0.8
- **良好**: 0.6-0.8 | **Good**: 0.6-0.8
- **一般**: 0.3-0.6 | **Fair**: 0.3-0.6
- **较差**: < 0.3 | **Poor**: < 0.3

## 🔧 故障排除 | Troubleshooting

### 常见问题 | Common Issues

#### 1. 没有显示投影点云 | No projected point cloud visible

**可能原因** | **Possible causes**:
- 话题名称错误 | Incorrect topic names
- 标定文件路径错误 | Wrong calibration file path
- 传感器数据不同步 | Sensor data not synchronized

**解决方案** | **Solutions**:
```bash
# 检查话题 | Check topics
rostopic list | grep -E "(image|points)"

# 检查话题频率 | Check topic frequency
rostopic hz /your_image_topic
rostopic hz /your_cloud_topic

# 检查标定文件 | Check calibration file
cat ~/catkin_ws/src/lidar_cam_validator/config/my_calibration.yaml
```

#### 2. 投影位置不准确 | Inaccurate projection

**可能原因** | **Possible causes**:
- 标定参数错误 | Incorrect calibration parameters
- 坐标系定义问题 | Coordinate frame issues
- 时间戳同步问题 | Timestamp synchronization issues

**解决方案** | **Solutions**:
- 重新检查标定结果 | Re-verify calibration results
- 确认坐标系变换 | Confirm coordinate transformations
- 检查传感器时间同步 | Check sensor time synchronization

#### 3. 性能问题 | Performance Issues

**解决方案** | **Solutions**:
```bash
# 启用降采样 | Enable downsampling
rosparam set /lidar_cam_validator/enable_downsampling true
rosparam set /lidar_cam_validator/max_points 50000

# 调整处理频率 | Adjust processing frequency
rosparam set /lidar_cam_validator/target_fps 10
```

## 📚 下一步 | Next Steps

1. **深入了解**: 阅读完整的[用户指南](USER_GUIDE.md) | **Learn more**: Read the complete [User Guide](USER_GUIDE.md)
2. **高级配置**: 查看[配置指南](CONFIGURATION.md) | **Advanced setup**: See [Configuration Guide](CONFIGURATION.md)
3. **API文档**: 查看[API参考](API_REFERENCE.md) | **API docs**: Check [API Reference](API_REFERENCE.md)
4. **贡献代码**: 阅读[贡献指南](../CONTRIBUTING.md) | **Contribute**: Read [Contributing Guide](../CONTRIBUTING.md)

## 🆘 获取帮助 | Getting Help

- 📖 **文档**: [完整文档](README.md) | **Documentation**: [Full Documentation](README.md)
- 🐛 **报告问题**: [GitHub Issues](https://github.com/BreCaspian/LiDAR-Camera_Calibration_Validator/issues)
- 💬 **讨论**: [GitHub Discussions](https://github.com/BreCaspian/LiDAR-Camera_Calibration_Validator/discussions)
- 📧 **联系**: yaoyuzhuo6@gmail.com

---

🎉 **恭喜！您已成功启动LiDAR-Camera Calibration Validator！**

🎉 **Congratulations! You have successfully launched LiDAR-Camera Calibration Validator!**
