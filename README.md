# 🚀 LiDAR-Camera Calibration Validator

一个用于验证激光雷达-相机联合标定质量的高性能ROS包，提供实时可视化、定量评价指标和精确的投影计算。

A high-performance ROS package for validating LiDAR-camera calibration quality through real-time visualization, quantitative metrics, and precise projection calculations.

---

## ✨ 主要特性 | Key Features

### 🌟 核心功能
- **高精度投影**: 包含完整畸变校正的精确点云投影计算

- **定量评价指标**: 边缘重叠度、归一化互信息等多种质量评价指标

- **实时参数调节**: 通过动态重配置进行无重启参数调整

- **边缘检测分析**: Canny边缘检测和重叠分析

- **性能监控**: 实时FPS、处理时间等性能统计

  ---

## 📊 系统流程图 | System Flow

```mermaid
%%{init: {"themeVariables": {
    "fontSize": "18px",
    "nodeSpacing": 28,
    "rankSpacing": 35
}}}%%

graph TB
    %% 节点样式
    classDef input   fill:#4DB6AC,stroke:#00695C,stroke-width:2px,color:#fff,font-weight:bold
    classDef config  fill:#64B5F6,stroke:#1E88E5,stroke-width:2px,color:#fff,font-weight:bold
    classDef sync    fill:#BA68C8,stroke:#6A1B9A,stroke-width:2px,color:#fff,font-weight:bold
    classDef process fill:#FFD54F,stroke:#F9A825,stroke-width:2px,color:#000,font-weight:bold
    classDef metrics fill:#FF8A65,stroke:#D84315,stroke-width:2px,color:#fff,font-weight:bold
    classDef output  fill:#90A4AE,stroke:#37474F,stroke-width:2px,color:#fff,font-weight:bold

    %% 输入层
    subgraph 输入层
        A["相机图像 -- /image_topic"]
        B["LiDAR 点云 -- /cloud_topic"]
    end
    class A,B input

    %% 配置层
    subgraph 配置层
        C["标定参数 -- K / C / E"]
        D["动态参数 -- Validator.cfg"]
    end
    class C,D config

    %% 同步层
    subgraph 同步层
        E["时间同步 -- ApproximateTime"]
    end
    class E sync

    %% 处理层
    subgraph 处理层
        F["点云预处理 -- 降采样/过滤"]
        G["点云投影 -- 投影到图像平面"]
        H["可视化叠加 -- 统计/边缘/色条"]
    end
    class F,G,H process

    %% 指标层
    subgraph 指标层
        I["质量指标 -- EdgeOverlap/NMI"]
        J["性能统计 -- FPS/延迟"]
    end
    class I,J metrics

    %% 输出层
    subgraph 输出层
        K["ROS 发布 -- /fused_image"]
        L["ROS 发布 -- /validation_info"]
        M["GUI 显示 -- OpenCV 窗口"]
    end
    class K,L,M output

    %% 数据流
    A --> E
    B --> E
    C --> G
    D --> G
    E --> F --> G --> H
    G --> I
    H --> K
    H --> M
    I --> L
    H --> J
    J --> L
```
---
## 🔍 推荐标定工具 | Recommended Calibration Tools

在使用本验证工具前，建议先完成 **相机标定** 与 **激光雷达-相机联合标定**，以保证结果的准确性。

### 📷 相机标定 (Camera Calibration)

- [ROS 官方相机标定工具](https://wiki.ros.org/camera_calibration)
   提供单目/双目相机标定，输出相机内参与畸变参数。

### 🔗 激光雷达-相机联合标定 (LiDAR-Camera Extrinsic Calibration)

- [direct_visual_lidar_calibration (GitHub)](https://github.com/koide3/direct_visual_lidar_calibration)
   基于视觉与点云的高精度标定工具。
  
- [官方教程 | Official Tutorial](https://koide3.github.io/direct_visual_lidar_calibration/)
   包含安装、数据准备、运行示例等详细步骤。
  
---

## 🛠️ 系统要求 | System Requirements

### 必需依赖 | Required Dependencies

#### 核心依赖
- **ROS Noetic** - Ubuntu 20.04
- **OpenCV 4.x** 
- **PCL 1.10+** 
- **Eigen3**
- **dynamic_reconfigure** 

#### GUI依赖 (可选)
- **python3-rospkg** 

- **ros-noetic-rqt-reconfigure**

- **ros-noetic-rqt-gui** 

  ---

## 📦 安装指南 | Installation Guide

### 一键安装 

#### 1. 克隆项目
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/BreCaspian/LiDAR-Camera_Calibration_Validator.git
cd lidar_cam_validator
```

#### 2. 检查依赖状态
```bash
./scripts/quick_start.sh --check-deps
```

#### 3. 根据提示安装依赖
```bash
# 如果有缺失依赖，按照脚本提示安装
```
---

## 🚀 快速开始 | Quick Start

### 1. 一键启动 (推荐)

#### 快速启动使用
```bash
# 进入项目目录
cd ~/catkin_ws/src/lidar_cam_validator

# 编辑配置标定参数
vim ~/catkin_ws/src/lidar_cam_validator/config/sample_calibration.yaml

# 一键启动（将会自动编译、启动验证器、参数GUI、可视化GUI）
./scripts/quick_start.sh -i /camera/image_raw -c /velodyne_points 
# 请替换为您的 相机图像话题 /camera/image_raw 和 点云话题 /velodyne_points 
```
---

#### 高级选项
```bash
# 检查依赖状态
./scripts/quick_start.sh --check-deps

# 运行系统测试
./scripts/quick_start.sh --test

# 强制重新编译
./scripts/quick_start.sh --force-compile

# 指定话题启动
./scripts/quick_start.sh -i /camera/image_raw -c /velodyne_points

# 指定标定文件
./scripts/quick_start.sh -f /path/to/your/calibration.yaml

# 不启动GUI
./scripts/quick_start.sh --no-gui

# 查看完整帮助
./scripts/quick_start.sh --help
```

### 2. 手动启动 (高级用户操作选项)

#### 启动验证器
```bash
# 设置ROS环境
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 启动验证器
roslaunch lidar_cam_validator validator.launch

# 启动参数调节界面 (新终端)
rosrun rqt_reconfigure rqt_reconfigure
```

### 3. 配置标定参数

#### 编辑标定文件
```bash
vim ~/catkin_ws/src/lidar_cam_validator/config/sample_calibration.yaml
```

#### 标定参数格式
```yaml
# 相机内参矩阵 (3x3)
K_0: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]

# 相机畸变系数 (1x5) - 支持径向和切向畸变校正
C_0: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [k1, k2, p1, p2, k3]

# 外参矩阵: 激光雷达 -> 相机 (4x4) (若是 相机 -> 雷达，请务必求逆！！！)
E_0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [R11, R12, R13, tx,
          R21, R22, R23, ty,
          R31, R32, R33, tz,
          0,   0,   0,   1]
```

### 4. 配置话题设置

#### 方法一：命令行指定 (推荐)
```bash
./scripts/quick_start.sh -i /your_camera/image_raw -c /your_lidar/cloudpoints
```

#### 方法二：编辑配置文件
```bash
vim ~/catkin_ws/src/lidar_cam_validator/config/settings.yaml
```

```yaml
image_topic: "/camera/image_raw"    # 替换为您的相机图像话题
cloud_topic: "/cloudpoints"        # 替换为您的点云话题

fused_topic: "/validator/fused_image"
info_topic: "/validator/validation_info"

calibration_file: "$(find lidar_cam_validator)/config/sample_calibration.yaml"

```

---

## ⚙️ 参数配置 | Parameter Configuration

### 动态参数调节 (实时生效)

使用rqt_reconfigure进行实时参数调节，所有修改立即生效，无需重启节点：

#### 可视化参数组 (Visualization)
| 参数名 | 类型 | 范围 | 默认值 | 描述 |
|--------|------|------|--------|------|
| **point_size** | int | 1-8 | 3 | 点云显示大小 (像素) |
| **enable_depth_color** | bool | - | true | 启用深度颜色编码 |
| **show_statistics** | bool | - | true | 显示统计信息 |
| **alpha** | double | 0.0-1.0 | 0.7 | 图像透明度 |

#### 处理参数组 (Processing)
| 参数名 | 类型 | 范围 | 默认值 | 描述 |
|--------|------|------|--------|------|
| **enable_edge_detection** | bool | - | true | 启用边缘检测分析 |
| **enable_metrics** | bool | - | true | 启用详细评价指标 |
| **enable_distortion_correction** | bool | - | true | 启用畸变校正 |
| **depth_filter_threshold** | double | 0.01-1.0 | 0.01 | 深度过滤阈值 (米) |

#### 性能参数组 (Performance)
| 参数名 | 类型 | 范围 | 默认值 | 描述 |
|--------|------|------|--------|------|
| **enable_parallel_processing** | bool | - | true | 启用并行处理 |
| **high_density_threshold** | int | 10000-1000000 | 100000 | 高密度点云阈值 |
| **batch_size** | int | 1000-50000 | 50000 | 批处理大小 |
| **num_threads** | int | 1-16 | 4 | 处理线程数 |

#### 控制参数组 (Control)
| 参数名 | 类型 | 描述 |
|--------|------|------|
| **reset_to_defaults** | bool | 一键重置所有参数到默认值 |

---

## 系统输出 | System Output

### 可视化输出

#### 主显示窗口
- **融合图像**: 点云投影叠加到相机图像
- **深度颜色编码**: 彩色深度映射 (红色=近，蓝色=远)
- **颜色条**: 右上角深度范围指示器
- **统计信息**: 左上角实时处理统计

#### 界面元素
- **点云可视化**: 3像素默认点大小，清晰可见
- **边缘检测**: Canny边缘检测叠加显示
- **性能监控**: FPS、处理时间、点云数量
- **质量指标**: 边缘重合度、归一化互信息

### ROS话题输出
- **`/validator/fused_image`**: 融合后的图像 (sensor_msgs/Image)
- **`/validator/validation_info`**: 验证信息和统计数据
- **`/validator/metrics`**: 定量评价指标
- **`/validator/status`**: 系统运行状态

---

## 📝 开发指南 | Development Guide

```
lidar_cam_validator/
├── include/
│   └── calibration_validator.h      # 主要头文件
├── src/
│   ├── calibration_validator.cpp    # 核心实现
│   └── validator_node.cpp          # 节点入口
├── scripts/
│   └── quick_start.sh              # 一键启动脚本
├── config/
│   ├── Validator.cfg               # 动态参数配置
│   ├── sample_calibration.yaml    # 标定参数示例
│   └── settings.yaml              # 话题设置
├── launch/
│   └── validator.launch           # 启动文件
├── README.md                       # 完整项目文档
└── CMakeLists.txt                 # 构建配置
```

---

## 🔧 贡献指南 | Contributing

1. Fork 项目到您的 GitHub 账户
2. 创建功能分支: `git checkout -b feature/your-feature`
3. 提交更改: `git commit -am 'Add some feature'`
4. 推送分支: `git push origin feature/your-feature`
5. 创建 Pull Request

---

## 📄 许可证 | License

本项目采用 GNU GPL v3 或更高版本 (GPL-3.0-or-later) 许可 - 详见 [LICENSE](LICENSE)

This project is licensed under the GNU General Public License v3.0 or later (GPL-3.0-or-later) - see the [LICENSE](LICENSE) file for details.

### 📋 您的义务

- **Copyleft保护**：基于本项目的衍生作品必须使用相同许可证

- **源码公开**：分发时必须提供源代码或提供获取源码的方式

- **许可证保留**：必须保留原始许可证和版权声明

- **修改声明**：必须明确标注对原始代码的修改

  ---

## 🤝 致谢 | Acknowledgments

- ROS社区提供的优秀框架
- OpenCV和PCL库的强大支持
- Dr. Kenji Koide 开源联合标定工具

---

## 📮 联系方式 | Contact

如有问题或建议，请通过以下方式联系：

**项目作者**: 姚宇倬
**邮箱**: yaoyuzhuo6@gmail.com

---

<p align="center">
  <b>LiDAR-Camera Calibration Validator v3.1</b> - 激光雷达-相机联合标定结果快速验证！ 🚀
</p>
