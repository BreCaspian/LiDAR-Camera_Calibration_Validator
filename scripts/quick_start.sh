# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2025 Yao Yuzhuo (yaoyuzhuo6@gmail.com)
# This file is part of LiDAR-Camera Calibration Validator and is licensed under
# the GNU General Public License v3.0 or later. See the LICENSE file for details.

#!/bin/bash
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m'

IMAGE_TOPIC="/camera/image_raw"
CLOUD_TOPIC="/cloudpoints"
CALIBRATION_FILE=""
LAUNCH_GUI=true
SHOW_HELP=false
CHECK_DEPS=false
RUN_TESTS=false
FORCE_COMPILE=false

show_help() {
    echo -e "${CYAN}🚀 LiDAR-Camera Calibration Validator 一键启动脚本${NC}"
    echo ""
    echo -e "${YELLOW}用法:${NC}"
    echo "  $0 [选项]"
    echo ""
    echo -e "${YELLOW}基本选项:${NC}"
    echo "  -i, --image-topic TOPIC    指定图像话题 (默认: /image_raw)"
    echo "  -c, --cloud-topic TOPIC    指定点云话题 (默认: /cloudpoints)"
    echo "  -f, --calibration-file FILE 指定标定文件路径"
    echo "  --no-gui                   不启动参数调节GUI"
    echo ""
    echo -e "${YELLOW}高级选项:${NC}"
    echo "  --check-deps               检查依赖并给出安装建议"
    echo "  --test                     运行系统测试"
    echo "  --force-compile            强制重新编译项目"
    echo "  -h, --help                 显示此帮助信息"
    echo ""
    echo -e "${YELLOW}示例:${NC}"
    echo "  $0                                    # 基本启动"
    echo "  $0 --check-deps                      # 检查依赖状态"
    echo "  $0 --test                            # 运行测试后启动"
    echo "  $0 -i /camera/image_raw -c /velodyne_points"
    echo "  $0 --no-gui --force-compile         # 强制编译，不启动GUI"
    echo ""
    echo -e "${YELLOW}功能特性:${NC}"
    echo -e "  ${GREEN}✅ 高性能处理${NC}: 点云高性能处理"
    echo -e "  ${GREEN}✅ 实时调节${NC}: 动态参数调节"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        -i|--image-topic)
            IMAGE_TOPIC="$2"
            shift 2
            ;;
        -c|--cloud-topic)
            CLOUD_TOPIC="$2"
            shift 2
            ;;
        -f|--calibration-file)
            CALIBRATION_FILE="$2"
            shift 2
            ;;
        --no-gui)
            LAUNCH_GUI=false
            shift
            ;;
        --check-deps)
            CHECK_DEPS=true
            shift
            ;;
        --test)
            RUN_TESTS=true
            shift
            ;;
        --force-compile)
            FORCE_COMPILE=true
            shift
            ;;
        -h|--help)
            SHOW_HELP=true
            shift
            ;;
        *)
            echo -e "${RED}❌ 未知选项: $1${NC}"
            echo "使用 -h 或 --help 查看帮助信息"
            exit 1
            ;;
    esac
done

if [ "$SHOW_HELP" = true ]; then
    show_help
    exit 0
fi

echo -e "${CYAN}🚀 LiDAR-Camera Calibration Validator 启动中...${NC}"
echo "=========================================="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"
CATKIN_WS="$(dirname "$(dirname "$PACKAGE_DIR")")"

if [ "$CHECK_DEPS" = true ]; then
    echo -e "${PURPLE}� 检查依赖状态...${NC}"
    echo "=========================================="

    MISSING_DEPS=()
    INSTALL_COMMANDS=()

    echo -e "${BLUE}📋 检查python3-rospkg...${NC}"
    if python3 -c "import rospkg" 2>/dev/null; then
        echo -e "${GREEN}✅ python3-rospkg已安装${NC}"
    else
        echo -e "${RED}❌ python3-rospkg未安装${NC}"
        MISSING_DEPS+=("python3-rospkg")
        INSTALL_COMMANDS+=("sudo apt install python3-rospkg")
    fi

    echo -e "${BLUE}� 检查ros-noetic-rqt-reconfigure...${NC}"
    if command -v rqt_reconfigure &> /dev/null; then
        echo -e "${GREEN}✅ ros-noetic-rqt-reconfigure已安装${NC}"
    else
        echo -e "${RED}❌ ros-noetic-rqt-reconfigure未安装${NC}"
        MISSING_DEPS+=("ros-noetic-rqt-reconfigure")
        INSTALL_COMMANDS+=("sudo apt install ros-noetic-rqt-reconfigure")
    fi

    echo -e "${BLUE}� 检查ROS GUI依赖...${NC}"
    if dpkg -l | grep -q "ros-noetic-rqt-gui"; then
        echo -e "${GREEN}✅ ROS GUI依赖已安装${NC}"
    else
        echo -e "${YELLOW}⚠️  部分ROS GUI依赖可能缺失${NC}"
        MISSING_DEPS+=("ros-noetic-rqt-gui")
        INSTALL_COMMANDS+=("sudo apt install ros-noetic-rqt-gui ros-noetic-rqt-gui-py ros-noetic-rqt-common-plugins")
    fi

    if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
        echo ""
        echo -e "${YELLOW}  缺失的依赖:${NC}"
        for dep in "${MISSING_DEPS[@]}"; do
            echo "  - $dep"
        done

        echo ""
        echo -e "${CYAN}💡 安装建议:${NC}"
        echo -e "${CYAN}请运行以下命令安装缺失的依赖:${NC}"
        echo ""
        echo -e "${YELLOW}# 更新包列表${NC}"
        echo "sudo apt update"
        echo ""
        echo -e "${YELLOW}# 安装缺失的依赖${NC}"
        for cmd in "${INSTALL_COMMANDS[@]}"; do
            echo "$cmd"
        done
        echo ""
        echo -e "${CYAN}安装完成后，重新运行此脚本即可正常启动。${NC}"
        exit 1
    else
        echo ""
        echo -e "${GREEN}🎉 所有依赖检查通过！${NC}"
    fi
fi

echo -e "${BLUE}📋 检查ROS环境...${NC}"
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ROS环境未设置，请先source ROS setup.bash${NC}"
    echo "请执行: source /opt/ros/noetic/setup.bash"
    exit 1
fi
echo -e "${GREEN}✅ ROS $ROS_DISTRO 环境已设置${NC}"

echo -e "${BLUE}📋 检查catkin工作空间...${NC}"

if [ "$FORCE_COMPILE" = true ] || [ ! -f "$CATKIN_WS/devel/lib/lidar_cam_validator/validator_node" ]; then
    echo -e "${YELLOW}⚡ 编译项目...${NC}"
    cd "$CATKIN_WS"
    if [ "$FORCE_COMPILE" = true ]; then
        catkin build lidar_cam_validator --force-cmake
    else
        catkin build lidar_cam_validator
    fi
    if [ $? -ne 0 ]; then
        echo -e "${RED}❌ 编译失败${NC}"
        exit 1
    fi
    echo -e "${GREEN}✅ 编译成功${NC}"
fi

source "$CATKIN_WS/devel/setup.bash"
echo -e "${GREEN}✅ 工作空间已设置: $CATKIN_WS${NC}"

if [ "$RUN_TESTS" = true ]; then
    echo -e "${PURPLE}🧪 运行系统测试...${NC}"
    echo "=========================================="
    
    echo -e "${BLUE}📋 基本功能测试...${NC}"
    
    if [ -f "$CATKIN_WS/devel/lib/lidar_cam_validator/validator_node" ]; then
        echo -e "${GREEN}✅ validator_node编译成功${NC}"
    else
        echo -e "${RED}❌ validator_node编译失败${NC}"
        exit 1
    fi
    
    if [ -f "$PACKAGE_DIR/config/Validator.cfg" ]; then
        echo -e "${GREEN}✅ 动态参数配置文件存在${NC}"
    else
        echo -e "${RED}❌ 动态参数配置文件不存在${NC}"
        exit 1
    fi
    
    if [ -f "$PACKAGE_DIR/launch/validator.launch" ]; then
        echo -e "${GREEN}✅ launch文件存在${NC}"
    else
        echo -e "${RED}❌ launch文件不存在${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}✅ 所有测试通过${NC}"
fi

echo -e "${BLUE}📋 检查roscore...${NC}"

if rostopic list &> /dev/null; then
    echo -e "${GREEN}✅ roscore已运行并可访问${NC}"
    ROSCORE_STARTED_BY_SCRIPT=false
else
    if pgrep -f "roscore\|rosmaster" > /dev/null; then
        echo -e "${YELLOW}⚠️  检测到其他roscore进程，但无法访问${NC}"
        echo -e "${YELLOW}⚠️  可能的ROS_MASTER_URI冲突，尝试使用现有roscore...${NC}"
        
        sleep 2
        if rostopic list &> /dev/null; then
            echo -e "${GREEN}✅ 成功连接到现有roscore${NC}"
            ROSCORE_STARTED_BY_SCRIPT=false
        else
            echo -e "${RED}❌ 无法连接到现有roscore，请手动解决冲突${NC}"
            echo -e "${CYAN}建议运行: killall -9 roscore rosmaster${NC}"
            exit 1
        fi
    else
        echo -e "${YELLOW}⚠️  roscore未运行，正在启动...${NC}"
        roscore &
        ROSCORE_PID=$!
        ROSCORE_STARTED_BY_SCRIPT=true
        sleep 3
        
        if rostopic list &> /dev/null; then
            echo -e "${GREEN}✅ roscore已启动并可访问 (PID: $ROSCORE_PID)${NC}"
        else
            echo -e "${RED}❌ roscore启动失败${NC}"
            exit 1
        fi
    fi
fi

echo -e "${BLUE}📋 配置启动参数...${NC}"
LAUNCH_FILE="$PACKAGE_DIR/launch/validator.launch"
TEMP_LAUNCH_FILE="/tmp/validator_temp.launch"

cp "$LAUNCH_FILE" "$TEMP_LAUNCH_FILE"

sed -i "s|<param name=\"image_topic\" value=\".*\"|<param name=\"image_topic\" value=\"$IMAGE_TOPIC\"|g" "$TEMP_LAUNCH_FILE"
sed -i "s|<param name=\"cloud_topic\" value=\".*\"|<param name=\"cloud_topic\" value=\"$CLOUD_TOPIC\"|g" "$TEMP_LAUNCH_FILE"

if [ ! -z "$CALIBRATION_FILE" ]; then
    sed -i "s|<param name=\"calibration_file\" value=\".*\"|<param name=\"calibration_file\" value=\"$CALIBRATION_FILE\"|g" "$TEMP_LAUNCH_FILE"
fi

echo -e "${GREEN}✅ 启动参数已配置${NC}"
echo "   图像话题: $IMAGE_TOPIC"
echo "   点云话题: $CLOUD_TOPIC"
if [ ! -z "$CALIBRATION_FILE" ]; then
    echo "   标定文件: $CALIBRATION_FILE"
fi

echo ""
echo -e "${PURPLE}🎯 启动LiDAR-Camera Calibration Validator...${NC}"
echo "=========================================="

roslaunch "$TEMP_LAUNCH_FILE" &
VALIDATOR_PID=$!
sleep 3

if ps -p $VALIDATOR_PID > /dev/null; then
    echo -e "${GREEN}✅ Validator启动成功 (PID: $VALIDATOR_PID)${NC}"
else
    echo -e "${RED}❌ Validator启动失败${NC}"
    exit 1
fi

if [ "$LAUNCH_GUI" = true ]; then
    echo ""
    echo -e "${PURPLE}🎛️ 启动参数调节GUI...${NC}"
    sleep 2

    echo -e "${BLUE}📋 检查rqt_reconfigure依赖...${NC}"

    if ! python3 -c "import rospkg" &> /dev/null; then
        echo -e "${YELLOW}⚠️  python3-rospkg未安装${NC}"
        echo -e "${CYAN}请运行: sudo apt install python3-rospkg${NC}"
        echo -e "${YELLOW}跳过GUI启动，继续运行验证器...${NC}"
    elif ! command -v rqt_reconfigure &> /dev/null; then
        echo -e "${YELLOW}⚠️  rqt_reconfigure未安装${NC}"
        echo -e "${CYAN}请运行: sudo apt install ros-noetic-rqt-reconfigure${NC}"
        echo -e "${YELLOW}跳过GUI启动，继续运行验证器...${NC}"
    else
        echo -e "${GREEN}✅ rqt_reconfigure依赖检查通过${NC}"
        rqt_reconfigure &
        GUI_PID=$!
        echo -e "${GREEN}✅ 参数调节GUI已启动 (PID: $GUI_PID)${NC}"
        echo ""
        echo -e "${YELLOW}🎛️ 参数调节使用方法:${NC}"
        echo "1. 在rqt_reconfigure中找到'validator_node'"
        echo "2. 展开参数组进行实时调节"
        echo "3. 所有参数修改立即生效"
    fi
fi

echo ""
echo -e "${CYAN}📊 系统状态${NC}"
echo "=========================================="
echo -e "${GREEN}✅ LiDAR-Camera Calibration Validator 运行中${NC}"
echo -e "${GREEN}✅ 高精度点云投影计算 ${NC}"
echo -e "${GREEN}✅ 点云高性能处理已优化 ${NC}"
echo ""
echo -e "${YELLOW}📺 输出话题:${NC}"
echo "  融合图像: /validator/fused_image"
echo "  验证信息: /validator/validation_info"
echo ""
echo -e "${YELLOW}🎮 使用提示:${NC}"
echo "  - 请在 rqt_reconfigure 中实时调节相关参数"
echo "  - 连按 Ctrl+C 两次 退出程序"
echo ""

echo -e "${BLUE}按 Ctrl+C 退出程序...${NC}"
trap 'echo -e "\n${YELLOW}🛑 正在关闭程序...${NC}"; kill $VALIDATOR_PID 2>/dev/null; [ ! -z "$GUI_PID" ] && kill $GUI_PID 2>/dev/null; if [ "$ROSCORE_STARTED_BY_SCRIPT" = true ] && [ ! -z "$ROSCORE_PID" ]; then echo -e "${YELLOW}关闭由脚本启动的roscore...${NC}"; kill $ROSCORE_PID 2>/dev/null; fi; rm -f "$TEMP_LAUNCH_FILE"; echo -e "${GREEN}✅ 程序已退出${NC}"; exit 0' INT

wait $VALIDATOR_PID

