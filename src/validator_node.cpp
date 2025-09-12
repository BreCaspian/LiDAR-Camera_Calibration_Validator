// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025 Yao Yuzhuo (yaoyuzhuo6@gmail.com)
// This file is part of LiDAR-Camera Calibration Validator and is licensed under
// the GNU General Public License v3.0 or later. See the LICENSE file for details.

#include <ros/ros.h>
#include <signal.h>
#include "calibration_validator.h"

bool g_shutdown_requested = false;

void signalHandler(int sig) {
    ROS_INFO("[ValidatorNode] Received signal %d, shutting down gracefully...", sig);
    g_shutdown_requested = true;
    ros::shutdown();
}

void printStartupInfo() {
    ROS_INFO("========================================");
    ROS_INFO("  LiDAR-Camera Calibration Validator   ");
    ROS_INFO("========================================");
    ROS_INFO("Version: 3.1.0");
    ROS_INFO("Based on ROBOMASTTER-HORIZON-LiDAR-2025 PointCloudVisualizer");
    ROS_INFO(" ");
    ROS_INFO("Features:");
    ROS_INFO("  - Real-time LiDAR-camera fusion visualization");
    ROS_INFO("  - Quantitative calibration quality metrics");
    ROS_INFO("  - Interactive parameter adjustment via rqt_reconfigure");
    ROS_INFO("  - Edge overlap score and normalized mutual information");
    ROS_INFO(" ");
    ROS_INFO("Usage:");
    ROS_INFO("  1. Configure calibration parameters in config/sample_calibration.yaml");
    ROS_INFO("  2. Adjust topics in config/settings.yaml if needed");
    ROS_INFO("  3. Run: rosrun rqt_reconfigure rqt_reconfigure (for parameter tuning)");
    ROS_INFO("  4. Monitor fusion quality in real-time");
    ROS_INFO(" ");
    ROS_INFO("Quality Guidelines:");
    ROS_INFO("  - Edge Overlap Score > 0.7: Good calibration");
    ROS_INFO("  - Edge Overlap Score 0.4-0.7: Fair calibration");
    ROS_INFO("  - Edge Overlap Score < 0.4: Poor calibration");
    ROS_INFO("  - Normalized MI > 0.3: Good correlation");
    ROS_INFO("========================================");
}

bool checkEnvironment() {
    if (!ros::param::has("/use_sim_time")) {
        ROS_WARN("[ValidatorNode] /use_sim_time parameter not found. This is normal for real robot operation.");
    }
    
    const char* display = getenv("DISPLAY");
    if (!display || strlen(display) == 0) {
        ROS_WARN("[ValidatorNode] No DISPLAY environment variable found.");
        ROS_WARN("[ValidatorNode] OpenCV windows may not display properly.");
        ROS_WARN("[ValidatorNode] Consider running with X11 forwarding or VNC.");
    }
    
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "validator_node", ros::init_options::NoSigintHandler);
    
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    printStartupInfo();
    
    if (!checkEnvironment()) {
        ROS_ERROR("[ValidatorNode] Environment check failed!");
        return -1;
    }
    
    try {
        ROS_INFO("[ValidatorNode] Initializing CalibrationValidator...");
        lidar_cam_validator::CalibrationValidator validator(nh, pnh);
        
        ROS_INFO("[ValidatorNode] CalibrationValidator initialized successfully!");
        ROS_INFO("[ValidatorNode] Waiting for image and point cloud data...");
        ROS_INFO("[ValidatorNode] Use Ctrl+C to exit gracefully");
        
        double loop_rate_hz;
        pnh.param<double>("loop_rate", loop_rate_hz, 30.0);
        ros::Rate loop_rate(loop_rate_hz);
        
        ros::Time last_warning_time = ros::Time::now();
        const double warning_interval = 10.0; 

        while (ros::ok() && !g_shutdown_requested) {
            ros::spinOnce();

            ros::Time current_time = ros::Time::now();
            if (validator.hasDataReceived()) {
                last_warning_time = current_time;
            } else {
                if ((current_time - last_warning_time).toSec() > warning_interval) {
                    ROS_WARN_THROTTLE(10.0, "[ValidatorNode] No data received for a while. Check topic connections:");
                    ROS_WARN_THROTTLE(10.0, "[ValidatorNode]   rostopic list | grep -E '(image|points)'");
                    ROS_WARN_THROTTLE(10.0, "[ValidatorNode]   rostopic hz <your_topic>");
                    last_warning_time = current_time;
                }
            }

            loop_rate.sleep();
        }
        
    } catch (const std::exception& e) {
        ROS_ERROR("[ValidatorNode] Exception caught: %s", e.what());
        return -1;
    } catch (...) {
        ROS_ERROR("[ValidatorNode] Unknown exception caught!");
        return -1;
    }
    
    ROS_INFO("[ValidatorNode] Shutting down gracefully...");
    
    cv::destroyAllWindows();
    
    ROS_INFO("[ValidatorNode] Shutdown complete. Goodbye!");
    return 0;
}

/****************************************************************************** 
 *                          版权声明（Copyright Notice）                         
 ****************************************************************************** 
 *
 * 本项目的所有源代码及相关文件均由 姚宇倬 独立开发完成，
 * 作者依法享有完整的知识产权及最终解释权。
 *
 * All source code and related files of this project were independently
 * developed by Yao Yuzhuo. The author retains full intellectual property
 * rights and the final right of interpretation.
 *
 * 本项目采用 GNU GPL v3 协议 授权分发，但附加以下限制：
 * This project is distributed under the terms of the GNU GPL v3 license,
 * with the following additional restrictions:
 *
 * 使用范围（Permitted Use）
 * 本代码及相关资源仅可用于科研、教学和个人学习。
 * The code and related resources may only be used for scientific research,
 * academic study, and educational purposes.
 *
 * 禁止用途（Prohibited Use）
 * 严禁任何形式的商业用途，包括但不限于：产品化、盈利性服务、
 * 商业推广、技术转让或其他任何营利行为。
 * Any form of commercial use is strictly forbidden, including but not limited to:
 * productization, profit-oriented services, commercial promotion,
 * technology transfer, or any other activities for financial gain.
 *
 * 开源与再分发要求（Open Source and Redistribution Requirements）
 * 若需再次分发源代码或其衍生版本，必须遵循 GPL v3 协议的相关规定，
 * 并附带本声明。修改或衍生的版本亦须保持开源并附带相同的许可限制。
 * Any redistribution of this source code or derivative works must comply with
 * the terms of the GNU GPL v3 license and must include this copyright notice.
 * Modified or derivative versions must also remain open source and include
 * the same licensing restrictions.
 *
 * 侵权责任（Liability for Infringement）
 * 任何未经授权的复制、修改、传播、再发布、出售或商业化利用，
 * 均构成对作者知识产权的严重侵犯。作者将依法追究侵权者的法律责任，
 * 必要时包括刑事责任。
 * Any unauthorized copying, modification, distribution, republication, sale,
 * or commercial exploitation constitutes a serious violation of the author’s
 * intellectual property rights. The author will pursue legal action against
 * infringers to the fullest extent of the law, including civil, administrative,
 * and, if necessary, criminal liability.
 *
 * 作者信息（Author Information）
 * 作者：姚宇倬
 * Author: Yao Yuzhuo
 *
 * 邮箱：yaoyuzhuo6@gmail.com
 * Email: yaoyuzhuo6@gmail.com
 *
 * 单位：华北理工大学 RoboMaster 机器人实验室 Horizon 战队
 * Institution: North China University of Science and Technology
 *              RoboMaster Robotics Laboratory - Horizon Team
 *
 ******************************************************************************/
