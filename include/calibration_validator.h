// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025 Yao Yuzhuo (yaoyuzhuo6@gmail.com)
// This file is part of LiDAR-Camera Calibration Validator and is licensed under
// the GNU General Public License v3.0 or later. See the LICENSE file for details.

#ifndef CALIBRATION_VALIDATOR_H
#define CALIBRATION_VALIDATOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>

#include <Eigen/Dense>

#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <future>
#include <algorithm>
#include <map>

#include <boost/thread/recursive_mutex.hpp>
#include <lidar_cam_validator/ValidatorConfig.h>

#define PARALLEL_PROCESSING_THRESHOLD 100000


namespace lidar_cam_validator {

struct ValidationMetrics {
    double edge_overlap_score     = 0.0;
    double normalized_mutual_info = 0.0;
    double projection_ratio       = 0.0;
    double mean_edge_distance     = 0.0;

    int valid_projections         = 0;
    int total_points              = 0;

    double processing_time_ms     = 0.0;
    double min_depth              = 0.0;
    double max_depth              = 0.0;
    double mean_depth             = 0.0;
};


class CalibrationValidator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CalibrationValidator(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~CalibrationValidator();

    bool hasDataReceived() const { return data_received_; }
    ros::Time getLastDataTime() const { return last_data_time_; }

private:
    ros::NodeHandle nh_, pnh_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::PointCloud2> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> image_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
    std::shared_ptr<Synchronizer> sync_;

    image_transport::ImageTransport it_;
    image_transport::Publisher fused_image_pub_;
    ros::Publisher validation_info_pub_;

    std::shared_ptr<dynamic_reconfigure::Server<lidar_cam_validator::ValidatorConfig>> config_server_;
    lidar_cam_validator::ValidatorConfig config_;

    cv::Mat K_matrix_;
    cv::Mat D_coeffs_;
    cv::Mat E_matrix_;
    Eigen::Matrix4f lidar_to_camera_;
    Eigen::Matrix3f camera_matrix_;
    bool calibration_loaded_;

    cv::Mat latest_image_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;

    std::vector<cv::Point2f> projected_points_cache_;
    std::vector<float> depths_cache_;
    std::vector<cv::Scalar> colors_cache_;

    bool use_parallel_processing_;
    int processing_batch_size_;
    std::chrono::steady_clock::time_point last_process_time_;
    double processing_fps_;
    int frame_count_;

    std::string image_topic_;
    std::string cloud_topic_;
    std::string fused_topic_;
    std::string info_topic_;

    ros::Timer param_timer_;
    bool window_initialized_;
    bool data_received_;
    ros::Time last_data_time_;

    std::mutex config_mutex_;
    boost::recursive_mutex config_server_mutex_;
    std::mutex data_mutex_;

    // Callbacks & Processing
    void syncCallback(const sensor_msgs::ImageConstPtr& image_msg,
                      const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    cv::Mat projectPointsToImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                 const cv::Mat& image,
                                 ValidationMetrics& metrics);

    cv::Scalar depthToColor(float depth, float min_depth, float max_depth);
    cv::Point2f applyDistortionCorrection(float u_norm, float v_norm);

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int max_points);

    ValidationMetrics calculateValidationMetrics(const cv::Mat& image,
                                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                 const std::vector<cv::Point2f>& projected_points,
                                                 const std::vector<float>& depths);

    double calculateEdgeOverlapScore(const cv::Mat& image,
                                     const std::vector<cv::Point2f>& projected_points);

    double calculateNormalizedMutualInfo(const cv::Mat& image,
                                         const std::vector<cv::Point2f>& projected_points,
                                         const std::vector<float>& depths);

    void drawStatistics(cv::Mat& image, const ValidationMetrics& metrics);
    void drawEdgeOverlay(cv::Mat& image);
    void drawDepthColorbar(cv::Mat& image, float min_depth, float max_depth);

    bool loadCalibrationParameters();
    void configCallback(lidar_cam_validator::ValidatorConfig& config, uint32_t level);
    void checkParameterUpdates(const ros::TimerEvent& event);

    cv::Mat projectPointsToImageFast(const cv::Mat& image,
                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                     ValidationMetrics& metrics);

    void processPointBatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                           int start_idx, int end_idx,
                           int width, int height,
                           float min_distance, float max_distance,
                           std::vector<cv::Point2f>& points,
                           std::vector<float>& depths,
                           std::vector<cv::Scalar>& colors);

    void publishValidationInfo(const ValidationMetrics& metrics);
    void updatePerformanceStats();
};

} // namespace lidar_cam_validator

#endif // CALIBRATION_VALIDATOR_H


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
