// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025 Yao Yuzhuo (yaoyuzhuo6@gmail.com)
// This file is part of LiDAR-Camera Calibration Validator and is licensed under
// the GNU General Public License v3.0 or later. See the LICENSE file for details.

#include "calibration_validator.h"
#include <pcl/filters/random_sample.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <numeric>
#include <algorithm>
#include <cmath>

namespace lidar_cam_validator {

CalibrationValidator::CalibrationValidator(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), it_(nh), calibration_loaded_(false),
      processing_fps_(0.0), frame_count_(0), window_initialized_(false),
      data_received_(false), last_data_time_(ros::Time::now()),
      config_mutex_(), data_mutex_() {

    latest_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    if (!loadCalibrationParameters()) {
        ROS_ERROR("[CalibrationValidator] Failed to load calibration parameters!");
        return;
    }

    config_server_.reset(new dynamic_reconfigure::Server<lidar_cam_validator::ValidatorConfig>(config_server_mutex_, pnh_));
    config_server_->setCallback(boost::bind(&CalibrationValidator::configCallback, this, _1, _2));

    pnh_.param<std::string>("image_topic", image_topic_, "/image_raw");
    pnh_.param<std::string>("cloud_topic", cloud_topic_, "/velodyne_points");
    pnh_.param<std::string>("fused_topic", fused_topic_, "/validator/fused_image");
    pnh_.param<std::string>("info_topic", info_topic_, "/validator/validation_info");

    image_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, image_topic_, 10));
    cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloud_topic_, 10));

    int queue_size;
    pnh_.param<int>("queue_size", queue_size, 10);
    sync_.reset(new Synchronizer(SyncPolicy(queue_size), *image_sub_, *cloud_sub_));
    sync_->registerCallback(boost::bind(&CalibrationValidator::syncCallback, this, _1, _2));

    fused_image_pub_ = it_.advertise(fused_topic_, 1);
    validation_info_pub_ = nh_.advertise<std_msgs::String>(info_topic_, 1);

    param_timer_ = nh_.createTimer(ros::Duration(1.0), &CalibrationValidator::checkParameterUpdates, this);

    last_process_time_ = std::chrono::steady_clock::now();
    processing_fps_ = 0.0;
    frame_count_ = 0;

    use_parallel_processing_ = true;
    processing_batch_size_ = 50000;
    projected_points_cache_.reserve(200000);
    depths_cache_.reserve(200000);
    colors_cache_.reserve(200000);

    window_initialized_ = false;

    ROS_INFO("[CalibrationValidator] Initialized successfully!");
    ROS_INFO("[CalibrationValidator] Subscribed to image: %s", image_topic_.c_str());
    ROS_INFO("[CalibrationValidator] Subscribed to cloud: %s", cloud_topic_.c_str());
    ROS_INFO("[CalibrationValidator] Publishing fused image: %s", fused_topic_.c_str());
    ROS_INFO("[CalibrationValidator] Use 'rosrun rqt_reconfigure rqt_reconfigure' to adjust parameters");
}
CalibrationValidator::~CalibrationValidator() {
    cv::destroyAllWindows();
}
bool CalibrationValidator::loadCalibrationParameters() {
    std::string calib_file;
    if (!pnh_.getParam("calibration_file", calib_file)) {
        ROS_ERROR("[CalibrationValidator] Calibration file parameter not found!");
        return false;
    }

    if (calib_file.find("$(find") != std::string::npos) {
        size_t start = calib_file.find("$(find ");
        size_t end = calib_file.find(")", start);
        if (start != std::string::npos && end != std::string::npos) {
            std::string package_part = calib_file.substr(start + 7, end - start - 7);
            std::string package_name = package_part.substr(0, package_part.find(" "));

            std::string cmd = "rospack find " + package_name;
            FILE* pipe = popen(cmd.c_str(), "r");
            if (pipe) {
                char buffer[256];
                std::string package_path;
                if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                    package_path = std::string(buffer);
                    if (!package_path.empty() && package_path.back() == '\n') {
                        package_path.pop_back();
                    }
                }
                pclose(pipe);
                
                if (!package_path.empty()) {
                    calib_file = package_path + calib_file.substr(end + 1);
                }
            }
        }
    }
    
    cv::FileStorage fs(calib_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        ROS_ERROR("[CalibrationValidator] Cannot open calibration file: %s", calib_file.c_str());
        return false;
    }
    
    fs["K_0"] >> K_matrix_;
    fs["C_0"] >> D_coeffs_;
    fs["E_0"] >> E_matrix_;
    fs.release();
    
    if (K_matrix_.rows != 3 || K_matrix_.cols != 3) {
        ROS_ERROR("[CalibrationValidator] Invalid camera matrix size: %dx%d", K_matrix_.rows, K_matrix_.cols);
        return false;
    }
    
    if (E_matrix_.rows != 4 || E_matrix_.cols != 4) {
        ROS_ERROR("[CalibrationValidator] Invalid extrinsic matrix size: %dx%d", E_matrix_.rows, E_matrix_.cols);
        return false;
    }
    
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            lidar_to_camera_(i, j) = static_cast<float>(E_matrix_.at<double>(i, j));
        }
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            camera_matrix_(i, j) = static_cast<float>(K_matrix_.at<double>(i, j));
        }
    }
    
    double fx = K_matrix_.at<double>(0,0);
    double fy = K_matrix_.at<double>(1,1);
    double cx = K_matrix_.at<double>(0,2);
    double cy = K_matrix_.at<double>(1,2);

    if (fx <= 0 || fy <= 0 || cx <= 0 || cy <= 0) {
        ROS_ERROR("[CalibrationValidator] Invalid camera intrinsic parameters!");
        return false;
    }

    cv::Mat R = E_matrix_(cv::Rect(0, 0, 3, 3));
    double det = cv::determinant(R);
    if (std::abs(det - 1.0) > 0.1) {
        ROS_WARN("[CalibrationValidator] Rotation matrix determinant = %.3f (should be ~1.0)", det);
    }

    ROS_INFO("[CalibrationValidator] Calibration parameters loaded successfully from: %s", calib_file.c_str());
    ROS_INFO("[CalibrationValidator] Camera intrinsics: fx=%.3f, fy=%.3f, cx=%.3f, cy=%.3f", fx, fy, cx, cy);
    ROS_INFO("[CalibrationValidator] Rotation matrix determinant: %.6f", det);
    ROS_INFO("[CalibrationValidator] Translation: [%.3f, %.3f, %.3f]",
             E_matrix_.at<double>(0,3), E_matrix_.at<double>(1,3), E_matrix_.at<double>(2,3));

    calibration_loaded_ = true;
    return true;
}
void CalibrationValidator::configCallback(lidar_cam_validator::ValidatorConfig& config, uint32_t level) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    if (config.reset_to_defaults && !config_.reset_to_defaults) {
        ROS_INFO("[CalibrationValidator] Resetting all parameters to defaults...");

        config.point_size = 3;
        config.min_depth = 0.5;
        config.max_depth = 50.0;
        config.show_depth_colorbar = true;
        config.show_edge_overlay = false;
        config.show_statistics = true;
        config.enable_downsampling = true;
        config.max_points = 100000;  
        config.filter_by_distance = true;
        config.min_distance = 0.3;
        config.max_distance = 100.0;
        config.enable_metrics = true;
        config.edge_threshold = 50;
        config.edge_distance_threshold = 5.0;
        config.min_points_for_nmi = 100;
        config.reset_to_defaults = false;  

        config_server_->updateConfig(config);

        ROS_INFO("[CalibrationValidator] All parameters reset to defaults!");
    }

    config_ = config;

    if (level == 0) {
        ROS_INFO("[CalibrationValidator] Dynamic reconfigure initialized");
    } else {
        ROS_DEBUG("[CalibrationValidator] Dynamic reconfigure updated");
    }
}
void CalibrationValidator::checkParameterUpdates(const ros::TimerEvent& /*event*/) {
    std::string new_image_topic, new_cloud_topic, new_fused_topic, new_info_topic;
    nh_.param<std::string>("image_topic", new_image_topic, "/image_raw");
    nh_.param<std::string>("cloud_topic", new_cloud_topic, "/cloudpoints");
    nh_.param<std::string>("fused_topic", new_fused_topic, "/validator/fused_image");
    nh_.param<std::string>("info_topic", new_info_topic, "/validator/validation_info");

    bool topics_changed = false;

    if (new_image_topic != image_topic_ || new_cloud_topic != cloud_topic_) {
        ROS_INFO("[CalibrationValidator] Topic configuration changed:");
        ROS_INFO("[CalibrationValidator]   Image: %s -> %s", image_topic_.c_str(), new_image_topic.c_str());
        ROS_INFO("[CalibrationValidator]   Cloud: %s -> %s", cloud_topic_.c_str(), new_cloud_topic.c_str());

        {
            std::lock_guard<std::mutex> lock(data_mutex_);

            if (sync_) {
                sync_.reset();
            }
            if (image_sub_) {
                image_sub_.reset();
            }
            if (cloud_sub_) {
                cloud_sub_.reset();
            }

            image_topic_ = new_image_topic;
            cloud_topic_ = new_cloud_topic;

            image_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, image_topic_, 10));
            cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloud_topic_, 10));

            int queue_size;
            pnh_.param<int>("queue_size", queue_size, 10);
            sync_.reset(new Synchronizer(SyncPolicy(queue_size), *image_sub_, *cloud_sub_));
            sync_->registerCallback(boost::bind(&CalibrationValidator::syncCallback, this, _1, _2));
        }

        topics_changed = true;
    }

    if (new_fused_topic != fused_topic_ || new_info_topic != info_topic_) {
        ROS_INFO("[CalibrationValidator] Output topic configuration changed:");
        ROS_INFO("[CalibrationValidator]   Fused: %s -> %s", fused_topic_.c_str(), new_fused_topic.c_str());
        ROS_INFO("[CalibrationValidator]   Info: %s -> %s", info_topic_.c_str(), new_info_topic.c_str());

        fused_topic_ = new_fused_topic;
        info_topic_ = new_info_topic;

        fused_image_pub_ = it_.advertise(fused_topic_, 1);
        validation_info_pub_ = nh_.advertise<std_msgs::String>(info_topic_, 1);

        topics_changed = true;
    }

    if (topics_changed) {
        ROS_INFO("[CalibrationValidator] Topic reconfiguration completed");
    }
}
void CalibrationValidator::syncCallback(const sensor_msgs::ImageConstPtr& image_msg,
                                       const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    data_received_ = true;
    last_data_time_ = ros::Time::now();

    if (!calibration_loaded_) {
        ROS_WARN_THROTTLE(5.0, "[CalibrationValidator] Calibration not loaded, skipping frame");
        return;
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_image->image.clone();
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);
        
        if (cloud->empty()) {
            ROS_WARN_THROTTLE(2.0, "[CalibrationValidator] Received empty point cloud");
            return;
        }
        
        lidar_cam_validator::ValidatorConfig current_config;
        {
            std::lock_guard<std::mutex> lock(config_mutex_);
            current_config = config_;
        }
        
        if (current_config.enable_downsampling && static_cast<int>(cloud->size()) > current_config.max_points) {
            cloud = downsamplePointCloud(cloud, current_config.max_points);
            ROS_DEBUG_THROTTLE(5.0, "[CalibrationValidator] Downsampled point cloud: %u -> %zu points",
                              cloud_msg->width * cloud_msg->height, cloud->size());
        }
        
        ValidationMetrics metrics;
        cv::Mat fused_image = projectPointsToImageFast(image, cloud, metrics);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        metrics.processing_time_ms = duration.count() / 1000.0;
        
        if (!fused_image.empty()) {
            if (!window_initialized_) {
                cv::namedWindow("LiDAR-Camera Calibration Validator", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
                cv::resizeWindow("LiDAR-Camera Calibration Validator", 1200, 800); 
                window_initialized_ = true;
                ROS_INFO("[CalibrationValidator] GUI window initialized (1200x800, resizable)");
            }

            cv::imshow("LiDAR-Camera Calibration Validator", fused_image);
            cv::waitKey(1);
            
            sensor_msgs::ImagePtr fused_msg = cv_bridge::CvImage(image_msg->header, "bgr8", fused_image).toImageMsg();
            fused_image_pub_.publish(fused_msg);
            
            publishValidationInfo(metrics);
        }
        
        updatePerformanceStats();

    } catch (const std::exception& e) {
        ROS_ERROR("[CalibrationValidator] Error in sync callback: %s", e.what());
    } catch (...) {
        ROS_ERROR("[CalibrationValidator] Unknown error in sync callback");
    }
}
cv::Mat CalibrationValidator::projectPointsToImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                  const cv::Mat& image,
                                                  ValidationMetrics& metrics) {
    cv::Mat result = image.clone();
    std::vector<cv::Point2f> projected_points;
    std::vector<float> depths;

    lidar_cam_validator::ValidatorConfig current_config;
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        current_config = config_;
    }

    float min_depth = std::numeric_limits<float>::max();
    float max_depth = std::numeric_limits<float>::min();
    float depth_sum = 0.0f;
    int valid_depth_count = 0;

    for (const auto& pt : *cloud) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }

        Eigen::Vector4f lidar_point(pt.x, pt.y, pt.z, 1.0f);
        Eigen::Vector4f camera_point = lidar_to_camera_ * lidar_point;

        if (camera_point.z() <= 0.01f) continue;

        float depth = camera_point.z(); 

        if (current_config.filter_by_distance) {
            if (depth < current_config.min_distance || depth > current_config.max_distance) {
                continue;
            }
        }

        min_depth = std::min(min_depth, depth);
        max_depth = std::max(max_depth, depth);
        depth_sum += depth;
        valid_depth_count++;
    }

    float display_min_depth = current_config.min_depth;
    float display_max_depth = current_config.max_depth;

    if (valid_depth_count > 0) {
        metrics.min_depth = min_depth;
        metrics.max_depth = max_depth;
        metrics.mean_depth = depth_sum / valid_depth_count;
    }

    int width = image.cols;
    int height = image.rows;

    for (const auto& pt : *cloud) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }

        Eigen::Vector4f lidar_point(pt.x, pt.y, pt.z, 1.0f);
        Eigen::Vector4f camera_point = lidar_to_camera_ * lidar_point;

        if (camera_point.z() <= 0.01f) continue;

        float depth = camera_point.z();

        if (current_config.filter_by_distance) {
            if (depth < current_config.min_distance || depth > current_config.max_distance) {
                continue;
            }
        }

        float u_norm = camera_point.x() / camera_point.z();
        float v_norm = camera_point.y() / camera_point.z();

        cv::Point2f corrected_point = applyDistortionCorrection(u_norm, v_norm);
        float u = corrected_point.x;
        float v = corrected_point.y;

        int u_int = static_cast<int>(std::round(u));
        int v_int = static_cast<int>(std::round(v));

        if (u_int >= 0 && u_int < width && v_int >= 0 && v_int < height) {
            projected_points.push_back(cv::Point2f(u, v));
            depths.push_back(depth);

            cv::Scalar color = depthToColor(depth, display_min_depth, display_max_depth);
            cv::circle(result, cv::Point(u_int, v_int), current_config.point_size, color, -1);
        }
    }

    metrics.valid_projections = projected_points.size();
    metrics.total_points = cloud->size();
    metrics.projection_ratio = metrics.total_points > 0 ?
        static_cast<double>(metrics.valid_projections) / metrics.total_points : 0.0;

    if (current_config.enable_metrics && !projected_points.empty()) {
        ValidationMetrics detailed_metrics = calculateValidationMetrics(image, cloud, projected_points, depths);
        metrics.edge_overlap_score = detailed_metrics.edge_overlap_score;
        metrics.normalized_mutual_info = detailed_metrics.normalized_mutual_info;
        metrics.mean_edge_distance = detailed_metrics.mean_edge_distance;
    }

    if (current_config.show_statistics) {
        drawStatistics(result, metrics);
    }

    if (current_config.show_edge_overlay) {
        drawEdgeOverlay(result);
    }

    if (current_config.show_depth_colorbar) {
        drawDepthColorbar(result, display_min_depth, display_max_depth);
    }

    return result;
}
cv::Scalar CalibrationValidator::depthToColor(float depth, float min_depth, float max_depth) {
    depth = std::max(min_depth, std::min(depth, max_depth));
    float normalized_depth = (max_depth - depth) / (max_depth - min_depth);
    float hue = 240.0f * normalized_depth; 

    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255));
    cv::Mat bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    cv::Vec3b color = bgr.at<cv::Vec3b>(0, 0);

    return cv::Scalar(color[0], color[1], color[2]);
}
cv::Point2f CalibrationValidator::applyDistortionCorrection(float u_norm, float v_norm) {

    if (D_coeffs_.empty() || D_coeffs_.rows < 4) {
        double fx = K_matrix_.at<double>(0, 0);
        double fy = K_matrix_.at<double>(1, 1);
        double cx = K_matrix_.at<double>(0, 2);
        double cy = K_matrix_.at<double>(1, 2);
        return cv::Point2f(u_norm * fx + cx, v_norm * fy + cy);
    }

    double k1 = D_coeffs_.at<double>(0, 0);
    double k2 = D_coeffs_.at<double>(0, 1);
    double p1 = D_coeffs_.at<double>(0, 2);
    double p2 = D_coeffs_.at<double>(0, 3);
    double k3 = (D_coeffs_.cols > 4) ? D_coeffs_.at<double>(0, 4) : 0.0;

    double r2 = u_norm * u_norm + v_norm * v_norm;
    double r4 = r2 * r2;
    double r6 = r4 * r2;
    double radial_distortion = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;

    double tangential_u = 2.0 * p1 * u_norm * v_norm + p2 * (r2 + 2.0 * u_norm * u_norm);
    double tangential_v = p1 * (r2 + 2.0 * v_norm * v_norm) + 2.0 * p2 * u_norm * v_norm;

    double u_distorted = u_norm * radial_distortion + tangential_u;
    double v_distorted = v_norm * radial_distortion + tangential_v;

    double fx = K_matrix_.at<double>(0, 0);
    double fy = K_matrix_.at<double>(1, 1);
    double cx = K_matrix_.at<double>(0, 2);
    double cy = K_matrix_.at<double>(1, 2);

    return cv::Point2f(u_distorted * fx + cx, v_distorted * fy + cy);
}
pcl::PointCloud<pcl::PointXYZ>::Ptr CalibrationValidator::downsamplePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int max_points) {

    if (static_cast<int>(cloud->size()) <= max_points) {
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::RandomSample<pcl::PointXYZ> random_sample;
    random_sample.setInputCloud(cloud);
    random_sample.setSample(max_points);
    random_sample.filter(*downsampled);

    return downsampled;
}
ValidationMetrics CalibrationValidator::calculateValidationMetrics(
    const cv::Mat& image,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& /*cloud*/,
    const std::vector<cv::Point2f>& projected_points,
    const std::vector<float>& depths) {

    ValidationMetrics metrics;

    if (projected_points.empty()) {
        return metrics;
    }

    lidar_cam_validator::ValidatorConfig current_config;
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        current_config = config_;
    }

    if (projected_points.size() >= 10) {  
        metrics.edge_overlap_score = calculateEdgeOverlapScore(image, projected_points);
    }

    if (static_cast<int>(projected_points.size()) >= current_config.min_points_for_nmi) {
        metrics.normalized_mutual_info = calculateNormalizedMutualInfo(image, projected_points, depths);
    }

    return metrics;
}
double CalibrationValidator::calculateEdgeOverlapScore(const cv::Mat& image,
                                                      const std::vector<cv::Point2f>& projected_points) {
    lidar_cam_validator::ValidatorConfig current_config;
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        current_config = config_;
    }

    cv::Mat gray, edges;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::Canny(gray, edges, current_config.edge_threshold, current_config.edge_threshold * 2);

    cv::Mat dist_transform;
    cv::distanceTransform(~edges, dist_transform, cv::DIST_L2, 3);

    int close_to_edge = 0;
    double total_distance = 0.0;

    for (const auto& point : projected_points) {
        int x = static_cast<int>(point.x);
        int y = static_cast<int>(point.y);

        if (x >= 0 && x < dist_transform.cols && y >= 0 && y < dist_transform.rows) {
            double distance = dist_transform.at<float>(y, x);
            total_distance += distance;

            if (distance <= current_config.edge_distance_threshold) {
                close_to_edge++;
            }
        }
    }

    return projected_points.empty() ? 0.0 : static_cast<double>(close_to_edge) / projected_points.size();
}
double CalibrationValidator::calculateNormalizedMutualInfo(const cv::Mat& image,
                                                          const std::vector<cv::Point2f>& projected_points,
                                                          const std::vector<float>& depths) {
    if (projected_points.size() < 50 || depths.size() != projected_points.size()) {
        return 0.0;
    }

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    std::vector<double> intensities, depth_values;

    for (size_t i = 0; i < projected_points.size(); ++i) {
        const cv::Point2f& proj = projected_points[i];
        int x = static_cast<int>(proj.x);
        int y = static_cast<int>(proj.y);

        if (x >= 0 && x < gray.cols && y >= 0 && y < gray.rows) {
            intensities.push_back(gray.at<uchar>(y, x));
            depth_values.push_back(depths[i]);
        }
    }

    if (intensities.size() < 50) return 0.0;

    double mean_intensity = 0.0, mean_depth = 0.0;
    for (size_t i = 0; i < intensities.size(); ++i) {
        mean_intensity += intensities[i];
        mean_depth += depth_values[i];
    }
    mean_intensity /= intensities.size();
    mean_depth /= depth_values.size();

    double numerator = 0.0, denom_intensity = 0.0, denom_depth = 0.0;
    for (size_t i = 0; i < intensities.size(); ++i) {
        double diff_intensity = intensities[i] - mean_intensity;
        double diff_depth = depth_values[i] - mean_depth;

        numerator += diff_intensity * diff_depth;
        denom_intensity += diff_intensity * diff_intensity;
        denom_depth += diff_depth * diff_depth;
    }

    if (denom_intensity == 0.0 || denom_depth == 0.0) return 0.0;

    double correlation = numerator / sqrt(denom_intensity * denom_depth);
    return std::abs(correlation);  
}
void CalibrationValidator::drawStatistics(cv::Mat& image, const ValidationMetrics& metrics) {
    int y_offset = 30;
    cv::Scalar text_color(0, 255, 0);  
    cv::Scalar bg_color(0, 0, 0);      
    double font_scale = 0.7;
    int thickness = 2;
    int line_height = 25;

    cv::Rect bg_rect(5, 5, 450, 200);
    cv::Mat overlay;
    image.copyTo(overlay);
    cv::rectangle(overlay, bg_rect, bg_color, -1);
    cv::addWeighted(image, 0.7, overlay, 0.3, 0, image);

    cv::putText(image, cv::format("Projection: %d/%d points (%.1f%%)",
                                 metrics.valid_projections, metrics.total_points,
                                 metrics.projection_ratio * 100.0),
                cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
    y_offset += line_height;

    cv::putText(image, cv::format("Edge Overlap Score: %.3f", metrics.edge_overlap_score),
                cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
    y_offset += line_height;

    cv::putText(image, cv::format("Normalized MI: %.3f", metrics.normalized_mutual_info),
                cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
    y_offset += line_height;

    if (metrics.total_points > 0) {
        cv::putText(image, cv::format("Depth Range: %.1f - %.1f m", metrics.min_depth, metrics.max_depth),
                    cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
        y_offset += line_height;

        cv::putText(image, cv::format("Mean Depth: %.1f m", metrics.mean_depth),
                    cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
        y_offset += line_height;
    }

    cv::putText(image, cv::format("Processing: %.1f ms (%.1f FPS)",
                                 metrics.processing_time_ms, processing_fps_),
                cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
    y_offset += line_height;

    std::string quality_text;
    cv::Scalar quality_color = text_color;

    if (metrics.edge_overlap_score > 0.7 && metrics.normalized_mutual_info > 0.3) {
        quality_text = "Calibration Quality: GOOD";
        quality_color = cv::Scalar(0, 255, 0);  
    } else if (metrics.edge_overlap_score > 0.4 && metrics.normalized_mutual_info > 0.1) {
        quality_text = "Calibration Quality: FAIR";
        quality_color = cv::Scalar(0, 255, 255);  
    } else if (metrics.valid_projections > 0) {
        quality_text = "Calibration Quality: POOR";
        quality_color = cv::Scalar(0, 0, 255);  
    } else {
        quality_text = "Calibration Quality: NO DATA";
        quality_color = cv::Scalar(128, 128, 128);  
    }

    cv::putText(image, quality_text, cv::Point(10, y_offset),
                cv::FONT_HERSHEY_SIMPLEX, font_scale, quality_color, thickness);
}
void CalibrationValidator::drawEdgeOverlay(cv::Mat& image) {
    lidar_cam_validator::ValidatorConfig current_config;
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        current_config = config_;
    }

    cv::Mat gray, edges;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::Canny(gray, edges, current_config.edge_threshold, current_config.edge_threshold * 2);

    std::vector<cv::Mat> channels;
    cv::split(image, channels);
    cv::bitwise_or(channels[1], edges, channels[1]);  
    cv::merge(channels, image);
}
void CalibrationValidator::drawDepthColorbar(cv::Mat& image, float min_depth, float max_depth) {
    int legend_width = 250;  
    int legend_height = 25;  
    int legend_x = image.cols - legend_width - 25;  
    int legend_y = 25; 

    cv::rectangle(image, cv::Point(legend_x - 8, legend_y - 8),
                  cv::Point(legend_x + legend_width + 8, legend_y + legend_height + 30),
                  cv::Scalar(0, 0, 0), -1);

    for (int i = 0; i < legend_width; i++) {
        float depth_ratio = static_cast<float>(i) / legend_width;
        float depth = max_depth - depth_ratio * (max_depth - min_depth);
        cv::Scalar color = depthToColor(depth, min_depth, max_depth);
        cv::line(image, cv::Point(legend_x + i, legend_y),
                cv::Point(legend_x + i, legend_y + legend_height), color, 1);
    }

    cv::putText(image, cv::format("%.1fm", min_depth),
                cv::Point(legend_x + legend_width - 35, legend_y + legend_height + 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1);

    cv::putText(image, cv::format("%.1fm", max_depth),
                cv::Point(legend_x, legend_y + legend_height + 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1);

    cv::putText(image, "Depth",
                cv::Point(legend_x + legend_width/2 - 25, legend_y - 8),
                cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255), 1);
}
void CalibrationValidator::publishValidationInfo(const ValidationMetrics& metrics) {
    std::ostringstream json_stream;
    json_stream << "{"
                << "\"timestamp\":" << ros::Time::now().toSec() << ","
                << "\"projection_ratio\":" << metrics.projection_ratio << ","
                << "\"edge_overlap_score\":" << metrics.edge_overlap_score << ","
                << "\"normalized_mutual_info\":" << metrics.normalized_mutual_info << ","
                << "\"valid_projections\":" << metrics.valid_projections << ","
                << "\"total_points\":" << metrics.total_points << ","
                << "\"processing_time_ms\":" << metrics.processing_time_ms << ","
                << "\"min_depth\":" << metrics.min_depth << ","
                << "\"max_depth\":" << metrics.max_depth << ","
                << "\"mean_depth\":" << metrics.mean_depth << ","
                << "\"processing_fps\":" << processing_fps_
                << "}";

    std_msgs::String info_msg;
    info_msg.data = json_stream.str();
    validation_info_pub_.publish(info_msg);
}
void CalibrationValidator::updatePerformanceStats() {
    frame_count_++;

    auto current_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_process_time_);

    if (duration.count() >= 1000) { 
        processing_fps_ = static_cast<double>(frame_count_) / (duration.count() / 1000.0);
        frame_count_ = 0;
        last_process_time_ = current_time;

        ROS_DEBUG_THROTTLE(5.0, "[CalibrationValidator] Processing FPS: %.1f", processing_fps_);
    }
}
cv::Mat CalibrationValidator::projectPointsToImageFast(const cv::Mat& image,
                                                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                      ValidationMetrics& metrics) {
    if (!cloud || cloud->empty()) {
        return image.clone();
    }

    cv::Mat result = image.clone();
    int width = image.cols;
    int height = image.rows;

    lidar_cam_validator::ValidatorConfig current_config;
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        current_config = config_;
    }

    projected_points_cache_.clear();
    depths_cache_.clear();
    colors_cache_.clear();

    size_t cloud_size = cloud->size();
    projected_points_cache_.reserve(cloud_size);
    depths_cache_.reserve(cloud_size);
    colors_cache_.reserve(cloud_size);
    if (use_parallel_processing_ && static_cast<int>(cloud_size) > PARALLEL_PROCESSING_THRESHOLD) {
        int num_threads = std::thread::hardware_concurrency();
        if (num_threads == 0) num_threads = 4;

        int batch_size = cloud_size / num_threads;
        std::vector<std::future<void>> futures;

        std::vector<std::vector<cv::Point2f>> thread_points(num_threads);
        std::vector<std::vector<float>> thread_depths(num_threads);
        std::vector<std::vector<cv::Scalar>> thread_colors(num_threads);

        float min_distance = current_config.min_distance;
        float max_distance = current_config.max_distance;

        for (int i = 0; i < num_threads; ++i) {
            int start_idx = i * batch_size;
            int end_idx = (i == num_threads - 1) ? cloud_size : (i + 1) * batch_size;

            futures.push_back(std::async(std::launch::async, [=, &thread_points, &thread_depths, &thread_colors]() {
                processPointBatch(cloud, start_idx, end_idx, width, height,
                                min_distance, max_distance,
                                thread_points[i], thread_depths[i], thread_colors[i]);
            }));
        }

        for (auto& future : futures) {
            future.wait();
        }

        for (int i = 0; i < num_threads; ++i) {
            projected_points_cache_.insert(projected_points_cache_.end(),
                                         thread_points[i].begin(), thread_points[i].end());
            depths_cache_.insert(depths_cache_.end(),
                                thread_depths[i].begin(), thread_depths[i].end());
            colors_cache_.insert(colors_cache_.end(),
                                thread_colors[i].begin(), thread_colors[i].end());
        }
    } else {
        processPointBatch(cloud, 0, cloud_size, width, height,
                         current_config.min_distance, current_config.max_distance,
                         projected_points_cache_, depths_cache_, colors_cache_);
    }

    for (size_t i = 0; i < projected_points_cache_.size(); ++i) {
        cv::circle(result, projected_points_cache_[i], current_config.point_size, colors_cache_[i], -1);
    }

    metrics.valid_projections = projected_points_cache_.size();
    metrics.total_points = cloud->size();
    metrics.projection_ratio = metrics.total_points > 0 ?
        static_cast<double>(metrics.valid_projections) / metrics.total_points : 0.0;

    if (!depths_cache_.empty()) {
        auto minmax = std::minmax_element(depths_cache_.begin(), depths_cache_.end());
        metrics.min_depth = *minmax.first;
        metrics.max_depth = *minmax.second;

        double sum = std::accumulate(depths_cache_.begin(), depths_cache_.end(), 0.0);
        metrics.mean_depth = sum / depths_cache_.size();
    }

    if (current_config.enable_metrics && !projected_points_cache_.empty()) {
        ValidationMetrics detailed_metrics = calculateValidationMetrics(image, cloud, projected_points_cache_, depths_cache_);
        metrics.edge_overlap_score = detailed_metrics.edge_overlap_score;
        metrics.normalized_mutual_info = detailed_metrics.normalized_mutual_info;
        metrics.mean_edge_distance = detailed_metrics.mean_edge_distance;
    }

    if (current_config.show_statistics) {
        drawStatistics(result, metrics);
    }

    if (current_config.show_edge_overlay) {
        drawEdgeOverlay(result);
    }

    if (current_config.show_depth_colorbar) {
        drawDepthColorbar(result, current_config.min_depth, current_config.max_depth);
    }

    return result;
}
void CalibrationValidator::processPointBatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                            int start_idx, int end_idx,
                                            int width, int height,
                                            float min_distance, float max_distance,
                                            std::vector<cv::Point2f>& points,
                                            std::vector<float>& depths,
                                            std::vector<cv::Scalar>& colors) {
    int batch_size = end_idx - start_idx;
    points.reserve(batch_size);
    depths.reserve(batch_size);
    colors.reserve(batch_size);

    float min_depth, max_depth;
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        min_depth = config_.min_depth;
        max_depth = config_.max_depth;
    }

    for (int i = start_idx; i < end_idx; ++i) {
        const auto& point = cloud->points[i];

        float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (distance < min_distance || distance > max_distance) {
            continue;
        }

        Eigen::Vector4f lidar_point(point.x, point.y, point.z, 1.0f);
        Eigen::Vector4f camera_point = lidar_to_camera_ * lidar_point;

        if (camera_point.z() <= 0.01f) {
            continue;
        }

        float u_norm = camera_point.x() / camera_point.z();
        float v_norm = camera_point.y() / camera_point.z();

        cv::Point2f corrected_point = applyDistortionCorrection(u_norm, v_norm);
        float u = corrected_point.x;
        float v = corrected_point.y;

        int u_int = static_cast<int>(std::round(u));
        int v_int = static_cast<int>(std::round(v));

        if (u_int >= 0 && u_int < width && v_int >= 0 && v_int < height) {
            points.emplace_back(u, v);
            depths.push_back(camera_point.z());
            colors.push_back(depthToColor(camera_point.z(), min_depth, max_depth));
        }
    }
}

} // namespace lidar_cam_validator

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
