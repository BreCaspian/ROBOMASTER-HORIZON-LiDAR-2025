#include "../include/PointCloudVisualizer.h"

void PointCloudVisualizer::setParameters(const Matrix<float, 3, 3>& K, const Matrix<float, 4, 4>& E) {
    _K = K;
    _E = E;
    if (logger) {
        logger->debug("PointCloudVisualizer: Camera parameters set");
    }
}

void PointCloudVisualizer::setEnabled(bool enabled) {
    if (_enabled != enabled) {
        _enabled = enabled;
        if (logger) {
            logger->info("PointCloud visualization {}", _enabled ? "enabled" : "disabled");
        }
    } else {
        _enabled = enabled;
    }
}

void PointCloudVisualizer::setPointSize(int size) {
    if (size > 0) {
        _point_size = size;
    } else {
        if (logger) {
            logger->warn("Invalid point size ({}), using default value (1)", size);
        }
        _point_size = 1;
    }
}

void PointCloudVisualizer::setShowInfo(bool show) {
    _show_info = show;
}

void PointCloudVisualizer::setDepthRange(float min_depth, float max_depth) {
    if (min_depth >= max_depth) {
        logger->warn("Invalid depth range: min ({}) must be less than max ({})", min_depth, max_depth);
        return;
    }
    _min_depth = min_depth;
    _max_depth = max_depth;
    _auto_depth_range = false;
}

void PointCloudVisualizer::setAutoDepthRange(bool enabled) {
    _auto_depth_range = enabled;
}

bool PointCloudVisualizer::isEnabled() const {
    return _enabled;
}

cv::Scalar PointCloudVisualizer::depthToColor(float depth) {
    depth = std::max(_min_depth, std::min(depth, _max_depth));
    float normalized_depth = (_max_depth - depth) / (_max_depth - _min_depth);
    float hue = 240.0f * normalized_depth;
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255));
    cv::Mat bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    cv::Vec3b color = bgr.at<cv::Vec3b>(0, 0);
    return cv::Scalar(color[0], color[1], color[2]);
}

void PointCloudVisualizer::visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, cv::Mat& image) {
    if (!_enabled || !cloud || cloud->empty() || image.empty())
        return;
    try {
        int width = image.cols;
        int height = image.rows;
        if (_auto_depth_range) {
            _min_depth = std::numeric_limits<float>::max();
            _max_depth = std::numeric_limits<float>::min();
            for (const auto& pt : *cloud) {
                float depth = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
                if (depth > 0) {
                    _min_depth = std::min(_min_depth, depth);
                    _max_depth = std::max(_max_depth, depth);
                }
            }
            if (_min_depth >= _max_depth) {
                _min_depth = 0.5f;
                _max_depth = 10.0f;
            }
            logger->info("Auto depth range: min={:.2f}m, max={:.2f}m", _min_depth, _max_depth);
        }
        int valid_points = 0;
        int total_points = cloud->size();
        for (const auto& pt : *cloud) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                continue;
            }
            Vector4f point_world(pt.x, pt.y, pt.z, 1);
            Vector4f point_camera = _E * point_world;
            if (point_camera[2] <= 0) {
                continue;
            }
            float depth = sqrt(point_camera[0] * point_camera[0] + 
                              point_camera[1] * point_camera[1] + 
                              point_camera[2] * point_camera[2]);
            Vector3f uvw = _K * Vector3f(point_camera[0], point_camera[1], point_camera[2]);
            if (uvw[2] == 0) {
                continue;
            }
            int u = static_cast<int>(uvw[0] / uvw[2]);
            int v = static_cast<int>(uvw[1] / uvw[2]);
            if (u >= 0 && u < width && v >= 0 && v < height) {
                valid_points++;
                cv::Scalar color = depthToColor(depth);
                cv::circle(image, cv::Point(u, v), _point_size, color, -1);
            }
        }
        if (_show_info && total_points > 0) {
            std::string info = cv::format("Point Cloud: %d/%d points (%.1f%%)", 
                                         valid_points, total_points, 
                                         100.0f * valid_points / total_points);
            cv::putText(image, info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                       0.8, cv::Scalar(0, 255, 0), 2);
            cv::putText(image, cv::format("Depth: %.1f-%.1f m", _min_depth, _max_depth), 
                       cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 
                       0.8, cv::Scalar(0, 255, 0), 2);
            int legend_width = 150;
            int legend_height = 30;
            int legend_x = 10;
            int legend_y = 80;
            for (int i = 0; i < legend_width; i++) {
                float depth_ratio = (float)i / legend_width;
                float depth = _max_depth - depth_ratio * (_max_depth - _min_depth);
                cv::Scalar color = depthToColor(depth);
                cv::line(image, cv::Point(legend_x + i, legend_y), 
                        cv::Point(legend_x + i, legend_y + legend_height), 
                        color, 1);
            }
            cv::putText(image, cv::format("%.1fm", _min_depth), 
                       cv::Point(legend_x + legend_width + 5, legend_y + 20), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);
            cv::putText(image, cv::format("%.1fm", _max_depth), 
                       cv::Point(legend_x - 35, legend_y + 20), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);
        }
    } catch (const std::exception& e) {
        if (logger) {
            logger->error("PointCloudVisualizer exception: {}", e.what());
        }
    }
}
