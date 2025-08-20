#ifndef __POINT_CLOUD_VISUALIZER_H
#define __POINT_CLOUD_VISUALIZER_H

#include "../../Common/include/public.h"

class PointCloudVisualizer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<PointCloudVisualizer> Ptr;

private:
    bool _enabled = false;
    int _point_size = 1;
    bool _show_info = true;
    
    float _min_depth = 0.5f;
    float _max_depth = 10.0f;
    bool _auto_depth_range = true;
    
    Matrix<float, 3, 3> _K;
    Matrix<float, 4, 4> _E;
    
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");
    
    cv::Scalar depthToColor(float depth);
    
public:
    PointCloudVisualizer() = default;
    
    void setParameters(const Matrix<float, 3, 3>& K, const Matrix<float, 4, 4>& E);
    void setEnabled(bool enabled);
    void setPointSize(int size);
    void setShowInfo(bool show);
    void setDepthRange(float min_depth, float max_depth);
    void setAutoDepthRange(bool enabled);
    bool isEnabled() const;
    void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, cv::Mat& image);
};

#endif
