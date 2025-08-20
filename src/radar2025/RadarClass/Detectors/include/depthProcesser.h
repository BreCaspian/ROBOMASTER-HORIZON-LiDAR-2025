#ifndef __DEPTHPROCESSER_H
#define __DEPTHPROCESSER_H

#include "../../Common/include/public.h"
#include <mutex>
#include <thread>
#include <atomic>
#include <future>
#include <condition_variable>

// YaoYuzhuo 2025-04-17  16:38--完全实现点云分块处理，不再依赖于点云降采样

/**
 * @brief 深度信息处理类
 * 处理雷达点云信息为相机对应的深度图
 * 支持高效处理极大规模点云数据
 */
class DepthQueue // 深度信息处理类------是整个雷达系统中的关键组件，负责将点云数据转换为深度图，为后续的目标检测和定位提供基础--------------
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<DepthQueue> Ptr;

private:
    bool _initflag = false;
    queue<MatrixXi> processQueue;
    Matrix<float, 3, 3> K_0;
    Matrix<float, 1, 5> C_0;
    Matrix<float, 4, 4> E_0;
    vector<vector<float>> depth;

    // 并行处理相关
    int _num_threads;                       // 并行处理的线程数
    std::atomic<bool> _is_processing;       // 是否正在处理点云
    std::mutex _depth_mutex;                // 保护深度图的互斥锁
    std::mutex _queue_mutex;                // 保护队列的互斥锁
    
    // 标准点云处理函数（小规模点云）
    void processPointCloudStandard(pcl::PointCloud<pcl::PointXYZ> &pc);
    
    // 并行点云处理函数（大规模点云）
    void processPointCloudParallel(pcl::PointCloud<pcl::PointXYZ> &pc);
    
    // 点云分块处理函数
    void processChunk(const pcl::PointCloud<pcl::PointXYZ>::Ptr& chunk, 
                     std::vector<std::vector<float>>& local_depth);
    
    // 处理队列更新函数
    int updateProcessQueue(const Matrix4Xf& pc_Matrix, int original_cols);

public:
    DepthQueue();
    DepthQueue(Matrix<float, 3, 3> &K_0, Matrix<float, 1, 5> &C_0, Matrix<float, 4, 4> &E_0);
    ~DepthQueue();

    int getProcessQueueSize() { return processQueue.size(); };
    
    const Matrix<float, 3, 3>& getK0() const { return K_0; }
    
    const Matrix<float, 4, 4>& getE0() const { return E_0; }

    vector<vector<float>> pushback(pcl::PointCloud<pcl::PointXYZ> &pc);
};

#endif
