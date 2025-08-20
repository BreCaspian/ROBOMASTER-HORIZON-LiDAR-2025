#include "../include/depthProcesser.h"
#include <spdlog/spdlog.h>
#include <chrono>
#include <thread>
#include <vector>
#include <atomic>
#include <functional>

// YaoYuzhuo 2025-04-17  16:38--完全实现点云分块处理，不再依赖于点云降采样

DepthQueue::DepthQueue() 
{
}

DepthQueue::DepthQueue(Matrix<float, 3, 3> &K_0, Matrix<float, 1, 5> &C_0, Matrix<float, 4, 4> &E_0)
    : K_0(K_0), C_0(C_0), E_0(E_0), _is_processing(false)
{
    vector<float> tmp(ImageW, 0.);
    this->depth.resize(ImageH, tmp);
    
    // 根据系统CPU核心数确定线程数，但不超过16个线程
    _num_threads = std::min(16, static_cast<int>(std::thread::hardware_concurrency()));
    // 确保至少有2个线程
    _num_threads = std::max(2, _num_threads);
    
    auto logger = spdlog::get("RadarLogger");
    if (logger) {
        logger->info("DepthQueue initialized with {} parallel processing threads", _num_threads);
        logger->info("Max points per chunk: {}", MaxPointsNum);
    }
}

DepthQueue::~DepthQueue()
{
    _is_processing = false;

    auto logger = spdlog::get("RadarLogger");
    if (logger) {
        logger->debug("DepthQueue destructor called, cleaning up resources");
    }
}

vector<vector<float>> DepthQueue::pushback(pcl::PointCloud<pcl::PointXYZ> &pc)
{
    static auto last_log_time = std::chrono::steady_clock::now();
    static const double LOG_INTERVAL = 5.0; 
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = current_time - last_log_time;
    bool should_log = (elapsed.count() >= LOG_INTERVAL);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    auto logger = spdlog::get("RadarLogger");
    
    if (pc.empty()) {
        if (logger && should_log) logger->warn("Received empty point cloud");
        return this->depth;
    }

    if (this->processQueue.empty()) {
        this->_initflag = true;
    }
    
    size_t total_points = pc.size();
    
    if (logger) {
        if (should_log || total_points > 100000) {
            logger->info("Processing point cloud with {} points", total_points);
        }
    }
    
    try {
        if (total_points <= 50000) {
            processPointCloudStandard(pc);
        } else {
            processPointCloudParallel(pc);
        }
        
        Matrix4Xf pc_Matrix = pc.getMatrixXfMap();
        int original_cols = std::min(static_cast<int>(pc_Matrix.cols()), MaxPointsNum);
        int cleared_points = updateProcessQueue(pc_Matrix, original_cols);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> total_time = end_time - start_time;
        
        if (logger && should_log) {
            logger->info("Point cloud processing completed in {:.2f} ms", total_time.count());
            logger->debug("Removed {} old points from queue", cleared_points);
            last_log_time = current_time;
        }
    } catch (const std::exception& e) {
        if (logger) {
            logger->error("Exception in DepthQueue::pushback: {}", e.what());
        }
    }
    
    return this->depth;
}

void DepthQueue::processPointCloudStandard(pcl::PointCloud<pcl::PointXYZ> &pc)
{
    static auto last_log_time = std::chrono::steady_clock::now();
    static const double LOG_INTERVAL = 5.0;
    
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = current_time - last_log_time;
    bool should_log = (elapsed.count() >= LOG_INTERVAL);
    
    auto logger = spdlog::get("RadarLogger");
    auto process_start = std::chrono::high_resolution_clock::now();
    
    Matrix4Xf pc_Matrix = pc.getMatrixXfMap();
    int cols = static_cast<int>(pc_Matrix.cols());
    
    Matrix3Xf transformed_points = (this->E_0 * pc_Matrix).topRows(3);
    
    Matrix<float, 3, Eigen::Dynamic> pointsBox = transformed_points;
    Matrix<float, 1, Eigen::Dynamic> dptBox = transformed_points.row(2);
    
    Matrix<int, 2, Eigen::Dynamic> ipBox = ((this->K_0 * pointsBox).array().rowwise() * 
                                          (pointsBox.row(2).array().inverse())).topRows(2).matrix().cast<int>();
    
    std::vector<std::vector<float>> temp_depth(ImageH, std::vector<float>(ImageW, 0.0f));
    
    for (int i = 0; i < cols; ++i) {
        int x = ipBox(0, i);
        int y = ipBox(1, i);
        float depth_val = dptBox(0, i);
        
        if (x >= 0 && x < ImageW && y >= 0 && y < ImageH && depth_val > 0) {
            if (temp_depth[y][x] == 0.0f || depth_val < temp_depth[y][x]) {
                temp_depth[y][x] = depth_val;
            }
        }
    }
    
    {
        std::lock_guard<std::mutex> lock(_depth_mutex);
        for (int y = 0; y < ImageH; ++y) {
            for (int x = 0; x < ImageW; ++x) {
                if (temp_depth[y][x] > 0) {
                    if (this->depth[y][x] == 0.0f || temp_depth[y][x] < this->depth[y][x]) {
                        this->depth[y][x] = temp_depth[y][x];
                    }
                }
            }
        }
    }
    
    auto process_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> process_time = process_end - process_start;
    
    if (logger && should_log) {
        logger->debug("Standard processing completed in {:.2f} ms", process_time.count());
        last_log_time = current_time;
    }
}

void DepthQueue::processPointCloudParallel(pcl::PointCloud<pcl::PointXYZ> &pc)
{
    static auto last_log_time = std::chrono::steady_clock::now();
    static const double LOG_INTERVAL = 5.0;
    
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = current_time - last_log_time;
    bool should_log = (elapsed.count() >= LOG_INTERVAL);
    
    auto logger = spdlog::get("RadarLogger");
    auto start_time = std::chrono::high_resolution_clock::now();
    
    size_t total_points = pc.size();
    
    size_t chunk_size = (total_points + _num_threads - 1) / _num_threads;
    chunk_size = std::min(chunk_size, size_t(100000));
    
    int num_chunks = (total_points + chunk_size - 1) / chunk_size;
    
    if (logger && should_log) {
        logger->info("Splitting point cloud into {} chunks (approx. {} points per chunk)", 
                   num_chunks, chunk_size);
    }
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> chunks(num_chunks);
    std::vector<std::vector<std::vector<float>>> chunk_depths(num_chunks);
    
    auto split_start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_chunks; i++) {
        chunks[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
        chunks[i]->reserve(chunk_size);
        
        size_t start_idx = i * chunk_size;
        size_t end_idx = std::min(start_idx + chunk_size, total_points);
        
        for (size_t j = start_idx; j < end_idx; j++) {
            chunks[i]->push_back(pc[j]);
        }
        
        chunk_depths[i].resize(ImageH, std::vector<float>(ImageW, 0.0f));
    }
    
    auto split_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> split_time = split_end - split_start;
    
    if (logger && should_log) {
        logger->debug("Point cloud split completed in {:.2f} ms", split_time.count());
    }
    
    std::vector<std::thread> threads;
    threads.reserve(num_chunks);
    
    auto process_start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_chunks; i++) {
        threads.emplace_back(
            [this, i, &chunks, &chunk_depths]() {
                this->processChunk(chunks[i], chunk_depths[i]);
            }
        );
    }
    
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    auto process_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> process_time = process_end - process_start;
    
    if (logger && should_log) {
        logger->debug("All chunks processed in {:.2f} ms", process_time.count());
    }
    
    auto merge_start = std::chrono::high_resolution_clock::now();
    
    {
        std::lock_guard<std::mutex> lock(_depth_mutex);
        for (const auto& local_depth : chunk_depths) {
            for (int y = 0; y < ImageH; ++y) {
                for (int x = 0; x < ImageW; ++x) {
                    if (local_depth[y][x] > 0) {
                        if (this->depth[y][x] == 0.0f || local_depth[y][x] < this->depth[y][x]) {
                            this->depth[y][x] = local_depth[y][x];
                        }
                    }
                }
            }
        }
    }
    
    auto merge_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> merge_time = merge_end - merge_start;
    
    if (logger && should_log) {
        logger->debug("Depth maps merged in {:.2f} ms", merge_time.count());
        logger->info("Direct processing completed in {:.2f} ms", 
                   std::chrono::duration<double, std::milli>(merge_end - start_time).count());
        
        double process_ms = process_time.count();
        double merge_ms = merge_time.count();
        double split_ms = split_time.count();
        double queue_ms = std::chrono::duration<double, std::milli>(
            start_time - std::chrono::high_resolution_clock::now()).count();
        
        logger->info("Performance breakdown: Process={:.1f}ms, Merge={:.1f}ms, Queue={:.1f}ms",
                   process_ms, merge_ms, queue_ms);
                   
        last_log_time = current_time;
    }
}

void DepthQueue::processChunk(const pcl::PointCloud<pcl::PointXYZ>::Ptr& chunk, 
                             std::vector<std::vector<float>>& local_depth)
{
    auto logger = spdlog::get("RadarLogger");
    
    try {
        if (!chunk || chunk->empty()) {
            return;
        }
        
        Matrix4Xf pc_Matrix = chunk->getMatrixXfMap();
        int cols = static_cast<int>(pc_Matrix.cols());
        
        Matrix3Xf transformed_points = (this->E_0 * pc_Matrix).topRows(3);
        
        Matrix<float, 3, Eigen::Dynamic> pointsBox = transformed_points;
        Matrix<float, 1, Eigen::Dynamic> dptBox = transformed_points.row(2);
        
        Matrix<int, 2, Eigen::Dynamic> ipBox = ((this->K_0 * pointsBox).array().rowwise() * 
                                           (pointsBox.row(2).array().inverse())).topRows(2).matrix().cast<int>();
        
        for (int i = 0; i < cols; ++i) {
            int x = ipBox(0, i);
            int y = ipBox(1, i);
            float depth_val = dptBox(0, i);
            
            if (x >= 0 && x < ImageW && y >= 0 && y < ImageH && depth_val > 0) {
                if (local_depth[y][x] == 0.0f || depth_val < local_depth[y][x]) {
                    local_depth[y][x] = depth_val;
                }
            }
        }
    } catch (const std::exception& e) {
        if (logger) {
            logger->error("Exception in processChunk: {}", e.what());
        }
    }
}

int DepthQueue::updateProcessQueue(const Matrix4Xf& pc_Matrix, int original_cols)
{
    auto logger = spdlog::get("RadarLogger");
    int cleared_points = 0;
    
    std::lock_guard<std::mutex> lock(_queue_mutex);
    
    try {
        Matrix3Xf transformed_points = (this->E_0 * pc_Matrix.leftCols(original_cols)).topRows(3);
        
        MatrixXi ipBox = MatrixXi::Zero(2, MaxPointsNum);
        
        auto projected = ((this->K_0 * transformed_points).array().rowwise() * 
                       (transformed_points.row(2).array().inverse())).topRows(2).matrix().cast<int>();
        
        ipBox.leftCols(original_cols) = projected;
        
        for (int i = 0; i < original_cols; ++i) {
            if (ipBox(0, i) < 0 || ipBox(0, i) >= ImageW) ipBox(0, i) = 0;
            if (ipBox(1, i) < 0 || ipBox(1, i) >= ImageH) ipBox(1, i) = 0;
        }
        
        this->processQueue.push(ipBox);
        
        if (this->processQueue.size() > maxQueueSize) {
            MatrixXi outpoints = this->processQueue.front();
            
            {
                std::lock_guard<std::mutex> depth_lock(_depth_mutex);
                for (int i = 0; i < original_cols; ++i) {
                    int x = outpoints(0, i);
                    int y = outpoints(1, i);
                    
                    if (x >= 0 && x < ImageW && y >= 0 && y < ImageH) {
                        if (this->depth[y][x] != 0.0f) {
                            this->depth[y][x] = 0.0f;
                            cleared_points++;
                        }
                    }
                }
            }
            
            this->processQueue.pop();
        }
    } catch (const std::exception& e) {
        if (logger) {
            logger->error("Exception in updateProcessQueue: {}", e.what());
        }
    }
    
    return cleared_points;
}



// -------------------------------------------------------------------------------------------------------------------------------------------

// 原作者原版代码，完全没有修改

// #include "../include/depthProcesser.h"

// DepthQueue::DepthQueue()
// {
// }

// DepthQueue::DepthQueue(Matrix<float, 3, 3> &K_0, Matrix<float, 1, 5> &C_0, Matrix<float, 4, 4> &E_0)
// {
//     this->K_0 = K_0;
//     this->C_0 = C_0;
//     this->E_0 = E_0;
//     vector<float> tmp(ImageW, 0.);
//     this->depth.resize(ImageH, tmp);
// }

// DepthQueue::~DepthQueue()
// {
// }

// vector<vector<float>> DepthQueue::pushback(pcl::PointCloud<pcl::PointXYZ> &pc)
// {
//     if (this->processQueue.empty())
//     {
//         this->_initflag = true;
//     }
//     Matrix4Xf pc_Matrix = pc.getMatrixXfMap();
//     int cols = pc_Matrix.cols();
//     Matrix3Xf transformed_points = (this->E_0 * pc_Matrix).topRows(3);
//     Matrix<float, 3, MaxPointsNum> pointsBox;
//     Matrix<float, 1, MaxPointsNum> dptBox;
//     Matrix<int, 2, MaxPointsNum> ipBox;
//     pointsBox.leftCols(cols) << transformed_points;
//     dptBox.leftCols(cols) << transformed_points.row(2);
//     ipBox << ((this->K_0 * pointsBox).array().rowwise() * (pointsBox.row(2).array().inverse())).topRows(2).matrix().cast<int>();
//     auto inside_x = (ipBox.row(0).array() >= 0 && ipBox.row(0).array() < ImageW);
//     auto inside_y = (ipBox.row(1).array() >= 0 && ipBox.row(1).array() < ImageH);
//     ipBox.row(0) << (inside_x).select(ipBox.row(0), MatrixXf::Constant(1, MaxPointsNum, 0));
//     ipBox.row(1) << (inside_y).select(ipBox.row(1), MatrixXf::Constant(1, MaxPointsNum, 0));
//     this->processQueue.push(ipBox);
//     if (this->processQueue.size() > maxQueueSize)
//     {
//         Matrix<int, 2, MaxPointsNum> outpoints = this->processQueue.front();
//         for (int i = 0; i < MaxPointsNum; ++i)
//         {
//             if (this->depth[outpoints(1, i)][outpoints(0, i)] == 0.)
//                 continue;
//             this->depth[outpoints(1, i)][outpoints(0, i)] = 0.;
//         }
//         this->processQueue.pop();
//     }
//     for (int i = 0; i < MaxPointsNum; ++i)
//     {
//         if (dptBox(0, i) > 0)
//         {
//             if ((fabs(this->depth[ipBox(1, i)][ipBox(0, i)]) > Epsilon && dptBox(0, i) < this->depth[ipBox(1, i)][ipBox(0, i)]) || fabs(this->depth[ipBox(1, i)][ipBox(0, i)]) < Epsilon)
//                 this->depth[ipBox(1, i)][ipBox(0, i)] = dptBox(0, i);
//         }
//         else
//             break;
//     }
//     return this->depth;
// }
