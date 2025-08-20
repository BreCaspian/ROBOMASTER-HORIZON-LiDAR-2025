#include "../include/Radar.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/uniform_sampling.h>
#include <iomanip>
#include <sys/stat.h>
#include <sys/types.h>
#ifdef _WIN32
#include <direct.h>
#define mkdir(path, mode) _mkdir(path)
#endif

void Radar::armor_filter(vector<bboxAndRect> &pred)
{
    vector<bboxAndRect> results;
    for (int i = 0; i < int(this->mapMapping->_ids.size()); ++i)
    {
        int max_id = 0;
        float max_conf = 0.f;
        for (size_t j = 0; j < pred.size(); ++j)
        {
            if ((int)pred[j].armor.cls == this->ids[i] && pred[j].armor.conf - max_conf > Epsilon)
            {
                max_id = j;
                max_conf = pred[j].armor.conf;
            }
        }
        if (fabs(max_conf) > Epsilon)
            results.emplace_back(pred[max_id]);
    }
    pred.swap(results);
}
void Radar::detectDepth(vector<bboxAndRect> &pred)
{
    if (pred.size() == 0)
        return;
    for (size_t i = 0; i < pred.size(); ++i)
    {
        if (pred[i].armor.x0 > ImageW || pred[i].armor.y0 > ImageH || pred[i].armor.x0 + pred[i].armor.w > ImageW || pred[i].armor.y0 + pred[i].armor.h > ImageH)
            continue;
        vector<float> tempBox;
        float center[2] = {pred[i].armor.x0 + pred[i].armor.w / 2.f, pred[i].armor.y0 + pred[i].armor.h / 2.f};
        for (int j = int(max<float>(center[1] - pred[i].armor.h / 2.f, 0.)); j < int(min<float>(center[1] + pred[i].armor.h / 2.f, ImageH)); ++j)
        {
            for (int k = int(max<float>(center[0] - pred[i].armor.w / 2.f, 0.)); k < int(min<float>(center[0] + pred[i].armor.w / 2.f, ImageW)); ++k)
            {
                if (this->publicDepth[j][k] == 0)
                    continue;
                tempBox.emplace_back(this->publicDepth[j][k]);
            }
        }
        float tempDepth = 0;
        for (const auto &jt : tempBox)
        {
            tempDepth += jt;
        }
        pred[i].armor.depth = tempBox.size() != 0 ? tempDepth / tempBox.size() : 0.;
        this->logger->info("Depth: [CLS] " + to_string(pred[i].armor.cls) + " [Depth] " + to_string(pred[i].armor.depth));
    }
}

void Radar::detectDepth(vector<ArmorBoundingBox> &armors)
{
    if (armors.size() == 0)
        return;
    for (size_t i = 0; i < armors.size(); ++i)
    {
        if (armors[i].x0 > ImageW || armors[i].y0 > ImageH || armors[i].x0 + armors[i].w > ImageW || armors[i].y0 + armors[i].h > ImageH)
            continue;
        int count = 0;
        vector<float> tempBox;
        float center[2] = {armors[i].x0 + armors[i].w / 2.f, armors[i].y0 + armors[i].h / 2.f};
        for (int j = int(max<float>(center[1] - armors[i].h / 2.f, 0.)); j < int(min<float>(center[1] + armors[i].h / 2.f, ImageH)); ++j)
        {
            for (int k = int(max<float>(center[0] - armors[i].w / 2.f, 0.)); k < int(min<float>(center[0] + armors[i].w / 2.f, ImageW)); ++k)
            {
                if (this->publicDepth[j][k] == 0)
                    continue;
                tempBox.emplace_back(this->publicDepth[j][k]);
                ++count;
            }
        }
        int tempNum = 0;
        for (const auto &jt : tempBox)
        {
            tempNum += jt;
        }
        armors[i].depth = count != 0 ? (float)tempNum / (float)count : 0.;
        this->logger->info("Depth: [CLS] " + to_string(armors[i].cls) + " [Depth] " + to_string(armors[i].depth));
    }
}
void Radar::send_judge(judge_message &message)
{
    vector<vector<float>> loc;
    switch (message.task)
    {
    case 1:
        for (int i = 0; i < int(message.loc.size() / 2); ++i)
        {
            vector<float> temp_location;
            temp_location.emplace_back(message.loc[i + this->ENEMY * 6].x);
            temp_location.emplace_back(message.loc[i + this->ENEMY * 6].y);
            loc.emplace_back(temp_location);
        }
        this->myUART->myUARTPasser.push_loc(loc);
        break;

    default:
        break;
    }
}

void Radar::drawBbox(vector<DetectBox> &bboxs, Mat &img)
{
    int lineWidth = (img.cols > 3000) ? 5 : 2;
    
    for (DetectBox &it : bboxs)
    {
        cv::rectangle(img, Rect(it.x1, it.y1, it.x2 - it.x1, it.y2 - it.y1), Scalar(0, 255, 0), lineWidth);
    }
}

void Radar::drawArmorsForDebug(vector<ArmorBoundingBox> &armors, Mat &img)
{
    int lineWidth = (img.cols > 3000) ? 5 : 2;
    double fontScale = (img.cols > 3000) ? 1.8 : 0.7;
    int textThickness = (img.cols > 3000) ? 5 : 1;
    
    for (auto &it : armors)
    {
        Rect temp = Rect(it.x0, it.y0, it.w, it.h);
        cv::rectangle(img, temp, Scalar(255, 255, 0), lineWidth);
        
        stringstream ss;
        ss << it.cls << "[Depth]" << it.depth << "[Conf]" << it.conf;
        cv::putText(img, ss.str(), 
                    Point2i(int(it.x0 + it.w / 2), int(it.y0 + it.h / 2)), 
                    FONT_HERSHEY_SIMPLEX, fontScale, Scalar(255, 0, 255), textThickness);
    }
}

void Radar::drawArmorsForDebug(vector<bboxAndRect> &armors, Mat &img)
{
    
    int lineWidth = (img.cols > 3000) ? 5 : 2;      
    double fontScale = (img.cols > 3000) ? 1.8 : 0.6; 
    int textThickness = (img.cols > 3000) ? 5 : 1;   
    
    for (auto &it : armors)
    {
        Rect temp = Rect(it.armor.x0, it.armor.y0, it.armor.w, it.armor.h);
        cv::rectangle(img, temp, Scalar(0, 255, 0), lineWidth);
        
        
        stringstream ss;
        ss << int(it.armor.cls) << "[" << fixed << setprecision(2) << it.armor.conf << "]";
        
        
        int textY = (temp.y > 20) ? (temp.y - 15) : (temp.y + temp.height + 30);
        
        
        cv::putText(img, ss.str(), cv::Point2i(temp.x, textY), 
                    cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(0, 0, 255), textThickness);
    }
}

Radar::Radar()
{
    this->myFrames.setDepth(FRAME_DEPTH);
    
    
    this->cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->cloud->reserve(1000000);  
    
    
    if (!logger) {
        logger = spdlog::get("RadarLogger");
        if (logger) {
            logger->info("Radar constructor initialized");
        }
    }
}

Radar::~Radar()
{
    if (this->is_alive)
        this->stop();
    this->logger->flush();
}

void Radar::init()
{
    assert(this->nh);
    assert(!share_path.empty());
    if (this->_init_flag)
        return;
    this->logger->info("Initing ...Process");
    if (ENEMY)
        this->logger->critical("YOU ARE RED");
    else
        this->logger->critical("YOU ARE BLUE");
    this->GUI_image_pub_ = this->image_transport->advertise("/radar2025/result_view", 1, true);
    this->pub_locations = this->nh->advertise<radar2025::Locations>("/radar2025/locations", 1, true);
    std::string param_name;
    if (this->nh->searchParam("/radar2025/EnemyType", param_name))
    {
        this->nh->getParam(param_name, this->ENEMY);
    }
    else
    {
        ROS_WARN("Parameter EnemyType not defined");
    }
    if (this->nh->searchParam("/radar2025/CameraParam", param_name))
    {
        this->nh->getParam(param_name, this->CAMERA_PARAM);
    }
    else
    {
        ROS_WARN("Parameter CameraParam not defined");
    }
    if (this->nh->searchParam("/radar2025/AuthPassword", param_name))
    {
        this->nh->getParam(param_name, this->PASSWORD);
    }
    else
    {
        ROS_WARN("Parameter AuthPassword not defined");
    }
    if (this->nh->searchParam("/radar2025/EngineForArmor", param_name))
    {
        this->nh->getParam(param_name, this->EngineForArmor);
    }
    else
    {
        ROS_WARN("Parameter EngineForArmor not defined");
    }
    if (this->nh->searchParam("/radar2025/OnnxForArmor", param_name))
    {
        this->nh->getParam(param_name, this->OnnxForArmor);
    }
    else
    {
        ROS_WARN("Parameter OnnxForArmor not defined");
    }
    if (this->nh->searchParam("/radar2025/CameraConfig", param_name))
    {
        this->nh->getParam(param_name, this->CameraConfig);
    }
    else
    {
        ROS_WARN("Parameter CameraConfig not defined");
    }
    if (this->nh->searchParam("/radar2025/SerialPortName", param_name))
    {
        this->nh->getParam(param_name, this->SerialPortName);
    }
    else
    {
        ROS_WARN("Parameter SerialPortName not defined");
    }

    
    this->RecorderPath = "Record"; 
    if (this->nh->getParam("/radar2025/recorder/path", this->RecorderPath))
    {
        this->logger->info("Video recorder path set to: {}", this->RecorderPath);
    }
    else
    {
        this->logger->info("Using default video recorder path: {}", this->RecorderPath);
    }

#ifdef UsingVideo
    if (this->nh->searchParam("/radar2025/TestVideo", param_name))
    {
        this->nh->getParam(param_name, this->TestVideo);
    }
    else
    {
        ROS_WARN("Parameter TestVideo not defined");
    }
#endif
#if !(defined UsePointCloudSepTarget || defined UseOneLayerInfer)
    if (this->nh->searchParam("/radar2025/EngineForCar", param_name))
    {
        this->nh->getParam(param_name, this->EngineForCar);
    }
    else
    {
        ROS_WARN("Parameter EngineForCar not defined");
    }
    if (this->nh->searchParam("/radar2025/OnnxForCar", param_name))
    {
        this->nh->getParam(param_name, this->OnnxForCar);
    }
    else
    {
        ROS_WARN("Parameter OnnxForCar not defined");
    }
#endif
#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
    if (this->nh->searchParam("/radar2025/EngineForSort", param_name))
    {
        this->nh->getParam(param_name, this->EngineForSort);
    }
    else
    {
        ROS_WARN("Parameter EngineForSort not defined");
    }
    if (this->nh->searchParam("/radar2025/OnnxForSort", param_name))
    {
        this->nh->getParam(param_name, this->OnnxForSort);
    }
    else
    {
        ROS_WARN("Parameter OnnxForSort not defined");
    }
#endif

    Matrix<float, 3, 3> K_0;
    Matrix<float, 1, 5> C_0;
    Matrix<float, 4, 4> E_0;
    if (!read_param(this->K_0_Mat, this->C_0_Mat, this->E_0_Mat, this->share_path + "/params/" + this->CAMERA_PARAM))
    {
        this->logger->error("Can't read CAMERA_PARAM: {}!", this->share_path + "/params/" + this->CAMERA_PARAM);
        return;
    }
    cv2eigen(this->K_0_Mat, K_0);
    cv2eigen(this->C_0_Mat, C_0);
    cv2eigen(this->E_0_Mat, E_0);
    this->depthQueue = std::make_shared<DepthQueue>(K_0, C_0, E_0);
    this->myUART = std::make_shared<UART>(this->ENEMY);
    this->myLocation = std::make_shared<Location>();
    this->videoRecorder = std::make_shared<VideoRecorder>();
    this->mySerial = std::make_shared<MySerial>();
    this->mapMapping = std::make_shared<MapMapping>();
    this->myLocation->decodeMapPoints(this->share_path + "/params/MapMappingPoints.json");

#ifdef UsingVideo
    this->cameraThread = std::make_shared<CameraThread>(this->share_path + "/params/" + this->CameraConfig,
                                                        this->share_path + "/resources/" + this->TestVideo);
#else
    this->cameraThread = std::make_shared<CameraThread>(this->share_path + "/params/" + this->CameraConfig);
#endif

    this->nh->setParam("/radar2025/ExitProgram", false);
    this->nh->setParam("/radar2025/Recorder", true);
    this->LidarListenerBegin();
    this->armorDetector = std::make_shared<ArmorDetector>(this->share_path + "/models/" + this->EngineForArmor,
                                                          this->share_path + "/models/" + this->OnnxForArmor);
    this->armorDetector->accessModelTest();

#if !(defined UsePointCloudSepTarget || defined UseOneLayerInfer)
    this->carDetector = std::make_shared<CarDetector>(this->share_path + "/models/" + this->EngineForCar,
                                                      this->share_path + "/models/" + this->OnnxForCar);
    this->carDetector->accessModelTest();
#endif

    if (!this->armorDetector->initModel())
    {
        this->stop();
        this->logger->flush();
        return;
    }

#if !(defined UsePointCloudSepTarget || defined UseOneLayerInfer)
    if (!this->carDetector->initModel())
    {
        this->stop();
        this->logger->flush();
        return;
    }
#endif
#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
    this->dsTracker = std::make_shared<DsTracker>(this->share_path + "/models/" + this->OnnxForSort,
                                                  this->share_path + "/models/" + this->EngineForSort);
#endif
#ifdef ExperimentalOutput
    this->myExpLog = std::make_shared<ExpLog>();
    this->myExpLog->init(this->share_path + "/ExpResultDir/");
#endif

    
    std::string fullRecordPath = this->share_path + "/" + this->RecorderPath;
    struct stat st = {0};
    if (stat(fullRecordPath.c_str(), &st) == -1) {
        
        if (mkdir(fullRecordPath.c_str(), 0755) == -1) {
            this->logger->error("Failed to create video recording directory: {}", fullRecordPath);
        } else {
            this->logger->info("Created video recording directory: {}", fullRecordPath);
        }
    }

    this->_recorder_block = !this->videoRecorder->init(fullRecordPath.c_str(), 
                                                     VideoWriter::fourcc('m', 'p', '4', 'v'), 
                                                     Size(ImageW, ImageH));
    if (this->_recorder_block) {
        this->logger->warn("Video recorder initialization failed. Recording will be disabled.");
    } else {
        this->logger->info("Video recorder initialized with path: {}", fullRecordPath);
    }
    
    this->cameraThread->start();
    this->_init_flag = true;
    this->logger->info("Init Done");
    this->is_alive = true;
    
    
    this->pointCloudVisualizer.setParameters(K_0, E_0);
    
    this->pointCloudVisualizer.setPointSize(2);  
    this->pointCloudVisualizer.setShowInfo(true); 
    this->pointCloudVisualizer.setAutoDepthRange(true); 
    if (logger) {
        logger->info("Point cloud visualizer initialized with camera parameters");
    }

    

    
    std::string method = "voxel_grid";
    nh->param<bool>("radar2025/downsample/enable", this->enable_downsample_, false);
    nh->param<std::string>("radar2025/downsample/method", method, "voxel_grid");
    nh->param<int>("radar2025/downsample/target_points", this->target_points_, 10000);
    nh->param<float>("radar2025/downsample/voxel_size", this->voxel_size_, 0.01f);
    nh->param<float>("radar2025/downsample/uniform_radius", this->uniform_radius_, 0.01f);
    nh->param<bool>("radar2025/downsample/adaptive_adjust", this->adaptive_adjust_, true);
    nh->param<int>("radar2025/downsample/adjust_count_max", this->adjust_count_max_, 40);
    
    if (method == "voxel_grid") {
        this->downsample_method_ = VOXEL_GRID;
    } else if (method == "random") {
        this->downsample_method_ = RANDOM;
    } else if (method == "uniform") {
        this->downsample_method_ = UNIFORM;
    }
    
    logger->info("Point cloud downsampling {}", this->enable_downsample_ ? "enabled" : "disabled");
    if (this->enable_downsample_) {
        logger->info("Downsample method: {}, target points: {}", method, this->target_points_);
        if (this->adaptive_adjust_) {
            logger->info("Using adaptive adjustment for voxel downsampling");
        } else {
            logger->info("Using fixed adjustment count: {}", this->adjust_count_max_);
        }
    }

    

    
    bool enable_uart_test = false;
    if (this->nh->getParam("/radar2025/UARTTestEnabled", enable_uart_test))
    {
        this->UARTTestEnabled = enable_uart_test;
    }
    
    
    int uart_test_mode = 0;
    int uart_test_interval = 100;
    
    
    if (!this->nh->getParam("/radar2025/uart_test/mode", uart_test_mode))
    {
        
        this->nh->getParam("/radar2025/UARTTestMode", uart_test_mode);
    }
    else
    {
        
        this->nh->setParam("/radar2025/UARTTestMode", uart_test_mode);
    }
    
    if (!this->nh->getParam("/radar2025/uart_test/interval", uart_test_interval))
    {
        
        this->nh->getParam("/radar2025/UARTTestInterval", uart_test_interval);
    }
    else
    {
        
        this->nh->setParam("/radar2025/UARTTestInterval", uart_test_interval);
    }

    
    bool enable_bytetracker = false;
    if (this->nh->getParam("/radar2025/bytetracker/enable", enable_bytetracker))
    {
        this->ByteTrackerEnabled = enable_bytetracker;
    }
    
    this->nh->getParam("/radar2025/bytetracker/frame_rate", this->bytetracker_frame_rate_);
    this->nh->getParam("/radar2025/bytetracker/track_buffer", this->bytetracker_track_buffer_);
    this->nh->getParam("/radar2025/bytetracker/track_thresh", this->bytetracker_track_thresh_);
    this->nh->getParam("/radar2025/bytetracker/high_thresh", this->bytetracker_high_thresh_);
    this->nh->getParam("/radar2025/bytetracker/match_thresh", this->bytetracker_match_thresh_);
    
    
    this->byteTracker = std::make_shared<ByteTrackerWrapper>();
    if (this->ByteTrackerEnabled) {
        if (!this->byteTracker->init(this->bytetracker_frame_rate_,
                                    this->bytetracker_track_buffer_,
                                    this->bytetracker_track_thresh_,
                                    this->bytetracker_high_thresh_,
                                    this->bytetracker_match_thresh_)) {
            this->logger->warn("Failed to initialize ByteTracker, disabling");
            this->ByteTrackerEnabled = false;
        } else {
            this->logger->info("ByteTracker initialized with frame_rate={}, buffer={}, thresh={:.2f}/{:.2f}/{:.2f}", 
                              this->bytetracker_frame_rate_, this->bytetracker_track_buffer_,
                              this->bytetracker_track_thresh_, this->bytetracker_high_thresh_, 
                              this->bytetracker_match_thresh_);
        }
    } else {
        this->logger->info("ByteTracker is disabled by configuration");
    }
    
    
    this->byteTracker->setEnabled(this->ByteTrackerEnabled);
}

void Radar::setRosNodeHandle(ros::NodeHandle &nh)
{
    this->nh = std::make_shared<ros::NodeHandle>(nh);
}

void Radar::setRosImageTransport(image_transport::ImageTransport &image_transport)
{
    this->image_transport = std::make_shared<image_transport::ImageTransport>(image_transport);
}

void Radar::setRosPackageSharedPath(String &path)
{
    this->share_path = path;
}

void Radar::LidarListenerBegin()
{
    assert(this->nh);
    if (this->_is_LidarInited)
        return;
    this->sub_lidar = this->nh->subscribe(lidarTopicName, LidarQueueSize, &Radar::LidarCallBack, this);
    this->_is_LidarInited = true;
    this->logger->info("Lidar inited");
}

void Radar::RosSpinLoop()
{
    while (this->__RosSpinLoop_working)
    {
        if (ros::ok())
            ros::spinOnce();
    }
    ros::shutdown();
    this->logger->critical("RosSpinLoop Exit");
}
void Radar::LidarCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    
    
    static int message_count = 0;
    static bool first_message = true;
    static ros::Time last_log_time = ros::Time::now();
    static ros::Time last_health_check_time = ros::Time::now(); 
    
    
    if (first_message) {
        this->logger->info(">>>>>>>>>>>>>>>>>>>>>>>>>LiDAR Data SUCCESSFULLY connected to Main Program<<<<<<<<<<<<<<<<<<<<<<<<<<");
        first_message = false;
    }
    
    message_count++;
    
    
    {
        unique_lock<shared_timed_mutex> cloud_lock(this->myMutex_cloud);
    
    pcl::fromROSMsg(*msg, *this->cloud);
    }
    
    
    ros::Time current_time = ros::Time::now();
    if ((current_time - last_health_check_time).toSec() > 3.0) {
        if (this->cloud->empty()) {
            this->logger->warn(">>>>>>>>>>>>>>>>>>>>>>>>>LiDAR Health Check: ABNORMAL - Point cloud is empty (message #{})<<<<<<<<<<<<<<<<<<<<<<<<<<", message_count);
        } else {
            
            this->logger->info(">>>>>>>>>>>>>>>>>>>>>>>>>LiDAR Health Check: NORMAL - Point cloud size: {} points<<<<<<<<<<<<<<<<<<<<<<<<<<", this->cloud->size());
        }
        last_health_check_time = current_time;
        
        
        
        #ifdef ShowPointCloud
        bool show_cloud = false;
        if (this->nh->getParam("/gui/ShowPointCloud", show_cloud)) {
            this->pointCloudVisualizer.setEnabled(show_cloud);
        }
        #endif
    }
    
    
    if (this->enable_downsample_) {
        unique_lock<shared_timed_mutex> cloud_lock(this->myMutex_cloud);
        downsamplePointCloud(this->cloud, this->target_points_);
        cloud_lock.unlock();
    }
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>);
    {
        shared_lock<shared_timed_mutex> cloud_lock(this->myMutex_cloud);
        *cloud_copy = *this->cloud;
    }
    
    
    std::vector<std::vector<float>> tempDepth = this->depthQueue->pushback(*cloud_copy);
    
    
    unique_lock<shared_timed_mutex> ulk(this->myMutex_publicDepth);
    this->publicDepth.swap(tempDepth);
    ulk.unlock();
}
void Radar::SerReadLoop()
{
    while (this->_Ser_working)
    {
        this->myUART->read(this->mySerial);
    }
    this->logger->critical("SerReadLoop Exit");
}

void Radar::SerWriteLoop()
{
    while (this->_Ser_working)
    {
        this->myUART->write(this->mySerial);
    }
    this->logger->critical("SerWriteLoop Exit");
}

void Radar::MainProcessLoop()
{
    while (this->__MainProcessLoop_working)
    {
        auto start_t = std::chrono::system_clock::now().time_since_epoch();
        if (!this->cameraThread->is_open())
        {
            this->cameraThread->open();
            continue;
        }
        FrameBag frameBag = this->cameraThread->read();
        if (frameBag.flag)
        {
            
            
            cv::Size originalSize(frameBag.frame.cols, frameBag.frame.rows);
            
            
            cv::Mat displayFrame = frameBag.frame.clone();
            
            
            if (frameBag.frame.cols > 3000 || frameBag.frame.rows > 2000) {
                
                double scale = std::min(2000.0 / frameBag.frame.cols, 2000.0 / frameBag.frame.rows);
                
                
                cv::resize(displayFrame, displayFrame, cv::Size(), scale, scale, cv::INTER_AREA);
                
                
                static bool first_resize = true;
                if (first_resize) {
                    this->logger->info("Creating scaled display image ({}x{} → {}x{}) for visualization ONLY. All processing uses full resolution {}x{}.", 
                        frameBag.frame.cols, frameBag.frame.rows, displayFrame.cols, displayFrame.rows,
                        originalSize.width, originalSize.height);
                    first_resize = false;
                }
            }
            
            if (this->_if_record)
            {
                
                this->myFrames.push(frameBag.frame.clone());
            }

#ifndef UseOneLayerInfer
#ifdef UsePointCloudSepTarget
            shared_lock<shared_timed_mutex> slk_md(this->myMutex_publicDepth);
            vector<Rect> sepTargets = this->movementDetector->applyMovementDetector(this->publicDepth);
            slk_md.unlock();
            vector<bboxAndRect> pred = this->movementDetector->_ifHistoryBuild() ? this->armorDetector.infer(frameBag.frame, sepTargets) : {};
#else
            vector<DetectBox> sepTargets = this->carDetector->infer(frameBag.frame);
#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
            this->dsTracker->sort(frameBag.frame, sepTargets);
#endif
            vector<bboxAndRect> pred = this->armorDetector->infer(frameBag.frame, sepTargets);
#endif
#else
            vector<bboxAndRect> pred = this->armorDetector->infer(frameBag.frame);
#endif

            
            double scale = 1.0;
            if (frameBag.frame.cols > 3000 || frameBag.frame.rows > 2000) {
                scale = std::min(2000.0 / frameBag.frame.cols, 2000.0 / frameBag.frame.rows);
            }
            
#if defined Test && defined TestWithVis
#ifndef UseOneLayerInfer
            
            vector<DetectBox> scaledBoxes;
            for (const auto& box : sepTargets) {
                DetectBox scaledBox;
                scaledBox.x1 = box.x1 * scale;
                scaledBox.y1 = box.y1 * scale;
                scaledBox.x2 = box.x2 * scale;
                scaledBox.y2 = box.y2 * scale;
                scaledBox.confidence = box.confidence;
                scaledBox.classID = box.classID;
                scaledBox.trackID = box.trackID;
                scaledBoxes.push_back(scaledBox);
            }
            this->drawBbox(scaledBoxes, displayFrame);
            if (this->logger && !sepTargets.empty()) {
                this->logger->info("Drawing {} vehicle detection boxes on display frame ({}x{})", 
                                  scaledBoxes.size(), displayFrame.cols, displayFrame.rows);
            }
#endif
            
            vector<bboxAndRect> scaledPred;
            for (const auto& p : pred) {
                bboxAndRect scaledBbox;
                scaledBbox.armor.flag = p.armor.flag;
                scaledBbox.armor.x0 = p.armor.x0 * scale;
                scaledBbox.armor.y0 = p.armor.y0 * scale;
                scaledBbox.armor.w = p.armor.w * scale;
                scaledBbox.armor.h = p.armor.h * scale;
                scaledBbox.armor.cls = p.armor.cls;
                scaledBbox.armor.conf = p.armor.conf;
                scaledBbox.armor.depth = p.armor.depth;
                
                scaledBbox.rect.x1 = p.rect.x1 * scale;
                scaledBbox.rect.y1 = p.rect.y1 * scale;
                scaledBbox.rect.x2 = p.rect.x2 * scale;
                scaledBbox.rect.y2 = p.rect.y2 * scale;
                scaledBbox.rect.confidence = p.rect.confidence;
                scaledBbox.rect.classID = p.rect.classID;
                scaledBbox.rect.trackID = p.rect.trackID;
                
                scaledPred.push_back(scaledBbox);
            }
            this->drawArmorsForDebug(scaledPred, displayFrame);
            if (this->logger && !pred.empty()) {
                this->logger->info("Drawing {} armor detection boxes on display frame ({}x{})", 
                                  scaledPred.size(), displayFrame.cols, displayFrame.rows);
            }
#endif
#ifdef ExperimentalOutput
            int pred_size = pred.size();
            float pred_conf_average = sumConfAverage(pred);
#endif

#ifdef Test
#ifdef ShowDepth
            if (this->_if_coverDepth)
            {
                shared_lock<shared_timed_mutex> slk_pd(this->myMutex_publicDepth);
                if (this->publicDepth.empty()) {
                    if (logger) {
                        logger->warn("Cannot render depth map: publicDepth is empty");
                    }
                } else {
                    
                    float min_depth = std::numeric_limits<float>::max();
                    float max_depth = 0.1f;  
                    int valid_depth_points = 0;
                    
                    
                    for (int i = 0; i < ImageH; ++i) {
                        for (int j = 0; j < ImageW; ++j) {
                            float depth_value = this->publicDepth[i][j];
                            if (depth_value > Epsilon) {
                                min_depth = std::min(min_depth, depth_value);
                                max_depth = std::max(max_depth, depth_value);
                                valid_depth_points++;
                            }
                        }
                    }
                    
                    
                    if (valid_depth_points > DEPTH_VIS_MIN_POINTS) {
                        
                        float depth_scale = 255.0f / (max_depth - min_depth + Epsilon);
                        
                        
                        for (int i = 0; i < ImageH; ++i) {
                    uchar *data = displayFrame.ptr<uchar>(i);
                            for (int j = 0; j < ImageW; ++j) {
                                float depth_value = this->publicDepth[i][j];
                                if (depth_value > Epsilon) {
                                    
                                    float normalized_depth = (depth_value - min_depth) * depth_scale;
                                    
                                    
                                    float hue = 240.0f * (1.0f - normalized_depth / 255.0f);
                                    
                            int b, g, r;
                                    hsv_to_bgr(hue, 255, 255, b, g, r);
                                    
                                    
                                    int offset = j * 3;
                                    data[offset] = b;
                                    data[offset + 1] = g;
                                    data[offset + 2] = r;
                        }
                            }
                        }
                        
                        
                        cv::putText(displayFrame, 
                                   cv::format("Depth range: %.2f - %.2f m", min_depth, max_depth),
                                   cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 
                                   1.0, cv::Scalar(0, 255, 0), 2);
                    } else {
                        cv::putText(displayFrame, 
                                   "Not enough depth points for visualization",
                                   cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 
                                   1.0, cv::Scalar(0, 0, 255), 2);
                    }
                }
                slk_pd.unlock();
            }
#endif

#ifdef ShowPointCloud
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy;
            {
                
                shared_lock<shared_timed_mutex> slk_cloud(this->myMutex_cloud);
            
            
            if (!this->cloud->empty()) {
                cloud_copy.reset(new pcl::PointCloud<pcl::PointXYZ>);
                *cloud_copy = *this->cloud;
                }
                
            }
            
            
            if (cloud_copy && cloud_copy->size() > 0) {
                
                this->pointCloudVisualizer.setEnabled(_if_showPointCloud);
                
                
                if (cloud_copy->size() > MAX_VISUALIZATION_POINTS) {
                    
                    pcl::VoxelGrid<pcl::PointXYZ> vis_filter;
                    vis_filter.setInputCloud(cloud_copy);
                    vis_filter.setLeafSize(VISUALIZATION_VOXEL_SIZE, VISUALIZATION_VOXEL_SIZE, VISUALIZATION_VOXEL_SIZE);
                    
                    pcl::PointCloud<pcl::PointXYZ>::Ptr vis_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    vis_filter.filter(*vis_cloud);
                    
                    
                    this->pointCloudVisualizer.visualizePointCloud(vis_cloud, displayFrame);
                    
                    if (logger) {
                        static bool log_once = true;
                        if (log_once) {
                            logger->info("Downsampling dense point cloud for visualization: {} → {} points", 
                                       cloud_copy->size(), vis_cloud->size());
                            log_once = false;
                        }
                    }
                } else {
                    
                this->pointCloudVisualizer.visualizePointCloud(cloud_copy, displayFrame);
                }
            }
#endif
#endif

            if (pred.size() != 0)
            {
                this->armor_filter(pred);
                
                
                if (this->ByteTrackerEnabled && this->byteTracker && this->byteTracker->isEnabled()) {
                    
                    size_t before_tracking = pred.size();
                    
                    
                    pred = this->byteTracker->track(pred);
                    
                    
                    static int tracking_frame_count = 0;
                    if (++tracking_frame_count % 500 == 0) {
                        auto stats = this->byteTracker->getStats();
                        this->logger->info("ByteTracker performance: Success rate={:.2f}%, Frames={}, Detections={}, Tracks={}",
                                          stats.success_rate * 100.0f, stats.tracks_processed, 
                                          stats.detections_processed, stats.tracks_generated);
                    }
                    
                    this->logger->debug("Using ByteTracker for object tracking, {} objects before, {} objects after tracking", 
                                       before_tracking, pred.size());
                }
                
                shared_lock<shared_timed_mutex> slk_pd(this->myMutex_publicDepth);
                if (this->publicDepth.size() == 0)
                {
                    slk_pd.unlock();
                    this->logger->info("No Lidar Msg , Return");
                }
                else
                {
                    this->detectDepth(pred);
                    
                    
#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
                    this->mapMapping->_DeepSort_prediction(pred, sepTargets);
#else
                    
                    if (this->ByteTrackerEnabled && this->byteTracker && this->byteTracker->isEnabled()) {
                        this->mapMapping->_GenericTracker_prediction(pred, sepTargets);
                    }
#endif

#ifndef UseOneLayerInfer
                    vector<ArmorBoundingBox> IouArmors = this->mapMapping->_IoU_prediction(pred, sepTargets);
#else
                    vector<ArmorBoundingBox> IouArmors = {};
#endif

                    this->detectDepth(IouArmors);
                    slk_pd.unlock();
                    this->mapMapping->mergeUpdata(pred, IouArmors, this->K_0_Mat, this->C_0_Mat);
                    judge_message myJudge_message;
                    myJudge_message.task = 1;
                    myJudge_message.loc = this->mapMapping->getloc();
                    this->send_judge(myJudge_message);
                    if (myJudge_message.loc.size() > 0)
                    {
                        radar2025::Locations locations_msg;
                        for (int i = 0, N = myJudge_message.loc.size(); i < N; ++i)
                        {
                            radar2025::Location location_msg;
                            location_msg.id = myJudge_message.loc[i].id;
                            location_msg.x = myJudge_message.loc[i].x;
                            location_msg.y = myJudge_message.loc[i].y;
                            locations_msg.locations.emplace_back(location_msg);
                        }
                        locations_msg.header = std_msgs::Header();
                        locations_msg.header.frame_id = "radar2025";
                        locations_msg.header.stamp = ros::Time::now();
                        locations_msg.header.seq = 1;
                        this->pub_locations.publish(locations_msg);
                    }
                }
            }
            auto end_t = std::chrono::system_clock::now().time_since_epoch();

#ifdef Test
            char ch[255];
            sprintf(ch, "FPS %d", int(std::chrono::nanoseconds(1000000000).count() / (end_t - start_t).count()));
            std::string fps_str = ch;
            cv::putText(displayFrame, fps_str, {10, 50}, cv::FONT_HERSHEY_SIMPLEX, 2.5, {0, 255, 0}, 4);
            
            
            if (this->cameraThread) {
                float camera_fps = this->cameraThread->getRealtimeFPS();
                sprintf(ch, "Camera FPS: %.1f", camera_fps);
                std::string cam_fps_str = ch;
                cv::putText(displayFrame, cam_fps_str, {10, 120}, cv::FONT_HERSHEY_SIMPLEX, 1.8, {0, 255, 255}, 4);
                
                
                bool isAutoGain = false;
                this->cameraThread->getGainMode(isAutoGain);
                
#ifndef UsingVideo
                
                if (this->cameraThread->is_open()) {
                    float exposure = this->cameraThread->getCurrentExposure();
                    float gain = this->cameraThread->getCurrentGain();
                    sprintf(ch, "Exposure: %.0f us  Gain: %.1f %s", 
                            exposure, gain, isAutoGain ? "(Auto)" : "");
                    std::string param_str = ch;
                    cv::putText(displayFrame, param_str, {10, 180}, cv::FONT_HERSHEY_SIMPLEX, 1.8, {0, 255, 255}, 4);
                }
#endif
            }
            
            this->mapMapping->_plot_region_rect(this->show_region, displayFrame, this->K_0_Mat, this->C_0_Mat);
#endif

            sensor_msgs::ImagePtr image_msg;
            try
            {
                image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", displayFrame).toImageMsg();
                image_msg->header.frame_id = "radar2025";
                image_msg->header.stamp = ros::Time::now();
                image_msg->header.seq = 1;
                this->GUI_image_pub_.publish(image_msg);
            }
            catch (cv_bridge::Exception &e)
            {
                this->logger->error("cv_bridge exception: %s", e.what());
                this->logger->flush();
                continue;
            }
            this->logger->flush();

#ifdef ExperimentalOutput
            std::vector<string> msg;
            msg.emplace_back(to_string(pred.size()));
            msg.emplace_back(to_string(pred_size));
            msg.emplace_back(to_string(sumConfAverage(pred)));
            msg.emplace_back(to_string(pred_conf_average));
            msg.emplace_back(to_string((end_t - start_t).count()));
            this->myExpLog->input(msg);
#endif
        }
        else
            continue;
    }
    this->logger->critical("MainProcessLoop Exit");
}

void Radar::VideoRecorderLoop()
{
    while (this->__VideoRecorderLoop_working)
    {
        if (this->_if_record && this->myFrames.size() > 0 && !this->_recorder_block)
        {
            this->videoRecorder->write(this->myFrames.front());
            this->myFrames.pop();
        }
        else if (!this->_if_record)
        {
            sleep(1);
        }
    }
    this->videoRecorder->close();
    this->_if_record = false;
    this->nh->setParam("/radar2025/Recorder", false);
    this->logger->critical("VideoRecorderLoop Exit");
}

void Radar::spin()
{
    
    this->init();
    assert(depthQueue && armorDetector);

#if !(defined UseOneLayerInfer)
#ifdef UsePointCloudSepTarget
    assert(movementDetector);
#else
    assert(carDetector);
#endif
#endif

    assert(cameraThread && myLocation && mapMapping && myUART && mySerial && videoRecorder);

#ifdef ExperimentalOutput
    assert(myExpLog);
#endif

    if (!this->_init_flag)
        return;

    
    FrameBag testFrame = this->cameraThread->read();
    if (testFrame.flag) {
        if (testFrame.frame.cols != ImageW || testFrame.frame.rows != ImageH) {
            this->logger->warn("Camera resolution ({} x {}) doesn't match config ({} x {}). Performance may be affected.", 
                testFrame.frame.cols, testFrame.frame.rows, ImageW, ImageH);
            
            
            if (testFrame.frame.cols > 3000 || testFrame.frame.rows > 2000) {
                this->logger->info("High resolution camera detected. Processing will be adjusted for optimal performance.");
            }
        }
    }
    
    std::string param_name;
    if (this->nh->searchParam("/radar2025/ExitProgram", param_name))
    {
        this->nh->getParam(param_name, this->ExitProgramSiginal);
    }
    else
    {
        ROS_WARN("Parameter ExitProgram not defined");
    }
    if (this->nh->searchParam("/radar2025/Recorder", param_name))
    {
        this->nh->getParam(param_name, this->VideoRecorderSiginal);
    }
    else
    {
        ROS_WARN("Parameter Recorder not defined");
    }

#ifdef ShowDepth
    if (this->nh->searchParam("/gui/CoverDepth", param_name))
    {
        this->nh->getParam(param_name, this->_if_coverDepth);
    }
    else
    {
        ROS_WARN("Parameter CoverDepth not defined");
    }
#endif

#ifdef ShowPointCloud
    
    if (this->nh->searchParam("/gui/ShowPointCloud", param_name))
    {
        this->nh->getParam(param_name, this->_if_showPointCloud);
        
        this->pointCloudVisualizer.setEnabled(this->_if_showPointCloud);
        
        
        if (this->_if_showPointCloud) {
            this->pointCloudVisualizer.setAutoDepthRange(true);
        }
    }
    else
    {
        ROS_WARN("Parameter ShowPointCloud not defined");
    }
#endif

    
    if (this->nh->searchParam("/gui/UseByteTracker", param_name))
    {
        this->nh->getParam(param_name, this->ByteTrackerEnabled);
        
        
        if (this->byteTracker) {
            this->byteTracker->setEnabled(this->ByteTrackerEnabled);
            if (this->ByteTrackerEnabled) {
              
            } else {
               
            }
        }
    }
    else
    {
        ROS_WARN("Parameter UseByteTracker not defined");
    }

    if (this->ExitProgramSiginal == true)
    {
        this->stop();
        return;
    }
    this->_if_record = VideoRecorderSiginal;
    
    
    if (!this->cameraThread->is_open()) {
        this->logger->info("Camera is not open, attempting to open it...");
        
        
        int retry_count = 0;
        while (retry_count < 3 && !this->cameraThread->is_open()) {
            this->cameraThread->open();
            if (!this->cameraThread->is_open()) {
                this->logger->warn("Failed to open camera (attempt {}/3), retrying...", retry_count + 1);
                retry_count++;
                std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

        if (!this->cameraThread->is_open()) {
            this->logger->error("Failed to open camera after 3 attempts, cannot proceed with calibration");
        return;
    }
        
        
        this->logger->info("Camera opened, waiting for stabilization...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    
    if (!this->mapMapping->_is_pass())
    {
        this->logger->info("Starting calibration process...");
        Mat rvec, tvec;
        
        
        cv::destroyAllWindows();
        cv::waitKey(100);
        
        
            unique_lock<shared_timed_mutex> ulk(myMutex_cameraThread);
        
        bool calibration_success = false;
            try
            {
            
                if (this->myLocation->locate_pick(this->cameraThread, this->ENEMY, rvec, tvec, this->K_0_Mat, this->C_0_Mat, this->E_0_Mat))
                {
                    calibration_success = true;
                    this->mapMapping->push_T(rvec, tvec);
                    
                    
                    stringstream ss_camera_intrinsics, ss_camera_distortion, ss_camera_lidar_extrinsics;
                    
                    ss_camera_intrinsics << "Camera Intrinsic Matrix K= " << endl
                            << " " << this->K_0_Mat << endl;
                    this->logger->info(ss_camera_intrinsics.str());
                    
                    ss_camera_distortion << "Camera Distortion Coefficients C= " << endl
                            << " " << this->C_0_Mat << endl;
                    this->logger->info(ss_camera_distortion.str());
                    
                    ss_camera_lidar_extrinsics << "Camera-LiDAR Extrinsic Matrix E= " << endl
                            << " " << this->E_0_Mat << endl;
                    this->logger->info(ss_camera_lidar_extrinsics.str());
                    
                    this->logger->info("All calibration matrices have been output for verification");
                    this->logger->info("Calibration completed successfully");
                }
                else
                {
                    this->logger->error("Calibration failed");
                }
            }
            catch (const std::exception &e)
            {
            this->logger->error("Exception during calibration: {}", e.what());
        }
        catch (...)
        {
            this->logger->error("Unknown exception during calibration");
        }
        
        
                ulk.unlock();
                
        
                try {
                    cv::destroyAllWindows();
            cv::waitKey(100);
        }
        catch (...) {
            
                }
            
        
        if (!calibration_success && !this->mapMapping->_is_pass()) {
            this->logger->error("Critical error: Calibration failed and no previous calibration data available");
            this->logger->error("Please restart the program and try again");
            return;
        }
    }
    
    
    if (!this->_thread_working && this->is_alive)
    {
        this->logger->info("Thread starting ...Process");
        this->_thread_working = true;
        if (!this->__RosSpinLoop_working)
        {
            this->__RosSpinLoop_working = true;
            this->rosSpinLoop = thread(std::bind(&Radar::RosSpinLoop, this));
        }
        if (!this->__MainProcessLoop_working)
        {
            this->__MainProcessLoop_working = true;
            this->processLoop = thread(std::bind(&Radar::MainProcessLoop, this));
        }
        if (!this->__VideoRecorderLoop_working)
        {
            this->__VideoRecorderLoop_working = true;
            this->videoRecoderLoop = thread(std::bind(&Radar::VideoRecorderLoop, this));
        }
        this->logger->info("Thread starting ...Done");
    }
    
    
    if (!this->mySerial->_is_open() && this->is_alive)
    {
        this->logger->info("Serial initing ...Process");
        this->mySerial->initSerial(this->SerialPortName, this->PASSWORD);
        this->logger->info("Serial initing ...Done");
    }
    
    
    if (!this->_Ser_working && this->mySerial->_is_open() && this->is_alive)
    {
        this->logger->info("SerThread initing ...Process");
        this->_Ser_working = true;
        this->serRead = thread(std::bind(&Radar::SerReadLoop, this));
        this->serWrite = thread(std::bind(&Radar::SerWriteLoop, this));   
        this->logger->info("SerThread initing ...Done");
    }
    
    
    if (myFrames.size() > 0 && this->is_alive && !this->_if_record)
    {
        myFrames.pop();
    }

    
    bool uartTestEnabled = false;
    if (this->nh->getParam("/radar2025/UARTTestEnabled", uartTestEnabled))
    {
        if (uartTestEnabled != this->UARTTestEnabled)
        {
            this->enableUARTTest(uartTestEnabled);
        }
    }
    
    
    if (this->uartTester && this->UARTTestEnabled)
    {
        
        int interval = 100;
        
        if (this->nh->getParam("/radar2025/uart_test/interval", interval))
        {
            if (interval != this->uartTester->getTestInterval())
            {
                this->setUARTTestInterval(interval);
                
                this->nh->setParam("/radar2025/UARTTestInterval", interval);
            }
        }
        
        else if (this->nh->getParam("/radar2025/UARTTestInterval", interval))
        {
            if (interval != this->uartTester->getTestInterval())
            {
                this->setUARTTestInterval(interval);
                
                this->nh->setParam("/radar2025/uart_test/interval", interval);
            }
        }
    }
}

void Radar::stop()
{
    this->is_alive = false;
    this->logger->warn("Start Shutdown Process...");
    this->logger->flush();
    if (this->cameraThread->is_open())
        this->cameraThread->stop();
    if (this->_thread_working)
    {
        this->_thread_working = false;
        if (this->__RosSpinLoop_working)
        {
            this->__RosSpinLoop_working = false;
            this->rosSpinLoop.join();
        }
        if (this->__MainProcessLoop_working)
        {
            this->__MainProcessLoop_working = false;
            this->processLoop.join();
        }
        if (this->__VideoRecorderLoop_working)
        {
            this->__VideoRecorderLoop_working = false;
            this->videoRecoderLoop.join();
        }
        if (this->_Ser_working)
        {
            this->_Ser_working = false;
            this->serRead.join();
            this->serWrite.join();
        }
    }
    this->armorDetector->unInit();

#if !(defined UsePointCloudSepTarget || defined UseOneLayerInfer)
    this->carDetector->unInit();
#endif

#ifdef ExperimentalOutput
    this->myExpLog->uninit();
#endif

    ros::shutdown();
    this->logger->warn("Program Shutdown");
}

bool Radar::alive()
{
    return this->is_alive;
}
void Radar::downsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int target_points) {
    
    static ros::Time last_log_time = ros::Time::now();
    static const double LOG_INTERVAL = 5.0; 
    bool should_log = (ros::Time::now() - last_log_time).toSec() >= LOG_INTERVAL;
    
    
    if (!cloud || cloud->empty()) {
        if (should_log) {
            logger->warn("Empty point cloud received for downsampling");
            last_log_time = ros::Time::now();
        }
        return;
    }
    
    size_t original_size = cloud->size();
    
    
    if (original_size <= static_cast<size_t>(target_points)) {
        if (should_log) {
            logger->info("Skipping downsampling, point cloud size ({}) already below target ({})", original_size, target_points);
            last_log_time = ros::Time::now();
        }
        return;  
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    try {
        switch (downsample_method_) {
            case VOXEL_GRID: {
                
                
                
                pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
                voxel_grid.setInputCloud(cloud);
                
                
                float leaf_size = voxel_size_;  
                int current_points = cloud->size();
                
                
                int max_iterations;
                if (adaptive_adjust_) {
                    
                    float ratio = static_cast<float>(current_points) / static_cast<float>(target_points);
                    max_iterations = std::min(100, std::max(10, static_cast<int>(ratio * 10)));
                    if (should_log) {
                        this->logger->info("Adaptive adjustment: using {} iterations (ratio: {:.2f})", max_iterations, ratio);
                    }
                } else {
                    
                    max_iterations = adjust_count_max_;
                }
                
                
                int adjust_count = 0;
                while (current_points > target_points && adjust_count < max_iterations) {
                    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
                    voxel_grid.filter(*downsampled_cloud);
                    current_points = downsampled_cloud->size();
                    leaf_size *= 1.05f;  
                    adjust_count++;
                }
                
                
                if (adjust_count >= max_iterations && current_points > target_points) {
                    if (should_log) {
                        this->logger->warn("Reached maximum iterations ({}) but still have {} points (target: {})", 
                            max_iterations, current_points, target_points);
                    }
                }
                break;
            }
            
            case RANDOM: {
                
                pcl::RandomSample<pcl::PointXYZ> random_sample;
                random_sample.setInputCloud(cloud);
                random_sample.setSample(target_points);
                random_sample.filter(*downsampled_cloud);
                break;
            }
            
            case UNIFORM: {
                
                pcl::UniformSampling<pcl::PointXYZ> uniform_sample;
                uniform_sample.setInputCloud(cloud);
                uniform_sample.setRadiusSearch(uniform_radius_);  
                uniform_sample.filter(*downsampled_cloud);
                
                
                if (downsampled_cloud->size() > static_cast<size_t>(target_points)) {
                    pcl::RandomSample<pcl::PointXYZ> random_sample;
                    random_sample.setInputCloud(downsampled_cloud);
                    random_sample.setSample(target_points);
                    random_sample.filter(*downsampled_cloud);
                }
                break;
            }
            
            default:
                if (should_log) {
                    logger->error("Unknown downsample method: {}", static_cast<int>(downsample_method_));
                }
                return;
        }
        
        
        if (downsampled_cloud->empty()) {
            if (should_log) {
                logger->warn("Downsampling resulted in empty cloud, keeping original");
            }
            return;
        }
        
        
        cloud = downsampled_cloud;
        
        
        if (should_log) {
            logger->info("Point cloud downsampled from {} to {} points", original_size, cloud->size());
            last_log_time = ros::Time::now();
        }
    }
    catch (const std::exception& e) {
        if (should_log) {
            logger->error("Exception in downsamplePointCloud: {}", e.what());
            last_log_time = ros::Time::now();
        }
    }
}

void Radar::enableUARTTest(bool enable)
{
    if (enable == UARTTestEnabled)
        return;
        
    UARTTestEnabled = enable;
    
    if (enable)
    {
        
        if (myUART && mySerial && mySerial->_is_open())
        {
            
            if (!uartTester)
            {
                uartTester = std::make_shared<UARTTester>(myUART, mySerial);
                
                
                int interval = 100;
                
                
                if (!this->nh->getParam("/radar2025/uart_test/interval", interval))
                {
                    
                    this->nh->getParam("/radar2025/UARTTestInterval", interval);
                }
                
                
                uartTester->setTestInterval(interval);
            }
            uartTester->start();
            logger->info("UART Test enabled with interval: {} ms", uartTester->getTestInterval());
        }
        else
        {
            UARTTestEnabled = false;
            logger->error("Cannot enable UART Test: UART or Serial not initialized");
        }
    }
    else if (uartTester)
    {
        
        uartTester->stop();
        
        
        if (myUART) {
            std::vector<std::vector<float>> emptyLocations;
            for (int i = 0; i < 6; ++i) {
                std::vector<float> pos = {0.0f, 0.0f};
                emptyLocations.push_back(pos);
            }
            myUART->myUARTPasser.push_loc(emptyLocations);
        }
        
        logger->info("UART Test disabled");
    }
}

void Radar::setUARTTestInterval(int intervalMs)
{
    if (uartTester)
    {
        uartTester->setTestInterval(intervalMs);
    }
}

void Radar::enableByteTracker(bool enable)
{
    if (this->ByteTrackerEnabled != enable) {
        this->ByteTrackerEnabled = enable;
        
        
        if (this->nh) {
            this->nh->setParam("/gui/UseByteTracker", enable);
        }
        
        
        if (this->byteTracker) {
            this->byteTracker->setEnabled(enable);
        }
        
        this->logger->info("ByteTracker {} enabled", enable ? "is" : "is not");
    }
}