/*---Debug---*/

#define Test        // 测试标志
#define TestWithVis // 显示可视化检测结果
// #define UsingVideo  // 是否使用视频(！！！可能造成OOM, 请注意设置FRAME_DEPTH)
#define ShowDepth   // 是否在GUI覆盖深度图
#define ShowPointCloud // 是否在GUI中覆盖可视化点云，用于检查点云与图像配准


/*---Camera Driver Selection---*/
// 选择一个相机驱动，注释掉另一个 
// 注意：若切换相机驱动，请务必更改params\camera0.SJTU.yaml文件中对应相机的内参矩阵K_0,畸变系数C_0,激光雷达与相机的外参矩阵E_0
#define USE_HIKVISION_CAMERA   // 使用海康威视相机
// #define USE_MINDVISION_CAMERA  // 使用迈德威视相机

// 防止同时启用两个相机驱动
#if defined(USE_HIKVISION_CAMERA) && defined(USE_MINDVISION_CAMERA)
#error "Only One Camera Driver Can Be Selected"
#endif

// 确保至少选择了一种相机驱动
#if !defined(USE_HIKVISION_CAMERA) && !defined(USE_MINDVISION_CAMERA) && !defined(UsingVideo)
#warning "No Camera Driver Selected, Default Using Hikvision"
#define USE_HIKVISION_CAMERA // 不选择默认使用海康威视相机驱动
#endif


/*---Common settings---*/

// #define lidarTopicName (char *)"/livox/lidar"

#define lidarTopicName (char *)"/cloudpoints" // NCST-HORIZON-CH128X 激光雷达点云 Topic

/*---For Video Record---*/

#define FRAME_DEPTH 80 // 图像队列深度，OOM时适当减少

/*---For depth and common function---*/
#define MaxPointsNum 20000 // 单帧最大点云数量
#define ImageH 1024        
#define ImageW 1280        //更改图像大小务必同时更改相机对应内外参数
#define maxQueueSize 100   // 点云最大帧队列长度
#define LidarQueueSize 1   // 雷达消息队列
#define Epsilon (const float)1e-6

/*---For UART game info---*/

#define MAXBO 3

/*---For Point Cloud Processing---*/
#define MAX_VISUALIZATION_POINTS 100000  // 超过此数量的点云会在可视化前降采样
#define DEPTH_VIS_MIN_POINTS 100        // 深度图可视化最小点数
#define VISUALIZATION_VOXEL_SIZE 0.05f  // 可视化降采样体素大小(m)

/*---For Location function---*/

#define Z_A true        // Z轴突变调整
#define L_P true        // 位置预测
#define Z_THRE 0.2      // Z轴突变阈值
#define Pre_Time 10     // 预测次数
#define Pre_ratio 0.1f  // 预测速度比例
#define Real_Size_W 15. // 真实宽度  
#define Real_Size_H 28. // 真实高度（长）
#define IoU_THRE 0.6f   // IoU预测阈值

/*---For old SepTarget method [实验性][已废弃][谨慎使用]---*/

// #define UsePointCloudSepTarget
#define MDHistorySize 200 // 背景深度图帧队列大小，影响背景深度图密度
#define _blockSizeH 36    // 栅格大小[建议取值能被图像大小整除]
#define _blockSizeW 36    // 栅格大小
#define MTBoxRatio 1.1    // 分割框扩大比例
#define OffsetRatio 1.3   // 点云离散兼容比例
#define K_size 3          // 卷积核大小

/*---For deepsort [实验性][高性能消耗][谨慎使用]---*/

// #define UseDeepSort

/*---For ExperimentalOutput [实验数据输出][仅作识别效率记录][识别数据将混乱]---*/

// #define ExperimentalOutput     //启用实验模式输出
// #define UseOneLayerInfer //启用单层神经网络预测模式

