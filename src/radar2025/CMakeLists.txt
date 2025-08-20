cmake_minimum_required(VERSION 3.9.0)
SET(CMAKE_BUILD_TYPE "Release")
project(radar2025 LANGUAGES CXX CUDA)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
  message_runtime
  image_transport
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters sample_consensus search kdtree visualization)

find_package(CUDA 11 REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
message("CUDA_LIBRARIES:${CUDA_LIBRARIES}")
message("CUDA_INCLUDE_DIRS:${CUDA_INCLUDE_DIRS}")
include_directories()
add_compile_options(-std=c++17)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,stack-size=16000000")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -pthread -lpthread -O3  -Wall -fstack-protector-all")
# RTX3060  根据所使用运算端更改
set(CUDA_GEN_CODE "-gencode=arch=compute_86,code=sm_86")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}  -std=c++17 -O3 -Xcompiler -fPIC -g -w ${CUDA_GEN_CODE}")

add_message_files(
  FILES
  Location.msg
  Locations.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(CATKIN_DEPENDS
	roscpp
  rospy
  roslib
  std_msgs
	sensor_msgs
  cv_bridge
  message_generation
  message_runtime
  image_transport
	)
# cuda
include_directories(/usr/local/cuda-12.6/cuda/include)
link_directories(/usr/local/cuda-12.6/cuda/lib)
# cudnn
include_directories(/usr/local/cuda-12.6/cuda/include)
link_directories(/usr/local/cuda-12.6/targets/x86_64-linux/lib)
# Tensorrt
include_directories(/opt/TensorRT/TensorRT-10.7.0.23/include)
link_directories(/opt/TensorRT/TensorRT-10.7.0.23/lib)
# mkl
# include_directories(${MKLROOT}/include)
# link_directories(${MKLROOT}/lib/intel64)
# fmt
include_directories(${FMT_INCLUDE_DIRS})

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  # 这里可以添加PCL的确切路径
  # 如果编译失败可以尝试添加下面的指定系统路径
  # /usr/include/pcl-1.10
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

# add_definitions(-DEIGEN_NO_DEBUG) 
# 添加这个宏定义，避免Eigen库的调试信息，提高性能。建议在功能完全完善的的情况下使用
# 这是为避免Eigen与PCL可能的冲突而添加的

# 添加海康相机MVS SDK路径
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../ThirdParty/MVS/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../ThirdParty/MVS/lib/64)


# 编译ByteTrack库
add_library(bytetrack SHARED
  RadarClass/ByteTracker/src/ByteTrack/BYTETracker.cpp
  RadarClass/ByteTracker/src/ByteTrack/KalmanFilter.cpp
  RadarClass/ByteTracker/src/ByteTrack/lapjv.cpp
  RadarClass/ByteTracker/src/ByteTrack/Object.cpp
  RadarClass/ByteTracker/src/ByteTrack/Rect.cpp
  RadarClass/ByteTracker/src/ByteTrack/STrack.cpp
  RadarClass/ByteTracker/src/ByteTrackerWrapper.cpp
)
target_include_directories(bytetrack PUBLIC
  RadarClass/ByteTracker/include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(bytetrack
  ${OpenCV_LIBS}
  Eigen3::Eigen
  spdlog::spdlog
)

set(radar2025_source
  main.cpp
  RadarClass/TRTInference/TRTInfer/CUDA/preprocess.cu
  RadarClass/TRTInference/TRTInfer/src/Inference.cpp
  RadarClass/TRTGenerator/TRTGenerator/src/Generator.cpp
  RadarClass/Camera/src/camera.cpp
  RadarClass/Camera/src/VideoRecorder.cpp
  RadarClass/Common/src/algorithm.cpp
  RadarClass/PointCloudVisualizer/src/PointCloudVisualizer.cpp
  RadarClass/Detectors/src/ArmorDetector.cpp
  RadarClass/Detectors/src/CarDetector.cpp
  RadarClass/Detectors/src/depthProcesser.cpp
  RadarClass/Detectors/src/MovementDetector.cpp
  RadarClass/Location/src/location.cpp
  RadarClass/Location/src/MapMapping.cpp
  RadarClass/Radar/src/Radar.cpp
  RadarClass/UART/src/offical_judge_Handler.cpp
  RadarClass/UART/src/serial.cpp
  RadarClass/UART/src/UART.cpp
  RadarClass/UART/src/UARTPasser.cpp
  RadarClass/UART/src/UARTTester.cpp
  RadarClass/Logger/src/Logger.cpp
  RadarClass/DsTracker/src/deepsort.cpp
  RadarClass/DsTracker/src/featuretensor.cpp
  RadarClass/DsTracker/src/kalmanfilter.cpp
  RadarClass/DsTracker/src/linear_assignment.cpp
  RadarClass/DsTracker/src/sort.cpp
  RadarClass/DsTracker/src/track.cpp
  RadarClass/DsTracker/src/tracker.cpp
  RadarClass/DsTracker/src/hungarianoper.cpp
  RadarClass/DsTracker/src/DsTracker.cpp
  RadarClass/Logger/src/ExpLog.cpp
  ../ThirdParty/Json/jsoncpp.cpp
  )

cuda_add_executable(radar2025
  ${radar2025_source}
)

add_executable(offline_point_cloud_publisher tools/offline_point_cloud_pub.cpp)
target_link_libraries(offline_point_cloud_publisher ${PCL_LIBRARIES}  ${catkin_LIBRARIES} )

add_executable(gui_node GUI/GUI.cpp)
target_link_libraries(gui_node ${catkin_LIBRARIES} )

target_sources(radar2025
  PRIVATE
  ${radar2025_source}
)
set_target_properties(radar2025 PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON)
add_dependencies(radar2025 ${catkin_EXPORTED_TARGETS})
target_include_directories(radar2025
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  logs)
target_link_libraries(radar2025
  ${catkin_LIBRARIES}
  ${LIBS} ${CUDA_LIBRARIES}
  ${CUDA_cudadevrt_LIBRARY}
  ${OpenCV_LIBS}
  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES}
  ${PCL_LIBRARIES}
  # libmkl_rt.so 
  pthread
  libm.so
  libdl.so
  MVSDK
  Eigen3::Eigen
  cuda
  spdlog
  fmt::fmt-header-only
  cudart
  nvonnxparser
  libnvinfer.so
  libnvinfer_plugin.so
  bytetrack  # 添加ByteTrack库
  MvCameraControl
)# 注意这里有个MvCameraControl，是用来适配海康工业相机的，如果使用海康工业相机遗漏MvCameraControl会编译报错

install(TARGETS radar2025 
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

install(DIRECTORY
  Config
  launch 
  logs 
  models 
  params 
  Record 
  ExpResultDir 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	)

macro(remove_warnings)
  add_definitions(-Wno-deprecated-declarations
    -Wno-unused-but-set-variable
    -Wno-unused-variable)
endmacro()

remove_warnings()