// 注意：若切换相机驱动，请务必更改params\camera0.SJTU.yaml文件中对应相机的内参矩阵K_0,畸变系数C_0,激光雷达与相机联合标定的外参矩阵E_0

#ifndef __CAMERA_H
#define __CAMERA_H

#include "../../Common/include/public.h"
#include "../../../config.h"
#include <chrono>
#include <deque>
#include <mutex>

#ifdef USE_HIKVISION_CAMERA

#include "../../../ThirdParty/MVS/include/MvCameraControl.h"
#include "../../../ThirdParty/MVS/include/CameraParams.h"
#include "../../../ThirdParty/MVS/include/PixelType.h"

#ifndef UsingVideo
class MV_Camera
{
public:
    typedef std::shared_ptr<MV_Camera> Ptr;

private:
    void* handle = NULL;
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    unsigned char* pData = NULL;
    unsigned int nDataSize = 0;
    
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");
    string CameraConfigPath;

public:
    bool _openflag = false;

public:
    MV_Camera();
    MV_Camera(bool Is_init, string config_path);
    ~MV_Camera();

    FrameBag read();
    void uninit();

    void setExposureTime(int ex = 30);
    void setGain(int gain);
    bool getGainMode(bool &isAuto);
    bool setGainMode(bool isAuto);
    float getFrameRate();
    bool setFrameRate(float fps);
    void saveParam(const char tCameraConfigPath[23]);
    void disableAutoEx();
    int getExposureTime();
    int getAnalogGain();
};
#endif

class CameraThread
{
public:
    typedef std::shared_ptr<CameraThread> Ptr;

private:
    bool _open = false;
    bool _alive = true;

#ifdef UsingVideo
    VideoCapture _cap;
    int frame_counter = 0;
    string TestVideoPath;
#else
    MV_Camera::Ptr _cap;
#endif

    bool _is_init = false;
    string CameraConfigPath;
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");
    std::mutex _cap_mutex;

public:
#ifndef UsingVideo
    void openCamera(bool is_init);
    void adjustExposure();
#endif

#ifdef UsingVideo
    CameraThread(string config_path, string video_path);
#else
    CameraThread(string config_path);
#endif

    ~CameraThread();
    void open();
    bool is_open();
    FrameBag read();
    void release();
    void start();
    void stop();
    
    bool getGainMode(bool &isAuto) {
#ifndef UsingVideo
        if (_cap && _open)
            return _cap->getGainMode(isAuto);
#endif
        return false;
    }
    
    bool setGainMode(bool isAuto) {
#ifndef UsingVideo
        if (_cap && _open)
            return _cap->setGainMode(isAuto);
#endif
        return false;
    }
    
    float getFrameRate() {
#ifndef UsingVideo
        if (_cap && _open)
            return _cap->getFrameRate();
#endif
        return 0.0f;
    }
    
    bool setFrameRate(float fps) {
#ifndef UsingVideo
        if (_cap && _open)
            return _cap->setFrameRate(fps);
#endif
        return false;
    }
    
    float getRealtimeFPS() {
        return _realtime_fps;
    }
    
    float getCurrentExposure() {
#ifndef UsingVideo
        if (_cap && _open)
            return _cap->getExposureTime();
#endif
        return 0.0f;
    }
    
    float getCurrentGain() {
#ifndef UsingVideo
        if (_cap && _open)
            return _cap->getAnalogGain();
#endif
        return 0.0f;
    }

private:
    std::chrono::steady_clock::time_point _last_frame_time;
    std::chrono::steady_clock::time_point _current_frame_time;
    float _realtime_fps = 0.0f;
    std::deque<float> _fps_history;
    const size_t _fps_history_size = 10;
    std::mutex _fps_mutex;
    
    void updateFPS() {
        std::lock_guard<std::mutex> lock(_fps_mutex);
        
        _current_frame_time = std::chrono::steady_clock::now();
        if (_last_frame_time.time_since_epoch().count() > 0) {
            float delta_ms = std::chrono::duration<float, std::milli>(_current_frame_time - _last_frame_time).count();
            if (delta_ms > 0) {
                float current_fps = 1000.0f / delta_ms;
                _fps_history.push_back(current_fps);
                if (_fps_history.size() > _fps_history_size) {
                    _fps_history.pop_front();
                }
                float sum = 0.0f;
                for (const auto& fps : _fps_history) {
                    sum += fps;
                }
                _realtime_fps = sum / _fps_history.size();
            }
        }
        _last_frame_time = _current_frame_time;
    }
};

#endif

#ifdef USE_MINDVISION_CAMERA

#include "/home/yao/MVSDK/include/CameraApi.h"

#ifndef UsingVideo
class MV_Camera
{
public:
    typedef std::shared_ptr<MV_Camera> Ptr;

private:
    int iCameraCounts = 1;
    int iStatus = -1;
    tSdkCameraDevInfo tCameraEnumList[2];
    int hCamera = -1;
    tSdkCameraCapbility tCapability;
    tSdkFrameHead sFrameInfo;
    unsigned char *pFrameBuffer;
    unsigned char *pRawDataBuffer;

    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");
    string CameraConfigPath;

public:
    bool _openflag = false;

public:
    MV_Camera();
    MV_Camera(bool Is_init, string config_path);
    ~MV_Camera();

    FrameBag read();
    void uninit();

    void setExposureTime(int ex = 30);
    void setGain(int gain);
    bool getGainMode(bool &isAuto);
    bool setGainMode(bool isAuto);
    float getFrameRate();
    bool setFrameRate(float fps);
    void saveParam(const char tCameraConfigPath[23]);
    void disableAutoEx();
    int getExposureTime();
    int getAnalogGain();
};
#endif

class CameraThread
{
public:
    typedef std::shared_ptr<CameraThread> Ptr;

private:
    bool _open = false;
    bool _alive = true;

#ifdef UsingVideo
    VideoCapture _cap;
    int frame_counter = 0;
    string TestVideoPath;
#else
    MV_Camera::Ptr _cap;
#endif

    bool _is_init = false;
    string CameraConfigPath;
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");
    std::mutex _cap_mutex;

    std::chrono::steady_clock::time_point _last_frame_time;
    std::chrono::steady_clock::time_point _current_frame_time;
    float _realtime_fps = 0.0f;
    std::deque<float> _fps_history;
    const size_t _fps_history_size = 10;
    std::mutex _fps_mutex;
    
    void updateFPS() {
        std::lock_guard<std::mutex> lock(_fps_mutex);
        
        _current_frame_time = std::chrono::steady_clock::now();
        if (_last_frame_time.time_since_epoch().count() > 0) {
            float delta_ms = std::chrono::duration<float, std::milli>(_current_frame_time - _last_frame_time).count();
            if (delta_ms > 0) {
                float current_fps = 1000.0f / delta_ms;
                _fps_history.push_back(current_fps);
                if (_fps_history.size() > _fps_history_size) {
                    _fps_history.pop_front();
                }
                float sum = 0.0f;
                for (const auto& fps : _fps_history) {
                    sum += fps;
                }
                _realtime_fps = sum / _fps_history.size();
            }
        }
        _last_frame_time = _current_frame_time;
    }

public:
#ifndef UsingVideo
    void openCamera(bool is_init);
    void adjustExposure();
#endif

#ifdef UsingVideo
    CameraThread(string config_path, string video_path);
#else
    CameraThread(string config_path);
#endif

    ~CameraThread();
    void open();
    bool is_open();
    FrameBag read();
    void release();
    void start();
    void stop();
    
    bool getGainMode(bool &isAuto) {
#ifndef UsingVideo
        if (_cap && _open)
            return _cap->getGainMode(isAuto);
#endif
        return false;
    }
    
    bool setGainMode(bool isAuto) {
#ifndef UsingVideo
        if (_cap && _open)
            return _cap->setGainMode(isAuto);
#endif
        return false;
    }
    
    float getFrameRate() {
#ifndef UsingVideo
        if (_cap && _open)
            return _cap->getFrameRate();
#endif
        return 0.0f;
    }
    
    bool setFrameRate(float fps) {
#ifndef UsingVideo
        if (_cap && _open)
            return _cap->setFrameRate(fps);
#endif
        return false;
    }
    
    float getRealtimeFPS() {
        return _realtime_fps;
    }
    
    float getCurrentExposure() {
#ifndef UsingVideo
        if (_cap && _open)
            return _cap->getExposureTime();
#endif
        return 0.0f;
    }
    
    float getCurrentGain() {
#ifndef UsingVideo
        if (_cap && _open)
            return _cap->getAnalogGain();
#endif
        return 0.0f;
    }
};

#endif
#endif
