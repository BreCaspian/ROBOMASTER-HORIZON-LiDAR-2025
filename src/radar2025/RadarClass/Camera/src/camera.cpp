
#include "../include/camera.h"
#ifdef USE_HIKVISION_CAMERA
#ifndef UsingVideo

// 注意：若切换相机驱动，请务必更改params\camera0.SJTU.yaml文件中对应相机的内参矩阵K_0,畸变系数C_0,激光雷达与相机联合标定的外参矩阵E_0

MV_Camera::MV_Camera()
{
}

MV_Camera::MV_Camera(bool Is_init, string config_path)
{
    this->CameraConfigPath = config_path;
    
    
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    
    
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK || stDeviceList.nDeviceNum == 0)
    {
        this->logger->error("No camera found!");
        printf("Camera not found!");
        sleep(1);
        return;
    }
    
    
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
    if (nRet != MV_OK)
    {
        this->logger->error("Create Handle fail! nRet: {}", nRet);
        return;
    }
    
    
    nRet = MV_CC_OpenDevice(handle);
    if (nRet != MV_OK)
    {
        this->logger->error("Open Device fail! nRet: {}", nRet);
        return;
    }
    
    
    nRet = MV_CC_SetEnumValue(handle, "PixelFormat", PixelType_Gvsp_BGR8_Packed);
    if (nRet != MV_OK)
    {
        this->logger->error("Set PixelFormat fail! nRet: {}", nRet);
    }
    
    
    MV_CC_SetFloatValue(handle, "ExposureTime", 5000.0f);
    MV_CC_SetFloatValue(handle, "Gain", 0.0f);
    
    
    nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true);
    if (nRet != MV_OK)
    {
        this->logger->error("Enable FrameRate control fail! nRet: {}", nRet);
    }
    else
    {
        
        
        nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", 19.2f);
        if (nRet != MV_OK)
        {
            this->logger->error("Set AcquisitionFrameRate fail! nRet: {}", nRet);
        }
        else
        {
            
            MVCC_FLOATVALUE stFrameRate = {0};
            MV_CC_GetFloatValue(handle, "ResultingFrameRate", &stFrameRate);
            this->logger->info("Camera FrameRate set to: {} fps", stFrameRate.fCurValue);
        }
    }
    
    
    if (Is_init || access(CameraConfigPath.c_str(), F_OK) == 0)
    {
        nRet = MV_CC_FeatureLoad(handle, const_cast<char*>(CameraConfigPath.c_str()));
        if (nRet != MV_OK)
        {
            this->logger->warn("Load parameters fail! nRet: {}", nRet);
            
        }
        else
        {
            this->logger->info("Camera parameters loaded from {}", CameraConfigPath);
            
            
            bool isAutoGain = false;
            if (getGainMode(isAutoGain) && isAutoGain)
            {
                this->logger->info("Auto gain is enabled");
            }
            else
            {
                this->logger->info("Auto gain is disabled");
            }
        }
    }
    
    
    disableAutoEx();
    
    
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    if (nRet != MV_OK)
    {
        this->logger->error("Get PayloadSize fail! nRet: {}", nRet);
        return;
    }
    
    
    nDataSize = stParam.nCurValue;
    pData = (unsigned char*)malloc(nDataSize);
    if (pData == NULL)
    {
        this->logger->error("Malloc memory failed!");
        return;
    }
    
    
    nRet = MV_CC_StartGrabbing(handle);
    if (nRet != MV_OK)
    {
        this->logger->error("Start Grabbing fail! nRet: {}", nRet);
        return;
    }
    
    this->logger->info("Camera setup complete");
    _openflag = true;
}

MV_Camera::~MV_Camera()
{
    uninit();
}

FrameBag MV_Camera::read()
{
    FrameBag framebag;
    framebag.flag = false;
    
    if (handle == NULL || pData == NULL)
    {
        this->logger->error("No handled camera found!");
        printf("No handled camera found!");
        sleep(1);
        return framebag;
    }
    
    
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    
    
    int nRet = MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stImageInfo, 1500); 
    
    
    if (nRet != MV_OK)
    {
        this->logger->warn("Get Image failed, retrying... nRet: {}", nRet);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
        nRet = MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stImageInfo, 2000); 
    }
    
    if (nRet != MV_OK)
    {
        this->logger->error("Get Image fail! nRet: {}", nRet);
        return framebag;
    }
    
    
    if (stImageInfo.enPixelType == PixelType_Gvsp_BGR8_Packed)
    {
        framebag.frame = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData).clone();
        if (!framebag.frame.empty()) {
        framebag.flag = true;
        } else {
            this->logger->error("Created empty frame after image acquisition");
        }
    }
    else
    {
        this->logger->error("Unsupported pixel format: 0x{:x}", stImageInfo.enPixelType);
    }
    
    return framebag;
}

void MV_Camera::uninit()
{
    if (handle != NULL)
    {
        MV_CC_StopGrabbing(handle);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        handle = NULL;
    }
    
    if (pData != NULL)
    {
        free(pData);
        pData = NULL;
    }
}

void MV_Camera::setExposureTime(int ex)
{
    
    ex = std::max(1000, std::min(ex, 700000)); 
    
    
    MV_CC_StopGrabbing(handle);
    
    
    int nRet = MV_CC_SetFloatValue(handle, "ExposureTime", static_cast<float>(ex));
    if (MV_OK != nRet)
    {
        this->logger->error("Set Exposure Time fail! nRet: {}", nRet);
    }
    else
    {
        this->logger->info("Set Exposure Time to {} μs", ex);
    }
    
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    
    MV_CC_StartGrabbing(handle);
}

void MV_Camera::setGain(int gain)
{
    if (handle == NULL)
        return;
    
    
    gain = std::max(0, std::min(gain, 24));
    
    
    bool isAutoGain = false;
    getGainMode(isAutoGain);
    
    
    if (isAutoGain) {
        if (!setGainMode(false)) {
            this->logger->error("Failed to set manual gain mode");
            return;
        }
    }
    
    
    MV_CC_StopGrabbing(handle);
    
    
    int nRet = MV_CC_SetFloatValue(handle, "Gain", float(gain));
    if (nRet != MV_OK) {
        this->logger->error("Set Gain fail! nRet: {}, value: {}", nRet, gain);
    } else {
        
        MVCC_FLOATVALUE stGain = {0};
        MV_CC_GetFloatValue(handle, "Gain", &stGain);
        this->logger->info("Set Gain to: {} (requested: {})", stGain.fCurValue, gain);
    }
    
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    
    MV_CC_StartGrabbing(handle);
}
bool MV_Camera::getGainMode(bool &isAuto)
{
    if (handle == NULL)
        return false;
    
    MVCC_ENUMVALUE enumValue = {0};
    int nRet = MV_CC_GetEnumValue(handle, "GainAuto", &enumValue);
    if (nRet != MV_OK)
    {
        this->logger->error("Get GainAuto fail! nRet: {}", nRet);
        return false;
    }
    
    
    isAuto = (enumValue.nCurValue != 0);
    return true;
}
bool MV_Camera::setGainMode(bool isAuto)
{
    if (handle == NULL)
        return false;
    
    
    int nValue = isAuto ? 2 : 0;
    int nRet = MV_CC_SetEnumValue(handle, "GainAuto", nValue);
    if (nRet != MV_OK)
    {
        this->logger->error("Set GainAuto fail! nRet: {}", nRet);
        return false;
    }
    
    
    this->logger->info("GainAuto set to {}: {}", nValue, isAuto ? "Continuous" : "Off");
    return true;
}

void MV_Camera::saveParam(const char *tCameraConfigPath)
{
    if (handle == NULL)
        return;
    
    
    
    
    int nRet = MV_CC_FeatureSave(handle, const_cast<char*>(tCameraConfigPath));
    if (nRet != MV_OK)
    {
        this->logger->error("Save camera parameters failed! nRet: {}", nRet);
    }
    else
    {
        this->logger->info("Camera parameters saved to {}", tCameraConfigPath);
    }
}

void MV_Camera::disableAutoEx()
{
    if (handle == NULL)
        return;
    
    
    int nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", 0); 
    if (MV_OK != nRet)
    {
        this->logger->error("Disable Auto Exposure fail! nRet: {}", nRet);
    }
    
    
    nRet = MV_CC_SetEnumValue(handle, "GainAuto", 0); 
    if (MV_OK != nRet)
    {
        this->logger->error("Disable Auto Gain fail! nRet: {}", nRet);
    }
    
    this->logger->info("Auto exposure and auto gain disabled");
}

int MV_Camera::getExposureTime()
{
    if (handle == NULL)
        return -1;
    float fExposureTime = 0.0;
    MVCC_FLOATVALUE stExposureTime = {0};
    MV_CC_GetFloatValue(handle, "ExposureTime", &stExposureTime);
    return (int)stExposureTime.fCurValue;
}

int MV_Camera::getAnalogGain()
{
    if (handle == NULL)
        return -1;
    MVCC_FLOATVALUE stGain = {0};
    MV_CC_GetFloatValue(handle, "Gain", &stGain);
    return (int)stGain.fCurValue;
}
float MV_Camera::getFrameRate()
{
    if (handle == NULL)
        return -1;
    
    MVCC_FLOATVALUE stFrameRate = {0};
    int nRet = MV_CC_GetFloatValue(handle, "ResultingFrameRate", &stFrameRate);
    if (nRet != MV_OK)
    {
        this->logger->error("Get FrameRate fail! nRet: {}", nRet);
        return -1;
    }
    
    return stFrameRate.fCurValue;
}
bool MV_Camera::setFrameRate(float fps)
{
    if (handle == NULL)
        return false;
    
    
    int nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true);
    if (nRet != MV_OK)
    {
        this->logger->error("Enable FrameRate control fail! nRet: {}", nRet);
        return false;
    }
    
    
    nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", fps);
    if (nRet != MV_OK)
    {
        this->logger->error("Set AcquisitionFrameRate fail! nRet: {}", nRet);
        return false;
    }
    
    
    MVCC_FLOATVALUE stFrameRate = {0};
    MV_CC_GetFloatValue(handle, "ResultingFrameRate", &stFrameRate);
    this->logger->info("Camera FrameRate set to: {} fps", stFrameRate.fCurValue);
    
    return true;
}
#endif

#ifdef UsingVideo
CameraThread::CameraThread(string config_path, string video_path)
{
    this->CameraConfigPath = config_path;
    this->TestVideoPath = video_path;
    
    
    _realtime_fps = 0.0f;
    _fps_history.clear();
}
#else
CameraThread::CameraThread(string config_path)
{
    this->CameraConfigPath = config_path;
    
    
    _realtime_fps = 0.0f;
    _fps_history.clear();
}
#endif
CameraThread::~CameraThread()
{
    
    if (_open) {
        stop();
    }
}

void CameraThread::start()
{
    while (!this->_open)
    {
        this->open();
    }
}

void CameraThread::stop()
{
    this->_is_init = false;
    this->_open = false;
    this->_alive = false;
    
    
    {
        std::lock_guard<std::mutex> lock(_fps_mutex);
        _realtime_fps = 0.0f;
        _fps_history.clear();
    }
    
    this->release();
}

#ifndef UsingVideo
void CameraThread::openCamera(bool is_init)
{
    const int MAX_RETRY = 3;
    int retry_count = 0;
    bool initFlag = false;
    
    
    cv::destroyAllWindows();
    cv::waitKey(100);
    
    while (retry_count < MAX_RETRY && !initFlag) {
    try
    {
            this->logger->info("Camera opening ...Process (attempt {}/{})", retry_count + 1, MAX_RETRY);
            
            
            if (this->_cap) {
                this->_cap.reset();
            }
            
            
        this->_cap = std::make_shared<MV_Camera>(is_init, this->CameraConfigPath);
            
            
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            
            
        FrameBag framebag = this->_cap->read();
            if (!framebag.flag || framebag.frame.empty())
        {
                this->logger->warn("Camera frame read failed, retrying...");
                retry_count++;
                
                
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
        }
            
            this->logger->info("Successfully captured frame: {}x{}", 
                              framebag.frame.cols, framebag.frame.rows);
            
            
        if (!is_init && framebag.flag)
        {
                
                std::lock_guard<std::mutex> lock(this->_cap_mutex);
                
                
                cv::destroyAllWindows();
                cv::waitKey(100);
                
                
                this->_cap->setExposureTime(15000);  
                this->_cap->setGain(20);            
                
                
            this->_cap->disableAutoEx();
                
                
                this->adjustExposure();
                
                
            this->_cap->saveParam(this->CameraConfigPath.c_str());
        }
            
        initFlag = true;
        this->logger->info("Camera opening ...Done");
    }
    catch (const std::exception &e)
    {
            this->logger->error("CameraThread::openCamera() error: {}", e.what());
            retry_count++;
            
            if (retry_count < MAX_RETRY) {
                this->logger->info("Retrying camera initialization in 1 second...");
                std::this_thread::sleep_for(std::chrono::seconds(1));
    }
        }
    }
    
    
    if (this->_cap) {
    this->_cap->_openflag = initFlag;
    } else {
        this->logger->error("Camera initialization failed after {} attempts", MAX_RETRY);
    }
}

void CameraThread::adjustExposure()
{
    
    cv::destroyAllWindows();
    cv::waitKey(100);

    try {
        
    namedWindow("EXPOSURE Press Q to Exit", WINDOW_NORMAL);
        resizeWindow("EXPOSURE Press Q to Exit", 800, 600);
    moveWindow("EXPOSURE Press Q to Exit", 200, 200);
        
        
        createTrackbar("Exposure (us)", "EXPOSURE Press Q to Exit", 0, 830000);
        setTrackbarPos("Exposure (us)", "EXPOSURE Press Q to Exit", 15000);
    
        
        createTrackbar("Gain", "EXPOSURE Press Q to Exit", 0, 23);  
        setTrackbarPos("Gain", "EXPOSURE Press Q to Exit", 10);
    
        
        createTrackbar("Exit", "EXPOSURE Press Q to Exit", 0, 1);
        setTrackbarPos("Exit", "EXPOSURE Press Q to Exit", 0);
        
    FrameBag framebag = this->_cap->read();
        bool shouldExit = false;
        
        while (framebag.flag && !shouldExit) {
            
            int expValue = getTrackbarPos("Exposure (us)", "EXPOSURE Press Q to Exit");
            int gainValue = getTrackbarPos("Gain", "EXPOSURE Press Q to Exit");
            
            
            this->_cap->setExposureTime(expValue);
            this->_cap->setGain(gainValue);
        
            
            Mat displayFrame = framebag.frame.clone();
            putText(displayFrame, 
                   "Exposure: " + to_string(expValue) + " us | Gain: " + to_string(gainValue),
                   Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
            
            
            imshow("EXPOSURE Press Q to Exit", displayFrame);
            
            
            int key = waitKey(1);
            if (key == 81 || key == 113 || getTrackbarPos("Exit", "EXPOSURE Press Q to Exit") == 1) {
                shouldExit = true;
            }
            
            
        framebag = this->_cap->read();
            if (!framebag.flag) {
                this->logger->warn("Failed to get frame during exposure adjustment");
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                framebag = this->_cap->read();
            }
        }
        
        
        int finalExp = getTrackbarPos("Exposure (us)", "EXPOSURE Press Q to Exit");
        int finalGain = getTrackbarPos("Gain", "EXPOSURE Press Q to Exit");
        
        this->logger->info("Final Camera Settings - Exposure: {} us, Gain: {}", 
                         finalExp, finalGain);
    }
    catch (const std::exception &e) {
        this->logger->error("Error in exposure adjustment: {}", e.what());
        }
    
    
    cv::destroyAllWindows();
    cv::waitKey(100);
}
#endif

void CameraThread::open()
{
#ifdef UsingVideo
    if (!this->_open)
    {
        if (access(TestVideoPath.c_str(), F_OK) != 0)
        {
            this->logger->error("No video file : {}", TestVideoPath);
            sleep(1);
            return;
        }
        this->_cap = VideoCapture(TestVideoPath);
        this->_open = true;
        if (!this->_is_init && this->_open)
            this->_is_init = true;
    }
#else
    if (!this->_open && this->_alive)
    {
        this->openCamera(this->_is_init);
        this->_open = this->_cap->_openflag;
        if (!this->_is_init && this->_open)
            this->_is_init = true;
    }
#endif
}

bool CameraThread::is_open()
{
    return this->_open;
}

FrameBag CameraThread::read()
{
    FrameBag framebag;
    framebag.flag = false;
    
    if (!this->_open)
        return framebag;
        
    std::lock_guard<std::mutex> lock(this->_cap_mutex);
    
#ifdef UsingVideo
    Mat frame;
    ++frame_counter;
    if (frame_counter >= this->_cap.get(cv::CAP_PROP_FRAME_COUNT))
    {
        this->_cap.set(cv::CAP_PROP_POS_FRAMES, 0);
        frame_counter = 0;
    }
    if (!this->_cap.read(frame))
        return framebag;
    framebag.frame = frame;
    framebag.flag = true;
#else
    framebag = this->_cap->read();
#endif
    
    
    updateFPS();
    
    return framebag;
}

void CameraThread::release()
{
#ifndef UsingVideo
    if (this->_cap) {
        
        std::lock_guard<std::mutex> lock(this->_cap_mutex);
        this->_cap->uninit();
        this->_cap.reset(); 
    }
#endif
    this->_open = false;
}

#endif 
#ifdef USE_MINDVISION_CAMERA
#ifndef UsingVideo
MV_Camera::MV_Camera()
{
    
    pFrameBuffer = NULL;
    pRawDataBuffer = NULL;
}

MV_Camera::MV_Camera(bool Is_init, string config_path)
{
    
    pFrameBuffer = NULL;
    pRawDataBuffer = NULL;
    
    this->CameraConfigPath = config_path;
    CameraSdkInit(1);
    this->iStatus = CameraEnumerateDevice(this->tCameraEnumList, &this->iCameraCounts);
    if (this->iCameraCounts == 0)
    {
        this->logger->error("No camera found!");
        sleep(1);
        return;
    }
    try
    {
        this->iStatus = CameraInit(&this->tCameraEnumList[0], -1, -1, &this->hCamera);
        this->logger->info("CameraIniting ...Process");
    }
    catch (const std::exception &e)
    {
        this->logger->error("CameraInitERROR!{}", e.what());
        sleep(1);
        return;
    }
    if (this->iStatus != CAMERA_STATUS_SUCCESS)
    {
        this->logger->error("CameraInit Failed!{}", this->iStatus);
        sleep(1);
        return;
    }
    else
    {
        this->logger->info("CameraIniting ...Done");
    }
    CameraGetCapability(this->hCamera, &this->tCapability);
    if (!tCapability.sIspCapacity.bMonoSensor)
        CameraSetIspOutFormat(this->hCamera, CAMERA_MEDIA_TYPE_BGR8);
    else
    {
        this->logger->error("None suitable camera!");
    }
    CameraSetTriggerMode(this->hCamera, 0);
    if (Is_init || access(CameraConfigPath.c_str(), F_OK) == 0)
        CameraReadParameterFromFile(this->hCamera, const_cast<char *>(CameraConfigPath.c_str()));
    CameraSetAeState(this->hCamera, 0);
    CameraPlay(this->hCamera);
    int frameBufferSize = this->tCapability.sResolutionRange.iWidthMax * this->tCapability.sResolutionRange.iHeightMax * 3;
    this->pFrameBuffer = CameraAlignMalloc(frameBufferSize, 16);
    if (this->pFrameBuffer == NULL) {
        this->logger->error("Failed to allocate frame buffer!");
        uninit();
        return;
    }
    this->logger->info("Camera setup complete");
    _openflag = true;
}

MV_Camera::~MV_Camera()
{
    
    uninit();
}

FrameBag MV_Camera::read()
{
    FrameBag framebag;
    framebag.flag = false;
    
    if (this->hCamera == -1)
    {
        this->logger->error("No handled camera found!");
        sleep(1);
        return framebag;
    }
    
    
    int ret = CameraGetImageBuffer(this->hCamera, &this->sFrameInfo, &this->pRawDataBuffer, 200);
    if (ret != CAMERA_STATUS_SUCCESS)
    {
        this->logger->error("Failed to get image buffer! Error code: {}", ret);
        return framebag;
    }
    
    
    if (this->pFrameBuffer == NULL) {
        this->logger->error("Frame buffer is NULL!");
        CameraReleaseImageBuffer(this->hCamera, this->pRawDataBuffer);
        return framebag;
    }
    
    ret = CameraImageProcess(this->hCamera, this->pRawDataBuffer, this->pFrameBuffer, &this->sFrameInfo);
    if (ret != CAMERA_STATUS_SUCCESS)
    {
        
        CameraReleaseImageBuffer(this->hCamera, this->pRawDataBuffer);
        this->logger->error("Can not process Image! Error code: {}", ret);
        return framebag;
    }
    
    try {
        
        framebag.frame = cv::Mat(
            Size(this->sFrameInfo.iWidth, this->sFrameInfo.iHeight),
            CV_8UC3,
            this->pFrameBuffer).clone();
            
        framebag.flag = true;
    } catch (const std::exception &e) {
        this->logger->error("Exception during Mat creation: {}", e.what());
        framebag.flag = false;
    }
    
    
    CameraReleaseImageBuffer(this->hCamera, this->pRawDataBuffer);
    return framebag;
}

void MV_Camera::uninit()
{
    try {
        
        if (this->hCamera != -1)
        {
            CameraUnInit(this->hCamera);
            this->hCamera = -1;
        }
        
        
        if (this->pFrameBuffer != NULL)
        {
            CameraAlignFree(this->pFrameBuffer);
            this->pFrameBuffer = NULL;
        }
    } catch (const std::exception &e) {
        this->logger->error("Exception during uninit: {}", e.what());
    }
}

void MV_Camera::setExposureTime(int ex)
{
    if (this->hCamera == -1)
        return;
    CameraSetExposureTime(this->hCamera, ex);
}

void MV_Camera::setGain(int gain)
{
    if (this->hCamera == -1)
        return;
    CameraSetAnalogGain(this->hCamera, gain);
}

void MV_Camera::saveParam(const char *tCameraConfigPath)
{
    if (access(tCameraConfigPath, F_OK) == 0)
        return;
    if (this->hCamera == -1)
        return;
    CameraSaveParameterToFile(this->hCamera, const_cast<char *>(tCameraConfigPath));
}

void MV_Camera::disableAutoEx()
{
    if (this->hCamera == -1)
        return;
    CameraSetAeState(this->hCamera, 0);
}

int MV_Camera::getExposureTime()
{
    if (this->hCamera == -1)
        return -1;
    double ex;
    CameraGetExposureTime(this->hCamera, &ex);
    return int(ex);
}

int MV_Camera::getAnalogGain()
{
    if (this->hCamera == -1)
        return -1;
    int gain;
    CameraGetAnalogGain(this->hCamera, &gain);
    return gain;
}

bool MV_Camera::getGainMode(bool &isAuto)
{
    BOOL autoGain = FALSE;
    int nRet = CameraGetAeState(hCamera, &autoGain);
    if (nRet == CAMERA_STATUS_SUCCESS) {
        isAuto = (autoGain == TRUE);
        return true;
    }
    this->logger->error("Failed to get gain mode: error={}", nRet);
    return false;
}

bool MV_Camera::setGainMode(bool isAuto)
{
    BOOL autoGain = isAuto ? TRUE : FALSE;
    int nRet = CameraSetAeState(hCamera, autoGain);
    if (nRet == CAMERA_STATUS_SUCCESS) {
        this->logger->info("Set gain mode to {}", isAuto ? "auto" : "manual");
        return true;
    }
    this->logger->error("Failed to set gain mode: error={}", nRet);
    return false;
}

float MV_Camera::getFrameRate()
{
    float fps = 0.0f;
    int speedLevel = 0;
    int nRet = CameraGetFrameSpeed(hCamera, &speedLevel);
    if (nRet == CAMERA_STATUS_SUCCESS) {
        
        
        switch (speedLevel) {
            case 0: fps = 5.0f; break;   
            case 1: fps = 10.0f; break;
            case 2: fps = 15.0f; break;
            case 3: fps = 20.0f; break;
            case 4: fps = 25.0f; break;
            case 5: fps = 30.0f; break;  
            default: fps = 15.0f;        
        }
        return fps;
    }
    this->logger->error("Failed to get frame rate: error={}", nRet);
    return 0.0f;
}

bool MV_Camera::setFrameRate(float fps)
{
    
    int speedLevel;
    if (fps <= 5.0f) speedLevel = 0;
    else if (fps <= 10.0f) speedLevel = 1;
    else if (fps <= 15.0f) speedLevel = 2;
    else if (fps <= 20.0f) speedLevel = 3;
    else if (fps <= 25.0f) speedLevel = 4;
    else speedLevel = 5;
    
    int nRet = CameraSetFrameSpeed(hCamera, speedLevel);
    if (nRet == CAMERA_STATUS_SUCCESS) {
        this->logger->info("Set frame rate to level {} (approx. {} fps)", speedLevel, fps);
        return true;
    }
    this->logger->error("Failed to set frame rate: error={}", nRet);
    return false;
}
#endif

#ifdef UsingVideo
CameraThread::CameraThread(string config_path, string video_path)
{
    this->CameraConfigPath = config_path;
    this->TestVideoPath = video_path;
}
#else
CameraThread::CameraThread(string config_path)
{
    this->CameraConfigPath = config_path;
}
#endif
CameraThread::~CameraThread()
{
}

void CameraThread::start()
{
    while (!this->_open)
    {
        this->open();
    }
}

void CameraThread::stop()
{
    this->_is_init = false;
    this->_open = false;
    this->_alive = false;
    this->release();
}

#ifndef UsingVideo
void CameraThread::openCamera(bool is_init)
{
    bool initFlag = false;
    try
    {
        this->logger->info("Camera opening ...Process");
        this->_cap = std::make_shared<MV_Camera>(is_init, this->CameraConfigPath);
        FrameBag framebag = this->_cap->read();
        if (!framebag.flag)
        {
            this->logger->warn("Camera not inited");
            return;
        }
        framebag = this->_cap->read();
        if (!is_init && framebag.flag)
        {
            namedWindow("PREVIEW", WINDOW_NORMAL);
            resizeWindow("PREVIEW", Size(840, 640));
            setWindowProperty("PREVIEW", WND_PROP_TOPMOST, 1);
            moveWindow("PREVIEW", 100, 100);
            imshow("PREVIEW", framebag.frame);
            int key = waitKey(0);
            destroyWindow("PREVIEW");
            this->_cap->disableAutoEx();
            if (key == 84 || key == 116)
                this->adjustExposure();
            this->_cap->saveParam(this->CameraConfigPath.c_str());
        }
        initFlag = true;
        this->logger->info("Camera opening ...Done");
    }
    catch (const std::exception &e)
    {
        this->logger->error("CameraThread::openCamera(){}", e.what());
        sleep(1);
        return;
    }
    this->_cap->_openflag = initFlag;
}

void CameraThread::adjustExposure()
{
    namedWindow("EXPOSURE Press Q to Exit", WINDOW_NORMAL);
    resizeWindow("EXPOSURE Press Q to Exit", 1280, 960);
    moveWindow("EXPOSURE Press Q to Exit", 200, 200);
    setWindowProperty("EXPOSURE Press Q to Exit", WND_PROP_TOPMOST, 1);
    createTrackbar("ex", "EXPOSURE Press Q to Exit", 0, 30000);
    setTrackbarPos("ex", "EXPOSURE Press Q to Exit", this->_cap->getExposureTime() != -1 ? this->_cap->getExposureTime() : 0);
    createTrackbar("gain", "EXPOSURE Press Q to Exit", 0, 256);
    setTrackbarPos("gain", "EXPOSURE Press Q to Exit", this->_cap->getAnalogGain() != -1 ? this->_cap->getAnalogGain() : 0);
    createTrackbar("Quit", "EXPOSURE Press Q to Exit", 0, 1);
    setTrackbarPos("Quit", "EXPOSURE Press Q to Exit", 0);
    FrameBag framebag = this->_cap->read();
    while (framebag.flag && waitKey(1) != 81 && waitKey(1) != 113 && getTrackbarPos("Quit", "EXPOSURE Press Q to Exit") == 0)
    {
        this->_cap->setExposureTime(getTrackbarPos("ex", "EXPOSURE Press Q to Exit"));
        this->_cap->setGain(getTrackbarPos("gain", "EXPOSURE Press Q to Exit"));
        imshow("EXPOSURE Press Q to Exit", framebag.frame);
        framebag = this->_cap->read();
        waitKey(1);
    }
    int ex = this->_cap->getExposureTime();
    int gain = this->_cap->getAnalogGain();
    this->logger->info("Setting Expoure Time {}us", ex);
    this->logger->info("Setting analog gain {}", gain);
    destroyWindow("EXPOSURE Press Q to Exit");
}
#endif

void CameraThread::open()
{
#ifdef UsingVideo
    if (!this->_open)
    {
        if (access(TestVideoPath.c_str(), F_OK) != 0)
        {
            this->logger->error("No video file : {}", TestVideoPath);
            sleep(1);
            return;
        }
        this->_cap = VideoCapture(TestVideoPath);
        this->_open = true;
        if (!this->_is_init && this->_open)
            this->_is_init = true;
    }
#else
    if (!this->_open && this->_alive)
    {
        this->openCamera(this->_is_init);
        this->_open = this->_cap->_openflag;
        if (!this->_is_init && this->_open)
            this->_is_init = true;
    }
#endif
}

bool CameraThread::is_open()
{
    return this->_open;
}

FrameBag CameraThread::read()
{
    FrameBag framebag;
    framebag.flag = false;
    
    if (!this->_open)
        return framebag;
        
    std::lock_guard<std::mutex> lock(this->_cap_mutex);
    
#ifdef UsingVideo
    Mat frame;
    ++frame_counter;
    if (frame_counter >= this->_cap.get(cv::CAP_PROP_FRAME_COUNT))
    {
        this->_cap.set(cv::CAP_PROP_POS_FRAMES, 0);
        frame_counter = 0;
    }
    if (!this->_cap.read(frame))
        return framebag;
    framebag.frame = frame;
    framebag.flag = true;
#else
    framebag = this->_cap->read();
#endif
    
    
    updateFPS();
    
    return framebag;
}

void CameraThread::release()
{
#ifndef UsingVideo
    if (this->_cap) {
        
        std::lock_guard<std::mutex> lock(this->_cap_mutex);
        this->_cap->uninit();
        this->_cap.reset(); 
    }
#endif
    this->_open = false;
}

#endif 

