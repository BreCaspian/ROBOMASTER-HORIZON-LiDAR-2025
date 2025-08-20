#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) 
: QMainWindow(parent)
, ui(new Ui::MainWindow)
, m_hWnd(NULL)
, m_bThreadState(false) 
, m_bOpenIF(false)
, m_bOpenDevice(false)                     
, m_bStartGrabbing(false)
, m_hInterface(NULL)
, m_hDevice(NULL)               
, m_hStream(NULL) 
, m_nThreadID(-1)
, m_nDeviceNum(0)
, m_nInterfaceNum(0)
, m_pSaveImageBuf(NULL)
, m_nDataBufSize(0)
, m_nSaveImageBufSize(0)
, m_pDataBuf(NULL)
{
    ui->setupUi(this);
    m_hWnd = (void *)ui->widgetDisplay->winId();
    pthread_mutex_init(&m_hSaveImageMux, NULL);
    memset(&m_stImageInfo, 0, sizeof(MV_FG_INPUT_IMAGE_INFO));
    m_lCamInfo.clear();
}

MainWindow::~MainWindow()
{
    pthread_mutex_destroy(&m_hSaveImageMux);
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if(m_bStartGrabbing)
    {
        on_bnStopGrabbing_clicked();
    }
    
    if(m_bOpenIF)
    {
        on_bnIntClose_clicked();
    }
    event->accept();
}

// ch:显示错误信息 | en:Show error message
void MainWindow::ShowErrorMsg(QString csMessage, unsigned int nErrorNum)
{
    QString errorMsg = csMessage;
    if (nErrorNum != 0)
    {
        QString TempMsg;
        TempMsg.sprintf(": Error = %x: ", nErrorNum);
        errorMsg += TempMsg;
    }

    switch (nErrorNum)
    {
    case MV_FG_ERR_INVALID_HANDLE:
        errorMsg += "Error or invalid handle ";
        break;
    case MV_FG_ERR_NOT_IMPLEMENTED:
        errorMsg += "Not supported function ";
        break;
    case MV_FG_ERR_INVALID_BUFFER:
        errorMsg += "Cache is full ";
        break;
    case MV_FG_ERR_INVALID_PARAMETER:
        errorMsg += "Incorrect parameter ";
        break;
    case MV_FG_ERR_RESOURCE_IN_USE:
        errorMsg += "Applying resource failed ";
        break;
    case MV_FG_ERR_NO_DATA:
        errorMsg += "No data ";
        break;
    case MV_FG_ERR_BUFFER_TOO_SMALL:
        errorMsg += "Insufficient memory ";
        break;
    case MV_FG_ERR_GC_UNKNOW:
        errorMsg += "Unknown error ";
        break;
    case MV_FG_ERR_GC_GENERIC:
        errorMsg += "General error ";
        break;
    case MV_FG_ERR_ACCESS_DENIED:
        errorMsg += "No permission ";
        break;
    case MV_FG_ERR_BUSY:
        errorMsg += "Device is busy, or network disconnected ";
        break;
    }

    QMessageBox::information(NULL, "PROMPT", errorMsg);
}

static void *WorkThread(void *pUser)
{
    if (pUser)
    {
        MainWindow *pMainWindow = (MainWindow *)pUser;
        if (NULL == pMainWindow)
        {
            return NULL;
        }
        pMainWindow->ThreadFun();
    }

    return NULL;
}

void MainWindow::ThreadFun()
{
    MV_FG_BUFFER_INFO           stFrameInfo = { 0 };    // ch:图像信息 | en:Frame info
    MV_FG_DISPLAY_FRAME_INFO    stDisplayInfo = { 0 };  // ch:显示的图像信息 | en:Display frame info
    MV_FG_HB_DECODE_PARAM       stHBDecodeInfo = { 0 }; // ch:HB解码图像信息 | en:HB decodes image info
    const unsigned int          nTimeout = 5000;
    int nRet = MV_FG_SUCCESS;

    while(m_bThreadState)
    {
        if (m_bStartGrabbing)
        {
            // ch:获取一帧图像缓存信息 | en:Get one frame buffer's info
            nRet = MV_FG_GetFrameBuffer(m_hStream, &stFrameInfo, nTimeout);
            if (MV_FG_SUCCESS == nRet)
            {
                printf("FrameNum:%ld Width:%d Height:%d, nImageBufLen:%d, enPixelType:%x\n", stFrameInfo.nFrameID, stFrameInfo.nWidth, 
                        stFrameInfo.nHeight, stFrameInfo.nFilledSize, stFrameInfo.enPixelType);
                fflush(stdout);

                // 用于保存图片
                pthread_mutex_lock(&m_hSaveImageMux);
                memset(&m_stImageInfo, 0, sizeof(MV_FG_INPUT_IMAGE_INFO));
                if (NULL == m_pDataBuf || stFrameInfo.nFilledSize > m_nDataBufSize)
                {
                    if (m_pDataBuf)
                    {
                        free(m_pDataBuf);
                        m_pDataBuf = NULL;
                    }

                    m_pDataBuf = (unsigned char*)malloc(sizeof(unsigned char) * stFrameInfo.nFilledSize);
                    if (NULL == m_pDataBuf)
                    {
                        pthread_mutex_unlock(&m_hSaveImageMux);
                        printf("thread no Memory...\n");
                    }
                    m_nDataBufSize = stFrameInfo.nFilledSize;
                }
                memset(m_pDataBuf, 0, m_nDataBufSize);
                memcpy(m_pDataBuf, stFrameInfo.pBuffer, stFrameInfo.nFilledSize);
                m_stImageInfo.nWidth        = stFrameInfo.nWidth;
                m_stImageInfo.nHeight       = stFrameInfo.nHeight;
                m_stImageInfo.enPixelType   = stFrameInfo.enPixelType;
                m_stImageInfo.pImageBuf     = m_pDataBuf;
                m_stImageInfo.nImageBufLen  = stFrameInfo.nFilledSize;
                pthread_mutex_unlock(&m_hSaveImageMux);

                // 自定义格式不支持显示
                if (RemoveCustomPixelFormats(stFrameInfo.enPixelType))
                {
                    MV_FG_ReleaseFrameBuffer(m_hStream, &stFrameInfo);
                    continue;
                }

                // 配置显示图像的参数
                memset(&stDisplayInfo, 0, sizeof(MV_FG_DISPLAY_FRAME_INFO));
                stDisplayInfo.nWidth        = stFrameInfo.nWidth;
                stDisplayInfo.nHeight       = stFrameInfo.nHeight;
                stDisplayInfo.enPixelType   = stFrameInfo.enPixelType;
                stDisplayInfo.pImageBuf     = (unsigned char*)stFrameInfo.pBuffer;
                stDisplayInfo.nImageBufLen  = stFrameInfo.nFilledSize;
                nRet = MV_FG_DisplayOneFrame(m_hStream, m_hWnd, &stDisplayInfo);
                if (MV_FG_SUCCESS != nRet)
                {
                    MV_FG_ReleaseFrameBuffer(m_hStream, &stFrameInfo);
                    printf("thread DisplayOneFrame Faild nRet 0x#%x...\n", nRet);
                    break;
                }

                MV_FG_ReleaseFrameBuffer(m_hStream, &stFrameInfo);
            }
        }
        else
        {
            usleep(5000);       // ch:还未开始取流，调用Sleep() | en:Acquisition not start, call Sleep()
        }
    }
}

void MainWindow::FlushControlButton(bool bOpen, bool bStart)
{
    if ((!bOpen) && (!bStart))
    {
        ui->bnCamOpen->setEnabled(true);
        ui->bnCamClose->setEnabled(false);
        ui->bnStartGrabbing->setEnabled(false);
        ui->bnStopGrabbing->setEnabled(false);
    }
    else if ((bOpen) && (!bStart))
    {
        ui->bnCamOpen->setEnabled(false);
        ui->bnCamClose->setEnabled(true);
        ui->bnStartGrabbing->setEnabled(true);
        ui->bnStopGrabbing->setEnabled(false);
    }
    else if ((bOpen) && (bStart))
    {
        ui->bnCamOpen->setEnabled(false);
        ui->bnCamClose->setEnabled(true);
        ui->bnStartGrabbing->setEnabled(false);
        ui->bnStopGrabbing->setEnabled(true);
    }

    ui->bnIntOpen->setEnabled(!m_bOpenIF);
    ui->bnIntClose->setEnabled(m_bOpenIF);
    ui->bnCamEnum->setEnabled(m_bOpenIF);

    ui->bnCamOpen->setEnabled(!m_bOpenDevice && m_bOpenIF);
    ui->bnCamClose->setEnabled(m_bOpenDevice);

    ui->bnBmpSave->setEnabled(m_bStartGrabbing);
    ui->bnJpegSave->setEnabled(m_bStartGrabbing);
    ui->bnPngSave->setEnabled(m_bStartGrabbing);
    ui->bnTiffSave->setEnabled(m_bStartGrabbing);
}

void MainWindow::on_bnIntEnum_clicked()
{
    int nRet = 0;

    bool bChanged = false;
    nRet = MV_FG_UpdateInterfaceList(MV_FG_CAMERALINK_INTERFACE, &bChanged);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Enum interfaces fail", nRet);
        return;
    }

    nRet = MV_FG_GetNumInterfaces(&m_nInterfaceNum);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Get Interfaces' number fail", nRet);
        return;
    }
    if (0 == m_nInterfaceNum)
    {
        ShowErrorMsg("No Interface", 0);
        return;
    }

    if (bChanged)
    {
        ui->ComboInterfaces->clear();

        // en:Add Interface info in ComboBox
        for (unsigned int i = 0; i < m_nInterfaceNum; ++i)
        {
            QString strMsg;
            MV_FG_INTERFACE_INFO stInterfaceInfo = {0};

            nRet = MV_FG_GetInterfaceInfo(i, &stInterfaceInfo);
            if (MV_FG_SUCCESS != nRet)
            {
                ShowErrorMsg("Get Interfaces' info fail", nRet);
                return;
            }

            switch (stInterfaceInfo.nTLayerType)
            {
            case MV_FG_CXP_INTERFACE:
            {
                strMsg.sprintf("CXP[%d]: %s | %s | %s", i, stInterfaceInfo.IfaceInfo.stCXPIfaceInfo.chDisplayName,
                               stInterfaceInfo.IfaceInfo.stCXPIfaceInfo.chInterfaceID,
                               stInterfaceInfo.IfaceInfo.stCXPIfaceInfo.chSerialNumber);
                break;
            }
            case MV_FG_GEV_INTERFACE:
            {
                strMsg.sprintf("GEV[%d]: %s | %s | %s", i, stInterfaceInfo.IfaceInfo.stGEVIfaceInfo.chDisplayName,
                               stInterfaceInfo.IfaceInfo.stGEVIfaceInfo.chInterfaceID,
                               stInterfaceInfo.IfaceInfo.stGEVIfaceInfo.chSerialNumber);
                break;
            }
            case MV_FG_CAMERALINK_INTERFACE:
            {
                strMsg.sprintf("CML[%d]: %s | %s | %s", i, stInterfaceInfo.IfaceInfo.stCMLIfaceInfo.chDisplayName,
                               stInterfaceInfo.IfaceInfo.stCMLIfaceInfo.chInterfaceID,
                               stInterfaceInfo.IfaceInfo.stCMLIfaceInfo.chSerialNumber);
                break;
            }
            case MV_FG_XoF_INTERFACE:
            {
                strMsg.sprintf("XoF[%d]: %s | %s | %s", i, stInterfaceInfo.IfaceInfo.stXoFIfaceInfo.chDisplayName,
                               stInterfaceInfo.IfaceInfo.stXoFIfaceInfo.chInterfaceID,
                               stInterfaceInfo.IfaceInfo.stXoFIfaceInfo.chSerialNumber);
                break;
            }
            default:
            {
                break;
            }
            }
            ui->ComboInterfaces->addItem(strMsg);
        }
    }

    ui->ComboInterfaces->setCurrentIndex(0);
}

void MainWindow::on_bnIntOpen_clicked()
{
    int nIndex = ui->ComboInterfaces->currentIndex();
    if (m_bOpenIF)
    {
        ShowErrorMsg("already open", 0);
        return;
    }
    else
    {
        for (unsigned int i = 0; i < MAX_INTERFACE_NUM; ++i)
        {
            if (m_bOpenIF)
            {
                ShowErrorMsg("Close Interface first", 0);
                return;
            }
        }
    }

    int nRet = MV_FG_SUCCESS;
    nRet = MV_FG_OpenInterface(nIndex, &m_hInterface);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Open Interface fail", nRet);
        return;
    }


    EnableIFParamsControls();
    m_bOpenIF = true;
    ui->bnIntOpen->setEnabled(false);
    ui->bnIntClose->setEnabled(true);
    ui->bnCamEnum->setEnabled(true);
	GetInterfaceParams();									
    EnableIFParamsControls();
}

void MainWindow::on_bnIntClose_clicked()
{
    int nRet = MV_FG_SUCCESS;
    int nIndex = ui->ComboInterfaces->currentIndex();

    //close all devices
    for (unsigned int i = 0; i < ui->ComboDevices->count(); ++i)
    {
        if (m_bOpenDevice)
        {
            ui->ComboDevices->setCurrentIndex(i);
            on_bnCamClose_clicked();
        }
    }

    nRet = MV_FG_CloseInterface(m_hInterface);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Close Interface fail", nRet);
        return;
    }

    m_bOpenIF = false;
    m_hInterface = NULL;

    FlushControlButton(false, false);
    EnableIFParamsControls();
}

void MainWindow::on_bnCamEnum_clicked()
{
    int nRet = MV_FG_SUCCESS;

    //enum devices
    bool bChanged = false;
    nRet = MV_FG_UpdateDeviceList(m_hInterface, &bChanged);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Enum devices fail", nRet);
        return;
    }

    //get devices' num
    nRet = MV_FG_GetNumDevices(m_hInterface, &m_nDeviceNum);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Get devices' num fail", nRet);
        return;
    }
    if (0 == m_nDeviceNum)
    {
        ShowErrorMsg("No devices", 0);
        return;
    }

    if (bChanged)
    {
        ui->ComboDevices->clear();

        for (unsigned int i = 0; i < m_nDeviceNum; ++i)
        {
            QString strMsg;
            MV_FG_DEVICE_INFO stDeviceInfo = {0};

            nRet = MV_FG_GetDeviceInfo(m_hInterface, i, &stDeviceInfo);
            if (MV_FG_SUCCESS != nRet)
            {
                ShowErrorMsg("Get Devices' Info fail", nRet);
                return;
            }

            switch (stDeviceInfo.nDevType)
            {
            case MV_FG_CXP_DEVICE:
            {
                if (0 != strcmp("", (const char *)stDeviceInfo.DevInfo.stCXPDevInfo.chUserDefinedName))
                {
                    strMsg.sprintf("[CXP]%s [%s]", stDeviceInfo.DevInfo.stCXPDevInfo.chModelName, stDeviceInfo.DevInfo.stCXPDevInfo.chUserDefinedName);
                }
                else
                {
                    strMsg.sprintf("[CXP]%s [%s]", stDeviceInfo.DevInfo.stCXPDevInfo.chModelName, stDeviceInfo.DevInfo.stCXPDevInfo.chSerialNumber);
                }
                break;
            }
            case MV_FG_GEV_DEVICE:
            {
                if (0 != strcmp("", (const char *)stDeviceInfo.DevInfo.stGEVDevInfo.chUserDefinedName))
                {
                    strMsg.sprintf("[GEV]%s [%s]", stDeviceInfo.DevInfo.stGEVDevInfo.chModelName, stDeviceInfo.DevInfo.stGEVDevInfo.chUserDefinedName);
                }
                else
                {
                    strMsg.sprintf("[GEV]%s [%s]", stDeviceInfo.DevInfo.stGEVDevInfo.chModelName, stDeviceInfo.DevInfo.stGEVDevInfo.chSerialNumber);
                }
                break;
            }
            case MV_FG_CAMERALINK_DEVICE:
            {
                if (0 != strcmp("", (const char *)stDeviceInfo.DevInfo.stCMLDevInfo.chUserDefinedName))
                {
                    strMsg.sprintf("[CML]%s [%s]", stDeviceInfo.DevInfo.stCMLDevInfo.chModelName, stDeviceInfo.DevInfo.stCMLDevInfo.chUserDefinedName);
                }
                else
                {
                    strMsg.sprintf("[CML]%s [%s]", stDeviceInfo.DevInfo.stCMLDevInfo.chModelName, stDeviceInfo.DevInfo.stCMLDevInfo.chSerialNumber);
                }
                break;
            }
            case MV_FG_XoF_DEVICE:
            {
                if (0 != strcmp("", (const char *)stDeviceInfo.DevInfo.stXoFDevInfo.chUserDefinedName))
                {
                    strMsg.sprintf("[XoF]%s [%s]", stDeviceInfo.DevInfo.stXoFDevInfo.chModelName, stDeviceInfo.DevInfo.stXoFDevInfo.chUserDefinedName);
                }
                else
                {
                    strMsg.sprintf("[XoF]%s [%s]", stDeviceInfo.DevInfo.stXoFDevInfo.chModelName, stDeviceInfo.DevInfo.stXoFDevInfo.chSerialNumber);
                }
                break;
            }
            default:
            {
                break;
            }
            }

            ui->ComboDevices->addItem(strMsg);
            m_lCamInfo.push_back(strMsg);
        }
    }
}

void MainWindow::on_bnCamOpen_clicked()
{
    int nIndex = ui->ComboDevices->currentIndex();
    if (m_bOpenDevice)
    {
        ShowErrorMsg("already open", 0);
        return;
    }
    int nRet = MV_FG_SUCCESS;
    nRet = MV_FG_OpenDevice(m_hInterface, nIndex, &(m_hDevice));
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Open Device fail", nRet);
        return;
    }
    MV_FG_SetEnumValue(m_hDevice, "AcquisitionMode", 2);
    MV_FG_SetEnumValue(m_hDevice, "TriggerMode", 0);
    
    EnableDevParamsControls();
    m_bOpenDevice = true;

	GetDeviceParams();
    EnableDevParamsControls();
    FlushControlButton(true, false);
}

void MainWindow::on_bnCamClose_clicked()
{
    int nIndex = ui->ComboDevices->currentIndex();

    if (!m_bOpenDevice)
    {
        ShowErrorMsg("not open device", 0);
        return;
    }

    if (m_bStartGrabbing)
    {
        on_bnStopGrabbing_clicked();
    }

    int nRet = MV_FG_SUCCESS;
    nRet = MV_FG_CloseDevice(m_hDevice);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Close Device fail", nRet);
        return;
    }
    m_hDevice = NULL;
    m_bOpenDevice = false;
    FlushControlButton(false, false);
    EnableDevParamsControls();
}

void MainWindow::on_bnStartGrabbing_clicked()
{
    if (true == m_bStartGrabbing)
    {
        return;
    }

    int nIndex = ui->ComboDevices->currentIndex();

    if (!m_bOpenDevice)
    {
        ShowErrorMsg("not open device", 0);
        return;
    }

    int nRet = MV_FG_SUCCESS;

    unsigned int nStreamNum = 0;
    nRet = MV_FG_GetNumStreams(m_hDevice, &nStreamNum);
    if (MV_FG_SUCCESS != nRet || 0 == nStreamNum)
    {
        ShowErrorMsg("Get Stream Number fail", nRet);
        return;
    }

    nRet = MV_FG_OpenStream(m_hDevice, 0, &(m_hStream));
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Open Stream fail", nRet);
        return;
    }

    nRet = MV_FG_SetBufferNum(m_hStream, BUFFER_NUMBER);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Set buffer number failed!", nRet);
        return;
    }

    m_bStartGrabbing = true;
    m_bThreadState = true;
    m_nDevIndex = nIndex;

    //pthread_t nThreadID;
    nRet = pthread_create(&(m_nThreadID), NULL, WorkThread, this);
    if (0 != nRet)
    {
        ShowErrorMsg("pthread create fail", 0);
        return;
    }

    nRet = MV_FG_StartAcquisition(m_hStream);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Start Acquisition fail", nRet);
        nRet = MV_FG_CloseStream(m_hStream);
        if (MV_FG_SUCCESS != nRet)
        {
            ShowErrorMsg("Close Stream fail", nRet);
        }
        m_hStream = NULL;
        return;
    }
    FlushControlButton(true, true);
    EnableIFParamsControls();
    EnableDevParamsControls();
}

void MainWindow::on_bnStopGrabbing_clicked()
{
    int nIndex = ui->ComboDevices->currentIndex();

    if (!m_bStartGrabbing)
    {
        ShowErrorMsg("not grabbing", 0);
        return;
    }

    m_bThreadState = false;

    int nRet = MV_FG_SUCCESS;

    //wait pthread exit
    pthread_join(m_nThreadID, NULL);
    m_nThreadID = -1;

    nRet = MV_FG_StopAcquisition(m_hStream);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Stop Acquisition fail", nRet);
        return;
    }
    m_bStartGrabbing = false;

    nRet = MV_FG_CloseStream(m_hStream);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Close Stream fail", nRet);
        return;
    }
    m_hStream = NULL;

    FlushControlButton(true, false);
    EnableIFParamsControls();
    EnableDevParamsControls();
}

void MainWindow::on_ComboInterfaces_currentIndexChanged(int index)
{
    if (!m_bOpenIF)
    {
        ui->ComboDevices->clear();
        ui->bnIntClose->setEnabled(false);
        ui->bnIntOpen->setEnabled(true);
        ui->bnCamEnum->setEnabled(false);
        ui->bnCamOpen->setEnabled(false);
        ui->bnCamClose->setEnabled(false);
        ui->bnStartGrabbing->setEnabled(false);
        ui->bnStopGrabbing->setEnabled(false);
    }
    else
    {
        list<QString>::iterator i;
        for (i = m_lCamInfo.begin(); i != m_lCamInfo.end(); ++i)
        {
            ui->ComboDevices->addItem(*i);
        }
        ui->bnIntClose->setEnabled(true);
        ui->bnIntOpen->setEnabled(false);
        ui->bnCamEnum->setEnabled(true);
        int nCamIndex = ui->ComboDevices->currentIndex();
        on_ComboDevices_currentIndexChanged(nCamIndex);
    }
}

void MainWindow::on_ComboDevices_currentIndexChanged(int index)
{
    FlushControlButton(m_bOpenDevice, m_bStartGrabbing);
}

bool MainWindow::RemoveCustomPixelFormats(MV_FG_PIXEL_TYPE enPixelFormat)
{
    int nResult = enPixelFormat & MV_FG_PIXEL_CUSTOM;
    if(MV_FG_PIXEL_CUSTOM == nResult)
    {
        return true;
    }
    else
    {
        return false;
    }
}

int MainWindow::SaveImage(SAVE_IAMGE_TYPE enSaveImageType)
{
    pthread_mutex_lock(&m_hSaveImageMux);

    if (NULL == m_pDataBuf)
    {
        pthread_mutex_unlock(&m_hSaveImageMux);
        return MV_FG_ERR_NO_DATA;
    }

    if (RemoveCustomPixelFormats(m_stImageInfo.enPixelType))
    {
        pthread_mutex_unlock(&m_hSaveImageMux);
        return MV_FG_ERR_INVALID_VALUE;
    }

    int             nRet = 0;
    unsigned int    nMaxImageLen = m_stImageInfo.nWidth * m_stImageInfo.nHeight * 4 + 2048; 
    time_t          rawtime;
    struct tm*      timeinfo;

    if (NULL == m_pSaveImageBuf || nMaxImageLen > m_nSaveImageBufSize)
    {
        if (m_pSaveImageBuf)
        {
            free(m_pSaveImageBuf);
            m_pSaveImageBuf = NULL;
        }

        m_pSaveImageBuf = (unsigned char*)malloc(sizeof(unsigned char) * nMaxImageLen);
        if (NULL == m_pSaveImageBuf)
        {
            pthread_mutex_unlock(&m_hSaveImageMux);
            return MV_FG_ERR_OUT_OF_MEMORY;
        }
        m_nSaveImageBufSize = nMaxImageLen;
    }

    memset(m_pSaveImageBuf, 0, nMaxImageLen);
    char szFileName[FILE_NAME_LEN] = { 0 };
    time ( &rawtime );
    timeinfo = localtime(&rawtime);

    do
    {
        if (Image_Bmp == enSaveImageType)
        {
            MV_FG_SAVE_BITMAP_INFO stBmpInfo = { 0 };

            stBmpInfo.stInputImageInfo   = m_stImageInfo;
            stBmpInfo.pBmpBuf            = m_pSaveImageBuf;
            stBmpInfo.nBmpBufSize        = m_nSaveImageBufSize;
            stBmpInfo.enCfaMethod        = MV_FG_CFA_METHOD_OPTIMAL;

            // ch:保存BMP图像 | en:Save to BMP
            nRet = MV_FG_SaveBitmap(m_hStream, &stBmpInfo);
            if (MV_FG_SUCCESS != nRet)
            {
                break;
            }

            sprintf(szFileName, "Image_w%d_h%d_%02d%02d%04d.bmp", 
                m_stImageInfo.nWidth, m_stImageInfo.nHeight, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

            FILE* pImgFile = NULL;
            if (NULL == (pImgFile = fopen(szFileName, "wb")))
            {
                nRet = MV_FG_ERR_INVALID_PARAMETER;
                break;
            }

            fwrite(stBmpInfo.pBmpBuf, 1, stBmpInfo.nBmpBufLen, pImgFile);
            fclose(pImgFile);
        }
        else if (Image_Jpeg == enSaveImageType)
        {
            MV_FG_SAVE_JPEG_INFO stJpgInfo = { 0 };

            stJpgInfo.stInputImageInfo  = m_stImageInfo;
            stJpgInfo.pJpgBuf           = m_pSaveImageBuf;
            stJpgInfo.nJpgBufSize       = m_nSaveImageBufSize;
            stJpgInfo.nJpgQuality       = 60;                   // JPG编码质量(0-100]
            stJpgInfo.enCfaMethod       = MV_FG_CFA_METHOD_OPTIMAL;

            // ch:保存JPEG图像 | en:Save to JPEG
            nRet = MV_FG_SaveJpeg(m_hStream, &stJpgInfo);
            if (MV_FG_SUCCESS != nRet)
            {
                break;
            }

            sprintf(szFileName, "Image_w%d_h%d_%02d%02d%04d.jpg", 
                m_stImageInfo.nWidth, m_stImageInfo.nHeight, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

            FILE* pImgFile = NULL;
            if (NULL == (pImgFile = fopen(szFileName, "wb")))
            {
                nRet = MV_FG_ERR_INVALID_PARAMETER;
                break;
            }

            fwrite(stJpgInfo.pJpgBuf, 1, stJpgInfo.nJpgBufLen, pImgFile);
            fclose(pImgFile);
        }
        else if (Image_Tiff == enSaveImageType)
        {
            MV_FG_SAVE_TIFF_TO_FILE_INFO stTiffInfo ={0};

            stTiffInfo.stInputImageInfo = m_stImageInfo;
            stTiffInfo.fXResolution     = (float)m_stImageInfo.nWidth;
            stTiffInfo.fYResolution     = (float)m_stImageInfo.nHeight;
            stTiffInfo.enResolutionUnit = MV_FG_Resolution_Unit_Inch;
            stTiffInfo.enCfaMethod      = MV_FG_CFA_METHOD_OPTIMAL;

            if (NULL == stTiffInfo.pcImagePath)
            {
                stTiffInfo.pcImagePath = (char*)malloc(sizeof(char) * FILE_NAME_LEN);
                if (NULL == stTiffInfo.pcImagePath)
                {
                    nRet = MV_FG_ERR_OUT_OF_MEMORY;
                    break;
                }
            }
            sprintf(stTiffInfo.pcImagePath, "Image_w%d_h%d_%02d%02d%04d.tif", 
                m_stImageInfo.nWidth, m_stImageInfo.nHeight, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

            // ch:保存Tiff图像 | en:Save to Tiff
            nRet = MV_FG_SaveTiffToFile(m_hStream,&stTiffInfo);
            if (NULL != stTiffInfo.pcImagePath)
            {
                free(stTiffInfo.pcImagePath);
                stTiffInfo.pcImagePath = NULL;
            }
            if (MV_FG_SUCCESS != nRet)
            {
                 break;
            }

        }
        else if (Image_Png == enSaveImageType)
        {
            MV_FG_SAVE_PNG_TO_FILE_INFO stPngInfo = {0};
            
            stPngInfo.stInputImageInfo = m_stImageInfo;
            stPngInfo.nPngCompression  = 6;
            stPngInfo.enCfaMethod      = MV_FG_CFA_METHOD_OPTIMAL;

            if (NULL == stPngInfo.pcImagePath)
            {
                stPngInfo.pcImagePath = (char*)malloc(sizeof(char) * FILE_NAME_LEN);
                if (NULL == stPngInfo.pcImagePath)
                {
                    nRet = MV_FG_ERR_OUT_OF_MEMORY;
                    break;
                }
            }
            sprintf(stPngInfo.pcImagePath, "Image_w%d_h%d_%02d%02d%04d.png", 
                m_stImageInfo.nWidth, m_stImageInfo.nHeight, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

            // ch:保存Png图像 | en:Save to Png
            nRet = MV_FG_SavePngToFile(m_hStream,&stPngInfo);
            if (NULL != stPngInfo.pcImagePath)
            {
                free(stPngInfo.pcImagePath);
                stPngInfo.pcImagePath = NULL;
            }
            if (MV_FG_SUCCESS != nRet)
            {
                break;
            }


        }
        else
        {
            nRet =  MV_FG_ERR_INVALID_PARAMETER;
            break;
        }
    } while (0);

    pthread_mutex_unlock(&m_hSaveImageMux);
    return nRet;
}

void MainWindow::on_bnBmpSave_clicked()
{
    int nRet = SaveImage(Image_Bmp);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Save bmp failed", nRet);
        return;
    }
    ShowErrorMsg("Save bmp succeed", 0);
} 

void MainWindow::on_bnJpegSave_clicked()
{
    int nRet = SaveImage(Image_Jpeg);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Save Jpeg failed", nRet);
        return;
    }
    ShowErrorMsg("Save Jpeg succeed", 0);
}        

void MainWindow::on_bnTiffSave_clicked()
{
    int nRet = SaveImage(Image_Tiff);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Save Tiff failed", nRet);
        return;
    }
    ShowErrorMsg("Save Tiff succeed", 0);
}   

void MainWindow::on_bnPngSave_clicked()
{
    int nRet = SaveImage(Image_Png);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Save Png failed", nRet);
        return;
    }
    ShowErrorMsg("Save Png succeed", 0);
}         

// ch:获取采集卡参数 | en:Get interfaces parameters
int MainWindow::GetInterfaceParams()
{
	if (NULL == m_hInterface)
	{
		return MV_FG_ERR_NOT_INITIALIZED;
	}

	MV_FG_ENUMVALUE stEnumValue = {0};
	MV_FG_INTVALUE  stIntValue = {0};
	int nRet = MV_FG_SUCCESS;

	// ch:获取CameraType | en:Get CameraType
	ui->comboCamType->clear();
	memset(&stEnumValue, 0, sizeof(MV_FG_ENUMVALUE));
	nRet = MV_FG_GetEnumValue(m_hInterface, "CameraType", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboCamType->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}
		}

		ui->comboCamType->setCurrentIndex(nIndex);
	}

	// ch:获取Image Height | en:Get Image Height
	memset(&stIntValue, 0, sizeof(stIntValue));
	nRet = MV_FG_GetIntValue(m_hInterface, "ImageHeight", &stIntValue);
	if (MV_FG_SUCCESS == nRet)
	{
		ui->LHeight->setText(QString("%1").arg(stIntValue.nCurValue));
	}

	// ch:获取LineSelector | en:Get LineSelector
	ui->comboLineSel->clear();
	memset(&stEnumValue, 0, sizeof(MV_FG_ENUMVALUE));
	nRet = MV_FG_GetEnumValue(m_hInterface, "LineSelector", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboLineSel->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}
		}

		ui->comboLineSel->setCurrentIndex(nIndex);
	}

	// ch:获取LineMode | en:Get LineMode
	ui->comboLineSel->clear();
	memset(&stEnumValue, 0, sizeof(MV_FG_ENUMVALUE));
	nRet = MV_FG_GetEnumValue(m_hInterface, "LineSelector", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboLineSel->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}
		}

		ui->comboLineSel->setCurrentIndex(nIndex);
	}


	// ch:获取LineInputPolarity | en:Get LineInputPolarity
	ui->comboLPolarity->clear();
	memset(&stEnumValue, 0, sizeof(MV_FG_ENUMVALUE));
	nRet = MV_FG_GetEnumValue(m_hInterface, "LineInputPolarity", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboLPolarity->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}
		}

		ui->comboLPolarity->setCurrentIndex(nIndex);
	}

	// ch:获取EncoderSelector | en:Get EncoderSelector
	ui->comboEncoderSel->clear();
	memset(&stEnumValue, 0, sizeof(MV_FG_ENUMVALUE));
	nRet = MV_FG_GetEnumValue(m_hInterface, "EncoderSelector", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboEncoderSel->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}
		}

		ui->comboEncoderSel->setCurrentIndex(nIndex);
	}

	// ch:获取EncoderSourceA | en:Get EncoderSourceA
	ui->comboEncoderA->clear();
	memset(&stEnumValue, 0, sizeof(MV_FG_ENUMVALUE));
	nRet = MV_FG_GetEnumValue(m_hInterface, "EncoderSourceA", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboEncoderA->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}
		}

		ui->comboEncoderA->setCurrentIndex(nIndex);
	}


	// ch:获取EncoderSourceB | en:Get EncoderSourceB
	ui->comboEncoderB->clear();
	memset(&stEnumValue, 0, sizeof(MV_FG_ENUMVALUE));
	nRet = MV_FG_GetEnumValue(m_hInterface, "EncoderSourceB", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboEncoderB->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}
		}

		ui->comboEncoderB->setCurrentIndex(nIndex);
	}

	// ch:获取EncoderTriggerMode | en:Get EncoderTriggerMode
	ui->comboEnTrigger->clear();
	memset(&stEnumValue, 0, sizeof(MV_FG_ENUMVALUE));
	nRet = MV_FG_GetEnumValue(m_hInterface, "EncoderTriggerMode", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboEnTrigger->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}
		}

		ui->comboEnTrigger->setCurrentIndex(nIndex);
	}

	// ch:获取CCSelector | en:Get CCSelector
	ui->comboCCSel->clear();
	memset(&stEnumValue, 0, sizeof(MV_FG_ENUMVALUE));
	nRet = MV_FG_GetEnumValue(m_hInterface, "CCSelector", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboCCSel->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}
		}

		ui->comboCCSel->setCurrentIndex(nIndex);
	}

	// ch:获取CCSource | en:Get CCSource
	ui->comboCCSource->clear();
	memset(&stEnumValue, 0, sizeof(MV_FG_ENUMVALUE));
	nRet = MV_FG_GetEnumValue(m_hInterface, "CCSource", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboCCSource->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}
		}

		ui->comboCCSource->setCurrentIndex(nIndex);
	}

	return nRet;
}

int MainWindow::GetDeviceParams()
{
	int nRet = 0;

	MV_FG_INTVALUE stIntValue = {0};
	MV_FG_ENUMVALUE stEnumValue = {0};

	// ch:获取Width | en:Get Width
	memset(&stIntValue, 0, sizeof(stIntValue));
	nRet = MV_FG_GetIntValue(m_hDevice, "Width", &stIntValue);
	if (MV_FG_SUCCESS == nRet)
	{
		ui->LCamWidth->setText(QString("%1").arg(stIntValue.nCurValue));
	}

	// ch:获取Height | en:Get Height
	memset(&stIntValue, 0, sizeof(stIntValue));
	nRet = MV_FG_GetIntValue(m_hDevice, "Height", &stIntValue);
	if (MV_FG_SUCCESS == nRet)
	{
		ui->LCamHeight->setText(QString("%1").arg(stIntValue.nCurValue));
	}

	// ch:获取PixelFormat | en:Get PixelFormat
	ui->comboPixelType->clear();
	memset(&stEnumValue, 0, sizeof(stEnumValue));
	nRet = MV_FG_GetEnumValue(m_hDevice, "PixelFormat", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboPixelType->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}

		}
		ui->comboPixelType->setCurrentIndex(nIndex);
	}

	// ch:获取ScanMode | en:Get ScanMode
	ui->comboScanMode->clear();
	memset(&stEnumValue, 0, sizeof(stEnumValue));
	nRet = MV_FG_GetEnumValue(m_hDevice, "ScanMode", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboScanMode->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}

		}
		ui->comboScanMode->setCurrentIndex(nIndex);
	}

	// ch:获取TriggerSelector | en:Get TriggerSelector
	ui->comboTriggerSel->clear();
	memset(&stEnumValue, 0, sizeof(stEnumValue));
	nRet = MV_FG_GetEnumValue(m_hDevice, "TriggerSelector", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboTriggerSel->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}

		}
		ui->comboTriggerSel->setCurrentIndex(nIndex);
	}

	// ch:获取TriggerMode | en:Get TriggerMode
	ui->comboTriggerMode->clear();
	memset(&stEnumValue, 0, sizeof(stEnumValue));
	nRet = MV_FG_GetEnumValue(m_hDevice, "TriggerMode", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboTriggerMode->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}

		}
		ui->comboTriggerMode->setCurrentIndex(nIndex);
	}

	// ch:获取TriggerSource | en:Get TriggerSource
	ui->comboTriggerSource->clear();
	memset(&stEnumValue, 0, sizeof(stEnumValue));
	nRet = MV_FG_GetEnumValue(m_hDevice, "TriggerSource", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboTriggerSource->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}

		}
		ui->comboTriggerSource->setCurrentIndex(nIndex);
	}


	// ch:获取TriggerActivation | en:Get TriggerActivation
	ui->comboTriggerActivation->clear();
	memset(&stEnumValue, 0, sizeof(stEnumValue));
	nRet = MV_FG_GetEnumValue(m_hDevice, "TriggerActivation", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboTriggerActivation->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}

		}
		ui->comboTriggerActivation->setCurrentIndex(nIndex);
	}


	// ch:获取 TestPattern | en:Get TestPattern
	ui->comboTestMode->clear();
	memset(&stEnumValue, 0, sizeof(stEnumValue));
	nRet = MV_FG_GetEnumValue(m_hDevice, "TestPattern", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboTestMode->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}

		}
		ui->comboTestMode->setCurrentIndex(nIndex);
	}

	return nRet;
}

void MainWindow::on_comboCamType_currentIndexChanged(int index)
{
	if (false == ui->comboCamType->isEnabled())
	{
		return;
	}

	int nIndex =  ui->comboCamType->currentIndex();
    QString strCBText =  ui->comboCamType->currentText();
	int nRet = MV_FG_SetEnumValueByString(m_hInterface, "CameraType", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set CameraType failed.", nRet);
		return;
	}
    EnableIFParamsControls();
}

void MainWindow::on_comboLineSel_currentIndexChanged(int index)
{
	if (false == ui->comboLineSel->isEnabled())
	{
		return;
	}

	int nIndex =  ui->comboLineSel->currentIndex();
    QString strCBText =  ui->comboLineSel->currentText();

	int nRet = MV_FG_SetEnumValueByString(m_hInterface, "LineSelector", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set LineSelector failed.", nRet);
		return;
	}
}

void MainWindow::on_comboLPolarity_currentIndexChanged(int index)
{
	if (false == ui->comboLPolarity->isEnabled())
	{
		return;
	}

    QString strCBText =  ui->comboLPolarity->currentText();

	int nRet = MV_FG_SetEnumValueByString(m_hInterface, "LineInputPolarity", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set LineInputPolarity failed.", nRet);
		return;
	}
}

void MainWindow::on_comboEncoderSel_currentIndexChanged(int index)
{
	if (false == ui->comboEncoderSel->isEnabled())
	{
		return;
	}

    QString strCBText =  ui->comboEncoderSel->currentText();

	int nRet = MV_FG_SetEnumValueByString(m_hInterface, "EncoderSelector", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set EncoderSelector failed.", nRet);
		return;
	}
}

void MainWindow::on_comboEncoderA_currentIndexChanged(int index)
{
	if (false == ui->comboEncoderA->isEnabled())
	{
		return;
	}

    QString strCBText =  ui->comboEncoderA->currentText();

	int nRet = MV_FG_SetEnumValueByString(m_hInterface, "EncoderSourceA", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set EncoderSourceA failed.", nRet);
		return;
	}
}

void MainWindow::on_comboEncoderB_currentIndexChanged(int index)
{
	if (false == ui->comboEncoderB->isEnabled())
	{
		return;
	}

    QString strCBText =  ui->comboEncoderB->currentText();

	int nRet = MV_FG_SetEnumValueByString(m_hInterface, "EncoderSourceB", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set EncoderSourceB failed.", nRet);
		return;
	}
}

void MainWindow::on_comboEnTrigger_currentIndexChanged(int index)
{
	if (false == ui->comboEnTrigger->isEnabled())
	{
		return;
	}

    QString strCBText =  ui->comboEnTrigger->currentText();

	int nRet = MV_FG_SetEnumValueByString(m_hInterface, "EncoderTriggerMode", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set EncoderTriggerMode failed.", nRet);
		return;
	}
}

void MainWindow::on_comboCCSel_currentIndexChanged(int index)
{
	if (false == ui->comboCCSel->isEnabled())
	{
		return;
	}

    QString strCBText =  ui->comboCCSel->currentText();

	int nRet = MV_FG_SetEnumValueByString(m_hInterface, "CCSelector", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set CCSelector failed.", nRet);
		return;
	}
}

void MainWindow::on_comboCCSource_currentIndexChanged(int index)
{
	if (false == ui->comboCCSource->isEnabled())
	{
		return;
	}

    QString strCBText =  ui->comboCCSource->currentText();

	int nRet = MV_FG_SetEnumValueByString(m_hInterface, "CCSource", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set CCSource failed.", nRet);
		return;
	}
}

void MainWindow::on_LHeight_editingFinished()
{
    if(ui->LHeight->hasFocus()) 
    {
        int nRet = MV_FG_SetIntValue(m_hInterface, "ImageHeight", ui->LHeight->text().toInt());
        if (MV_FG_SUCCESS != nRet)
        {
            ShowErrorMsg("Set ImageHeight failed.", nRet);
            return;
        }
    }
}


void MainWindow::EnableIFParamsControls()
{
    if(m_bOpenIF && !m_bStartGrabbing)
    {
        QString strLBText =  ui->comboCamType->currentText();
        ui->LHeight->setEnabled(strLBText.toStdString() == "LineScan");
    }
    else
    {
        ui->LHeight->setEnabled(false);
    }

    ui->comboLineSel->setEnabled(m_bOpenIF && !m_bStartGrabbing);
    ui->comboLPolarity->setEnabled(m_bOpenIF && !m_bStartGrabbing);
    ui->comboEncoderSel->setEnabled(m_bOpenIF && !m_bStartGrabbing);
    ui->comboEncoderA->setEnabled(m_bOpenIF && !m_bStartGrabbing);
    ui->comboEncoderB->setEnabled(m_bOpenIF && !m_bStartGrabbing);
    ui->comboEnTrigger->setEnabled(m_bOpenIF && !m_bStartGrabbing);
    ui->comboCCSel->setEnabled(m_bOpenIF && !m_bStartGrabbing);
    ui->comboCCSource->setEnabled(m_bOpenIF && !m_bStartGrabbing);
    ui->comboCamType->setEnabled(m_bOpenIF && !m_bStartGrabbing);
}


void MainWindow::on_comboPixelType_currentIndexChanged(int index)
{
	if (false == ui->comboPixelType->isEnabled())
	{
		return;
	}

    QString strCBText =  ui->comboPixelType->currentText();
	int nRet = MV_FG_SetEnumValueByString(m_hDevice, "PixelFormat", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set PixelFormat failed.", nRet);
		return;
	}
}

void MainWindow::on_comboScanMode_currentIndexChanged(int index)
{
	if (false == ui->comboScanMode->isEnabled())
	{
		return;
	}

    QString strCBText =  ui->comboScanMode->currentText();

	int nRet = MV_FG_SetEnumValueByString(m_hDevice, "ScanMode", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set ScanMode failed.", nRet);
		return;
	}
}

void MainWindow::on_comboTriggerSel_currentIndexChanged(int index)
{
	if (false == ui->comboTriggerSel->isEnabled())
	{
		return;
	}

    QString strCBText =  ui->comboTriggerSel->currentText();

	int nRet = MV_FG_SetEnumValueByString(m_hDevice, "TriggerSelector", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set TriggerSelector failed.", nRet);
		return;
	}
}

void MainWindow::on_comboTriggerMode_currentIndexChanged(int index)
{
	if (false == ui->comboTriggerMode->isEnabled())
	{
		return;
	}

    QString strCBText =  ui->comboTriggerMode->currentText();

	int nRet = MV_FG_SetEnumValueByString(m_hDevice, "TriggerMode", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set TriggerMode failed.", nRet);
		return;
	}
}

void MainWindow::on_comboTriggerSource_currentIndexChanged(int index)
{
	if (false == ui->comboTriggerSource->isEnabled())
	{
		return;
	}

    QString strCBText =  ui->comboTriggerSource->currentText();

	int nRet = MV_FG_SetEnumValueByString(m_hDevice, "TriggerSource", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set TriggerSource failed.", nRet);
		return;
	} 
}

void MainWindow::on_comboTriggerActivation_currentIndexChanged(int index)
{
	if (false == ui->comboTriggerActivation->isEnabled())
	{
		return;
	}

    QString strCBText =  ui->comboTriggerActivation->currentText();

	int nRet = MV_FG_SetEnumValueByString(m_hDevice, "TriggerActivation", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set TriggerActivation failed.", nRet);
		return;
	} 
}

void MainWindow::on_comboTestMode_currentIndexChanged(int index)
{
	if (false == ui->comboTestMode->isEnabled())
	{
		return;
	}

    QString strCBText =  ui->comboTestMode->currentText();

	int nRet = MV_FG_SetEnumValueByString(m_hDevice, "TestPattern", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set TestPattern failed.", nRet);
		return;
	} 
}

void MainWindow::on_bnTriggerOnce_clicked()
{
    if (true != m_bStartGrabbing)
    {
        return;
    }

    int nRet = MV_FG_SetCommandValue(m_hDevice, "TriggerSoftware");
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Trigger software failed", nRet);
    }  
}

void MainWindow::on_LCamWidth_editingFinished()
{
    if(ui->LCamWidth->hasFocus()) 
    {
        int nRet = MV_FG_SetIntValue(m_hDevice, "Width", ui->LCamWidth->text().toInt());
        if (MV_FG_SUCCESS != nRet)
        {
            ShowErrorMsg("Set Width failed.", nRet);
            return;
        }
    }
}

void MainWindow::on_LCamHeight_editingFinished()
{
    if(ui->LCamHeight->hasFocus()) 
    {
        int nRet = MV_FG_SetIntValue(m_hDevice, "Height", ui->LCamHeight->text().toInt());
        if (MV_FG_SUCCESS != nRet)
        {
            ShowErrorMsg("Set Height failed.", nRet);
            return;
        }
    }
}

void MainWindow::EnableDevParamsControls()
{
    // 触发模式 on 触发源 software 使能TriggerOnce
    ui->LCamWidth->setEnabled(m_bOpenDevice && !m_bStartGrabbing);
    ui->LCamHeight->setEnabled(m_bOpenDevice && !m_bStartGrabbing);
    ui->comboPixelType->setEnabled(m_bOpenDevice && !m_bStartGrabbing);
    ui->comboScanMode->setEnabled(m_bOpenDevice && !m_bStartGrabbing);
    ui->comboTriggerSel->setEnabled(m_bOpenDevice && !m_bStartGrabbing);
    ui->comboTriggerMode->setEnabled(m_bOpenDevice && !m_bStartGrabbing);
    ui->comboTriggerSource->setEnabled(m_bOpenDevice && !m_bStartGrabbing);
    ui->comboTriggerActivation->setEnabled(m_bOpenDevice && !m_bStartGrabbing);
    ui->comboTestMode->setEnabled(m_bOpenDevice && !m_bStartGrabbing);

    if (m_bOpenDevice)
	{
        QString strTriggerMode =  ui->comboTriggerMode->currentText();
        QString strTriggerSource =  ui->comboTriggerSource->currentText();
        ui->bnTriggerOnce->setEnabled(strTriggerMode.toStdString() == "On" && strTriggerSource.toStdString() == "Software");
	}
	else
	{
        ui->bnTriggerOnce->setEnabled(false);
	}
}