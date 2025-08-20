#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) 
: QMainWindow(parent)
, ui(new Ui::MainWindow)
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
, m_nTriggerMode(TRIGGER_MODE_OFF)
, m_bSoftWareTriggerCheck(false)
, m_nTriggerSource(-1)
{
    ui->setupUi(this);
    m_hWnd[0] = (void *)ui->widgetDisplay->winId();
    m_hWnd[1] = (void *)ui->widgetDisplay_2->winId();
    pthread_mutex_init(&m_hSaveImageMux, NULL);
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
    MV_FG_BUFFER_INFO           stFrameInfo = { 0 };        // ch:图像信息 | en:Frame info
    MV_FG_DISPLAY_FRAME_INFO    stDisplayInfo = { 0 };      // ch:显示的图像信息 | en:Display frame info
    MV_FG_RECONSTRUCT_INFO      stReconstructInfo = {0};    // ch:图像重组信息 | en:Image reconstruction info
    unsigned char*              pImageBufferList[MAX_EXPOSURE_NUM] = {0};
    unsigned int                nImageBufferSize = 0;
    unsigned int                nLightNum = 0;
    const unsigned int          nTimeout = 1000;
    int                         nRet = 0;

    // 本示例程序只做功能演示，最大支持2灯，后续用户可自定义数量
    nRet = MV_FG_SetEnumValue(m_hDevice, "MultiLightControl", MAX_EXPOSURE_NUM);
    if (MV_FG_SUCCESS == nRet)
    {
        nLightNum = MAX_EXPOSURE_NUM;
    }
    else    // 不支持分时频闪的相机此处不报错
    {
        nLightNum = 0;
    }

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

                // 自定义格式不支持显示
                if (RemoveCustomPixelFormats(stFrameInfo.enPixelType))
                {
                    MV_FG_ReleaseFrameBuffer(m_hStream, &stFrameInfo);
                    continue;
                }

                // 开始图像重组
                if (nLightNum != 0)
                {
                    // 如果图像大小由小变大需要重新分配缓存
                    if (stFrameInfo.nFilledSize / MAX_EXPOSURE_NUM > nImageBufferSize)
                    {
                        for (unsigned int i = 0; i < MAX_EXPOSURE_NUM; i++)
                        {
                            if (NULL != pImageBufferList[i])
                            {
                                free(pImageBufferList[i]);
                                pImageBufferList[i] = NULL;
                            }

                            pImageBufferList[i] = (unsigned char*)malloc(sizeof(unsigned char) * stFrameInfo.nFilledSize / MAX_EXPOSURE_NUM);
                            if (NULL != pImageBufferList[i])
                            {
                                stReconstructInfo.stOutputImageInfo[i].pImageBuf = pImageBufferList[i];
                                stReconstructInfo.stOutputImageInfo[i].nImageBufSize = stFrameInfo.nFilledSize / MAX_EXPOSURE_NUM;
                            }
                            else
                            {
                                printf("threadFunc malloc faild, no resources [0x#%x]\n", MV_FG_ERR_RESOURCE_EXHAUSTED);
                                return ;
                            }
                        }

                        nImageBufferSize = stFrameInfo.nFilledSize / MAX_EXPOSURE_NUM;
                    }

                    stReconstructInfo.stInputImageInfo.nWidth = stFrameInfo.nWidth;
                    stReconstructInfo.stInputImageInfo.nHeight = stFrameInfo.nHeight;
                    stReconstructInfo.stInputImageInfo.enPixelType = stFrameInfo.enPixelType;
                    stReconstructInfo.stInputImageInfo.pImageBuf = (unsigned char*)stFrameInfo.pBuffer;
                    stReconstructInfo.stInputImageInfo.nImageBufLen = stFrameInfo.nFilledSize;
                    stReconstructInfo.enReconstructMode = RECONSTRUCT_MODE_SPLIT_BY_LINE_2;

                    nRet = MV_FG_ReconstructImage(m_hStream, &stReconstructInfo);
                    if (MV_FG_SUCCESS != nRet)
                    {
                        continue;
                    }

                    for (unsigned int i = 0; i < MAX_EXPOSURE_NUM; i++)
                    {
                        // 配置显示图像的参数
                        memset(&stDisplayInfo, 0, sizeof(MV_FG_DISPLAY_FRAME_INFO));
                        stDisplayInfo.nWidth        = stReconstructInfo.stOutputImageInfo[i].nWidth;
                        stDisplayInfo.nHeight       = stReconstructInfo.stOutputImageInfo[i].nHeight;
                        stDisplayInfo.enPixelType   = stReconstructInfo.stOutputImageInfo[i].enPixelType;
                        stDisplayInfo.pImageBuf     = (unsigned char*)stReconstructInfo.stOutputImageInfo[i].pImageBuf;
                        stDisplayInfo.nImageBufLen  = stReconstructInfo.stOutputImageInfo[i].nImageBufLen;
                        nRet = MV_FG_DisplayOneFrame(m_hStream, m_hWnd[i], &stDisplayInfo);
                        if (MV_FG_SUCCESS != nRet)
                        {
                            MV_FG_ReleaseFrameBuffer(m_hStream, &stFrameInfo);
                            printf("threadFunc DisplayOneFrame faild nRet %d\n", nRet);
                            break;
                        }
                    }

                    // 有其中任意一张图像显示失败则退出
                    if (MV_FG_SUCCESS != nRet)
                    {
                        break;
                    }
                }
                else    // 不满足分时频闪条件
                {
                    // 配置显示图像的参数
                    memset(&stDisplayInfo, 0, sizeof(MV_FG_DISPLAY_FRAME_INFO));
                    stDisplayInfo.nWidth        = stFrameInfo.nWidth;
                    stDisplayInfo.nHeight       = stFrameInfo.nHeight;
                    stDisplayInfo.enPixelType   = stFrameInfo.enPixelType;
                    stDisplayInfo.pImageBuf     = (unsigned char*)stFrameInfo.pBuffer;
                    stDisplayInfo.nImageBufLen  = stFrameInfo.nFilledSize;
                    nRet = MV_FG_DisplayOneFrame(m_hStream, m_hWnd[0], &stDisplayInfo);
                    if (MV_FG_SUCCESS != nRet)
                    {
                        MV_FG_ReleaseFrameBuffer(m_hStream, &stFrameInfo);
                        printf("threadFunc DisplayOneFrame faild nRet %d\n", nRet);
                        break;
                    }
                }

                MV_FG_ReleaseFrameBuffer(m_hStream, &stFrameInfo);
            }
            else
            {
                if (TRIGGER_MODE_ON ==  m_nTriggerMode)
                {
                    usleep(5000);    
                }
            }
        }
        else
        {
            usleep(5000);      
        }
    }

    for (unsigned int i = 0; i < MAX_EXPOSURE_NUM; i++)
    {
        if (NULL != pImageBufferList[i])
        {
            free(pImageBufferList[i]);
            pImageBufferList[i] = NULL;
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
}


void MainWindow::on_bnIntEnum_clicked()
{
    int nRet = 0;

    bool bChanged = false;
    nRet = MV_FG_UpdateInterfaceList(MV_FG_GEV_INTERFACE | MV_FG_CXP_INTERFACE | MV_FG_CAMERALINK_INTERFACE | MV_FG_XoF_INTERFACE, &bChanged);
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
    ui->bnIntOpen->setEnabled(true);
}

void MainWindow::on_bnIntOpen_clicked()
{
    int nIndex = ui->ComboInterfaces->currentIndex();
    if (m_bOpenIF)
    {
        ShowErrorMsg("already open, close Interface first", 0);
        return;
    }

    int nRet = MV_FG_SUCCESS;
    nRet = MV_FG_OpenInterface(nIndex, &m_hInterface);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Open Interface fail", nRet);
        return;
    }

    m_bOpenIF = true;

    ui->bnIntOpen->setEnabled(false);
    ui->bnIntClose->setEnabled(true);
    ui->bnCamEnum->setEnabled(true);
}

void MainWindow::on_bnIntClose_clicked()
{
    int nRet = MV_FG_SUCCESS;
 
    if(m_hDevice)
    {
        on_bnCamClose_clicked();
    }

    // ch:关闭采集卡 | en:Close interface
    if (m_hInterface)
    {
        nRet = MV_FG_CloseInterface(m_hInterface);
        if (MV_FG_SUCCESS != nRet)
        {
            ShowErrorMsg("Close Interface fail", nRet);
        }
        m_hInterface = NULL;
    }


    m_bOpenIF = false;
    ui->bnIntOpen->setEnabled(true);
    ui->bnIntClose->setEnabled(false);
    ui->bnCamEnum->setEnabled(false);
    
    FlushControlButton(false, false);
    ui->bnCamOpen->setEnabled(false);
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
        ui->bnCamOpen->setEnabled(true);
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

    m_bOpenDevice = true;

    nRet = GetTriggerMode();
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Get Trigger Mode Failed", nRet);
    }

    nRet = GetTriggerSource();
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Get Trigger Source Failed", nRet);
    }

    ui->bnCamClose->setEnabled(true);
    ui->bnStartGrabbing->setEnabled(true);
}

void MainWindow::on_bnCamClose_clicked()
{
    int nIndex = ui->ComboDevices->currentIndex();

    if (!m_bOpenDevice)
    {
        ShowErrorMsg("not open device", 0);
        return;
    }

    //stop stream first
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


int MainWindow::SetTriggerMode()
{
    return MV_FG_SetEnumValue(m_hDevice, "TriggerMode", (unsigned int)m_nTriggerMode);
}               

int MainWindow::GetTriggerMode()
{
    MV_FG_ENUMVALUE stEnumValue = { 0 };

    int nRet = MV_FG_GetEnumValue(m_hDevice, "TriggerMode", &stEnumValue);
    if (MV_FG_SUCCESS != nRet)
    {
        return nRet;
    }

    m_nTriggerMode = stEnumValue.nCurValue;
    if (TRIGGER_MODE_ON ==  m_nTriggerMode)
    {
        on_RadioTrigger_clicked();
    }
    else
    {
        m_nTriggerMode = TRIGGER_MODE_OFF;
        on_RadioContinus_clicked();
    }

    return MV_FG_SUCCESS;
}

int MainWindow::SetTriggerSource()
{
    if (ui->CheckSoftTrigger->isChecked())
    {
        m_nTriggerSource = TRIGGER_SOURCE_SOFTWARE;
        int nRet = MV_FG_SetEnumValue(m_hDevice, "TriggerSource", (unsigned int)m_nTriggerSource);
        if (MV_FG_SUCCESS != nRet)
        {
            ShowErrorMsg("Set Software Trigger Failed", nRet);
            return nRet;
        }
        ui->bnSoftTriggerOnce->setEnabled(true);
    }
    else
    {
        m_nTriggerSource = TRIGGER_SOURCE_LINE0;
        int nRet = MV_FG_SetEnumValue(m_hDevice, "TriggerSource", (unsigned int)m_nTriggerSource);
        if (MV_FG_SUCCESS != nRet)
        {
            ShowErrorMsg("Set Hardware Trigger Failed", nRet);
            return nRet;
        }
        ui->bnSoftTriggerOnce->setEnabled(false);
    }

    return MV_FG_SUCCESS;
}   

int MainWindow::GetTriggerSource()
{
    MV_FG_ENUMVALUE stEnumValue = { 0 };

    int nRet = MV_FG_GetEnumValue(m_hDevice, "TriggerSource", &stEnumValue);
    if (MV_FG_SUCCESS != nRet)
    {
        return nRet;
    }

    if (TRIGGER_SOURCE_SOFTWARE == stEnumValue.nCurValue)
    {
        m_bSoftWareTriggerCheck = true;
    }
    else
    {
        m_bSoftWareTriggerCheck = false;
    }

    return MV_FG_SUCCESS;
}

void MainWindow::on_RadioContinus_clicked()
{
    ui->RadioContinus->setChecked(true);
    ui->RadioTrigger->setChecked(false);

    m_nTriggerMode = TRIGGER_MODE_OFF;

    int nRet = SetTriggerMode();
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Set Continus Mode Failed", nRet);
        return;
    }

    ui->bnSoftTriggerOnce->setEnabled(false);
}

void MainWindow::on_RadioTrigger_clicked()
{
    ui->RadioContinus->setChecked(false);
    ui->RadioTrigger->setChecked(true);
    m_nTriggerMode = TRIGGER_MODE_ON;

    int nRet = SetTriggerMode();
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Set Trigger Mode Failed", nRet);
        return;
    }

    if (true == m_bStartGrabbing)
    {
        if (true == m_bSoftWareTriggerCheck)
        {
            ui->bnSoftTriggerOnce->setEnabled(true);
        }
    }
}

void MainWindow::on_CheckSoftTrigger_clicked()
{
    int nRet = SetTriggerSource();
    if (MV_FG_SUCCESS != nRet)
    {
        return;
    }
}

void MainWindow::on_bnSoftTriggerOnce_clicked()
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
