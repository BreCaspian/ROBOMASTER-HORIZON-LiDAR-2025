#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) 
: QMainWindow(parent)
, ui(new Ui::MainWindow)
, m_bOpenOneIF(false)
, m_bStartGrabbing(false)  
, m_bFrameRateCanSet(false)
, m_bExposureTimeCanSet(false)
{
    ui->setupUi(this);

    m_hWnd = (void *)ui->widgetDisplay->winId();

    m_nInterfaceNum = 0;
    m_hInterface = NULL;
    for (unsigned int i = 0; i < MAX_INTERFACE_NUM; ++i)
    {
        m_bOpenIF[i] = false;
    }

    m_nDeviceNum = 0;
    for (unsigned int i = 0; i < MAX_DEVICE_NUM; ++i)
    {
        m_stDevHandle[i].hDevice = NULL;
        m_stDevHandle[i].hInterface = NULL;
        m_stDevHandle[i].bOpenDevice = false;
        m_stDevHandle[i].bStartGrabbing = false;
        m_stDevHandle[i].hStream = NULL;
        for (unsigned int j = 0; j < BUFFER_NUMBER; ++j)
        {
            m_stDevHandle[i].hBuffer[i] = NULL;
            m_stDevHandle[i].pBuffer[j] = NULL;
        }
        m_stDevHandle[i].nThreadID = NULL;
    }
    m_nDevIndex = -1;

    m_hWndDisplay[0] = (void *)ui->widgetDisplay->winId();
    m_hWndDisplay[1] = (void *)ui->widgetDisplay_2->winId();
    m_hWndDisplay[2] = (void *)ui->widgetDisplay_3->winId();
    m_hWndDisplay[3] = (void *)ui->widgetDisplay_4->winId();

    m_lCamInfo.clear();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if(m_bStartGrabbing)
    {
        on_bnStopGrabbing_clicked();
    }
    
    if(m_bOpenOneIF)
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
        if (-1 == pMainWindow->m_nDevIndex)
        {
            return NULL;
        }
        pMainWindow->ThreadFun(pMainWindow->m_nDevIndex);
    }

    return NULL;
}

void MainWindow::ThreadFun(int nCurCameraIndex)
{
    MV_FG_BUFFER_INFO stFrameInfo = {0};
    BUFFERHANDLE hBuffer = NULL;
    int nRet = MV_FG_SUCCESS;

    while (1)
    {
        if (!m_stDevHandle[nCurCameraIndex].bStartGrabbing)
        {
            break;
        }
        memset(&stFrameInfo, 0, sizeof(MV_FG_BUFFER_INFO));
        nRet = MV_FG_GetImageBuffer(m_stDevHandle[nCurCameraIndex].hStream, &hBuffer, 5000);
        if (MV_FG_SUCCESS == nRet)
        {
            nRet = MV_FG_GetBufferInfo(hBuffer, &stFrameInfo);
            if (MV_FG_SUCCESS == nRet)
            {
                printf("Index[%d] FrameNum:%ld Width:%d Height:%d, nImageBufLen:%d, enPixelType:%x\n", nCurCameraIndex, stFrameInfo.nFrameID, stFrameInfo.nWidth, 
                                                                                stFrameInfo.nHeight, stFrameInfo.nFilledSize, stFrameInfo.enPixelType);
                fflush(stdout);
                
                //display one frame
                MV_FG_DISPLAY_FRAME_INFO stDisplayInfo = {0};
                stDisplayInfo.pImageBuf = (unsigned char *)stFrameInfo.pBuffer;
                stDisplayInfo.nWidth = stFrameInfo.nWidth;
                stDisplayInfo.nHeight = stFrameInfo.nHeight;
                stDisplayInfo.nImageBufLen = stFrameInfo.nFilledSize;
                stDisplayInfo.enPixelType = stFrameInfo.enPixelType;
                nRet = MV_FG_DisplayOneFrame(m_stDevHandle[nCurCameraIndex].hDevice, m_hWndDisplay[nCurCameraIndex], &stDisplayInfo);
                if(MV_FG_SUCCESS != nRet)
                {
                    printf("Display fail, nRet[%x]\n", nRet);
                }
            }
            nRet = MV_FG_QueueBuffer(hBuffer);
            if (MV_FG_SUCCESS != nRet)
            {
                printf("queue bufer fail, nRet[%x]\n", nRet);
                fflush(stdout);
            }
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
        FlushParamConfiguration(false);
    }
    else if ((bOpen) && (!bStart))
    {
        ui->bnCamOpen->setEnabled(false);
        ui->bnCamClose->setEnabled(true);
        ui->bnStartGrabbing->setEnabled(true);
        ui->bnStopGrabbing->setEnabled(false);
        FlushParamConfiguration(true);
    }
    else if ((bOpen) && (bStart))
    {
        ui->bnCamOpen->setEnabled(false);
        ui->bnCamClose->setEnabled(true);
        ui->bnStartGrabbing->setEnabled(false);
        ui->bnStopGrabbing->setEnabled(true);
        FlushParamConfiguration(false);
    }
}

void MainWindow::FlushParamConfiguration(bool bOpen)
{
    CheckExposureTimeCanSet();
    CheckFrameRateCanSet();

    if (bOpen)
    {
        ui->LeExposure->setEnabled(m_bExposureTimeCanSet);
        ui->leFrameRate->setEnabled(m_bFrameRateCanSet);
        ui->bnGetParam->setEnabled(true);
        ui->bnSetParam->setEnabled(true);
    }
    else
    {
        ui->LeExposure->setEnabled(false);
        ui->leFrameRate->setEnabled(false);
        ui->bnGetParam->setEnabled(false);
        ui->bnSetParam->setEnabled(false);
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
    if (m_bOpenIF[nIndex])
    {
        ShowErrorMsg("already open", 0);
        return;
    }
    else
    {
        for (unsigned int i = 0; i < MAX_INTERFACE_NUM; ++i)
        {
            if (m_bOpenIF[i])
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

    m_bOpenIF[nIndex] = true;
    ui->bnIntOpen->setEnabled(false);
    ui->bnIntClose->setEnabled(true);

    ui->bnCamEnum->setEnabled(true);
    m_bOpenOneIF = true;
}

void MainWindow::on_bnIntClose_clicked()
{
    int nRet = MV_FG_SUCCESS;
    int nIndex = ui->ComboInterfaces->currentIndex();

    //close all devices
    for (unsigned int i = 0; i < ui->ComboDevices->count(); ++i)
    {
        if (m_stDevHandle[i].bOpenDevice)
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

    m_bOpenIF[nIndex] = false;
    m_bOpenOneIF = false;
    m_hInterface = NULL;
    ui->bnIntOpen->setEnabled(true);
    ui->bnIntClose->setEnabled(false);

    ui->bnCamEnum->setEnabled(false);
    FlushControlButton(false, false);
    ui->bnCamOpen->setEnabled(false);
}

void MainWindow::on_bnCamEnum_clicked()
{
    int nRet = MV_FG_SUCCESS;
    m_lCamInfo.clear();

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

            m_stDevHandle[i].hInterface = m_hInterface;
        }
        ui->bnCamOpen->setEnabled(true);
    }
}

void MainWindow::on_bnCamOpen_clicked()
{
    int nIndex = ui->ComboDevices->currentIndex();
    if (m_stDevHandle[nIndex].bOpenDevice)
    {
        ShowErrorMsg("already open", 0);
        return;
    }
    int nRet = MV_FG_SUCCESS;
    nRet = MV_FG_OpenDevice(m_hInterface, nIndex, &(m_stDevHandle[nIndex].hDevice));
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Open Device fail", nRet);
        return;
    }
    MV_FG_SetEnumValue(m_stDevHandle[nIndex].hDevice, "AcquisitionMode", 2);
    MV_FG_SetEnumValue(m_stDevHandle[nIndex].hDevice, "TriggerMode", 0);

    m_stDevHandle[nIndex].bOpenDevice = true;

    FlushControlButton(true, false);

    ui->LeExposure->setEnabled(true);
    ui->leFrameRate->setEnabled(true);
    ui->bnGetParam->setEnabled(true);
    ui->bnSetParam->setEnabled(true);

    //GetParam
    on_bnGetParam_clicked();
}

void MainWindow::on_bnCamClose_clicked()
{
    int nIndex = ui->ComboDevices->currentIndex();

    if (!m_stDevHandle[nIndex].bOpenDevice)
    {
        ShowErrorMsg("not open device", 0);
        return;
    }

    //stop stream first
    if (m_stDevHandle[nIndex].bStartGrabbing)
    {
        on_bnStopGrabbing_clicked();
    }

    int nRet = MV_FG_SUCCESS;
    nRet = MV_FG_CloseDevice(m_stDevHandle[nIndex].hDevice);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Close Device fail", nRet);
        return;
    }
    m_stDevHandle[nIndex].hDevice = NULL;
    m_stDevHandle[nIndex].bOpenDevice = false;
    FlushControlButton(false, false);

    ui->LeExposure->setEnabled(false);
    ui->leFrameRate->setEnabled(false);
    ui->bnGetParam->setEnabled(false);
    ui->bnSetParam->setEnabled(false);
}

void MainWindow::on_bnStartGrabbing_clicked()
{
    int nIndex = ui->ComboDevices->currentIndex();

    if (!m_stDevHandle[nIndex].bOpenDevice)
    {
        ShowErrorMsg("not open device", 0);
        return;
    }

    int nRet = MV_FG_SUCCESS;
    nRet = MV_FG_SetEnumValueByString(m_stDevHandle[nIndex].hDevice, "TriggerMode", "Off");
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Set TriggerMode fail", nRet);
        return;
    }

    unsigned int nStreamNum = 0;
    nRet = MV_FG_GetNumStreams(m_stDevHandle[nIndex].hDevice, &nStreamNum);
    if (MV_FG_SUCCESS != nRet || 0 == nStreamNum)
    {
        ShowErrorMsg("Get Stream Number fail", nRet);
        return;
    }

    nRet = MV_FG_OpenStream(m_stDevHandle[nIndex].hDevice, 0, &(m_stDevHandle[nIndex].hStream));
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Open Stream fail", nRet);
        return;
    }

#if 0
    //not compliment register image callback
    nRet = MV_FG_SetImageNum(m_stDevHandle[nIndex].hStream, BUFFER_NUMBER);
    if(MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Set Buffer Num fail", nRet);
        nRet = MV_FG_CloseStream(m_stDevHandle[nIndex].hStream);
        if(MV_FG_SUCCESS != nRet)
        {
            ShowErrorMsg("Close Stream fail", nRet);
        }
        m_stDevHandle[nIndex].hStream = NULL;
        return;
    }

#endif

    unsigned int nImagePayloadSize = 0;
    nRet = MV_FG_GetPayloadSize(m_stDevHandle[nIndex].hStream, &nImagePayloadSize);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Get PayloadSize fail", nRet);
        return;
    }

    for (unsigned int i = 0; i < BUFFER_NUMBER; ++i)
    {
        m_stDevHandle[nIndex].pBuffer[i] = malloc(nImagePayloadSize);
        if (NULL == m_stDevHandle[nIndex].pBuffer[i])
        {
            ShowErrorMsg("malloc fail", nRet);
            return;
        }

        nRet = MV_FG_AnnounceBuffer(m_stDevHandle[nIndex].hStream, m_stDevHandle[nIndex].pBuffer[i], nImagePayloadSize, NULL, &(m_stDevHandle[nIndex].hBuffer[i]));
        if (MV_FG_SUCCESS != nRet)
        {
            ShowErrorMsg("Announce Buffer fail", nRet);
            return;
        }
    }

    nRet = MV_FG_FlushQueue(m_stDevHandle[nIndex].hStream, MV_FG_BUFFER_QUEUE_ALL_TO_INPUT);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Flush Queue fail", nRet);
        return;
    }

    nRet = MV_FG_StartAcquisition(m_stDevHandle[nIndex].hStream);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Start Acquisition fail", nRet);
        nRet = MV_FG_CloseStream(m_stDevHandle[nIndex].hStream);
        if (MV_FG_SUCCESS != nRet)
        {
            ShowErrorMsg("Close Stream fail", nRet);
        }
        m_stDevHandle[nIndex].hStream = NULL;
        return;
    }
    m_stDevHandle[nIndex].bStartGrabbing = true;
    m_nDevIndex = nIndex;

    //pthread_t nThreadID;
    nRet = pthread_create(&(m_stDevHandle[nIndex].nThreadID), NULL, WorkThread, this);
    if (0 != nRet)
    {
        ShowErrorMsg("pthread create fail", 0);
        return;
    }

    FlushControlButton(true, true);
    m_bStartGrabbing = true;
}

void MainWindow::on_bnStopGrabbing_clicked()
{
    int nIndex = ui->ComboDevices->currentIndex();

    if (!m_stDevHandle[nIndex].bStartGrabbing)
    {
        ShowErrorMsg("not grabbing", 0);
        return;
    }

    m_stDevHandle[nIndex].bStartGrabbing = false;

    int nRet = MV_FG_SUCCESS;

    //wait pthread exit
    pthread_join(m_stDevHandle[nIndex].nThreadID, NULL);
    m_stDevHandle[nIndex].nThreadID = NULL;

    nRet = MV_FG_StopAcquisition(m_stDevHandle[nIndex].hStream);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Stop Acquisition fail", nRet);
        return;
    }

    nRet = MV_FG_FlushQueue(m_stDevHandle[nIndex].hStream, MV_FG_BUFFER_QUEUE_ALL_DISCARD);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Flush Queue fail", nRet);
        return;
    }

    for (unsigned int i = 0; i < BUFFER_NUMBER; ++i)
    {
        if (NULL != m_stDevHandle[nIndex].hBuffer[i])
        {
            nRet = MV_FG_RevokeBuffer(m_stDevHandle[nIndex].hStream, m_stDevHandle[nIndex].hBuffer[i], NULL, NULL);
            if (MV_FG_SUCCESS != nRet)
            {
                ShowErrorMsg("Revoke Buffer fail", nRet);
                return;
            }
            m_stDevHandle[nIndex].hBuffer[i] = NULL;

            if (NULL != m_stDevHandle[nIndex].pBuffer[i])
            {
                free(m_stDevHandle[nIndex].pBuffer[i]);
                m_stDevHandle[nIndex].pBuffer[i] = NULL;
            }
        }
    }

    nRet = MV_FG_CloseStream(m_stDevHandle[nIndex].hStream);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Close Stream fail", nRet);
        return;
    }
    m_stDevHandle[nIndex].hStream = NULL;
    m_bStartGrabbing = false;
    FlushControlButton(true, false);
}

void MainWindow::CheckExposureTimeCanSet()
{
    MV_FG_NODE_ACCESS_MODE enAccessMode = ACCESS_MODE_NI;
    int nIndex = ui->ComboDevices->currentIndex();
    
    int nRet = MV_FG_SUCCESS;
    MV_FG_FLOATVALUE stFloatValue;
    memset(&stFloatValue, 0, sizeof(MV_FG_FLOATVALUE));

    nRet = MV_FG_GetNodeAccessMode(m_stDevHandle[nIndex].hDevice, "ExposureTime", &enAccessMode);
    if(MV_FG_SUCCESS != nRet)
    {
        return;
    }
    if(enAccessMode == ACCESS_MODE_WO || enAccessMode == ACCESS_MODE_RW)
    {
        m_bExposureTimeCanSet = true;
    }
    else
    {
        m_bExposureTimeCanSet = false;
    }
}

void MainWindow::CheckFrameRateCanSet()
{
    MV_FG_NODE_ACCESS_MODE enAccessMode = ACCESS_MODE_NI;
    int nIndex = ui->ComboDevices->currentIndex();
    
    int nRet = MV_FG_SUCCESS;
    MV_FG_FLOATVALUE stFloatValue;
    memset(&stFloatValue, 0, sizeof(MV_FG_FLOATVALUE));

    nRet = MV_FG_GetNodeAccessMode(m_stDevHandle[nIndex].hDevice, "AcquisitionFrameRate", &enAccessMode);
    if(MV_FG_SUCCESS != nRet)
    {
        return;
    }
    if(enAccessMode == ACCESS_MODE_WO || enAccessMode == ACCESS_MODE_RW)
    {
        m_bFrameRateCanSet = true;
    }
    else
    {
        m_bFrameRateCanSet = false;
    }
}

void MainWindow::on_bnGetParam_clicked()
{
    int nIndex = ui->ComboDevices->currentIndex();
    int nRet = MV_FG_SUCCESS;
    MV_FG_FLOATVALUE stFloatValue;
    memset(&stFloatValue, 0, sizeof(MV_FG_FLOATVALUE));
    if (!m_stDevHandle[nIndex].bOpenDevice)
    {
        ui->leFrameRate->clear();
        ui->LeExposure->clear();
        return;
    }
    CheckExposureTimeCanSet();
    CheckFrameRateCanSet();

    nRet = MV_FG_GetFloatValue(m_stDevHandle[nIndex].hDevice, "ExposureTime", &stFloatValue);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Get ExposureTime fail", nRet);
    }
    else
    {
        ui->LeExposure->setText(QString("%1").arg(stFloatValue.fCurValue));
    }
    if(!m_bExposureTimeCanSet)
    {
        ui->LeExposure->setEnabled(false);
    }

    nRet = MV_FG_GetFloatValue(m_stDevHandle[nIndex].hDevice, "ResultingFrameRate", &stFloatValue);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Get FrameRate fail", nRet);
    }
    else
    {
        ui->leFrameRate->setText(QString("%1").arg(stFloatValue.fCurValue));
    }

    if(!m_bFrameRateCanSet)
    {
        ui->leFrameRate->setEnabled(false);
    }
}

void MainWindow::on_bnSetParam_clicked()
{
    bool bFrameRateEnableCanSet = false;
    bool bExposureAutoCanSet = false;
    int nIndex = ui->ComboDevices->currentIndex();
    int nRet = MV_FG_SUCCESS;
    MV_FG_NODE_ACCESS_MODE enAccessMode = ACCESS_MODE_NI;

    if (!m_stDevHandle[nIndex].bOpenDevice)
    {
        ShowErrorMsg("Camera not open", 0);
        return;
    }
    
    nRet = MV_FG_GetNodeAccessMode(m_stDevHandle[nIndex].hDevice, "ExposureAuto", &enAccessMode);
    if(MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Camera does not have node ExposureAuto", 0);
        return;
    }
    if(enAccessMode == ACCESS_MODE_WO || enAccessMode == ACCESS_MODE_RW)
    {
        bExposureAutoCanSet = true;
    }

    if(bExposureAutoCanSet)
    {
        nRet = MV_FG_SetEnumValue(m_stDevHandle[nIndex].hDevice, "ExposureAuto", 0);
        if (MV_FG_SUCCESS != nRet)
        {
            ShowErrorMsg("Set ExposureTime fail", nRet);
        }
    }
    
    if(m_bExposureTimeCanSet)
    {
        nRet = MV_FG_SetFloatValue(m_stDevHandle[nIndex].hDevice, "ExposureTime", ui->LeExposure->text().toFloat());
        if (MV_FG_SUCCESS != nRet)
        {
            ShowErrorMsg("Set ExposureTime fail", nRet);
        }
    }


    //////////////////////////////////////

    nRet = MV_FG_GetNodeAccessMode(m_stDevHandle[nIndex].hDevice, "AcquisitionFrameRateEnable", &enAccessMode);
    if(MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Camera does not have node AcquisitionFrameRateEnable", 0);
        return;
    }
    if(enAccessMode == ACCESS_MODE_WO || enAccessMode == ACCESS_MODE_RW)
    {
        bFrameRateEnableCanSet = true;
    }

    if(bFrameRateEnableCanSet)
    {
        nRet = MV_FG_SetBoolValue(m_stDevHandle[nIndex].hDevice, "AcquisitionFrameRateEnable", true);
        if (MV_FG_SUCCESS != nRet)
        {
            ShowErrorMsg("Set ExposureTime fail", nRet);
        }
    }
    if(m_bFrameRateCanSet)
    {
        nRet = MV_FG_SetFloatValue(m_stDevHandle[nIndex].hDevice, "AcquisitionFrameRate", ui->leFrameRate->text().toFloat());
        if (MV_FG_SUCCESS != nRet)
        {
            ShowErrorMsg("Set ExposureTime fail", nRet);
        }
    }

}

void MainWindow::on_ComboInterfaces_currentIndexChanged(int index)
{
    if (!m_bOpenIF[index])
    {
        ui->ComboDevices->clear();
        ui->bnIntClose->setEnabled(false);
        ui->bnIntOpen->setEnabled(true);
        ui->bnCamEnum->setEnabled(false);
        ui->bnCamOpen->setEnabled(false);
        ui->bnCamClose->setEnabled(false);
        ui->bnStartGrabbing->setEnabled(false);
        ui->bnStopGrabbing->setEnabled(false);
        FlushParamConfiguration(false);
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
    FlushControlButton(m_stDevHandle[index].bOpenDevice, m_stDevHandle[index].bStartGrabbing);
    on_bnGetParam_clicked();
    FlushParamConfiguration(true);
}
