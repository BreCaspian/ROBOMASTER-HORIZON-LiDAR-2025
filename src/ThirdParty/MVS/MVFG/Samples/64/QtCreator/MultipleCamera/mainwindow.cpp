#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) 
: QMainWindow(parent)
, ui(new Ui::MainWindow)
, m_bStartGrabbing(false) 
, m_nCurCameraIndex(-1)
, m_nCurListIndex(-1)
, m_nInterfaceNum(0)
, m_bOpenIF(false)
, m_bOpenDevice(false)
{
    ui->setupUi(this);

    for (int i = 0; i < MAX_INTERFACE_NUM; i++)
    {
        for (int j = 0; j < MAX_DEVICE_NUM; j++)
        {
            m_bCamCheck[i][j] = false;
            m_bDeviceGrabbingFlag[i][j] = false;
        } 
    }
    memset(m_stInterface, 0, sizeof(IF_INFO) * MAX_INTERFACE_NUM);
    
    m_hWndDisplay[0] = (void *)ui->IF1Cam1W->winId();
    m_hWndDisplay[1] = (void *)ui->IF1Cam2W->winId();
    m_hWndDisplay[2] = (void *)ui->IF1Cam3W->winId();
    m_hWndDisplay[3] = (void *)ui->IF1Cam4W->winId();
    m_hWndDisplay[4] = (void *)ui->IF2Cam1W->winId();
    m_hWndDisplay[5] = (void *)ui->IF2Cam2W->winId();
    m_hWndDisplay[6] = (void *)ui->IF2Cam3W->winId();
    m_hWndDisplay[7] = (void *)ui->IF2Cam4W->winId();
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
    
    if(m_bOpenIF)
    {
        on_bnIFClose_clicked();
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
        int nCurCameraIndex = pMainWindow->m_nCurCameraIndex;
        int nCurListInde    = pMainWindow->m_nCurListIndex;
        pMainWindow->m_nCurCameraIndex = -1;
        pMainWindow->m_nCurListIndex   = -1;
        printf("WorkThread nCurListInde %d, nCurCameraIndex %d\n", nCurListInde, nCurCameraIndex);
        pMainWindow->ThreadFun(nCurListInde, nCurCameraIndex);
    }
    return NULL;
}

void MainWindow::ThreadFun(int nCurListIndex, int nCurCameraIndex)
{
    MV_FG_BUFFER_INFO stFrameInfo = {0};
    MV_FG_DISPLAY_FRAME_INFO stDisplayFrameInfo = {0};
    int nRet = MV_FG_SUCCESS;

    while(m_bStartGrabbing && m_bDeviceGrabbingFlag[nCurListIndex][nCurCameraIndex]) 
    {
        nRet = MV_FG_GetFrameBuffer(m_stInterface[nCurListIndex].hStream[nCurCameraIndex], &stFrameInfo, TIMEOUT);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Get Frame Buffer fail! DevIndex[%d], nRet[%#x]\r\n", nCurCameraIndex, nRet);
            continue;
        }

        
        if (NULL != stFrameInfo.pBuffer)
        {
            printf("FrameNumber:%d, Width:%d, Height:%d\n", stFrameInfo.nFrameID, stFrameInfo.nWidth, stFrameInfo.nHeight);

            stDisplayFrameInfo.pImageBuf = (unsigned char*)stFrameInfo.pBuffer;
            stDisplayFrameInfo.nImageBufLen = stFrameInfo.nFilledSize;
            stDisplayFrameInfo.nWidth = stFrameInfo.nWidth;
            stDisplayFrameInfo.nHeight = stFrameInfo.nHeight;
            stDisplayFrameInfo.enPixelType = stFrameInfo.enPixelType;

            printf("nCurListIndex:%d, nCurCameraIndex:%d, displayWinID %d\n", nCurListIndex, nCurCameraIndex, nCurListIndex * nCurCameraIndex + nCurCameraIndex);
 
            nRet =  MV_FG_DisplayOneFrame(m_stInterface[nCurListIndex].hStream[nCurCameraIndex], 
                        m_hWndDisplay[nCurListIndex == 0 ? nCurCameraIndex : nCurCameraIndex + 4], &stDisplayFrameInfo);
            if (MV_FG_SUCCESS != nRet)
            {
                printf("Display OneFrame fail! DevIndex[%d], nRet[%#x]\r\n", nCurCameraIndex, nRet);
            }

            // ch:将缓存放回输入队列 | en:Put the buffer back into the input queue
            nRet = MV_FG_ReleaseFrameBuffer(m_stInterface[nCurListIndex].hStream[nCurCameraIndex], &stFrameInfo);
            if (MV_FG_SUCCESS != nRet)
            {
                printf("Release frame buffer failed! %#x\n", nRet);
            }
        }
    }
    printf("thread [%d][%d] end ...\n", nCurListIndex, nCurCameraIndex);
}

void MainWindow::FlushControlButton(bool bIsCameraReady)
{
    ui->bnIFEnum->setEnabled(m_bOpenIF ? false : true);
    ui->bnIFOpen->setEnabled(m_bOpenIF ? false : (m_nInterfaceNum > 0 ? true : false));
    ui->bnIFClose->setEnabled(m_bOpenDevice ? false : m_bOpenIF ? true : false);
    ui->IFCombo1->setEnabled(m_bOpenIF ? false : (m_nInterfaceNum > 0 ? true : false));
    ui->IFCombo2->setEnabled(m_bOpenIF ? false : (m_nInterfaceNum > 0 ? true : false));

    ui->bnDevEnum->setEnabled(m_bOpenDevice ? false : m_bOpenIF ? true : false);
    ui->bnDevOpen->setEnabled(m_bOpenDevice ? false : (bIsCameraReady ? true : false));
    ui->bnDevClose->setEnabled((m_bOpenDevice && bIsCameraReady) ? true : false);
    ui->bnStartGrabbing->setEnabled(m_bStartGrabbing ? false : (m_bOpenDevice && bIsCameraReady) ? true : false);
    ui->bnStopGrabbing->setEnabled(m_bStartGrabbing ? true : false);
}


void MainWindow::on_bnIFEnum_clicked()
{
    int nRet = 0;

    bool bChanged = false;
    nRet = MV_FG_UpdateInterfaceList(MV_FG_GEV_INTERFACE | MV_FG_CXP_INTERFACE | MV_FG_CAMERALINK_INTERFACE | MV_FG_XoF_INTERFACE, &bChanged);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Enum interfaces fail", nRet);
        return;
    }
    m_nInterfaceNum = 0;

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
        ui->IFCombo1->clear();
        ui->IFCombo2->clear();

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

            ui->IFCombo1->addItem(strMsg); 
            ui->IFCombo2->addItem(strMsg); 
        }
    }

    if (m_nInterfaceNum > 0)
    {
        ui->IFCombo1->setCurrentIndex(0);
        ui->IFCombo2->setCurrentIndex(m_nInterfaceNum > 1 ? 1 : 0);
    }

    ui->bnIFOpen->setEnabled(true);
    FlushControlButton(false);
}

void MainWindow::on_bnIFOpen_clicked()
{
    int nRet = MV_FG_SUCCESS;
    int nInterfaceIndex[MAX_INTERFACE_NUM] = {0};
    nInterfaceIndex[0] = ui->IFCombo1->currentIndex();
    nInterfaceIndex[1] = ui->IFCombo2->currentIndex();

    // ch:打开采集卡，获得采集卡句柄 | en:Open interface, get handle
    nRet = MV_FG_OpenInterface((unsigned int)nInterfaceIndex[0], &m_stInterface[FIRST_INTERFACE].hInterface);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("List 1:Open interface failed", nRet);
        return;
    }
    
    if (nInterfaceIndex[FIRST_INTERFACE] != nInterfaceIndex[SECOND_INTERFACE])
    {
        nRet = MV_FG_OpenInterface((unsigned int)nInterfaceIndex[1], &m_stInterface[SECOND_INTERFACE].hInterface);
        if (MV_FG_SUCCESS != nRet)
        {
            ShowErrorMsg("List 2:Open interface failed", nRet);
            return;
        }
        
    }
    m_bOpenIF = true;

    ui->bnIFOpen->setEnabled(false);
    ui->bnIFClose->setEnabled(true);
    ui->bnDevEnum->setEnabled(true);
}

void MainWindow::on_bnIFClose_clicked()
{
    int nRet = MV_FG_SUCCESS;
    m_bStartGrabbing = false;

    on_bnDevClose_clicked();

    nRet = MV_FG_CloseInterface(m_stInterface[FIRST_INTERFACE].hInterface);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("List 1:Close interface fail! nRet[%#x]\r\n", nRet);
    }
    
    if (NULL != m_stInterface[SECOND_INTERFACE].hInterface)
    {
        nRet = MV_FG_CloseInterface(m_stInterface[SECOND_INTERFACE].hInterface);
        if (MV_FG_SUCCESS != nRet)
        {
            ShowErrorMsg("List 2:Close interface fail! nRet[%#x]\r\n", nRet);
        }
    }

    memset(m_stInterface, 0, sizeof(IF_INFO) * MAX_INTERFACE_NUM);
    m_bOpenIF = false;
    FlushControlButton(false);
}

void MainWindow::on_bnDevClose_clicked()
{
    on_bnStopGrabbing_clicked();

    int nRet = MV_FG_SUCCESS;
    for (int i = 0; i < MAX_INTERFACE_NUM; i++)
    {
        for (int j = 0; j < MAX_DEVICE_NUM; j++)
        {
            if (!m_stInterface[i].hDevice[j])
            {
                continue;
            }
        
            nRet = MV_FG_CloseDevice(m_stInterface[i].hDevice[j]);
            if (MV_FG_SUCCESS != nRet)
            {
                printf("List %d:Close device fail! DevIndex[%d], nRet[%#x]\r\n", i+1, j+1, nRet);
            }
            m_stInterface[i].hDevice[j] = NULL;
        }
    }

    m_bOpenDevice = false;
    FlushControlButton(true);

}

void MainWindow::on_bnDevEnum_clicked()
{
    int nRet = MV_FG_SUCCESS;
    unsigned int nDeviceNum[MAX_INTERFACE_NUM] = {0};           // ch:每张采集卡下设备数量 | en:Number of devices under each acquisition card
    bool bChanged[MAX_INTERFACE_NUM]  = {false};
    QString strTmp;
    QString strMsg;

    //enum devices
    nRet = MV_FG_UpdateDeviceList(m_stInterface[FIRST_INTERFACE].hInterface, &bChanged[0]);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Enum devices fail", nRet);
        return;
    }


    //get devices' num
    nRet = MV_FG_GetNumDevices(m_stInterface[FIRST_INTERFACE].hInterface, &nDeviceNum[0]);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("List 1: Get devices' num fail", nRet);
        return;
    }

    if (NULL != m_stInterface[SECOND_INTERFACE].hInterface)
    {
        nRet = MV_FG_UpdateDeviceList(m_stInterface[SECOND_INTERFACE].hInterface, &bChanged[1]);
        if (MV_FG_SUCCESS != nRet)
        {
            ShowErrorMsg("List 2:Enum devices failed", nRet);
            return;
        }

        nRet = MV_FG_GetNumDevices(m_stInterface[SECOND_INTERFACE].hInterface, &nDeviceNum[1]);
        if (MV_FG_SUCCESS != nRet)
        {
            ShowErrorMsg("List 2:Get device number failed", nRet);
            return;
        }
    }
    
    if (0 == nDeviceNum[FIRST_INTERFACE] && 0 == nDeviceNum[SECOND_INTERFACE])
    {
        ShowErrorMsg("No devices", 0);
        return;
    }

    if (bChanged[FIRST_INTERFACE] || bChanged[SECOND_INTERFACE])
    {
        // ch:向相机复选框添加设备信息 | en:Add device info in Check box
        unsigned int i = 0;
        for (i = 0; i < (nDeviceNum[FIRST_INTERFACE] + nDeviceNum[SECOND_INTERFACE]) && i < MAX_DEVICE_NUM * MAX_INTERFACE_NUM; i++)
        {
            char                strDevInfo[DEVICE_INFO_LEN] = { 0 };
            MV_FG_DEVICE_INFO   stDeviceInfo = { 0 };
            QString strMsg;
            QString strTmp;

            if (i < nDeviceNum[FIRST_INTERFACE])
            {
                nRet = MV_FG_GetDeviceInfo(m_stInterface[FIRST_INTERFACE].hInterface, i, &stDeviceInfo);
                if (MV_FG_SUCCESS != nRet)
                {
                    ShowErrorMsg("List 1:Get Devices info failed", nRet);
                    return;
                }

            }
            else
            {
                nRet = MV_FG_GetDeviceInfo(m_stInterface[SECOND_INTERFACE].hInterface, i - nDeviceNum[0], &stDeviceInfo);
                if (MV_FG_SUCCESS != nRet)
                {
                    ShowErrorMsg("List 2:Get Devices info failed", nRet);
                    return;
                }
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

            if(i < nDeviceNum[0]) 
            {
                printf("Enum Dev IF1Box i[%d]\n", i);
                strTmp.clear();
                strTmp.sprintf("IF%dCam%d", 1, i+1);
                QGroupBox *groupBox = ui->IF1Box;
                QCheckBox *checkBoxes = groupBox->findChild<QCheckBox *>(strTmp);
                if(NULL != checkBoxes)
                {
                    checkBoxes->setText(strMsg);
                    checkBoxes->setEnabled(true);
                }

                m_bCamCheck[FIRST_INTERFACE][i] = false;
                m_stInterface[FIRST_INTERFACE].nCurIndex[i] = i;
            }
            else 
            {
                printf("Enum Dev IF1Box i[%d]\n", i);
                strTmp.clear();
                strTmp.sprintf("IF%dCam%d", 2, i - nDeviceNum[0] + 1);
                QGroupBox *groupBox = ui->IF2Box;
                QCheckBox *checkBoxes = groupBox->findChild<QCheckBox *>(strTmp);
                if(NULL != checkBoxes)
                {
                    checkBoxes->setText(strMsg);
                    checkBoxes->setEnabled(true);
                }
                m_bCamCheck[SECOND_INTERFACE][i - nDeviceNum[FIRST_INTERFACE]] = false;
                m_stInterface[SECOND_INTERFACE].nCurIndex[i - nDeviceNum[FIRST_INTERFACE]] = i - nDeviceNum[FIRST_INTERFACE];
            }
        }


        ui->bnDevOpen->setEnabled(true);
    }

    for (int i = 0; i< MAX_INTERFACE_NUM; i++)
    {
        printf("List %d:Total Find %d devices!\r\n", i + 1, nDeviceNum[i]);
    }
    FlushControlButton(true);
}

void MainWindow::on_bnDevOpen_clicked()
{
    if (true == m_bOpenDevice)
    {
        return;
    }

    int nRet = MV_FG_SUCCESS;
    bool bHaveCheck[MAX_INTERFACE_NUM]  = {false};
    QString strTmp;

    for (int i = 0; i < MAX_INTERFACE_NUM; i++)
    {
        QGroupBox *groupBox = ui->IF1Box;
        if(i == 1) //TODO stupid!
        {
            groupBox = ui->IF2Box;
        }

        for (unsigned int nDeviceIndex = 0; nDeviceIndex < MAX_DEVICE_NUM; nDeviceIndex++)
        {   
            // 遍历 QGroupBox下的 QCheckBox判断是否有复选框被选中
            strTmp.clear();
            strTmp.sprintf("IF%dCam%d", i + 1, nDeviceIndex + 1 );
            QCheckBox *checkBoxes = groupBox->findChild<QCheckBox *>(strTmp);
            if(NULL == checkBoxes)
            {
                printf("checkBoxes is NULL IF = %d, Dev = %d, continue\n", i+1, nDeviceIndex+1);
                continue;
            }
            else if(false == checkBoxes->isChecked())
            {
                printf("IF = %d, Dev = %d, continue\n", i+1, nDeviceIndex);
                continue;
            }

            bHaveCheck[i] = true;
            int nRet = MV_FG_OpenDevice(m_stInterface[i].hInterface, m_stInterface[i].nCurIndex[nDeviceIndex], &(m_stInterface[i].hDevice[nDeviceIndex]));
            if (MV_FG_SUCCESS != nRet)
            {
                printf("List %d:Open device failed! DevIndex[%d], nRet[%#x]\r\n", i + 1, nDeviceIndex+1, nRet);
                m_stInterface[i].hDevice[nDeviceIndex] = NULL;
                continue;
            }
            else
            {
                printf("List %d:Open device success! DevIndex[%d]", i + 1, nDeviceIndex + 1);
                m_bOpenDevice = true;
            }

            // ch:设置连续采集模式 | en:Set Continuous Aquisition Mode
            MV_FG_SetEnumValue(m_stInterface[i].hDevice[nDeviceIndex], "AcquisitionMode", 2);  // 0 - SingleFrame, 2 - Continuous
            MV_FG_SetEnumValue(m_stInterface[i].hDevice[nDeviceIndex], "TriggerMode", (unsigned int)0);  // 0 - Trigger off, 1 - Trigger on
            
        }
    }

    if (true == m_bOpenDevice)
    {
        FlushControlButton(true);
    }
    else
    {
        for (int i = 0; i < MAX_INTERFACE_NUM; i++)
        {
            if (false == bHaveCheck[i])
            {
                printf("List %d:Unchecked device!\r\n", i + 1);
            }
            else
            {
                printf("List %d:No device opened successfully!\r\n", i + 1);
            }
        }
    }
}

void MainWindow::on_bnStartGrabbing_clicked()
{
    if (false == m_bOpenDevice || true == m_bStartGrabbing)
    {        
        return;
    }

    int nRet = MV_FG_SUCCESS;

    for (int i = 0; i < MAX_INTERFACE_NUM; ++i)
    {
        for (int j = 0; j < MAX_DEVICE_NUM; ++j)
        {
            if (!m_stInterface[i].hDevice[j])
            {
                continue;
            }
        
            // ch:获取流通道个数 | en:Get number of stream
            unsigned int nStreamNum = 0;
            nRet = MV_FG_GetNumStreams(m_stInterface[i].hDevice[j], &nStreamNum);
            if (MV_FG_SUCCESS != nRet || 0 == nStreamNum)
            {
                printf("List %d:No stream available! number = %d, DevIndex[%d], nRet[%#x]\r\n", i + 1, nStreamNum, i+1, nRet);
                continue;
            }

            // ch:打开流通道(目前只支持单个通道) | en:Open stream(Only a single stream is supported now)
            nRet = MV_FG_OpenStream(m_stInterface[i].hDevice[j], 0, &m_stInterface[i].hStream[j]);
            if (MV_FG_SUCCESS != nRet)
            {
                printf("List %d:Open Stream fail! DevIndex[%d], nRet[%#x]\r\n", i + 1, j+1, nRet);
                continue;
            }

            // ch:设置SDK内部缓存数量 | en:Set internal buffer number
            nRet = MV_FG_SetBufferNum(m_stInterface[i].hStream[j], BUFFER_NUMBER);
            if (MV_FG_SUCCESS != nRet)
            {
                printf("List %d:Set buffer number failed! DevIndex[%d], nRet[%#x]\r\n", i + 1, j+1, nRet);
                nRet = MV_FG_CloseStream(m_stInterface[i].hStream[j]);
                if (MV_FG_SUCCESS != nRet)
                {
                    printf("List %d:Close stream failed! %#x\n", i + 1, nRet);
                }
                m_stInterface[i].hStream[j] = NULL;
                continue;
            }


            // ch:开始取流 | en:Start Acquisition
            nRet = MV_FG_StartAcquisition(m_stInterface[i].hStream[j]);
            if (MV_FG_SUCCESS != nRet)
            {
                printf("List %d:Start Acquisition failed! DevIndex[%d], nRet[%#x]\r\n", i + 1, j+1, nRet);
                nRet = MV_FG_CloseStream(m_stInterface[i].hStream[j]);
                if (MV_FG_SUCCESS != nRet)
                {
                    printf("List %d:Close stream failed! %#x\n", i + 1, nRet);
                }
                m_stInterface[i].hStream[j] = NULL;
                continue;
            }

            m_bStartGrabbing = true; 

            // ch:开始采集之后才创建workthread线程
            unsigned int nThreadID = 0;
            m_nCurListIndex   = i;
            m_nCurCameraIndex = j;
            printf("[i= %d, j= %d] create pthread \n", i, j);
            nRet = pthread_create(&(m_stInterface[i].hGrabThread[j]), NULL, WorkThread, this);
            if (0 != nRet)
            {
                ShowErrorMsg("pthread create fail", 0);
                return;
            }
            m_bDeviceGrabbingFlag[i][j] = true;
        
        }
    }

    FlushControlButton(true);
}

void MainWindow::on_bnStopGrabbing_clicked()
{
    if (false == m_bOpenDevice || false == m_bStartGrabbing)
    {        
        return;
    }

    int nRet = MV_FG_SUCCESS;

    for (int i = 0; i < MAX_INTERFACE_NUM; i++)
    {
        for (int j = 0; j < MAX_DEVICE_NUM; j++)
        {
            if (!m_stInterface[i].hStream[j])
            {
                continue;
            }
        
            m_bDeviceGrabbingFlag[i][j] = false;

            if (m_stInterface[i].hGrabThread[j])
            {
                pthread_join(m_stInterface[i].hGrabThread[j], NULL);
                m_stInterface[i].hGrabThread[j] = NULL;
            }

            nRet = MV_FG_StopAcquisition(m_stInterface[i].hStream[j]);
            if (MV_FG_SUCCESS != nRet)
            {
                printf("List %d:Stop grabbing fail! DevIndex[%d], nRet[%#x]\r\n", i + 1, j+1, nRet);
            }

            nRet = MV_FG_CloseStream(m_stInterface[i].hStream[j]);
            if (MV_FG_SUCCESS != nRet)
            {
                printf("List %d:Close stream failed! %#x\n", i + 1, nRet);
            }
            m_stInterface[i].hStream[j] = NULL;

        }
    }
    
    m_bStartGrabbing = false;

    FlushControlButton(true);
}
