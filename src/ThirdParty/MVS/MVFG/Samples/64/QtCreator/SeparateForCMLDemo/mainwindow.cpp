#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) 
: QMainWindow(parent)
, ui(new Ui::MainWindow)
, m_hWnd(NULL)
, m_bHBMode(false)               
, m_bThreadState(false) 
, m_bOpenIF(false)
, m_bOpenDevice(false)                     
, m_bStartGrabbing(false)
, m_hInterface(NULL)
, m_hDevice(NULL)               
, m_hStream(NULL) 
, m_hWndDisplay(NULL)
, m_nThreadID(-1)
, m_nDeviceNum(0)
, m_nInterfaceNum(0)
{
    ui->setupUi(this);

    m_hWnd = (void *)ui->widgetDisplay->winId();
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


void MainWindow::ThreadFun()
{
    MV_FG_BUFFER_INFO           stFrameInfo = { 0 };    // ch:图像信息 | en:Frame info
    MV_FG_DISPLAY_FRAME_INFO    stDisplayInfo = { 0 };  // ch:显示的图像信息 | en:Display frame info
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
            else
            {
                usleep(5000);      
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

    ui->LCamWidth->setEnabled(m_bOpenIF && !m_bStartGrabbing);
    ui->LCamHeight->setEnabled(m_bOpenIF && !m_bStartGrabbing);
    ui->comboPixelFormat->setEnabled(m_bOpenIF && !m_bStartGrabbing);
    ui->comboPixelSize->setEnabled(m_bOpenIF && !m_bStartGrabbing);
    ui->comboCamTapGeometry->setEnabled(m_bOpenIF && !m_bStartGrabbing);
    ui->comboClConfig->setEnabled(m_bOpenIF && !m_bStartGrabbing);

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
            case MV_FG_CAMERALINK_INTERFACE:
            {
                strMsg.sprintf("CML[%d]: %s | %s | %s", i, stInterfaceInfo.IfaceInfo.stCMLIfaceInfo.chDisplayName,
                               stInterfaceInfo.IfaceInfo.stCMLIfaceInfo.chInterfaceID,
                               stInterfaceInfo.IfaceInfo.stCMLIfaceInfo.chSerialNumber);
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

    // ch:开启采集卡上的分离方案使能 | en:Enable seperate scheme on interface.
    nRet = MV_FG_SetBoolValue(m_hInterface, "CameraDeviceEnable", true);
    if (MV_FG_SUCCESS != nRet)
    {
        MV_FG_CloseInterface(m_hInterface);
        m_hInterface = NULL;

        ShowErrorMsg("Set CameraDeviceEnable failed", nRet);
        return;
    }

	InitInterfaceParam();

    m_bOpenIF = true;

    ui->bnIntOpen->setEnabled(false);
    ui->bnIntClose->setEnabled(true);
    ui->bnCamEnum->setEnabled(true);
}

void MainWindow::on_bnIntClose_clicked()
{
    int nRet = MV_FG_SUCCESS;
    int nIndex = ui->ComboInterfaces->currentIndex();
 
    if(m_hDevice)
    {
        on_bnCamClose_clicked();
    }

    // ch:关闭采集卡 | en:Close interface
    if (m_hInterface)
    {
        // ch:关闭采集卡上的分离方案使能 | en:Disable seperate scheme on interface.
        MV_FG_SetBoolValue(m_hInterface, "CameraDeviceEnable", false);

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

    if (!m_bOpenDevice)
    {
        ShowErrorMsg("not open device", 0);
        return;
    }
    int nRet = MV_FG_SUCCESS;

    // ch:根据相机参数配置采集卡端的输出图像参数 | en:Configure output image parameters.
    nRet = MV_FG_SetIntValue(m_hInterface, "CameraWidth", ui->LCamWidth->text().toInt());
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Set CameraWidth failed", nRet);
        return;
    }

    nRet = MV_FG_SetIntValue(m_hInterface, "CameraHeight", ui->LCamHeight->text().toInt());
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Set CameraHeight failed", nRet);
        return;
    }

    QString strCBText =  ui->comboPixelFormat->currentText();
	nRet = MV_FG_SetEnumValueByString(m_hInterface, "CameraPixelFormat", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set CameraPixelFormat failed.", nRet);
		return;
	}

    strCBText.clear();
    strCBText =  ui->comboPixelSize->currentText();
	nRet = MV_FG_SetEnumValueByString(m_hInterface, "CameraPixelSize", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set CameraPixelSize failed.", nRet);
		return;
	}

    strCBText.clear();
    strCBText =  ui->comboCamTapGeometry->currentText();
	nRet = MV_FG_SetEnumValueByString(m_hInterface, "CameraTapGeometry", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set CameraTapGeometry failed.", nRet);
		return;
	}

    strCBText.clear();
    strCBText =  ui->comboClConfig->currentText();
	nRet = MV_FG_SetEnumValueByString(m_hInterface, "CameraClConfiguration", strCBText.toStdString().c_str());
	if (MV_FG_SUCCESS != nRet)
	{
		ShowErrorMsg("Set CameraClConfiguration failed.", nRet);
		return;
	}

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

    nRet = MV_FG_CloseStream(m_hStream);
    if (MV_FG_SUCCESS != nRet)
    {
        ShowErrorMsg("Close Stream fail", nRet);
        return;
    }
    m_bStartGrabbing = false;
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

// ch:获取采集卡参数 | en:Init interfaces parameters
int MainWindow::InitInterfaceParam()
{
	int nRet = MV_FG_SUCCESS;
	MV_FG_ENUMVALUE stEnumValue = {0};
	MV_FG_INTVALUE  stIntValue = {0};

	// ch:获取Image Width | en:Get Image Width
	memset(&stIntValue, 0, sizeof(stIntValue));
	nRet = MV_FG_GetIntValue(m_hInterface, "CameraWidth", &stIntValue);
	if (MV_FG_SUCCESS == nRet)
	{
		ui->LCamWidth->setText(QString("%1").arg(stIntValue.nCurValue));
	}

	// ch:获取Image Height | en:Get Image Height
	memset(&stIntValue, 0, sizeof(stIntValue));
	nRet = MV_FG_GetIntValue(m_hInterface, "CameraHeight", &stIntValue);
	if (MV_FG_SUCCESS == nRet)
	{
		ui->LCamHeight->setText(QString("%1").arg(stIntValue.nCurValue));
	}

	// ch:CameraPixelFormat | en:Get CameraPixelFormat
	ui->comboPixelFormat->clear();
	memset(&stEnumValue, 0, sizeof(MV_FG_ENUMVALUE));
    nRet = MV_FG_GetEnumValue(m_hInterface, "CameraPixelFormat", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboPixelFormat->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}
		}

		ui->comboPixelFormat->setCurrentIndex(nIndex);
	}

	// ch:获取 CameraPixelSize | en:Get CameraPixelSize
	ui->comboPixelSize->clear();
	memset(&stEnumValue, 0, sizeof(MV_FG_ENUMVALUE));
	nRet = MV_FG_GetEnumValue(m_hInterface, "CameraPixelSize", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboPixelSize->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}
		}

		ui->comboPixelSize->setCurrentIndex(nIndex);
	}

	// ch:获取CameraTapGeometry | en:Get CameraTapGeometry
	ui->comboCamTapGeometry->clear();
	memset(&stEnumValue, 0, sizeof(MV_FG_ENUMVALUE));
	nRet = MV_FG_GetEnumValue(m_hInterface, "CameraTapGeometry", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboCamTapGeometry->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}
		}

		ui->comboCamTapGeometry->setCurrentIndex(nIndex);
	}


	// ch:获取CameraClConfiguration | en:Get CameraClConfiguration
	ui->comboClConfig->clear();
	memset(&stEnumValue, 0, sizeof(MV_FG_ENUMVALUE));
	nRet = MV_FG_GetEnumValue(m_hInterface, "CameraClConfiguration", &stEnumValue);
	if (MV_FG_SUCCESS == nRet)
	{
		unsigned int nIndex = 0;
		for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++)
		{
			ui->comboClConfig->addItem(stEnumValue.strSymbolic[i]);
			if (stEnumValue.nSupportValue[i] == stEnumValue.nCurValue)
			{
				nIndex = i;
			}
		}

		ui->comboClConfig->setCurrentIndex(nIndex);
	}

	return nRet;
}
