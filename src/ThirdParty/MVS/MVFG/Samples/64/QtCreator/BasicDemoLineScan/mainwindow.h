#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "MVFGControl.h"
#include <stdio.h>
#include <QMainWindow>
#include <QCloseEvent>
#include <list>
#include <unistd.h>
#include <pthread.h>
using namespace std;

namespace Ui {
class MainWindow;
}

#define FILE_NAME_LEN           256             // ch:文件名长度 | en:Length of file name

enum SAVE_IAMGE_TYPE
{
    Image_Undefined                  = 0,                        ///< \~chinese 未定义的图像格式             \~english Undefined Image Type
    Image_Bmp                        = 1,                        ///< \~chinese BMP图像格式                  \~english BMP Image Type
    Image_Jpeg                       = 2,                        ///< \~chinese JPEG图像格式                 \~english Jpeg Image Type
	Image_Tiff                       = 3,                        ///< \~chinese Tiff图像格式                 \~english Tiff Image Type
	Image_Png                        = 4,                        ///< \~chinese Png图像格式                  \~english Png Image Type

};

#define MAX_INTERFACE_NUM   5
#define MAX_DEVICE_NUM   4
#define MAX_STREAM_NUM   4
#define BUFFER_NUMBER    3
#define DISPLAY_NUM      4

typedef struct _INTERFACE_INFO_
{
    IFHANDLE     hInterface;
    bool         bOpenIF;
}INTERFACE_INFO;

typedef struct _CAMERA_INFO_
{
    IFHANDLE     hInterface;
    DEVHANDLE    hDevice;
    bool         bOpenDevice;
    bool         bStartGrabbing;
    STREAMHANDLE hStream;
    BUFFERHANDLE hBuffer[BUFFER_NUMBER];
    void*        pBuffer[BUFFER_NUMBER];
    pthread_t    nThreadID;
}CAMERA_INFO;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void ThreadFun();
    void closeEvent(QCloseEvent *event);

private:
    void ShowErrorMsg(QString csMessage, unsigned int nErrorNum);
    void FlushControlButton(bool bOpen, bool bStart);

private:

	int  GetInterfaceParams();									    // ch:获取采集卡参数 | en:Get interface parameters
	int  GetDeviceParams();										    // ch:获取相机参数 | en:Get device parameters
	void EnableIFParamsControls();
	void EnableDevParamsControls();
    
	bool RemoveCustomPixelFormats(MV_FG_PIXEL_TYPE enPixelFormat);  // ch:去除自定义的像素格式 | en:Remove custom pixel formats
	int SaveImage(SAVE_IAMGE_TYPE enSaveImageType);                 // ch:保存图片 | en:Save image



private slots:

    void on_bnIntEnum_clicked();

    void on_bnIntOpen_clicked();

    void on_bnIntClose_clicked();

    void on_bnCamEnum_clicked();

    void on_bnCamOpen_clicked();

    void on_bnCamClose_clicked();

    void on_bnStartGrabbing_clicked();

    void on_bnStopGrabbing_clicked();

    void on_ComboInterfaces_currentIndexChanged(int index);

    void on_ComboDevices_currentIndexChanged(int index);

    void on_bnBmpSave_clicked();           
    void on_bnJpegSave_clicked();           
    void on_bnTiffSave_clicked();           
    void on_bnPngSave_clicked();          

    void on_comboCamType_currentIndexChanged(int index);
    void on_comboLineSel_currentIndexChanged(int index);
    void on_comboLPolarity_currentIndexChanged(int index);
    void on_comboEncoderSel_currentIndexChanged(int index);
    void on_comboEncoderA_currentIndexChanged(int index);
    void on_comboEncoderB_currentIndexChanged(int index);
    void on_comboEnTrigger_currentIndexChanged(int index);
    void on_comboCCSel_currentIndexChanged(int index);
    void on_comboCCSource_currentIndexChanged(int index);
    void on_LHeight_editingFinished() ;


    void on_comboPixelType_currentIndexChanged(int index);
    void on_comboScanMode_currentIndexChanged(int index);
    void on_comboTriggerSel_currentIndexChanged(int index);
    void on_comboTriggerMode_currentIndexChanged(int index);
    void on_comboTriggerSource_currentIndexChanged(int index);
    void on_comboTriggerActivation_currentIndexChanged(int index);
    void on_comboTestMode_currentIndexChanged(int index);
    void on_bnTriggerOnce_clicked();          
    void on_LCamWidth_editingFinished() ;
    void on_LCamHeight_editingFinished() ;

    
public:
    int m_nDevIndex;
private:
    Ui::MainWindow *ui;

    void *m_hWnd;
    bool         m_bThreadState;  
    bool         m_bOpenIF;
    bool         m_bOpenDevice;                     
    bool         m_bStartGrabbing;     

    IFHANDLE     m_hInterface;
    DEVHANDLE    m_hDevice;               
    STREAMHANDLE m_hStream; 

    pthread_t    m_nThreadID;

    unsigned int m_nDeviceNum;
    unsigned int m_nInterfaceNum;
                
    list<QString>           m_lCamInfo;
                
    pthread_mutex_t         m_hSaveImageMux;            // ch:存图锁 | en:Mutex for saving image
    unsigned char*          m_pDataBuf;                 // ch:数据缓存 | en:Data buffer
    unsigned int            m_nDataBufSize;             // ch:数据缓存大小 | en:Length of data buffer
    unsigned char*          m_pSaveImageBuf;            // ch:图像缓存 | en:Image buffer
    unsigned int            m_nSaveImageBufSize;        // ch:图像缓存大小 | en:Length of image buffer
    MV_FG_INPUT_IMAGE_INFO  m_stImageInfo;              // ch:图像信息 | en:Image info

};

#endif // MAINWINDOW_H
