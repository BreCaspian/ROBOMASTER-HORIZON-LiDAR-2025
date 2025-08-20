#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "MVFGControl.h"
#include <stdio.h>
#include <QMainWindow>
#include <QCloseEvent>
#include <list>
#include <unistd.h>
#include <pthread.h>

#define TRIGGER_MODE_ON             1           // ch:触发模式开 | en:Trigger mode on
#define TRIGGER_MODE_OFF            0           // ch:触发模式关 | en:Trigger mode off

#define TRIGGER_SOURCE_LINE0                0           // ch:Line0 | en:Line0
#define TRIGGER_SOURCE_LINE1                1           // ch:Line1 | en:Line1
#define TRIGGER_SOURCE_LINE2                2           // ch:Line2 | en:Line2
#define TRIGGER_SOURCE_LINE3                3           // ch:Line3 | en:Line3
#define TRIGGER_SOURCE_COUNTER0             4           // ch:Conuter0 | en:Conuter0
#define TRIGGER_SOURCE_SOFTWARE             7           // ch:软触发 | en:Software
#define TRIGGER_SOURCE_FrequencyConverter   8           // ch:变频器 | en:Frequency Converter

#define FILE_NAME_LEN           256   // ch:文件名长度 | en:Length of file name
#define BUFFER_NUMBER           3
#define DISPLAY_NUM             2

#define MAX_EXPOSURE_NUM        2     // 分时频闪的最大灯数(用户可根据相机具体的节点等方式自定义曝光个数)

enum CAMERA_TYPE
{
    TYPE_FRAME_SCAN = 0,
    TYPE_LINE_SCAN  = 1,
};


using namespace std;

namespace Ui {
class MainWindow;
}


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

    int  SetTriggerMode();                // ch:设置触发模式 | en:Set Trigger Mode
    int  GetTriggerMode();
    int  SetTriggerSource();              // ch:设置触发源 | en:Set Trigger Source
    int  GetTriggerSource();
    bool RemoveCustomPixelFormats(MV_FG_PIXEL_TYPE enPixelFormat);

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

    void on_RadioContinus_clicked();

    void on_RadioTrigger_clicked();

    void on_CheckSoftTrigger_clicked();

    void on_bnSoftTriggerOnce_clicked();

    
private:
    Ui::MainWindow *ui;

    void*        m_hWnd[DISPLAY_NUM];
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
    unsigned int m_nTriggerMode;
    unsigned int m_nTriggerSource;
    bool         m_bSoftWareTriggerCheck;

    list<QString>           m_lCamInfo;
                
    pthread_mutex_t         m_hSaveImageMux;            // ch:存图锁 | en:Mutex for saving image
    unsigned char*          m_pDataBuf;                 // ch:数据缓存 | en:Data buffer
    unsigned int            m_nDataBufSize;             // ch:数据缓存大小 | en:Length of data buffer
    unsigned char*          m_pSaveImageBuf;            // ch:图像缓存 | en:Image buffer
    unsigned int            m_nSaveImageBufSize;        // ch:图像缓存大小 | en:Length of image buffer
    MV_FG_INPUT_IMAGE_INFO  m_stImageInfo;              // ch:图像信息 | en:Image info
};

#endif // MAINWINDOW_H
