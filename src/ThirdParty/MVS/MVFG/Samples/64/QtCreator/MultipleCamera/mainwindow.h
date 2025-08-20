#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "MVFGControl.h"
#include <stdio.h>
#include <QMainWindow>
#include <QCloseEvent>
#include <list>
using namespace std;

namespace Ui {
class MainWindow;
}

#define MAX_DEVICE_NUM          4        // ch:每张采集卡的最大设备个数 | en:Maximum number of devices on each card
#define MAX_STREAM_NUM          4        // ch:每个相机的最大流通道个数 | en:Maximum number of stream channels per camera
#define BUFFER_NUMBER           3        // ch:申请的缓存个数 | en:Number of requested buffer
#define TIMEOUT                 5000     // ch:超时时间，单位毫秒 | en:Timeout, unit ms
#define DEVICE_INFO_LEN         256      // ch:设备信息长度 | en:Device information length
#define MAX_LOGINFO_LEN         1024     // ch:日志信息长度 | en:Log information length
#define MAX_INTERFACE_NUM       2        // ch:采集卡列表个数 | en:Number of acquisition card lists
#define DISPLAY_NUM             8

#define  FIRST_INTERFACE  0
#define  SECOND_INTERFACE 1

typedef struct _INTERFACE_INFO_
{
    IFHANDLE          hInterface;                               // ch:采集卡句柄 | en:Interface handle
    DEVHANDLE         hDevice[MAX_DEVICE_NUM];                  // ch:设备句柄 | en:Device handle
    STREAMHANDLE      hStream[MAX_DEVICE_NUM];                  // ch:流通道句柄 | en:Stream handle
    void*             hwndDisplay[MAX_DEVICE_NUM];              // ch:显示句柄 | en:Display handle
    void*             hwndRect[MAX_DEVICE_NUM];                 // ch:显示窗口 | en:Display window
    pthread_t         hGrabThread[MAX_DEVICE_NUM];              // ch:取流线程 | en: Grabbing thread
    unsigned int      nCurIndex[MAX_DEVICE_NUM];                // ch:设备的索引值 | en:Index value of the device
}IF_INFO;


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void ThreadFun(int nCurListIndex, int nCurCameraIndex);
    void closeEvent(QCloseEvent *event);

private:
    void ShowErrorMsg(QString csMessage, unsigned int nErrorNum);
    void FlushControlButton(bool bIsCameraReady);
private slots:

    void on_bnIFEnum_clicked();

    void on_bnIFOpen_clicked();

    void on_bnIFClose_clicked();

    void on_bnDevEnum_clicked();

    void on_bnDevOpen_clicked();

    void on_bnDevClose_clicked();

    void on_bnStartGrabbing_clicked();

    void on_bnStopGrabbing_clicked();

public:
    int m_nCurCameraIndex;
    int m_nCurListIndex;

private:

    void*        m_hWnd;

    unsigned int m_nInterfaceNum;
    IF_INFO      m_stInterface[MAX_INTERFACE_NUM];

    bool         m_bOpenIF;
    bool         m_bOpenDevice;
    bool         m_bStartGrabbing;
    bool         m_bDeviceGrabbingFlag[MAX_INTERFACE_NUM][MAX_DEVICE_NUM];
    
    bool         m_bCamCheck[MAX_INTERFACE_NUM][MAX_DEVICE_NUM];
    unsigned int m_nDeviceNum;
    void*        m_hWndDisplay[DISPLAY_NUM];

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
