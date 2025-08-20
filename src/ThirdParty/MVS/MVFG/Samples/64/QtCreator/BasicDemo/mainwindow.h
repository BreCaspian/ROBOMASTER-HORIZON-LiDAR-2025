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
    void ThreadFun(int nCurCameraIndex);
    void closeEvent(QCloseEvent *event);

private:
    void ShowErrorMsg(QString csMessage, unsigned int nErrorNum);
    void FlushControlButton(bool bOpen, bool bStart);
    void FlushParamConfiguration(bool bOpen);
    void CheckExposureTimeCanSet();
    void CheckFrameRateCanSet();

private slots:

    void on_bnIntEnum_clicked();

    void on_bnIntOpen_clicked();

    void on_bnIntClose_clicked();

    void on_bnCamEnum_clicked();

    void on_bnCamOpen_clicked();

    void on_bnCamClose_clicked();

    void on_bnStartGrabbing_clicked();

    void on_bnStopGrabbing_clicked();

    void on_bnGetParam_clicked();

    void on_bnSetParam_clicked();

    void on_ComboInterfaces_currentIndexChanged(int index);

    void on_ComboDevices_currentIndexChanged(int index);

public:
    int m_nDevIndex;
private:
    Ui::MainWindow *ui;

    void *m_hWnd;

    bool         m_bOpenOneIF;
    bool         m_bStartGrabbing;   

    unsigned int m_nInterfaceNum;
    IFHANDLE     m_hInterface;
    bool         m_bOpenIF[MAX_INTERFACE_NUM];
    //INTERFACE_INFO m_hInterface[MAX_INTERFACE_NUM];
    unsigned int m_nDeviceNum;
    CAMERA_INFO  m_stDevHandle[MAX_DEVICE_NUM];

    void*        m_hWndDisplay[DISPLAY_NUM];
    list<QString>       m_lCamInfo;

    bool        m_bFrameRateCanSet = false;
    bool        m_bExposureTimeCanSet = false;
};

#endif // MAINWINDOW_H
