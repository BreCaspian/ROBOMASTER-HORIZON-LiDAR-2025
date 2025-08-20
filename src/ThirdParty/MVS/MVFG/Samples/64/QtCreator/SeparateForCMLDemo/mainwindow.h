#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "MVFGControl.h"
#include <stdio.h>
#include <QMainWindow>
#include <QCloseEvent>
#include <list>
#include <unistd.h>
#include <pthread.h>

#define BUFFER_NUMBER 3

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

	int InitInterfaceParam();									    
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

    
private:
    Ui::MainWindow *ui;

    void *m_hWnd;
    bool         m_bHBMode;                  // ch:是否获取图像压缩模式 | en:Whether to get Image Compress Mode
    bool         m_bThreadState;  
    bool         m_bOpenIF;
    bool         m_bOpenDevice;                     
    bool         m_bStartGrabbing;     

    IFHANDLE     m_hInterface;
    DEVHANDLE    m_hDevice;               
    STREAMHANDLE m_hStream; 

    void*        m_hWndDisplay;
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
