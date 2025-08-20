
#ifndef __MV_MODULE_H__
#define __MV_MODULE_H__


#include "linux/kernel.h"
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/signal.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/interrupt.h> 
#include <asm/uaccess.h> 
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/version.h>
#include "linux/interrupt.h"
#include "linux/dmaengine.h"
#include "linux/mmzone.h"
#include "asm-generic/termbits.h"

#define MV_MODULE_LONG_NAME "MVFG Virtual Serial"
#define MV_COMPANY          "MVFG"
#define MV_COPYRIGHT        "GPL"
#define MV_VERSION          "2.3.1.2 build 20231114"

#define MV_CLASS_NAME       "ttymvfg_class"
#define MV_DEVICE_NAME      "ttymvfg"

#define VIRTUAL_SERIAL_DEVICE_NAME                      "VirSerialDev"

#define VIRTUAL_UART_DRIVER_NAME                        "ttyvirserial"
#define VIRTUAL_UART_DEVICE_NAME                        "ttyvirserial"

//根据设备的id填写,厂商id和设备id
// #define CML_VENDOR_ID      0x10EE
// #define CML_DEVICE_ID      0x7024

// #define CML_SUBVENDOR_ID    0x10EE
// #define CML_SUBDEVICE_ID    0x3127

#define MV_TCGETS               00000001
#define MV_TCSETS               00000002
#define MV_TCFLSH               00000003


#define MV_CSIZE                CSIZE
#define MV_CS5                  CS5
#define MV_CS6                  CS6
#define MV_CS7                  CS7
#define MV_CS8                  CS8
#define MV_CSTOPB               CSTOPB
#define MV_PARENB               PARENB
#define MV_INPCK                INPCK
#define MV_PARODD               PARODD
#define MV_ISTRIP               ISTRIP


#define MSI_COUNT                  8


#ifndef DEBUG
#define DEBUG_INFO(fmt, args...)        // MV_DEBUG_PRINTK(fmt, ## args);
#define DEBUG_WARNNING(fmt, args...)    // MV_DEBUG_PRINTK(fmt, ## args);
#define DEBUG_ERR(fmt, args...)         // MV_DEBUG_PRINTK(fmt, ## args);

#else
#define DEBUG_INFO(fmt, args...)        MV_DEBUG_PRINTK(fmt, ## args);
#define DEBUG_WARNNING(fmt, args...)    MV_DEBUG_PRINTK(fmt, ## args);
#define DEBUG_ERR(fmt, args...)         MV_DEBUG_PRINTK(fmt, ## args);
#endif

extern void MV_DEBUG_PRINTK(const char *fmt, ...);

#define CML_MSI_UART_CONTROL_INDEX                      5       // 默认下标为5的中断号为虚拟设备中断号

// // 设备对象上下文
// typedef struct _DEVICE_DATA_
// {
//     bool                    bBar0Found;                             // 资源是否接收到并进行映射处理
//     void*                   pBar0Phy;
//     void*                   pBar0Vir;
//     unsigned long           nBar0Length;
//     unsigned long           nBar0IoFlag;                            // IO的访问方式

//     // MSI相关变量
//     bool                    bMsiFound;                              // 是否收到系统分配的中断资源
//     unsigned char           hMSInterrupt[MSI_COUNT];                // 中断对象数组
// }DEVICE_DATA, *PDEVICE_DATA;

// 一个PCI设备实例，即表示采集卡实例
typedef struct _SERIAL_DEVICE_
{
    void*                   pUartPort;
    void*                   pCdvClass;
    void*                   pCdvClassDev;
    char                    chVirSerialDevClassName[20];
    char                    chVirSerialDevName[20];

    void*                   pPlatformData;                      // platform_device -> platform_data
    void*                   pPciRegister;
    unsigned int            nPciRegisterLength;
    int                     nIrq;
    unsigned int            nIrqNum;
    void*                   pSerialDeviceContext;               // PSERIAL_DEVICE_CONTEXT
    dev_t                   nDevMajor; 
    void*                   pPlatformDev;                       // platform_device

    //request
    char*                   pRxBuffer;
    unsigned int            nRxBufferLen;
    char*                   pTxBuffer;
    unsigned int            nTxBufferLen;
    void*                   pFile;
    const char*             pChTtyName;
    int                     nLine;
    unsigned int            nReserved[8];
    struct cdev             stCdev;
} SERIAL_DEVICE, *PSERIAL_DEVICE;

typedef struct _MV_KTERMIOS_
{
    void*       pKtermios;
}MV_KTERMIOS, *PMV_KTERMIOS;

extern int VirSerial_ModuleProbe(struct platform_device * pstPlatformDevice);
extern int VirSerial_ModuleRemove(struct platform_device * pstPlatformDevice);

extern int ModuleOpen(struct inode * pstInode, struct file * file);
extern int ModuleRelease( struct inode* aNode, struct file* aFile );

extern ssize_t ModuleRead (struct file *pstFile, char __user * pchUserBuffer, size_t nBufferLen, loff_t * offset);
extern ssize_t ModuleWrite (struct file *pstFile, const char __user * pchUserBuffer, size_t nBufferLen, loff_t * offset);

extern int MV_ModuleRead(PSERIAL_DEVICE pstSerialDevice);

extern int MV_ModuleWrite(PSERIAL_DEVICE pstSerialDevice);

// #ifdef HAVE_UNLOCKED_IOCTL
extern long ModuleUnlockedIoctl( struct file* filp, unsigned int cmd, unsigned long arg );
// #else
// extern long  ModuleIoctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
// #endif

extern int MV_ModuleProbe(PSERIAL_DEVICE pstSerialDevice);
extern void MV_ModuleRemove(PSERIAL_DEVICE pstSerialDevice);


extern int MV_AddDevice(PSERIAL_DEVICE pstSerialDevice);
extern int MV_RemoveDeivce(PSERIAL_DEVICE pstSerialDevice);

extern int MV_UartDataSet(PSERIAL_DEVICE pstSerialDevice);


extern int MV_RequestIrq(unsigned int nIrq, irqreturn_t (*interrputfn)(int nIrq, void *pData), void* pDriverData);
extern void MV_FreeIrq(unsigned int nIrq, void* pDriverData);

extern int MV_SetDriverData(void* pPciDevice, void* pDriverData, void* pIrqData);

extern int MV_ModuleOpen(PSERIAL_DEVICE pstSerialDev);
extern int MV_ModuleRelease(PSERIAL_DEVICE pstSerialDevice, void* pFile);

extern long MV_ModuleIoctl(void* pFile, unsigned int nCmd, unsigned long nArg, void* pDriverData);


extern int InitModuleDevice(void** ppOutClassForDev, void** ppOutDevNode, const void* pMvDevMajor);
extern int DeinitModuleDevice(void);


extern void MV_ReadConfigDword(void* pPciDevice, unsigned long nType, int* value);
extern void MV_ReadConfigPci(void* pPciDevice, unsigned int* nDevFn, unsigned char* nNumber);

extern int MV_InitAsyncLogMoudle(void);

extern int MV_DeInitAsyncLogMoudle(void);

extern unsigned int	virserial_tx_empty(struct uart_port * pstUartPort);
extern void virserial_set_mctrl(struct uart_port * pstUartPort, unsigned int mctrl);
extern unsigned int virserial_get_mctrl(struct uart_port * pstUartPort);
extern void virserial_stop_tx(struct uart_port * pstUartPort);
extern void virserial_start_tx(struct uart_port * pstUartPort);
// extern void virserial_throttle(struct uart_port * pstUartPort);
// extern void virserial_unthrottle(struct uart_port * pstUartPort);
// extern void virserial_send_xchar(struct uart_port * pstUartPort, char ch);
extern void virserial_stop_rx(struct uart_port * pstUartPort);
extern void virserial_enable_ms(struct uart_port * pstUartPort);
extern void virserial_break_ctl(struct uart_port * pstUartPort, int ctl);
extern int virserial_startup(struct uart_port * pstUartPort);
extern void virserial_shutdown(struct uart_port * pstUartPort);
extern void virserial_flush_buffer(struct uart_port * pstUartPort);

#if ( LINUX_VERSION_CODE > KERNEL_VERSION(6,0,19) )
extern void virserial_set_termios(struct uart_port * pstUartPort, struct ktermios *new, const struct ktermios *old);
#else
extern void virserial_set_termios(struct uart_port * pstUartPort, struct ktermios *new, struct ktermios *old);
#endif
// extern void virserial_set_ldisc(struct uart_port *, struct ktermios *);
// extern void		(*pm)(struct uart_port *, unsigned int state, unsigned int oldstate);
extern const char* virserial_type(struct uart_port *pstUartPort);
extern void virserial_release_port(struct uart_port *pstUartPort);
extern int virserial_request_port(struct uart_port *pstUartPort);
extern void virserial_config_port(struct uart_port *pstUartPort, int data);
extern int virserial_verify_port(struct uart_port *pstUartPort, struct serial_struct *pstSerial);
extern int virserial_ioctl_port(struct uart_port * port, unsigned int cmd, unsigned long args);
// extern int		(*ioctl)(struct uart_port *, unsigned int, unsigned long);
#ifdef CONFIG_CONSOLE_POLL
// extern int		(*poll_init)(struct uart_port *);
extern void virserial_poll_put_char(struct uart_port *pstUartPort, unsigned char data);
extern int virserial_poll_get_char(struct uart_port *pstUartPort);
#endif


extern int virserial_read_char(PSERIAL_DEVICE pstSerialDev, char chData);
extern void virserial_read_push(PSERIAL_DEVICE pstSerialDev);

extern void MV_TermiosInit(PSERIAL_DEVICE pstSerialDev, void* pstTermiosOld);
extern int MV_TermiosMalloc(PMV_KTERMIOS pstKtermios);
extern int MV_TermiosFree(PMV_KTERMIOS pstKtermios);
extern int MV_TermiosFromUser(unsigned long nArg, PMV_KTERMIOS pstKtermios);
extern int MV_TermiosToUser(unsigned long nArg, void* pstKtermios);
extern int MV_GetUartBaudrate(void* uart, void* pKtermiosNew, void* pKtermiosOld, int* pnBaudrate);
extern int MV_UpdateUartTimeout(void* uart, unsigned int nCcflag, int nBaudrate);

extern int MV_GetTermiosCcflag(void* pKtermios, unsigned int* pValue);
extern int MV_GetTermiosCiflag(void* pKtermios, unsigned int* pValue);

extern int MV_SetTermiosCiflag(void* pKtermiosNew, void* pKtermiosOld);
extern int MV_SetTermiosCoflag(void* pKtermiosNew, void* pKtermiosOld);
extern int MV_SetTermiosCcflag(void* pKtermiosNew, void* pKtermiosOld);
extern int MV_SetTermiosClflag(void* pKtermiosNew, void* pKtermiosOld);
extern int MV_SetTermiosCline(void* pKtermiosNew, void* pKtermiosOld);
extern int MV_SetTermiosCCc(void* pKtermiosNew, void* pKtermiosOld);
extern int MV_SetTermiosSpeed(void* pKtermiosNew, void* pKtermiosOld);


extern int MV_InitModule(void);
extern int MV_DeInitModule(void);

#endif  // __MV_MODULE_H__