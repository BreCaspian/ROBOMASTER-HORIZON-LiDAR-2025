
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

#ifdef _XOF
#define MV_MODULE_LONG_NAME "MVFG Quad Xof Frame Grabber"
#else
#define MV_MODULE_LONG_NAME "MVFG Quad Cxp Frame Grabber"
#endif
#define MV_COMPANY          "MVFG"
#define MV_COPYRIGHT        "GPL"
#define MV_VERSION          "2.3.1.2 build 20231114"

#define MV_CLASS_NAME       "mvfgcxp_class"
#define MV_DEVICE_NAME      "mvfgcxp"

//根据设备的id填写,厂商id和设备id

#define CXP_VENDOR_ID      0x10ee

#ifdef _XOF
//GS-1002F
#define XOF_DEVICE_ID       0x7028
#define XOF_SUBDEVICE_ID    0x8627

#else
//GX-1004
#define CXP6_DEVICE_ID      0x7028
#define CXP6_SUBDEVICE_ID   0x0007

//GY-1004
#define CXP12_DEVICE_ID       0x8038
#define CXP12_SUBDEVICE_ID    0x1247
#endif

#define MSI_COUNT                  8


#ifndef DEBUG
#define DEBUG_INFO(fmt, args...)         //MV_DEBUG_PRINTK(fmt, ## args);
#define DEBUG_WARNNING(fmt, args...)     //MV_DEBUG_PRINTK(fmt, ## args);
#define DEBUG_ERR(fmt, args...)          //MV_DEBUG_PRINTK(fmt, ## args);
#else
#define DEBUG_INFO(fmt, args...)        MV_DEBUG_PRINTK(fmt, ## args);
#define DEBUG_WARNNING(fmt, args...)    MV_DEBUG_PRINTK(fmt, ## args);
#define DEBUG_ERR(fmt, args...)         MV_DEBUG_PRINTK(fmt, ## args);
#endif

extern void MV_DEBUG_PRINTK(const char *fmt, ...);

// 设备对象上下文
typedef struct _DEVICE_DATA_
{
    //  Bar0 空间资源以及资源映射
    bool                    bBar0Found;                             // 资源是否接收到并进行映射处理
    void*                   pBar0Phy;
    void*                   pBar0Vir;
    unsigned long           nBar0Length;
    unsigned long           nBar0IoFlag;                            // IO的访问方式

    // MSI相关变量
    bool                    bMsiFound;                              // 是否收到系统分配的中断资源
    unsigned char           hMSInterrupt[MSI_COUNT];                // 中断对象数组
}DEVICE_DATA, *PDEVICE_DATA;

// 一个PCI设备实例，即表示采集卡实例
typedef struct _PCI_DEVICE_
{
    struct pci_dev*                 pstPciDev;
    struct cdev                     stCdev;                                     // 字符设备
    dev_t                           nDevMajor;                                   // 主设备号
    struct class*                   pstClass;
    struct device*                  pstClassDev;
    char                            chCxpClassName[100];
    char                            chCxpDeviceName[100];
    PDEVICE_DATA                    pstDeviceData;
    void*                           pDriverData;
    void*                           pIrqData;
} PCI_DEVICE, *PPCI_DEVICE;

extern int ModuleProbe(struct pci_dev* pstPciDev, const struct pci_device_id* pstPciDeId);
extern void ModuleRemove(struct pci_dev* pstPciDev);

extern int ModuleOpen(struct inode * pstInode, struct file * file);
extern int ModuleRelease( struct inode* aNode, struct file* aFile );

// #ifdef HAVE_UNLOCKED_IOCTL
extern long ModuleUnlockedIoctl( struct file* filp, unsigned int cmd, unsigned long arg );
// #else
// extern long  ModuleIoctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
// #endif

extern int MV_ModuleProbe(void* pstPciDevice, void* pDev, PDEVICE_DATA pstDeviceData);
extern void MV_ModuleRemove(void* pPciDevice, void* pDriverData);


extern int MV_AddDevice(void* pPciDevice, char* chDeviceName, char* chCxpClassName);
extern int MV_RemoveDeivce(void* pPciDevice, void* pBarVir);


extern int MV_RequestIrq(void* pPciDevice, irqreturn_t (*interrputfn)(int nIrq, void *pData), void* pDriverData);


extern void* MV_AllocConsistent(void* pPciDevice, int nSize, void* pPhy);
extern void MV_FreeConsistent(void* pPciDevice, int nSize, void* pPhy, void* pVir);

extern int MV_SetDriverData(void* pPciDevice, void* pDriverData, void* pIrqData);

extern int MV_ModuleOpen(void* pDriverData);
extern int MV_ModuleRelease(void * pDriverData, void* pFile);
// extern long ModuleRead(/*unsigned*/ char *pBuffer, unsigned long nCount);
// extern long ModuleWrite(void * pFile, const char *pBuffer, unsigned long nCount);
extern long MV_ModuleIoctl(void* pFile, unsigned int nCmd, unsigned long nArg, void* pDriverData);


extern int InitModuleDevice(void** ppOutClassForDev, void** ppOutDevNode, const void* pMvDevMajor);
extern int DeinitModuleDevice(void);


extern void MV_ReadConfigDword(void* pPciDevice, unsigned long nType, int* value);
extern void MV_ReadConfigPci(void* pPciDevice, unsigned int* nDevFn, unsigned char* nNumber);

extern int MV_InitAsyncLogMoudle(void);

extern int MV_DeInitAsyncLogMoudle(void);

#endif  // __MV_MODULE_H__
