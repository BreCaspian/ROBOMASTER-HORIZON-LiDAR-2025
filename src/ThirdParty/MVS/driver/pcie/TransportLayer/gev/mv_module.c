
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>

#include <linux/ioctl.h>
#include <linux/mm.h>

#include <linux/uaccess.h>

#include <linux/blkdev.h>

#include <linux/skbuff.h>

#include <linux/slab.h>
#include <linux/types.h>  
#include <linux/string.h>
//#include <asm/uaccess.h>
#include <linux/vmalloc.h>

#include <linux/delay.h>

#include <linux/kthread.h>

#include <linux/utsname.h>
#include <linux/version.h>

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

#include "mv_module.h"
#include "mv_misc.h"
#include "mv_lock.h"
#include "mv_memory.h"
#include "mv_device.h"

#include "MvErrorDefine.h"



// 驱动支持的设备
struct pci_device_id GigEDeviceId[] = {
    {GIGE_VENDOR_ID, GIGE_DEVICE_ID, GIGE_SUBVENDOR_ID, GIGE_SUBDEVICE_ID, 0, 0, 0},
    {GIGE_10VENDOR_ID, GIGE_10DEVICE_ID, GIGE_GSSUBVENDOR_ID, GIGE_GSSUBDEVICE_ID, 0, 0, 0},
    {GIGE_10VENDOR_ID, GIGE_10DEVICE_ID, GIGE_GTSUBVENDOR_ID, GIGE_GTSUBDEVICE_ID, 0, 0, 0},
    {0,}
};

MODULE_DEVICE_TABLE(pci, GigEDeviceId);

struct pci_driver GigEDriver = {
    .name = MV_MODULE_LONG_NAME,
    .id_table = GigEDeviceId,
    .probe = ModuleProbe,
    .remove = ModuleRemove,
};

struct file_operations gige_fops = {
    .owner          = THIS_MODULE,
    .open           = ModuleOpen,
    .release        = ModuleRelease,
//    .read  = MV_ModuleRead,
//    .write = MV_ModuleWrite,
// #if HAVE_UNLOCKED_IOCTL
    .unlocked_ioctl  = ModuleUnlockedIoctl, 
    .compat_ioctl    = ModuleUnlockedIoctl,
// #else
//     .ioctl           = ModuleIoctl,
// #endif
//    .mmap           = MV_ModuleMmap,
};

void MV_DEBUG_PRINTK(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintk(fmt, args);
    va_end(args);
}


int MV_AddDevice(void* pPciDevice, char* chDeviceName, char* chGigEClassName)
{
    PPCI_DEVICE pstPciDevice = (PPCI_DEVICE) pPciDevice;
    // 内核分配一个尚未使用的主设备号
    // 参数含义：申请的主设备号、次设备号起始、申请次设备号的个数以及执行cat /proc/devices显示的名称
    int nStatus = alloc_chrdev_region(&pstPciDevice->nDevMajor, 0, 1, (const char *)chDeviceName);
    if (nStatus < 0) {
        printk(KERN_ERR"[%s:%d]:failed:alloc_chrdev_region, nStatus[%d]\n", __FUNCTION__, __LINE__, nStatus);
        return nStatus;
    }
    // 将cdev和file_operations关联起来
    cdev_init(&pstPciDevice->stCdev, &gige_fops);
    // 将cdev和设备号关联起来
    nStatus = cdev_add(&pstPciDevice->stCdev, pstPciDevice->nDevMajor, 1);
    if (nStatus < 0) {
        printk(KERN_ERR"[%s:%d]:failed:cdev_add, nStatus[%d]\n", __FUNCTION__, __LINE__, nStatus);

        unregister_chrdev_region(pstPciDevice->nDevMajor, 1);
        return nStatus;
    }
    
    pstPciDevice->pstClass = class_create(THIS_MODULE, (const char *)chGigEClassName);

    // 因需创建多个设备节点，需要将此方法移动到probe函数中实现
    pstPciDevice->pstClassDev = device_create(pstPciDevice->pstClass, NULL, pstPciDevice->nDevMajor, NULL, (const char *)chDeviceName);

    return MV_OK;
}

int MV_RemoveDeivce(void* pPciDevice, void* pBarVir)
{
    PPCI_DEVICE pstPciDevice = (PPCI_DEVICE) pPciDevice;

    // 移除设备节点
    device_destroy(pstPciDevice->pstClass, pstPciDevice->nDevMajor);

    class_destroy(pstPciDevice->pstClass);

    cdev_del(&(pstPciDevice->stCdev));

    unregister_chrdev_region(pstPciDevice->nDevMajor, 1);

    // 释放中断向量
    pci_free_irq_vectors(pstPciDevice->pstPciDev);

    // 释放IO内存
    iounmap(pBarVir);

    pci_release_regions(pstPciDevice->pstPciDev);
    pci_disable_device(pstPciDevice->pstPciDev);

    return MV_OK;
}

int MV_SetDriverData(void* pPciDevice, void* pDriverData, void* pIrqData)
{
    if (NULL == pPciDevice)
    {
        return MV_E_PARAMETER;
    }
    
    if (NULL == pDriverData)
    {
        return MV_E_PARAMETER;
    }

    if (NULL == pIrqData)
    {
        return MV_E_PARAMETER;
    }

    PPCI_DEVICE pstPciDevice = (PPCI_DEVICE) pPciDevice;
    pstPciDevice->pDriverData = pDriverData;
    pstPciDevice->pstPciDev->dev.driver_data = pstPciDevice;
    pstPciDevice->pIrqData = pIrqData;

    return MV_OK;
}

void MV_ReadConfigDword(void* pPciDevice, unsigned long nType, int* value)
{
    PPCI_DEVICE pstPciDevice = (PPCI_DEVICE) pPciDevice;
    pci_read_config_dword(pstPciDevice->pstPciDev, nType, value);
}

void MV_ReadConfigPci(void* pPciDevice, unsigned int* nDevFn, unsigned char* nNumber)
{
    PPCI_DEVICE pstPciDevice = (PPCI_DEVICE) pPciDevice;
    *nNumber = pstPciDevice->pstPciDev->bus->number;
    *nDevFn = pstPciDevice->pstPciDev->devfn;
}

int MV_RequestIrq(void* pPciDevice, irqreturn_t (*interrputfn)(int irq, void *data), void* pDriverData)
{
    PPCI_DEVICE pstPciDevice = NULL;
    PDEVICE_DATA pstDeviceData = NULL;
    int nRet = MV_OK;

    do
    {
        pstPciDevice = (PPCI_DEVICE)pPciDevice;

        pstDeviceData = pstPciDevice->pstDeviceData;
        
        int nIrqCount = 0;
        for (nIrqCount = 0; nIrqCount < MSI_COUNT; nIrqCount++)
        {
            if (CML_MSI_UART_CONTROL_INDEX == nIrqCount)
            {
                continue;
            }

            nRet = request_irq(pstDeviceData->hMSInterrupt[nIrqCount], interrputfn, 0, MV_MODULE_LONG_NAME, pDriverData);
            // printk(KERN_INFO"[%s:%d]:request_irq, irq[%d]\n", __FUNCTION__, __LINE__, pstDeviceData->hMSInterrupt[nIrqCount]);
            if (unlikely(nRet)) {
                printk(KERN_ERR"[%s:%d]:failed:request_irq\n", __FUNCTION__, __LINE__);
                nRet = MV_E_RESOURCE;
                break;
            }
        }

        if (MV_OK != nRet)
        {
            for (nIrqCount = 0; nIrqCount < MSI_COUNT; nIrqCount++)
            {
                if (CML_MSI_UART_CONTROL_INDEX == nIrqCount)
                {
                    continue;
                }

                if (0 < pstDeviceData->hMSInterrupt[nIrqCount])
                {
                    free_irq(pstDeviceData->hMSInterrupt[nIrqCount], pDriverData);
                }
            }
        }
        
        
    } while (MV_FALSE);
    
    

    return nRet;
}

void* MV_AllocConsistent(void* pPciDevice, int nSize, void* pPhy)
{
    PPCI_DEVICE pstPciDevice = (PPCI_DEVICE)pPciDevice;
    return dma_alloc_coherent(&pstPciDevice->pstPciDev->dev, nSize, (dma_addr_t *)pPhy, GFP_ATOMIC);
}

void MV_FreeConsistent(void* pPciDevice, int nSize, void* pPhy, void* pVir)
{
    PPCI_DEVICE pstPciDevice = (PPCI_DEVICE)pPciDevice;
    
    dma_free_coherent(&pstPciDevice->pstPciDev->dev, nSize, pVir, pPhy);
}

int ModuleProbe(struct pci_dev* pstPciDev, const struct pci_device_id* pstPciDeId)
{
    int                             nRet = MV_OK;
    PPCI_DEVICE                     pstPciDevice = NULL;
    PDEVICE_DATA                    pstDeviceData = NULL;

    do
    {
        pstPciDevice = kzalloc(sizeof(PCI_DEVICE), GFP_KERNEL);
        if (NULL == pstPciDevice)
        {
            nRet = MV_E_RESOURCE;
            break;
        }
        

        pstDeviceData = kzalloc(sizeof(DEVICE_DATA), GFP_KERNEL);
        if (NULL == pstDeviceData)
        {
            nRet = MV_E_RESOURCE;
            break;
        }

        pstDeviceData->pBar0Phy = NULL;
        pstDeviceData->pBar0Vir = NULL;

        pstPciDevice->pstDeviceData = pstDeviceData;
        pstPciDevice->pstPciDev = pstPciDev;

    } while (MV_FALSE);

    if (MV_OK != nRet)
    {
        if (pstDeviceData)
        {
            kfree(pstDeviceData);
            pstDeviceData = NULL;
        }
        
        if (pstPciDevice)
        {
            kfree(pstPciDevice);
            pstPciDevice = NULL;
        }
        
        return nRet;
    }
    
    
    if (pci_enable_device(pstPciDev)) {
        nRet = -EIO;
        printk(KERN_ERR"[%s:%d]:pci_enable_device error\n", __FUNCTION__, __LINE__);
        goto pci_enable_err;
    }

    pci_set_master(pstPciDev);

    if (unlikely(pci_request_regions(pstPciDev, MV_MODULE_LONG_NAME))) {
        printk(KERN_ERR"[%s:%d]:failed:pci_request_regions\n", __FUNCTION__, __LINE__);
        nRet = -EIO;
        goto pci_request_regions_err;
    }

    pstDeviceData->pBar0Phy = pci_resource_start(pstPciDev, 0);
    if (pstDeviceData->pBar0Phy < 0) {
        printk(KERN_ERR"[%s:%d]:failed:pci_resource_start\n", __FUNCTION__, __LINE__);
        nRet = -EIO;
        goto pci_resource_start_err;
    }

    pstDeviceData->nBar0Length = pci_resource_len(pstPciDev, 0);   // 4M
    if (pstDeviceData->nBar0Length != 0)
    {
        pstDeviceData->pBar0Vir = ioremap(pstDeviceData->pBar0Phy, pstDeviceData->nBar0Length);
        if (NULL == pstDeviceData->pBar0Vir)
        {
            goto ioremap_err;
        }
        
    }
    else
    {
        nRet = -EIO;
        goto pci_resource_len_err;
    }

    // printk(KERN_INFO"[%s:%d]:bar0 info,bar0_phy[0x%x], bar0_length[%d], bar0_vir[%p], bar0_vir[%lx]\n", __FUNCTION__, __LINE__,
    //     pstDeviceData->pBar0Phy, pstDeviceData->nBar0Length, pstDeviceData->pBar0Vir, pstDeviceData->pBar0Vir);

    // IO访问方式
    pstDeviceData->nBar0IoFlag = pci_resource_flags(pstPciDev, 0);

    if (!(pstDeviceData->nBar0IoFlag & IORESOURCE_MEM)) {
        printk(KERN_ERR"[%s:%d]:bar0_flag not an MMIO resource\n", __FUNCTION__, __LINE__);
    }

    int nRequestIrqNum = pci_alloc_irq_vectors(pstPciDev, 1, MSI_COUNT, PCI_IRQ_MSI);
    if (nRequestIrqNum != MSI_COUNT) {
        printk(KERN_ERR"[%s:%d]:failed:pci_alloc_irq_vectors, nRequestIrqNum[%d]\n", __FUNCTION__, __LINE__, nRequestIrqNum);
        nRet = MV_E_RESOURCE;
        goto pci_alloc_irq_vectors_err;
    }
    pstDeviceData->bMsiFound = true;

    int nIrq = 0;
    for (nIrq = 0; nIrq < MSI_COUNT; nIrq++)
    {
        pstDeviceData->hMSInterrupt[nIrq] = pci_irq_vector(pstPciDev, nIrq);
    }

    nRet = MV_ModuleProbe(pstPciDevice, &pstPciDev->dev, pstDeviceData);

    if (MV_OK == nRet)
    {
        return nRet;
    }
    else
    {
        if (pstDeviceData)
        {
            kfree(pstDeviceData);
            pstDeviceData = NULL;
        }
        
        if (pstPciDevice)
        {
            kfree(pstPciDevice);
            pstPciDevice = NULL;
        }
        
        return nRet;
    }
    
pci_alloc_irq_vectors_err:
    pci_free_irq_vectors(pstPciDev);
    pci_disable_msi(pstPciDev);
    iounmap((void*)pstDeviceData->pBar0Vir);
pci_resource_len_err:
ioremap_err:
pci_resource_start_err:
    pci_release_regions(pstPciDev);
pci_request_regions_err:
    pci_disable_device(pstPciDev);
pci_enable_err:
    return nRet;

}

void ModuleRemove(struct pci_dev* pstPciDev)
{
    PPCI_DEVICE pstPciDevice = pstPciDev->dev.driver_data;
    if (NULL == pstPciDevice)
    {
        return;
    }
    

    size_t i = 0;
    for (i = 0; i < MSI_COUNT; i++)
    {
        if (CML_MSI_UART_CONTROL_INDEX == i)
        {
            continue;
        }

        free_irq(pstPciDev->irq + i, (void*)pstPciDevice->pIrqData);
    }

    MV_ModuleRemove(pstPciDevice, pstPciDevice->pDriverData);

    pci_disable_msi(pstPciDev);

    pstPciDev->dev.driver_data = 0;

    if (pstPciDevice->pstDeviceData)
    {
        kfree(pstPciDevice->pstDeviceData);
        pstPciDevice->pstDeviceData = NULL;
    }

    kfree(pstPciDevice);
    pstPciDevice = NULL;
    
}



int ModuleOpen(struct inode * pstInode, struct file * file)
{
    PPCI_DEVICE pstPciDevice = container_of(pstInode->i_cdev, PCI_DEVICE, stCdev);
    file->private_data = pstPciDevice->pDriverData;

    return MV_ModuleOpen(pstPciDevice->pDriverData);
}

int ModuleRelease( struct inode* aNode, struct file* aFile )
{  
    int nRet = MV_OK;

    void* pDriverData = aFile->private_data;


    MV_ModuleRelease(pDriverData, aFile);

    aFile->private_data = 0;


    return nRet;
}

/*******************************************************************************
@   函数原型：static ssize_t simple_write(struct file *filp, const char *buffer, size_t count, loff_t *ppos)
@   函数功能：用户层向底层写数据(系统函数，格式固定)
@   参数filp： 未使用
@   参数buffer： 上层向底层写入的数据
@   参数count： 上层向底层写入的数据字节数
@   参数ppos： 未使用
@   返回值：写入字节数或错误码
*******************************************************************************/
// static ssize_t MV_ModuleWrite(struct file *filp, const char *buffer, size_t count, loff_t *ppos)
// {
//     if(NULL == buffer)
//     {
//         printk("write buf is NULL\n");
//         return MV_E_PARAMETER;
//     }

//     return ModuleWrite((void*)filp, buffer, count);
// }

/*******************************************************************************
@   函数原型：long  simple_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long        arg)
@   函数功能：用户层向底层写数据(系统函数，格式固定)
@   参数inode： 未使用
@   参数filp： 未使用
@   参数cmd： 上层向底层写入的数据字节数
@   参数arg： 上层向底层写入的数据
@   返回值：写入字节数或错误码
*******************************************************************************/
// #if HAVE_UNLOCKED_IOCTL
long ModuleUnlockedIoctl( struct file* filp, unsigned int cmd, unsigned long arg )
// #else
// long  ModuleIoctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
// #endif
{
 
     //MV_USER_CONFIG stInputConfig = {0};
     //int nRet = copy_from_user(&stInputConfig, (MV_USER_CONFIG*)arg, sizeof(MV_USER_CONFIG)); 
     //MV_Printk("nret [%d]\n", nRet);
 
     //MV_Printk("MV_ModuleIoctl buf: flag[%x] srcip[%x] dstip[%x] paksize[%d] MemMapSize[%d]\n", 
     //    stInputConfig.flag, stInputConfig.nFilterSrcIp, stInputConfig.nFilterDstPort, stInputConfig.nGvspPacketSize, stInputConfig.nMemoryMapSize);
     //return 0;

     //MV_USER_CONFIG *pstArg = (MV_USER_CONFIG*)arg;
     //MV_Printk("test: flag[%x] srcip[%x] dstip[%x] paksize[%d] MemMapSize[%d]\n", 
     //    pstArg->flag, pstArg->nFilterSrcIp, pstArg->nFilterDstPort, pstArg->nGvspPacketSize, pstArg->nMemoryMapSize);

    void* pDriverData = filp->private_data;

    return MV_ModuleIoctl(( void* ) filp, cmd, arg, pDriverData);
}

// int MV_ModuleMmap (struct file * filp, struct vm_area_struct * vma)
// {
//     return ModuleMmap((void*) filp, (void*)vma);
// }


/*******************************************************************************
@   函数原型：static ssize_t simple_read(struct file *filp, unsigned char *buffer, size_t count, loff_t *ppos)
@   函数功能：用户层从底层读取数据(系统函数，格式固定)
@   参数filp： 未使用
@   参数aBuffer： 上层从底层读取的数据
@   参数aCount： 上层从底层读取的数据字节数
@   参数ppos： 未使用
@   返回值：读取字节数或错误码
*******************************************************************************/
//static ssize_t MV_ModuleRead(struct file *filp, /*unsigned*/ char *buffer, size_t count, loff_t *ppos)
//{
//     if(NULL == buffer)
//     {
//         printk("read buf is NULL\n");
//         return MV_E_PARAMETER;
//     }
//    return ModuleRead(buffer, count);
//}



/*******************************************************************************
@   函数原型：int MV_InitModule()    
@   函数功能：模块初始化
@   参数： 无
@   返回值：成功MV_OK 失败MV_E_PARAMETER
*******************************************************************************/
static int InitModule(void)
{
    int nRet = MV_OK;

    printk(KERN_INFO"[%s:%d]:GIGE Driver Enter: Version[%s]\n", __FUNCTION__, __LINE__, MV_VERSION);

    // 注册pci driver
    nRet = pci_register_driver(&GigEDriver);
    if (nRet < 0) {
        printk(KERN_ERR"[%s:%d]:failed:pci_register_driver\n", __FUNCTION__, __LINE__);
        return nRet;
    }

    // printk(KERN_INFO"[%s:%d]:end\n", __FUNCTION__, __LINE__);

    MV_InitAsyncLogMoudle();

    return MV_OK;
}


/*******************************************************************************
@   函数原型：void MV_ExitModule()  
@   函数功能：模块卸载
@   参数： 无
@   返回值：无
*******************************************************************************/     
static void ExitModule(void)  
{
    MV_DeInitAsyncLogMoudle();

    pci_unregister_driver(&GigEDriver);
    printk(KERN_INFO"[%s:%d]:end\n", __FUNCTION__, __LINE__);
}

module_init( InitModule );
module_exit( ExitModule );

MODULE_AUTHOR( MV_COMPANY );
MODULE_LICENSE( MV_COPYRIGHT );
MODULE_DESCRIPTION( MV_MODULE_LONG_NAME );
MODULE_VERSION( MV_VERSION );