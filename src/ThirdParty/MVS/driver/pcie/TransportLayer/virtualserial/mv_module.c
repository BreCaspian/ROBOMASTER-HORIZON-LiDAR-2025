
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
#include "linux/platform_device.h"
#include "linux/tty.h"
#include "linux/serial.h"
#include "linux/tty_flip.h"

#include "linux/serial_core.h"
#include <linux/amba/bus.h>
#include <linux/amba/serial.h>
#include "asm/termbits.h"

#include <linux/poll.h>
#include <linux/tty_flip.h>
#include <linux/circ_buf.h>

#include <asm/io.h>
#include <linux/platform_device.h>
#include "asm/termbits.h"
#include "asm/ioctls.h"

#include "uapi/linux/serial_reg.h"

#include "linux/of.h"

#include "mv_module.h"
#include "mv_misc.h"
#include "mv_lock.h"
#include "mv_memory.h"
#include "mv_device.h"

#include "MvErrorDefine.h"

#if 0

#include "include/linux/serial_core.h"


#endif

#define BOTH_EMPTY     (UART_LSR_TEMT | UART_LSR_THRE)

void MV_DEBUG_PRINTK(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintk(fmt, args);
    va_end(args);
}

struct platform_driver virSerialDriver = {
    .probe = VirSerial_ModuleProbe,
    .remove = VirSerial_ModuleRemove,
    .driver = {
        .name = VIRTUAL_SERIAL_DEVICE_NAME,
        .owner = THIS_MODULE,
    }
};

struct file_operations virserial_fops = {
    .open           = ModuleOpen,
    .release        = ModuleRelease,
    .read           = ModuleRead,
    .write          = ModuleWrite,
    .unlocked_ioctl  = ModuleUnlockedIoctl, 
    .compat_ioctl    = ModuleUnlockedIoctl,
};

static struct uart_ops virtual_uart_ops = {
    .tx_empty       = virserial_tx_empty,             /* 检测发送FIFO缓冲区是否为空                  */
    .set_mctrl      = virserial_set_mctrl,            // 设置串口流控      发现设备的时候执行/关闭句柄会执行
    .get_mctrl      = virserial_get_mctrl,            /* 检测串口流控                                   */
    .stop_tx        = virserial_stop_tx,              /* 停止发送                                     */
    .start_tx       = virserial_start_tx,             /* 开始发送                                     */
    .stop_rx        = virserial_stop_rx,              // 停止接收          关闭句柄的时候会执行
    .enable_ms      = virserial_enable_ms,            /* 空函数                                      */
    .break_ctl      = virserial_break_ctl,            /* 发送break信号                                */
    .startup        = virserial_startup,              // 串口发送/接收,以及中断初始函数,打开串口的时候执行
    .shutdown       = virserial_shutdown,             // 关闭串口,关闭句柄的时候会执行
    .flush_buffer   = virserial_flush_buffer,         /* 刷新缓冲区，并丢弃任何剩下的数据             */
    .set_termios    = virserial_set_termios,          // 设置串口波特率，数据位等,打开串口的时候执行
    .type           = virserial_type,                 // 串口类型,发现设备的时候执行
    .release_port   = virserial_release_port,         /* 释放串口                                    */
    .request_port   = virserial_request_port,         /* 申请串口                                    */
    .config_port    = virserial_config_port,          // 串口的一些配置,发现设备的时候执行
    .verify_port    = virserial_verify_port,          /* 串口检测                                    */
    .ioctl          = virserial_ioctl_port,
#ifdef CONFIG_CONSOLE_POLL                          /*                                              */
    .poll_get_char  = NULL,                         /* 设备阻塞与非阻塞访问相关                     */
    .poll_put_char  = NULL,
#endif
};

struct uart_driver uartVirSerialDriver = {
    .owner          = THIS_MODULE,
    .driver_name    = VIRTUAL_UART_DRIVER_NAME,
    .dev_name       = VIRTUAL_UART_DEVICE_NAME,
    .major          = 0,                            /* 设为0，表示由系统分配设备号                   */
    .minor          = 0,
    .nr             = 20,                            /* 设备数量                                     */
    .cons           = NULL,                         /* 不支持serial console                            */
    // .state          = NULL,
    // .tty_driver     = NULL
};


int MV_AddDevice(PSERIAL_DEVICE pstSerialDevice)
{

    int                         nRet = MV_OK;
    struct cdev*                pstCDev = NULL;
    struct platform_device*     pstPlatformDev = NULL;

    // printk(KERN_INFO"MV_AddDevice enter");

    if (NULL == pstSerialDevice)
    {
        return -ENAVAIL;
    }
    pstCDev = &(pstSerialDevice->stCdev);
    pstPlatformDev = pstSerialDevice->pPlatformDev;

    // 内核分配一个尚未使用的主设备号
    // 参数含义：申请的主设备号、次设备号起始、申请次设备号的个数以及执行cat /proc/devices显示的名称
    int nStatus = alloc_chrdev_region(&pstSerialDevice->nDevMajor, 0, 1, (const char *)pstSerialDevice->chVirSerialDevName);
    if (nStatus < 0) {
        printk(KERN_ERR"[%s:%d]:failed:alloc_chrdev_region, nStatus[%d]\n", __FUNCTION__, __LINE__, nStatus);
        return nStatus;
    }
    // 将cdev和file_operations关联起来
    cdev_init(pstCDev, &virserial_fops);
    pstCDev->owner = THIS_MODULE;
    // 将cdev和设备号关联起来
    nStatus = cdev_add(pstCDev, pstSerialDevice->nDevMajor, 1);
    if (nStatus < 0) {
        printk(KERN_ERR"[%s:%d]:failed:cdev_add, nStatus[%d]\n", __FUNCTION__, __LINE__, nStatus);

        unregister_chrdev_region(pstSerialDevice->nDevMajor, 1);
        return nStatus;
    }
    
    pstSerialDevice->pCdvClass = class_create(THIS_MODULE, (const char *)pstSerialDevice->chVirSerialDevClassName);

    // 因需创建多个设备节点，需要将此方法移动到probe函数中实现
    pstSerialDevice->pCdvClassDev = device_create(pstSerialDevice->pCdvClass, NULL, pstSerialDevice->nDevMajor, NULL, (const char *)pstSerialDevice->chVirSerialDevName);
    // printk(KERN_INFO"MV_AddDevice end!, DevClassName[%s],DevName[%s]", pstSerialDevice->chVirSerialDevClassName, pstSerialDevice->chVirSerialDevName);
    return MV_OK;
}

int MV_RemoveDeivce(PSERIAL_DEVICE pstSerialDevice)
{
    struct cdev*                pstCDev = NULL;
    struct platform_device*     pstPlatformDev = NULL;

    pstCDev = &(pstSerialDevice->stCdev);
    pstPlatformDev = pstSerialDevice->pPlatformDev;

    // 移除设备节点
    device_destroy(pstSerialDevice->pCdvClass, pstSerialDevice->nDevMajor);

    class_destroy(pstSerialDevice->pCdvClass);

    cdev_del(pstCDev);

    unregister_chrdev_region(pstSerialDevice->nDevMajor, 1);

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

    return MV_OK;
}

void MV_ReadConfigDword(void* pPciDevice, unsigned long nType, int* value)
{

}

void MV_ReadConfigPci(void* pPciDevice, unsigned int* nDevFn, unsigned char* nNumber)
{

}

int MV_RequestIrq(unsigned int nIrq, irqreturn_t (*interrputfn)(int irq, void *data), void* pDriverData)
{
    int nRet = MV_OK;

    do
    {
        nRet = request_irq(nIrq, interrputfn, 0, MV_MODULE_LONG_NAME, pDriverData);
        
        // printk(KERN_INFO"[%s:%d]:request_irq, irq[%d], pDriverData[%lx]\n", __FUNCTION__, __LINE__, nIrq, pDriverData);
        if (unlikely(nRet)) {
            printk(KERN_ERR"[%s:%d]:failed:request_irq\n", __FUNCTION__, __LINE__);
            nRet = MV_E_RESOURCE;
            break;
        }
        
        
    } while (MV_FALSE);
    
    

    return nRet;
}

extern int MV_UartDataSet(PSERIAL_DEVICE pstSerialDevice)
{
    struct uart_port*               pstUartPort = NULL;

    pstUartPort = pstSerialDevice->pUartPort;
    
    pstUartPort->private_data = pstSerialDevice;

    return MV_OK;
}

void MV_FreeIrq(unsigned int nIrq, void* pDriverData)
{
    free_irq(nIrq, pDriverData);
    // printk(KERN_INFO"[%s:%d]:free_irq, irq[%d]\n", __FUNCTION__, __LINE__, nIrq);
}


int VirSerial_ModuleProbe(struct platform_device * pstPlatformDevice)
{
    int                             nRet = MV_OK;
    PSERIAL_DEVICE                  pstSerialDevice = NULL;
    struct uart_port*               pstUartPort = NULL;
    struct cdev*                    pstCDev = NULL;
    bool                            bAddPort = MV_FALSE;
    int                             nId = 0;

    // printk(KERN_INFO"VirSerial_ModuleProbe enter");
    do
    {
        pstSerialDevice = kzalloc(sizeof(SERIAL_DEVICE), GFP_KERNEL);
        if (NULL == pstSerialDevice)
        {
            nRet = -ENOMEM;
            break;
        }

        pstSerialDevice->pUartPort = kzalloc(sizeof(struct uart_port), GFP_KERNEL);
        if (NULL == pstSerialDevice->pUartPort)
        {
            nRet = -ENOMEM;
            break;
        }

        pstSerialDevice->pPlatformDev = pstPlatformDevice;

        pstCDev = &(pstSerialDevice->stCdev);

        pstUartPort = pstSerialDevice->pUartPort;

        pstUartPort->dev = &(pstPlatformDevice->dev);
        pstUartPort->mapbase = 0;
        pstUartPort->membase = (unsigned char *)(0xdeadbeef);
        pstUartPort->iotype = UPIO_MEM;
        pstUartPort->irq = 0;
        pstUartPort->fifosize = 16;
        pstUartPort->ops = &virtual_uart_ops;
        pstUartPort->flags = UPF_BOOT_AUTOCONF;
        pstUartPort->line = pstPlatformDevice->id;
        pstUartPort->type = PORT_16550;
        pstUartPort->uartclk = 9600 * 16;

        // printk(KERN_INFO"[Serial]uart_add_one_port enter, id[%d]", pstPlatformDevice->id);
        nRet = uart_add_one_port(&uartVirSerialDriver, pstUartPort);
        if (MV_OK != nRet)
        {
            printk(KERN_ERR"uart_add_one_port failed![%d], nId[%d]", nRet, nId);
            break;
        }
        // printk(KERN_INFO"[Serial]uart_add_one_port end");

        bAddPort = MV_TRUE;
        // pstSerialDevice->pPlatformData = pstPlatformDevice->dev.platform_data;

        pstSerialDevice->pPlatformData = pstPlatformDevice->dev.platform_data;
        pstPlatformDevice->dev.driver_data = pstSerialDevice;
    
        nRet = MV_ModuleProbe(pstSerialDevice);
        if (MV_OK != nRet)
        {
            printk(KERN_ERR"MV_ModuleProbe failed![%d]", nRet);
            break;
        }

        pstSerialDevice->pChTtyName = pstUartPort->name;
        pstSerialDevice->nLine = pstUartPort->line;

    } while (MV_FALSE);


    if (MV_OK != nRet)
    {
        if (bAddPort)
        {
            uart_remove_one_port(&uartVirSerialDriver, pstUartPort);
        }
        
        if (pstSerialDevice)
        {
            if (pstSerialDevice->pUartPort)
            {
                kfree(pstSerialDevice->pUartPort);
                pstSerialDevice->pUartPort = NULL;
            }

            kfree(pstSerialDevice);
            pstSerialDevice = NULL;
        }
        
    }

    // printk(KERN_INFO"VirSerial_ModuleProbe end");
    return nRet;

}

int VirSerial_ModuleRemove(struct platform_device * pstPlatformDevice)
{
    int                             nRet = MV_OK;
    PSERIAL_DEVICE                  pstSerialDevice = NULL;
    struct uart_port*               pstUartPort = NULL;
    // printk(KERN_INFO"[%s:%d][Serial]:enter\n", __FUNCTION__, __LINE__);
    do
    {
        pstSerialDevice = pstPlatformDevice->dev.driver_data;
        if (NULL == pstSerialDevice)
        {
            nRet = -ENAVAIL;
            break;
        }

        pstUartPort = pstSerialDevice->pUartPort;
        if (NULL == pstUartPort)
        {
            nRet = -ENAVAIL;
            break;
        }

        uart_remove_one_port(&uartVirSerialDriver, pstUartPort);

        // MV_RemoveDeivce(pstSerialDevice);


        MV_ModuleRemove(pstSerialDevice);
    } while (MV_FALSE);

    

    if (pstSerialDevice)
    {
        if (pstSerialDevice->pUartPort)
        {
            kfree(pstSerialDevice->pUartPort);
            pstSerialDevice->pUartPort = NULL;
        }

        kfree(pstSerialDevice);
        pstSerialDevice = NULL;
    }

    return nRet;
}



int ModuleOpen(struct inode * pstInode, struct file * file)
{
    return 0;
}

int ModuleRelease( struct inode* aNode, struct file* aFile )
{
    int                         nRet = MV_OK;
    PSERIAL_DEVICE              pstSerialDev = NULL;


    // printk(KERN_INFO"[%s:%d]:enter\n", __FUNCTION__, __LINE__);


    pstSerialDev = aFile->private_data;

    pstSerialDev->pFile = aFile;

    MV_ModuleRelease(pstSerialDev, aFile);

    aFile->private_data = 0;


    return nRet;
}

ssize_t ModuleRead (struct file *pstFile, char __user * pchUserBuffer, size_t nBufferLen, loff_t * offset)
{
    return 0;
}

ssize_t ModuleWrite (struct file *pstFile, const char __user * pchUserBuffer, size_t nBufferLen, loff_t * offset)
{

    return 0;
}

/*******************************************************************************
@   函数原型：long  simple_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long        arg)
@   函数功能：用户层向底层写数据(系统函数，格式固定)
@   参数inode： 未使用
@   参数filp： 未使用
@   参数cmd： 上层向底层写入的数据字节数
@   参数arg： 上层向底层写入的数据
@   返回值：写入字节数或错误码
*******************************************************************************/
long ModuleUnlockedIoctl( struct file* filp, unsigned int cmd, unsigned long arg)
{

    void* pDriverData = filp->private_data;

    return MV_ModuleIoctl(( void* ) filp, cmd, arg, pDriverData);
}

/**
 * @brief 串口的Tx FIFO缓存是否为空。如果为空，函数应返回TIOCSER_TEMT，否则返回0。如果端口不支持此操作，返回TIOCSER_TEMT。
 * 
 * @param pstUartPort 
 * @return unsigned int 
 */
extern unsigned int virserial_tx_empty(struct uart_port * pstUartPort)
{
    // printk(KERN_INFO"[%s:%d]: enter\n",__FUNCTION__, __LINE__);
    return TIOCSER_TEMT;
}

/**
 * @brief 设置串口modem控制
 * 
 * @param pstUartPort 
 * @param mctrl 
 */
extern void virserial_set_mctrl(struct uart_port * pstUartPort, unsigned int mctrl)
{
    // printk(KERN_INFO"[%s:%d]: enter\n",__FUNCTION__, __LINE__);
}

/**
 * @brief 获取串口modem控制
 * 
 * @param pstUartPort 
 * @return unsigned int 
 */
extern unsigned int virserial_get_mctrl(struct uart_port * pstUartPort)
{
    // printk(KERN_INFO"[%s:%d]: enter\n",__FUNCTION__, __LINE__);
    return MV_OK;
}

/**
 * @brief 禁止串口发送数据
 * 
 * @param pstUartPort 
 */
extern void virserial_stop_tx(struct uart_port * pstUartPort)
{
    // printk(KERN_INFO"[%s:%d]: enter\n",__FUNCTION__, __LINE__);
}

/**
 * @brief 使能串口发送数据
 * 
 * @param pstUartPort 
 */
extern void virserial_start_tx(struct uart_port * pstUartPort)
{
    // printk(KERN_INFO"[%s:%d]: enter\n",__FUNCTION__, __LINE__);

    int                         nRet = MV_OK;
    PSERIAL_DEVICE              pstSerialDev = NULL;
    struct uart_port *port = pstUartPort;
    struct circ_buf *xmit = &port->state->xmit;
    int count;
    int tx_count = 0;


    if (uart_circ_empty(xmit)) {
        return;
    }

    pstSerialDev = pstUartPort->private_data;

    if (NULL == pstSerialDev)
    {
        return;
    }

    count = (xmit->head - xmit->tail);
    do {
        pstSerialDev->pTxBuffer = xmit->buf;
        // printk(KERN_INFO"[%s:%d]: xmit buf[%02x], tail[%d], head[%d]\n",__FUNCTION__, __LINE__, xmit->buf[xmit->tail], xmit->tail, xmit->head);
        xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
        port->icount.tx++;
        tx_count++;
        if (uart_circ_empty(xmit))
        {
            xmit->head = 0;
            xmit->tail = 0;
            // printk(KERN_WARNING"[%s:%d]: xmit buf is empty\n",__FUNCTION__, __LINE__);
            break;
        }
    } while (--count > 0);

    if (0 == tx_count)
    {
        return;
    }
    

    pstSerialDev->nTxBufferLen = tx_count;

    // printk(KERN_INFO"[%s:%d]: xmit buf[%d], tx_count[%d]\n",__FUNCTION__, __LINE__, xmit->buf, tx_count);

    nRet = MV_ModuleWrite(pstSerialDev);
}


/**
 * @brief 禁止串口接收数据
 * 
 * @param pstUartPort 
 */
extern void virserial_stop_rx(struct uart_port * pstUartPort)
{
    // printk(KERN_INFO"[%s:%d]: enter\n",__FUNCTION__, __LINE__);
}

/**
 * @brief 使能modem的状态信号
 * 
 * @param pstUartPort 
 */
extern void virserial_enable_ms(struct uart_port * pstUartPort)
{
    // printk(KERN_INFO"[%s:%d]: enter\n",__FUNCTION__, __LINE__);
}

/**
 * @brief 设置break信号
 * 
 * @param pstUartPort 
 * @param break_state 
 */
extern void virserial_break_ctl(struct uart_port * pstUartPort, int break_state)
{
    // printk(KERN_INFO"[%s:%d]: enter\n",__FUNCTION__, __LINE__);
}

/**
 * @brief 启动串口,应用程序打开串口设备文件时,该函数会被调用
 * 
 * @param pstUartPort 
 * @return int 
 */
extern int virserial_startup(struct uart_port * pstUartPort)
{
    // printk(KERN_INFO"[%s:%d]: enter\n",__FUNCTION__, __LINE__);
    MV_ModuleOpen(NULL);
    return MV_OK;
}


/**
 * @brief 关闭串口,应用程序关闭串口设备文件时,该函数会被调用
 * 
 * @param pstUartPort 
 */
extern void virserial_shutdown(struct uart_port * pstUartPort)
{
    // printk(KERN_INFO"[%s:%d]: enter\n",__FUNCTION__, __LINE__);
}

/**
 * @brief 刷新队列
 * 
 * @param pstUartPort 
 */
extern void virserial_flush_buffer(struct uart_port * pstUartPort)
{
    // printk(KERN_INFO"[%s:%d]: enter\n",__FUNCTION__, __LINE__);
}

/**
 * @brief 设置串口参数
 * 
 * @param pstUartPort 
 * @param new 
 * @param old 
 */
#if (LINUX_VERSION_CODE > KERNEL_VERSION(6,0,19) )
extern void virserial_set_termios(struct uart_port * pstUartPort, struct ktermios *new, const struct ktermios *old)
#else
extern void virserial_set_termios(struct uart_port * pstUartPort, struct ktermios *new, struct ktermios *old)
#endif
{
    PSERIAL_DEVICE                  pstSerialDev = NULL;
    // printk(KERN_INFO"[%s:%d]: enter\n",__FUNCTION__, __LINE__);

    pstSerialDev = pstUartPort->private_data;

    MV_TermiosInit(pstSerialDev, new);

    new->c_iflag = 0x4;
    new->c_oflag = 0x4;
    new->c_cflag = 0x1cb2;
    new->c_lflag = 0xa30;
    new->c_cc[VINTR] = 3;
    new->c_cc[VQUIT] = 28;
    new->c_cc[VERASE] = 127;
    new->c_cc[VKILL] = 21;
    new->c_cc[VEOF] = 4;
    new->c_cc[VTIME] = 1;
    new->c_cc[VMIN] = 1;
    new->c_cc[VSWTC] = 0;
    new->c_cc[VSTART] = 17;
    new->c_cc[VSTOP] = 19;
    new->c_cc[VSUSP] = 26;
    new->c_cc[VEOL] = 0;
    new->c_cc[VREPRINT] = 18;
    new->c_cc[VDISCARD] = 15;
    new->c_cc[VWERASE] = 23;
    new->c_cc[VLNEXT] = 22;
    new->c_cc[VEOL2] = 0;
    new->c_ispeed = 0x1c200;
    new->c_ospeed = 0x1c200;

    unsigned int baud, quot;
    uart_update_timeout(pstUartPort, new->c_cflag, 115200);
}

extern const char* virserial_type(struct uart_port *pstUartPort)
{
    // printk(KERN_INFO"[%s:%d]: enter\n",__FUNCTION__, __LINE__);
    return pstUartPort->name;
}

/**
 * @brief 释放串口已申请的IO端口/IO内存资源,必要时还需iounmap
 * 
 * @param pstUartPort 
 */
extern void virserial_release_port(struct uart_port *pstUartPort)
{
    // printk(KERN_INFO"[%s:%d]: enter\n",__FUNCTION__, __LINE__);
}

/**
 * @brief 申请必要的IO端口/IO内存资源,必要时还可以重新映射串口端口
 * 
 * @param pstUartPort 
 * @return int 
 */
extern int virserial_request_port(struct uart_port *pstUartPort)
{
    // printk(KERN_INFO"[%s:%d]: enter\n",__FUNCTION__, __LINE__);
    return MV_OK;
}

/**
 * @brief 执行串口所需的自动配置
 * 
 * @param pstUartPort 
 * @param data 
 */
extern void virserial_config_port(struct uart_port *pstUartPort, int data)
{
    // printk(KERN_INFO"[%s:%d]: enter, data[%d]\n",__FUNCTION__, __LINE__, data);

    pstUartPort->type = PORT_16550;
}

/**
 * @brief 核实新串口的信息
 * 
 * @param pstUartPort 
 * @param pstSerial 
 * @return int 
 */
extern int virserial_verify_port(struct uart_port *pstUartPort, struct serial_struct *pstSerial)
{
    // printk(KERN_INFO"[%s:%d]: enter\n",__FUNCTION__, __LINE__);
    return MV_OK;
}

/**
 * @brief ioctl控制
 * 
 * @param port 
 * @param cmd 
 * @param args 
 * @return int 
 */
extern int virserial_ioctl_port(struct uart_port * port, unsigned int cmd, unsigned long args)
{
    unsigned int nMvCmd = 0;
    // printk(KERN_INFO"[%s:%d]: enter\n",__FUNCTION__, __LINE__);
    // printk(KERN_INFO"[%s:%d]: enter, name[%s], timeout[%d]\n",__FUNCTION__, __LINE__, port->name, port->timeout);

    switch (cmd)
    {
        case TCGETS:
            nMvCmd = MV_TCGETS;
            break;
        case TCSETS:
            nMvCmd = MV_TCSETS;
            break;
        case TCFLSH:
            nMvCmd = MV_TCFLSH;
            break;
        default:
            nMvCmd = cmd;
            break;
    }

    return MV_ModuleIoctl(NULL, nMvCmd, args, port->private_data);
}

#ifdef CONFIG_CONSOLE_POLL
// extern int        (*poll_init)(struct uart_port *);
// extern void virserial_poll_put_char(struct uart_port *pstUartPort, unsigned char data);
// extern int virserial_poll_get_char(struct uart_port *pstUartPort);
#endif

extern int virserial_read_char(PSERIAL_DEVICE pstSerialDev, char chData)
{
    struct uart_port *              pstUartPort = NULL;
    struct tty_port *               pstTtyport = NULL;
    if (NULL == pstSerialDev)
    {
        return -EINVAL;
    }
    
    pstUartPort = pstSerialDev->pUartPort;

    if (NULL == pstUartPort)
    {
        return -EINVAL;
    }
    pstTtyport = &(pstUartPort->state->port);
    pstUartPort->icount.rx ++;

    int nRet = tty_insert_flip_char(pstTtyport, chData, TTY_NORMAL);
    // printk(KERN_INFO"tty_insert_flip_char: 0x%02x is read char, nRet %d", chData, nRet);
    return nRet;
}

extern void virserial_read_push(PSERIAL_DEVICE pstSerialDev)
{
    struct uart_port *              pstUartPort = NULL;
    struct tty_port *               pstTtyport = NULL;
    if (NULL == pstSerialDev)
    {
        return;
    }
    
    pstUartPort = pstSerialDev->pUartPort;

    if (NULL == pstUartPort)
    {
        return;
    }
    pstTtyport = &(pstUartPort->state->port);
    // printk(KERN_INFO"tty_flip_buffer_push");
    tty_flip_buffer_push(pstTtyport);
}


int MV_TermiosMalloc(PMV_KTERMIOS pstKtermios)
{

    if (NULL == pstKtermios)
    {
        return -EINVAL;
    }
    
    if (pstKtermios->pKtermios)
    {
        kfree(pstKtermios->pKtermios);
        pstKtermios->pKtermios = NULL;
    }

    pstKtermios->pKtermios = (struct ktermios*)kmalloc(sizeof(struct ktermios), GFP_KERNEL);
    if (NULL == pstKtermios->pKtermios)
    {
        return -ENOMEM;
    }
    
    return 0;
}

int MV_TermiosFree(PMV_KTERMIOS pstKtermios)
{
    if (NULL == pstKtermios)
    {
        return -EINVAL;
    }

    if (pstKtermios->pKtermios)
    {
        kfree(pstKtermios->pKtermios);
        pstKtermios->pKtermios = NULL;
    }

    return 0;
}


int MV_TermiosFromUser(unsigned long nArg, PMV_KTERMIOS pstKtermios)
{
    if (NULL == pstKtermios || NULL == pstKtermios->pKtermios)
    {
        return -EINVAL;
    }

    return copy_from_user( pstKtermios->pKtermios, nArg, sizeof(struct termios));
}

int MV_TermiosToUser(unsigned long nArg, void* pKtermios)
{
    if (NULL == pKtermios)
    {
        return -EINVAL;
    }
    
    return copy_to_user(nArg, pKtermios, sizeof(struct termios));
}

int MV_GetUartBaudrate(void* uart, void* pKtermiosNew, void* pKtermiosOld, int* pnBaudrate)
{
    if (NULL == uart || NULL == pKtermiosNew || NULL == pKtermiosOld || NULL == pnBaudrate)
    {
        return -EINVAL;
    }
    
    *pnBaudrate = uart_get_baud_rate(uart, pKtermiosNew, pKtermiosOld, 0, 921600);

    return 0;
}

int MV_UpdateUartTimeout(void* uart, unsigned int nCcflag, int nBaudrate)
{
    if (NULL == uart)
    {
        return -EINVAL;
    }

    uart_update_timeout(uart, nCcflag, nBaudrate);

    return 0;
}

int MV_GetTermiosCcflag(void* pKtermios, unsigned int* pValue)
{
    struct ktermios *           pstKermios;

    if (NULL == pKtermios)
    {
        return -EINVAL;
    }
    
    pstKermios = pKtermios;

    *pValue = pstKermios->c_cflag;

    return 0;
}

int MV_GetTermiosCiflag(void* pKtermios, unsigned int* pValue)
{
    struct ktermios *           pstKermios;

    if (NULL == pKtermios)
    {
        return -EINVAL;
    }
    
    pstKermios = pKtermios;

    *pValue = pstKermios->c_iflag;

    return 0;
}

int MV_SetTermiosCiflag(void* pKtermiosNew, void* pKtermiosOld)
{
    struct ktermios *           pstKermiosNew;
    struct ktermios *           pstKermiosOld;

    if (NULL == pKtermiosNew || NULL == pKtermiosOld)
    {
        return -EINVAL;
    }
    
    pstKermiosNew = pKtermiosNew;
    pstKermiosOld = pKtermiosOld;

    pstKermiosOld->c_iflag = pstKermiosNew->c_iflag;

    return 0;

}


int MV_SetTermiosCoflag(void* pKtermiosNew, void* pKtermiosOld)
{
    struct ktermios *           pstKermiosNew;
    struct ktermios *           pstKermiosOld;

    if (NULL == pKtermiosNew || NULL == pKtermiosOld)
    {
        return -EINVAL;
    }
    
    pstKermiosNew = pKtermiosNew;
    pstKermiosOld = pKtermiosOld;

    pstKermiosOld->c_oflag = pstKermiosNew->c_oflag;

    return 0;
}

int MV_SetTermiosCcflag(void* pKtermiosNew, void* pKtermiosOld)
{
    struct ktermios *           pstKermiosNew;
    struct ktermios *           pstKermiosOld;

    if (NULL == pKtermiosNew || NULL == pKtermiosOld)
    {
        return -EINVAL;
    }
    
    pstKermiosNew = pKtermiosNew;
    pstKermiosOld = pKtermiosOld;

    pstKermiosOld->c_cflag = pstKermiosNew->c_cflag;

    return 0;
}

int MV_SetTermiosClflag(void* pKtermiosNew, void* pKtermiosOld)
{
    struct ktermios *           pstKermiosNew;
    struct ktermios *           pstKermiosOld;

    if (NULL == pKtermiosNew || NULL == pKtermiosOld)
    {
        return -EINVAL;
    }
    
    pstKermiosNew = pKtermiosNew;
    pstKermiosOld = pKtermiosOld;

    pstKermiosOld->c_lflag = pstKermiosNew->c_lflag;

    return 0;
}

int MV_SetTermiosCline(void* pKtermiosNew, void* pKtermiosOld)
{
    struct ktermios *           pstKermiosNew;
    struct ktermios *           pstKermiosOld;

    if (NULL == pKtermiosNew || NULL == pKtermiosOld)
    {
        return -EINVAL;
    }
    
    pstKermiosNew = pKtermiosNew;
    pstKermiosOld = pKtermiosOld;

    pstKermiosOld->c_line = pstKermiosNew->c_line;

    return 0;
}


int MV_SetTermiosCCc(void* pKtermiosNew, void* pKtermiosOld)
{
    struct ktermios *           pstKermiosNew;
    struct ktermios *           pstKermiosOld;
    size_t                      i = 0;
    if (NULL == pKtermiosNew || NULL == pKtermiosOld)
    {
        return -EINVAL;
    }
    
    pstKermiosNew = pKtermiosNew;
    pstKermiosOld = pKtermiosOld;

    for (i = 0; i < NCCS; i++)
    {
        pstKermiosOld->c_cc[i] = pstKermiosNew->c_cc[i];
    }

    return 0;
}

int MV_SetTermiosSpeed(void* pKtermiosNew, void* pKtermiosOld)
{
    struct ktermios *           pstKermiosNew;
    struct ktermios *           pstKermiosOld;

    if (NULL == pKtermiosNew || NULL == pKtermiosOld)
    {
        return -EINVAL;
    }
    
    pstKermiosNew = pKtermiosNew;
    pstKermiosOld = pKtermiosOld;

    pstKermiosOld->c_ispeed = pstKermiosNew->c_ispeed;
    pstKermiosOld->c_ospeed = pstKermiosNew->c_ospeed;

    return 0;
}


/*******************************************************************************
@   函数原型：int MV_InitModule()    
@   函数功能：模块初始化
@   参数： 无
@   返回值：成功MV_OK 失败MV_E_PARAMETER
*******************************************************************************/
static int InitModule(void)
{
    int nRet = MV_OK;

    printk(KERN_INFO"[%s:%d]:Vir Serial Driver Enter: Version[%s]\n", __FUNCTION__, __LINE__, MV_VERSION);

    nRet = MV_InitModule();
    if (MV_OK != nRet)
    {
        printk(KERN_ERR"[%s:%d]:failed:init global resource, [%d]\n", __FUNCTION__, __LINE__, nRet);
        return nRet;
    }

    nRet = uart_register_driver(&uartVirSerialDriver);
    if (nRet < 0) {
        printk(KERN_ERR"[%s:%d]:failed:uart driver register, [%d]\n", __FUNCTION__, __LINE__, nRet);
        return nRet;
    }

    // 注册platform driver
    nRet = platform_driver_register(&virSerialDriver);
    if (nRet < 0) {
        printk(KERN_ERR"[%s:%d]:failed:platform driver register, [%d]\n", __FUNCTION__, __LINE__, nRet);
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
    // printk(KERN_INFO"[%s:%d][Serial]:enter\n", __FUNCTION__, __LINE__);
    MV_DeInitAsyncLogMoudle();

    platform_driver_unregister(&virSerialDriver);

    uart_unregister_driver(&uartVirSerialDriver);

    MV_DeInitModule();

    printk(KERN_INFO"[%s:%d][Serial]:end\n", __FUNCTION__, __LINE__);
}

module_init( InitModule );
module_exit( ExitModule );

MODULE_AUTHOR( MV_COMPANY );
MODULE_LICENSE( MV_COPYRIGHT );
MODULE_DESCRIPTION( MV_MODULE_LONG_NAME );
MODULE_VERSION( MV_VERSION );