#ifndef _MV_FG_CTRL_H_
#define _MV_FG_CTRL_H_

#include "MVFGDefines.h"
#include "MVFGErrorDefine.h"

/************************************************************************/
/* 动态库导入导出定义                                                   */
/* Import and export definition of the dynamic library                  */
/************************************************************************/
#ifndef MV_FGCTRL_API
    #if (defined (_WIN32) || defined(WIN64))
        #if defined(MV_FGCTRL_EXPORTS)
            #define MV_FGCTRL_API __declspec(dllexport)
        #else
            #define MV_FGCTRL_API __declspec(dllimport)
        #endif
    #else
        #ifndef __stdcall
            #define __stdcall
        #endif 

        #ifndef MV_FGCTRL_API
            #if defined(MV_FGCTRL_ROC_EXPORTS)
                #define  MV_FGCTRL_API __attribute__((visibility("default")))
            #else
                #define  MV_FGCTRL_API
            #endif
        #endif
    #endif
#endif

#ifndef IN
    #define IN
#endif

#ifndef OUT
    #define OUT
#endif

#ifdef __cplusplus
extern "C" {
#endif 

/***********************************************************************/
/*                             版本信息                                */
/*                        Version information                          */
/***********************************************************************/
/********************************************************************//**
 *  @~chinese
 *  @brief  获取SDK版本信息
 *  @return 以字符串形式返回SDK版本信息
 *  @remarks 格式：“版本 + 类型 + 编译时间”
  
 *  @~english
 *  @brief  Get SDK Version Information
 *  @return Return SDK version information in a string.
 *  @remarks Format: "Version number + Type + Compile Time".
 ************************************************************************/
MV_FGCTRL_API unsigned char* __stdcall MV_FG_GetSDKVersion();


/***********************************************************************/
/*                             基本操作                                */
/*                         Basic operation                             */
/***********************************************************************/
/********************************************************************//**
 *  @~chinese
 *  @brief  初始化SDK资源
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 可以在调用其他接口之前调用，初始化SDK资源。 \n
 *  @~english
 *  @brief  Initialize SDK resources 
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks It can be called before calling other functions to initialize SDK resources.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_Initialize();

/********************************************************************//**
 *  @~chinese
 *  @brief  结束SDK调用，销毁资源
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 该接口调用后，SDK会释放申请的所有资源；如果要继续使用SDK接口，需要先调用MV_FG_Initialize或者重新枚举采集卡。 \n
 *  @~english
 *  @brief  End the SDK call and destroy the resource
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks It can be called before calling other functions to initialize SDK resources.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_Finalize();

/********************************************************************//**
 *  @~chinese
 *  @brief  更新采集卡列表
 *  @param  nTLayerType             [IN]           采集卡类型
 *  @param  pbChanged               [OUT]          采集卡列表是否改变
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 进行采集卡操作之前必须调用该接口，只有在调用该接口时内部采集卡列表才会更新。 \n
             采集卡类型说明如下：\n
                | 序号 | 采集卡类型定义 | 值 | 说明 |
                | :---:| :---:| :---:| :---:|
                | 1 | MV_FG_GEV_INTERFACE | 0x00000001 | GigE Vision采集卡 |
                | 2 | MV_FG_CAMERALINK_INTERFACE | 0x00000004 | Camera Link采集卡 |
                | 3 | MV_FG_CXP_INTERFACE | 0x00000008 | CoaXPress采集卡 |
                | 4 | MV_FG_XoF_INTERFACE | 0x00000010 | XoFLink采集卡 |
 *  @~english
 *  @brief  Update frame grabber list
 *  @param  nTLayerType             [IN]            Frame grabber type
 *  @param  pbChanged               [OUT]           Whether the frame grabber list changed
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks This API must be called before the operation of the frame grabber. The internal frame grabber list will be updated only when this API is called. \n
             TLayerType description: \n
                | No. | TLayerType Definition | Value | explain |
                | :---:| :---:| :---:| :---:|
                | 1 | MV_FG_GEV_INTERFACE | 0x00000001 | GigE Vision interface |
                | 2 | MV_FG_CAMERALINK_INTERFACE | 0x00000004 | Camera Link interface |
                | 3 | MV_FG_CXP_INTERFACE | 0x00000008 | CoaXPress interface |
                | 4 | MV_FG_XoF_INTERFACE | 0x00000010 | XoFLink interface |
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_UpdateInterfaceList(IN unsigned int nTLayerType, OUT bool8_t *pbChanged);

/********************************************************************//**
 *  @~chinese
 *  @brief  释放指定类型的采集卡资源
 *  @param  nTLayerType             [IN]           采集卡类型
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 调用该接口之前必须关闭所有该类型的采集卡。通过FreeLibrary函数释放该库前需要调用该接口释放所有采集卡资源
             采集卡类型说明如下：\n
                | 序号 | 采集卡类型定义 | 值 | 说明 |
                | :---:| :---:| :---:| :---:|
                | 1 | MV_FG_GEV_INTERFACE | 0x00000001 | GigE Vision采集卡 |
                | 2 | MV_FG_CAMERALINK_INTERFACE | 0x00000004 | Camera Link采集卡 |
                | 3 | MV_FG_CXP_INTERFACE | 0x00000008 | CoaXPress采集卡 |
                | 4 | MV_FG_XoF_INTERFACE | 0x00000010 | XoFLink采集卡 |
  
 *  @~english
 *  @brief  Release frame grabber resources of specified types
 *  @param  nTLayerType             [IN]            Frame grabber type
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks All frame grabbers of the specified type should be disabled before calling this API. Before releasing the library libMVFGControl.so via function FreeLibrary, you need to call this API to release all frame grabber resources.
             TLayerType description: \n
                | No. | TLayerType Definition | Value | explain |
                | :---:| :---:| :---:| :---:|
                | 1 | MV_FG_GEV_INTERFACE | 0x00000001 | GigE Vision interface |
                | 2 | MV_FG_CAMERALINK_INTERFACE | 0x00000004 | Camera Link interface |
                | 3 | MV_FG_CXP_INTERFACE | 0x00000008 | CoaXPress interface |
                | 4 | MV_FG_XoF_INTERFACE | 0x00000010 | XoFLink interface |
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_ReleaseTLayerResource(IN unsigned int nTLayerType);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取采集卡数量
 *  @param  pnNumIfaces             [OUT]          采集卡数量
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Get the number of frame grabbers
 *  @param  pnNumIfaces             [OUT]           Number of frame grabbers.
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_GetNumInterfaces(OUT unsigned int *pnNumIfaces);

/********************************************************************//**
 *  @~chinese
 *  @brief  根据索引获取采集卡信息
 *  @param  nIndex                  [IN]           采集卡索引，从0开始
 *  @param  pstIfaceInfo            [OUT]          采集卡信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Get the frame grabber information via frame grabber index
 *  @param  nIndex                  [IN]            Frame grabber index, which starts from 0
 *  @param  pstIfaceInfo            [OUT]           Frame grabber information
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_GetInterfaceInfo(IN unsigned int nIndex, OUT MV_FG_INTERFACE_INFO *pstIfaceInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  打开采集卡
 *  @param  nIndex                  [IN]           采集卡索引，从0开始
 *  @param  phIface                 [OUT]          采集卡句柄
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Open the frame grabber
 *  @param  nIndex                  [IN]            Frame grabber index, which starts from 0
 *  @param  phIface                 [OUT]           Frame grabber handle
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_OpenInterface(IN unsigned int nIndex, OUT IFHANDLE* phIface);

/********************************************************************//**
 *  @~chinese
 *  @brief  打开采集卡，可指定权限
 *  @param  nIndex                  [IN]           采集卡索引，从0开始
 *  @param  nAccess                 [IN]           权限
 *  @param  phIface                 [OUT]          采集卡句柄
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks nAccess权限说明如下：\n。
                | 序号 | 权限类型定义 | 值 | 说明 |
                | :---:| :---:| :---:| :---:|
                | 1 | MV_FG_ACCESS_READONLY | 0x1 | 只读权限，仅可以获取节点值|
                | 2 | MV_FG_ACCESS_CONTROL | 0x2 | 控制权限
  
 *  @~english
 *  @brief  Open the frame grabber and specify its access mode
 *  @param  nIndex                  [IN]            Frame grabber index, which starts from 0
 *  @param  nAccess                 [IN]            Access mode
 *  @param  phIface                 [OUT]           Frame grabber handle
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks Access mode description: \n
                | No. | Access Definition | Value | explain |
                | :---:| :---:| :---:| :---:|
                | 1 | MV_FG_ACCESS_READONLY | 0x1 | Read only, Get node value only |
                | 2 | MV_FG_ACCESS_CONTROL | 0x2 | Control mode
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_OpenInterfaceEx(IN unsigned int nIndex, IN int nAccess, OUT IFHANDLE* phIface);

/********************************************************************//**
 *  @~chinese
 *  @brief  通过采集卡ID打开采集卡，可指定权限
 *  @param  pcInterfaceID           [IN]           采集卡ID
 *  @param  nAccess                 [IN]           权限
 *  @param  phIface                 [OUT]          采集卡句柄
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks nAccess权限说明如下：\n。
                | 序号 | 权限类型定义 | 值 | 说明 |
                | :---:| :---:| :---:| :---:|
                | 1 | MV_FG_ACCESS_READONLY | 0x1 | 只读权限 |
                | 2 | MV_FG_ACCESS_CONTROL | 0x2 | 控制权限
  
 *  @~english
 *  @brief  Open the frame grabber with its ID and specify its access mode
 *  @param  pcInterfaceID           [IN]            Frame grabber ID
 *  @param  nAccess                 [IN]            Access mode
 *  @param  phIface                 [OUT]           Frame grabber handle
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks Access mode description: \n
                | No. | Access Definition | Value | explain |
                | :---:| :---:| :---:| :---:|
                | 1 | MV_FG_ACCESS_READONLY | 0x1 | Read only, Get node value only |
                | 2 | MV_FG_ACCESS_CONTROL | 0x2 | Control mode
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_OpenInterfaceByID(IN char* pcInterfaceID, IN int nAccess, OUT IFHANDLE* phIface);

/********************************************************************//**
 *  @~chinese
 *  @brief  打开采集卡，导入配置文件（配置文件来自MV_FG_FeatureSave接口）
 *  @param  nIndex                  [IN]           采集卡索引，从0开始
 *  @param  pcConfigFile            [IN]           配置文件路径，用户需提供，为NULL时报错
 *  @param  phIface                 [OUT]          采集卡句柄
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 使用此接口需提供配置文件，采集卡将以控制权限打开。
  
 *  @~english
 *  @brief  Open the Interface with config file
 *  @param  nIndex                  [IN]            Interface index, starting from 0
 *  @param  pcConfigFile            [IN]            Config file path，users need to provide. An error was reported when it was NULL
 *  @param  phIface                 [OUT]           Interface handle
 *  @return Success, return MV_FG_SUCCESS. Failure, return error code 
 *  @remarks To use this interface, you need to provide a configuration file. The Interface will be opened with control permissions.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_OpenInterfaceWithConfig(IN unsigned int nIndex, IN char* pcConfigFile, OUT IFHANDLE* phIface);

/********************************************************************//**
 *  @~chinese
 *  @brief  关闭采集卡
 *  @param  hIface                  [IN]           采集卡句柄
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Close the frame grabber
 *  @param  hIface                  [IN]            Frame grabber handle
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_CloseInterface(IN IFHANDLE hIface);

/********************************************************************//**
 *  @~chinese
 *  @brief  更新指定采集卡下的相机列表
 *  @param  hIface                  [IN]           采集卡句柄
 *  @param  pbChanged               [OUT]          相机列表是否改变
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 只有在调用该接口时内部相机列表才会更新。
  
 *  @~english
 *  @brief  Update the device list of a specified frame grabber
 *  @param  hIface                  [IN]            Frame grabber handle
 *  @param  pbChanged               [OUT]           Whether the device list is updated
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks The internal device list is updated only when this API is called.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_UpdateDeviceList(IN IFHANDLE hIface, OUT bool8_t *pbChanged);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取相机数量
 *  @param  hIface                  [IN]           采集卡句柄
 *  @param  pnNumDevices            [OUT]          相机数量
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Get the number of devices of a specified frame grabber.
 *  @param  hIface                  [IN]            Frame grabber handle
 *  @param  pnNumDevices            [OUT]           Number of devices
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_GetNumDevices(IN IFHANDLE hIface, OUT unsigned int *pnNumDevices);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取相机信息
 *  @param  hIface                  [IN]           采集卡句柄
 *  @param  nIndex                  [IN]           相机索引，从0开始
 *  @param  pstDevInfo              [OUT]          相机信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Get the device information
 *  @param  hIface                  [IN]            Frame grabber handle
 *  @param  nIndex                  [IN]            Device index, which starts from 0
 *  @param  pstDevInfo              [OUT]           Device information
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_GetDeviceInfo(IN IFHANDLE hIface, IN unsigned int nIndex, OUT MV_FG_DEVICE_INFO *pstDevInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  打开相机
 *  @param  hIface                  [IN]           采集卡句柄
 *  @param  nIndex                  [IN]           相机索引，从0开始
 *  @param  phDevice                [OUT]          相机句柄
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Open the device according to index
 *  @param  hIface                  [IN]            Frame grabber handle
 *  @param  nIndex                  [IN]            Device index, which starts from 0
 *  @param  phDevice                [OUT]           Device handle
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_OpenDevice(IN IFHANDLE hIface, IN unsigned int nIndex, OUT DEVHANDLE* phDevice);

/********************************************************************//**
 *  @~chinese
 *  @brief  打开相机
 *  @param  hIface                  [IN]           采集卡句柄
 *  @param  pcDeviceID              [IN]           相机ID
 *  @param  phDevice                [OUT]          相机句柄
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Open the device according to ID
 *  @param  hIface                  [IN]            Frame grabber handle
 *  @param  pcDeviceID              [IN]            Device ID
 *  @param  phDevice                [OUT]           Device handle
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_OpenDeviceByID(IN IFHANDLE hIface, IN char* pcDeviceID, OUT DEVHANDLE* phDevice);

/********************************************************************//**
 *  @~chinese
 *  @brief  关闭相机
 *  @param  hDevice                 [IN]           相机句柄
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Close the device
 *  @param  hDevice                 [IN]            Device handle
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_CloseDevice(IN DEVHANDLE hDevice);


/***********************************************************************/
/*                             图像采集                                */
/*                        Image acquisition                            */
/***********************************************************************/
/********************************************************************//**
 *  @~chinese
 *  @brief  获取流通道数量
 *  @param  hDevice                 [IN]           相机句柄
 *  @param  pnNumStreams            [OUT]          流通道数量
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 目前一个相机只支持一个流通道。
  
 *  @~english
 *  @brief  Get the number of stream channels
 *  @param  hDevice                 [IN]            Device handle
 *  @param  pnNumStreams            [OUT]           Number of stream channels
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks Currently, a device only supports one stream channel.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_GetNumStreams(IN DEVHANDLE hDevice, OUT unsigned int *pnNumStreams);

/********************************************************************//**
 *  @~chinese
 *  @brief  打开流通道
 *  @param  hDevice                 [IN]           相机句柄
 *  @param  nIndex                  [IN]           流通道索引
 *  @param  phStream                [OUT]          流通道句柄
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Open the stream channel
 *  @param  hDevice                 [IN]            Device handle
 *  @param  nIndex                  [IN]            Stream channel index
 *  @param  phStream                [OUT]           Stream channel handle
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_OpenStream(IN DEVHANDLE hDevice, IN unsigned int nIndex, OUT STREAMHANDLE* phStream);

/********************************************************************//**
 *  @~chinese
 *  @brief  关闭流通道
 *  @param  hStream                 [IN]           流通道句柄
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Close the stream channel
 *  @param  hStream                 [IN]            Stream channel handle
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_CloseStream(IN STREAMHANDLE hStream);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置SDK内部缓存数量
 *  @param  hStream                 [IN]           流通道句柄
 *  @param  nBufferNum              [IN]           缓存数量
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 当未调用该接口或者nBufferNum参数设置为0时，表示SDK内部无缓存，此时需要用户申请缓存并向流通道注册缓存，否则无法开始取流。\n
             当nBufferNum参数设置大于0时，表示缓存由SDK内部进行申请，此时不允许用户向流通道注册缓存。\n
             需合理分配图像缓存数量。在Linux操作系统中，当分配的图像缓存数量过大，导致系统内存过低，应用程序会被操作系统杀死。
  
 *  @~english
 *  @brief  Set the number of internal buffers for the SDK
 *  @param  hStream                 [IN]            Stream channel handle
 *  @param  nBufferNum              [IN]            Number of buffers
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks The SDK has no internal buffer if this API is not called or the value of parameter nBufferNum is set to 0.When there is no internal buffer, you need to request and register buffers to stream channels before starting image acquisition. \n
             When the parameter nBufferNum is set to a value greater than 0, buffers will be registered internally by the SDK, and it is not allowed to register buffers to stream channels at this time. \n
             The number of image buffers should be reasonably allocated. For Linux operating systems, if the number of image buffers is too large, then the system memory will be too low and the application will be killed by the operating system.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_SetBufferNum(IN STREAMHANDLE hStream, IN unsigned int nBufferNum);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置取流策略
 *  @param  hStream                 [IN]           流通道句柄
 *  @param  enGrabStrategy          [IN]           取流策略
 *  @param  bUseTrashBuffer         [IN]           是否使用垃圾缓存, 0表示关闭，1表示开启
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 该接口定义了四种取流策略，用户可以根据实际需求进行选择。具体描述如下：
        -OneByOne:从旧到新一帧一帧的从输出缓存列表中获取图像，创建流通道后默认为该策略
        -LatestImagesOnly:仅从输出缓存列表中获取最新的一帧图像，同时清空输出缓存列表
        -LatestImages:从输出缓存列表中获取最新的图像，不清空输出缓存列表
        -UpcomingImage:在调用取图接口时清空输出缓存列表中所有图像，并等待相机即将生成的一帧图像。
        bUseTrashBuffer：当bUseTrashBuffer=true时, 如果注册的缓存全部被取走且没有调用MV_FG_ReleaseFrameBuffer\MV_FG_QueueBuffer\MV_FG_FlushQueue将缓存放回，
        开始使用内部垃圾缓存接收图像，接收到的图像会被丢弃，防止采集卡端丢图。

 *  @~english
 *  @brief  Set grab strategy
 *  @param  hStream                 [IN]            Stream handle
 *  @param  enGrabStrategy          [IN]            Grab strategy
 *  @param  bUseTrashBuffer         [IN]            Whether use trash buffer to receive buffers
 *  @return Success, return MV_FG_SUCCESS. Failure, return error code 
 *  @remarks This interface is set by four image acquisition approaches, the user may choose one as needed. Specific details are as followed: 
        -OneByOne:Obtain image from output cache list frame by frame in order, this function is default strategy when device is on.
        -LatestImagesOnly:Obtain the latest image from output cache list only, meanwhile clear output cache list.
        -LatestImages:Obtain the latest OutputQueueSize image from output cache list, the range of OutputQueueSize is 1-ImageNodeNum, the user may set the value of MV_CC_SetOutputQueueSizeinterface,the default value of ImageNodeNum is 1,If the user usesMV_CC_SetImageNodeNuminterface to set up OutputQueueSize,when the value of OutputQueueSize is set to be 1, the function will be same as LatestImagesOnly; if the value of OutputQueueSize is set to be ImageNodeNum, the function will be same as OneByOne.
        -UpcomingImage:Ignore all images in output cache list when calling get image interface, wait the next upcoming image generated. Support for GigE camera only, not support for U3V camera.
        bUseTrashBuffer: when bUseTrashBuffer is true, start use trash buffer to receive buffers while all announced buffers has been taken and not put back, images in trash buffer will be discarded to avaid frame loss on hardware layer. 
        ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_SetGrabStrategy(IN STREAMHANDLE hStream, IN MV_FG_GRAB_STRATEGY enGrabStrategy, bool8_t bUseTrashBuffer);

/********************************************************************//**
 *  @~chinese
 *  @brief  注册帧缓存信息回调函数(SDK内部申请缓存方式)
 *  @param  hStream                 [IN]           流通道句柄
 *  @param  cbFrame                 [IN]           帧缓存信息回调函数
 *  @param  pUser                   [IN]           用户自定义数据
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 在回调函数中处理耗时操作会阻塞后续帧信息获取。\n
             注册回调方式和MV_FG_GetFrameBuffer互斥。\n
             必须在调用MV_FG_StartAcquisition前注册回调函数。
  
 *  @~english
 *  @brief  Register the callback function for frame buffer information. This API is valid only when buffers are requested internally by the SDK.
 *  @param  hStream                 [IN]            Stream channel handle
 *  @param  cbFrame                 [IN]            Frame buffer information callback function
 *  @param  pUser                   [IN]            User-defined data
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks Time-consuming operations in the callback function will block the access to the subsequent frame buffer information. \n
             This API and MV_FG_GetFrameBuffer() are mutually exclusive. \n
             You need to call this API to register the callback function before calling API MV_FG_StartAcquisition() .
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_RegisterFrameCallBack(IN STREAMHANDLE hStream, IN MV_FG_FrameCallBack cbFrame, IN void* pUser);

/********************************************************************//**
 *  @~chinese
 *  @brief  注册帧缓存信息回调函数(SDK内部申请缓存方式)
 *  @param  hStream                 [IN]           流通道句柄
 *  @param  cbFrame                 [IN]           帧缓存信息回调函数
 *  @param  pUser                   [IN]           用户自定义数据
 *  @param  bBufferRelease          [IN]           缓存回收标记，1:回调函数结束后内存自动被回收;0:回调函数结束后，需要调用MV_FG_ReleaseFrameBuffer才能回收
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 在回调函数中处理耗时操作会阻塞后续帧信息获取。\n
             注册回调方式和MV_FG_GetFrameBuffer互斥。\n
             必须在调用MV_FG_StartAcquisition前注册回调函数。\n
             当bBufferRelease为0时，buffer不会自动释放，需要及时调用 MV_FG_ReleaseFrameBuffer 释放buffer。如果内部buffer用完，无法继续取流。
  
 *  @~english
 *  @brief  Register the callback function for frame buffer information. This API is valid only when buffers are requested internally by the SDK.
 *  @param  hStream                 [IN]            Stream channel handle
 *  @param  cbFrame                 [IN]            Frame buffer information callback function
 *  @param  pUser                   [IN]            User-defined data
 *  @param  bBufferRelease          [IN]            Method of releasing buffer. 1: The buffer will be released automatically after the callback function ends. 0: Call MV_FG_ReleaseFrameBuffer() to release the buffer after the callback function ends.
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks Time-consuming operations in the callback function will block the access to the subsequent frame buffer information. \n
             This API and MV_FG_GetFrameBuffer() are mutually exclusive. \n
             You need to call this API to register the callback function before calling API MV_FG_StartAcquisition() . \n
             When bBufferRelease is 0, the buffer will not be released automatically, and MV_FG_ReleaseFrameBuffer needs to be called in time to release the buffer. If the internal buffer is used up, the streaming cannot continue.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_RegisterFrameCallBackEx(IN STREAMHANDLE hStream, IN MV_FG_FrameCallBack cbFrame, IN void* pUser, IN bool8_t bBufferRelease);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取一帧图像的缓存信息(SDK内部申请缓存方式)
 *  @param  hStream                 [IN]           流通道句柄
 *  @param  pstBufferInfo           [OUT]          缓存信息
 *  @param  nTimeout                [IN]           超时时间，以毫秒为单位
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 需要调用MV_FG_ReleaseFrameBuffer接口进行缓存信息的释放。
  
 *  @~english
 *  @brief  Get the buffer information of a frame. This API is valid only when buffers are requested internally by the SDK.
 *  @param  hStream                 [IN]            Stream channel handle
 *  @param  pstBufferInfo           [OUT]           Buffer information
 *  @param  nTimeout                [IN]            Timeout, unit: millisecond
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks The API MV_FG_ReleaseFrameBuffer() should be called to release the buffer information.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_GetFrameBuffer(IN STREAMHANDLE hStream, OUT MV_FG_BUFFER_INFO* pstBufferInfo, IN unsigned int nTimeout);

/********************************************************************//**
 *  @~chinese
 *  @brief  释放缓存信息(SDK内部申请缓存方式，此接口用于释放不再使用的图像缓存，与MV_FG_GetFrameBuffer配套使用)
 *  @param  hStream                 [IN]           流通道句柄
 *  @param  pstBufferInfo           [IN]           缓存信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 该接口与MV_FG_GetFrameBuffer配套使用，使用MV_FG_GetFrameBuffer接口取到的图像数据pstBufferInfo，需要用MV_FG_ReleaseFrameBuffer接口进行权限释放。
  
 *  @~english
 *  @brief  Release the buffer information. This API is valid only when buffers are requested internally by the SDK. This API is used to release the image buffer which is no longer used.
 *  @param  hStream                 [IN]            Stream channel handle
 *  @param  pstBufferInfo           [IN]            Buffer information
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks This API and MV_FG_GetFrameBuffer() should be used in pair. The image data (pstBufferInfo) obtained via MV_FG_GetFrameBuffer() should be released by this API.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_ReleaseFrameBuffer(IN STREAMHANDLE hStream, IN MV_FG_BUFFER_INFO* pstBufferInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取缓存内的ChunkData信息
 *  @param  hStream                 [IN]           流通道句柄
 *  @param  pstBufferInfo           [IN]           缓存信息
 *  @param  nIndex                  [IN]           ChunkData索引
 *  @param  pstChunkDataInfo        [OUT]          ChunkData信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 在获取到缓存信息之后，调用MV_FG_ReleaseFrameBuffer或者MV_FG_QueueBuffer之前调用该接口才能得到有效信息。
  
 *  @~english
 *  @brief  Get the chunk data information of a buffer
 *  @param  hStream                 [IN]            Stream channel handle
 *  @param  pstBufferInfo           [IN]            Buffer information
 *  @param  nIndex                  [IN]            Chunk data index
 *  @param  pstChunkDataInfo        [OUT]           Chunk data information
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks After getting the buffer information, this API must be called before calling the APIs MV_FG_ReleaseFrameBuffer() and MV_FG_QueueBuffer() to get valid information of the chunk data.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_GetBufferChunkData(IN STREAMHANDLE hStream, IN MV_FG_BUFFER_INFO* pstBufferInfo, IN unsigned int nIndex, OUT MV_FG_CHUNK_DATA_INFO* pstChunkDataInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取流通道的图像大小
 *  @param  hStream                 [IN]           流通道句柄
 *  @param  pnPayloadSize           [OUT]          流通道的图像大小
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 修改相机图像相关参数如宽高、像素格式等参数后需要重新获取流通道的图像大小。
  
 *  @~english
 *  @brief  Get the image size of the stream channel
 *  @param  hStream                 [IN]            Stream channel handle
 *  @param  pnPayloadSize           [OUT]           Image size of the stream channel
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks The image size of the stream channel needs to be reacquired after the image-related parameters of the camera are changed, such as width, height, and pixel format.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_GetPayloadSize(IN STREAMHANDLE hStream, OUT unsigned int* pnPayloadSize);

/********************************************************************//**
 *  @~chinese
 *  @brief  向流通道中注册缓存(必须在MV_FG_StartAcquisition前注册缓存)
 *  @param  hStream                 [IN]           流通道句柄
 *  @param  pBuffer                 [IN]           图像缓存地址
 *  @param  nSize                   [IN]           图像缓存大小
 *  @param  pPrivate                [IN]           私有信息地址
 *  @param  phBuffer                [OUT]          缓存句柄
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 图像缓存大小通过MV_FG_GetPayloadSize接口获取；私有信息由用户自定义。
  
 *  @~english
 *  @brief  Register buffer to the stream channel. This API must be called prior to MV_FG_StartAcquisition.
 *  @param  hStream                 [IN]            Stream channel handle
 *  @param  pBuffer                 [IN]            Image buffer address
 *  @param  nSize                   [IN]            Image buffer size
 *  @param  pPrivate                [IN]            Private information address
 *  @param  phBuffer                [OUT]           Buffer handle
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks The image buffer size is acquired by calling the API MV_FG_GetPayloadSize(). The private information is user-defined.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_AnnounceBuffer(IN STREAMHANDLE hStream, IN void *pBuffer, IN unsigned int nSize, IN void *pPrivate, OUT BUFFERHANDLE *phBuffer); 

/********************************************************************//**
 *  @~chinese
 *  @brief  从流通道中销毁由MV_FG_AnnounceBuffer接口注册的缓存
 *  @param  hStream                 [IN]           流通道句柄
 *  @param  hBuffer                 [IN]           缓存句柄
 *  @param  pBuffer                 [OUT]          图像缓存地址
 *  @param  pPrivate                [OUT]          私有信息地址
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 只能撤销未使用队列中的缓存，可通过MV_FG_FlushQueue接口将缓存放到未使用队列。
  
 *  @~english
 *  @brief  Revoke buffer from the stream channel
 *  @param  hStream                 [IN]            Stream channel handle
 *  @param  hBuffer                 [IN]            Buffer handle
 *  @param  pBuffer                 [OUT]           Image buffer address
 *  @param  pPrivate                [OUT]           Private information address
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks Only buffers in unused queues can be revoked. You can allocate buffers to unused queues by calling the API MV_FG_FlushQueue().
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_RevokeBuffer(IN STREAMHANDLE hStream, IN BUFFERHANDLE hBuffer, OUT void **pBuffer, OUT void **pPrivate);

/********************************************************************//**
 *  @~chinese
 *  @brief  刷新缓存队列
 *  @param  hStream                 [IN]           流通道句柄
 *  @param  enQueueType             [IN]           队列类型
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 取流过程中不支持MV_FG_BUFFER_QUEUE_INPUT_TO_OUTPUT、MV_FG_BUFFER_QUEUE_ALL_DISCARD。
  
 *  @~english
 *  @brief  Refresh the buffer queue
 *  @param  hStream                 [IN]            Stream channel handle
 *  @param  enQueueType             [IN]            Buffer queue type
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks The following queue types are not supported during image acquisition: MV_FG_BUFFER_QUEUE_INPUT_TO_OUTPUT and MV_FG_BUFFER_QUEUE_ALL_DISCARD.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_FlushQueue(IN STREAMHANDLE hStream, IN MV_FG_BUFFER_QUEUE_TYPE enQueueType);

/********************************************************************//**
 *  @~chinese
 *  @brief  开始取流
 *  @param  hStream                 [IN]           流通道句柄
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Start image acquisition
 *  @param  hStream                 [IN]            Stream channel handle
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_StartAcquisition(IN STREAMHANDLE hStream);

/********************************************************************//**
 *  @~chinese
 *  @brief  停止取流
 *  @param  hStream                 [IN]           流通道句柄
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Stop image acquisition
 *  @param  hStream                 [IN]            Stream channel handle
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_StopAcquisition(IN STREAMHANDLE hStream);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取一帧图像的缓存信息(使用MV_FG_AnnounceBuffer注册缓存的取流方式)
 *  @param  hStream                 [IN]           流通道句柄
 *  @param  phBuffer                [OUT]          缓存句柄
 *  @param  nTimeout                [IN]           超时时间，以毫秒为单位
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 通过MV_FG_GetBufferInfo接口根据缓存句柄获取图像信息。
  
 *  @~english
 *  @brief  Get the buffer handle of a frame. This API is valid only when buffers are registered and managed externally.
 *  @param  hStream                 [IN]            Stream channel handle
 *  @param  phBuffer                [OUT]           Buffer handle
 *  @param  nTimeout                [IN]            Timeout, unit: millisecond
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks By calling the API MV_FG_GetBufferInfo(), you can get the buffer information with the acquired buffer handle.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_GetImageBuffer(IN STREAMHANDLE hStream, OUT BUFFERHANDLE *phBuffer, IN unsigned int nTimeout);

/********************************************************************//**
 *  @~chinese
 *  @brief  通过缓存句柄获取缓存信息
 *  @param  hBuffer                 [IN]           缓存句柄
 *  @param  pstBufferInfo           [OUT]          缓存信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Get the buffer information by buffer handle
 *  @param  hBuffer                 [IN]            Buffer handle
 *  @param  pstBufferInfo           [OUT]           Buffer information
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_GetBufferInfo(IN BUFFERHANDLE hBuffer, OUT MV_FG_BUFFER_INFO* pstBufferInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  将图像缓存放回输入队列，用于等待采集图像（图像缓存句柄来自MV_FG_GetImageBuffer接口）
 *  @param  hBuffer                 [IN]           缓存句柄
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Insert the buffer back to the input queue
 *  @param  hBuffer                 [IN]            Buffer handle
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_QueueBuffer(IN BUFFERHANDLE hBuffer);


/***********************************************************************/
/*                             图像处理                                */
/*                         Image processing                            */
/***********************************************************************/
/********************************************************************//**
 *  @~chinese
 *  @brief  显示一帧图像
 *  @param  hImage                  [IN]           图像句柄(可以使用采集卡/相机/流句柄)
 *  @param  hWnd                    [IN]           窗口句柄
 *  @param  pstDisplayFrameInfo     [IN]           显示的图像信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remark 在Linux平台进行远程显示时，需要OpenGL支持远程直接渲染模式，OpenGL版本高于4.0版本，并且显卡驱动支持高版本OpenGL。
  
 *  @~english
 *  @brief  Display one frame of image
 *  @param  hImage                  [IN]            Image handle. You can use the handle of frame grabbers, devices, or stream channels
 *  @param  hWnd                    [IN]            Window handle
 *  @param  pstDisplayFrameInfo     [IN]            Image information to be displayed
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remark When remote display is performed on the Linux platform, OpenGL must support the remote direct rendering mode, the OpenGL version need be later than 4.0, and the graphics card driver needs support a later version of OpenGL.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_DisplayOneFrame(IN IMAGEHANDLE hImage, IN void* hWnd, IN MV_FG_DISPLAY_FRAME_INFO *pstDisplayFrameInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  在图像上绘制矩形框辅助线(Linux暂不支持)
 *  @param  hImage                      [IN]            图像句柄(可以使用采集卡/相机/流句柄) 句柄需与显示接口句柄一致
 *  @param  pRectInfo                   [IN]            矩形辅助线的信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks  
 
 *  @~english
 *  @brief  Draw Rect Auxiliary Line(Linux is currently not supported)
 *  @param  handle                      [IN]            Image handle. You can use the handle of frame grabbers, devices, or stream channels. The handle should be consistent with the MV_FG_DisplayOneFrame interface.
 *  @param  pRectInfo                   [IN]            Rect Auxiliary Line Info
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks 
 ***********************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_DrawRect(IN IMAGEHANDLE hImage, IN MVFG_RECT_INFO* pRectInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  在图像上绘制圆形辅助线(Linux暂不支持)
 *  @param  hImage                      [IN]            图像句柄(可以使用采集卡/相机/流句柄) 句柄需与显示接口句柄一致
 *  @param  pCircleInfo                 [IN]            圆形辅助线的信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks  
 
 *  @~english
 *  @brief  Draw Circle Auxiliary Line(Linux is currently not supported)
 *  @param  handle                      [IN]            Image handle. You can use the handle of frame grabbers, devices, or stream channels. The handle should be consistent with the MV_FG_DisplayOneFrame interface.
 *  @param  pCircleInfo                 [IN]            Circle Auxiliary Line Info
 *  @return  Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks 
 ***********************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_DrawCircle(IN IMAGEHANDLE hImage, IN MVFG_CIRCLE_INFO* pCircleInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  在图像上绘制线条(Linux暂不支持)
 *  @param  hImage                      [IN]            图像句柄(可以使用采集卡/相机/流句柄) 句柄需与显示接口句柄一致
 *  @param  pLinesInfo                  [IN]            线条辅助线信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks  
 
 *  @~english
 *  @brief  Draw Line Auxiliary Line(Linux is currently not supported)
 *  @param  handle                      [IN]            Image handle. You can use the handle of frame grabbers, devices, or stream channels. The handle should be consistent with the MV_FG_DisplayOneFrame interface.
 *  @param  pLinesInfo                  [IN]            Linear Auxiliary Line Info
 *  @return  Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks 
 ***********************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_DrawLines(IN IMAGEHANDLE hImage, IN MVFG_LINES_INFO* pLinesInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  保存BMP图像
 *  @param  hImage                  [IN]           图像句柄(可以使用采集卡/相机/流句柄)
 *  @param  pstSaveBitmapInfo       [IN][OUT]      BMP图像信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Save the BMP image
 *  @param  hImage                  [IN]            Image handle. You can use the handle of frame grabbers, devices, or stream channels
 *  @param  pstSaveBitmapInfo       [IN][OUT]       BMP image information
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_SaveBitmap(IN IMAGEHANDLE hImage, IN OUT MV_FG_SAVE_BITMAP_INFO *pstSaveBitmapInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  保存JPEG图像
 *  @param  hImage                  [IN]           图像句柄(可以使用采集卡/相机/流句柄)
 *  @param  pstSaveJpegInfo         [IN][OUT]      JPEG图像信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Save the JPEG image
 *  @param  hImage                  [IN]            Image handle. You can use the handle of frame grabbers, devices, or stream channels
 *  @param  pstSaveJpegInfo         [IN][OUT]       JPEG image information
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_SaveJpeg(IN IMAGEHANDLE hImage, IN OUT MV_FG_SAVE_JPEG_INFO *pstSaveJpegInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  保存TIFF图像
 *  @param  hImage                  [IN]           图像句柄(可以使用采集卡/相机/流句柄)
 *  @param  pstSaveTiffInfo         [IN][OUT]      TIFF图像信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Save the TIFF image
 *  @param  hImage                  [IN]            Image handle. You can use the handle of frame grabbers, devices, or stream channels
 *  @param  pstSaveTiffInfo         [IN][OUT]       TIFF image information
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_SaveTiffToFile(IN IMAGEHANDLE hImage, IN OUT MV_FG_SAVE_TIFF_TO_FILE_INFO *pstSaveTiffInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  保存PNG图像
 *  @param  hImage                  [IN]           图像句柄(可以使用采集卡/相机/流句柄)
 *  @param  pstSavePngInfo          [IN][OUT]      PNG图像信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Save the PNG image
 *  @param  hImage                  [IN]            Image handle. You can use the handle of frame grabbers, devices, or stream channels
 *  @param  pstSavePngInfo          [IN][OUT]       PNG image information
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_SavePngToFile(IN IMAGEHANDLE hImage, IN OUT MV_FG_SAVE_PNG_TO_FILE_INFO *pstSavePngInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  像素格式转换
 *  @param  hImage                  [IN]           图像句柄(可以使用采集卡/相机/流句柄)
 *  @param  pstConvertPixelInfo     [IN][OUT]      像素格式转换信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码

 *  @~english
 *  @brief  Convert the pixel format
 *  @param  hImage                  [IN]            Image handle. You can use the handle of frame grabbers, devices, or stream channels
 *  @param  pstConvertPixelInfo     [IN][OUT]       Pixel format conversion information
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_ConvertPixelType(IN IMAGEHANDLE hImage, IN OUT MV_FG_CONVERT_PIXEL_INFO *pstConvertPixelInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  无损解码
 *  @param  hImage                      [IN]            图像句柄(可以使用采集卡/相机/流句柄)
 *  @param  pstDecodeParam              [IN][OUT]       无损解码参数结构体
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 将从相机中取到的无损压缩码流解码成裸数据，同时支持解析当前相机实时图像的水印信息（如果输入的无损码流不是当前相机或者不是实时取流的，则水印解析可能异常），
             必须传入当前相机的相机句柄才能解析出图像的水印信息，传入采集卡句柄或流句柄只能进行无损解码。
 
 *  @~english
 *  @brief  Lossless decoding
 *  @param  hImage                      [IN]            Image handle. You can use the handle of frame grabbers, devices, or stream channels
 *  @param  pstDecodeParam              [IN][OUT]       Lossless decoding parameter structure
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks Decode the lossless compressed stream data from the camera into raw data. \n
             The API can also parse the watermark information of the real-time image of the current camera. (If the input lossless stream data is not from the current camera, or is not real-time stream, then exception might occur during the watermark parsing.) \n
             The watermark information of the image can be parsed only when you input the handle of the current camera. The frame grabber handle or the stream channel handle can only be used for lossless decoding.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_HB_Decode(IN IMAGEHANDLE hImage, IN OUT MV_FG_HB_DECODE_PARAM* pstDecodeParam);

/********************************************************************//**
 *  @~chinese
 *  @brief  JPEG解码
 *  @param  hImage                      [IN]            图像句柄(可以使用采集卡/相机/流句柄)
 *  @param  pstDecodeParam              [IN][OUT]       JPEG解码参数结构体
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 将从相机中取到的JPEG流解码成裸数据。
 
 *  @~english
 *  @brief  JPEG decoding
 *  @param  hImage                      [IN]            Image handle. You can use the handle of frame grabbers, cameras, or stream channels
 *  @param  pstDecodeParam              [IN][OUT]       JPEG decoding parameter structure
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks Decode the JPEG stream from the camera into raw data.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_DecodeJpeg(IN IMAGEHANDLE hImage, IN OUT MV_FG_DECODE_JPEG_PARAM* pstDecodeParam);

/********************************************************************//**
 *  @~chinese
 *  @brief  重组图像
 *  @param  hImage                  [IN]           图像句柄(可以使用采集卡/相机/流句柄)
 *  @param  pstReconstructInfo      [IN][OUT]      重组信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 支持图像旋转，图像翻转，图像分割，但需搭配特定前置条件如：\n
             图像旋转和图像翻转支持的像素格式为MV_FG_PIXEL_TYPE_Mono8、MV_FG_PIXEL_TYPE_RGB8_Packed和MV_FG_PIXEL_TYPE_BGR8_Packed。\n
             图像分割支持任意像素格式，图像分割应与线阵相机的“MultiLightControl”节点搭配使用，该节点可设置多个不同的曝光值，如MultiLightControl=2，\n
             相机会将两个不同曝光值所对应的两张图像交叠合并为一张图像(实际高度为两张图像的高度)发送给上层应用程序，\n
             调用该接口并传入重组方式为RECONSTRUCT_MODE_SPLIT_BY_LINE_2，可将相机发送的一张图像分割为2张图像，这两张图像分别对应一个曝光值。\n
             若使用普通相机或未打开线阵相机的“MultiLightControl”节点，则图像分割无意义，只是将图像按行分割为2，3，4张图像，\n
             每张图像的高度变为原图像的1/2，1/3，1/4(由enReconstructMode决定)。
  
 *  @~english
 *  @brief  Reconstruct Image
 *  @param  hImage                  [IN]            Image handle. You can use the handle of frame grabbers, devices, or stream channels
 *  @param  pstReconstructInfo      [IN][OUT]       Reconstruct imag information
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks Image rotation, image flipping, and image splitting are supported, but with specific preconditions. \n
             The pixel format supported by image rotation and image flipping are MV_FG_PIXEL_TYPE_Mono8, MV_FG_PIXEL_TYPE_RGB8_Packet, and MV_FG_PIXEL_TYPE_BGR8_Packed. \n
             Image segmentation supports all pixel formats. Image segmentation should be used with the "MultiLightControl" node of the linear array camera. This node can set multiple different exposure values, such as MultiLightControl=2. \n
             The camera will overlap and merge the two images with two different exposure values into one image (its height is the sum of the two images) and send it to the upper application. \n
             Call this API and pass in RECONSTRUCT_MODE_SPLIT_BY_LINE_2 to divide the image sent by the camera into two images, each of which corresponds to an exposure value. \n
             If an ordinary camera is used or the "MultiLightControl" node of the linear array camera is not enabled, then the image segmentation is meaningless. \n
             The image will only be divided into 2, 3, and 4 images by line, and the height of each image becomes 1/2, 1/3, 1/4 of the original image (determined by enReconstructMode).
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_ReconstructImage(IN IMAGEHANDLE hImage, IN OUT MV_FG_RECONSTRUCT_INFO *pstReconstructInfo);


/***********************************************************************/
/*                             通用接口                                */
/*                            General API                              */
/***********************************************************************/
/********************************************************************//**
 *  @~chinese
 *  @brief  获取XML文件
 *  @param  hPort                   [IN]           参数句柄(可以使用采集卡/相机句柄)
 *  @param  pData                   [IN][OUT]      存放XML文件的缓存地址
 *  @param  nDataSize               [IN]           存放XML文件的缓存大小
 *  @param  pnDataLen               [OUT]          XML文件长度
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Get XML file of frame grabbers/devices
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  pData                   [IN][OUT]       Address of the buffer in which the XML file is stored
 *  @param  nDataSize               [IN]            Size of the buffer in which the XML file is stored
 *  @param  pnDataLen               [OUT]           Length of the XML file
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_GetXMLFile(IN PORTHANDLE hPort, IN OUT unsigned char* pData, IN unsigned int nDataSize, OUT unsigned int* pnDataLen);

/********************************************************************//**
 *  @~chinese
 *  @brief  获得节点的访问模式
 *  @param  hPort                   [IN]           参数句柄(可以使用采集卡/相机句柄)
 *  @param  strName                 [IN]           节点名称
 *  @param  penAccessMode           [OUT]          节点访问模式
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 获取EnumEntry类型节点的访问模式，strName应传入EnumEntry_NodeName_EnumEntryName。\n
             如获取PixelFormat枚举类型下的EnumEntry节点Mono8，strName格式为“EnumEntry_PixelFormat_Mono8”。
  
 *  @~english
 *  @brief  Get the access mode of a node
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  strName                 [IN]            Node name
 *  @param  penAccessMode           [OUT]           Node access mode
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks To get the access mode of the EnumEntry type node, the input value for strName should be in the format "EnumEntry_NodeName_EnumEntryName". \n
             For example, to get the access mode of the node MV_FG_PIXEL_TYPE_Mono8 of the enumeration MV_FG_PIXEL_TYPE, the strName should be "EnumEntry_PixelFormat_Mono8".
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_GetNodeAccessMode(IN PORTHANDLE hPort, IN const char * strName, OUT MV_FG_NODE_ACCESS_MODE *penAccessMode);

/********************************************************************//**
 *  @~chinese
 *  @brief  获得节点的类型
 *  @param  hPort                   [IN]           参数句柄(可以使用采集卡/相机句柄)
 *  @param  strName                 [IN]           节点名称
 *  @param  penInterfaceType        [OUT]          节点类型
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Get the type of a node
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  strName                 [IN]            Node name
 *  @param  penInterfaceType        [OUT]           Node type
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_GetNodeInterfaceType(IN PORTHANDLE hPort, IN const char * strName, OUT MV_FG_NODE_INTERFACE_TYPE *penInterfaceType);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取整型节点信息
 *  @param  hPort                   [IN]           参数句柄(可以使用采集卡/相机句柄)
 *  @param  strKey                  [IN]           节点名称
 *  @param  pstIntValue             [OUT]          整型节点信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Get the information about an integer type node
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  strKey                  [IN]            Node name
 *  @param  pstIntValue             [OUT]           Integer node information
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_GetIntValue(IN PORTHANDLE hPort, IN const char* strKey, OUT MV_FG_INTVALUE *pstIntValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置整型节点信息
 *  @param  hPort                   [IN]           参数句柄(可以使用采集卡/相机句柄)
 *  @param  strKey                  [IN]           节点名称
 *  @param  nValue                  [IN]           要设置的值
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Set the information of an integer type node
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  strKey                  [IN]            Node name
 *  @param  nValue                  [IN]            Value to be set
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_SetIntValue(IN PORTHANDLE hPort, IN const char* strKey, IN int64_t nValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取枚举类型节点的信息
 *  @param  hPort                   [IN]           参数句柄(可以使用采集卡/相机句柄)
 *  @param  strKey                  [IN]           节点名称
 *  @param  pstEnumValue            [OUT]          枚举类型节点信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Get the information about an enumerated type node
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  strKey                  [IN]            Node name
 *  @param  pstEnumValue            [OUT]           Information about the enumerated type node
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_GetEnumValue(IN PORTHANDLE hPort, IN const char* strKey, OUT MV_FG_ENUMVALUE *pstEnumValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置枚举类型节点的信息
 *  @param  hPort                   [IN]           参数句柄(可以使用采集卡/相机句柄)
 *  @param  strKey                  [IN]           节点名称
 *  @param  nValue                  [IN]           要设置的值
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Set the information about an enumerated type node
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  strKey                  [IN]            Node name
 *  @param  nValue                  [IN]            Value to be set
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_SetEnumValue(IN PORTHANDLE hPort, IN const char* strKey, IN unsigned int nValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  通过字符串设置枚举类型节点的信息
 *  @param  hPort                   [IN]           参数句柄(可以使用采集卡/相机句柄)
 *  @param  strKey                  [IN]           节点名称
 *  @param  strValue                [IN]           要设置的值
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Set the information of an enumeration type node by string
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  strKey                  [IN]            Node name
 *  @param  strValue                [IN]            Value to be set
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_SetEnumValueByString(IN PORTHANDLE hPort, IN const char* strKey, IN const char* strValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取单精度浮点型节点的信息
 *  @param  hPort                   [IN]           参数所属对象句柄
 *  @param  strKey                  [IN]           节点名称
 *  @param  pstFloatValue           [OUT]          单精度浮点型节点信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Get the information about a float type node
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  strKey                  [IN]            Node name
 *  @param  pstFloatValue           [OUT]           Information about the float type node
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_GetFloatValue(IN PORTHANDLE hPort, IN const char* strKey, OUT MV_FG_FLOATVALUE *pstFloatValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置单精度浮点型节点的信息
 *  @param  hPort                   [IN]           参数句柄(可以使用采集卡/相机句柄)
 *  @param  strKey                  [IN]           节点名称
 *  @param  fValue                  [IN]           要设置的值
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Set the information of a float type node
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  strKey                  [IN]            Node name
 *  @param  fValue                  [IN]            Value to be set
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_SetFloatValue(IN PORTHANDLE hPort, IN const char* strKey, IN float fValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取布尔型节点的信息
 *  @param  hPort                   [IN]           参数句柄(可以使用采集卡/相机句柄)
 *  @param  strKey                  [IN]           节点名称
 *  @param  pbValue                 [OUT]          布尔型节点信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Get the information about a boolean type node
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  strKey                  [IN]            Node name
 *  @param  pbValue                 [OUT]            Information about the boolean type node
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_GetBoolValue(IN PORTHANDLE hPort, IN const char* strKey, OUT bool8_t *pbValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置布尔类型节点的信息
 *  @param  hPort                   [IN]           参数句柄(可以使用采集卡/相机句柄)
 *  @param  strKey                  [IN]           节点名称
 *  @param  bValue                  [IN]           要设置的值
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Set the information of a boolean type node
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  strKey                  [IN]            Node name
 *  @param  bValue                  [IN]            Value to be set
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_SetBoolValue(IN PORTHANDLE hPort, IN const char* strKey, IN bool8_t bValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取字符串型节点的信息
 *  @param  hPort                   [IN]           参数句柄(可以使用采集卡/相机句柄)
 *  @param  strKey                  [IN]           节点名称
 *  @param  pstStringValue          [OUT]          字符串型节点信息
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Get the information about a string type node
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  strKey                  [IN]            Node name
 *  @param  pstStringValue          [OUT]           Information about the string type node
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_GetStringValue(IN PORTHANDLE hPort, IN const char* strKey, OUT MV_FG_STRINGVALUE *pstStringValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置字符串型节点的信息
 *  @param  hPort                   [IN]           参数句柄(可以使用采集卡/相机句柄)
 *  @param  strKey                  [IN]           节点名称
 *  @param  strValue                [IN]           要设置的值
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Set the information of a string type node
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  strKey                  [IN]            Node name
 *  @param  strValue                [IN]            Value to be set
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_SetStringValue(IN PORTHANDLE hPort, IN const char* strKey, IN const char* strValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  执行命令型节点的命令
 *  @param  hPort                   [IN]           参数句柄(可以使用采集卡/相机句柄)
 *  @param  strKey                  [IN]           节点名称
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
  
 *  @~english
 *  @brief  Execute the commands of a command type node
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  strKey                  [IN]            Node name
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_SetCommandValue(IN PORTHANDLE hPort, IN const char* strKey);

/********************************************************************//**
 *  @~chinese
 *  @brief  配置自定义的整型值
 *  @param  hPort                   [IN]           参数句柄(只能使用相机句柄)
 *  @param  enConfigCmd             [IN]           配置指令，例如波特率
 *  @param  nValue                  [IN]           要配置的值
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 仅支持CML相机。
  
 *  @~english
 *  @brief  Configure custom integer type value
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can only be a device
 *  @param  enConfigCmd             [IN]            Configuration command, such as baud rate
 *  @param  nValue                  [IN]            Values to be set
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks Only Camera Link cameras are supported
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_SetConfigIntValue(IN PORTHANDLE hPort, IN MV_FG_CONFIG_CMD enConfigCmd, IN int64_t nValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  保存设备属性
 *  @param  hPort                   [IN]            参数句柄(可以使用采集卡/相机句柄)
 *  @param  strFileName             [IN]            属性文件名，不支持中文路径
 *  @return 成功，返回MV_FG_SUCCESS；错误，返回错误码 
 *  @remarks 不支持CML相机, 建议用户使用.hcf 作为保存的属性文件后缀名 \n
 *           当传入相机句柄时会同时保存采集卡和相机属性。

 *  @~english
 *  @brief  Save device feature
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  strFileName             [IN]            Name of the file in which the device attributes and features will be saved (only supports English)
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks Camera Link cameras are not supported. It is recommended that users use .hcf as the suffix name for saved attribute files. \n
 *           When passed to the device handle, both the frame grabber and device properties are saved.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_FeatureSave(IN PORTHANDLE hPort, IN const char* strFileName);

/********************************************************************//**
 *  @~chinese
 *  @brief  导入设备属性
 *  @param  hPort                   [IN]            参数句柄(可以使用采集卡/相机句柄)
 *  @param  strFileName             [IN]            属性文件名，不支持中文路径
 *  @return 成功，返回MV_FG_SUCCESS；错误，返回错误码 
 *  @remarks 不支持CML相机，当传入相机句柄时会同时导入采集卡和相机属性。

 *  @~english
 *  @brief  Load device feature
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  strFileName             [IN]            Name of the file in which the device attributes and features will be saved (only supports English)
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks Camera Link cameras are not supported. \n
 *           When passed to the device handle, both the frame grabber and device properties are loaded.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_FeatureLoad(IN PORTHANDLE hPort, IN const char* strFileName);

/********************************************************************//**
 *  @~chinese
 *  @brief  从设备读取文件
 *  @param  hPort                       [IN]            参数句柄(可以使用采集卡/相机句柄)
 *  @param  pstFileAccess               [IN]            文件存取结构体
 *  @return 成功，返回MV_FG_SUCCESS；错误，返回错误码 
 *  @remarks 可能因为文件操作权限的问题导致读失败 \n
 *           卡/相机上的文件名来自UserSetSelector属性节点。
 
 *  @~english
 *  @brief  Read the file data from a device
 *  @param  hPort                       [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  pstFileAccess               [IN]            File access structure
 *  @return Success, return MV_FG_SUCCESS. Failure, return error code
 *  @remarks Reading might fail if you do not have the operation permission of the files. \n
 *           The file name on the interface/camera comes from the UserSetSelector attribute node.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_FileAccessRead(IN PORTHANDLE hPort, IN OUT MV_FG_FILE_ACCESS* pstFileAccess);

/********************************************************************//**
 *  @~chinese
 *  @brief  将文件写入设备
 *  @param  hPort                       [IN]            参数句柄(可以使用采集卡/相机句柄)
 *  @param  pstFileAccess               [IN]            文件存取结构体
 *  @return 成功，返回MV_FG_SUCCESS；错误，返回错误码
 *  @remarks 卡/相机上的文件名来自UserSetSelector属性节点。
 
 *  @~english
 *  @brief  Write the file to the device
 *  @param  hPort                       [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  pstFileAccess               [IN]            File access structure
 *  @return Success, return MV_FG_SUCCESS. Failure, return error code
 *  @remark The file name on the interface/camera comes from the UserSetSelector attribute node.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_FileAccessWrite(IN PORTHANDLE hPort, IN MV_FG_FILE_ACCESS* pstFileAccess);

/********************************************************************//**
 *  @~chinese
 *  @brief  清除GenICam节点缓存
 *  @param  hPort                       [IN]            参数句柄(可以使用采集卡/相机句柄)
 *  @return 成功，返回MV_FG_SUCCESS；错误，返回错误码
 
 *  @~english
 *  @brief  Clear the GenICam nodes buffer
 *  @param  hPort                       [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @return Success, return MV_FG_SUCCESS. Failure, return error code
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_InvalidateNodes(IN PORTHANDLE hPort);

/********************************************************************//**
 *  @~chinese
 *  @brief  读取采集卡/相机寄存器
 *  @param  hPort                   [IN]            句柄(可以使用采集卡/相机句柄)
 *  @param  pBuffer                 [IN]            作为返回值使用，保存读到的内存值（注意根据采集卡/相机类型区分大小端）
 *  @param  nAddress                [IN]            读取的内存地址，该地址可以从采集卡/相机的XML文件中获取，形如xxx_RegAddr的节点值
 *  @param  nLength                 [IN]            读取的内存长度
 *  @param  pnOutLength             [OUT]           实际返回的内存长度
 *  @return 成功，返回MV_FG_SUCCESS；错误，返回错误码 
 *  @remarks 访问采集卡/相机，读取某段寄存器的数据

 *  @~english
 *  @brief  Read frame grabber or device register
 *  @param  handle                      [IN]            Handle (frame grabber or device).
 *  @param  pBuffer                     [IN][OUT]       Used as a returned value to save the read-in memory value. (Memory value is stored in accordance with the big end and little end model.)
 *  @param  nAddress                    [IN]            Memory address to be read, which can be obtained from the xml file of the frame grabber or device. It is a node value in the format of xxx_RegAddr
 *  @param  nLength                     [IN]            Length of the memory to be read
 *  @param  pnOutLength                 [OUT]           Actual length of the returned memory
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks Access the frame grabber or device and read the data from a certain segment of register.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_ReadPort(IN PORTHANDLE hPort, void *pBuffer, int64_t nAddress, int64_t nLength, int64_t* pnOutLength);

/********************************************************************//**
 *  @~chinese
 *  @brief  写入采集卡/相机寄存器
 *  @param  hPort                   [IN]            句柄(可以使用采集卡/相机句柄)
 *  @param  pBuffer                 [IN]            待写入的内存值（注意根据采集卡/相机类型区分大小端）
 *  @param  nAddress                [IN]            待写入的内存地址，该地址可以从采集卡/相机的XML文件中获取，形如xxx_RegAddr的节点值
 *  @param  nLength                 [IN]            待写入的内存长度
 *  @param  pnOutLength             [OUT]           实际写入的内存长度
 *  @return 成功，返回MV_FG_SUCCESS；错误，返回错误码 
 *  @remarks 访问采集卡/相机，把一段数据写入某段寄存器

 *  @~english
 *  @brief  Write to frame grabber or device register
 *  @param  handle                      [IN]            Handle (frame grabber or device)
 *  @param  pBuffer                     [IN]            Memory value to be written. (Memory value is stored in accordance with the big end and little end model.)
 *  @param  nAddress                    [IN]            Memory address to be written, which can be obtained from the xml file of the frame grabber or device. It is a node value in the format of xxx_RegAddr
 *  @param  nLength                     [IN]            Length of the memory to be written
 *  @param  pnOutLength                 [OUT]           Actual length of the written memory
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks Access the frame grabber or device and write a piece of data into a certain segment of register.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_WritePort(IN PORTHANDLE hPort, const void *pBuffer, int64_t nAddress, int64_t nLength, int64_t* pnOutLength);

/********************************************************************//**
 *  @~chinese
 *  @brief  注册事件回调函数
 *  @param  hPort                   [IN]           参数句柄(可以使用采集卡/相机句柄)
 *  @param  strEventName            [IN]           事件名称
 *  @param  cbEvent                 [IN]           事件回调函数
 *  @param  pUser                   [IN]           用户自定义数据
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 在回调函数中处理耗时操作会阻塞后续事件信息的获取。\n
             该接口适用用采集卡和相机事件回调，通过传入不同句柄进行注册。
  
 *  @~english
 *  @brief  Register event callback function
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  strEventName            [IN]            Event name
 *  @param  cbEvent                 [IN]            Event callback function
 *  @param  pUser                   [IN]            User-defined data
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks Time-consuming operations in the callback function will block the access to the subsequent event information. \n
             This API is applicable to the callback of frame grabber, camera, and stream events. It registers callback functions by passing in different handles.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_RegisterEventCallBack(IN PORTHANDLE hPort, IN const char* strEventName, IN MV_FG_EventCallBack cbEvent, IN void* pUser);

/********************************************************************//**
 *  @~chinese
 *  @brief  注册异常信息回调函数
 *  @param  hPort                   [IN]           参数句柄(可以使用采集卡/相机句柄)
 *  @param  cbException             [IN]           异常信息回调函数
 *  @param  pUser                   [IN]           用户自定义数据
 *  @return 成功，返回MV_FG_SUCCESS；失败返回错误码
 *  @remarks 在回调函数中处理耗时操作会阻塞后续异常信息的获取。\n
             该接口适用用采集卡、相机异常回调，通过传入不同句柄进行注册。
             目前不支持采集卡的掉线事件，采集卡不支持热插拔，不会出现掉线的情况；采集卡升级后必须要重启PC才能使用，已给出友好提示。
  
 *  @~english
 *  @brief  Register Exception Information Callback Function
 *  @param  hPort                   [IN]            Handle of the object to which the parameter belongs; the object can be either a frame grabber or a device
 *  @param  cbException             [IN]            Exception information callback function
 *  @param  pUser                   [IN]            User-defined data
 *  @return Return MV_FG_SUCCESS on success, and return Error Code on failure.
 *  @remarks Time-consuming operations in the callback function will block the access to the subsequent exception information. \n
             This API is applicable to the callback of frame grabber and camera exceptions. It registers callback functions by passing in different handles. \n
             Currently, the disconnected event of the frame grabbers is not supported. The frame grabbers do not support hot swap, so disconnected events will not occur. \n
             After the frame grabber is upgraded, you need to restart the PC to use it. A friendly reminder has been given.
 ************************************************************************/
MV_FGCTRL_API int __stdcall MV_FG_RegisterExceptionCallBack(IN PORTHANDLE hPort, IN MV_FG_ExceptionCallBack cbException, IN void* pUser);

#ifdef __cplusplus
}
#endif 

#endif //_MV_FG_CTRL_H_
