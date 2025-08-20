#ifndef _MV_FG_DEFINES_H_
#define _MV_FG_DEFINES_H_

#  if defined(_WIN32)
#    if defined(_MSC_VER) && _MSC_VER >= 1600 /* VS2010 provides stdint.h */
#      include <stdint.h>
#    elif !defined _STDINT_H && !defined _STDINT
       /* stdint.h is usually not available under Windows */
       typedef unsigned char uint8_t;
       typedef __int32 int32_t;
       typedef __int64 int64_t;
       typedef unsigned __int32 uint32_t;
       typedef unsigned __int64 uint64_t;
#    endif
#  else
#    include <stdint.h>
#  endif

#ifdef __cplusplus
     typedef bool bool8_t;
#else
     typedef uint8_t bool8_t;
#endif
typedef unsigned int bool32_t;

#ifndef WIN32
   #ifndef __stdcall
        #define __stdcall
    #endif
#endif

#ifdef __cplusplus
extern "C" {
#endif 

/// \~chinese 句柄类型定义        \~english Handle type definition
typedef void*  IFHANDLE;                                            ///< \~chinese 采集卡句柄    \~english Interface handle
typedef void*  DEVHANDLE;                                           ///< \~chinese 相机句柄      \~english Device handle
typedef void*  STREAMHANDLE;                                        ///< \~chinese 取流句柄      \~english Stream handle
typedef void*  BUFFERHANDLE;                                        ///< \~chinese 缓存句柄      \~english Buffer handle
typedef void*  PORTHANDLE;                                          ///< \~chinese 参数句柄，可以使用采集卡/相机/流句柄  \~english Port handle, (Can use Interface/Device/Stream handle)
typedef void*  IMAGEHANDLE;                                         ///< \~chinese 图像句柄，可以使用采集卡/相机/流句柄  \~english Image handle, (Can use Interface/Device/Stream handle)

///< \~chinese 采集卡类型         \~english Interface type
#define MV_FG_GEV_INTERFACE                     0x00000001          ///< \~chinese GigE Vision采集卡        \~english GigE Vision interface
#define MV_FG_U3V_INTERFACE                     0x00000002          ///< \~chinese U3V采集卡                \~english U3V interface
#define MV_FG_CAMERALINK_INTERFACE              0x00000004          ///< \~chinese Camera Link采集卡        \~english Camera Link interface
#define MV_FG_CXP_INTERFACE                     0x00000008          ///< \~chinese CoaXPress采集卡          \~english CoaXPress interface
#define MV_FG_XoF_INTERFACE                     0x00000010          ///< \~chinese XoFLink采集卡            \~english XoFLink interface

///< \~chinese 卡信息的字符串最大长度      \~english Maximum string length of interface information
#define MV_FG_MAX_IF_INFO_SIZE                  64

///< \~chinese 采集卡权限         \~english Interface access mode
#define MV_FG_ACCESS_UNKNOWN                    0x0                 ///< \~chinese 权限未定义    \~english Access mode not defined
#define MV_FG_ACCESS_READONLY                   0x1                 ///< \~chinese 只读权限      \~english Read only mode
#define MV_FG_ACCESS_CONTROL                    0x2                 ///< \~chinese 控制权限      \~english Control mode

///< \~chinese CoaXPress采集卡信息         \~english CoaXPress interface information
typedef struct _MV_CXP_INTERFACE_INFO_
{
    unsigned char       chInterfaceID[MV_FG_MAX_IF_INFO_SIZE];      ///< \~chinese 采集卡ID    \~english Interface ID
    unsigned char       chDisplayName[MV_FG_MAX_IF_INFO_SIZE];      ///< \~chinese 显示名称    \~english Display name
    unsigned char       chSerialNumber[MV_FG_MAX_IF_INFO_SIZE];     ///< \~chinese 序列号      \~english Serial number

    // 低16位有效: bits(0~2)代表功能, bits(3~7)代表相机, bits(8-15)代表总线
    // The lower 16 bits are valid: bits (0~2) represents the function, bits (3~7) represents the device, and bits (8~15) represents the bus
    // |15  14  13  12  11  10  9  8 | 7  6  5  4  3 | 2  1  0 |
    // +-----------------------------+---------------+---------+
    // |             bus             |    device     |  func   |
    unsigned int        nPCIEInfo;                                  ///< \~chinese 采集卡的PCIE插槽信息  \~english PCIe slot information of interface

    unsigned char       chModelName[MV_FG_MAX_IF_INFO_SIZE];        ///< [OUT] \~chinese 型号       \~english model name
    unsigned char       chManufacturer[MV_FG_MAX_IF_INFO_SIZE];     ///< [OUT] \~chinese 厂商       \~english manufacturer name
    unsigned char       chDeviceVersion[MV_FG_MAX_IF_INFO_SIZE];    ///< [OUT] \~chinese 版本号     \~english device version
    unsigned char       chUserDefinedName[MV_FG_MAX_IF_INFO_SIZE];  ///< [OUT] \~chinese 自定义名称 \~english user defined name

}MV_CXP_INTERFACE_INFO;

///< \~chinese GigE Vision采集卡信息         \~english GigE Vision interface information
typedef struct _MV_GEV_INTERFACE_INFO_
{
    unsigned char       chInterfaceID[MV_FG_MAX_IF_INFO_SIZE];      ///< \~chinese 采集卡ID    \~english Interface ID
    unsigned char       chDisplayName[MV_FG_MAX_IF_INFO_SIZE];      ///< \~chinese 显示名称    \~english Display name
    unsigned char       chSerialNumber[MV_FG_MAX_IF_INFO_SIZE];     ///< \~chinese 序列号      \~english Serial number

    // 低16位有效: bits(0~2)代表功能, bits(3~7)代表相机, bits(8-15)代表总线
    // The lower 16 bits are valid: bits (0~2) represents the function, bits (3~7) represents the device, and bits (8~15) represents the bus
    // |15  14  13  12  11  10  9  8 | 7  6  5  4  3 | 2  1  0 |
    // +-----------------------------+---------------+---------+
    // |             bus             |    device     |  func   |
    unsigned int        nPCIEInfo;                                  ///< \~chinese 采集卡的PCIE插槽信息  \~english PCIe slot information of interface

    unsigned char       chModelName[MV_FG_MAX_IF_INFO_SIZE];        ///< [OUT] \~chinese 型号       \~english model name
    unsigned char       chManufacturer[MV_FG_MAX_IF_INFO_SIZE];     ///< [OUT] \~chinese 厂商       \~english manufacturer name
    unsigned char       chDeviceVersion[MV_FG_MAX_IF_INFO_SIZE];    ///< [OUT] \~chinese 版本号     \~english device version
    unsigned char       chUserDefinedName[MV_FG_MAX_IF_INFO_SIZE];  ///< [OUT] \~chinese 自定义名称 \~english user defined name

}MV_GEV_INTERFACE_INFO;

///< \~chinese Camera Link采集卡信息         \~english Camera Link interface information
typedef struct _MV_CML_INTERFACE_INFO_
{
    unsigned char       chInterfaceID[MV_FG_MAX_IF_INFO_SIZE];      ///< \~chinese 采集卡ID    \~english Interface ID
    unsigned char       chDisplayName[MV_FG_MAX_IF_INFO_SIZE];      ///< \~chinese 显示名称    \~english Display name
    unsigned char       chSerialNumber[MV_FG_MAX_IF_INFO_SIZE];     ///< \~chinese 序列号      \~english Serial number

    // 低16位有效: bits(0~2)代表功能, bits(3~7)代表相机, bits(8-15)代表总线
    // The lower 16 bits are valid: bits (0~2) represents the function, bits (3~7) represents the device, and bits (8~15) represents the bus
    // |15  14  13  12  11  10  9  8 | 7  6  5  4  3 | 2  1  0 |
    // +-----------------------------+---------------+---------+
    // |             bus             |    device     |  func   |
    unsigned int        nPCIEInfo;                                  ///< \~chinese 采集卡的PCIE插槽信息  \~english PCIe slot information of interface

    unsigned char       chModelName[MV_FG_MAX_IF_INFO_SIZE];        ///< [OUT] \~chinese 型号       \~english model name
    unsigned char       chManufacturer[MV_FG_MAX_IF_INFO_SIZE];     ///< [OUT] \~chinese 厂商       \~english manufacturer name
    unsigned char       chDeviceVersion[MV_FG_MAX_IF_INFO_SIZE];    ///< [OUT] \~chinese 版本号     \~english device version
    unsigned char       chUserDefinedName[MV_FG_MAX_IF_INFO_SIZE];  ///< [OUT] \~chinese 自定义名称 \~english user defined name

}MV_CML_INTERFACE_INFO;

///< \~chinese XoFLink采集卡信息         \~english XoFLink interface information
typedef struct _MV_XoF_INTERFACE_INFO_
{
    unsigned char       chInterfaceID[MV_FG_MAX_IF_INFO_SIZE];      ///< \~chinese 采集卡ID    \~english Interface ID
    unsigned char       chDisplayName[MV_FG_MAX_IF_INFO_SIZE];      ///< \~chinese 显示名称    \~english Display name
    unsigned char       chSerialNumber[MV_FG_MAX_IF_INFO_SIZE];     ///< \~chinese 序列号      \~english Serial number

    // 低16位有效: bits(0~2)代表功能, bits(3~7)代表相机, bits(8-15)代表总线
    // The lower 16 bits are valid: bits (0~2) represents the function, bits (3~7) represents the device, and bits (8~15) represents the bus
    // |15  14  13  12  11  10  9  8 | 7  6  5  4  3 | 2  1  0 |
    // +-----------------------------+---------------+---------+
    // |             bus             |    device     |  func   |
    unsigned int        nPCIEInfo;                                  ///< \~chinese 采集卡的PCIE插槽信息  \~english PCIe slot information of interface

    unsigned char       chModelName[MV_FG_MAX_IF_INFO_SIZE];        ///< [OUT] \~chinese 型号       \~english model name
    unsigned char       chManufacturer[MV_FG_MAX_IF_INFO_SIZE];     ///< [OUT] \~chinese 厂商       \~english manufacturer name
    unsigned char       chDeviceVersion[MV_FG_MAX_IF_INFO_SIZE];    ///< [OUT] \~chinese 版本号     \~english device version
    unsigned char       chUserDefinedName[MV_FG_MAX_IF_INFO_SIZE];  ///< [OUT] \~chinese 自定义名称 \~english user defined name

}MV_XoF_INTERFACE_INFO;

///< \~chinese 采集卡信息            \~english Interface information
typedef struct _MV_FG_INTERFACE_INFO_
{
    unsigned int        nTLayerType;                                ///< \~chinese 采集卡类型    \~english Interface type
    unsigned int        nReserved[4];                               ///< \~chinese 保留字段      \~english Reserved

    union
    {
        MV_CXP_INTERFACE_INFO   stCXPIfaceInfo;                     ///< \~chinese CoaXPress采集卡信息          \~english CoaXPress interface information
        MV_GEV_INTERFACE_INFO   stGEVIfaceInfo;                     ///< \~chinese GigE Vision采集卡信息        \~english GigE Vision interface information
        MV_CML_INTERFACE_INFO   stCMLIfaceInfo;                     ///< \~chinese Camera Link采集卡信息        \~english Camera Link interface information
        MV_XoF_INTERFACE_INFO   stXoFIfaceInfo;                     ///< \~chinese XoFLink采集卡信息            \~english XoFLink interface information

        unsigned int            nReserved[256];                     ///< \~chinese 保留字段     \~english Reserved
        // more ...
    }IfaceInfo;

}MV_FG_INTERFACE_INFO;

///< \~chinese 相机信息的最大长度        \~english Maximum length of device information
#define MV_FG_MAX_DEV_INFO_SIZE 64

///< \~chinese CoaXPress相机信息      \~english CoaXPress device information
typedef struct _MV_CXP_DEVICE_INFO_
{
    unsigned char       chVendorName[MV_FG_MAX_DEV_INFO_SIZE];      ///< \~chinese 供应商名字       \~english Vendor name
    unsigned char       chModelName[MV_FG_MAX_DEV_INFO_SIZE];       ///< \~chinese 型号名字         \~english Model name
    unsigned char       chManufacturerInfo[MV_FG_MAX_DEV_INFO_SIZE];///< \~chinese 厂商信息         \~english Manufacturer information
    unsigned char       chDeviceVersion[MV_FG_MAX_DEV_INFO_SIZE];   ///< \~chinese 相机版本         \~english Device version
    unsigned char       chSerialNumber[MV_FG_MAX_DEV_INFO_SIZE];    ///< \~chinese 序列号           \~english Serial number
    unsigned char       chUserDefinedName[MV_FG_MAX_DEV_INFO_SIZE]; ///< \~chinese 用户自定义名字   \~english User defined name
    unsigned char       chDeviceID[MV_FG_MAX_DEV_INFO_SIZE];        ///< \~chinese 相机ID            \~english Device ID

    unsigned int        nReserved[48];                              ///< \~chinese 保留字段      \~english Reserved

}MV_CXP_DEVICE_INFO;

#define MV_FG_GEV_IFCONFIG_LLA_BIT        0x00000004                ///< \~chinese GigE Vision相机LLA使能标志位    \~english LLA enable flag bit of GigE Vision device
#define MV_FG_GEV_IFCONFIG_DHCP_BIT       0x00000002                ///< \~chinese GigE Vision相机DHCP使能标志位   \~english DHCP enable flag bit of GigE Vision device
#define MV_FG_GEV_IFCONFIG_PERSISTENT_BIT 0x00000001                ///< \~chinese GigE Vision相机静态IP使能标志位        \~english Persistent IP enable flag bit of GigE Vision device
#define MV_FG_GEV_IFCONFIG_PR_BIT         0x80000000                ///< \~chinese GigE Vision相机能否处理PAUSE帧标志位   \~english Can the GigE Vision device handle the pause frame flag bit
#define MV_FG_GEV_IFCONFIG_PG_BIT         0x40000000                ///< \~chinese GigE Vision相机能否生成PAUSE帧标志位   \~english Can the GigE Vision device generate pause frame flag bit

///< \~chinese GigE Vision相机信息          \~english GigE Vision device information
typedef struct _MV_GEV_DEVICE_INFO_
{
    // 支持IP配置，1代表支持。bit0：静态IP, bit1：DHCP, bits2：LLA
    // Support IP configuration, and 1 represents support. Bit0: persistent IP, Bit1: DHCP, bits2: LLA
    // |31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 | 15  14  13  12  11  10  9  8 7  6  5  4  3 |   2       1         0     |
    // |-----------------------------------------------------------------------------------------------------+-------+-----------|
    // |PR PG                                           |                                            |  LLA  +  DHCP + PERSISTENT|
    unsigned int        nIpCfgOption;                               ///< \~chinese 支持的IP配置                       \~english Supported IP configurations
    unsigned int        nIpCfgCurrent;                              ///< \~chinese 当前IP配置，参考支持的IP配置说明   \~english For the current IP configuration, refer to the supported IP configuration description

    unsigned int        nCurrentIp;                                 ///< \~chinese 当前IP地址       \~english Current IP address
    unsigned int        nCurrentSubNetMask;                         ///< \~chinese 当前子网掩码     \~english Current subnet mask
    unsigned int        nDefultGateWay;                             ///< \~chinese 当前网关         \~english Current gateway
    unsigned int        nNetExport;                                 ///< \~chinese 网口IP地址       \~english Network port IP address
    uint64_t            nMacAddress;                                ///< \~chinese MAC地址          \~english MAC address

    unsigned char       chVendorName[MV_FG_MAX_DEV_INFO_SIZE];      ///< \~chinese 供应商名字       \~english Vendor name
    unsigned char       chModelName[MV_FG_MAX_DEV_INFO_SIZE];       ///< \~chinese 型号名字         \~english Model name
    unsigned char       chManufacturerInfo[MV_FG_MAX_DEV_INFO_SIZE];///< \~chinese 厂商信息         \~english Manufacturer information
    unsigned char       chDeviceVersion[MV_FG_MAX_DEV_INFO_SIZE];   ///< \~chinese 相机版本         \~english Device version
    unsigned char       chSerialNumber[MV_FG_MAX_DEV_INFO_SIZE];    ///< \~chinese 序列号           \~english Serial number
    unsigned char       chUserDefinedName[MV_FG_MAX_DEV_INFO_SIZE]; ///< \~chinese 用户自定义名字   \~english User defined name
    unsigned char       chDeviceID[MV_FG_MAX_DEV_INFO_SIZE];        ///< \~chinese 相机ID           \~english Device ID

    unsigned int        nCurrentPort;                               ///< \~chinese 当前端口         \~english Current Port

    unsigned int        nReserved[47];                              ///< \~chinese 保留字段         \~english Reserved

}MV_GEV_DEVICE_INFO;

///< \~chinese Camera Link相机信息          \~english Camera Link device information
typedef struct _MV_CML_DEVICE_INFO_
{
    unsigned char       chVendorName[MV_FG_MAX_DEV_INFO_SIZE];      ///< \~chinese 供应商名字       \~english Vendor name
    unsigned char       chModelName[MV_FG_MAX_DEV_INFO_SIZE];       ///< \~chinese 型号名字         \~english Model name
    unsigned char       chManufacturerInfo[MV_FG_MAX_DEV_INFO_SIZE];///< \~chinese 厂商信息         \~english Manufacturer information
    unsigned char       chDeviceVersion[MV_FG_MAX_DEV_INFO_SIZE];   ///< \~chinese 相机版本         \~english Device version
    unsigned char       chSerialNumber[MV_FG_MAX_DEV_INFO_SIZE];    ///< \~chinese 序列号           \~english Serial number
    unsigned char       chUserDefinedName[MV_FG_MAX_DEV_INFO_SIZE]; ///< \~chinese 用户自定义名字   \~english User defined name
    unsigned char       chDeviceID[MV_FG_MAX_DEV_INFO_SIZE];        ///< \~chinese 相机ID            \~english Device ID

    unsigned int        nReserved[48];                              ///< \~chinese 保留字段      \~english Reserved

}MV_CML_DEVICE_INFO;

///< \~chinese XoFLink相机信息      \~english XoFLink device information
typedef struct _MV_XoF_DEVICE_INFO_
{
    unsigned char       chVendorName[MV_FG_MAX_DEV_INFO_SIZE];      ///< \~chinese 供应商名字       \~english Vendor name
    unsigned char       chModelName[MV_FG_MAX_DEV_INFO_SIZE];       ///< \~chinese 型号名字         \~english Model name
    unsigned char       chManufacturerInfo[MV_FG_MAX_DEV_INFO_SIZE];///< \~chinese 厂商信息         \~english Manufacturer information
    unsigned char       chDeviceVersion[MV_FG_MAX_DEV_INFO_SIZE];   ///< \~chinese 相机版本         \~english Device version
    unsigned char       chSerialNumber[MV_FG_MAX_DEV_INFO_SIZE];    ///< \~chinese 序列号           \~english Serial number
    unsigned char       chUserDefinedName[MV_FG_MAX_DEV_INFO_SIZE]; ///< \~chinese 用户自定义名字   \~english User defined name
    unsigned char       chDeviceID[MV_FG_MAX_DEV_INFO_SIZE];        ///< \~chinese 相机ID            \~english Device ID

    unsigned int        nReserved[48];                              ///< \~chinese 保留字段      \~english Reserved

}MV_XoF_DEVICE_INFO;

///< \~chinese 相机类型         \~english Device type
#define MV_FG_GEV_DEVICE                     0x00000001             ///< \~chinese GigE Vision相机        \~english GigE Vision device
#define MV_FG_U3V_DEVICE                     0x00000002             ///< \~chinese U3V相机         \~english U3V device
#define MV_FG_CAMERALINK_DEVICE              0x00000003             ///< \~chinese Camera Link相机   \~english Camera Link device
#define MV_FG_CXP_DEVICE                     0x00000004             ///< \~chinese CoaXPress相机         \~english CoaXPress device
#define MV_FG_XoF_DEVICE                     0x00000005             ///< \~chinese XoFLink相机         \~english XoFLink device

///< \~chinese 相机信息         \~english Device information
typedef struct _MV_FG_DEVICE_INFO_
{
    unsigned int                nDevType;                           ///< \~chinese 相机类型     \~english Device type
    unsigned int                nReserved[3];                       ///< \~chinese 保留字段     \~english Reserved

    union
    {
        MV_CXP_DEVICE_INFO      stCXPDevInfo;                       ///< \~chinese CoaXPress相机信息          \~english CoaXPress device information
        MV_GEV_DEVICE_INFO      stGEVDevInfo;                       ///< \~chinese GigE Vision相机信息          \~english GigE Vision device information
        MV_CML_DEVICE_INFO      stCMLDevInfo;                       ///< \~chinese Camera Link相机信息    \~english Camera Link device information
        MV_XoF_DEVICE_INFO      stXoFDevInfo;                       ///< \~chinese XoFLink相机信息           \~english XoFLink device information

        unsigned int            nReserved[256];                     ///< \~chinese 保留字段     \~english Reserved
        // more ...
    }DevInfo;

}MV_FG_DEVICE_INFO;

///< \~chinese 缓存队列类型           \~english Buffer queue type
typedef enum _MV_FG_BUFFER_QUEUE_TYPE_
{
    MV_FG_BUFFER_QUEUE_INPUT_TO_OUTPUT          = 0,                ///< \~chinese 将输入队列的BUFFER放到输出队列   \~english Put BUFFER of input queue into output queue
    MV_FG_BUFFER_QUEUE_OUTPUT_DISCARD           = 1,                ///< \~chinese 放弃输出队列的BUFFER              \~english Discard BUFFER of output queue
    MV_FG_BUFFER_QUEUE_ALL_TO_INPUT             = 2,                ///< \~chinese 将所有的BUFFER(包括输出队列)放到输入队列  \~english Put all BUFFER (including output queue) into input queue
    MV_FG_BUFFER_QUEUE_UNQUEUED_TO_INPUT        = 3,                ///< \~chinese 将未使用的BUFFER放到输入队列     \~english Put unused BUFFER into input queue
    MV_FG_BUFFER_QUEUE_ALL_DISCARD              = 4                 ///< \~chinese 放弃输入和输出队列中的BUFFER     \~english Discard BUFFER in input and output queues

}MV_FG_BUFFER_QUEUE_TYPE;

///< \~chinese 像素格式定义           \~english Pixel format definition
#define MV_FG_PIXEL_MONO                        0x01000000          ///< \~chinese Mono格式        \~english Mono format
#define MV_FG_PIXEL_COLOR                       0x02000000          ///< \~chinese 彩色格式        \~english Color format
#define MV_FG_PIXEL_CUSTOM                      0x80000000          ///< \~chinese 自定义          \~english custom
#define MV_FG_PIXEL_BIT_COUNT(n)                ((n) << 16)         ///< \~chinese 位个数所在位置  \~english Location of the number of bits
typedef enum _MV_FG_PIXEL_TYPE_
{
    ///< \~chinese 未定义的格式       \~english Undefined format
    MV_FG_PIXEL_TYPE_Undefined                  = 0xFFFFFFFF,

    ///< \~chinese Mono格式           \~english Mono format
    MV_FG_PIXEL_TYPE_Mono8                      = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(8) | 0x0001),   ///< \~chinese Mono8            \~english Mono8
    MV_FG_PIXEL_TYPE_Mono10                     = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x0003),  ///< \~chinese Mono10           \~english Mono10
    MV_FG_PIXEL_TYPE_Mono10_Packed              = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x0004),  ///< \~chinese Mono10_Packed    \~english Mono10_Packed
    MV_FG_PIXEL_TYPE_Mono12                     = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x0005),  ///< \~chinese Mono12           \~english Mono12
    MV_FG_PIXEL_TYPE_Mono12_Packed              = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x0006),  ///< \~chinese Mono12_Packed    \~english Mono12_Packed
    MV_FG_PIXEL_TYPE_Mono16                     = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x0007),  ///< \~chinese Mono16           \~english Mono16

    ///< \~chinese Bayer格式          \~english Bayer format
    MV_FG_PIXEL_TYPE_BayerGR8                   = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(8) | 0x0008),   ///< \~chinese BayerGR8         \~english BayerGR8
    MV_FG_PIXEL_TYPE_BayerRG8                   = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(8) | 0x0009),   ///< \~chinese BayerRG8         \~english BayerRG8
    MV_FG_PIXEL_TYPE_BayerGB8                   = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(8) | 0x000A),   ///< \~chinese BayerGB8         \~english BayerGB8
    MV_FG_PIXEL_TYPE_BayerBG8                   = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(8) | 0x000B),   ///< \~chinese BayerBG8         \~english BayerBG8
    MV_FG_PIXEL_TYPE_BayerRBGG8                 = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(8) | 0x0046),   ///< \~chinese BayerRBGG8       \~english BayerRBGG8
    MV_FG_PIXEL_TYPE_BayerGR10                  = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x000C),  ///< \~chinese BayerGR10        \~english BayerGR10
    MV_FG_PIXEL_TYPE_BayerRG10                  = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x000D),  ///< \~chinese BayerRG10        \~english BayerRG10
    MV_FG_PIXEL_TYPE_BayerGB10                  = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x000E),  ///< \~chinese BayerGB10        \~english BayerGB10
    MV_FG_PIXEL_TYPE_BayerBG10                  = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x000F),  ///< \~chinese BayerBG10        \~english BayerBG10
    MV_FG_PIXEL_TYPE_BayerGR12                  = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x0010),  ///< \~chinese BayerGR12        \~english BayerGR12
    MV_FG_PIXEL_TYPE_BayerRG12                  = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x0011),  ///< \~chinese BayerRG12        \~english BayerRG12
    MV_FG_PIXEL_TYPE_BayerGB12                  = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x0012),  ///< \~chinese BayerGB12        \~english BayerGB12
    MV_FG_PIXEL_TYPE_BayerBG12                  = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x0013),  ///< \~chinese BayerBG12        \~english BayerBG12
    MV_FG_PIXEL_TYPE_BayerGR10_Packed           = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x0026),  ///< \~chinese BayerGR10_Packed \~english BayerGR10_Packed
    MV_FG_PIXEL_TYPE_BayerRG10_Packed           = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x0027),  ///< \~chinese BayerRG10_Packed \~english BayerRG10_Packed
    MV_FG_PIXEL_TYPE_BayerGB10_Packed           = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x0028),  ///< \~chinese BayerGB10_Packed \~english BayerGB10_Packed
    MV_FG_PIXEL_TYPE_BayerBG10_Packed           = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x0029),  ///< \~chinese BayerBG10_Packed \~english BayerBG10_Packed
    MV_FG_PIXEL_TYPE_BayerGR12_Packed           = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x002A),  ///< \~chinese BayerGR12_Packed \~english BayerGR12_Packed
    MV_FG_PIXEL_TYPE_BayerRG12_Packed           = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x002B),  ///< \~chinese BayerRG12_Packed \~english BayerRG12_Packed
    MV_FG_PIXEL_TYPE_BayerGB12_Packed           = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x002C),  ///< \~chinese BayerGB12_Packed \~english BayerGB12_Packed
    MV_FG_PIXEL_TYPE_BayerBG12_Packed           = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x002D),  ///< \~chinese BayerBG12_Packed \~english BayerBG12_Packed
    MV_FG_PIXEL_TYPE_BayerGR16                  = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x002E),  ///< \~chinese BayerGR16        \~english BayerGR16
    MV_FG_PIXEL_TYPE_BayerRG16                  = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x002F),  ///< \~chinese BayerRG16        \~english BayerRG16
    MV_FG_PIXEL_TYPE_BayerGB16                  = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x0030),  ///< \~chinese BayerGB16        \~english BayerGB16
    MV_FG_PIXEL_TYPE_BayerBG16                  = (MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x0031),  ///< \~chinese BayerBG16        \~english BayerBG16

    ///< \~chinese RGB格式            \~english RGB format
    MV_FG_PIXEL_TYPE_RGB8_Packed                = (MV_FG_PIXEL_COLOR | MV_FG_PIXEL_BIT_COUNT(24) | 0x0014),  ///< \~chinese RGB8_Packed     \~english RGB8_Packed
    MV_FG_PIXEL_TYPE_BGR8_Packed                = (MV_FG_PIXEL_COLOR | MV_FG_PIXEL_BIT_COUNT(24) | 0x0015),  ///< \~chinese BGR8_Packed     \~english BGR8_Packed
    MV_FG_PIXEL_TYPE_RGBA8_Packed               = (MV_FG_PIXEL_COLOR | MV_FG_PIXEL_BIT_COUNT(32) | 0x0016),  ///< \~chinese RGBA8_Packed    \~english RGBA8_Packed
    MV_FG_PIXEL_TYPE_BGRA8_Packed               = (MV_FG_PIXEL_COLOR | MV_FG_PIXEL_BIT_COUNT(32) | 0x0017),  ///< \~chinese BGRA8_Packed    \~english BGRA8_Packed
    MV_FG_PIXEL_TYPE_RGB16_Packed               = (MV_FG_PIXEL_COLOR | MV_FG_PIXEL_BIT_COUNT(48) | 0x0033),  ///< \~chinese RGB16_Packed    \~english RGB16_Packed

    ///< \~chinese YUV格式            \~english YUV format
    MV_FG_PIXEL_TYPE_YUV422_Packed              = (MV_FG_PIXEL_COLOR | MV_FG_PIXEL_BIT_COUNT(16) | 0x001F),  ///< \~chinese YUV422_Packed       \~english YUV422_Packed
    MV_FG_PIXEL_TYPE_YUV422_YUYV_Packed         = (MV_FG_PIXEL_COLOR | MV_FG_PIXEL_BIT_COUNT(16) | 0x0032),  ///< \~chinese YUV422_YUYV_Packed  \~english YUV422_YUYV_Packed

    ///< \~chinese Jpeg格式           \~english Jpeg format
    MV_FG_PIXEL_TYPE_JPEG                       = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_BIT_COUNT(24) | 0x0001), ///< \~chinese JPEG            \~english JPEG

    ///< \~chinese 无损压缩像素格式定义
    MV_FG_PIXEL_TYPE_HB_Mono8                   = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(8) | 0x0001),
    MV_FG_PIXEL_TYPE_HB_Mono10                  = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x0003),
    MV_FG_PIXEL_TYPE_HB_Mono10_Packed           = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x0004),
    MV_FG_PIXEL_TYPE_HB_Mono12                  = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x0005),
    MV_FG_PIXEL_TYPE_HB_Mono12_Packed           = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x0006),
    MV_FG_PIXEL_TYPE_HB_Mono16                  = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x0007),
    MV_FG_PIXEL_TYPE_HB_BayerGR8                = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(8) | 0x0008),
    MV_FG_PIXEL_TYPE_HB_BayerRG8                = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(8) | 0x0009),
    MV_FG_PIXEL_TYPE_HB_BayerGB8                = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(8) | 0x000A),
    MV_FG_PIXEL_TYPE_HB_BayerBG8                = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(8) | 0x000B),
    MV_FG_PIXEL_TYPE_HB_BayerRBGG8              = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(8) | 0x0046),
    MV_FG_PIXEL_TYPE_HB_BayerGR10               = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x000C),
    MV_FG_PIXEL_TYPE_HB_BayerRG10               = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x000D),
    MV_FG_PIXEL_TYPE_HB_BayerGB10               = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x000E),
    MV_FG_PIXEL_TYPE_HB_BayerBG10               = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x000F),
    MV_FG_PIXEL_TYPE_HB_BayerGR12               = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x0010),
    MV_FG_PIXEL_TYPE_HB_BayerRG12               = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x0011),
    MV_FG_PIXEL_TYPE_HB_BayerGB12               = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x0012),
    MV_FG_PIXEL_TYPE_HB_BayerBG12               = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(16) | 0x0013),
    MV_FG_PIXEL_TYPE_HB_BayerGR10_Packed        = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x0026),
    MV_FG_PIXEL_TYPE_HB_BayerRG10_Packed        = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x0027),
    MV_FG_PIXEL_TYPE_HB_BayerGB10_Packed        = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x0028),
    MV_FG_PIXEL_TYPE_HB_BayerBG10_Packed        = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x0029),
    MV_FG_PIXEL_TYPE_HB_BayerGR12_Packed        = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x002A),
    MV_FG_PIXEL_TYPE_HB_BayerRG12_Packed        = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x002B),
    MV_FG_PIXEL_TYPE_HB_BayerGB12_Packed        = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x002C),
    MV_FG_PIXEL_TYPE_HB_BayerBG12_Packed        = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_MONO | MV_FG_PIXEL_BIT_COUNT(12) | 0x002D),
    MV_FG_PIXEL_TYPE_HB_YUV422_Packed           = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_COLOR | MV_FG_PIXEL_BIT_COUNT(16) | 0x001F),
    MV_FG_PIXEL_TYPE_HB_YUV422_YUYV_Packed      = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_COLOR | MV_FG_PIXEL_BIT_COUNT(16) | 0x0032),
    MV_FG_PIXEL_TYPE_HB_RGB8_Packed             = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_COLOR | MV_FG_PIXEL_BIT_COUNT(24) | 0x0014),
    MV_FG_PIXEL_TYPE_HB_BGR8_Packed             = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_COLOR | MV_FG_PIXEL_BIT_COUNT(24) | 0x0015),
    MV_FG_PIXEL_TYPE_HB_RGBA8_Packed            = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_COLOR | MV_FG_PIXEL_BIT_COUNT(32) | 0x0016),
    MV_FG_PIXEL_TYPE_HB_BGRA8_Packed            = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_COLOR | MV_FG_PIXEL_BIT_COUNT(32) | 0x0017),
    MV_FG_PIXEL_TYPE_HB_RGB16_Packed            = (MV_FG_PIXEL_CUSTOM | MV_FG_PIXEL_COLOR | MV_FG_PIXEL_BIT_COUNT(48) | 0x0033),

}MV_FG_PIXEL_TYPE;

///< \~chinese 输出的帧缓存信息          \~english Output Frame buffer information
typedef struct _MV_FG_BUFFER_INFO_
{
    void*               pBuffer;                                    ///< [OUT] \~chinese 图像缓存地址       \~english Image buffer address
    unsigned int        nSize;                                      ///< [OUT] \~chinese 地址大小           \~english Address size
    unsigned int        nFilledSize;                                ///< [OUT] \~chinese 图像长度           \~english Image length
    void*               pPrivate;                                   ///< [OUT] \~chinese 私有数据           \~english Private data

    unsigned int        nWidth;                                     ///< [OUT] \~chinese 宽度               \~english Width
    unsigned int        nHeight;                                    ///< [OUT] \~chinese 高度               \~english Height
    MV_FG_PIXEL_TYPE    enPixelType;                                ///< [OUT] \~chinese 像素格式           \~english Pixel type

    bool                bNewData;                                   ///< [OUT] \~chinese 是否是新图像到来   \~english Is it a new image coming
    bool                bQueued;                                    ///< [OUT] \~chinese 是否在取图队列中   \~english Is it in the drawing queue
    bool                bAcquiring;                                 ///< [OUT] \~chinese 是否在取图         \~english Is it taking pictures
    bool                bIncomplete;                                ///< [OUT] \~chinese 是否未完成         \~english Is it incomplete

    int64_t             nFrameID;                                   ///< [OUT] \~chinese 帧号               \~english Frame ID
    int64_t             nDevTimeStamp;                              ///< [OUT] \~chinese 相机时间戳         \~english Device timestamp
    int64_t             nHostTimeStamp;                             ///< [OUT] \~chinese 主机时间戳         \~english Host timestamp

    unsigned int        nNumChunks;                                 ///< [OUT] \~chinese Chunk个数          \~english Number of Chunks
    unsigned int        nChunkPayloadSize;                          ///< [OUT] \~chinese Chunk负载长度      \~english Payload size of Chunk

    unsigned int        nSecondCount;                               ///< [OUT] \~chinese 秒数(时标)         \~english Seconds (time scale)
    unsigned int        nCycleCount;                                ///< [OUT] \~chinese 周期数(时标)       \~english Number of cycles (time scale)
    unsigned int        nCycleOffset;                               ///< [OUT] \~chinese 周期偏移量(时标)   \~english Cycle offset (time scale)

    float               fGain;                                      ///< [OUT] \~chinese 增益               \~english Gain
    float               fExposureTime;                              ///< [OUT] \~chinese 曝光时间           \~english Exposure time
    unsigned int        nAverageBrightness;                         ///< [OUT] \~chinese 平均亮度           \~english Average brightness
    unsigned int        nFrameCounter;                              ///< [OUT] \~chinese 总帧数             \~english Total frames
    unsigned int        nTriggerIndex;                              ///< [OUT] \~chinese 触发计数           \~english Trigger index

    unsigned int        nInput;                                     ///< [OUT] \~chinese 输入               \~english Input
    unsigned int        nOutput;                                    ///< [OUT] \~chinese 输出               \~english Output

    unsigned int        nRed;                                       ///< [OUT] \~chinese 红色(白平衡)       \~english Red (white balance)
    unsigned int        nGreen;                                     ///< [OUT] \~chinese 绿色(白平衡)       \~english Green (white balance)
    unsigned int        nBlue;                                      ///< [OUT] \~chinese 蓝色(白平衡)       \~english Blue (white balance)

    unsigned int        nOffsetX;                                   ///< [OUT] \~chinese 水平偏移量(ROI位置)   \~english Horizontal offset (ROI position)
    unsigned int        nOffsetY;                                   ///< [OUT] \~chinese 垂直偏移量(ROI位置)   \~english Vertical offset (ROI position)
    unsigned int        nChunkWidth;                                ///< [OUT] \~chinese 宽度(ROI位置)         \~english Width (ROI position)
    unsigned int        nChunkHeight;                               ///< [OUT] \~chinese 高度(ROI位置)         \~english Height (ROI position)

    unsigned int        nLastFrameFlag;                             ///< [OUT] \~chinese 电平结束最后一帧   \~english last level frame flag
    unsigned int        nReserved[44];                              ///<       \~chinese 保留字段           \~english Reserved

}MV_FG_BUFFER_INFO;

///< \~chinese Chunk信息          \~english Chunk information
typedef struct _MV_FG_CHUNK_DATA_INFO_
{
    unsigned char*      pChunkData;                                 ///< [OUT] \~chinese Chunk数据    \~english Chunk data
    unsigned int        nChunkID;                                   ///< [OUT] \~chinese Chunk ID     \~english Chunk ID
    unsigned int        nChunkLen;                                  ///< [OUT] \~chinese Chunk的长度  \~english Length of Chunk

    unsigned int        nReserved[4];                               ///<       \~chinese 保留字段     \~english Reserved

}MV_FG_CHUNK_DATA_INFO;

/// \~chinese 输入的图像信息           \~english Input image information
typedef struct _MV_FG_INPUT_IMAGE_INFO_
{
    unsigned int        nWidth;                                     ///< [IN]   \~chinese 图像宽       \~english Width
    unsigned int        nHeight;                                    ///< [IN]   \~chinese 图像高       \~english Height
    MV_FG_PIXEL_TYPE    enPixelType;                                ///< [IN]   \~chinese 像素格式     \~english Pixel type

    unsigned char*      pImageBuf;                                  ///< [IN]   \~chinese 输入图像缓存    \~english Input image buffer
    unsigned int        nImageBufLen;                               ///< [IN]   \~chinese 输入图像长度    \~english Input image length

    unsigned int        nReserved[4];                               ///<        \~chinese 预留字段      \~english Reserved

}MV_FG_INPUT_IMAGE_INFO, MV_FG_DISPLAY_FRAME_INFO;

/// \~chinese 输出的图像信息           \~english Output image information
typedef struct _MV_FG_OUTPUT_IMAGE_INFO_
{
    unsigned int        nWidth;                                     ///< [OUT]  \~chinese 图像宽       \~english Width
    unsigned int        nHeight;                                    ///< [OUT]  \~chinese 图像高       \~english Height
    MV_FG_PIXEL_TYPE    enPixelType;                                ///< [OUT]   \~chinese 像素格式     \~english Pixel type

    unsigned char*      pImageBuf;                                  ///< [IN][OUT]  \~chinese 数据缓存  \~english Data buffer
    unsigned int        nImageBufSize;                              ///< [IN]   \~chinese 数据缓冲大小  \~english Data buffer size
    unsigned int        nImageBufLen;                               ///< [OUT]  \~chinese 图像长度      \~english Image length

    unsigned int        nReserved[4];                               ///<        \~chinese 预留字段      \~english Reserved
}MV_FG_OUTPUT_IMAGE_INFO;

/// \~chinese 插值方法          \~english CFA method
typedef enum _MV_FG_CFA_METHOD_
{
    MV_FG_CFA_METHOD_QUICK              = 0,                        ///< \~chinese 快速       \~english Quick
    MV_FG_CFA_METHOD_BALANCE            = 1,                        ///< \~chinese 均衡       \~english Balance
    MV_FG_CFA_METHOD_OPTIMAL            = 2,                        ///< \~chinese 最优       \~english Optimal

}MV_FG_CFA_METHOD;

/// \~chinese 保存BMP图像信息         \~english Save BMP image information
typedef struct _MV_FG_SAVE_BITMAP_INFO_
{
    MV_FG_INPUT_IMAGE_INFO  stInputImageInfo;                       ///< [IN]  \~chinese 输入图片信息     \~english Input image information

    unsigned char*      pBmpBuf;                                    ///< [IN][OUT]  \~chinese 输出的BMP图片缓存    \~english Output BMP image buffer
    unsigned int        nBmpBufSize;                                ///< [IN]  \~chinese 输出的缓冲区大小          \~english Output buffer size
    unsigned int        nBmpBufLen;                                 ///< [OUT] \~chinese 输出的BMP图片长度         \~english Output BMP picture length
    
    MV_FG_CFA_METHOD    enCfaMethod;                                ///< [IN]  \~chinese 插值方法       \~english CFA method

    unsigned int        nReserved[4];                               ///<       \~chinese 预留字段       \~english Reserved

}MV_FG_SAVE_BITMAP_INFO;

/// \~chinese 保存JPEG图像信息            \~english Save JPEG image information
typedef struct _MV_FG_SAVE_JPEG_INFO_
{
    MV_FG_INPUT_IMAGE_INFO  stInputImageInfo;                       ///< [IN]  \~chinese 输入图片信息     \~english Input image information

    unsigned char*      pJpgBuf;                                    ///< [IN][OUT]  \~chinese 输出的JPEG图片缓存   \~english Output JPEG image buffer
    unsigned int        nJpgBufSize;                                ///< [IN]  \~chinese 输出的缓冲区大小          \~english Output buffer size
    unsigned int        nJpgBufLen;                                 ///< [OUT] \~chinese 输出的JPEG图片长度        \~english Output JPEG picture length

    unsigned int        nJpgQuality;                                ///< [IN]  \~chinese 编码质量, (0-100]      \~english Encoding quality, (0-100]
    MV_FG_CFA_METHOD    enCfaMethod;                                ///< [IN]  \~chinese 插值方法               \~english CFA method

    unsigned int        nReserved[4];                               ///<       \~chinese 预留字段       \~english Reserved

}MV_FG_SAVE_JPEG_INFO;

typedef enum _MV_FG_RESOLUTION_UNIT_
{
    MV_FG_Resolution_Unit_None = 1,           // 无单位
    MV_FG_Resolution_Unit_Inch = 2,           // 英寸
    MV_FG_Resolution_Unit_CENTIMETER = 3,     // 厘米

}MV_FG_RESOLUTION_UNIT;

/// \~chinese 保存Tiff图像信息            \~english Save Tiff image information
typedef struct _MV_FG_SAVE_TIFF_TO_FILE_INFO_
{
    MV_FG_INPUT_IMAGE_INFO  stInputImageInfo;                       ///< [IN]  \~chinese 输入图片信息     \~english Input image information

    char*               pcImagePath;                                ///< [IN]  \~chinese 输入文件路径     \~english Input image path
    float               fXResolution;                               ///< [IN]  \~chinese 水平分辨率       \~english Horizontal resolution
    float               fYResolution;                               ///< [IN]  \~chinese 垂直分辨率       \~english Vertical resolution
    MV_FG_RESOLUTION_UNIT  enResolutionUnit;                        ///< [IN]  \~chinese 分辨率单位       \~english Resolution unit
    MV_FG_CFA_METHOD    enCfaMethod;                                ///< [IN]  \~chinese 插值方法         \~english CFA method

    unsigned int        nReserved[4];                               ///<       \~chinese 预留字段         \~english Reserved

}MV_FG_SAVE_TIFF_TO_FILE_INFO;

/// \~chinese 保存Png图像信息            \~english Save Png image information
typedef struct _MV_FG_SAVE_PNG_TO_FILE_INFO_
{
    MV_FG_INPUT_IMAGE_INFO  stInputImageInfo;                       ///< [IN]  \~chinese 输入图片信息           \~english Input image information

    char*               pcImagePath;                                ///< [IN]  \~chinese 输入文件路径           \~english Input image path
    unsigned int        nPngCompression;                            ///< [IN]  \~chinese 编码压缩率, [0-9]。 目前均采用无损压缩，不支持压缩率      \~english Encoding compression, [0-9]. Currently, lossless compression is used and compression rate is not supported.
    MV_FG_CFA_METHOD    enCfaMethod;                                ///< [IN]  \~chinese 插值方法               \~english CFA method

    unsigned int        nReserved[4];                               ///<       \~chinese 预留字段               \~english Reserved

}MV_FG_SAVE_PNG_TO_FILE_INFO;

/// \~chinese Gamma类型           \~english Gamma type
typedef enum _MV_FG_GAMMA_TYPE_
{
    MV_FG_GAMMA_TYPE_NONE               = 0,                        ///< \~chinese 不启用                       \~english Disable
    MV_FG_GAMMA_TYPE_VALUE              = 1,                        ///< \~chinese Gamma值                      \~english Gamma value
    MV_FG_GAMMA_TYPE_USER_CURVE         = 2,                        ///< \~chinese Gamma曲线                    \~english Gamma curve
                                                                    ///< \~chinese 输出图像数据为8位时          \~english Output image data is 8 bit
                                                                    ///< \~chinese      曲线长度：256*sizeof(unsigned char)      \~english 8bit,length:256*sizeof(unsigned char)
                                                                    ///< \~chinese 输出图像数据为16位时，曲线数据根据输入图像有所不同      \~english Output image data is 16 bit
                                                                    ///< \~chinese      源图像格式10位, 曲线长度：1024*sizeof(unsigned short)   \~english Input image 10bit, curve length:1024*sizeof(unsigned short)
                                                                    ///< \~chinese      源图像格式12位, 曲线长度：4096*sizeof(unsigned short)   \~english Input image 12bit, curve length:4096*sizeof(unsigned short)
                                                                    ///< \~chinese      源图像格式16位, 曲线长度：65536*sizeof(unsigned short)  \~english Input image 16bit, curve length:65536*sizeof(unsigned short)
    MV_FG_GAMMA_TYPE_LRGB2SRGB          = 3,                        ///< \~chinese linear RGB to sRGB           \~english linear RGB to sRGB
    MV_FG_GAMMA_TYPE_SRGB2LRGB          = 4,                        ///< \~chinese sRGB to linear RGB(仅色彩插值时支持，色彩校正时无效) \~english sRGB to linear RGB

}MV_FG_GAMMA_TYPE;

/// \~chinese Gamma信息, 设置Gamma曲线矫正时，需输入有效的矫正曲线   \~english Gamma information, setting gamma curve correction, input a valid correction curve
typedef struct _MV_FG_GAMMA_INFO_
{
    MV_FG_GAMMA_TYPE    enGammaType;                                ///< [IN]  \~chinese Gamma类型              \~english Gamma type

    float               fGammaValue;                                ///< [IN]  \~chinese Gamma值[0.1,4.0]       \~english Gamma value[0.1,4.0]

    unsigned char*      pGammaCurveBuf;                             ///< [IN]  \~chinese Gamma曲线缓存          \~english Gamma curve buffer
    unsigned int        nGammaCurveBufLen;                          ///< [IN]  \~chinese Gamma曲线长度          \~english Gamma curve buffer size

    unsigned int        nReserved[4];                               ///<       \~chinese 预留字段       \~english Reserved

}MV_FG_GAMMA_INFO;

/// \~chinese CCM信息         \~english CCM information
typedef struct _MV_FG_CCM_INFO_
{
    bool32_t            bCCMEnable;                                 ///< [IN]  \~chinese 是否启用CCM            \~english CCM enable

    int                 nCCMat[9];                                  ///< [IN]  \~chinese CCM矩阵(-65536~65536)  \~english Color correction matrix(-65536~65536)
    unsigned int        nCCMScale;                                  ///< [IN]  \~chinese 量化系数（2的整数幂,最大65536）    \~english Quantitative scale(Integer power of 2, <= 65536)

    unsigned int        nReserved[4];                               ///<       \~chinese 预留字段       \~english Reserved

}MV_FG_CCM_INFO;

/// \~chinese 像素格式转换信息          \~english Pixel format conversion information
typedef struct _MV_FG_CONVERT_PIXEL_INFO_
{
    MV_FG_INPUT_IMAGE_INFO  stInputImageInfo;                       ///< [IN]  \~chinese 输入图片信息     \~english Input image information

    MV_FG_OUTPUT_IMAGE_INFO stOutputImageInfo;                      ///< [IN][OUT]  \~chinese 输出图片信息    \~english Output image information

    // 以下参数在Bayer插值中起效
    // The following parameters work in Bayer CFA
    MV_FG_CFA_METHOD    enCfaMethod;                                ///< [IN]  \~chinese 插值方法   \~english CFA method
    bool32_t            bFilterEnable;                              ///< [IN]  \~chinese 平滑使能   \~english Filter enable
    MV_FG_GAMMA_INFO    stGammaInfo;                                ///< [IN]  \~chinese Gamma信息  \~english Gamma information
    MV_FG_CCM_INFO      stCCMInfo;                                  ///< [IN]  \~chinese CCM信息,仅windows支持    \~english CCM information，support only windows

    unsigned int        nReserved[4];                               ///<       \~chinese 预留字段   \~english Reserved

}MV_FG_CONVERT_PIXEL_INFO;

/// \~chinese 水印信息                  \~english  Frame-specific information
typedef struct _MV_FG_FRAME_SPEC_INFO_
{
    /// \~chinese 相机水印时标      \~english Device frame-specific time scale
    unsigned int        nSecondCount;                               ///< [OUT] \~chinese 秒数                   \~english The Seconds
    unsigned int        nCycleCount;                                ///< [OUT] \~chinese 周期数                 \~english The Count of Cycle
    unsigned int        nCycleOffset;                               ///< [OUT] \~chinese 周期偏移量             \~english The Offset of Cycle

    float               fGain;                                      ///< [OUT] \~chinese 增益                   \~english Gain
    float               fExposureTime;                              ///< [OUT] \~chinese 曝光时间               \~english Exposure Time
    unsigned int        nAverageBrightness;                         ///< [OUT] \~chinese 平均亮度               \~english Average brightness

    /// \~chinese 白平衡相关        \~english White balance
    unsigned int        nRed;                                       ///< [OUT] \~chinese 红色                   \~english Red
    unsigned int        nGreen;                                     ///< [OUT] \~chinese 绿色                   \~english Green
    unsigned int        nBlue;                                      ///< [OUT] \~chinese 蓝色                   \~english Blue

    unsigned int        nFrameCounter;                              ///< [OUT] \~chinese 总帧数                 \~english Frame Counter
    unsigned int        nTriggerIndex;                              ///< [OUT] \~chinese 触发计数               \~english Trigger Counting

    unsigned int        nInput;                                     ///< [OUT] \~chinese 输入                   \~english Input
    unsigned int        nOutput;                                    ///< [OUT] \~chinese 输出                   \~english Output

    /// \~chinese ROI区域           \~english ROI Region
    unsigned short      nOffsetX;                                   ///< [OUT] \~chinese 水平偏移量             \~english OffsetX
    unsigned short      nOffsetY;                                   ///< [OUT] \~chinese 垂直偏移量             \~english OffsetY
    unsigned short      nFrameWidth;                                ///< [OUT] \~chinese 水印宽                 \~english The Width of Chunk
    unsigned short      nFrameHeight;                               ///< [OUT] \~chinese 水印高                 \~english The Height of Chunk

    unsigned int        nReserved[16];                              ///<       \~chinese 预留                   \~english Reserved

}MV_FG_FRAME_SPEC_INFO;

/// \~chinese 无损解码参数              \~english High Bandwidth decode structure
typedef struct _MV_FG_HB_DECODE_PARAM_T_
{
    unsigned char*          pSrcBuf;                                ///< [IN]  \~chinese 输入数据缓存           \~english Input data buffer
    unsigned int            nSrcLen;                                ///< [IN]  \~chinese 输入数据大小           \~english Input data size

    MV_FG_OUTPUT_IMAGE_INFO stOutputImageInfo;                      ///< [IN][OUT]  \~chinese 输出图片信息       \~english Output image information

    MV_FG_FRAME_SPEC_INFO   stFrameSpecInfo;                        ///< [OUT] \~chinese 水印信息（暂不支持）   \~english Frame Spec Info(Not support)

    unsigned int            nRes[8];                                ///<       \~chinese 预留                   \~english Reserved

}MV_FG_HB_DECODE_PARAM;

/// \~ chinese JPEG解码参数             \~english JPEG decoding structure
typedef struct _MV_FG_DECODE_JPEG_PARAM_T_
{
    unsigned char*          pSrcBuf;                                ///< [IN]  \~chinese 输入数据缓存           \~english Input data buffer
    unsigned int            nSrcLen;                                ///< [IN]  \~chinese 输入数据大小           \~english Input data size

    MV_FG_OUTPUT_IMAGE_INFO stOutputImageInfo;                      ///< [IN][OUT]  \~chinese 输出图片信息       \~english Output image information

    unsigned int            nRes[8];                                ///<       \~chinese 预留                   \~english Reserved


}MV_FG_DECODE_JPEG_PARAM;

/// \~chinese 图像重组方式          \~english Image reconstruct mode
#define MV_FG_MAX_SPLIT_NUM                     8                   ///< \~chinese 最多将源图像拆分的个数  \~english The maximum number of source images to split
#define MV_FG_ROTATE_MODE                       0x1000              ///< \~chinese 旋转方式       \~english Rotation mode
#define MV_FG_FLIP_MODE                         0x2000              ///< \~chinese 翻转方式       \~english Flip mode
#define MV_FG_SPLIT_BY_LINE_MODE                0x3000              ///< \~chinese 按行拆分方式   \~english Split by row

/// \~chinese 重组方式          \~english Reconstruct mode
typedef enum _MV_FG_RECONSTRUCT_MODE_
{
    /// \~chinese 旋转和翻转只支持MV_FG_PIXEL_TYPE_Mono8、MV_FG_PIXEL_TYPE_RGB8_Packed、MV_FG_PIXEL_TYPE_BGR8_Packed三种像素格式 \~english Rotate and flip only support MV_FG_PIXEL_TYPE_Mono8, MV_FG_PIXEL_TYPE_RGB8_Packed and MV_FG_PIXEL_TYPE_BGR8_Packed pixel formats
    RECONSTRUCT_MODE_ROTATE_90                  = (MV_FG_ROTATE_MODE | 0x0001),         ///< \~chinese 旋转90度    \~english Rotate 90 degrees
    RECONSTRUCT_MODE_ROTATE_180                 = (MV_FG_ROTATE_MODE | 0x0002),         ///< \~chinese 旋转180度   \~english Rotate 180 degrees
    RECONSTRUCT_MODE_ROTATE_270                 = (MV_FG_ROTATE_MODE | 0x0003),         ///< \~chinese 旋转270度   \~english Rotate 270 degrees

    RECONSTRUCT_MODE_FLIP_VERTICAL              = (MV_FG_FLIP_MODE | 0x0001),           ///< \~chinese 垂直翻转    \~english Flip vertically
    RECONSTRUCT_MODE_FLIP_HORIZONTAL            = (MV_FG_FLIP_MODE | 0x0002),           ///< \~chinese 水平翻转    \~english Flip horizontally

    /// \~chinese 按行拆分只支持对线阵相机采集的图像进行拆分  \~english Split by line only supports splitting the image collected by the line scan camera
    RECONSTRUCT_MODE_SPLIT_BY_LINE_2            = (MV_FG_SPLIT_BY_LINE_MODE | 0x0002),  ///< \~chinese 按行拆分成2张图像    \~english Split into 2 images by line
    RECONSTRUCT_MODE_SPLIT_BY_LINE_3            = (MV_FG_SPLIT_BY_LINE_MODE | 0x0003),  ///< \~chinese 按行拆分成3张图像    \~english Split into 3 images by line
    RECONSTRUCT_MODE_SPLIT_BY_LINE_4            = (MV_FG_SPLIT_BY_LINE_MODE | 0x0004),  ///< \~chinese 按行拆分成4张图像    \~english Split into 4 images by line

}MV_FG_RECONSTRUCT_MODE;

/// \~chinese 重组信息          \~english Reconstruct information
typedef struct _MV_FG_RECONSTRUCT_INFO_
{
    MV_FG_INPUT_IMAGE_INFO  stInputImageInfo;                       ///< [IN]  \~chinese 输入图片信息   \~english Input image information

    MV_FG_OUTPUT_IMAGE_INFO stOutputImageInfo[MV_FG_MAX_SPLIT_NUM]; ///< [IN][OUT]  \~chinese 输出图片信息    \~english Output image information

    MV_FG_RECONSTRUCT_MODE  enReconstructMode;                      ///< [IN]  \~chinese 重组方式       \~english Reconstruct mode

    unsigned int        nReserved[4];                               ///<       \~chinese 预留字段       \~english Reserved

}MV_FG_RECONSTRUCT_INFO;

///< \~chinese 节点权限         \~english Node permissions
typedef enum _MV_FG_NODE_ACCESS_MODE_
{
    ACCESS_MODE_NI                              = 0,                ///< \~chinese Value不可实现   \~english Value is not realizable
    ACCESS_MODE_NA                              = 1,                ///< \~chinese Value不可用     \~english Value not available
    ACCESS_MODE_WO                              = 2,                ///< \~chinese Value只写       \~english Value write only
    ACCESS_MODE_RO                              = 3,                ///< \~chinese Value只读       \~english Value read only
    ACCESS_MODE_RW                              = 4,                ///< \~chinese Value读写       \~english Value read and write
    ACCESS_MODE_UNDEFINED                       = 5,                ///< \~chinese Value未定义     \~english Value undefined

}MV_FG_NODE_ACCESS_MODE;

///< \~chinese 节点类型         \~english Node type
typedef enum _MV_FG_NODE_INTERFACE_TYPE_
{
    INTERFACE_TYPE_Value                        = 0,                ///< \~chinese Value        \~english Value
    INTERFACE_TYPE_Base                         = 1,                ///< \~chinese Base         \~english Base
    INTERFACE_TYPE_Integer                      = 2,                ///< \~chinese Integer      \~english Integer
    INTERFACE_TYPE_Boolean                      = 3,                ///< \~chinese Boolean      \~english Boolean
    INTERFACE_TYPE_Command                      = 4,                ///< \~chinese Command      \~english Command
    INTERFACE_TYPE_Float                        = 5,                ///< \~chinese Float        \~english Float
    INTERFACE_TYPE_String                       = 6,                ///< \~chinese String       \~english String
    INTERFACE_TYPE_Register                     = 7,                ///< \~chinese Register     \~english Register
    INTERFACE_TYPE_Category                     = 8,                ///< \~chinese Category     \~english Category
    INTERFACE_TYPE_Enumeration                  = 9,                ///< \~chinese Enumeration  \~english Enumeration
    INTERFACE_TYPE_EnumEntry                    = 10,               ///< \~chinese EnumEntry    \~english EnumEntry
    INTERFACE_TYPE_Port                         = 11,               ///< \~chinese Port         \~english Port

}MV_FG_NODE_INTERFACE_TYPE;

/// \~chinese Int类型信息           \~english Integer type information
typedef struct _MV_FG_INTVALUE_
{
    int64_t             nCurValue;                                  ///< [OUT] \~chinese 当前值    \~english Current value
    int64_t             nMax;                                       ///< [OUT] \~chinese 最大值    \~english Maximum value
    int64_t             nMin;                                       ///< [OUT] \~chinese 最小值    \~english Minimum value
    int64_t             nInc;                                       ///< [OUT] \~chinese 步长      \~english Increment

    unsigned int        nReserved[16];                              ///<       \~chinese 保留字段  \~english Reserved

}MV_FG_INTVALUE;

/// \~chinese XML描述文件中的最大符号数        \~english Maximum number of symbols in XML description file
#define MV_FG_MAX_SYMBOLIC_NUM                  64

/// \~chinese XML描述文件的符号最大长度        \~english Maximum symbol length of XML description file
#define MV_FG_MAX_SYMBOLIC_STRLEN               64

/// \~chinese 枚举类型信息            \~english Enumeration type information
typedef struct _MV_FG_ENUMVALUE_
{
    unsigned int        nCurValue;                                  ///< [OUT] \~chinese 当前值             \~english Current value
    char                strCurSymbolic[MV_FG_MAX_SYMBOLIC_STRLEN];  ///< [OUT] \~chinese 当前值的符号名称   \~english The symbolic name of the current value

    unsigned int        nSupportedNum;                              ///< [OUT] \~chinese 支持的枚举类型个数      \~english Number of supported enumeration types
    unsigned int        nSupportValue[MV_FG_MAX_SYMBOLIC_NUM];      ///< [OUT] \~chinese 支持的枚举类型的值      \~english Values of supported enumeration types
    char                strSymbolic[MV_FG_MAX_SYMBOLIC_NUM][MV_FG_MAX_SYMBOLIC_STRLEN];///< [OUT] \~chinese 支持的枚举类型的符号名称    \~english Symbolic names of supported enumeration types

    unsigned int        nReserved[4];                               ///<       \~chinese 保留字段       \~english Reserved

}MV_FG_ENUMVALUE;

/// \~chinese Float类型信息         \~english Float type information
typedef struct _MV_FG_FLOATVALUE_
{
    float               fCurValue;                                  ///< [OUT] \~chinese 当前值    \~english Current value
    float               fMax;                                       ///< [OUT] \~chinese 最大值    \~english Maximum value
    float               fMin;                                       ///< [OUT] \~chinese 最小值    \~english Minimum value

    unsigned int        nReserved[4];                               ///<       \~chinese 保留字段  \~english Reserved

}MV_FG_FLOATVALUE;

/// \~chinese String类型信息            \~english String type information
typedef struct _MV_FG_STRINGVALUE_
{
    char                strCurValue[256];                           ///< [OUT] \~chinese 当前值     \~english Current value
    int64_t             nMaxLength;                                 ///< [OUT] \~chinese 最大长度   \~english Maximum length

    unsigned int        nReserved[4];                               ///<       \~chinese 保留字段   \~english Reserved

}MV_FG_STRINGVALUE;

/// \~chinese 文件存取                  \~english File Access
typedef struct _MV_FG_FILE_ACCESS_
{
    const char*         pUserFileName;                              ///< [IN]  \~chinese 用户自定义的文件名     \~english User file name
    const char*         pDevFileName;                               ///< [IN]  \~chinese 卡/相机中的文件名, 来自UserSetSelector属性节点, 枚举型, 包括Default, UserSet1等值   \~english The file name in the Interface/Camera, from the UserSetSelector attribute node, enumeration type, including Default, UserSet1, etc

    unsigned int        nReserved[32];                              ///<       \~chinese 预留                   \~english Reserved

}MV_FG_FILE_ACCESS;

/// \~chinese 波特率           \~english Baud rate
#define MV_FG_BAUDRATE_9600                     0x00000001          ///< \~chinese 9600     \~english 9600
#define MV_FG_BAUDRATE_19200                    0x00000002          ///< \~chinese 19200    \~english 19200
#define MV_FG_BAUDRATE_38400                    0x00000004          ///< \~chinese 38400    \~english 38400
#define MV_FG_BAUDRATE_57600                    0x00000008          ///< \~chinese 57600    \~english 57600
#define MV_FG_BAUDRATE_115200                   0x00000010          ///< \~chinese 115200   \~english 115200
#define MV_FG_BAUDRATE_230400                   0x00000020          ///< \~chinese 230400   \~english 230400
#define MV_FG_BAUDRATE_460800                   0x00000040          ///< \~chinese 460800   \~english 460800
#define MV_FG_BAUDRATE_921600                   0x00000080          ///< \~chinese 921600   \~english 921600
#define MV_FG_BAUDRATE_AUTOMAX                  0x40000000          ///< \~chinese 自动协商至相机支持的最大值   \~english Automatically negotiates to the maximum value supported by the camera

///< \~chinese 配置指令         \~english Configuration commandy
typedef enum _MV_FG_CONFIG_CMD_
{
    CONFIG_CMD_INT64_BAUDRATE                   = 1,                ///< \~chinese 波特率，整型, 范围MV_FG_BAUDRATE_9600 ~ MV_FG_BAUDRATE_AUTOMAX, 默认值MV_FG_BAUDRATE_115200   \~english Baud rate, integer, Value range:MV_FG_BAUDRATE_9600 ~ MV_FG_BAUDRATE_AUTOMAX, Default vaule:MV_FG_BAUDRATE_115200

}MV_FG_CONFIG_CMD;

/// \~chinese 异常信息类型        \~english Exception information type
typedef enum _MV_FG_EXCEPTION_TYPE_
{
    EXCEPTION_TYPE_SYSTEM_TEMPERATURE_UPLIMIT          = 0x0080,           ///< \~chinese 温度上限                            \~english Temperature upper limit
    EXCEPTION_TYPE_SYSTEM_TEMPERATURE_LOWLIMIT         = 0x0081,           ///< \~chinese 温度下限限                          \~english Temperature lower limit
    EXCEPTION_TYPE_SYSTEM_DDR_INIT                     = 0x0082,           ///< \~chinese DDR初始化失败                       \~english DDR Initial Error

    EXCEPTION_TYPE_CARD_PACKETBUF_ERR                  = 0x0180,           ///< \~chinese 包缓存错误                          \~english Card Packet Buffer Error
    EXCEPTION_TYPE_CARD_ACKPACKETBUF_ERR               = 0x0181,           ///< \~chinese 响应包缓存错误                      \~english Card Ack Packet Buffer Error
    EXCEPTION_TYPE_CARD_EVENT_PACKETBUF_ERR            = 0x0182,           ///< \~chinese 事件包缓存错误                      \~english Card Event Packet Buffer Error

    EXCEPTION_TYPE_LINK0_STREAM_CRC_ERR                = 0x1080,           ///< \~chinese Link0 流CRC校验错误                 \~english Link0 Stream CRC Error Event
    EXCEPTION_TYPE_LINK0_STREAM_PACKET_RESEND          = 0x1081,           ///< \~chinese Link0 流重发包                      \~english Link0 Stream Packet Resend
    EXCEPTION_TYPE_LINK0_STREAM_CTRLPACKET_ERR         = 0x1082,           ///< \~chinese Link0 控制包错误                    \~english Link0 Control Packet Error Event
    EXCEPTION_TYPE_LINK0_PRETREATBUF_ERR               = 0x1090,           ///< \~chinese Link0 预处理缓存错误                \~english Link0 Pretreat Buffer Error
    EXCEPTION_TYPE_LINK0_CAM_ACK_RECVBUF_ERR           = 0x1091,           ///< \~chinese Link0 相机回包接收缓存错误          \~english Link0 Camera Ack Packet Receiver Buffer Error
    EXCEPTION_TYPE_LINK0_CAM_ACK_TRANSMITBUF_ERR       = 0x1092,           ///< \~chinese Link0 相机回包发送缓存错误          \~english Link0 Camera Packet Transmitter Buffer Error

    EXCEPTION_TYPE_LINK1_STREAM_CRC_ERR                = 0x1180,           ///< \~chinese Link1 流CRC校验错误                 \~english Link1 Stream CRC Error Event
    EXCEPTION_TYPE_LINK1_STREAM_PACKET_RESEND          = 0x1181,           ///< \~chinese Link1 流重发包                      \~english Link1 Stream Packet Resend
    EXCEPTION_TYPE_LINK1_STREAM_CTRLPACKET_ERR         = 0x1182,           ///< \~chinese Link1 控制包错误                    \~english Link1 Control Packet Error Event
    EXCEPTION_TYPE_LINK1_PRETREATBUF_ERR               = 0x1190,           ///< \~chinese Link1 预处理缓存错误                \~english Link1 Pretreat Buffer Error
    EXCEPTION_TYPE_LINK1_CAM_ACK_RECVBUF_ERR           = 0x1191,           ///< \~chinese Link1 相机回包接收缓存错误          \~english Link1 Camera Ack Packet Receiver Buffer Error
    EXCEPTION_TYPE_LINK1_CAM_ACK_TRANSMITBUF_ERR       = 0x1192,           ///< \~chinese Link1 相机回包发送缓存错误          \~english Link1 Camera Packet Transmitter Buffer Error

    EXCEPTION_TYPE_LINK2_STREAM_CRC_ERR                = 0x1280,           ///< \~chinese Link2 流CRC校验错误                 \~english Link2 Stream CRC Error Event
    EXCEPTION_TYPE_LINK2_STREAM_PACKET_RESEND          = 0x1281,           ///< \~chinese Link2 流重发包                      \~english Link2 Stream Packet Resend
    EXCEPTION_TYPE_LINK2_STREAM_CTRLPACKET_ERR         = 0x1282,           ///< \~chinese Link2 控制包错误                    \~english Link2 Control Packet Error Event
    EXCEPTION_TYPE_LINK2_PRETREATBUF_ERR               = 0x1290,           ///< \~chinese Link2 预处理缓存错误                \~english Link2 Pretreat Buffer Error
    EXCEPTION_TYPE_LINK2_CAM_ACK_RECVBUF_ERR           = 0x1291,           ///< \~chinese Link2 相机回包接收缓存错误          \~english Link2 Camera Ack Packet Receiver Buffer Error
    EXCEPTION_TYPE_LINK2_CAM_ACK_TRANSMITBUF_ERR       = 0x1292,           ///< \~chinese Link2 相机回包发送缓存错误          \~english Link2 Camera Packet Transmitter Buffer Error

    EXCEPTION_TYPE_LINK3_STREAM_CRC_ERR                = 0x1380,           ///< \~chinese Link3 流CRC校验错误                 \~english Link3 Stream CRC Error Event
    EXCEPTION_TYPE_LINK3_STREAM_PACKET_RESEND          = 0x1381,           ///< \~chinese Link3 流重发包                      \~english Link3 Stream Packet Resend
    EXCEPTION_TYPE_LINK3_STREAM_CTRLPACKET_ERR         = 0x1382,           ///< \~chinese Link3 控制包错误                    \~english Link3 Control Packet Error Event
    EXCEPTION_TYPE_LINK3_PRETREATBUF_ERR               = 0x1390,           ///< \~chinese Link3 预处理缓存错误                \~english Link3 Pretreat Buffer Error
    EXCEPTION_TYPE_LINK3_CAM_ACK_RECVBUF_ERR           = 0x1391,           ///< \~chinese Link3 相机回包接收缓存错误          \~english Link3 Camera Ack Packet Receiver Buffer Error
    EXCEPTION_TYPE_LINK3_CAM_ACK_TRANSMITBUF_ERR       = 0x1392,           ///< \~chinese Link3 相机回包发送缓存错误          \~english Link3 Camera Packet Transmitter Buffer Error

    EXCEPTION_TYPE_STREAM0_DROP_FRAME_IMAGE            = 0x2080,           ///< \~chinese STREAM0 卡端图像帧缓存丢弃          \~english STREAM0 Drop Image Frame
    EXCEPTION_TYPE_STREAM0_IMAGE_DATACOUNT_ERR         = 0x2081,           ///< \~chinese STREAM0 接收图像（大小）计数异常    \~english STREAM0 Receive Image Data Count Error
    EXCEPTION_TYPE_STREAM0_DROP_FRAME_TRIGGER          = 0x2082,           ///< \~chinese STREAM0 卡端帧触发丢弃              \~english STREAM0 Drop Frame Trigger
    EXCEPTION_TYPE_STREAM0_QUEUEBUF_ERR                = 0x2090,           ///< \~chinese STREAM0 QUEUE缓存异常               \~english STREAM0 QUEUE Buffer Error
    EXCEPTION_TYPE_STREAM0_WDMABUF_ERR                 = 0x2091,           ///< \~chinese STREAM0 WDMA缓存异常                \~english STREAM0 WDMA Buffer Error
    EXCEPTION_TYPE_STREAM0_RDMABUF_ERR                 = 0x2092,           ///< \~chinese STREAM0 RDMA缓存异常                \~english STREAM0 RDMA Buffer Error
    EXCEPTION_TYPE_STREAM0_PACKETBUF_ERR               = 0x2093,           ///< \~chinese STREAM0 PACKET缓存异常              \~english STREAM0 PACKET Buffer Error
    EXCEPTION_TYPE_STREAM0_WDMALENGTH_ERR              = 0x2094,           ///< \~chinese STREAM0 WDMA长度异常                \~english STREAM0 WDMA Length Error
    EXCEPTION_TYPE_STREAM0_RDMALENGTH_ERR              = 0x2095,           ///< \~chinese STREAM0 RDMA长度异常                \~english STREAM0 RDMA Length Error

    EXCEPTION_TYPE_STREAM1_DROP_FRAME_IMAGE            = 0x2180,           ///< \~chinese STREAM1 卡端图像帧缓存丢弃          \~english STREAM1 Drop Image Frame
    EXCEPTION_TYPE_STREAM1_IMAGE_DATACOUNT_ERR         = 0x2181,           ///< \~chinese STREAM1 接收图像（大小）计数异常    \~english STREAM1 Receive Image Data Count Error
    EXCEPTION_TYPE_STREAM1_DROP_FRAME_TRIGGER          = 0x2182,           ///< \~chinese STREAM1 卡端帧触发丢弃              \~english STREAM1 Drop Frame Trigger
    EXCEPTION_TYPE_STREAM1_QUEUEBUF_ERR                = 0x2190,           ///< \~chinese STREAM1 QUEUE缓存异常               \~english STREAM1 QUEUE Buffer Error
    EXCEPTION_TYPE_STREAM1_WDMABUF_ERR                 = 0x2191,           ///< \~chinese STREAM1 WDMA缓存异常                \~english STREAM1 WDMA Buffer Error
    EXCEPTION_TYPE_STREAM1_RDMABUF_ERR                 = 0x2192,           ///< \~chinese STREAM1 RDMA缓存异常                \~english STREAM1 RDMA Buffer Error
    EXCEPTION_TYPE_STREAM1_PACKETBUF_ERR               = 0x2193,           ///< \~chinese STREAM1 PACKET缓存异常              \~english STREAM1 PACKET Buffer Error
    EXCEPTION_TYPE_STREAM1_WDMALENGTH_ERR              = 0x2194,           ///< \~chinese STREAM1 WDMA长度异常                \~english STREAM1 WDMA Length Error
    EXCEPTION_TYPE_STREAM1_RDMALENGTH_ERR              = 0x2195,           ///< \~chinese STREAM1 RDMA长度异常                \~english STREAM1 RDMA Length Error

    EXCEPTION_TYPE_STREAM2_DROP_FRAME_IMAGE            = 0x2280,           ///< \~chinese STREAM2 卡端图像帧缓存丢弃          \~english STREAM2 Drop Image Frame
    EXCEPTION_TYPE_STREAM2_IMAGE_DATACOUNT_ERR         = 0x2281,           ///< \~chinese STREAM2 接收图像（大小）计数异常    \~english STREAM2 Receive Image Data Count Error
    EXCEPTION_TYPE_STREAM2_DROP_FRAME_TRIGGER          = 0x2282,           ///< \~chinese STREAM2 卡端帧触发丢弃              \~english STREAM2 Drop Frame Trigger
    EXCEPTION_TYPE_STREAM2_QUEUEBUF_ERR                = 0x2290,           ///< \~chinese STREAM2 QUEUE缓存异常               \~english STREAM2 QUEUE Buffer Error
    EXCEPTION_TYPE_STREAM2_WDMABUF_ERR                 = 0x2291,           ///< \~chinese STREAM2 WDMA缓存异常                \~english STREAM2 WDMA Buffer Error
    EXCEPTION_TYPE_STREAM2_RDMABUF_ERR                 = 0x2292,           ///< \~chinese STREAM2 RDMA缓存异常                \~english STREAM2 RDMA Buffer Error
    EXCEPTION_TYPE_STREAM2_PACKETBUF_ERR               = 0x2293,           ///< \~chinese STREAM2 PACKET缓存异常              \~english STREAM2 PACKET Buffer Error
    EXCEPTION_TYPE_STREAM2_WDMALENGTH_ERR              = 0x2294,           ///< \~chinese STREAM2 WDMA长度异常                \~english STREAM2 WDMA Length Error
    EXCEPTION_TYPE_STREAM2_RDMALENGTH_ERR              = 0x2295,           ///< \~chinese STREAM2 RDMA长度异常                \~english STREAM2 RDMA Length Error

    EXCEPTION_TYPE_STREAM3_DROP_FRAME_IMAGE            = 0x2380,           ///< \~chinese STREAM3 卡端图像帧缓存丢弃          \~english STREAM3 Drop Image Frame
    EXCEPTION_TYPE_STREAM3_IMAGE_DATACOUNT_ERR         = 0x2381,           ///< \~chinese STREAM3 接收图像（大小）计数异常    \~english STREAM3 Receive Image Data Count Error
    EXCEPTION_TYPE_STREAM3_DROP_FRAME_TRIGGER          = 0x2382,           ///< \~chinese STREAM3 卡端帧触发丢弃              \~english STREAM3 Drop Frame Trigger
    EXCEPTION_TYPE_STREAM3_QUEUEBUF_ERR                = 0x2390,           ///< \~chinese STREAM3 QUEUE缓存异常               \~english STREAM3 QUEUE Buffer Error
    EXCEPTION_TYPE_STREAM3_WDMABUF_ERR                 = 0x2391,           ///< \~chinese STREAM3 WDMA缓存异常                \~english STREAM3 WDMA Buffer Error
    EXCEPTION_TYPE_STREAM3_RDMABUF_ERR                 = 0x2392,           ///< \~chinese STREAM3 RDMA缓存异常                \~english STREAM3 RDMA Buffer Error
    EXCEPTION_TYPE_STREAM3_PACKETBUF_ERR               = 0x2393,           ///< \~chinese STREAM3 PACKET缓存异常              \~english STREAM3 PACKET Buffer Error
    EXCEPTION_TYPE_STREAM3_WDMALENGTH_ERR              = 0x2394,           ///< \~chinese STREAM3 WDMA长度异常                \~english STREAM3 WDMA Length Error
    EXCEPTION_TYPE_STREAM3_RDMALENGTH_ERR              = 0x2395,           ///< \~chinese STREAM3 RDMA长度异常                \~english STREAM3 RDMA Length Error


    EXCEPTION_TYPE_PCIE_SCHEDULEBUF_ERR                = 0x3088,           ///< \~chinese 调度模块缓存异常                    \~english Sched Buffer Error
    EXCEPTION_TYPE_PCIE_SCHEDULE_ERR                   = 0x3089,           ///< \~chinese 调度结果到异常值                    \~english Sched Error

    EXCEPTION_TYPE_PCIE_LINK0_RECVBUF_ERR              = 0x3180,           ///< \~chinese Link0 缓存Link的buffer异常           \~english Link0 Reveive Buffer Error
    EXCEPTION_TYPE_PCIE_LINK0_LENGTH_ERR               = 0x3181,           ///< \~chinese Link0 控制包长度异常                \~english Link0 Length Erro 
    EXCEPTION_TYPE_PCIE_LINK0_SOFT_RECVBUF_ERR         = 0x3280,           ///< \~chinese Link0 缓存Link的buffer异常           \~english Link0 Soft Reveive Buffer Error
    EXCEPTION_TYPE_PCIE_LINK0_SOFT_LENGTH_ERR          = 0x3281,           ///< \~chinese Link0 控制包长度异常                \~english Link0 Soft Length Erro 
    
    EXCEPTION_TYPE_PCIE_LINK1_RECVBUF_ERR              = 0x3188,           ///< \~chinese Link1 缓存Link的buffer异常           \~english Link1 Reveive Buffer Error
    EXCEPTION_TYPE_PCIE_LINK1_LENGTH_ERR               = 0x3189,           ///< \~chinese Link1 控制包长度异常                \~english Link1 Length Erro 
    EXCEPTION_TYPE_PCIE_LINK1_SOFT_RECVBUF_ERR         = 0x3288,           ///< \~chinese Link1 缓存Link的buffer异常           \~english Link1 Soft Reveive Buffer Error
    EXCEPTION_TYPE_PCIE_LINK1_SOFT_LENGTH_ERR          = 0x3289,           ///< \~chinese Link1 控制包长度异常                \~english Link1 Soft Length Erro 

    EXCEPTION_TYPE_PCIE_LINK2_RECVBUF_ERR              = 0x3190,           ///< \~chinese Link2 缓存Link的buffer异常           \~english Link2 Reveive Buffer Error
    EXCEPTION_TYPE_PCIE_LINK2_LENGTH_ERR               = 0x3191,           ///< \~chinese Link2 控制包长度异常                \~english Link2 Length Erro 
    EXCEPTION_TYPE_PCIE_LINK2_SOFT_RECVBUF_ERR         = 0x3290,           ///< \~chinese Link2 缓存Link的buffer异常           \~english Link2 Soft Reveive Buffer Error
    EXCEPTION_TYPE_PCIE_LINK2_SOFT_LENGTH_ERR          = 0x3291,           ///< \~chinese Link2 控制包长度异常                \~english Link2 Soft Length Erro 

    EXCEPTION_TYPE_PCIE_LINK3_RECVBUF_ERR              = 0x3198,           ///< \~chinese Link3 缓存Link的buffer异常           \~english Link3 Reveive Buffer Error
    EXCEPTION_TYPE_PCIE_LINK3_LENGTH_ERR               = 0x3199,           ///< \~chinese Link3 控制包长度异常                \~english Link3 Length Erro 
    EXCEPTION_TYPE_PCIE_LINK3_SOFT_RECVBUF_ERR         = 0x3298,           ///< \~chinese Link3 缓存Link的buffer异常           \~english Link3 Soft Reveive Buffer Error
    EXCEPTION_TYPE_PCIE_LINK3_SOFT_LENGTH_ERR          = 0x3299,           ///< \~chinese Link3 控制包长度异常                \~english Link3 Soft Length Erro 
    
    EXCEPTION_TYPE_PCIE_STREAM0_RECVBUF_ERR            = 0x3382,           ///< \~chinese STREAM0 缓存Stream的fifo异常         \~english STREAM0 Receive Buffer Error
    EXCEPTION_TYPE_PCIE_STREAM0_LIST_ERR               = 0x3383,           ///< \~chinese STREAM0 链表格式错误                \~english STREAM0 List Error
    EXCEPTION_TYPE_PCIE_STREAM0_SIZE_ERR               = 0x3384,           ///< \~chinese STREAM0 图像大小与内存不匹配        \~english STREAM0 Size Error

    EXCEPTION_TYPE_PCIE_STREAM1_RECVBUF_ERR            = 0x338A,           ///< \~chinese STREAM1 缓存Stream的fifo异常         \~english STREAM1 Receive Buffer Error
    EXCEPTION_TYPE_PCIE_STREAM1_LIST_ERR               = 0x338B,           ///< \~chinese STREAM1 链表格式错误                \~english STREAM1 List Error
    EXCEPTION_TYPE_PCIE_STREAM1_SIZE_ERR               = 0x338C,           ///< \~chinese STREAM1 图像大小与内存不匹配        \~english STREAM1 Size Error

    EXCEPTION_TYPE_PCIE_STREAM2_RECVBUF_ERR            = 0x3392,           ///< \~chinese STREAM2 缓存Stream的fifo异常         \~english STREAM2 Receive Buffer Error
    EXCEPTION_TYPE_PCIE_STREAM2_LIST_ERR               = 0x3393,           ///< \~chinese STREAM2 链表格式错误                \~english STREAM2 List Error
    EXCEPTION_TYPE_PCIE_STREAM2_SIZE_ERR               = 0x3394,           ///< \~chinese STREAM2 图像大小与内存不匹配        \~english STREAM2 Size Error

    EXCEPTION_TYPE_PCIE_STREAM3_RECVBUF_ERR            = 0x339A,           ///< \~chinese STREAM3 缓存Stream的fifo异常         \~english STREAM3 Receive Buffer Error
    EXCEPTION_TYPE_PCIE_STREAM3_LIST_ERR               = 0x339B,           ///< \~chinese STREAM3 链表格式错误                \~english STREAM3 List Error
    EXCEPTION_TYPE_PCIE_STREAM3_SIZE_ERR               = 0x339C,           ///< \~chinese STREAM3 图像大小与内存不匹配        \~english STREAM3 Size Error

    EXCEPTION_TYPE_CAMERA_DISCONNECT_ERR               = 0x10000001,       ///< \~chinese CAMERA 相机掉线错误                 \~english CAMERA Disconnect Error


}MV_FG_EXCEPTION_TYPE;

///< \~chinese 事件名称最大长度         \~english Maximum length of event name
#define MV_FG_MAX_EVENT_NAME_SIZE               128

/// \~chinese 事件信息          \~english Event information
typedef struct _MV_FG_EVENT_INFO_
{
    char                EventName[MV_FG_MAX_EVENT_NAME_SIZE];       ///< [OUT] \~chinese 事件名称   \~english Event name
    unsigned int        nEventID;                                   ///< [OUT] \~chinese 事件号     \~english Event ID

    uint64_t            nBlockID;                                   ///< [OUT] \~chinese 帧号，流事件有效（暂无固件支持）   \~english Frame ID, stream event valid（）

    uint64_t            nTimestamp;                                 ///< [OUT] \~chinese 时间戳     \~english Timestamp

    void*               pEventData;                                 ///< [OUT] \~chinese 事件数据，内部缓存，需要及时进行数据处理（暂无固件支持）   \~english Event data, internal buffer, need to be processed in time
    unsigned int        nEventDataSize;                             ///< [OUT] \~chinese 事件数据长度（暂无固件支持）     \~english Event data szie

    unsigned int        nReserved[4];                               ///<       \~chinese 预留     \~english Reserved

}MV_FG_EVENT_INFO;

/// \~chinese 自定义点                    \~english Point defined
typedef struct _MVFG_POINTF
{
    float           fX;             ///<  \~chinese 该点距离图像左边缘距离，根据图像的相对位置，范围为[0.0 , 1.0]   \~english Distance From Left，Range[0.0, 1.0]
    float           fY;             ///<  \~chinese 该点距离图像上边缘距离，根据图像的相对位置，范围为[0.0 , 1.0]   \~english Distance From Top，Range[0.0, 1.0]
    unsigned int    nReserved[4];   ///<  \~chinese 预留                        \~english Reserved

}MVFG_POINTF;

/// \~chinese 辅助线颜色                \~english Color of Auxiliary Line
typedef struct _MVFG_COLORF
{
    float           fR;             ///<  \~chinese 红色，根据像素颜色的相对深度，范围为[0.0 , 1.0]，代表着[0, 255]的颜色深度   \~english Red，Range[0.0, 1.0]
    float           fG;             ///<  \~chinese 绿色，根据像素颜色的相对深度，范围为[0.0 , 1.0]，代表着[0, 255]的颜色深度   \~english Green，Range[0.0, 1.0]
    float           fB;             ///<  \~chinese 蓝色，根据像素颜色的相对深度，范围为[0.0 , 1.0]，代表着[0, 255]的颜色深度   \~english Blue，Range[0.0, 1.0]
    float           fAlpha;         ///<  \~chinese 透明度，根据像素颜色的相对透明度，范围为[0.0 , 1.0] (此参数功能暂不支持)    \~english Alpha，Range[0.0, 1.0](Not Support)
    unsigned int    nReserved[4];   ///<  \~chinese 预留                        \~english Reserved

}MVFG_COLORF;

/// \~chinese 矩形框区域信息            \~english Rect Area Info
typedef struct _MVFG_RECT_INFO
{
    float           fTop;           ///<  \~chinese 矩形上边缘距离图像上边缘的距离，根据图像的相对位置，范围为[0.0 , 1.0]   \~english Distance From Image Top，Range[0, 1.0]
    float           fBottom;        ///<  \~chinese 矩形下边缘距离图像上边缘的距离，根据图像的相对位置，范围为[0.0 , 1.0]   \~english Distance From Image Top，Range[0, 1.0]
    float           fLeft;          ///<  \~chinese 矩形左边缘距离图像左边缘的距离，根据图像的相对位置，范围为[0.0 , 1.0]   \~english Distance From Image Left，Range[0, 1.0]
    float           fRight;         ///<  \~chinese 矩形右边缘距离图像左边缘的距离，根据图像的相对位置，范围为[0.0 , 1.0]   \~english Distance From Image Left，Range[0, 1.0]

    MVFG_COLORF     stColor;        ///<  \~chinese 辅助线颜色                      \~english Color of Auxiliary Line
    unsigned int    nLineWidth;     ///<  \~chinese 辅助线宽度，宽度只能是1或2      \~english Width of Auxiliary Line, width is 1 or 2
    unsigned int    nReserved[4];   ///<  \~chinese 预留                            \~english Reserved

}MVFG_RECT_INFO;

/// \~chinese 圆形框区域信息            \~english Circle Area Info
typedef struct _MVFG_CIRCLE_INFO
{
    MVFG_POINTF     stCenterPoint;  ///<  \~chinese 圆心信息                        \~english Circle Point Info

    float           fR1;            ///<  \~chinese 宽向半径，根据图像的相对位置[0, 1.0]，半径与圆心的位置有关，需保证画出的圆在显示框范围之内，否则报错  \~english Windth Radius, Range[0, 1.0]
    float           fR2;            ///<  \~chinese 高向半径，根据图像的相对位置[0, 1.0]，半径与圆心的位置有关，需保证画出的圆在显示框范围之内，否则报错  \~english Height Radius, Range[0, 1.0]

    MVFG_COLORF     stColor;        ///<  \~chinese 辅助线颜色信息                  \~english Color of Auxiliary Line
    unsigned int    nLineWidth;     ///<  \~chinese 辅助线宽度，宽度只能是1或2      \~english Width of Auxiliary Line, width is 1 or 2
    unsigned int    nReserved[4];   ///<  \~chinese 预留                            \~english Reserved

}MVFG_CIRCLE_INFO;

/// \~chinese 线条辅助线信息    \~english Linear Auxiliary Line Info
typedef struct _MVFG_LINES_INFO
{
    MVFG_POINTF     stStartPoint;   ///<  \~chinese 线条辅助线的起始点坐标          \~english The Start Point of Auxiliary Line
    MVFG_POINTF     stEndPoint;     ///<  \~chinese 线条辅助线的终点坐标            \~english The End Point of Auxiliary Line
    MVFG_COLORF     stColor;        ///<  \~chinese 辅助线颜色信息                  \~english Color of Auxiliary Line
    unsigned int    nLineWidth;     ///<  \~chinese 辅助线宽度，宽度只能是1或2      \~english Width of Auxiliary Line, width is 1 or 2
    unsigned int    nReserved[4];   ///<  \~chinese 预留                            \~english Reserved

}MVFG_LINES_INFO;

/// \~chinese 取流策略                  \~english The strategy of Grabbing
typedef enum _MV_FG_GRAB_STRATEGY_
{
    MV_FG_GrabStrategy_OneByOne            = 0,                        ///< \~chinese 从旧到新一帧一帧的获取图像   \~english Grab One By One
    MV_FG_GrabStrategy_LatestImagesOnly    = 1,                        ///< \~chinese 获取列表中最新的一帧图像     \~english Grab The Latest Image
    MV_FG_GrabStrategy_LatestImages        = 2,                        ///< \~chinese 获取列表中最新的图像         \~english Grab The Latest Images
    MV_FG_GrabStrategy_UpcomingImage       = 3,                        ///< \~chinese 等待下一帧图像               \~english Grab The Upcoming Image

}MV_FG_GRAB_STRATEGY;

/// \~chinese 帧缓存信息回调函数         \~english Frame buffer information callback function
typedef void(__stdcall *MV_FG_FrameCallBack)(MV_FG_BUFFER_INFO* pstBufferInfo, void* pUser);

/// \~chinese 异常信息回调函数          \~english Exception information callback function
typedef void(__stdcall *MV_FG_ExceptionCallBack)(MV_FG_EXCEPTION_TYPE enExceptionType, void* pUser);

/// \~chinese 事件回调函数            \~english Event callback function
typedef void(__stdcall *MV_FG_EventCallBack)(MV_FG_EVENT_INFO* pstEventInfo, void* pUser);

#ifdef __cplusplus
}
#endif 

#endif //_MV_FG_DEFINES_H_
