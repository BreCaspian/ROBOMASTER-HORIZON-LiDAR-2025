#ifndef _MV_FG_ERROR_DEFINE_H_
#define _MV_FG_ERROR_DEFINE_H_

typedef int MV_FG_ERROR;

/********************************************************************/
///  \~chinese
///  \name 正确码定义
///  @{
///  \~english
///  \name Definition of correct code
///  @{
#define    MV_FG_SUCCESS                  0          ///< \~chinese 成功，无错误             \~english Successed, no error
/// @}

/********************************************************************/
///  \~chinese
///  \name SDK错误:范围0x80190001-0x801900FF
///  @{
///  \~english
///  \name SDK Error Codes: Range from 0x80190001-0x801900FF
///  @{
#define    MV_FG_ERR_ERROR                0x80190001    ///< \~chinese 未知的错误                                                             \~english Unknown error
#define    MV_FG_ERR_NOT_INITIALIZED      0x80190002    ///< \~chinese 模块或资源未初始化                                                   \~english Uninitialized
#define    MV_FG_ERR_NOT_IMPLEMENTED      0x80190003    ///< \~chinese 功能未实现                                                           \~english Not implemented
#define    MV_FG_ERR_RESOURCE_IN_USE      0x80190004    ///< \~chinese 请求的资源正在使用中                                                 \~english Resource occupied
#define    MV_FG_ERR_ACCESS_DENIED        0x80190005    ///< \~chinese 请求的操作无权限执行;例如,可能是指采集卡或相机已经被其他进程打开了        \~english No permission
#define    MV_FG_ERR_INVALID_HANDLE       0x80190006    ///< \~chinese 无效的句柄;例如,可能是指使用了错误的句柄或者空指针句柄                     \~english Invalid handle
#define    MV_FG_ERR_INVALID_ID           0x80190007    ///< \~chinese 无效的ID                                                             \~english Invalid ID
#define    MV_FG_ERR_NO_DATA              0x80190008    ///< \~chinese 没有可以操作的数据                                                         \~english No data
#define    MV_FG_ERR_INVALID_PARAMETER    0x80190009    ///< \~chinese 无效的参数                                                               \~english Invalid parameter
#define    MV_FG_ERR_IO                   0x80190010    ///< \~chinese IO错误                                                                   \~english IO error
#define    MV_FG_ERR_TIMEOUT              0x80190011    ///< \~chinese 操作超时;例如,可能是指在获取图像的接口,设置的超时时间太小,导致在超时时间内未能获取到图像数据   \~english Timeout
#define    MV_FG_ERR_ABORT                0x80190012    ///< \~chinese 操作在完成之前被中止                                                        \~english The operation was interrupted
#define    MV_FG_ERR_INVALID_BUFFER       0x80190013    ///< \~chinese 无效的缓存或者无缓存                                                     \~english Invalid buffer
#define    MV_FG_ERR_NOT_AVAILABLE        0x80190014    ///< \~chinese 在当前状态下，请求的资源或信息不可用                                    \~english Unreachable
#define    MV_FG_ERR_INVALID_ADDRESS      0x80190015    ///< \~chinese 地址无效或超过范围                                                       \~english Invalid address
#define    MV_FG_ERR_BUFFER_TOO_SMALL     0x80190016    ///< \~chinese 缓存太小                                                                 \~english Buffer too small
#define    MV_FG_ERR_INVALID_INDEX        0x80190017    ///< \~chinese 无效的索引                                                                 \~english Invalid index
#define    MV_FG_ERR_PARSING_CHUNK_DATA   0x80190018    ///< \~chinese 解析Chunk失败                                                            \~english Failed to parse Chunk
#define    MV_FG_ERR_INVALID_VALUE        0x80190019    ///< \~chinese 无效的值                                                                 \~english Invalid value
#define    MV_FG_ERR_RESOURCE_EXHAUSTED   0x80190020    ///< \~chinese 系统资源耗尽;例如,可能是指可用文件句柄数量达到上限。                         \~english Resource exhaustion
#define    MV_FG_ERR_OUT_OF_MEMORY        0x80190021    ///< \~chinese 内存申请失败;例如,可能是指系统或系统中的其他硬件（采集卡）中的内存不足。     \~english Applying memory failed
#define    MV_FG_ERR_BUSY                 0x80190022    ///< \~chinese 由于请求操作的模块，此时正在执行其他操作，无法和请求的操作同时执行。             \~english Be busy
#define    MV_FG_ERR_LOADLIBRARY          0x80190023    ///< \~chinese 动态库加载失败                                                           \~english Dynamic library loading failed
#define    MV_FG_ERR_CALLORDER            0x80190024    ///< \~chinese 函数调用错误                                                             \~english The order of function calls is wrong
#define    MV_FG_ERR_VERSION              0x80190025    ///< \~chinese 版本不匹配                                                             \~english Version not matched
/// @}

/********************************************************************/
///  \~chinese
///  \name GenICam系列错误:范围0x80190100-0x801901FF
///  @{
///  \~english
///  \name GenICam Series Error Codes: Range from 0x80190100-0x801901FF
///  @{
#define    MV_FG_ERR_GC_GENERIC             0x80190100  ///< \~chinese 通用错误         \~english General error
#define    MV_FG_ERR_GC_ARGUMENT            0x80190101  ///< \~chinese 无效的参数       \~english Illegal parameters
#define    MV_FG_ERR_GC_RANGE               0x80190102  ///< \~chinese 参数范围错误     \~english The value is out of range
#define    MV_FG_ERR_GC_PROPERTY            0x80190103  ///< \~chinese 属性错误         \~english Property error
#define    MV_FG_ERR_GC_RUNTIME             0x80190104  ///< \~chinese 运行环境错误     \~english Running environment error
#define    MV_FG_ERR_GC_LOGICAL             0x80190105  ///< \~chinese 逻辑错误         \~english Logical error
#define    MV_FG_ERR_GC_ACCESS              0x80190106  ///< \~chinese 权限错误         \~english Accessing condition error
#define    MV_FG_ERR_GC_TIMEOUT             0x80190107  ///< \~chinese 超时             \~english Timeout
#define    MV_FG_ERR_GC_DYNAMICCAST         0x80190108  ///< \~chinese 转换异常         \~english Transformation exception
#define    MV_FG_ERR_GC_UNKNOW              0x801901FF  ///< \~chinese 未知错误         \~english Unknown error
/// @}

/********************************************************************/
///  \~chinese
///  \name 图像处理错误:范围0x80190200-0x801902FF
///  @{
///  \~english
///  \name Image Process Error Codes: Range from 0x80190200-0x801902FF
///  @{
#define    MV_FG_ERR_IMG_HANDLE           0x80190200    ///< \~chinese 图像处理库句柄错误             \~english Handle error
#define    MV_FG_ERR_IMG_SUPPORT          0x80190201    ///< \~chinese 图像处理库不支持               \~english Not supported
#define    MV_FG_ERR_IMG_PARAMETER        0x80190202    ///< \~chinese 图像处理库参数错误             \~english Parameter error
#define    MV_FG_ERR_IMG_OVERFLOW         0x80190203    ///< \~chinese 图像处理库内存溢出             \~english Out of memory
#define    MV_FG_ERR_IMG_INITIALIZED      0x80190204    ///< \~chinese 图像处理库操作未初始化         \~english Uninitialized
#define    MV_FG_ERR_IMG_RESOURCE         0x80190205    ///< \~chinese 图像处理库资源申请释放失败     \~english Resource release failed
#define    MV_FG_ERR_IMG_ENCRYPT          0x80190206    ///< \~chinese 图像加密错误                   \~english Encryption error
#define    MV_FG_ERR_IMG_FORMAT           0x80190207    ///< \~chinese 图像格式不正确或者不支持       \~english Incorrect format or unsupported
#define    MV_FG_ERR_IMG_SIZE             0x80190208    ///< \~chinese 图像宽高不正确或者超出范围     \~english Incorrect width and height or out of range
#define    MV_FG_ERR_IMG_STEP             0x80190209    ///< \~chinese 图像宽高与step参数不匹配       \~english Width and height do not match step parameters
#define    MV_FG_ERR_IMG_DATA_NULL        0x80190210    ///< \~chinese 图像数据存储地址为空(某个分量) \~english Data storage address is empty (a component)
#define    MV_FG_ERR_IMG_ABILITY_ARG      0x80190211    ///< \~chinese 图像算法ABILITY存在无效参数    \~english Invalid parameter for algorithm ABILITY
#define    MV_FG_ERR_IMG_UNKNOW           0x801902FF    ///< \~chinese 图像处理未知错误               \~english Unknown error
/// @}

#endif //_MV_FG_ERROR_DEFINE_H_
