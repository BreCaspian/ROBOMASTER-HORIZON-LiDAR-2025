#ifndef     __MV_IO_H__
#define     __MV_IO_H__

#ifdef __cplusplus
extern "C"
{
#endif 

extern void MV_IOWRITE32(unsigned int value, volatile void* addr);

extern unsigned int MV_IOREAD32(const volatile void* addr);

#ifdef __cplusplus
}
#endif

#endif  // __MV_IO_H__
