
#ifndef     __MV_LOCK_H__
#define     __MV_LOCK_H__

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <asm-generic/errno.h>
#include <asm-generic/errno-base.h>
// #include "InnerDefine.h"


typedef unsigned long OS_LOCK_FLAG;


// #define MV_OK               0
// #define MV_ERROR            -1


typedef struct
{
    void* pSpinLock;
}MV_LOCK;

extern int MV_LockCheck(void);
extern int MV_LockInit(MV_LOCK* aLock);
extern int MV_LockFree(MV_LOCK* aLock);
extern int MV_LockAcquire(MV_LOCK* aLock);
extern int MV_LockRelease(MV_LOCK* aLock);
extern int MV_LockIrqAcquire(MV_LOCK* aLock);
extern int MV_LockIrqRelease(MV_LOCK* aLock);
extern int MV_LockBhAcquire(MV_LOCK* aLock);
extern int MV_LockBhRelease(MV_LOCK* aLock);

typedef struct
{
    void* pMutex;
}MV_MUTEX;

extern int MV_MutexInit(MV_MUTEX* pstMvMutex);
extern int MV_MutexFree(MV_MUTEX* pstMvMutex);
extern int MV_MutexLock(MV_MUTEX* pstMvMutex);
extern int MV_MutexUnlock(MV_MUTEX* pstMvMutex);

#define OS_ASSERT( aExpression )                                                                    \
{                                                                                                   \
    if (!(aExpression))                                                                             \
    {                                                                                               \
        printk("OS: ** ASSERT ** ");                                                              \
        printk("\n");                                                                             \
    }                                                                                               \
}    //assert(aExpression);                                                                          




#ifdef __cplusplus
}
#endif // __cplusplus


#endif        //    __MV_LOCK_H__