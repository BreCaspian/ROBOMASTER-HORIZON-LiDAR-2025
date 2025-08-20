
#ifndef     __MV_SOCKET_H__
#define     __MV_SOCKET_H__

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <linux/kernel.h>
#include "linux/net.h"
#include "linux/socket.h"
#include "linux/uio.h"
#include "net/sock.h"

typedef struct
{
    void*       hSocket;
    void*       hMsgHdr;
    void*       hMsgData;
    void*       hSockAddrIn;
    void*       hSockAddrInServer;
}MV_SOCKET;

extern int MV_SocketCreate(MV_SOCKET* pSocket);

extern int MV_SocketBind(MV_SOCKET* pSocket);

extern int MV_SocketSendto(MV_SOCKET* pSocket, void* pData, int nDataLen);

extern int MV_SocketRelease(MV_SOCKET* pSocket);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif  // __MV_SOCKET_H__
