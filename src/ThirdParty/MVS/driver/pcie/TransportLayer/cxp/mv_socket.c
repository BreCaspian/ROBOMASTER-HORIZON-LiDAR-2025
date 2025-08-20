#include "mv_socket.h"
#include "MvErrorDefine.h"

int MV_SocketCreate(MV_SOCKET* pSocket)
{
    int                 nRet = 0;
    if (NULL == pSocket)
    {
        return -EINVAL;
    }
    
    do
    {
        pSocket->hMsgHdr = (struct msghdr*)kzalloc(sizeof(struct msghdr), GFP_KERNEL);
        if (NULL == pSocket->hMsgHdr)
        {
            nRet = -ENOMEM;
            break;
        }

        pSocket->hMsgData = (struct kvec*)kzalloc(sizeof(struct kvec), GFP_KERNEL);
        if (NULL == pSocket->hMsgData)
        {
            nRet = -ENOMEM;
            break;
        }

        pSocket->hSockAddrIn = (struct sockaddr_in*)kzalloc(sizeof(struct sockaddr_in), GFP_KERNEL);
        if (NULL == pSocket->hSockAddrIn)
        {
            nRet = -ENOMEM;
            break;
        }

        pSocket->hSockAddrInServer = (struct sockaddr_in*)kzalloc(sizeof(struct sockaddr_in), GFP_KERNEL);
        if (NULL == pSocket->hSockAddrInServer)
        {
            nRet = -ENOMEM;
            break;
        }

        nRet = sock_create_kern(&init_net, AF_INET, SOCK_DGRAM, IPPROTO_UDP, (struct socket**)&pSocket->hSocket);
    } while (false);
    

    if (0 > nRet)
    {
        if (pSocket->hMsgData)
        {
            kfree(pSocket->hMsgData);
            pSocket->hMsgData = NULL;
        }

        if (pSocket->hMsgHdr)
        {
            kfree(pSocket->hMsgHdr);
            pSocket->hMsgHdr = NULL;
        }

        if (pSocket->hSockAddrIn)
        {
            kfree(pSocket->hSockAddrIn);
            pSocket->hSockAddrIn = NULL;
        }

        if (pSocket->hSockAddrInServer)
        {
            kfree(pSocket->hSockAddrInServer);
            pSocket->hSockAddrInServer = NULL;
        }
        return nRet;
    }

    return MV_OK;
}

int MV_SocketBind(MV_SOCKET* pSocket)
{
    struct sockaddr_in*         pstSockaddrIn = NULL;
    struct sockaddr_in*         pstSockaddrInServer = NULL;
    if (NULL == pSocket)
    {
        return -EINVAL;
    }

    if (NULL == pSocket->hSockAddrIn)
    {
        return -EINVAL;
    }

    if (NULL == pSocket->hSocket)
    {
        return -EINVAL;
    }

    if (NULL == pSocket->hSockAddrInServer)
    {
        return -EINVAL;
    }

    pstSockaddrIn = (struct sockaddr_in*)pSocket->hSockAddrIn;
    memset(pstSockaddrIn, 0, sizeof(struct sockaddr_in));
    pstSockaddrIn->sin_family = AF_INET;
    pstSockaddrIn->sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    pstSockaddrIn->sin_port = 0;

    pstSockaddrInServer = (struct sockaddr_in*)pSocket->hSockAddrInServer;
    pstSockaddrInServer->sin_family = AF_INET;
    pstSockaddrInServer->sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    pstSockaddrInServer->sin_port = htons(12355);

    int nRet = kernel_bind(
            (struct socket*)pSocket->hSocket,
            (struct sockaddr*)pstSockaddrIn,
            sizeof(struct sockaddr));
    if (0 > nRet)
    {
        return nRet;
    }
    else
    {
        return MV_OK;
    }
    
}

int MV_SocketSendto(MV_SOCKET* pSocket, void* pData, int nDataLen)
{
    struct kvec*            pstKvec = NULL;
    struct msghdr*          pstMsgHdr = NULL;
    if (NULL == pSocket)
    {
        return -EINVAL;
    }

    if (NULL == pSocket->hSocket)
    {
        return -EINVAL;
    }

    if (NULL == pSocket->hMsgHdr)
    {
        return -EINVAL;
    }

    if (NULL == pSocket->hMsgData)
    {
        return -EINVAL;
    }

    if (NULL == pSocket->hSockAddrInServer)
    {
        return -EINVAL;
    }

    if (NULL == pData)
    {
        return -EINVAL;
    }

    pstKvec = pSocket->hMsgData;
    pstMsgHdr = pSocket->hMsgHdr;

    pstMsgHdr->msg_name = (struct sockaddr *)pSocket->hSockAddrInServer;
    pstMsgHdr->msg_namelen = sizeof(struct sockaddr);

    pstMsgHdr->msg_control = NULL;
    pstMsgHdr->msg_controllen = 0;

    pstMsgHdr->msg_flags = MSG_DONTWAIT;

    pstKvec->iov_base = pData;
    pstKvec->iov_len = nDataLen;

    int nRet = kernel_sendmsg(
                        pSocket->hSocket,
                        pstMsgHdr,
                        pstKvec,
                        1,
                        nDataLen);
    return nRet;
}

int MV_SocketRelease(MV_SOCKET* pSocket)
{
    if (NULL == pSocket)
    {
        return -EINVAL;
    }

    if (pSocket->hSocket)
    {
        sock_release((struct socket*)pSocket->hSocket);
    }
    
    if (pSocket->hMsgData)
    {
        kfree(pSocket->hMsgData);
        pSocket->hMsgData = NULL;
    }

    if (pSocket->hMsgHdr)
    {
        kfree(pSocket->hMsgHdr);
        pSocket->hMsgHdr = NULL;
    }

    if (pSocket->hSockAddrIn)
    {
        kfree(pSocket->hSockAddrIn);
        pSocket->hSockAddrIn = NULL;
    }

    if (pSocket->hSockAddrInServer)
    {
        kfree(pSocket->hSockAddrInServer);
        pSocket->hSockAddrInServer = NULL;
    }

    return MV_OK;
}