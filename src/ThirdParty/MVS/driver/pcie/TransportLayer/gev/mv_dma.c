#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include "linux/dmaengine.h"
#include <asm/uaccess.h> 

#include "mv_dma.h"

#include "MvErrorDefine.h"

int MV_DmaInit(PMV_DIR_MEM_ACCESS pstDma)
{
    struct sg_table*           pstSgStable = NULL;
    int                         nRet = MV_OK;

    if (NULL == pstDma)
    {
        return MV_E_PARAMETER;
    }

    pstSgStable = (struct sg_table*)kzalloc(sizeof(struct sg_table), GFP_KERNEL);

    if (NULL == pstSgStable)
    {
        return MV_E_RESOURCE;
    }

    pstDma->pSgDma = pstSgStable;

    return nRet;
}

int MV_DmaFree(PMV_DIR_MEM_ACCESS pstDma)
{
    struct sg_table*           pstSgStable = NULL;
    int                         nRet = MV_OK;

    if (NULL == pstDma)
    {
        return MV_E_PARAMETER;
    }

    pstSgStable = pstDma->pSgDma;
    if (NULL == pstSgStable)
    {
        return MV_E_PARAMETER;
    }
    
    kfree(pstSgStable);
    pstSgStable = NULL;

    return nRet;
}

int MV_DmaSgAlloc (PMV_DIR_MEM_ACCESS pstDma, int nTableNum)
{
    struct sg_table*           pstSgStable = NULL;
    int                         nRet = MV_OK;

    if (NULL == pstDma)
    {
        return MV_E_PARAMETER;
    }

    if (0 >= nTableNum)
    {
        return MV_E_PARAMETER;
    }

    pstSgStable = pstDma->pSgDma;
    if (NULL == pstSgStable)
    {
        return MV_E_PARAMETER;
    }

    if (sg_alloc_table(pstSgStable, nTableNum, GFP_KERNEL))
    {
        return MV_E_RESOURCE;
    }

    pstDma->pSglList = pstSgStable->sgl;
    pstDma->nNents = pstSgStable->nents;
    pstDma->nOrigNents = pstSgStable->orig_nents;

    //printk(KERN_INFO"[%s:%d]nNents[%d], orig_nents[%d], nTableNum[%d]", __FUNCTION__, __LINE__, pstSgStable->nents, pstSgStable->orig_nents, nTableNum);

    return MV_OK;
}

int MV_DmaSgFreeTable (PMV_DIR_MEM_ACCESS pstDma)
{
    struct sg_table*           pstSgStable = NULL;
    int                         nRet = MV_OK;

    if (NULL == pstDma)
    {
        return MV_E_PARAMETER;
    }

    pstSgStable = pstDma->pSgDma;
    if (NULL == pstSgStable)
    {
        return MV_E_PARAMETER;
    }

    sg_free_table(pstSgStable);

    return MV_OK;
}

int MV_DmaSgMap(void* pDev, PMV_DIR_MEM_ACCESS pstDma, int nTableNum, int* nRetMapNum)
{
    struct sg_table*           pstSgStable = NULL;
    int                        nRet = MV_OK;

    if (NULL == pstDma)
    {
        return MV_E_PARAMETER;
    }

    if (NULL == pDev)
    {
        return MV_E_PARAMETER;
    }

    pstSgStable = pstDma->pSgDma;
    if (NULL == pstSgStable)
    {
        return MV_E_PARAMETER;
    }

    // printk(KERN_ERR"[%s:%d]nTableNum[%d]", __FILE__, __LINE__, nTableNum);

    *nRetMapNum = dma_map_sg(pDev, pstSgStable->sgl, nTableNum, DMA_FROM_DEVICE);

    // printk(KERN_ERR"[%s:%d]*nRetMapNum[%d]", __FILE__, __LINE__, *nRetMapNum);


    return nRet;
}

int MV_DmaSgUnMap(void* pDev, PMV_DIR_MEM_ACCESS pstDma, int nTableNum)
{
    struct sg_table*           pstSgStable = NULL;
    int                         nRet = MV_OK;

    if (NULL == pstDma)
    {
        return MV_E_PARAMETER;
    }

    if (NULL == pDev)
    {
        return MV_E_PARAMETER;
    }
    
    if (nTableNum <= 0)
    {
        return MV_E_PARAMETER;
    }
    

    pstSgStable = pstDma->pSgDma;
    if (NULL == pstSgStable)
    {
        return MV_E_PARAMETER;
    }
    dma_unmap_sg(pDev, pstSgStable->sgl, nTableNum, DMA_FROM_DEVICE);

    return MV_OK;
}

int MV_DmaSgSetPage(void *pSg, void* pPage, unsigned int nLen, unsigned int nOffset)
{
    int nRet = MV_OK;

    if (NULL == pSg)
    {
        return MV_E_PARAMETER;
    }
    
    if (NULL == pPage)
    {
        return MV_E_PARAMETER;
    }
    
    // printk(KERN_ERR"[%s:%d] nLen[%d], nOffset[%d]", __FILE__, __LINE__, nLen, nOffset);

    sg_set_page((struct scatterlist *)pSg, (struct page *)pPage, nLen, nOffset);

    return nRet;
}

void* MV_DmaSgGetSglList(PMV_DIR_MEM_ACCESS pstDma)
{
    if (NULL == pstDma)
    {
        return NULL;
    }

    return pstDma->pSglList;

}

int MV_DmaSgGetsgNents(PMV_DIR_MEM_ACCESS pstDma)
{
    if (NULL == pstDma)
    {
        return 0;
    }

    return pstDma->nNents;
}

int MV_DmaSgGetSgOrigNents(PMV_DIR_MEM_ACCESS pstDma)
{
    if (NULL == pstDma)
    {
        return 0;
    }

    return pstDma->nOrigNents;
}

int MV_DmaSgGetSclistLength(void* pSclist)
{
    if (NULL == pSclist)
    {
        return MV_E_PARAMETER;
    }
    
    struct scatterlist* pstScatterlist = pSclist;

    return sg_dma_len(pstScatterlist);
}

unsigned long MV_DmaSgGetSclistAddr(void* pSclist)
{
    if (NULL == pSclist)
    {
        return 0;
    }
    
    struct scatterlist* pstScatterlist = pSclist;

    return (unsigned long)sg_dma_address(pstScatterlist);
}


int MV_DmaSgSetSgNents(PMV_DIR_MEM_ACCESS pstDma, int nNents)
{
    int nRet = MV_OK;
    if (NULL == pstDma || 0 >= nNents)
    {
        return MV_E_PARAMETER;
    }

    pstDma->nNents = nNents;
    return nRet;
}

int MV_DmaSgSetSclistLength(void* pSclist, int nSgElemLen)
{
    int nRet = MV_OK;
    if (NULL == pSclist || 0 > nSgElemLen)
    {
        return MV_E_PARAMETER;
    }
    
    struct scatterlist* pstScatterlist = pSclist;

    sg_dma_len(pstScatterlist) = nSgElemLen;
    return nRet;
}

int MV_DmaSgSetSclistAddrOffset(void* pSclist, unsigned long nAddr)
{
    int nRet = MV_OK;
    if (NULL == pSclist)
    {
        return MV_E_PARAMETER;
    }
    
    struct scatterlist* pstScatterlist = pSclist;

    sg_dma_address(pstScatterlist) = nAddr;
    return nRet;
}

struct scatterlist * MV_DmaSgGetNextSgElem(void* pSclist)
{
    if(NULL == pSclist)
    {
        return NULL;
    }
    struct scatterlist* pstScatterlist = pSclist;

    return sg_next(pstScatterlist);
}

