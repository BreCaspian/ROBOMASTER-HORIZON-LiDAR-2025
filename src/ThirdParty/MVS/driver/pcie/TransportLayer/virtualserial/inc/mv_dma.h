#ifndef     __MV_DMA_H__
#define     __MV_DMA_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * Loop over each sg element, following the pointer to a new list if necessary
 */
#define MV_ForEachSg(sglist, sg, nr, __i)	\
	for (__i = 0, sg = (sglist); __i < (nr); __i++, sg = sg_next(sg))

typedef struct _MV_DIR_MEM_ACCESS_
{
    void*           pSgDma;
    void*           pSglList;
    unsigned int    nNents;		/* number of mapped entries */
    unsigned int    nOrigNents;	/* original size of list */
}MV_DIR_MEM_ACCESS, *PMV_DIR_MEM_ACCESS;

extern int MV_DmaInit(PMV_DIR_MEM_ACCESS pstDma);

extern int MV_DmaFree(PMV_DIR_MEM_ACCESS pstDma);

extern int MV_DmaSgMap(void* pPciDevice, PMV_DIR_MEM_ACCESS pstDma, int nTableNum, int* nRetMapNum);

extern int MV_DmaSgUnMap(void* pPciDevice, PMV_DIR_MEM_ACCESS pstDma, int nTableNum);

extern int MV_DmaSgAlloc (PMV_DIR_MEM_ACCESS pstDma, int nTableNum);

extern int MV_DmaSgFreeTable (PMV_DIR_MEM_ACCESS pstDma);

extern int MV_DmaSgSetPage(void *pSg, void* pPage, unsigned int nLen, unsigned int nOffset);

extern void* MV_DmaSgGetSglList(PMV_DIR_MEM_ACCESS pstDma);

extern int MV_DmaSgGetsgNents(PMV_DIR_MEM_ACCESS pstDma);

extern int MV_DmaSgGetSgOrigNents(PMV_DIR_MEM_ACCESS pstDma);

extern int MV_DmaSgGetSclistLength(void* pSclist);

extern unsigned long MV_DmaSgGetSclistAddr(void* pSclist);

#ifdef __cplusplus
}
#endif

#endif  // __MV_DMA_H__
