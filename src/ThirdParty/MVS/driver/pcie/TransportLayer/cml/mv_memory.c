
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <asm/page.h>
#include <linux/mm.h>
#include <linux/gfp.h>

#include "mv_memory.h"
#include "MvErrorDefine.h"

/*******************************************************************************
@   ����ԭ�ͣ�void* MV_KMalloc(int aSize)
@   �������ܣ�kmalloc������װ
@   ����aSize�� ��Ҫkmalloc�Ĵ�С
@   ����ֵ���ɹ������ڴ��׵�ַ��ʧ��NULL
*******************************************************************************/
void* MV_KMalloc(int aSize)
{
    return kmalloc(aSize, GFP_KERNEL);
}

/*******************************************************************************
@   ����ԭ�ͣ�void* MV_KMallocAtomic(int aSize)
@   �������ܣ�kmalloc������װ
@   ����aSize�� ��Ҫkmalloc�Ĵ�С
@   ����ֵ���ɹ������ڴ��׵�ַ��ʧ��NULL
*******************************************************************************/
void* MV_KMallocAtomic(int aSize)
{
    return kmalloc(aSize, GFP_ATOMIC);
}


/*******************************************************************************
@   ����ԭ�ͣ�void MV_KFree(void* aAddress)
@   �������ܣ�kfree������װ
@   ����aAddress�� ��Ҫkfree���ڴ��׵�ַ
@   ����ֵ����
*******************************************************************************/
void MV_KFree(void* aAddress)
{
    if(NULL == aAddress)
    {
        return;
    }
    kfree(aAddress);
}

/*******************************************************************************
@   ����ԭ�ͣ�void* MV_VMalloc(int aSize)
@   �������ܣ�vmalloc������װ
@   ����aSize�� ��Ҫvmalloc�Ĵ�С
@   ����ֵ���ɹ������ڴ��׵�ַ��ʧ��NULL
*******************************************************************************/
void* MV_VMalloc(int aSize)
{

    return vmalloc(aSize);
}

/*******************************************************************************
@   ����ԭ�ͣ�void MV_VFree(void* aAddress)
@   �������ܣ�vfree������װ
@   ����aAddress�� ��Ҫvfree���ڴ��׵�ַ
@   ����ֵ����
*******************************************************************************/
void MV_VFree(void* aAddress)
{
    if(NULL == aAddress)
    {
        return;
    }
    vfree(aAddress);
}

/*******************************************************************************
@   ����ԭ�ͣ�void MV_Memset(void* aAddress, unsigned int aValue, int aSize)
@   �������ܣ�memset������װ
@   ����aAddress: ��Ҫmemset���ڴ��׵�ַ
@   ����aValue: ��Ҫmemset��ֵ
@   ����aSize: ��Ҫmemset�Ĵ�С
@   ����ֵ����
*******************************************************************************/
void MV_Memset(void* aAddress, unsigned int aValue, int aSize)
{
    if(NULL == aAddress)
    {
        return;
    }
    memset(aAddress, aValue, aSize);
}

/*******************************************************************************
@   ����ԭ�ͣ�void MV_Memcpy(void* aDestination, const void* aSource, int aSize)
@   �������ܣ�memcpy������װ
@   ����aDestination: ����Ŀ�ĵ�ַ
@   ����aSource: ����ԭ��ַ
@   ����aSize: �����ֽ���
@   ����ֵ����
*******************************************************************************/
void MV_Memcpy(void* aDestination, const void* aSource, int aSize)
{
    if(NULL == aDestination || NULL == aSource)
    {
        return;
    }
    memcpy(aDestination, aSource, aSize);
}

int MV_Strlen(const char *str)
{
    if (NULL == str)
    {
        return 0;
    }
    
    return strlen(str);
}

/*******************************************************************************
@   ����ԭ�ͣ�unsigned int MV_MemcpyToUser(void* aTo, const void* aFrom, int aSize)
@   �������ܣ�copy_to_user������װ
@   ����aTo: �������û����Ŀ�ĵ�ַ
@   ����aFrom: ����ԭ��ַ
@   ����aSize: �����ֽ���
@   ����ֵ���ɹ�����0 ʧ�ܷ���MV_E_PARAMETER
*******************************************************************************/
unsigned int MV_MemcpyToUser(void* aTo, const void* aFrom, int aSize)
{
    if(NULL == aTo || NULL == aFrom)
    {
        return MV_E_PARAMETER;
    }
    return copy_to_user(aTo, aFrom, (unsigned long)aSize);
}

/*******************************************************************************
@   ����ԭ�ͣ�unsigned int MV_MemcpyFromUser(void* aTo, const void* aFrom, int aSize )
@   �������ܣ�copy_from_user������װ
@   ����aTo: �û��㿽�����ں˵�Ŀ�ĵ�ַ
@   ����aFrom: �û��㿽��ԭ��ַ
@   ����aSize: �����ֽ���
@   ����ֵ���ɹ�����0 ʧ�ܷ���MV_E_PARAMETER
*******************************************************************************/
unsigned int MV_MemcpyFromUser(void* aTo, const void* aFrom, int aSize )
{
    aFrom = aFrom;
    aSize = aSize;
    //printk("aTo[%p] aFrom[%p] size[%d]\n", aTo, aFrom, aSize);
    if(NULL == aTo || NULL == aFrom)
    {
        return MV_E_PARAMETER;
    }

    return copy_from_user(aTo, aFrom, ( unsigned long )aSize);
}

unsigned int test(MV_TEST myTest, int e)
{
//     a = a;
//     b = b;
//     int *p;
//     printk("p:%d\n", sizeof(p));
//     printk("add2 a:%p b:%p\n", &a, &b);
//     printk("a[%d], b[%d]\n", a, *(&b+8));
    printk("myTest[%d][%d]\n", myTest.b/*, myTest.b*/, e);
    return 0;
}


int MV_MallocPages(MV_MEMORYPAGELIST* pPageList, unsigned int nSize)
{
    unsigned int nPageCount = PAGE_ALIGN(nSize) >> PAGE_SHIFT;
    struct page** lPageList     = NULL;
    int i = 0;
    const gfp_t alloc_mask = GFP_KERNEL | __GFP_NOWARN;
    const gfp_t highmem_mask = __GFP_HIGHMEM;
    struct page* pPage = NULL;

    lPageList = (struct page**)kmalloc( sizeof( struct page* ) * nPageCount, GFP_KERNEL );
    if( !lPageList )
    {
        printk("Driver: kmalloc PageList failed\n");
        return -1;
    }

    for (i = 0; i < nPageCount; i++)
    {
        pPage = alloc_page(alloc_mask | highmem_mask);
        if (NULL == pPage)
        {
            printk("Driver: alloc_page failed, i = [%d]\n", i);
            break;
        }
        lPageList[i] = pPage;
    }

    if (i != nPageCount)
    {
        for (; i >=0; i--)
        {
            __free_page(lPageList[i]);
            lPageList[i] = NULL;
        }

        kfree(lPageList);

        return -1;
    }

    pPageList->PageList = (void**)lPageList;
    pPageList->PageCount = nPageCount;

    return 0;
}

unsigned int MV_GetVmaSize(void* vma)
{
    struct vm_area_struct * pVma = NULL;
    if (NULL == vma)
    {
        return 0;
    }

    pVma = (struct vm_area_struct *)vma;

    return pVma->vm_end - pVma->vm_start;
}

int MV_FreePages(MV_MEMORYPAGELIST* pPageList)
{
    int i = 0;
    if (NULL == pPageList)
    {
        return -1;
    }

    if (NULL == pPageList->PageList)
    {
        return -1;
    }

    for (i = 0; i < pPageList->PageCount; i++)
    {
        if (pPageList->PageList[i])
        {
            __free_page(pPageList->PageList[i]);
            pPageList->PageList[i] = NULL;
        }
    }

    kfree(pPageList->PageList);
    pPageList->PageList = NULL;
    pPageList->PageCount = 0;

    return 0;
}

void* MV_KZalloc(int aSize)
{
    return kzalloc(aSize, GFP_KERNEL);
}

void* MV_KZallocAtomic(int aSize)
{
    return kzalloc(aSize, GFP_ATOMIC);
}