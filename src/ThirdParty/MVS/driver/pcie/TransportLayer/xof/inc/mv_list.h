
#ifndef     __MV_LIST_H__
#define     __MV_LIST_H__

#ifdef __cplusplus
extern "C"
{
#endif 

//typedef struct list_head MV_LIST_HEAD;

#define MV_CONTAINING_RECORD(address, type, field) \
    ((type *)((char*)(address) - (char*)(&((type *)0)->field)))

typedef struct _MV_LIST_HEAD_
{
    struct _MV_LIST_HEAD_ *next, *prev;
}MV_LIST_HEAD;

#define MV_ListEntry(ptr, type, member) \
   MV_CONTAINING_RECORD(ptr, type, member)

#define MV_ListForEach(pos, head) \
    for (pos = (head)->next; pos->next, pos != (head); \
    pos = pos->next)

#define MV_ListFrist_Entry(ptr, type, member) \
            MV_ListEntry((ptr)->next, type, member)

#define MV_List_Last_Entry(ptr, type, member) \
             MV_ListEntry((ptr)->prev, type, member)

#define MV_List_Next_Entry(pos, member) \
             MV_ListEntry((pos)->member.next, typeof(*(pos)), member)

/* list_for_each_entry  -       iterate over list of given type
 * @pos:        the type * to use as a loop cursor.
 * @head:       the head for your list.
 * @member:     the name of the list_head within the struct.
 */
#define MV_ListForEachEntry(pos, head, member)                          \
         for (pos = MV_ListFrist_Entry(head, typeof(*pos), member);        \
              &pos->member != (head);                                    \
              pos = MV_List_Next_Entry(pos, member))

/**
 * list_for_each_entry_safe - iterate over list of given type safe against removal of list entry
 * @pos:	the type * to use as a loop cursor.
 * @n:		another type * to use as temporary storage
 * @head:	the head for your list.
 * @member:	the name of the list_head within the struct.
 */
#define MV_ListForEachEntrySafe(pos, n, head, member)			\
    for (pos = MV_ListFrist_Entry(head, typeof(*pos), member),	\
        n = MV_List_Next_Entry(pos, member);			\
         &pos->member != (head); 					\
         pos = n, n = MV_List_Next_Entry(n, member))


#define MV_LIST_HEAD_INIT(name) { &(name), &(name) }
//括号内参数为地址/指针

#define MV_LIST_HEAD(name) \
	struct list_head name = MV_LIST_HEAD_INIT(name)
//LIST_HEAD(name) 即为创建一个名为‘name’的list_head结构体，即链表头,pre和next指针都指向自己

static inline void MV_ListInit(MV_LIST_HEAD *head)
{
    head->next = head;
    head->prev = head;
}

static inline void __mv_list_add(MV_LIST_HEAD *new,
    MV_LIST_HEAD *prev,
    MV_LIST_HEAD *next)
{
    next->prev = new;
    new->next = next;
    new->prev = prev;
    prev->next = new;
}

static inline void MV_ListAdd(MV_LIST_HEAD *new, MV_LIST_HEAD *head)
{
    __mv_list_add(new, head, head->next);
}

static inline void MV_ListAddTail(MV_LIST_HEAD *new, MV_LIST_HEAD *head)
{
    __mv_list_add(new, head->prev, head);
}

static inline void __mv_list_del(MV_LIST_HEAD * prev, MV_LIST_HEAD* next)
{
    next->prev = prev;
    prev->next = next;
}

static inline void MV_ListDel(MV_LIST_HEAD *entry)
{
    __mv_list_del(entry->prev, entry->next);
}

static inline void MV_ListDelInit(MV_LIST_HEAD *entry)
{
    __mv_list_del(entry->prev, entry->next);
    MV_ListInit(entry);
}

static inline void MV_ListMove(MV_LIST_HEAD *list, MV_LIST_HEAD *head)
{
    __mv_list_del(list->prev, list->next);
    MV_ListAdd(list, head);
}
static inline void MV_ListMoveTail(MV_LIST_HEAD *list, MV_LIST_HEAD *head)
{
    __mv_list_del(list->prev, list->next);
    MV_ListAddTail(list, head);
}

static inline void MV_AppendTailList(
               MV_LIST_HEAD* ListToAppend,
               MV_LIST_HEAD* ListHead
               )
{
    MV_LIST_HEAD* ListEnd = ListHead->prev;

    ListHead->prev->next = ListToAppend;
    ListHead->prev = ListToAppend->prev;
    ListToAppend->prev->next = ListHead;
    ListToAppend->prev = ListEnd;
} 

static inline int MV_ListEmpty(const MV_LIST_HEAD *head)
{
    return head->next == head;
}

static inline MV_LIST_HEAD* MV_ListNext(MV_LIST_HEAD *head)
{
    return head->next;
}

static inline MV_LIST_HEAD* MV_ListPrev(MV_LIST_HEAD *head)
{
    return head->prev;
}

//extern int MV_ListRemoveEntry(MV_LIST_HEAD *entry);

#ifdef __cplusplus
}
#endif

#endif  // __MV_LIST_H__
