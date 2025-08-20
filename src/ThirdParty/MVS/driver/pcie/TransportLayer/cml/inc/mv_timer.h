
#ifndef __TIMER_H__
#define __TIMER_H__

typedef struct
{
    void* pTimerList;
}MV_TIMER_LIST;


extern int MV_InitTimerList(MV_TIMER_LIST* timer, void (*function)(struct timer_list*), void* pData);
extern int MV_AddTimerList(MV_TIMER_LIST* timer);

extern int MV_DelTimerList(MV_TIMER_LIST* timer);
extern int MV_ModTimerList(MV_TIMER_LIST* timer);

extern int MV_GetTimerSize(MV_TIMER_LIST* timer);

extern int MV_SetTimerJiffies(MV_TIMER_LIST* timer);

#endif	//__TIMER_H__