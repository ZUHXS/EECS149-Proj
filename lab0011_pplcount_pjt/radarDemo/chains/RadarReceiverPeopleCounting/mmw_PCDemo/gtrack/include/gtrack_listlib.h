/**
 *   @file  gtrack_listlib.h
 *
 *   @brief   
 *      Header file for a double linked list 
 *      implementation.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2009 Texas Instruments, Inc.
 *  \par
 */
#ifndef GTRACK_LIST_LIB_H__
#define GTRACK_LIST_LIB_H__

/**
 * @brief 
 *  GTrack ListElement
 *
 * @details
 *  The structure describes a list node which has links to the previous
 *  and next element in the list.
 */
typedef struct GTrack_ListElem_t
{
	uint32_t					data;
	struct GTrack_ListElem_t	*prev;
	struct GTrack_ListElem_t	*next;
} GTrack_ListElem;

/**
 * @brief 
 *  GTrack List Object
 *
 * @details
 *  The structure describes the list object 
 */
typedef struct
{
    uint32_t			count;
    GTrack_ListElem		*begin;
    GTrack_ListElem		*end;
} GTrack_ListObj;

/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/

extern void gtrack_listInit (GTrack_ListObj *list);
extern int32_t gtrack_isListEmpty (GTrack_ListObj *list);
extern void gtrack_listEnqueue (GTrack_ListObj *list, GTrack_ListElem *elem);
extern GTrack_ListElem *gtrack_listDequeue (GTrack_ListObj *list);
extern GTrack_ListElem* gtrack_listGetFirst (GTrack_ListObj *list);
extern GTrack_ListElem* gtrack_listGetNext (GTrack_ListElem *elem);
extern uint32_t gtrack_listGetCount (GTrack_ListObj *list);
extern int32_t gtrack_listRemoveElement (GTrack_ListObj *list, GTrack_ListElem *elem);

#endif /* GTRACK_LIST_LIB_H__ */

