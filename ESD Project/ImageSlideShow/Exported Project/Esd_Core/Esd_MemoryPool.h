/**
 * This source code ("the Software") is provided by Bridgetek Pte Ltd
 * ("Bridgetek") subject to the licence terms set out
 *   http://brtchip.com/BRTSourceCodeLicenseAgreement/ ("the Licence Terms").
 * You must read the Licence Terms before downloading or using the Software.
 * By installing or using the Software you agree to the Licence Terms. If you
 * do not agree to the Licence Terms then do not download or use the Software.
 *
 * Without prejudice to the Licence Terms, here is a summary of some of the key
 * terms of the Licence Terms (and in the event of any conflict between this
 * summary and the Licence Terms then the text of the Licence Terms will
 * prevail).
 *
 * The Software is provided "as is".
 * There are no warranties (or similar) in relation to the quality of the
 * Software. You use it at your own risk.
 * The Software should not be used in, or for, any medical device, system or
 * appliance. There are exclusions of Bridgetek liability for certain types of loss
 * such as: special loss or damage; incidental loss or damage; indirect or
 * consequential loss or damage; loss of income; loss of business; loss of
 * profits; loss of revenue; loss of contracts; business interruption; loss of
 * the use of money or anticipated savings; loss of information; loss of
 * opportunity; loss of goodwill or reputation; and/or loss of, damage to or
 * corruption of data.
 * There is a monetary cap on Bridgetek's liability.
 * The Software may have subsequently been amended by another user and then
 * distributed by that other user ("Adapted Software").  If so that user may
 * have additional licence terms that apply to those amendments. However, Bridgetek
 * has no liability in relation to those amendments.
 */

#ifndef ESD_MEMORYPOOL___H
#define ESD_MEMORYPOOL___H

#include "Esd_Base.h"

#ifdef ESD_MEMORYPOOL_ALLOCATOR

#include <stdlib.h>
//#include <string.h>

typedef size_t mem_size_t;
#define KB (mem_size_t)(1 << 10)
#define MB (mem_size_t)(1 << 20)

typedef struct _mp_chunk
{
	mem_size_t alloc_mem;
	struct _mp_chunk *prev;
	struct _mp_chunk *next;
	int is_free;
} Esd_MpChunk;

typedef struct _mp_mem_pool_list
{
	char *start;
	unsigned int id;
	mem_size_t mem_pool_size;
	mem_size_t alloc_mem;
	mem_size_t alloc_prog_mem;
	Esd_MpChunk *free_list;
	Esd_MpChunk *alloc_list;
	struct _mp_mem_pool_list *next;
} Esd_MpMemory;

typedef struct _mp_mem_pool
{
	unsigned int last_id;
	int auto_extend;
	mem_size_t mem_pool_size;
	mem_size_t total_mem_pool_size;
	mem_size_t max_mem_pool_size;
	struct _mp_mem_pool_list *mlist;
} Esd_MemoryPool;

// below 3 are debug functions, get information only
#if 0
void esd_get_memory_list_count(Esd_MemoryPool *mp, mem_size_t *mlist_len);

void esd_get_memory_info(Esd_MemoryPool *mp,
    Esd_MpMemory *mm,
    mem_size_t *free_list_len,
    mem_size_t *alloc_list_len);

int esd_get_memory_id(Esd_MpMemory *mm);
#endif

Esd_MemoryPool *Esd_MemoryPool_Init(mem_size_t maxmempoolsize, mem_size_t mempoolsize);
void *Esd_MemoryPool_Alloc(Esd_MemoryPool *mp, mem_size_t wantsize);
int Esd_MemoryPool_Free(Esd_MemoryPool *mp, void *p);
Esd_MemoryPool *Esd_MemoryPool_Clear(Esd_MemoryPool *mp);
int Esd_MemoryPool_Destroy(Esd_MemoryPool *mp);

mem_size_t Esd_MemoryPool_GetTotal(Esd_MemoryPool *mp);

mem_size_t Esd_MemoryPool_GetTotalUsed(Esd_MemoryPool *mp);
// float Esd_MemoryPool_GetFactorUsed(Esd_MemoryPool *mp);

mem_size_t Esd_MemoryPool_GetTotalProg(Esd_MemoryPool *mp);
// float Esd_MemoryPool_GetFactorProg(Esd_MemoryPool *mp);

#endif /* #ifdef ESD_MEMORYPOOL_ALLOCATOR*/

#endif /* #ifndef ESD_MEMORYPOOL__H */

/* end of file */