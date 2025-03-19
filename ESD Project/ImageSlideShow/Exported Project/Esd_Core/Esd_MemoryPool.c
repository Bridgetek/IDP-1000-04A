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

#include "Esd_MemoryPool.h"

#ifdef ESD_MEMORYPOOL_ALLOCATOR

#include "EVE_Hal.h"

#define MP_CHUNKHEADER sizeof(struct _mp_chunk)
#define MP_CHUNKEND sizeof(struct _mp_chunk *)
// 8bit align
#define MP_ALIGN_SIZE(_n) (_n + sizeof(long) - ((sizeof(long) - 1) & _n))
#define MP_INIT_MEMORY_STRUCT(mm, mem_sz)         \
	do                                            \
	{                                             \
		mm->mem_pool_size = mem_sz;               \
		mm->alloc_mem = 0;                        \
		mm->alloc_prog_mem = 0;                   \
		mm->free_list = (Esd_MpChunk *)mm->start; \
		mm->free_list->is_free = 1;               \
		mm->free_list->alloc_mem = mem_sz;        \
		mm->free_list->prev = NULL;               \
		mm->free_list->next = NULL;               \
		mm->alloc_list = NULL;                    \
	} while (0)

#define MP_DLINKLIST_INS_FRT(head, x) \
	do                                \
	{                                 \
		x->prev = NULL;               \
		x->next = head;               \
		if (head)                     \
			head->prev = x;           \
		head = x;                     \
	} while (0)

#define MP_DLINKLIST_DEL(head, x)        \
	do                                   \
	{                                    \
		if (!x->prev)                    \
		{                                \
			head = x->next;              \
			if (x->next)                 \
				x->next->prev = NULL;    \
		}                                \
		else                             \
		{                                \
			x->prev->next = x->next;     \
			if (x->next)                 \
				x->next->prev = x->prev; \
		}                                \
	} while (0)

#if 0
void esd_get_memory_list_count(Esd_MemoryPool *mp, mem_size_t *mlist_len)
{
	mem_size_t mlist_l = 0;
	Esd_MpMemory *mm = mp->mlist;
	while (mm)
	{
		mlist_l++;
		mm = mm->next;
	}
	*mlist_len = mlist_l;
}

void esd_get_memory_info(Esd_MemoryPool *mp,
    Esd_MpMemory *mm,
    mem_size_t *free_list_len,
    mem_size_t *alloc_list_len)
{
	mem_size_t free_l = 0, alloc_l = 0;
	Esd_MpChunk *p = mm->free_list;
	while (p)
	{
		free_l++;
		p = p->next;
	}

	p = mm->alloc_list;
	while (p)
	{
		alloc_l++;
		p = p->next;
	}
	*free_list_len = free_l;
	*alloc_list_len = alloc_l;
}

int esd_get_memory_id(Esd_MpMemory *mm)
{
	return mm->id;
}
#endif

static Esd_MpMemory *extend_memory_list(Esd_MemoryPool *mp, mem_size_t new_mem_sz)
{
	char *s = (char *)malloc(sizeof(Esd_MpMemory) + new_mem_sz * sizeof(char));
	if (!s)
		return NULL;

	Esd_MpMemory *mm = (Esd_MpMemory *)s;
	mm->start = s + sizeof(Esd_MpMemory);

	MP_INIT_MEMORY_STRUCT(mm, new_mem_sz);
	mm->id = mp->last_id++;
	mm->next = mp->mlist;
	mp->mlist = mm;
	eve_printf_debug("[Esd MemoryPool] extended new mem_pool_size %d\n", (int)mp->mem_pool_size);
	eve_printf_debug("[Esd MemoryPool] extended new total_mem_pool_size %d\n", (int)mp->total_mem_pool_size);
	return mm;
}

static Esd_MpMemory *find_memory_list(Esd_MemoryPool *mp, void *p)
{
	Esd_MpMemory *tmp = mp->mlist;
	while (tmp)
	{
		if (tmp->start <= (char *)p && tmp->start + mp->mem_pool_size > (char *)p)
			break;
		tmp = tmp->next;
	}

	return tmp;
}

static int merge_free_chunk(Esd_MemoryPool *mp, Esd_MpMemory *mm, Esd_MpChunk *c)
{
	Esd_MpChunk *p0 = c, *p1 = c;
	while (p0->is_free)
	{
		p1 = p0;
		if ((char *)p0 - MP_CHUNKEND - MP_CHUNKHEADER <= mm->start)
			break;
		p0 = *(Esd_MpChunk **)((char *)p0 - MP_CHUNKEND);
	}

	p0 = (Esd_MpChunk *)((char *)p1 + p1->alloc_mem);
	while ((char *)p0 < mm->start + mp->mem_pool_size && p0->is_free)
	{
		MP_DLINKLIST_DEL(mm->free_list, p0);
		p1->alloc_mem += p0->alloc_mem;
		p0 = (Esd_MpChunk *)((char *)p0 + p0->alloc_mem);
	}

	*(Esd_MpChunk **)((char *)p1 + p1->alloc_mem - MP_CHUNKEND) = p1;
	return 0;
}

Esd_MemoryPool *Esd_MemoryPool_Init(mem_size_t maxmempoolsize, mem_size_t mempoolsize)
{
	if (mempoolsize > maxmempoolsize)
	{
		// eve_printf("[MemoryPool_Init] MemPool Init ERROR! Mempoolsize is too big!\n");
		eve_printf("[Esd MemoryPool] MemPool Init ERROR! Mempoolsize is too big!\n");
		return NULL;
	}
	Esd_MemoryPool *mp = (Esd_MemoryPool *)malloc(sizeof(Esd_MemoryPool));
	eve_printf_debug("[Esd MemoryPool] assign size %d\n", sizeof(Esd_MemoryPool));
	if (!mp)
	{
		eve_printf("[Esd MemoryPool] MemPool Init ERROR! Assign pointer fail\n");
		return NULL;
	}
	eve_printf_debug("[Esd MemoryPool] address of mp is %p\n", mp);

	mp->last_id = 0;
	if (mempoolsize < maxmempoolsize)
	{
		mp->auto_extend = 1;
		eve_printf_debug("[Esd MemoryPool] auto extend enabled \n");
	}
	mp->max_mem_pool_size = maxmempoolsize;
	mp->total_mem_pool_size = mp->mem_pool_size = mempoolsize;

	char *s = (char *)malloc(sizeof(Esd_MpMemory) + sizeof(char) * mp->mem_pool_size);
	if (!s)
	{
		eve_printf("[Esd MemoryPool] MemPool Init ERROR! Assign s fail\n");
		return NULL;
	}

	mp->mlist = (Esd_MpMemory *)s;
	mp->mlist->start = s + sizeof(Esd_MpMemory);
	MP_INIT_MEMORY_STRUCT(mp->mlist, mp->mem_pool_size);
	mp->mlist->next = NULL;
	mp->mlist->id = mp->last_id++;

	return mp;
}

void *Esd_MemoryPool_Alloc(Esd_MemoryPool *mp, mem_size_t wantsize)
{
	if (wantsize <= 0)
		return NULL;
	mem_size_t total_needed_size = MP_ALIGN_SIZE(wantsize + MP_CHUNKHEADER + MP_CHUNKEND);
	eve_printf_debug("[Esd MemoryPool] total needed size %llu \n", total_needed_size);
	if (total_needed_size > mp->max_mem_pool_size)
	// if (total_needed_size > mp->mem_pool_size)
	{
		eve_printf("[Esd MemoryPool], total needed size %llu bigger than total size %llu\n", total_needed_size, mp->max_mem_pool_size);
		return NULL;
	}

	Esd_MpMemory *mm = NULL, *mm1 = NULL;
	Esd_MpChunk *_free = NULL, *_not_free = NULL;

FIND_FREE_CHUNK:
	mm = mp->mlist;
	while (mm)
	{
		eve_printf_debug_once("[Esd MemoryPool] mem_pool_size %llu, alloc size %llu\n", mp->mem_pool_size, mm->alloc_mem);
		if (mp->mem_pool_size - mm->alloc_mem < total_needed_size)
		{
			mm = mm->next;
			continue;
		}

		_free = mm->free_list;
		_not_free = NULL;

		while (_free)
		{
			eve_printf_debug("[Esd MemoryPool] alloc mem is %llu\n", _free->alloc_mem);
			if (_free->alloc_mem >= total_needed_size)
			{
				// if left size is big enough, divide the left memory
				if (_free->alloc_mem - total_needed_size > MP_CHUNKHEADER + MP_CHUNKEND)
				{
					// divide from the start
					_not_free = _free;

					_free = (Esd_MpChunk *)((char *)_not_free + total_needed_size);
					*_free = *_not_free;
					_free->alloc_mem -= total_needed_size;
					*(Esd_MpChunk **)((char *)_free + _free->alloc_mem - MP_CHUNKEND) = _free;

					// update free_list
					if (!_free->prev)
					{
						mm->free_list = _free;
					}
					else
					{
						_free->prev->next = _free;
					}
					if (_free->next)
						_free->next->prev = _free;

					_not_free->is_free = 0;
					_not_free->alloc_mem = total_needed_size;

					*(Esd_MpChunk **)((char *)_not_free + total_needed_size - MP_CHUNKEND) = _not_free;
				}
				// or alloc all left
				else
				{
					eve_printf_debug("[Esd MemoryPool] alloc all left\n");
					_not_free = _free;
					MP_DLINKLIST_DEL(mm->free_list, _not_free);
					_not_free->is_free = 0;
				}
				MP_DLINKLIST_INS_FRT(mm->alloc_list, _not_free);

				mm->alloc_mem += _not_free->alloc_mem;
				mm->alloc_prog_mem += (_not_free->alloc_mem - MP_CHUNKHEADER - MP_CHUNKEND);

				return (void *)((char *)_not_free + MP_CHUNKHEADER);
			}
			_free = _free->next;
		}

		mm = mm->next;
	}

	if (mp->auto_extend)
	{
		if (mp->total_mem_pool_size > mp->max_mem_pool_size)
		{
			eve_printf("[Esd MemoryPool] ideal size %llu bigger than max size %llu\n", mp->total_mem_pool_size, mp->max_mem_pool_size);
			goto err_out;
		}
		mem_size_t add_mem_sz = mp->max_mem_pool_size - mp->mem_pool_size;
		// mem_size_t add_mem_sz = total_needed_size - mp->mem_pool_size;
		eve_printf("[Esd MemoryPool], trigger auto extended, add mem %llu\n", add_mem_sz);
		add_mem_sz = add_mem_sz >= mp->mem_pool_size ? mp->mem_pool_size : add_mem_sz;
		eve_printf("[Esd MemoryPool], add mem after recaulculation %llu\n", add_mem_sz);
		mm1 = extend_memory_list(mp, add_mem_sz);
		// eve_printf_debug("[Esd MemoryPool],trigger extend memory\n");
		if (!mm1)
		{
			eve_printf("[Esd MemoryPool], extend memory failed, mm1 null\n");
			goto err_out;
		}
		mp->total_mem_pool_size += add_mem_sz;
		mp->mem_pool_size += add_mem_sz;
		eve_printf_debug("[Esd MemoryPool] total mem size after extend %llu\n", mp->total_mem_pool_size);
		eve_printf_debug("[Esd MemoryPool] mem_pool_size after extend %llu\n", mp->mem_pool_size);
		goto FIND_FREE_CHUNK;
	}

err_out:
	eve_printf("[Esd MemoryPool] No enough memory! \n");
	return NULL;
}

int Esd_MemoryPool_Free(Esd_MemoryPool *mp, void *p)
{
	if (p == NULL || mp == NULL)
		return 1;
	Esd_MpMemory *mm = mp->mlist;
	if (mp->auto_extend)
		mm = find_memory_list(mp, p);

	Esd_MpChunk *ck = (Esd_MpChunk *)((char *)p - MP_CHUNKHEADER);

	MP_DLINKLIST_DEL(mm->alloc_list, ck);
	MP_DLINKLIST_INS_FRT(mm->free_list, ck);
	ck->is_free = 1;

	mm->alloc_mem -= ck->alloc_mem;
	eve_printf_debug("Free memory %d\n", (int)ck->alloc_mem);
	mm->alloc_prog_mem -= (ck->alloc_mem - MP_CHUNKHEADER - MP_CHUNKEND);

	return merge_free_chunk(mp, mm, ck);
}

Esd_MemoryPool *Esd_MemoryPool_Clear(Esd_MemoryPool *mp)
{
	if (!mp)
		return NULL;
	Esd_MpMemory *mm = mp->mlist;
	while (mm)
	{
		MP_INIT_MEMORY_STRUCT(mm, mm->mem_pool_size);
		mm = mm->next;
	}
	return mp;
}

int Esd_MemoryPool_Destroy(Esd_MemoryPool *mp)
{
	if (mp == NULL)
		return 1;

	Esd_MpMemory *mm = mp->mlist, *mm1 = NULL;
	while (mm)
	{
		mm1 = mm;
		mm = mm->next;
		free(mm1);
	}
	free(mp);
	return 0;
}

mem_size_t Esd_MemoryPool_GetTotal(Esd_MemoryPool *mp)
{
	return mp->total_mem_pool_size;
}

mem_size_t Esd_MemoryPool_GetTotalUsed(Esd_MemoryPool *mp)
{

	mem_size_t total_alloc = 0;
	Esd_MpMemory *mm = mp->mlist;
	while (mm)
	{
		total_alloc += mm->alloc_mem;
		mm = mm->next;
	}
	return total_alloc;
}

mem_size_t Esd_MemoryPool_GetTotalProg(Esd_MemoryPool *mp)
{
	mem_size_t total_alloc_prog = 0;
	Esd_MpMemory *mm = mp->mlist;
	while (mm)
	{
		total_alloc_prog += mm->alloc_prog_mem;
		mm = mm->next;
	}
	return total_alloc_prog;
}

/*
float Esd_MemoryPool_GetFactorUsed(Esd_MemoryPool *mp)
{
    return (float)Esd_MemoryPool_GetTotalUsed(mp) / Esd_MemoryPool_GetTotal(mp);
}

float Esd_MemoryPool_GetFactorProg(Esd_MemoryPool *mp)
{
    return (float)Esd_MemoryPool_GetTotalProg(mp) / Esd_MemoryPool_GetTotal(mp);
}
*/

#undef MP_CHUNKHEADER
#undef MP_CHUNKEND
#undef MP_ALIGN_SIZE
#undef MP_INIT_MEMORY_STRUCT
#undef MP_DLINKLIST_INS_FRT
#undef MP_DLINKLIST_DEL

#endif /* #ifdef ESD_MEMORYPOOL_ALLOCATOR*/

/* end of file */
