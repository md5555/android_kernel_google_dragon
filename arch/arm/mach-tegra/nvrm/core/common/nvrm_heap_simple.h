/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the NVIDIA Corporation nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef NVRM_HEAP_SIMPLE_H
#define NVRM_HEAP_SIMPLE_H

#include "nvcommon.h"
#include "nvrm_init.h"
#include "nvos.h"


typedef struct NvRmHeapSimpleBlockRec NvRmHeapSimpleBlock;
struct NvRmHeapSimpleBlockRec
{
    NvBool       IsFree;
    NvRmPhysAddr PhysAddr;
    NvU32        size;

    NvU32        NextIndex;

    // debug info
    NvU32        touched;
};


typedef struct NvRmHeapSimpleRec
{
    NvRmPhysAddr          base;
    NvU32                 size;
    NvU32                 ArraySize;

    NvRmHeapSimpleBlock   *RawBlockArray;

    NvU32   BlockIndex;
    NvU32   SpareBlockIndex;

    NvOsMutexHandle       mutex;
} NvRmHeapSimple;

NvError NvRmPrivHeapSimple_HeapAlloc(NvRmPhysAddr Base, NvU32 Size, NvRmHeapSimple *pNewHeap);
void NvRmPrivHeapSimple_HeapFree(NvRmHeapSimple *);

NvError NvRmPrivHeapSimpleAlloc(NvRmHeapSimple *, NvU32 size, NvU32 align, NvRmPhysAddr *paddr);

NvError NvRmPrivHeapSimplePreAlloc(
    NvRmHeapSimple *,
    NvRmPhysAddr Address,
    NvU32 Length);

void NvRmPrivHeapSimpleFree(NvRmHeapSimple *, NvRmPhysAddr paddr);

NvS32 NvRmPrivHeapSimpleMemoryUsed(NvRmHeapSimple* pHeap);

NvS32 NvRmPrivHeapSimpleLargestFreeBlock(NvRmHeapSimple* pHeap);

#endif // INCLUDED_HEAP_H
