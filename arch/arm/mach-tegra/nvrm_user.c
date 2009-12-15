/*
 * arch/arm/mach-tegra/nvrm_user.c
 *
 * User-land access to NvRm APIs
 *
 * Copyright (c) 2008-2009, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/cpumask.h>
#include <linux/sched.h>
#include <linux/cpu.h>
#include "nvcommon.h"
#include "nvassert.h"
#include "nvos.h"
#include "nvrm_memmgr.h"
#include "nvrm_memmgr_private.h"
#include "nvrm_ioctls.h"
#include "mach/nvrm_linux.h"
#include "linux/nvos_ioctl.h"
#include "nvrm_power_private.h"
#include "nvreftrack.h"

NvError NvRm_Dispatch(void *InBuffer,
                      NvU32 InSize,
                      void *OutBuffer,
                      NvU32 OutSize,
                      NvDispatchCtx* Ctx);

static int nvrm_open(struct inode *inode, struct file *file);
static int nvrm_close(struct inode *inode, struct file *file);
static long nvrm_unlocked_ioctl(struct file *file,
    unsigned int cmd, unsigned long arg);
static int nvrm_mmap(struct file *file, struct vm_area_struct *vma);
extern void reset_cpu(unsigned int cpu, unsigned int reset);

static NvOsThreadHandle s_DfsThread = NULL;
static NvRtHandle s_RtHandle = NULL;

#define DEVICE_NAME "nvrm"

extern int tegra_fb_control(void *in, void *out);

static const struct file_operations nvrm_fops =
{
    .owner = THIS_MODULE,
    .open = nvrm_open,
    .release = nvrm_close,
    .unlocked_ioctl = nvrm_unlocked_ioctl,
    .mmap = nvrm_mmap
};

static struct miscdevice nvrm_dev =
{
    .name = DEVICE_NAME,
    .fops = &nvrm_fops,
    .minor = MISC_DYNAMIC_MINOR,
};

static void MemReadStrided(
    NvRmMemHandle hMem,
    NvU32 Offset,
    NvU32 SrcStride,
    void *pDst,
    NvU32 DstStride,
    NvU32 ElementSize,
    NvU32 Count)
{
    const void *p = (NvU8 *)hMem->VirtualAddress + Offset;
    NvU32 Size = ElementSize + SrcStride * (Count-1);
    NV_ASSERT(Offset + Size <= hMem->size);
    if ((ElementSize == SrcStride) && (ElementSize == DstStride))
    {
        NV_ASSERT_SUCCESS( NvOsCopyOut(pDst, p, Size) );
    }
    else
    {
        while (Count--)
        {
            NV_ASSERT_SUCCESS( NvOsCopyOut(pDst, p, ElementSize) );
            p = (const NvU8 *)p + SrcStride;
            pDst = (NvU8 *)pDst + DstStride;
        }
    }
}

static void MemWriteStrided(
    NvRmMemHandle hMem,
    NvU32 Offset,
    NvU32 DstStride,
    const void *pSrc,
    NvU32 SrcStride,
    NvU32 ElementSize,
    NvU32 Count)
{
    void *p = (NvU8 *)hMem->VirtualAddress + Offset;
    NvU32 Size = ElementSize + DstStride * (Count-1);
    NV_ASSERT(Offset + Size <= hMem->size);
    if ((ElementSize == SrcStride) && (ElementSize == DstStride))
    {
        NV_ASSERT_SUCCESS( NvOsCopyIn(p, pSrc, Size) );
    }
    else
    {
        while (Count--)
        {
            NV_ASSERT_SUCCESS( NvOsCopyIn(p, pSrc, ElementSize) );
            p = (NvU8 *)p + DstStride;
            pSrc = (const NvU8 *)pSrc + SrcStride;
        }
    }
}

static void NvRmDfsThread(void *args)
{
    NvRmDeviceHandle hRm = (NvRmDeviceHandle)args;
    struct cpumask cpu_mask;

    //Ensure that only cpu0 is in the affinity mask    
    cpumask_clear(&cpu_mask);
    cpumask_set_cpu(0, &cpu_mask);
    if (sched_setaffinity(0, &cpu_mask))
    {
        panic("Unable to setaffinity of DFS thread!\n");
    }

    //Confirm that only CPU0 can run this thread
    if (!cpumask_test_cpu(0, &cpu_mask) || cpumask_weight(&cpu_mask) != 1)
    {
        panic("Unable to setaffinity of DFS thread!\n");
    }

    if (NvRmDfsGetState(hRm) > NvRmDfsRunState_Disabled)
    {
        NvRmDfsSetState(hRm, NvRmDfsRunState_ClosedLoop);

        for (;;)
        {
            NvRmPmRequest Request = NvRmPrivPmThread();
            if (Request & NvRmPmRequest_ExitFlag)
            {
                break;
            }
            if (Request & NvRmPmRequest_CpuOnFlag)
            {
#ifdef CONFIG_HOTPLUG_CPU                
                printk("DFS requested CPU ON\n");
                cpu_up(1);
#endif                
            }
            if (Request & NvRmPmRequest_CpuOffFlag)
            {
#ifdef CONFIG_HOTPLUG_CPU                
                printk("DFS requested CPU OFF\n");
                cpu_down(1);
#endif                
            }                                    
        }
    }
}

static void client_detach(NvRtClientHandle client)
{
    if (NvRtUnregisterClient(s_RtHandle, client))
    {
        NvDispatchCtx dctx;
        
        dctx.Rt = s_RtHandle;
        dctx.Client = client;
        dctx.PackageIdx = 0;
    
        for (;;)
        {
            void* ptr = NvRtFreeObjRef(&dctx,
                                       NvRtObjType_NvRm_NvRmMemHandle,
                                       NULL);
            if (!ptr) break;
            NVRT_LEAK("NvRm", "NvRmMemHandle", ptr);
            NvRmMemHandleFree(ptr);        
        }

        NvRtUnregisterClient(s_RtHandle, client);
    }    
}

static int __init nvrm_init( void )
{
    int e = 0;
    NvU32 NumTypes = NvRtObjType_NvRm_Num;
    
    printk("nvrm init\n");

    NV_ASSERT(s_RtHandle == NULL);
    
    if (NvRtCreate(1, &NumTypes, &s_RtHandle) != NvSuccess)
    {
        e = -ENOMEM;
    }

    if (e == 0)
    {
        e = misc_register( &nvrm_dev );
    }

    if( e < 0 )
    {
        if (s_RtHandle)
        {
            NvRtDestroy(s_RtHandle);
            s_RtHandle = NULL;
        }
        
        printk("nvrm failed to open\n");
    }

    return e;
}

static void __exit nvrm_deinit( void )
{
    misc_deregister( &nvrm_dev );
    NvRtDestroy(s_RtHandle);
    s_RtHandle = NULL;
}

int nvrm_open(struct inode *inode, struct file *file)
{
    NvRtClientHandle Client;
    
    if (NvRtRegisterClient(s_RtHandle, &Client) != NvSuccess)
    {
        return -ENOMEM;
    }

    file->private_data = (void*)Client;
    
    return 0;
}

int nvrm_close(struct inode *inode, struct file *file)
{
    client_detach((NvRtClientHandle)file->private_data);
    return 0;
}

long nvrm_unlocked_ioctl(struct file *file,
    unsigned int cmd, unsigned long arg)
{
    NvError err;
    NvOsIoctlParams p;
    NvRmMemHandle hMem;
    NvU32 offset;
    NvU32 size;
    NvU32 *pBufIn;
    NvU32 small_buf[8];
    void *unsafep = 0;
    void *ptr = 0;
    long e;
    NvBool bAlloc = NV_FALSE;

    switch( cmd ) {
    case NvRmIoctls_Generic:
    {
        NvDispatchCtx dctx;

        dctx.Rt         = s_RtHandle;
        dctx.Client     = (NvRtClientHandle)file->private_data;
        dctx.PackageIdx = 0;
    
        err = NvOsCopyIn( &p, (void *)arg, sizeof(p) );
        if( err != NvSuccess )
        {
            printk( "NvRmIoctls_Generic: copy in failed\n" );
            goto fail;
        }

        //printk( "NvRmIoctls_Generic: %d %d %d\n", p.InBufferSize,
        //    p.InOutBufferSize, p.OutBufferSize );

        size = p.InBufferSize + p.InOutBufferSize + p.OutBufferSize;
        if( size <= sizeof(small_buf) )
        {
            ptr = small_buf;
        }
        else
        {
            ptr = NvOsAlloc( size );
            if( !ptr )
            {
                printk( "NvRmIoctls_Generic: alloc failure (%d bytes)\n",
                    size );
                goto fail;
            }
    
            bAlloc = NV_TRUE;
        }

        err = NvOsCopyIn( ptr, p.pBuffer, p.InBufferSize +
            p.InOutBufferSize );
        if( err != NvSuccess )
        {
            printk( "NvRmIoctls_Generic: copy in failure\n" );
            goto fail;
        }

        err = NvRm_Dispatch( ptr, p.InBufferSize + p.InOutBufferSize,
            ((NvU8 *)ptr) + p.InBufferSize, p.InOutBufferSize +
            p.OutBufferSize, &dctx );
        if( err != NvSuccess )
        {
            printk( "NvRmIoctls_Generic: dispatch failure\n" );
            goto fail;
        }

        if( p.InOutBufferSize || p.OutBufferSize )
        {
            err = NvOsCopyOut( ((NvU8 *)((NvOsIoctlParams *)arg)->pBuffer)
                + p.InBufferSize,
                ((NvU8 *)ptr) + p.InBufferSize,
                p.InOutBufferSize + p.OutBufferSize );
            if( err != NvSuccess )
            {
                printk( "NvRmIoctls_Generic: copy out failure\n" );
                goto fail;
            }
        }

        break;
    }
    case NvRmIoctls_NvRmGraphics:
        printk( "NvRmIoctls_NvRmGraphics: not supported\n" );
        goto fail;
    case NvRmIoctls_NvRmFbControl:
        tegra_fb_control(0, 0);
        break;
    case NvRmIoctls_NvRmMemRead:
        err = NvOsCopyIn( &p, (void *)arg, sizeof(p) );
        if( err != NvSuccess )
        {
            printk( "NvRmIoctls_NvRmMemRead: copy in failed\n" );
            goto fail;
        }

        pBufIn = (NvU32 *)small_buf;

        err = NvOsCopyIn( pBufIn, p.pBuffer, 4 * 4 );
        if( err != NvSuccess )
        {
            printk( "NvRmIoctls_NvRmMemRead: copy in failed\n" );
            goto fail;
        }

        hMem = (NvRmMemHandle)((NvU32*)pBufIn)[0];
        offset = ((NvU32*)pBufIn)[1];
        unsafep = (void *)((NvU32*)pBufIn)[2];
        size = ((NvU32*)pBufIn)[3];

        //printk( "NvRmIoctls_NvRmMemRead\n" );

        ptr = (NvU8 *)hMem->VirtualAddress + offset;
        NV_ASSERT(offset + size <= hMem->size);

        err = NvOsCopyOut(unsafep, ptr, size);
        if( err != NvSuccess )
        {
            printk( "NvRmIoctls_NvRmMemRead: copy out failed\n" );
            goto fail;
        }

        break;
    case NvRmIoctls_NvRmMemWrite:
        err = NvOsCopyIn( &p, (void *)arg, sizeof(p) );
        if( err != NvSuccess )
        {
            printk( "NvRmIoctls_NvRmMemWrite: copy in failed\n" );
            goto fail;
        }

        pBufIn = (NvU32 *)small_buf;

        err = NvOsCopyIn( pBufIn, p.pBuffer, 4 * 4 );
        if( err != NvSuccess )
        {
            printk( "NvRmIoctls_NvRmMemWrite: copy in failed\n" );
            goto fail;
        }

        hMem = (NvRmMemHandle)((NvU32*)pBufIn)[0];
        offset = ((NvU32*)pBufIn)[1];
        unsafep = (void *)((NvU32*)pBufIn)[2];
        size = ((NvU32*)pBufIn)[3];

        //printk( "NvRmIoctls_NvRmMemWrite\n" );

        ptr = (NvU8 *)hMem->VirtualAddress + offset;
        NV_ASSERT(offset + size <= hMem->size);

        err = NvOsCopyIn(ptr, unsafep, size);
        if( err != NvSuccess )
        {
            printk( "NvRmIoctls_NvRmMemWrite: copy out failed\n" );
            goto fail;
        }
        break;
    case NvRmIoctls_NvRmMemReadStrided:
        err = NvOsCopyIn( &p, (void *)arg, sizeof(p) );
        if( err != NvSuccess )
        {
            printk( "NvRmIoctls_NvRmMemReadStrided: copy in failed\n" );
            goto fail;
        }

        pBufIn = (NvU32 *)small_buf;

        //printk( "NvRmIoctls_NvRmMemReadStrided\n" );

        err = NvOsCopyIn( pBufIn, p.pBuffer, 7 * 4 );
        if( err != NvSuccess )
        {
            printk( "NvRmIoctls_NvRmMemReadStrided: copy in failed\n" );
            goto fail;
        }

        MemReadStrided(
            (NvRmMemHandle)((NvU32*)pBufIn)[0],
            ((NvU32*)pBufIn)[1],
            ((NvU32*)pBufIn)[2],
            (void *)((NvU32*)pBufIn)[3],
            ((NvU32*)pBufIn)[4],
            ((NvU32*)pBufIn)[5],
            ((NvU32*)pBufIn)[6]);
        break;
    case NvRmIoctls_NvRmMemWriteStrided:
        err = NvOsCopyIn( &p, (void *)arg, sizeof(p) );
        if( err != NvSuccess )
        {
            printk( "NvRmIoctls_NvRmMemWriteStrided: copy in failed\n" );
            goto fail;
        }

        pBufIn = (NvU32 *)small_buf;

        //printk( "NvRmIoctls_NvRmMemWriteStrided\n" );

        err = NvOsCopyIn( pBufIn, p.pBuffer, 7 * 4 );
        if( err != NvSuccess )
        {
            printk( "NvRmIoctls_NvRmMemWriteStrided: copy in failed\n" );
            goto fail;
        }

        MemWriteStrided(
            (NvRmMemHandle)((NvU32*)pBufIn)[0],
            ((NvU32*)pBufIn)[1],
            ((NvU32*)pBufIn)[2],
            (const void *)((NvU32*)pBufIn)[3],
            ((NvU32*)pBufIn)[4],
            ((NvU32*)pBufIn)[5],
            ((NvU32*)pBufIn)[6]);
        break;
    case NvRmIoctls_NvRmMemMapIntoCallerPtr:
        // FIXME: implement?
        printk( "NvRmIoctls_NvRmMemMapIntoCallerPtr: not supported\n" );
        goto fail;
    case NvRmIoctls_NvRmGetCarveoutInfo:
    {
        NvU32 *pBufOut32;
        NvU32 out[3];

        err = NvOsCopyIn( &p, (void *)arg, sizeof(p) );
        if( err != NvSuccess )
        {
            printk( "NvRmIoctls_NvRmGetCarveoutInfo: copy in failed\n" );
            goto fail;
        }

        pBufOut32 = (NvU32 *)((NvU8 *)p.pBuffer) + p.InBufferSize +
            p.InOutBufferSize;

        //printk( "NvRmIoctls_NvRmGetCarveoutInfo\n" );

        // get all the great information about the carveout.
        NvRmPrivHeapCarveoutGetInfo( &out[0], (void *)&out[1], &out[2] );

        err = NvOsCopyOut( pBufOut32, out, sizeof(out) );
        if( err != NvSuccess )
        {
            printk( "NvRmIoctls_NvRmGetCarveoutInfo: copy out failed\n" );
            goto fail;
        }

        break;
    }
    case NvRmIoctls_NvRmBootDone:
        if (!s_DfsThread)
        {
            if (NvOsInterruptPriorityThreadCreate(NvRmDfsThread,
                    (void*)s_hRmGlobal, &s_DfsThread)!=NvSuccess)
            {
                NvOsDebugPrintf("Failed to create DFS processing thread\n");
                goto fail;
            }
        }
        break;        
    case NvRmIoctls_NvRmGetClientId:
        err = NvOsCopyIn(&p, (void*)arg, sizeof(p));
        if (err != NvSuccess)
        {
            NvOsDebugPrintf("NvRmIoctls_NvRmGetClientId: copy in failed\n");
            goto fail;
        }

        NV_ASSERT(p.InBufferSize == 0);
        NV_ASSERT(p.OutBufferSize == sizeof(NvRtClientHandle));
        NV_ASSERT(p.InOutBufferSize == 0);
        
        if (NvOsCopyOut(p.pBuffer,
                        &file->private_data,
                        sizeof(NvRtClientHandle)) != NvSuccess)
        {
            NvOsDebugPrintf("Failed to copy client id\n");
            goto fail;
        }
        break;
    case NvRmIoctls_NvRmClientAttach:
    {
        NvRtClientHandle Client;

        err = NvOsCopyIn(&p, (void*)arg, sizeof(p));
        if (err != NvSuccess)
        {
            NvOsDebugPrintf("NvRmIoctls_NvRmClientAttach: copy in failed\n");
            goto fail;
        }
        
        NV_ASSERT(p.InBufferSize == sizeof(NvRtClientHandle));
        NV_ASSERT(p.OutBufferSize == 0);
        NV_ASSERT(p.InOutBufferSize == 0);

        if (NvOsCopyIn((void*)&Client,
                       p.pBuffer,
                       sizeof(NvRtClientHandle)) != NvSuccess)
        {
            NvOsDebugPrintf("Failed to copy client id\n");
            goto fail;
        }

        NV_ASSERT(Client || !"Bad client");
        
        if (Client == (NvRtClientHandle)file->private_data)
        {
            // The daemon is attaching to itself, no need to add refcount
            break;
        }
        if (NvRtAddClientRef(s_RtHandle, Client) != NvSuccess)
        {
            NvOsDebugPrintf("Client ref add unsuccessful\n");
            goto fail;
        }
        break;
    }
    case NvRmIoctls_NvRmClientDetach:
    {
        NvRtClientHandle Client;

        err = NvOsCopyIn(&p, (void*)arg, sizeof(p));
        if (err != NvSuccess)
        {
            NvOsDebugPrintf("NvRmIoctls_NvRmClientAttach: copy in failed\n");
            goto fail;
        }
        
        NV_ASSERT(p.InBufferSize == sizeof(NvRtClientHandle));
        NV_ASSERT(p.OutBufferSize == 0);
        NV_ASSERT(p.InOutBufferSize == 0);
        
        if (NvOsCopyIn((void*)&Client,
                       p.pBuffer,
                       sizeof(NvRtClientHandle)) != NvSuccess)
        {
            NvOsDebugPrintf("Failed to copy client id\n");
            goto fail;
        }

        NV_ASSERT(Client || !"Bad client");

        if (Client == (NvRtClientHandle)file->private_data)
        {
            // The daemon is detaching from itself, no need to dec refcount
            break;
        }
        
        client_detach(Client);        
        break;
    }        
    // FIXME: power ioctls?
    default:
        printk( "unknown ioctl code\n" );
        goto fail;
    }

    e = 0;
    goto clean;

fail:
    e = -EINVAL;

clean:
    if( bAlloc )
    {
        NvOsFree( ptr );
    }

    return e;
}

int nvrm_mmap(struct file *file, struct vm_area_struct *vma)
{
    return 0;
}

module_init(nvrm_init);
module_exit(nvrm_deinit);
