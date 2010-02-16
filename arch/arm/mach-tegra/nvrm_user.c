/*
 * arch/arm/mach-tegra/nvrm_user.c
 *
 * User-land access to NvRm APIs
 *
 * Copyright (c) 2008-2010, NVIDIA Corporation.
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
#include <linux/platform_device.h>
#include <linux/freezer.h>
#include <linux/suspend.h>
#include <linux/percpu.h>
#include <asm/cpu.h>
#include "nvcommon.h"
#include "nvassert.h"
#include "nvos.h"
#include "nvrm_memmgr.h"
#include "nvrm_ioctls.h"
#include "mach/nvrm_linux.h"
#include "linux/nvos_ioctl.h"
#include "nvrm_power_private.h"
#include "nvreftrack.h"

pid_t s_nvrm_daemon_pid = 0;
int s_nvrm_daemon_sig = 0;

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

#ifdef CONFIG_FB_TEGRA
extern int tegra_fb_control(void *in, void *out);
#else
#define tegra_fb_control(_i, _o) do {} while (0)
#endif

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

    set_freezable_with_signal();

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
                printk("DFS requested CPU1 ON\n");
                preset_lpj = per_cpu(cpu_data, 0).loops_per_jiffy;
                cpu_up(1);
#endif
            }

            if (Request & NvRmPmRequest_CpuOffFlag)
            {
#ifdef CONFIG_HOTPLUG_CPU
                printk("DFS requested CPU1 OFF\n");
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
    NvU32 size;
    NvU32 small_buf[8];
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
    case NvRmIoctls_NvRmMemWrite:
    case NvRmIoctls_NvRmMemReadStrided:
    case NvRmIoctls_NvRmGetCarveoutInfo:
    case NvRmIoctls_NvRmMemWriteStrided:
        goto fail;

    case NvRmIoctls_NvRmMemMapIntoCallerPtr:
        // FIXME: implement?
        printk( "NvRmIoctls_NvRmMemMapIntoCallerPtr: not supported\n" );
        goto fail;
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

static int nvrm_probe(struct platform_device *pdev)
{
    int e = 0;
    NvU32 NumTypes = NvRtObjType_NvRm_Num;

    printk("nvrm probe\n");

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

        printk("nvrm probe failed to open\n");
    }
    return e;
}

static int nvrm_remove(struct platform_device *pdev)
{
    misc_deregister( &nvrm_dev );
    NvRtDestroy(s_RtHandle);
    s_RtHandle = NULL;
    return 0;
}

static int nvrm_suspend(struct platform_device *pdev, pm_message_t state)
{
	if(NvRmKernelPowerSuspend(s_hRmGlobal)) {
		printk(KERN_INFO "%s : FAILED\n", __func__);
		return -1;
	}
	return 0;
}

static int nvrm_resume(struct platform_device *pdev)
{
	if(NvRmKernelPowerResume(s_hRmGlobal)) {
		printk(KERN_INFO "%s : FAILED\n", __func__);
		return -1;
	}
	return 0;

}

static struct platform_driver nvrm_driver =
{
    .probe	= nvrm_probe,
    .remove	= nvrm_remove,
    .suspend	= nvrm_suspend,
    .resume	= nvrm_resume,
    .driver	= { .name = "nvrm" }
};

//
// /sys/power/nvrm/notifier
//

wait_queue_head_t tegra_pm_notifier_wait;
int tegra_pm_notifier_continue_ok;

struct kobject *nvrm_kobj;

char* nvrm_notifier;

#define STRING_PM_NONE            "none"               // initial state
#define STRING_PM_SUSPEND_PREPARE "PM_SUSPEND_PREPARE" // notify to daemon
#define STRING_PM_POST_SUSPEND    "PM_POST_SUSPEND"    // notify to daemon
#define STRING_PM_CONTINUE        "PM_CONTINUE"        // reply from daemon
#define STRING_PM_SIGNAL          "PM_SIGNAL"          // request signal

ssize_t
nvrm_notifier_show(struct kobject *kobj, struct kobj_attribute *attr,
		   char *buf)
{
    return sprintf(buf, "%s\n", nvrm_notifier);
}

ssize_t
nvrm_notifier_store(struct kobject *kobj, struct kobj_attribute *attr,
		    const char *buf, size_t count)
{
    printk(KERN_INFO "%s: /sys/power/nvrm/notifier=%s\n", __func__, buf);

    if (! strcmp(buf, STRING_PM_CONTINUE))
    {
	nvrm_notifier = STRING_PM_CONTINUE;

	// Wake up pm_notifier.
	tegra_pm_notifier_continue_ok = 1;
	wake_up(&tegra_pm_notifier_wait);
    }
    else if (! strncmp(buf, STRING_PM_SIGNAL, strlen(STRING_PM_SIGNAL)))
    {
	s_nvrm_daemon_pid = 0;
	s_nvrm_daemon_sig = 0;
	sscanf(buf, STRING_PM_SIGNAL " %d %d",
	       &s_nvrm_daemon_pid, &s_nvrm_daemon_sig);
	printk(KERN_INFO "%s: nvrm_daemon=%d signal=%d\n",
	       __func__, s_nvrm_daemon_pid, s_nvrm_daemon_sig);
    }
    else
    {
	printk(KERN_ERR "%s: Wrong value '%s'.\n", __func__, buf);
    }

    return count;
}

static struct kobj_attribute nvrm_notifier_attribute =
    __ATTR(notifier, 0666, nvrm_notifier_show, nvrm_notifier_store);

//
// PM notifier
//

int tegra_pm_notifier(struct notifier_block *nb,
		      unsigned long event, void *nouse)
{
    int err;
    long timeout = HZ * 30;

    printk(KERN_INFO "%s: event=%lx\n", __func__, event);

    // Notify the event to nvrm_daemon.
    if (event == PM_SUSPEND_PREPARE)
    {
	tegra_pm_notifier_continue_ok = 0; // Clear before kicking nvrm_daemon.
	nvrm_notifier = STRING_PM_SUSPEND_PREPARE;
    }
    else if (event == PM_POST_SUSPEND)
    {
	tegra_pm_notifier_continue_ok = 0; // Clear before kicking nvrm_daemon.
	nvrm_notifier = STRING_PM_POST_SUSPEND;
    }
    else
    {
	printk(KERN_ERR "%s: Unknown event %ld.\n", __func__, event);
	return NOTIFY_DONE;
    }

    // In case if daemon's pid is not reported, do not signal or wait.
    if (! s_nvrm_daemon_pid)
    {
	printk(KERN_ERR "%s: Don't know nvrm_daemon's PID.\n", __func__);
	return NOTIFY_DONE;
    }

    // Send signal to nvrm_daemon.
    printk(KERN_INFO "%s: Sending signal=%d to pid=%d.\n",
	   __func__, s_nvrm_daemon_sig, s_nvrm_daemon_pid);
    err = kill_pid(find_get_pid(s_nvrm_daemon_pid), s_nvrm_daemon_sig, 0);
    if (err)
    {
	printk(KERN_ERR "%s: Cannot send signal to nvrm_daemon (PID=%d).\n",
	       __func__, s_nvrm_daemon_pid);
	return NOTIFY_DONE;
    }

    // Wait for the reply from nvrm_daemon.
    printk(KERN_INFO "%s: Wait for nvrm_daemon.\n", __func__);
    timeout = wait_event_timeout(tegra_pm_notifier_wait,
				 tegra_pm_notifier_continue_ok, timeout);

    // Go back to the initial state.
    nvrm_notifier = STRING_PM_NONE;

    // In case of timeout.
    if (timeout == 0)
    {
	printk(KERN_ERR "%s: Timed out. nvrm_daemon did not reply.\n", __func__);
	return NOTIFY_DONE;
    }

    printk(KERN_INFO "%s: Woken up.\n", __func__);
    return NOTIFY_OK;
}

static int __init nvrm_init(void)
{
    int ret = 0;
    printk(KERN_INFO "%s called\n", __func__);

    // Register PM notifier.
    pm_notifier(tegra_pm_notifier, 0);
    init_waitqueue_head(&tegra_pm_notifier_wait);

    // Create /sys/power/nvrm/notifier.
    nvrm_kobj = kobject_create_and_add("nvrm", power_kobj);
    sysfs_create_file(nvrm_kobj, &nvrm_notifier_attribute.attr);
    nvrm_notifier = STRING_PM_NONE;

    // Register NvRm platform driver.
    ret= platform_driver_register(&nvrm_driver);

    return ret;
}

static void __exit nvrm_deinit(void)
{
    printk(KERN_INFO "%s called\n", __func__);
    platform_driver_unregister(&nvrm_driver);
}

module_init(nvrm_init);
module_exit(nvrm_deinit);
