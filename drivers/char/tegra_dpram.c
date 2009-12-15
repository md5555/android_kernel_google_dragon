/*
 * drivers/char/tegra_dpram.c
 *
 * Support for dual-ported RAM devices attached to the SNOR interface on
 * NVIDIA Tegra SoCs
 *
 * Copyright (c) 2009, NVIDIA Corporation.
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
 
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>

#include "nvsnor_controller.h"

#define DRIVER_NAME    "tegra_dpram"
#define DRIVER_DESC    "Nvidia Tegra dpram driver to access dual-port RAM by using SNOR controller"

#define NV_DPRAM_DEBUG_PRINT 0

#if NV_DPRAM_DEBUG_PRINT
#define NV_DPRAM_DBG(fmt,args...) \
    do { pr_info("("DRIVER_NAME")" fmt, ##args); } while (0)
#else
#define NV_DPRAM_DBG(fmt,args...) \
    do{} while (0);    
#endif

extern NvSnorInformation *g_pSnorInfo;
extern struct mutex snor_operation_lock;

SnorControllerTimingRegVals TimingRegVals_dpram = {0x1, 0x1, 0x1, 0x3, 0x3, 0x8};  

NvBool dpram_poweron(void)
{
    NvOdmServicesPmuHandle hPmu;
    NvU32 i;
    NvU32 settle_us;
    NvU64 guid;
    NvOdmPeripheralConnectivity const *conn;

    /* get the dpram guid */
    guid = NV_ODM_GUID('d','p','r','a','m','d','e','v');

    /* get the connectivity info */
    conn = NvOdmPeripheralGetGuid( guid );
    if (!conn) return NV_FALSE;
    
    hPmu = NvOdmServicesPmuOpen();
    if (!hPmu) return NV_FALSE;

    for( i = 0; i < conn->NumAddress; i++ )
    {
        if( conn->AddressList[i].Interface == NvOdmIoModule_Vdd )
        {
            NvOdmServicesPmuVddRailCapabilities cap;

            /* address is the vdd rail id */
            NvOdmServicesPmuGetCapabilities( hPmu,
                conn->AddressList[i].Address, &cap );

            /* set the rail volatage to the recommended */
            NvOdmServicesPmuSetVoltage( hPmu,
                conn->AddressList[i].Address, cap.requestMilliVolts,
                &settle_us );

            /* wait for rail to settle */
            NvOdmOsWaitUS( settle_us );
        }
    }

    NvOdmServicesPmuClose( hPmu );
    return NV_TRUE;
}

static ssize_t tegra_dpram_read(struct file *file, char __user *buf, size_t len,
                         loff_t *ppos)
{
    NvSnorHandle HandleSnor;
    NvU32 length;
    void * final_snorside_addr;
    NvU32 f_pos_local;

    if (!g_pSnorInfo->hSnor) 
    {
        printk("tegra_dpram_read fail since g_pSnorInfo->hSnor is zero!!\n");
        NV_ASSERT(0);
        return 0;
    }
   
    if (len==0) return 0;
    if ((len%2)==1)
    {
        printk("tegra_dpram_read, byte access is not allowed by SNOR controller!!\n");
        NV_ASSERT(0); //Byte access is limited due to SNOR controller's limitation. 
        return 0;
    }


    if (len%4!=0)
    {
        printk("tegra_dpram_read, in case DMA mode, only support read of 4bytes, 8bytes, 12 bytes, 16 bytes, ...!!");
        printk("SNOR controller's minimum DMA-able data size is a word (32bit.) ...!!\n");
        NV_ASSERT(0); //only support 4bytes, 8bytes, 12 bytes, 16 bytes in case DMA mode.
        return 0;
                     
    }
    mutex_lock(&snor_operation_lock);
    
    HandleSnor=g_pSnorInfo->hSnor;
    
    f_pos_local = (NvU32)(HandleSnor->ConnectedDevReg.DevicePureAddress);


    if (f_pos_local + (NvU32)len > NvOdmQueryMemSize(NvOdmMemoryType_Dpram))
    {
        len = NvOdmQueryMemSize(NvOdmMemoryType_Dpram) - f_pos_local;
        
        if ((len==0)||((len%2)==1)||((len%4)!=0)) 
        {
            printk("tegra_dpram_read: nothing read or calculated len for remained region do not satisfies condition!!\n");
            mutex_unlock(&snor_operation_lock);    
            return 0;
        }
    }

    final_snorside_addr = (NvU8 *)(HandleSnor->ConnectedDevReg.DeviceBaseAddress) 
        + (NvU32)(f_pos_local); 
        
    if (len<=(HandleSnor->Snor_DmaBufSize))
    {
        NvReadViaSNORControllerDMA (HandleSnor, final_snorside_addr, len>>2);
        NvOsMemcpy((void *)buf, (void *)HandleSnor->pAhbDmaBuffer, len);
    }
    else
    {
        NvU32 remained_len;
        NvU32 copy_len;
        NvU32 i=0;
        NvU8* buf_backup;
        copy_len = (HandleSnor->Snor_DmaBufSize);
        remained_len = len;
        

        buf_backup = (NvU8 *)buf;
        do
        {
            if (i!=0)
            {
                final_snorside_addr = (NvU8 *)final_snorside_addr + (HandleSnor->Snor_DmaBufSize);
                buf = (NvU8 *)buf + (HandleSnor->Snor_DmaBufSize);
            }
            NvReadViaSNORControllerDMA (HandleSnor, final_snorside_addr, copy_len>>2);
            NvOsMemcpy((void *)buf, (void *)HandleSnor->pAhbDmaBuffer, copy_len);
            remained_len = remained_len - (HandleSnor->Snor_DmaBufSize);
            copy_len = (copy_len<remained_len)?(copy_len):(remained_len);
            i++;
            
        }while(remained_len>0);
        buf = (NvU8 *)buf_backup;
    }  
    f_pos_local+=len;
    HandleSnor->ConnectedDevReg.DevicePureAddress = (NvU32)f_pos_local;
    
    length=len;
    mutex_unlock(&snor_operation_lock);    
    return length;
}

static ssize_t tegra_dpram_write(struct file *file, const char __user *data,
                          size_t len, loff_t *ppos)
{
    NvSnorHandle HandleSnor;
    NvU32 length;
    void * final_snorside_addr;
    NvU32 f_pos_local;
    
    if (!g_pSnorInfo->hSnor) 
    {
        NV_ASSERT(0);
        return 0;
    }
    
    if (len==0) return 0;
    if ((len%2)==1)
    {
        printk("tegra_dpram_write, byte access is not allowed by SNOR controller!!\n");
        NV_ASSERT(0); //Byte access is limited due to SNOR controller's limitation. 
        return 0;
    }

    if (len%4!=0)
    {
        printk("tegra_dpram_write, in case DMA mode, only support write of 4bytes, 8bytes, 12 bytes, 16 bytes, ...!!");
        printk("SNOR controller's minimum DMA-able data size is a word (32bit.) ...!!\n");
        NV_ASSERT(0); //only support 4bytes, 8bytes, 12 bytes, 16 bytes in case DMA mode.
        return 0;
                     
    }
    mutex_lock(&snor_operation_lock);
    
    HandleSnor=g_pSnorInfo->hSnor;
    
    f_pos_local = (NvU32)(HandleSnor->ConnectedDevReg.DevicePureAddress);
        
    if (f_pos_local + (NvU32)len > NvOdmQueryMemSize(NvOdmMemoryType_Dpram))
    {
        len = NvOdmQueryMemSize(NvOdmMemoryType_Dpram) - f_pos_local;
        
        if ((len==0)||((len%2)==1)||((len%4)!=0)) 
        {
            printk("tegra_dpram_write, nothing write or calculated len for remained region do not satisfies condition!!\n");
            mutex_unlock(&snor_operation_lock);    
            return 0;
        }
    }
 
    final_snorside_addr = (NvU8 *)(HandleSnor->ConnectedDevReg.DeviceBaseAddress) 
        + (NvU32)(f_pos_local); 
        
    {
        if (len<=(HandleSnor->Snor_DmaBufSize))
        {
            NvOsMemcpy((void *)HandleSnor->pAhbDmaBuffer, (void *)data, len);
            NvWriteViaSNORControllerDMA (HandleSnor, final_snorside_addr, len>>2);
        }
        else
        {
            NvU32 remained_len;
            NvU32 copy_len;
            NvU32 i=0;
            NvU8* data_backup;
            copy_len = (HandleSnor->Snor_DmaBufSize);
            remained_len = len;
            

            data_backup = (NvU8 *)data;
            do
            {
                if (i!=0)
                {
                    final_snorside_addr = (NvU8 *)final_snorside_addr + (HandleSnor->Snor_DmaBufSize);
                    data = (NvU8 *)data + (HandleSnor->Snor_DmaBufSize);
                }
                NvOsMemcpy((void *)HandleSnor->pAhbDmaBuffer, (void *)data, copy_len);
                NvWriteViaSNORControllerDMA (HandleSnor, final_snorside_addr, copy_len>>2);
                remained_len = remained_len - (HandleSnor->Snor_DmaBufSize);
                copy_len = (copy_len<remained_len)?(copy_len):(remained_len);
                i++;
                
            }while(remained_len>0);
            data = (NvU8 *)data_backup;
        }
    }
    f_pos_local+=len;
    HandleSnor->ConnectedDevReg.DevicePureAddress = (NvU32)f_pos_local; 

    length=len;    
    mutex_unlock(&snor_operation_lock);
    return length;
}

loff_t tegra_dpram_lseek(struct file *file, loff_t offset, int origin) 
{ 
    NvSnorHandle HandleSnor;
    
    if (!g_pSnorInfo->hSnor) 
    {
        NV_ASSERT(0);
        return -EINVAL;
    }
    
    if (origin!=SEEK_SET) return -EINVAL; //SEEK_SET is only supported.
    if (offset<0) return -EINVAL;

    if (offset%4!=0)
    {
        printk("tegra_dpram_lseek, in case DMA mode, only support write of 4bytes, 8bytes, 12 bytes, 16 bytes, ...!!");
        printk("SNOR controller's minimum DMA-able data size is a word (32bit.) ...!!\n");
        printk("So, offset also needs to be Word boundary ...!!\n");
        return -EINVAL;
                     
    }
        
    if (offset>NvOdmQueryMemSize(NvOdmMemoryType_Dpram)) return -EINVAL; //out of space
    mutex_lock(&snor_operation_lock);
    HandleSnor=g_pSnorInfo->hSnor;
    HandleSnor->ConnectedDevReg.DevicePureAddress  = (NvU32)offset; //Assume input offset in user mode layer is Byte boundary.
    mutex_unlock(&snor_operation_lock);
    return (loff_t)offset;
    
} 

static int  
tegra_dpram_open(struct inode *inode, struct file *file)
{

    NvError Error = NvError_Success;

    printk("tegra_dpram_open is called!!\n");
    
    if (g_pSnorInfo->hSnor){
        if (g_pSnorInfo->hSnor->OpenCount)
        {
            printk("tegra_dpram_open : Already opened in other place!!\n");
            printk("Support one pair of open/close.\n");
            return 1;
        }
    }
    
    if (NV_FALSE==dpram_poweron()) 
    {
        printk("tegra_dpram_open...DPRAM power rail on fails!!\n");
        return 1;
    }
    Error = CreateSnorHandle(g_pSnorInfo->hRmDevice, &g_pSnorInfo->hSnor);
    
    if (!Error) 
    {
        g_pSnorInfo->hSnor->OpenCount++;
        printk("%d th tegra_dpram_open!!\n", g_pSnorInfo->hSnor->OpenCount);
        printk("tegra_dpram_open...dpram size is 0x%x Byte\n", NvOdmQueryMemSize(NvOdmMemoryType_Dpram));
    }
    else  
    {
        printk("tegra_dpram_open CreateSnorHandle fails!!\n");
        return 1;
    }
    
    //MOD_INC_USE_COUT; //TODO ... Is this working?
    return 0;
}


static int  
tegra_dpram_release(struct inode *inode, struct file *file)
{
    printk("tegra_dpram_release is called!!\n");
    if (!g_pSnorInfo->hSnor)
    {
        printk("tegra dpram is not opened before, or already closed!!\n");
        return 1;
    }
    if (g_pSnorInfo->hSnor->OpenCount==1)
    {
       printk("tegra_dpram_release will do 1)DestorySnorHandle!!\n");
       DestroySnorHandle(g_pSnorInfo->hSnor);
       //DeinitSnorInformation();
    }
    //MOD_DEC_USE_COUT; //TODO ... Is this working?
    return 0;
}

static struct file_operations tegra_dpram_fops = {
    .owner      = THIS_MODULE,
    .read       = tegra_dpram_read,
    .write      = tegra_dpram_write,
    .open       = tegra_dpram_open,
    .release    = tegra_dpram_release,
    .llseek     = tegra_dpram_lseek, 
};

static struct miscdevice tegra_dpram_device =
{
    .name = "dpram",
    .fops = &tegra_dpram_fops,
    .minor = 0,
};

static int __init
tegra_dpram_init(void)
{
    int retVal = 0;
    retVal = misc_register(&tegra_dpram_device);
    
    
    if (retVal <0) printk("tegra_dpram_init fails!!\n"); 
    else printk("tegra_dpram_init succeeds! This is to access Dual-Port RAM by using Tegra SNOR controller interface!!\n");
    
    return retVal;
}

static void __exit
tegra_dpram_exit(void)
{
    misc_deregister (&tegra_dpram_device);
}

module_init(tegra_dpram_init);
module_exit(tegra_dpram_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);

