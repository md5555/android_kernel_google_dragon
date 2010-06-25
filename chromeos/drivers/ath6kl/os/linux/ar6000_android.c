/*
 *
 * Copyright (c) 2004-2010 Atheros Communications Inc.
 * All rights reserved.
 *
 * 
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation;
//
// Software distributed under the License is distributed on an "AS
// IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
// implied. See the License for the specific language governing
// rights and limitations under the License.
//
//
 *
 */
#include "ar6000_drv.h"
#undef ATH_MODULE_NAME
#define ATH_MODULE_NAME android
#include "htc.h"
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/inetdevice.h>

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif
#include <linux/earlysuspend.h>

enum {
    WLAN_PWR_CTRL_UP = 0,
    WLAN_PWR_CTRL_CUT_PWR,
    WLAN_PWR_CTRL_DEEP_SLEEP,
    WLAN_PWR_CTRL_WOW,
    WLAN_PWR_CTRL_DEEP_SLEEP_DISABLED
};

enum {
    WOW_STATE_NONE = 0,
    WOW_STATE_SUSPENDED,
    WOW_STATE_SUSPENDING,
};

#define WOW_ENABLE_MAX_INTERVAL 0
#define WOW_SET_SCAN_PARAMS 0

#define IS_MAC_NULL(mac) (mac[0]==0 && mac[1]==0 && mac[2]==0 && mac[3]==0 && mac[4]==0 && mac[5]==0)
#define MAX_BUF (8*1024)

#define  ATH_DEBUG_SUSPEND       ATH_DEBUG_MAKE_MODULE_MASK(0)

#ifdef DEBUG

static ATH_DEBUG_MASK_DESCRIPTION android_debug_desc[] = {
    { ATH_DEBUG_SUSPEND      , "Android Debug Logs"},
};

ATH_DEBUG_INSTANTIATE_MODULE_VAR(android,
                                 "android",
                                 "Android Driver Interface",
                                 ATH_DEBUG_MASK_DEFAULTS | ATH_DEBUG_SUSPEND,
                                 ATH_DEBUG_DESCRIPTION_COUNT(android_debug_desc),
                                 android_debug_desc);
                                 
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
char fwpath[256] = "/system/wifi";
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0) */
int buspm = WLAN_PWR_CTRL_WOW;
int wow2mode = WLAN_PWR_CTRL_DEEP_SLEEP;
int wowledon;
unsigned int enablelogcat;

extern int bmienable;
extern int wlaninitmode;
extern unsigned int wmitimeout;
extern wait_queue_head_t arEvent;
extern struct net_device *ar6000_devices[];
#ifdef CONFIG_HOST_TCMD_SUPPORT
extern unsigned int testmode;
#endif
extern char ifname[];
extern unsigned int bypasswmi;

const char def_ifname[] = "wlan0";
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
module_param_string(fwpath, fwpath, sizeof(fwpath), 0644);
module_param(buspm, int, 0644);
module_param(enablelogcat, uint, 0644);
module_param(wowledon, int, 0644);
#else
#define __user
/* for linux 2.4 and lower */
MODULE_PARM(buspm,"i");
MODULE_PARAM(wowledon,"i");
#endif 

struct wake_lock ar6k_init_wake_lock;
struct wake_lock ar6k_wow_wake_lock;
static int screen_is_off;
static struct early_suspend ar6k_early_suspend;
static A_STATUS (*ar6000_avail_ev_p)(void *, void *);

extern int ar6000_init(struct net_device *dev);
extern A_STATUS ar6000_configure_target(AR_SOFTC_T *ar);
extern void ar6000_stop_endpoint(struct net_device *dev, A_BOOL keepprofile);
extern A_STATUS ar6000_sysfs_bmi_get_config(AR_SOFTC_T *ar, A_UINT32 mode);
extern void ar6000_destroy(struct net_device *dev, unsigned int unregister);

static void ar6000_enable_mmchost_detect_change(int enable);
static void ar6000_restart_endpoint(struct net_device *dev);

#if defined(CONFIG_PM)
static A_STATUS ar6000_suspend_ev(void *context);

static A_STATUS ar6000_resume_ev(void *context);
#endif

#ifndef CONFIG_MMC_MSM
int logger_write(const enum logidx index,
                const unsigned char prio,
                const char __kernel * const tag,
                const char __kernel * const fmt,
                ...)
{
    int ret = 0;
    va_list vargs;
    struct file *filp = (struct file *)-ENOENT;
    mm_segment_t oldfs;
    struct iovec vec[3];
    int tag_bytes = strlen(tag) + 1, msg_bytes;
    char *msg;      
    va_start(vargs, fmt);
    msg = kvasprintf(GFP_ATOMIC, fmt, vargs);
    va_end(vargs);
    if (!msg)
        return -ENOMEM;
    if (in_interrupt()) {
        /* we have no choice since aio_write may be blocked */
        printk(KERN_ALERT "%s", msg);
        goto out_free_message;
    }
    msg_bytes = strlen(msg) + 1;
    if (msg_bytes <= 1) /* empty message? */
        goto out_free_message; /* don't bother, then */
    if ((msg_bytes + tag_bytes + 1) > 2048) {
        ret = -E2BIG;
        goto out_free_message;
    }
            
    vec[0].iov_base  = (unsigned char *) &prio;
    vec[0].iov_len    = 1;
    vec[1].iov_base   = (void *) tag;
    vec[1].iov_len    = strlen(tag) + 1;
    vec[2].iov_base   = (void *) msg;
    vec[2].iov_len    = strlen(msg) + 1; 

    oldfs = get_fs();
    set_fs(KERNEL_DS);
    do {
        filp = filp_open("/dev/log/main", O_WRONLY, S_IRUSR);
        if (IS_ERR(filp) || !filp->f_op) {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("%s: filp_open /dev/log/main error\n", __FUNCTION__));
            ret = -ENOENT;
            break;
        }

        if (filp->f_op->aio_write) {
            int nr_segs = sizeof(vec) / sizeof(vec[0]);
            int len = vec[0].iov_len + vec[1].iov_len + vec[2].iov_len;
            struct kiocb kiocb;
            init_sync_kiocb(&kiocb, filp);
            kiocb.ki_pos = 0;
            kiocb.ki_left = len;
            kiocb.ki_nbytes = len;
            ret = filp->f_op->aio_write(&kiocb, vec, nr_segs, kiocb.ki_pos);
        }
        
    } while (0);

    if (!IS_ERR(filp)) {
        filp_close(filp, NULL);
    }
    set_fs(oldfs);
out_free_message:
    if (msg) {
        kfree(msg);
    }
    return ret;
}
#endif

int android_logger_lv(void *module, int mask)
{
    switch (mask) {
    case ATH_DEBUG_ERR:
        return 6;
    case ATH_DEBUG_INFO:
        return 4;
    case ATH_DEBUG_WARN:
        return 5; 
    case ATH_DEBUG_TRC:        
        return 3; 
    default:
#ifdef DEBUG
        if (!module) {
            return 3;
        } else if (module == &GET_ATH_MODULE_DEBUG_VAR_NAME(driver)) {
            return (mask <=ATH_DEBUG_MAKE_MODULE_MASK(3)) ? 3 : 2;
        } else if (module == &GET_ATH_MODULE_DEBUG_VAR_NAME(htc)) {
            return 2;
        } else {
            return 3;
        }
#else
        return 3; /* DEBUG */
#endif
    }
}

static int android_readwrite_file(const A_CHAR *filename, A_CHAR *rbuf, const A_CHAR *wbuf, size_t length)
{
    int ret = 0;
    struct file *filp = (struct file *)-ENOENT;
    mm_segment_t oldfs;
    oldfs = get_fs();
    set_fs(KERNEL_DS);
    do {
        int mode = (wbuf) ? O_RDWR : O_RDONLY;
        filp = filp_open(filename, mode, S_IRUSR);
        if (IS_ERR(filp) || !filp->f_op) {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("%s: file %s filp_open error\n", __FUNCTION__, filename));
            ret = -ENOENT;
            break;
        }
    
        if (length==0) {
            /* Read the length of the file only */
            struct inode    *inode;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
            inode = filp->f_path.dentry->d_inode;
#else
            inode = filp->f_dentry->d_inode;
#endif
		    if (!inode) {
                AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("%s: Get inode from %s failed\n", __FUNCTION__, filename));
                ret = -ENOENT;
                break;
            }
            ret = i_size_read(inode->i_mapping->host);
            break;
        }

        if (wbuf) {
            if ( (ret=filp->f_op->write(filp, wbuf, length, &filp->f_pos)) < 0) {
                AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("%s: Write %u bytes to file %s error %d\n", __FUNCTION__, 
                                length, filename, ret));
                break;
            }
        } else {
            if ( (ret=filp->f_op->read(filp, rbuf, length, &filp->f_pos)) < 0) {
                AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("%s: Read %u bytes from file %s error %d\n", __FUNCTION__,
                                length, filename, ret));
                break;
            }
        }
    } while (0);

    if (!IS_ERR(filp)) {
        filp_close(filp, NULL);
    }
    set_fs(oldfs);

    return ret;
}

int android_request_firmware(const struct firmware **firmware_p, const char *name,
                     struct device *device)
{
    int ret = 0;
    struct firmware *firmware;
    char filename[256];
    const char *raw_filename = name;
	*firmware_p = firmware = kzalloc(sizeof(*firmware), GFP_KERNEL);
    if (!firmware) 
		return -ENOMEM;
	sprintf(filename, "%s/%s", fwpath, raw_filename);
    do {
        size_t length, bufsize, bmisize;

        if ( (ret=android_readwrite_file(filename, NULL, NULL, 0)) < 0) {
            break;
        } else {
            length = ret;
        }
    
        bufsize = ALIGN(length, PAGE_SIZE);
        bmisize = A_ROUND_UP(length, 4);
        bufsize = max(bmisize, bufsize);
        firmware->data = vmalloc(bufsize);
        firmware->size = bmisize;
        if (!firmware->data) {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("%s: Cannot allocate buffer for firmware\n", __FUNCTION__));
            ret = -ENOMEM;
            break;
        }
    
        if ( (ret=android_readwrite_file(filename, (char*)firmware->data, NULL, length)) != length) {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("%s: file read error, ret %d request %d\n", __FUNCTION__, ret, length));
            ret = -1;
            break;
        }
    
    } while (0);

    if (ret<0) {
        if (firmware) {
            if (firmware->data)
                vfree(firmware->data);
            kfree(firmware);
        }
        *firmware_p = NULL;
    } else {
        ret = 0;
    }
    return ret;    
}

void android_release_firmware(const struct firmware *firmware)
{
	if (firmware) {
        if (firmware->data)
            vfree(firmware->data);
        kfree(firmware);
    }
}

#if defined(CONFIG_PM)
static void ar6k_send_asleep_event_to_app(AR_SOFTC_T *ar, A_BOOL asleep)
{
    char buf[128];
    union iwreq_data wrqu;

    snprintf(buf, sizeof(buf), "HOST_ASLEEP=%s", asleep ? "asleep" : "awake");
    A_MEMZERO(&wrqu, sizeof(wrqu));
    wrqu.data.length = strlen(buf);
    wireless_send_event(ar->arNetDev, IWEVCUSTOM, &wrqu, buf);
}

static void ar6000_wow_resume(AR_SOFTC_T *ar)
{
    if (ar->arWowState!=WOW_STATE_NONE) {
        A_UINT16 fg_start_period = (ar->scParams.fg_start_period==0) ? 1 : ar->scParams.fg_start_period;
        A_UINT16 bg_period = (ar->scParams.bg_period==0) ? 60 : ar->scParams.bg_period;
        WMI_SET_HOST_SLEEP_MODE_CMD hostSleepMode = {TRUE, FALSE};
        ar->arWowState = WOW_STATE_NONE;
        wake_lock_timeout(&ar6k_wow_wake_lock, 3*HZ);
        if (wmi_set_host_sleep_mode_cmd(ar->arWmi, &hostSleepMode)!=A_OK) {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("Fail to setup restore host awake\n"));
        }
#if WOW_SET_SCAN_PARAMS
        wmi_scanparams_cmd(ar->arWmi, fg_start_period,
                                   ar->scParams.fg_end_period,
                                   bg_period,
                                   ar->scParams.minact_chdwell_time,
                                   ar->scParams.maxact_chdwell_time,
                                   ar->scParams.pas_chdwell_time,
                                   ar->scParams.shortScanRatio,
                                   ar->scParams.scanCtrlFlags,
                                   ar->scParams.max_dfsch_act_time,
                                   ar->scParams.maxact_scan_per_ssid);
#else
       (void)fg_start_period; 
       (void)bg_period;
#endif 


#if WOW_ENABLE_MAX_INTERVAL /* we don't do it if the power consumption is already good enough. */
        if (wmi_listeninterval_cmd(ar->arWmi, ar->arListenIntervalT, ar->arListenIntervalB) == A_OK) {
        }
#endif
        ar6k_send_asleep_event_to_app(ar, FALSE); 
        AR_DEBUG_PRINTF(ATH_DEBUG_SUSPEND, ("Resume WoW successfully\n"));
    } else {
        AR_DEBUG_PRINTF(ATH_DEBUG_SUSPEND, ("WoW does not invoked. skip resume"));
    }
}

static void ar6000_wow_suspend(AR_SOFTC_T *ar)
{
#define ANDROID_WOW_LIST_ID 1
    if (ar->arNetworkType != AP_NETWORK) {
        /* Setup WoW for unicast & Arp request for our own IP
        disable background scan. Set listen interval into 1000 TUs
        Enable keepliave for 110 seconds
        */
        struct in_ifaddr **ifap = NULL;
        struct in_ifaddr *ifa = NULL;
        struct in_device *in_dev;
        A_UINT8 macMask[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
        A_STATUS status;
        WMI_ADD_WOW_PATTERN_CMD addWowCmd = { .filter = { 0 } };
        WMI_DEL_WOW_PATTERN_CMD delWowCmd;
        WMI_SET_HOST_SLEEP_MODE_CMD hostSleepMode = {FALSE, TRUE};
        WMI_SET_WOW_MODE_CMD wowMode = {    .enable_wow = TRUE, 
                                            .hostReqDelay = 500 };/*500 ms delay*/
        
        if (ar->arWowState!=WOW_STATE_NONE) {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("System already go into wow mode!\n"));
            return;
        }

        ar6000_TxDataCleanup(ar); /* IMPORTANT, otherwise there will be 11mA after listen interval as 1000*/

#if WOW_ENABLE_MAX_INTERVAL /* we don't do it if the power consumption is already good enough. */
        if (wmi_listeninterval_cmd(ar->arWmi, A_MAX_WOW_LISTEN_INTERVAL, 0) == A_OK) {
        }
#endif

#if WOW_SET_SCAN_PARAMS
        status = wmi_scanparams_cmd(ar->arWmi, 0xFFFF, 0, 0xFFFF, 0, 0, 0, 0, 0, 0, 0);
#endif 
        /* clear up our WoW pattern first */
        delWowCmd.filter_list_id = ANDROID_WOW_LIST_ID;
        delWowCmd.filter_id = 0;
        wmi_del_wow_pattern_cmd(ar->arWmi, &delWowCmd);

        /* setup unicast packet pattern for WoW */
        if (ar->arNetDev->dev_addr[1]) {
            addWowCmd.filter_list_id = ANDROID_WOW_LIST_ID;
            addWowCmd.filter_size = 6; /* MAC address */
            addWowCmd.filter_offset = 0;
            status = wmi_add_wow_pattern_cmd(ar->arWmi, &addWowCmd, ar->arNetDev->dev_addr, macMask, addWowCmd.filter_size);
            if (status != A_OK) {
                AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("Fail to add WoW pattern\n"));
            }
        }
        /* setup ARP request for our own IP */
        if ((in_dev = __in_dev_get_rtnl(ar->arNetDev)) != NULL) {
            for (ifap = &in_dev->ifa_list; (ifa = *ifap) != NULL; ifap = &ifa->ifa_next) {
                if (!strcmp(ar->arNetDev->name, ifa->ifa_label)) {
                    break; /* found */
                }
            }
        }
        if (ifa && ifa->ifa_local) {
            WMI_SET_IP_CMD ipCmd;
            memset(&ipCmd, 0, sizeof(ipCmd));
            ipCmd.ips[0] = ifa->ifa_local;
            status = wmi_set_ip_cmd(ar->arWmi, &ipCmd);
            if (status != A_OK) {
                AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("Fail to setup IP for ARP agent\n"));
            }
        }

#ifndef ATH6K_CONFIG_OTA_MODE
        wmi_powermode_cmd(ar->arWmi, REC_POWER);
#endif

        status = wmi_set_wow_mode_cmd(ar->arWmi, &wowMode);
        if (status != A_OK) {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("Fail to enable wow mode\n"));
        }
        ar6k_send_asleep_event_to_app(ar, TRUE);

        status = wmi_set_host_sleep_mode_cmd(ar->arWmi, &hostSleepMode);
        if (status != A_OK) {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("Fail to set host asleep\n"));
        }

        ar->arWowState = WOW_STATE_SUSPENDING;
        if (ar->arTxPending[ar->arControlEp]) {
            long timeleft = wait_event_interruptible_timeout(arEvent,
            ar->arTxPending[ar->arControlEp] == 0, wmitimeout * HZ);
            if (!timeleft || signal_pending(current)) {
               /* what can I do? wow resume at once */
                AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("Fail to setup WoW. Pending wmi control data %d\n", ar->arTxPending[ar->arControlEp]));
            }
        }
    } else {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("Not allowed to go to WOW at this moment.\n"));
    }
}

static void ar6000_pwr_on(AR_SOFTC_T *ar)
{
    if (ar == NULL) {
        /* turn on for all cards */
    }
    AR_DEBUG_PRINTF(ATH_DEBUG_INFO,("%s --enter\n", __FUNCTION__));

}

static void ar6000_pwr_down(AR_SOFTC_T *ar)
{
    if (ar == NULL) {
        /* shutdown for all cards */
    }
    AR_DEBUG_PRINTF(ATH_DEBUG_INFO,("%s --enter\n", __FUNCTION__));

}

static A_STATUS ar6000_suspend_ev(void *context)
{
    A_STATUS status = A_OK;
    int pmmode = buspm;
    AR_SOFTC_T *ar = (AR_SOFTC_T *)context;
wow_not_connected:

    switch (pmmode) {
    case WLAN_PWR_CTRL_DEEP_SLEEP:
        if (ar->arWlanState == WLAN_DISABLED) {
            ar->arOsPowerCtrl = WLAN_PWR_CTRL_DEEP_SLEEP_DISABLED;
            AR_DEBUG_PRINTF(ATH_DEBUG_SUSPEND,("%s:Suspend for deep sleep disabled mode %d\n", __func__, ar->arOsPowerCtrl));
        } else {
            ar6000_set_wlan_state(ar, WLAN_DISABLED);
            ar->arOsPowerCtrl = WLAN_PWR_CTRL_DEEP_SLEEP;
            AR_DEBUG_PRINTF(ATH_DEBUG_SUSPEND,("%s:Suspend for deep sleep mode %d\n", __func__, ar->arOsPowerCtrl));
        }              
        status = A_EBUSY;
        break;
    case WLAN_PWR_CTRL_WOW:
        if (ar->arWmiReady && ar->arWlanState==WLAN_ENABLED && ar->arConnected) {
            ar->arOsPowerCtrl = WLAN_PWR_CTRL_WOW;
            AR_DEBUG_PRINTF(ATH_DEBUG_SUSPEND,("%s:Suspend for wow mode %d\n", __func__, ar->arOsPowerCtrl));
            ar6000_wow_suspend(ar);
            /* leave for pm_device to setup wow */
            status = A_EBUSY;
        } else {
            pmmode = wow2mode;
            goto wow_not_connected;
        }
        break;
    case WLAN_PWR_CTRL_CUT_PWR:
        /* fall through */
    default:        
        ar->arOsPowerCtrl = WLAN_PWR_CTRL_CUT_PWR;
        AR_DEBUG_PRINTF(ATH_DEBUG_SUSPEND,("%s: Suspend for cut off mode %d\n", __func__, ar->arOsPowerCtrl));
        ar6000_stop_endpoint(ar->arNetDev, TRUE);
        status = A_OK;
        break;
    }

    ar->scan_triggered = 0;
    return status;
}

static A_STATUS ar6000_resume_ev(void *context)
{
    AR_SOFTC_T *ar = (AR_SOFTC_T *)context;
    A_UINT16 powerCtrl = ar->arOsPowerCtrl;
    wake_lock(&ar6k_init_wake_lock);
    AR_DEBUG_PRINTF(ATH_DEBUG_SUSPEND, ("%s: enter previous state %d wowState %d\n", __func__, powerCtrl, ar->arWowState));
    ar->arOsPowerCtrl = WLAN_PWR_CTRL_UP;
    switch (powerCtrl) {
    case WLAN_PWR_CTRL_WOW:
        ar6000_wow_resume(ar);
        break;
    case WLAN_PWR_CTRL_CUT_PWR:
        ar6000_restart_endpoint(ar->arNetDev);
        break;
    case WLAN_PWR_CTRL_DEEP_SLEEP:
        ar6000_set_wlan_state(ar, WLAN_ENABLED);
        break;
    case WLAN_PWR_CTRL_DEEP_SLEEP_DISABLED:
        break;
    case WLAN_PWR_CTRL_UP:
        break;
    default:
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("Strange SDIO bus power mode!!\n"));
        break; 
    }
    wake_unlock(&ar6k_init_wake_lock);
    return A_OK;
}

static A_STATUS ar6000_android_avail_ev(void *context, void *hif_handle)
{
    A_STATUS ret;    
    wake_lock(&ar6k_init_wake_lock);
    ar6000_enable_mmchost_detect_change(0);
    ret = ar6000_avail_ev_p(context, hif_handle);
    wake_unlock(&ar6k_init_wake_lock);
    return ret;
}


static int ar6000_pm_suspend(struct platform_device *dev, pm_message_t state)
{
    int i;
    for (i = 0; i < MAX_AR6000; i++) {
        AR_SOFTC_T *ar;

        if (ar6000_devices[i] == NULL)
            continue;
        ar = (AR_SOFTC_T*)netdev_priv(ar6000_devices[i]);
        AR_DEBUG_PRINTF(ATH_DEBUG_SUSPEND,("%s: enter status %d\n", __func__, ar->arOsPowerCtrl));
        switch (ar->arOsPowerCtrl) {
        case WLAN_PWR_CTRL_CUT_PWR:
            ar6000_pwr_down(ar);
            break;
        case WLAN_PWR_CTRL_WOW:
            if (ar->arTxPending[ar->arControlEp]) {
                AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("Fail to setup WoW. Pending wmi control data %d\n", ar->arTxPending[ar->arControlEp]));
                ar->arWowState = WOW_STATE_NONE;
            } else {
                ar->arWowState = WOW_STATE_SUSPENDED;
                AR_DEBUG_PRINTF(ATH_DEBUG_SUSPEND,("Setup WoW successfully\n"));
            }
            break;
        case WLAN_PWR_CTRL_DEEP_SLEEP:
            /* fall through */
        case WLAN_PWR_CTRL_DEEP_SLEEP_DISABLED:
            /* nothing to do. keep the power on */
            break;
        default:
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("Something is strange for ar6000_pm_suspend %d\n", ar->arOsPowerCtrl));
            break;
       }
    }
    return 0;
}

static int ar6000_pm_resume(struct platform_device *dev)
{
    int i;
    for (i = 0; i < MAX_AR6000; i++) {
        AR_SOFTC_T *ar;

        if (ar6000_devices[i] == NULL)
            continue;
        ar = (AR_SOFTC_T*)netdev_priv(ar6000_devices[i]);
        AR_DEBUG_PRINTF(ATH_DEBUG_SUSPEND,("%s: enter status %d\n", __func__, ar->arOsPowerCtrl));
        switch (ar->arOsPowerCtrl) {
        case WLAN_PWR_CTRL_CUT_PWR:
            ar6000_pwr_on(ar);
            break;
        case WLAN_PWR_CTRL_WOW:
            /* nothing to do. keep the power on */
            break;
        case WLAN_PWR_CTRL_DEEP_SLEEP:
            /* fall through */
        case WLAN_PWR_CTRL_DEEP_SLEEP_DISABLED:
            /* nothing to do. keep the power on */
            break;
        default:
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("Something is strange for ar6000_pm_resume %d\n", ar->arOsPowerCtrl));
            break;
       }
    }
    return 0;
}

static int ar6000_pm_probe(struct platform_device *pdev)
{
	ar6000_pwr_on(NULL);
	return 0;
}

static int ar6000_pm_remove(struct platform_device *pdev)
{
	ar6000_pwr_down(NULL);
	return 0;
}

static struct platform_driver ar6000_pm_device = {
	.probe		= ar6000_pm_probe,
	.remove		= ar6000_pm_remove,
	.suspend	= ar6000_pm_suspend,
	.resume		= ar6000_pm_resume,
	.driver		= {
			.name = "wlan_ar6000_pm_dev",
	},
};
#endif /* CONFIG_PM */

/* Useful for qualcom platform to detect our wlan card for mmc stack */
static void ar6000_enable_mmchost_detect_change(int enable)
{
#ifdef CONFIG_MMC_MSM
#define MMC_MSM_DEV "msm_sdcc.1"
    char buf[3];
    int length;
    length = snprintf(buf, sizeof(buf), "%d\n", enable ? 1 : 0);
    if (android_readwrite_file("/sys/devices/platform/" MMC_MSM_DEV "/detect_change", 
                               NULL, buf, length) < 0) {
        /* fall back to polling */
        android_readwrite_file("/sys/devices/platform/" MMC_MSM_DEV "/polling", NULL, buf, length);
    }
#endif
}

static void
ar6000_restart_endpoint(struct net_device *dev)
{
    A_STATUS status = A_OK;
    AR_SOFTC_T *ar = (AR_SOFTC_T*)netdev_priv(dev);
    if (down_interruptible(&ar->arSem)) {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("%s(): down_interruptible failed \n", __func__));
        return ;
    }
    if (ar->bIsDestroyProgress) {
        up(&ar->arSem);
        return;
    }
    BMIInit();
    do {        
        if ( (status=ar6000_configure_target(ar))!=A_OK)
            break;
		if ( (status=ar6000_sysfs_bmi_get_config(ar, wlaninitmode)) != A_OK)
		{
			AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("ar6000_avail: ar6000_sysfs_bmi_get_config failed\n"));
			break;
		}     
        rtnl_lock();
        status = (ar6000_init(dev)==0) ? A_OK : A_ERROR;
        rtnl_unlock();

        if (status!=A_OK) {
            break;
        }  
        if (ar->arWlanState==WLAN_ENABLED) {
            if (ar->arSsidLen) {
                ar6000_connect_to_ap(ar);
            }
        } else {
            WMI_SET_WOW_MODE_CMD wowMode = { .enable_wow = FALSE };
            WMI_SET_HOST_SLEEP_MODE_CMD hostSleepMode = { .awake = FALSE, .asleep = TRUE };
            WMI_REPORT_SLEEP_STATE_EVENT wmiSleepEvent = {
                .sleepState = WMI_REPORT_SLEEP_STATUS_IS_DEEP_SLEEP
            };

            wmi_set_wow_mode_cmd(ar->arWmi, &wowMode);
            ar6000_send_event_to_app(ar, WMI_REPORT_SLEEP_STATE_EVENTID, (A_UINT8*)&wmiSleepEvent, 
                                    sizeof(WMI_REPORT_SLEEP_STATE_EVENTID));
            wmi_set_host_sleep_mode_cmd(ar->arWmi, &hostSleepMode);            
        }
    } while (0);

    up(&ar->arSem);    
    if (status==A_OK) {
        return;
    }

    ar6000_devices[ar->arDeviceIndex] = NULL;
    ar6000_destroy(ar->arNetDev, 1);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void android_early_suspend(struct early_suspend *h)
{
    screen_is_off = 1;
}

static void android_late_resume(struct early_suspend *h)
{
    screen_is_off = 0;
}
#endif

void android_module_init(OSDRV_CALLBACKS *osdrvCallbacks)
{
    bmienable = 1;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    if (ifname[0] == '\0')
        strcpy(ifname, def_ifname);
#endif 
    if (wow2mode!=WLAN_PWR_CTRL_CUT_PWR && wow2mode!=WLAN_PWR_CTRL_DEEP_SLEEP) {
        wow2mode=WLAN_PWR_CTRL_CUT_PWR;
    }

    wake_lock_init(&ar6k_init_wake_lock, WAKE_LOCK_SUSPEND, "ar6k_init");
    wake_lock_init(&ar6k_wow_wake_lock, WAKE_LOCK_SUSPEND, "ar6k_wow");

#ifdef CONFIG_HAS_EARLYSUSPEND
    ar6k_early_suspend.suspend = android_early_suspend;
    ar6k_early_suspend.resume  = android_late_resume;
    ar6k_early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
    register_early_suspend(&ar6k_early_suspend);
#endif

#if defined(CONFIG_PM)
    osdrvCallbacks->deviceSuspendHandler = ar6000_suspend_ev;
    osdrvCallbacks->deviceResumeHandler = ar6000_resume_ev;
#endif
    ar6000_avail_ev_p = osdrvCallbacks->deviceInsertedHandler;
    osdrvCallbacks->deviceInsertedHandler = ar6000_android_avail_ev;

#if defined(CONFIG_PM)
    /* Register ar6000_pm_device into system.
     * We should also add platform_device into the first item of array devices[] in
     * file arch/xxx/mach-xxx/board-xxxx.c
     * Otherwise, WoW may not work properly since we may trigger WoW GPIO before system suspend
     */
    if (platform_driver_register(&ar6000_pm_device))
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("ar6000: fail to register the power control driver.\n"));
#endif

    ar6000_enable_mmchost_detect_change(1);
}

void android_module_exit(void)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ar6k_early_suspend);
#endif
    wake_lock_destroy(&ar6k_wow_wake_lock);
    wake_lock_destroy(&ar6k_init_wake_lock);

#ifdef CONFIG_PM
    platform_driver_unregister(&ar6000_pm_device);
#endif
    ar6000_enable_mmchost_detect_change(1);
}

A_BOOL android_ar6k_endpoint_is_stop(AR_SOFTC_T *ar)
{
#ifdef CONFIG_PM
    return ar->arOsPowerCtrl == WLAN_PWR_CTRL_CUT_PWR;
#else
    return FALSE;
#endif 
}

void android_ar6k_check_wow_status(AR_SOFTC_T *ar, struct sk_buff *skb, A_BOOL isEvent)
{
#ifdef CONFIG_PM
    if (ar->arWowState!=WOW_STATE_NONE) {
        if (ar->arWowState==WOW_STATE_SUSPENDING) {
            AR_DEBUG_PRINTF(ATH_DEBUG_SUSPEND,("%s: Received IRQ while we are wow suspending!!!\n", __func__));
            return;
        }
        /* Wow resume from irq interrupt */
        AR_DEBUG_PRINTF(ATH_DEBUG_SUSPEND, ("%s: WoW resume from irq thread status %d\n", 
                        __func__, ar->arOsPowerCtrl));
        ar6000_wow_resume(ar);
        ar->arOsPowerCtrl = WLAN_PWR_CTRL_UP;
    } else if (screen_is_off && skb && ar->arConnected) {
        A_BOOL needWake = FALSE;
        if (isEvent) {
            if (A_NETBUF_LEN(skb) >= sizeof(A_UINT16)) {
                A_UINT16 cmd = *(const A_UINT16 *)A_NETBUF_DATA(skb);
                switch (cmd) {
                case WMI_CONNECT_EVENTID:
                case WMI_DISCONNECT_EVENTID:
                    needWake = TRUE;
                    break;
                default:
                    /* dont wake lock the system for other event */
                    break;
                }
            }
        } else if (A_NETBUF_LEN(skb) >= sizeof(ATH_MAC_HDR)) {
            ATH_MAC_HDR *datap = (ATH_MAC_HDR *)A_NETBUF_DATA(skb);
            if (!IEEE80211_IS_MULTICAST(datap->dstMac)) {
                switch (A_BE2CPU16(datap->typeOrLen)) {
                case 0x0800: /* IP */
                case 0x888e: /* EAPOL */
                case 0x88c7: /* RSN_PREAUTH */
                case 0x88b4: /* WAPI */
                     needWake = TRUE;
                     break;
                case 0x0806: /* ARP is not important to hold wake lock */
                default:
                    break;
                }
            }
        }
        if (needWake) {
            /* keep host wake up if there is any event and packate comming in*/
            wake_lock_timeout(&ar6k_wow_wake_lock, 3*HZ);
            if (wowledon) {
                char buf[32];
                int len = sprintf(buf, "on");         
                android_readwrite_file("/sys/power/state", NULL, buf, len);

                len = sprintf(buf, "%d", 127);
                android_readwrite_file("/sys/class/leds/lcd-backlight/brightness", 
                                       NULL, buf,len);                    
            }
        }
    }
#endif /* CONFIG_PM */
}

A_STATUS android_ar6k_start(AR_SOFTC_T *ar)
{
    if (!bypasswmi) {
#ifdef ATH6K_CONFIG_OTA_MODE
        wmi_powermode_cmd(ar->arWmi, MAX_PERF_POWER);
#endif
        wmi_disctimeout_cmd(ar->arWmi, 3);

    }
    return A_OK;
}
