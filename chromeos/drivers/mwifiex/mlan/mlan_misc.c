/**
 * @file mlan_misc.c
 *
 *  @brief This file include miscellaneous functions for MLAN module
 *
 *
 *  Copyright (C) 2009, Marvell International Ltd. 
 *
 *  This software file (the "File") is distributed by Marvell International
 *  Ltd. under the terms of the GNU General Public License Version 2, June 1991
 *  (the "License").  You may use, redistribute and/or modify this File in
 *  accordance with the terms and conditions of the License, a copy of which
 *  is available by writing to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 *  worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *  
 *  THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 *  ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 *  this warranty disclaimer.
 */

/*************************************************************
Change Log:
    05/11/2009: initial version
************************************************************/
#include "mlan.h"
#include "mlan_join.h"
#include "mlan_util.h"
#include "mlan_fw.h"
#include "mlan_main.h"
#include "mlan_wmm.h"
#include "mlan_11n.h"
#include "mlan_sdio.h"

/********************************************************
                Local Variables
********************************************************/

/********************************************************
                Global Variables
********************************************************/

/********************************************************
                Local Functions
********************************************************/

/********************************************************
                Global Functions
********************************************************/

/** 
 *  @brief send host cmd
 *
 *  @param pmadapter	A pointer to mlan_adapter structure
 *  @param pioctl_req	A pointer to ioctl request buffer
 *
 *  @return		MLAN_STATUS_SUCCESS --success, otherwise fail
 */
mlan_status
wlan_misc_ioctl_host_cmd(IN pmlan_adapter pmadapter,
                         IN pmlan_ioctl_req pioctl_req)
{
    mlan_status ret = MLAN_STATUS_SUCCESS;
    mlan_private *pmpriv = pmadapter->priv[pioctl_req->bss_num];
    mlan_ds_misc_cfg *misc = MNULL;

    ENTER();

    misc = (mlan_ds_misc_cfg *) pioctl_req->pbuf;

    /* Send request to firmware */
    ret = wlan_prepare_cmd(pmpriv,
                           0,
                           0,
                           0,
                           (t_void *) pioctl_req,
                           (t_void *) & misc->param.hostcmd);
    if (ret == MLAN_STATUS_SUCCESS)
        ret = MLAN_STATUS_PENDING;

    LEAVE();
    return ret;
}

/**
 *  @brief Send function init/shutdown command to firmware
 *
 *  @param pmadapter	A pointer to mlan_adapter structure
 *  @param pioctl_req	A pointer to ioctl request buffer
 *
 *  @return		MLAN_STATUS_PENDING --success, otherwise fail
 */
mlan_status
wlan_misc_ioctl_init_shutdown(IN pmlan_adapter pmadapter,
                              IN pmlan_ioctl_req pioctl_req)
{
    mlan_private *pmpriv = pmadapter->priv[pioctl_req->bss_num];
    mlan_status ret = MLAN_STATUS_SUCCESS;
    mlan_ds_misc_cfg *misc_cfg = MNULL;
    t_u16 cmd;

    ENTER();

    misc_cfg = (mlan_ds_misc_cfg *) pioctl_req->pbuf;
    if (misc_cfg->param.func_init_shutdown == MLAN_FUNC_INIT)
        cmd = HostCmd_CMD_FUNC_INIT;
    else if (misc_cfg->param.func_init_shutdown == MLAN_FUNC_SHUTDOWN)
        cmd = HostCmd_CMD_FUNC_SHUTDOWN;
    else {
        PRINTM(MERROR, "Unsupported parameter\n");
        ret = MLAN_STATUS_FAILURE;
        goto exit;
    }

    /* Send command to firmware */
    ret = wlan_prepare_cmd(pmpriv,
                           cmd,
                           HostCmd_ACT_GEN_SET,
                           0, (t_void *) pioctl_req, MNULL);

    if (ret == MLAN_STATUS_SUCCESS)
        ret = MLAN_STATUS_PENDING;

  exit:
    LEAVE();
    return ret;
}

/** 
 *  @brief Get debug information
 *
 *  @param pmadapter	A pointer to mlan_adapter structure
 *  @param pioctl_req	A pointer to ioctl request buffer
 *
 *  @return		MLAN_STATUS_SUCCESS --success, otherwise fail
 */
mlan_status
wlan_get_info_debug_info(IN pmlan_adapter pmadapter,
                         IN pmlan_ioctl_req pioctl_req)
{
    pmlan_private pmpriv = pmadapter->priv[pioctl_req->bss_num];
    mlan_status ret = MLAN_STATUS_SUCCESS;
    mlan_ds_get_info *info;

    ENTER();

    info = (mlan_ds_get_info *) pioctl_req->pbuf;

    if (pioctl_req->action == MLAN_ACT_SET) {
        memcpy(pmpriv->wmm.packets_out, info->param.debug_info.packets_out,
               sizeof(pmpriv->wmm.packets_out));
        pmadapter->max_tx_buf_size =
            (t_u16) info->param.debug_info.max_tx_buf_size;
        pmadapter->tx_buf_size = (t_u16) info->param.debug_info.tx_buf_size;
        pmadapter->ps_mode = info->param.debug_info.ps_mode;
        pmadapter->ps_state = info->param.debug_info.ps_state;
        pmadapter->is_deep_sleep = info->param.debug_info.is_deep_sleep;
        pmadapter->pm_wakeup_card_req =
            info->param.debug_info.pm_wakeup_card_req;
        pmadapter->pm_wakeup_fw_try = info->param.debug_info.pm_wakeup_fw_try;
        pmadapter->is_hs_configured = info->param.debug_info.is_hs_configured;
        pmadapter->hs_activated = info->param.debug_info.hs_activated;

        pmadapter->dbg.num_cmd_host_to_card_failure =
            info->param.debug_info.num_cmd_host_to_card_failure;
        pmadapter->dbg.num_cmd_sleep_cfm_host_to_card_failure =
            info->param.debug_info.num_cmd_sleep_cfm_host_to_card_failure;
        pmadapter->dbg.num_tx_host_to_card_failure =
            info->param.debug_info.num_tx_host_to_card_failure;
        pmadapter->dbg.num_event_deauth =
            info->param.debug_info.num_event_deauth;
        pmadapter->dbg.num_event_disassoc =
            info->param.debug_info.num_event_disassoc;
        pmadapter->dbg.num_event_link_lost =
            info->param.debug_info.num_event_link_lost;
        pmadapter->dbg.num_cmd_deauth = info->param.debug_info.num_cmd_deauth;
        pmadapter->dbg.num_cmd_assoc_success =
            info->param.debug_info.num_cmd_assoc_success;
        pmadapter->dbg.num_cmd_assoc_failure =
            info->param.debug_info.num_cmd_assoc_failure;
        pmadapter->dbg.num_tx_timeout = info->param.debug_info.num_tx_timeout;
        pmadapter->dbg.num_cmd_timeout = info->param.debug_info.num_cmd_timeout;
        pmadapter->dbg.timeout_cmd_id = info->param.debug_info.timeout_cmd_id;
        pmadapter->dbg.timeout_cmd_act = info->param.debug_info.timeout_cmd_act;
        memcpy(pmadapter->dbg.last_cmd_id, info->param.debug_info.last_cmd_id,
               sizeof(pmadapter->dbg.last_cmd_id));
        memcpy(pmadapter->dbg.last_cmd_act, info->param.debug_info.last_cmd_act,
               sizeof(pmadapter->dbg.last_cmd_act));
        pmadapter->dbg.last_cmd_index = info->param.debug_info.last_cmd_index;
        memcpy(pmadapter->dbg.last_cmd_resp_id,
               info->param.debug_info.last_cmd_resp_id,
               sizeof(pmadapter->dbg.last_cmd_resp_id));
        pmadapter->dbg.last_cmd_resp_index =
            info->param.debug_info.last_cmd_resp_index;
        memcpy(pmadapter->dbg.last_event, info->param.debug_info.last_event,
               sizeof(pmadapter->dbg.last_event));
        pmadapter->dbg.last_event_index =
            info->param.debug_info.last_event_index;

        pmadapter->data_sent = info->param.debug_info.data_sent;
        pmadapter->cmd_sent = info->param.debug_info.cmd_sent;
        pmadapter->mp_rd_bitmap = info->param.debug_info.mp_rd_bitmap;
        pmadapter->mp_wr_bitmap = info->param.debug_info.mp_wr_bitmap;
        pmadapter->curr_rd_port = info->param.debug_info.curr_rd_port;
        pmadapter->curr_wr_port = info->param.debug_info.curr_wr_port;
        pmadapter->cmd_resp_received = info->param.debug_info.cmd_resp_received;
    } else {                    /* MLAN_ACT_GET */
        memcpy(info->param.debug_info.packets_out, pmpriv->wmm.packets_out,
               sizeof(pmpriv->wmm.packets_out));
        info->param.debug_info.max_tx_buf_size =
            (t_u32) pmadapter->max_tx_buf_size;
        info->param.debug_info.tx_buf_size = (t_u32) pmadapter->tx_buf_size;
        info->param.debug_info.rx_tbl_num =
            wlan_get_rxreorder_tbl(pmpriv, info->param.debug_info.rx_tbl);
        info->param.debug_info.tx_tbl_num =
            wlan_get_txbastream_tbl(pmpriv, info->param.debug_info.tx_tbl);
        info->param.debug_info.ps_mode = pmadapter->ps_mode;
        info->param.debug_info.ps_state = pmadapter->ps_state;
        info->param.debug_info.is_deep_sleep = pmadapter->is_deep_sleep;

        info->param.debug_info.pm_wakeup_card_req =
            pmadapter->pm_wakeup_card_req;
        info->param.debug_info.pm_wakeup_fw_try = pmadapter->pm_wakeup_fw_try;
        info->param.debug_info.is_hs_configured = pmadapter->is_hs_configured;
        info->param.debug_info.hs_activated = pmadapter->hs_activated;

        info->param.debug_info.num_cmd_host_to_card_failure
            = pmadapter->dbg.num_cmd_host_to_card_failure;
        info->param.debug_info.num_cmd_sleep_cfm_host_to_card_failure
            = pmadapter->dbg.num_cmd_sleep_cfm_host_to_card_failure;
        info->param.debug_info.num_tx_host_to_card_failure
            = pmadapter->dbg.num_tx_host_to_card_failure;
        info->param.debug_info.num_event_deauth =
            pmadapter->dbg.num_event_deauth;
        info->param.debug_info.num_event_disassoc =
            pmadapter->dbg.num_event_disassoc;
        info->param.debug_info.num_event_link_lost =
            pmadapter->dbg.num_event_link_lost;
        info->param.debug_info.num_cmd_deauth = pmadapter->dbg.num_cmd_deauth;
        info->param.debug_info.num_cmd_assoc_success =
            pmadapter->dbg.num_cmd_assoc_success;
        info->param.debug_info.num_cmd_assoc_failure =
            pmadapter->dbg.num_cmd_assoc_failure;
        info->param.debug_info.num_tx_timeout = pmadapter->dbg.num_tx_timeout;
        info->param.debug_info.num_cmd_timeout = pmadapter->dbg.num_cmd_timeout;
        info->param.debug_info.timeout_cmd_id = pmadapter->dbg.timeout_cmd_id;
        info->param.debug_info.timeout_cmd_act = pmadapter->dbg.timeout_cmd_act;
        memcpy(info->param.debug_info.last_cmd_id, pmadapter->dbg.last_cmd_id,
               sizeof(pmadapter->dbg.last_cmd_id));
        memcpy(info->param.debug_info.last_cmd_act, pmadapter->dbg.last_cmd_act,
               sizeof(pmadapter->dbg.last_cmd_act));
        info->param.debug_info.last_cmd_index = pmadapter->dbg.last_cmd_index;
        memcpy(info->param.debug_info.last_cmd_resp_id,
               pmadapter->dbg.last_cmd_resp_id,
               sizeof(pmadapter->dbg.last_cmd_resp_id));
        info->param.debug_info.last_cmd_resp_index =
            pmadapter->dbg.last_cmd_resp_index;
        memcpy(info->param.debug_info.last_event, pmadapter->dbg.last_event,
               sizeof(pmadapter->dbg.last_event));
        info->param.debug_info.last_event_index =
            pmadapter->dbg.last_event_index;

        info->param.debug_info.mp_rd_bitmap = pmadapter->mp_rd_bitmap;
        info->param.debug_info.mp_wr_bitmap = pmadapter->mp_wr_bitmap;
        info->param.debug_info.curr_rd_port = pmadapter->curr_rd_port;
        info->param.debug_info.curr_wr_port = pmadapter->curr_wr_port;
        info->param.debug_info.data_sent = pmadapter->data_sent;
        info->param.debug_info.cmd_sent = pmadapter->cmd_sent;
        info->param.debug_info.cmd_resp_received = pmadapter->cmd_resp_received;
    }

    pioctl_req->data_read_written =
        sizeof(mlan_debug_info) + MLAN_SUB_COMMAND_SIZE;

    LEAVE();
    return ret;
}

/**
 *  @brief This function wakes up the card.
 *
 *  @param pmadapter		A pointer to mlan_adapter structure
 *
 *  @return			MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_pm_wakeup_card(IN pmlan_adapter pmadapter)
{
    mlan_status ret = MLAN_STATUS_SUCCESS;
    pmlan_callbacks pcb = &pmadapter->callbacks;

    ENTER();
    PRINTM(MCMND, "Wakeup device...\n");
    ret =
        pcb->moal_write_reg(pmadapter->pmoal_handle, CONFIGURATION_REG,
                            HOST_POWER_UP);

    LEAVE();
    return ret;
}

/**
 *  @brief This function resets the PM setting of the card.
 *
 *  @param pmadapter		A pointer to mlan_adapter structure
 *
 *  @return			MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_pm_reset_card(IN pmlan_adapter pmadapter)
{
    mlan_status ret = MLAN_STATUS_SUCCESS;
    pmlan_callbacks pcb = &pmadapter->callbacks;

    ENTER();

    ret = pcb->moal_write_reg(pmadapter->pmoal_handle, CONFIGURATION_REG, 0);

    LEAVE();
    return ret;
}

/** 
 *  @brief This function allocates a mlan_buffer.
 *
 *  @param pcb        Pointer to mlan_callbacks
 *  @param data_len   Data length
 *
 *  @return           mlan_buffer pointer or MNULL
 */
pmlan_buffer
wlan_alloc_mlan_buffer(pmlan_callbacks pcb, t_u32 data_len)
{
    mlan_status ret = MLAN_STATUS_SUCCESS;
    pmlan_buffer pmbuf = MNULL;
    t_u32 buf_size = sizeof(mlan_buffer) + data_len;

    ENTER();

    ret = pcb->moal_malloc(buf_size, (t_u8 **) & pmbuf);
    if ((ret != MLAN_STATUS_SUCCESS) || !pmbuf) {
        pmbuf = MNULL;
        goto exit;
    }

    memset(pmbuf, 0, sizeof(mlan_buffer));

    pmbuf->pdesc = MNULL;
    pmbuf->pbuf = (t_u8 *) pmbuf + sizeof(mlan_buffer);
    pmbuf->data_offset = 0;
    pmbuf->data_len = data_len;

  exit:
    LEAVE();
    return pmbuf;
}

/** 
 *  @brief This function frees a mlan_buffer.
 *
 *  @param pcb        Pointer to mlan_callbacks
 *  @param pmbuf      Pointer to mlan_buffer
 *
 *  @return           N/A
 */
t_void
wlan_free_mlan_buffer(pmlan_callbacks pcb, pmlan_buffer pmbuf)
{
    ENTER();

    if (pcb && pmbuf)
        pcb->moal_mfree((t_u8 *) pmbuf);

    LEAVE();
    return;
}

/** 
 *  @brief Delay function implementation
 *   
 *  @param pmadapter        A pointer to mlan_adapter structure
 *  @param delay            Delay value
 *  @param u                Units of delay (sec, msec or usec)
 */
t_void
wlan_delay_func(mlan_adapter * pmadapter, t_u32 delay, t_delay_unit u)
{
    t_u32 now_tv_sec, now_tv_usec;
    t_u32 upto_tv_sec, upto_tv_usec;
    pmlan_callbacks pcb = &pmadapter->callbacks;

    pcb->moal_get_system_time(&upto_tv_sec, &upto_tv_usec);

    switch (u) {
    case SEC:
        upto_tv_sec += delay;
        break;
    case MSEC:
        delay *= 1000;
    case USEC:
        upto_tv_sec += (delay / 1000000);
        upto_tv_usec += (delay % 1000000);
        break;
    }

    do {
        pcb->moal_get_system_time(&now_tv_sec, &now_tv_usec);
        if (now_tv_sec > upto_tv_sec)
            return;

        if ((now_tv_sec == upto_tv_sec) && (now_tv_usec >= upto_tv_usec))
            return;
    } while (MTRUE);

    return;
}
