/**
 * @file mlan_mac80211.c
 *
 *  @brief This file contains the handling of CMD/EVENT in MLAN
 *
 *
 *  Copyright 2018-2021 NXP
 *
 *  NXP CONFIDENTIAL
 *  The source code contained or described herein and all documents related to
 *  the source code (Materials) are owned by NXP, its
 *  suppliers and/or its licensors. Title to the Materials remains with NXP,
 *  its suppliers and/or its licensors. The Materials contain
 *  trade secrets and proprietary and confidential information of NXP, its
 *  suppliers and/or its licensors. The Materials are protected by worldwide
 *  copyright and trade secret laws and treaty provisions. No part of the
 *  Materials may be used, copied, reproduced, modified, published, uploaded,
 *  posted, transmitted, distributed, or disclosed in any way without NXP's
 *  prior express written permission.
 *
 *  No license under any patent, copyright, trade secret or other intellectual
 *  property right is granted to or conferred upon you by disclosure or delivery
 *  of the Materials, either expressly, by implication, inducement, estoppel or
 *  otherwise. Any license under such intellectual property rights must be
 *  express and approved by NXP in writing.
 *
 */

/*************************************************************
Change Log:
    10/08/2018: initial version
************************************************************/
#include "mlan.h"
#ifdef STA_SUPPORT
#include "mlan_join.h"
#endif
#include "mlan_util.h"
#include "mlan_fw.h"
#include "mlan_main.h"
#include "mlan_wmm.h"
#include "mlan_11n.h"
#include "mlan_11ac.h"
#include "mlan_11h.h"
#ifdef PCIE
#include "mlan_pcie.h"
#endif /* PCIE */
#include "mlan_init.h"
/********************************************************
	    Local Variables
********************************************************/

/*******************************************************
	    Global Variables
********************************************************/

/********************************************************
	    Local Functions
********************************************************/

/**
 *  @brief  This function issues commands to initialize firmware
 *
 *  @param priv         A pointer to mlan_private structure
 *  @param first_bss    flag for first BSS
 *
 *  @return		MLAN_STATUS_PENDING or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_ops_mac80211_init_cmd(t_void *priv, t_u8 first_bss)
{
	pmlan_private pmpriv = (pmlan_private)priv;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_11n_amsdu_aggr_ctrl amsdu_aggr_ctrl;

	ENTER();

	if (!pmpriv) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	if (first_bss == MTRUE)
		wlan_adapter_init_cmd(pmpriv->adapter);

	memset(pmpriv->adapter, &amsdu_aggr_ctrl, 0, sizeof(amsdu_aggr_ctrl));
	amsdu_aggr_ctrl.enable = MLAN_ACT_ENABLE;
	/* Send request to firmware */
	ret = wlan_prepare_cmd(pmpriv, HostCmd_CMD_AMSDU_AGGR_CTRL,
			       HostCmd_ACT_GEN_SET, 0, MNULL,
			       (t_void *)&amsdu_aggr_ctrl);
	if (ret) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	/* MAC Control must be the last command in init_fw */
	/* set MAC Control */
	ret = wlan_prepare_cmd(pmpriv, HostCmd_CMD_MAC_CONTROL,
			       HostCmd_ACT_GEN_SET, 0, MNULL,
			       &pmpriv->curr_pkt_filter);
	if (ret) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	/** set last_init_cmd */
	pmpriv->adapter->last_init_cmd = HostCmd_CMD_MAC_CONTROL;

	if (first_bss == MFALSE) {
		/* Get MAC address */
		ret = wlan_prepare_cmd(pmpriv, HostCmd_CMD_802_11_MAC_ADDRESS,
				       HostCmd_ACT_GEN_GET, 0, MNULL, MNULL);
		if (ret) {
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		pmpriv->adapter->last_init_cmd = HostCmd_CMD_802_11_MAC_ADDRESS;
	}

	ret = MLAN_STATUS_PENDING;
done:
	LEAVE();
	return ret;
}

#if defined(MAC80211_SUPPORT_UAP) || defined(MAC80211_SUPPORT_MESH)
/**
 *  @brief Set mac80211 beacon
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pioctl_req   A pointer to ioctl request buffer
 *
 *  @return             MLAN_STATUS_SUCCESS --success, otherwise fail
 */
static mlan_status
wlan_set_beacon_cfg(pmlan_adapter pmadapter, pmlan_ioctl_req pioctl_req)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_mac80211_cfg *mac80211_cfg = MNULL;
	t_u16 cmd_action = 0;
	mlan_private *pmpriv = pmadapter->priv[pioctl_req->bss_index];

	ENTER();

	mac80211_cfg = (mlan_ds_mac80211_cfg *) pioctl_req->pbuf;
	/* Only beacon set is implemented */
	cmd_action = HostCmd_ACT_GEN_SET;

	/* Send request to firmware */
	ret = wlan_prepare_cmd(pmpriv, Hostcmd_CMD_DOT11S_BEACON_CFG,
			       cmd_action, 0, (t_void *)pioctl_req,
			       &mac80211_cfg->params.bcn_cfg);

	if (ret == MLAN_STATUS_SUCCESS)
		ret = MLAN_STATUS_PENDING;
	LEAVE();
	return ret;
}
#endif

/**
 *  @brief this function sends the station states to firmware
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pioctl_req   A pointer to ioctl request buffer
 *
 *  @return             MLAN_STATUS_SUCCESS --success, otherwise fail
 */
static mlan_status
wlan_sta_state(pmlan_adapter pmadapter, pmlan_ioctl_req pioctl_req)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_mac80211_cfg *mac80211_cfg = MNULL;
	t_u16 cmd_action = 0;
	mlan_private *pmpriv = pmadapter->priv[pioctl_req->bss_index];

	ENTER();

	mac80211_cfg = (mlan_ds_mac80211_cfg *) pioctl_req->pbuf;
	/* Only beacon set is implemented */
	cmd_action = HostCmd_ACT_GEN_SET;

	/* Send request to firmware */
	ret = wlan_prepare_cmd(pmpriv, Hostcmd_CMD_DOT11S_STA_STATE, cmd_action,
			       0, (t_void *)pioctl_req,
			       &mac80211_cfg->params.sta_state);

	if (ret == MLAN_STATUS_SUCCESS)
		ret = MLAN_STATUS_PENDING;
	LEAVE();
	return ret;
}

/**
 *  @brief this function sends ampdu configuration to firmware
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pioctl_req   A pointer to ioctl request buffer
 *
 *  @return             MLAN_STATUS_SUCCESS --success, otherwise fail
 */
static mlan_status
wlan_ampdu_cfg_txrx(pmlan_adapter pmadapter, pmlan_ioctl_req pioctl_req)
{
	mlan_private *priv = pmadapter->priv[pioctl_req->bss_index];
	mlan_ds_mac80211_cfg *mac80211_cfg =
		(mlan_ds_mac80211_cfg *) pioctl_req->pbuf;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	mac80211_cfg = (mlan_ds_mac80211_cfg *) pioctl_req->pbuf;

	if (mac80211_cfg->params.ampdu_cfg.is_tx) {
		ret = wlan_prepare_cmd(priv, HostCmd_CMD_11N_ADDBA_REQ, 0, 0,
				       (t_void *)pioctl_req,
				       &mac80211_cfg->params.ampdu_cfg);
	} else {
		ret = wlan_prepare_cmd(priv, HostCmd_CMD_11N_ADDBA_RSP, 0, 0,
				       (t_void *)pioctl_req,
				       &mac80211_cfg->params.ampdu_cfg);
	}

	if (ret == MLAN_STATUS_SUCCESS)
		ret = MLAN_STATUS_PENDING;

	LEAVE();
	return ret;
}

/**
 *  @brief this function deleted ampdu configuration in firmware
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pioctl_req   A pointer to ioctl request buffer
 *
 *  @return             MLAN_STATUS_SUCCESS --success, otherwise fail
 */
static mlan_status
wlan_ampdu_delete(pmlan_adapter pmadapter, pmlan_ioctl_req pioctl_req)
{
	mlan_private *priv = pmadapter->priv[pioctl_req->bss_index];
	mlan_ds_mac80211_cfg *mac80211_cfg =
		(mlan_ds_mac80211_cfg *) pioctl_req->pbuf;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	mac80211_cfg = (mlan_ds_mac80211_cfg *) pioctl_req->pbuf;

	ret = wlan_prepare_cmd(priv, HostCmd_CMD_11N_DELBA, 0, 0,
			       (t_void *)pioctl_req,
			       &mac80211_cfg->params.ampdu_cfg);

	if (ret == MLAN_STATUS_SUCCESS)
		ret = MLAN_STATUS_PENDING;

	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get MAC address for MAC80211 interface
 *
 *  @param pmadapter	A pointer to mlan_adapter structure
 *  @param pioctl_req	A pointer to ioctl request buffer
 *
 *  @return		MLAN_STATUS_PENDING --success, otherwise fail
 */
static mlan_status
wlan_mac80211_ioctl_mac_address(pmlan_adapter pmadapter,
				pmlan_ioctl_req pioctl_req)
{
	mlan_private *pmpriv = pmadapter->priv[pioctl_req->bss_index];
	mlan_ds_mac80211_cfg *mac80211_cfg =
		(mlan_ds_mac80211_cfg *) pioctl_req->pbuf;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u16 cmd_action = 0;

	ENTER();

	if (pioctl_req->action == MLAN_ACT_SET) {
		memcpy_ext(pmadapter, pmpriv->curr_addr,
			   &mac80211_cfg->params.mac_addr,
			   sizeof(mac80211_cfg->params.mac_addr),
			   sizeof(pmpriv->curr_addr));
		cmd_action = HostCmd_ACT_GEN_SET;
	} else {
		cmd_action = HostCmd_ACT_GEN_GET;
	}
	/* Send request to firmware */
	ret = wlan_prepare_cmd(pmpriv, HostCmd_CMD_802_11_MAC_ADDRESS,
			       cmd_action, 0, (t_void *)pioctl_req, MNULL);

	if (ret == MLAN_STATUS_SUCCESS)
		ret = MLAN_STATUS_PENDING;

	LEAVE();
	return ret;
}

/**
 *  @brief mac80211 based mesh handler
 *
 *  @param pmadapter	A pointer to mlan_adapter structure
 *  @param pioctl_req	A pointer to ioctl request buffer
 *
 *  @return		MLAN_STATUS_SUCCESS --success, otherwise fail
 */
static mlan_status
wlan_mac80211_ioctl(pmlan_adapter pmadapter, pmlan_ioctl_req pioctl_req)
{
	mlan_status status = MLAN_STATUS_SUCCESS;
	mlan_ds_mac80211_cfg *mac80211_cfg = MNULL;

	ENTER();

	if (pioctl_req->buf_len < sizeof(mlan_ds_mac80211_cfg)) {
		PRINTM(MWARN,
		       "MLAN IOCTL information buffer length is too short.\n");
		pioctl_req->data_read_written = 0;
		pioctl_req->buf_len_needed = sizeof(mlan_ds_mac80211_cfg);
		pioctl_req->status_code = MLAN_ERROR_INVALID_PARAMETER;
		LEAVE();
		return MLAN_STATUS_RESOURCE;
	}
	mac80211_cfg = (mlan_ds_mac80211_cfg *) pioctl_req->pbuf;
	switch (mac80211_cfg->sub_command) {
#if defined(MAC80211_SUPPORT_UAP) || defined(MAC80211_SUPPORT_MESH)
	case MLAN_OID_SET_BEACON_CFG:
		status = wlan_set_beacon_cfg(pmadapter, pioctl_req);
		break;
#endif
	case MLAN_OID_STA_STATE:
		status = wlan_sta_state(pmadapter, pioctl_req);
		break;
	case MLAN_OID_AMPDU_CFG_TXRX:
		status = wlan_ampdu_cfg_txrx(pmadapter, pioctl_req);
		break;
	case MLAN_OID_AMPDU_DELETE:
		status = wlan_ampdu_delete(pmadapter, pioctl_req);
		break;
	case MLAN_OID_MAC_ADDR:
		status = wlan_mac80211_ioctl_mac_address(pmadapter, pioctl_req);
		break;
	default:
		pioctl_req->status_code = MLAN_ERROR_IOCTL_INVALID;
		status = MLAN_STATUS_FAILURE;
		break;
	}
	LEAVE();
	return status;
}

/**
 *  @brief Set key
 *
 *  @param pmadapter	A pointer to mlan_adapter structure
 *  @param pioctl_req	A pointer to ioctl request buffer
 *
 *  @return		MLAN_STATUS_PENDING --success, otherwise fail
 */
static mlan_status
wlan_sec_ioctl_mac80211_set_key(pmlan_adapter pmadapter,
				pmlan_ioctl_req pioctl_req)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_private *pmpriv = pmadapter->priv[pioctl_req->bss_index];
	mlan_ds_sec_cfg *sec = MNULL;

	ENTER();

	sec = (mlan_ds_sec_cfg *)pioctl_req->pbuf;
	/* Current driver only supports key length of up to 32 bytes */
	if (sec->param.encrypt_key.key_len > MLAN_MAX_KEY_LENGTH) {
		PRINTM(MERROR, "Key length is incorrect\n");
		pioctl_req->status_code = MLAN_ERROR_INVALID_PARAMETER;
		ret = MLAN_STATUS_FAILURE;
		goto exit;
	}

	ret = wlan_prepare_cmd(pmpriv, HostCmd_CMD_802_11_KEY_MATERIAL,
			       HostCmd_ACT_GEN_SET, 0, (t_void *)pioctl_req,
			       &sec->param.encrypt_key);

	if (ret == MLAN_STATUS_SUCCESS)
		ret = MLAN_STATUS_PENDING;

exit:
	LEAVE();
	return ret;
}

/**
 *  @brief Set security key(s)
 *
 *  @param pmadapter	A pointer to mlan_adapter structure
 *  @param pioctl_req	A pointer to ioctl request buffer
 *
 *  @return		MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --success,
 * otherwise fail
 */
static mlan_status
wlan_sec_ioctl_mac80211_encrypt_key(pmlan_adapter pmadapter,
				    pmlan_ioctl_req pioctl_req)
{
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (pioctl_req->action == MLAN_ACT_SET) {
		status = wlan_sec_ioctl_mac80211_set_key(pmadapter, pioctl_req);
	} else {
		/* GET not supported */
		status = MLAN_STATUS_FAILURE;
	}

	LEAVE();
	return status;
}

/**
 *  @brief Security configuration handler
 *
 *  @param pmadapter	A pointer to mlan_adapter structure
 *  @param pioctl_req	A pointer to ioctl request buffer
 *
 *  @return		MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --success,
 * otherwise fail
 */
static mlan_status
wlan_sec_ioctl_mac80211(pmlan_adapter pmadapter, pmlan_ioctl_req pioctl_req)
{
	mlan_status status = MLAN_STATUS_SUCCESS;
	mlan_ds_sec_cfg *sec = MNULL;

	ENTER();

	if (pioctl_req->buf_len < sizeof(mlan_ds_sec_cfg)) {
		PRINTM(MWARN, "MLAN bss IOCTL length is too short.\n");
		pioctl_req->data_read_written = 0;
		pioctl_req->buf_len_needed = sizeof(mlan_ds_sec_cfg);
		pioctl_req->status_code = MLAN_ERROR_INVALID_PARAMETER;
		LEAVE();
		return MLAN_STATUS_RESOURCE;
	}

	sec = (mlan_ds_sec_cfg *)pioctl_req->pbuf;
	switch (sec->sub_command) {
	case MLAN_OID_SEC_CFG_ENCRYPT_KEY:
		status = wlan_sec_ioctl_mac80211_encrypt_key(pmadapter,
							     pioctl_req);
		break;
	default:
		pioctl_req->status_code = MLAN_ERROR_IOCTL_INVALID;
		status = MLAN_STATUS_FAILURE;
		break;
	}
	LEAVE();
	return status;
}

/**
 *  @brief Set power save configurations
 *
 *  @param pmadapter	A pointer to mlan_adapter structure
 *  @param pioctl_req	A pointer to ioctl request buffer
 *  @param ps_mode	Power save mode
 *
 *  @return		MLAN_STATUS_PENDING --success, otherwise fail
 */
static mlan_status
wlan_pm_ioctl_ps_mode(pmlan_adapter pmadapter,
		      pmlan_ioctl_req pioctl_req, t_u16 ps_mode)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_private *pmpriv = pmadapter->priv[pioctl_req->bss_index];
	t_u16 sub_cmd;

	ENTER();

	if (pioctl_req->action == MLAN_ACT_SET) {
		sub_cmd = (pmadapter->ps_mode == Wlan802_11PowerModePSP) ?
			EN_AUTO_PS : DIS_AUTO_PS;
		ret = wlan_prepare_cmd(pmpriv, HostCmd_CMD_802_11_PS_MODE_ENH,
				       sub_cmd, BITMAP_STA_PS,
				       (t_void *)pioctl_req, MNULL);
		if ((ret == MLAN_STATUS_SUCCESS) && (sub_cmd == DIS_AUTO_PS)) {
			ret = wlan_prepare_cmd(pmpriv,
					       HostCmd_CMD_802_11_PS_MODE_ENH,
					       GET_PS, 0, MNULL, MNULL);
		}
	} else {
		ret = wlan_prepare_cmd(pmpriv, HostCmd_CMD_802_11_PS_MODE_ENH,
				       GET_PS, 0, (t_void *)pioctl_req, MNULL);
	}

	if (ret == MLAN_STATUS_SUCCESS)
		ret = MLAN_STATUS_PENDING;

	LEAVE();
	return ret;
}

/**
 *  @brief Power save command handler
 *
 *  @param pmadapter	A pointer to mlan_adapter structure
 *  @param pioctl_req	A pointer to ioctl request buffer
 *
 *  @return		MLAN_STATUS_SUCCESS --success, otherwise fail
 */
static mlan_status
wlan_pm_ioctl(pmlan_adapter pmadapter, pmlan_ioctl_req pioctl_req)
{
	mlan_status status = MLAN_STATUS_SUCCESS;
	mlan_ds_pm_cfg *pm = MNULL;

	ENTER();

	if (pioctl_req->buf_len < sizeof(mlan_ds_pm_cfg)) {
		PRINTM(MWARN, "MLAN bss IOCTL length is too short.\n");
		pioctl_req->data_read_written = 0;
		pioctl_req->buf_len_needed = sizeof(mlan_ds_pm_cfg);
		LEAVE();
		return MLAN_STATUS_RESOURCE;
	}
	pm = (mlan_ds_pm_cfg *)pioctl_req->pbuf;
	switch (pm->sub_command) {
	case MLAN_OID_PM_CFG_IEEE_PS:
		switch (pioctl_req->action) {
		case MLAN_ACT_SET:
			/**Block ieee power save disable command when bt coex
			 * enable*/
			if (pmadapter->coex_scan && !pm->param.ps_mode)
				break;
			if (pm->param.ps_mode)
				pmadapter->ps_mode = Wlan802_11PowerModePSP;
			else
				pmadapter->ps_mode = Wlan802_11PowerModeCAM;
			status = wlan_pm_ioctl_ps_mode(pmadapter, pioctl_req,
						       pmadapter->ps_mode);
			break;
		case MLAN_ACT_GET:
			status = wlan_pm_ioctl_ps_mode(pmadapter, pioctl_req,
						       pmadapter->ps_mode);
			break;
		default:
			pioctl_req->status_code = MLAN_ERROR_IOCTL_INVALID;
			status = MLAN_STATUS_FAILURE;
			break;
		}
		break;
	}
	LEAVE();
	return status;
}

/**
 *  @brief MAC80211 ioctl handler
 *
 *  @param adapter  A pointer to mlan_adapter structure
 *  @param pioctl_req	A pointer to ioctl request buffer
 *
 *  @return		MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING --success,
 * otherwise fail
 */
mlan_status
wlan_ops_mac80211_ioctl(t_void *adapter, pmlan_ioctl_req pioctl_req)
{
	pmlan_adapter pmadapter = (pmlan_adapter)adapter;
	mlan_status status = MLAN_STATUS_SUCCESS;
	mlan_private *pmpriv = pmadapter->priv[pioctl_req->bss_index];
	mlan_ds_get_info *pget_info = MNULL;
	mlan_ds_misc_cfg *misc = MNULL;
	mlan_ds_radio_cfg *radiocfg = MNULL;
	mlan_ds_snmp_mib *snmp = MNULL;

	ENTER();

	switch (pioctl_req->req_id) {
	case MLAN_IOCTL_MAC80211:
		status = wlan_mac80211_ioctl(pmadapter, pioctl_req);
		break;
	case MLAN_IOCTL_MISC_CFG:
		misc = (mlan_ds_misc_cfg *)pioctl_req->pbuf;
		if (misc->sub_command == MLAN_OID_MISC_RX_MGMT_IND)
			status = wlan_reg_rx_mgmt_ind(pmadapter, pioctl_req);
		else if (misc->sub_command == MLAN_OID_MISC_INIT_SHUTDOWN)
			status = wlan_misc_ioctl_init_shutdown(pmadapter,
							       pioctl_req);
		else if (misc->sub_command == MLAN_OID_MISC_NET_MONITOR)
			status = wlan_misc_ioctl_net_monitor(pmadapter,
							     pioctl_req);
		break;
	case MLAN_IOCTL_RADIO_CFG:
		radiocfg = (mlan_ds_radio_cfg *)pioctl_req->pbuf;
		if (radiocfg->sub_command == MLAN_OID_REMAIN_CHAN_CFG)
			status = wlan_radio_ioctl_remain_chan_cfg(pmadapter,
								  pioctl_req);
		break;
	case MLAN_IOCTL_PM_CFG:
		status = wlan_pm_ioctl(pmadapter, pioctl_req);
		break;
	case MLAN_IOCTL_GET_INFO:
		pget_info = (mlan_ds_get_info *)pioctl_req->pbuf;
		if (pget_info->sub_command == MLAN_OID_GET_FW_INFO) {
			pioctl_req->data_read_written =
				sizeof(mlan_fw_info) + MLAN_SUB_COMMAND_SIZE;
			pget_info->param.fw_info.fw_ver =
				pmadapter->fw_release_number;
			pget_info->param.fw_info.hotfix_version =
				pmadapter->fw_hotfix_ver;
			memcpy_ext(pmadapter,
				   &pget_info->param.fw_info.mac_addr,
				   pmpriv->curr_addr, MLAN_MAC_ADDR_LENGTH,
				   MLAN_MAC_ADDR_LENGTH);
			pget_info->param.fw_info.fw_bands = pmadapter->fw_bands;
			pget_info->param.fw_info.region_code =
				pmadapter->region_code;
			pget_info->param.fw_info.ecsa_enable =
				pmadapter->ecsa_enable;
			pget_info->param.fw_info.getlog_enable =
				pmadapter->getlog_enable;
			pget_info->param.fw_info.hw_dev_mcs_support =
				pmadapter->hw_dev_mcs_support;
			pget_info->param.fw_info.hw_dot_11n_dev_cap =
				pmadapter->hw_dot_11n_dev_cap;
			pget_info->param.fw_info.usr_dev_mcs_support =
				pmpriv->usr_dev_mcs_support;
			pget_info->param.fw_info.hw_dot_11ac_mcs_support =
				pmadapter->hw_dot_11ac_mcs_support;
			pget_info->param.fw_info.hw_dot_11ac_dev_cap =
				pmadapter->hw_dot_11ac_dev_cap;
			pget_info->param.fw_info.usr_dot_11ac_dev_cap_bg =
				pmpriv->usr_dot_11ac_dev_cap_bg;
			pget_info->param.fw_info.usr_dot_11ac_mcs_support =
				pmpriv->usr_dot_11ac_mcs_support;
			pget_info->param.fw_info.usr_dot_11ac_dev_cap_a =
				pmpriv->usr_dot_11ac_dev_cap_a;
			pget_info->param.fw_info.fw_supplicant_support =
				IS_FW_SUPPORT_SUPPLICANT(pmadapter) ? 0x01 :
				0x00;
			pget_info->param.fw_info.antinfo = pmadapter->antinfo;
			pget_info->param.fw_info.max_ap_assoc_sta =
				pmadapter->max_sta_conn;
			pget_info->param.fw_info.fw_roaming_support =
				(pmadapter->fw_cap_info & FW_ROAMING_SUPPORT) ?
				0x01 : 0x00;
		}
		break;
	case MLAN_IOCTL_11N_CFG:
		status = wlan_11n_cfg_ioctl(pmadapter, pioctl_req);
		break;
	case MLAN_IOCTL_11AC_CFG:
		status = wlan_11ac_cfg_ioctl(pmadapter, pioctl_req);
		break;
	case MLAN_IOCTL_SEC_CFG:
		status = wlan_sec_ioctl_mac80211(pmadapter, pioctl_req);
		break;
	case MLAN_IOCTL_SNMP_MIB:
		snmp = (mlan_ds_snmp_mib *)pioctl_req->pbuf;
		if (snmp->sub_command == MLAN_OID_SNMP_MIB_RTS_THRESHOLD) {
			status = wlan_prepare_cmd(pmpriv,
						  HostCmd_CMD_802_11_SNMP_MIB,
						  pioctl_req->action,
						  RtsThresh_i,
						  (t_void *)pioctl_req,
						  &snmp->param.rts_threshold);
			if (status == MLAN_STATUS_SUCCESS) {
				status = MLAN_STATUS_PENDING;
			}
		}
		break;
	default:
		pioctl_req->status_code = MLAN_ERROR_IOCTL_INVALID;
		status = MLAN_STATUS_FAILURE;
		break;
	}
	LEAVE();
	return status;
}

/**
 *  @brief This function handles events generated by firmware
 *
 *  @param priv A pointer to mlan_private structure
 *
 *  @return     MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_ops_mac80211_process_event(t_void *priv)
{
	pmlan_private pmpriv = (pmlan_private)priv;
	pmlan_adapter pmadapter = pmpriv->adapter;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u32 eventcause = pmadapter->event_cause;
	pmlan_buffer pmbuf = pmadapter->pmlan_buffer_event;
	mlan_event *pevent = MNULL;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u8 *event_buf = MNULL;

	ENTER();

	if (!pmbuf) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	/* Event length check */
	if ((pmbuf->data_len - sizeof(eventcause)) > MAX_EVENT_SIZE) {
		pmbuf->status_code = MLAN_ERROR_PKT_SIZE_INVALID;
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	/* Allocate memory for event buffer */
	ret = pcb->moal_malloc(pmadapter->pmoal_handle,
			       MAX_EVENT_SIZE + sizeof(mlan_event),
			       MLAN_MEM_DEF, &event_buf);
	if ((ret != MLAN_STATUS_SUCCESS) || !event_buf) {
		PRINTM(MERROR, "Could not allocate buffer for event buf\n");
		if (pmbuf)
			pmbuf->status_code = MLAN_ERROR_NO_MEM;
		goto done;
	}
	pevent = (pmlan_event)event_buf;
	memset(pmadapter, event_buf, 0, MAX_EVENT_SIZE);

	if (eventcause != EVENT_PS_SLEEP && eventcause != EVENT_PS_AWAKE &&
	    pmbuf->data_len > sizeof(eventcause))
		DBG_HEXDUMP(MEVT_D, "EVENT", pmbuf->pbuf + pmbuf->data_offset,
			    pmbuf->data_len);

	switch (eventcause) {
	case EVENT_VDLL_IND:
		wlan_process_vdll_event(pmpriv, pmbuf);
		break;
	case EVENT_FW_DEBUG_INFO:
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_id = MLAN_EVENT_ID_FW_DEBUG_INFO;
		pevent->event_len = pmbuf->data_len - sizeof(eventcause);
		memcpy_ext(pmadapter, (t_u8 *)pevent->event_buf,
			   pmbuf->pbuf + pmbuf->data_offset +
			   sizeof(eventcause),
			   pevent->event_len, pevent->event_len);
		PRINTM(MEVENT, "EVENT: FW Debug Info %s\n",
		       (t_u8 *)pevent->event_buf);
		wlan_recv_event(pmpriv, pevent->event_id, pevent);
		break;
	case EVENT_PS_SLEEP:
		PRINTM(MINFO, "EVENT: SLEEP\n");
		PRINTM(MEVENT, "_");

		/* Handle unexpected PS SLEEP event */
		if (pmadapter->ps_state == PS_STATE_SLEEP_CFM)
			break;
		pmadapter->ps_state = PS_STATE_PRE_SLEEP;

		wlan_check_ps_cond(pmadapter);
		break;

	case EVENT_PS_AWAKE:
		PRINTM(MINFO, "EVENT: AWAKE\n");
		PRINTM(MEVENT, "|");
		if (!pmadapter->pps_uapsd_mode && pmpriv->media_connected &&
		    (pmpriv->port_open || !pmpriv->port_ctrl_mode) &&
		    pmadapter->sleep_period.period) {
			pmadapter->pps_uapsd_mode = MTRUE;
			PRINTM(MEVENT, "PPS/UAPSD mode activated\n");
		}
		/* Handle unexpected PS AWAKE event */
		if (pmadapter->ps_state == PS_STATE_SLEEP_CFM)
			break;
		pmadapter->tx_lock_flag = MFALSE;
		if (pmadapter->pps_uapsd_mode && pmadapter->gen_null_pkt) {
			if (MTRUE == wlan_check_last_packet_indication(pmpriv)) {
				if (!pmadapter->data_sent) {
					if (wlan_send_null_packet(pmpriv,
								  MRVDRV_TxPD_POWER_MGMT_NULL_PACKET
								  |
								  MRVDRV_TxPD_POWER_MGMT_LAST_PACKET)
					    == MLAN_STATUS_SUCCESS) {
						LEAVE();
						return MLAN_STATUS_SUCCESS;
					}
				}
			}
		}
		pmadapter->ps_state = PS_STATE_AWAKE;
		pmadapter->pm_wakeup_card_req = MFALSE;
		pmadapter->pm_wakeup_fw_try = MFALSE;
		break;
	case EVENT_TX_STATUS_REPORT:
		PRINTM(MINFO, "EVENT: TX_STATUS\n");
		pevent = (pmlan_event)event_buf;
		pevent->bss_index = pmpriv->bss_index;
		pevent->event_id = MLAN_EVENT_ID_FW_TX_STATUS;
		pevent->event_len = pmbuf->data_len;
		memcpy_ext(pmadapter, (t_u8 *)pevent->event_buf,
			   pmbuf->pbuf + pmbuf->data_offset, pevent->event_len,
			   pevent->event_len);
		wlan_recv_event(pmpriv, pevent->event_id, pevent);
		break;
	default:
		PRINTM(MEVENT, "EVENT: unknown event id: %#x\n", eventcause);
		wlan_recv_event(pmpriv, MLAN_EVENT_ID_FW_UNKNOWN, MNULL);
		break;
	}

done:
	if (event_buf)
		pcb->moal_mfree(pmadapter->pmoal_handle, event_buf);

	LEAVE();
	return ret;
}

/**
 *   @brief This function processes the received buffer
 *
 *   @param adapter A pointer to mlan_adapter
 *   @param pmbuf     A pointer to the received buffer
 *
 *   @return        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_ops_mac80211_process_rx_packet(t_void *adapter, pmlan_buffer pmbuf)
{
	pmlan_adapter pmadapter = (pmlan_adapter)adapter;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	RxPD *prx_pd = (RxPD *)(pmbuf->pbuf + pmbuf->data_offset);
	pmlan_private priv = pmadapter->priv[pmbuf->bss_index];

	ENTER();

	/* Endian conversion */
	endian_convert_RxPD(prx_pd);
	DBG_HEXDUMP(MDAT_D, "RxPD", (t_u8 *)prx_pd,
		    MIN(sizeof(RxPD), MAX_DATA_DUMP_LEN));
	DBG_HEXDUMP(MDAT_D, "Rx Payload",
		    ((t_u8 *)prx_pd + prx_pd->rx_pkt_offset),
		    MIN(prx_pd->rx_pkt_length, MAX_DATA_DUMP_LEN));

	priv->rxpd_rx_info = (t_u8)(prx_pd->rx_info >> 16);

	if (prx_pd->rx_pkt_type == PKT_TYPE_MRVL_MAC80211) {
		pmbuf->data_len -= prx_pd->rx_pkt_offset;
		pmbuf->data_offset += prx_pd->rx_pkt_offset;
		pmbuf->pparent = MNULL;
		ret = pmadapter->callbacks.moal_recv_packet(pmadapter->
							    pmoal_handle,
							    pmbuf);
		if (ret != MLAN_STATUS_PENDING)
			pmadapter->ops.data_complete(pmadapter, pmbuf, ret);
	}

	LEAVE();
	return ret;
}

/**
 *  @brief This function fill the txpd for tx packet
 *
 *  @param priv    A pointer to mlan_private structure
 *  @param pmbuf   A pointer to the mlan_buffer for process
 *
 *  @return        headptr or MNULL
 */
t_void *
wlan_ops_mac80211_process_txpd(t_void *priv, pmlan_buffer pmbuf)
{
	mlan_private *pmpriv = (mlan_private *)priv;
	pmlan_adapter pmadapter = pmpriv->adapter;
	TxPD *plocal_tx_pd;
	t_u8 *head_ptr = MNULL;
	t_u32 pkt_type;
	t_u32 tx_control;

	ENTER();

	if (!pmbuf->data_len) {
		PRINTM(MERROR, "STA Tx Error: Invalid packet length: %d\n",
		       pmbuf->data_len);
		pmbuf->status_code = MLAN_ERROR_PKT_SIZE_INVALID;
		goto done;
	}
	if (pmbuf->buf_type == MLAN_BUF_TYPE_RAW_DATA) {
		memcpy_ext(pmpriv->adapter, &pkt_type,
			   pmbuf->pbuf + pmbuf->data_offset, sizeof(pkt_type),
			   sizeof(pkt_type));
		memcpy_ext(pmpriv->adapter, &tx_control,
			   pmbuf->pbuf + pmbuf->data_offset + sizeof(pkt_type),
			   sizeof(tx_control), sizeof(tx_control));
		pmbuf->data_offset += sizeof(pkt_type) + sizeof(tx_control);
		pmbuf->data_len -= sizeof(pkt_type) + sizeof(tx_control);
	}

	if (pmbuf->data_offset <
	    (sizeof(TxPD) + pmpriv->intf_hr_len + DMA_ALIGNMENT)) {
		PRINTM(MERROR,
		       "not enough space for TxPD: headroom=%d pkt_len=%d, required=%d\n",
		       pmbuf->data_offset, pmbuf->data_len,
		       sizeof(TxPD) + pmpriv->intf_hr_len + DMA_ALIGNMENT);
		pmbuf->status_code = MLAN_ERROR_PKT_SIZE_INVALID;
		goto done;
	}

	/* head_ptr should be aligned */
	head_ptr = pmbuf->pbuf + pmbuf->data_offset - sizeof(TxPD) -
		pmpriv->intf_hr_len;
	head_ptr = (t_u8 *)((t_ptr)head_ptr & ~((t_ptr)(DMA_ALIGNMENT - 1)));

	plocal_tx_pd = (TxPD *)(head_ptr + pmpriv->intf_hr_len);
	memset(pmadapter, plocal_tx_pd, 0, sizeof(TxPD));
	/* Set the BSS number to TxPD */
	plocal_tx_pd->bss_num = GET_BSS_NUM(pmpriv);
	plocal_tx_pd->bss_type = pmpriv->bss_type;
	plocal_tx_pd->tx_pkt_length = (t_u16)pmbuf->data_len;
	plocal_tx_pd->mac80211_qos_ctrl = pmbuf->priority >> 8;
	plocal_tx_pd->priority = (t_u8)(pmbuf->priority & 0xf);
	plocal_tx_pd->pkt_delay_2ms =
		wlan_wmm_compute_driver_packet_delay(pmpriv, pmbuf);

	if (plocal_tx_pd->priority <
	    NELEMENTS(pmpriv->wmm.user_pri_pkt_tx_ctrl))
		/*
		 * Set the priority specific tx_control field, setting of 0 will
		 *   cause the default value to be used later in this function
		 */
		plocal_tx_pd->tx_control =
			pmpriv->wmm.user_pri_pkt_tx_ctrl[plocal_tx_pd->
							 priority];
	if (pmadapter->pps_uapsd_mode) {
		if (MTRUE == wlan_check_last_packet_indication(pmpriv)) {
			pmadapter->tx_lock_flag = MTRUE;
			plocal_tx_pd->flags =
				MRVDRV_TxPD_POWER_MGMT_LAST_PACKET;
		}
	}
	/* Offset of actual data */
	plocal_tx_pd->tx_pkt_offset = (t_u16)((t_ptr)pmbuf->pbuf +
					      pmbuf->data_offset -
					      (t_ptr)plocal_tx_pd);

	if (!plocal_tx_pd->tx_control) {
		/* TxCtrl set by user or default */
		plocal_tx_pd->tx_control = pmpriv->pkt_tx_ctrl;
	}

	if (pmbuf->buf_type == MLAN_BUF_TYPE_RAW_DATA) {
		plocal_tx_pd->tx_pkt_type = (t_u16)pkt_type;
		plocal_tx_pd->tx_control = tx_control;
	}

	if (pmbuf->flags & MLAN_BUF_FLAG_TX_STATUS) {
		plocal_tx_pd->tx_control_1 |= pmbuf->tx_seq_num << 8;
		plocal_tx_pd->flags |= MRVDRV_TxPD_FLAGS_TX_PACKET_STATUS;
	}
#ifdef MAC80211_SUPPORT
	if (pmbuf->flags & MLAN_BUF_FLAG_NULL_PKT) {
		plocal_tx_pd->flags |= MRVDRV_TxPD_POWER_MGMT_NULL_PACKET;
	}
#endif

	if (pmbuf->flags & MLAN_BUF_FLAG_RATE_ADAPT_PROBE_PKT) {
		plocal_tx_pd->tx_pkt_type =
			PKT_TYPE_MRVL_MAC80211_RATE_ADAPT_PROBE;
	}

	if (pmbuf->flags & MLAN_BUF_FLAG_TX_CTRL) {
		if (pmbuf->u.tx_info.data_rate) {
			plocal_tx_pd->tx_control |= (pmbuf->u.tx_info.data_rate)
				<< 16;
			plocal_tx_pd->tx_control |= TXPD_TXRATE_ENABLE;
		}
		if (pmbuf->u.tx_info.retry_limit) {
			plocal_tx_pd->tx_control |= pmbuf->u.tx_info.retry_limit
				<< 8;
			plocal_tx_pd->tx_control |= TXPD_RETRY_ENABLE;
		}
	}
	endian_convert_TxPD(plocal_tx_pd);

	/* Adjust the data offset and length to include TxPD in pmbuf */
	pmbuf->data_len += pmbuf->data_offset;
	pmbuf->data_offset = (t_u32)(head_ptr - pmbuf->pbuf);
	pmbuf->data_len -= pmbuf->data_offset;

done:
	LEAVE();
	return head_ptr;
}

#if defined(MAC80211_SUPPORT_UAP) || defined(MAC80211_SUPPORT_MESH)
/**
 *  @brief This function prepares command to set mac80211 beacon.
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param cmd          A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   The action: GET or SET
 *  @param pdata_buf    A pointer to data buffer
 *
 *  @return             MLAN_STATUS_SUCCESS
 */
static mlan_status
wlan_cmd_beacon_cfg(pmlan_private pmpriv,
		    HostCmd_DS_COMMAND *cmd,
		    t_u16 cmd_action, t_void *pdata_buf)
{
	HostCmd_DS_DOT11S_BEACON_CFG *beacon_cfg = &cmd->params.bcn_cfg;
	mlan_ds_beacon_cfg *cfg = (mlan_ds_beacon_cfg *) pdata_buf;

	ENTER();
	cmd->size = wlan_cpu_to_le16((sizeof(HostCmd_DS_DOT11S_BEACON_CFG)) +
				     cfg->beacon_len + S_DS_GEN);
	cmd->command = wlan_cpu_to_le16(Hostcmd_CMD_DOT11S_BEACON_CFG);
	beacon_cfg->action = cmd_action;
	if (cmd_action == HostCmd_ACT_GEN_SET) {
		beacon_cfg->beacon_len = wlan_cpu_to_le16(cfg->beacon_len);
		beacon_cfg->beacon_enable =
			wlan_cpu_to_le16(cfg->beacon_enable);
		beacon_cfg->beacon_int = wlan_cpu_to_le16(cfg->beacon_period);
		beacon_cfg->channel = cfg->channel;
		beacon_cfg->bandConfig.chan2Offset = cfg->bandcfg.chan2Offset;
		beacon_cfg->bandConfig.chanWidth = cfg->bandcfg.chanWidth;
		beacon_cfg->bandConfig.chanBand = cfg->bandcfg.chanBand;
		memcpy_ext(pmpriv->adapter, beacon_cfg->beacon_data,
			   cfg->beacon_data, cfg->beacon_len, cfg->beacon_len);
	}
	beacon_cfg->action = wlan_cpu_to_le16(beacon_cfg->action);
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}
#endif

/**
 *  @brief This function prepares command to inform firmware about
 *  various state of the peer sta
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param cmd          A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   The action: GET or SET
 *  @param pdata_buf    A pointer to data buffer
 *
 *  @return             MLAN_STATUS_SUCCESS
 */
static mlan_status
wlan_cmd_sta_state(pmlan_private pmpriv,
		   HostCmd_DS_COMMAND *cmd, t_u16 cmd_action, t_void *pdata_buf)
{
	HostCmd_DS_DOT11S_STA_STATE *sta_state = &cmd->params.sta_state;
	mlan_ds_sta_state *sta = (mlan_ds_sta_state *) pdata_buf;
	MrvlIETypes_HTCap_t *ht_cap;
	MrvlIETypes_VHTCap_t *vht_cap;
	t_u16 tlv_buf_left;
	MrvlIEtypesHeader_t *tlv_buf_src;
	MrvlIEtypesHeader_t *tlv_buf_dst;
	mlan_adapter *pmadapter = pmpriv->adapter;
	t_u16 tlv_len = 0;

	ENTER();

	cmd->size = wlan_cpu_to_le16((sizeof(HostCmd_DS_DOT11S_STA_STATE) +
				      S_DS_GEN));
	cmd->command = wlan_cpu_to_le16(Hostcmd_CMD_DOT11S_STA_STATE);
	sta_state->action = cmd_action;
	if (cmd_action == HostCmd_ACT_GEN_SET) {
		sta_state->add = wlan_cpu_to_le16(sta->add);
		sta_state->aid = wlan_cpu_to_le16(sta->aid);
		memcpy_ext(pmpriv->adapter, sta_state->peer_mac, sta->peer_mac,
			   MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
		sta_state->bitmap_supp_rates =
			wlan_cpu_to_le16(sta->bitmap_supp_rates);
		sta_state->bw = sta->bw;
		sta_state->wme = sta->wme;
		sta_state->uapsd_queues = sta->uapsd_queues;
		sta_state->bcn_interval = sta->bcn_interval;
		sta_state->bcn_dtim_period = sta->bcn_dtim_period;

		tlv_buf_left = sta->tlv_len;
		tlv_buf_src = (MrvlIEtypesHeader_t *)sta->tlv_buf;
		tlv_buf_dst = (MrvlIEtypesHeader_t *)sta_state->tlv_buf;
		while (tlv_buf_left >= sizeof(MrvlIEtypesHeader_t)) {
			switch (tlv_buf_src->type) {
			case HT_CAPABILITY:
				ht_cap = (MrvlIETypes_HTCap_t *)tlv_buf_dst;
				ht_cap->header.type =
					wlan_cpu_to_le16(HT_CAPABILITY);
				ht_cap->header.len =
					wlan_cpu_to_le16(sizeof
							 (ht_cap->ht_cap));
				memcpy_ext(pmadapter, (t_u8 *)&ht_cap->ht_cap,
					   (t_u8 *)tlv_buf_src +
					   sizeof(MrvlIEtypesHeader_t),
					   tlv_buf_src->len,
					   sizeof(ht_cap->ht_cap));
				break;
			case VHT_CAPABILITY:
				vht_cap = (MrvlIETypes_VHTCap_t *)tlv_buf_dst;
				vht_cap->header.type =
					wlan_cpu_to_le16(VHT_CAPABILITY);
				vht_cap->header.len =
					wlan_cpu_to_le16(sizeof
							 (vht_cap->vht_cap));
				memcpy_ext(pmadapter, (t_u8 *)&vht_cap->vht_cap,
					   (t_u8 *)tlv_buf_src +
					   sizeof(MrvlIEtypesHeader_t),
					   tlv_buf_src->len,
					   sizeof(vht_cap->vht_cap));
				break;
			default:
				break;
			}
			tlv_buf_left -=
				sizeof(MrvlIEtypesHeader_t) + tlv_buf_src->len;
			tlv_buf_src = (MrvlIEtypesHeader_t
				       *)((t_u8 *)tlv_buf_src +
					  sizeof(MrvlIEtypesHeader_t) +
					  tlv_buf_src->len);
			tlv_len +=
				sizeof(MrvlIEtypesHeader_t) + tlv_buf_dst->len;
			tlv_buf_dst = (MrvlIEtypesHeader_t
				       *)((t_u8 *)tlv_buf_dst +
					  sizeof(MrvlIEtypesHeader_t) +
					  tlv_buf_dst->len);
		}
		cmd->size += tlv_len;
	}

	sta_state->action = wlan_cpu_to_le16(sta_state->action);
	cmd->size = wlan_cpu_to_le16(cmd->size);

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function prepares command of snmp_mib.
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param cmd          A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   The action: GET or SET
 *  @param cmd_oid      OID: ENABLE or DISABLE
 *  @param pdata_buf    A pointer to command information buffer
 *
 *  @return             MLAN_STATUS_SUCCESS
 */
static mlan_status
wlan_mac80211_cmd_snmp_mib(pmlan_private pmpriv,
			   HostCmd_DS_COMMAND *cmd,
			   t_u16 cmd_action, t_u32 cmd_oid, t_void *pdata_buf)
{
	HostCmd_DS_802_11_SNMP_MIB *psnmp_mib = &cmd->params.smib;
	t_u32 ul_temp;

	ENTER();
	PRINTM(MINFO, "SNMP_CMD: cmd_oid = 0x%x\n", cmd_oid);
	cmd->command = wlan_cpu_to_le16(HostCmd_CMD_802_11_SNMP_MIB);
	cmd->size = sizeof(HostCmd_DS_802_11_SNMP_MIB) - 1 + S_DS_GEN;

	if (cmd_action == HostCmd_ACT_GEN_GET) {
		psnmp_mib->query_type = wlan_cpu_to_le16(HostCmd_ACT_GEN_GET);
		psnmp_mib->buf_size = wlan_cpu_to_le16(MAX_SNMP_BUF_SIZE);
		cmd->size += MAX_SNMP_BUF_SIZE;
	}

	switch (cmd_oid) {
	case ECSAEnable_i:
		psnmp_mib->oid = wlan_cpu_to_le16((t_u16)ECSAEnable_i);
		if (cmd_action == HostCmd_ACT_GEN_SET) {
			psnmp_mib->query_type =
				wlan_cpu_to_le16(HostCmd_ACT_GEN_SET);
			psnmp_mib->buf_size = wlan_cpu_to_le16(sizeof(t_u8));
			psnmp_mib->value[0] = *((t_u8 *)pdata_buf);
			cmd->size += sizeof(t_u8);
		}
		break;
	case RtsThresh_i:
		psnmp_mib->oid = wlan_cpu_to_le16((t_u16)RtsThresh_i);
		if (cmd_action == HostCmd_ACT_GEN_SET) {
			psnmp_mib->query_type =
				wlan_cpu_to_le16(HostCmd_ACT_GEN_SET);
			psnmp_mib->buf_size = wlan_cpu_to_le16(sizeof(t_u16));
			ul_temp = *((t_u32 *)pdata_buf);
			*(t_u16 *)(psnmp_mib->value) =
				wlan_cpu_to_le16((t_u16)ul_temp);
			cmd->size += sizeof(t_u16);
		}
		break;
	default:
		break;
	}
	cmd->size = wlan_cpu_to_le16(cmd->size);
	PRINTM(MINFO,
	       "SNMP_CMD: Action=0x%x, OID=0x%x, OIDSize=0x%x, Value=0x%x\n",
	       cmd_action, cmd_oid, wlan_le16_to_cpu(psnmp_mib->buf_size),
	       wlan_le16_to_cpu(*(t_u16 *)psnmp_mib->value));
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function prepares command of key_material.
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param cmd          A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   The action: GET or SET
 *  @param cmd_oid      OID: ENABLE or DISABLE
 *  @param pdata_buf    A pointer to data buffer
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_mac80211_cmd_key_material(pmlan_private pmpriv,
			       HostCmd_DS_COMMAND *cmd,
			       t_u16 cmd_action,
			       t_u32 cmd_oid, t_void *pdata_buf)
{
	HostCmd_DS_802_11_KEY_MATERIAL *pkey_material =
		&cmd->params.key_material;
	mlan_ds_encrypt_key *pkey = (mlan_ds_encrypt_key *)pdata_buf;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();
	if (!pkey) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	cmd->command = wlan_cpu_to_le16(HostCmd_CMD_802_11_KEY_MATERIAL);
	if (cmd_action == HostCmd_ACT_GEN_GET) {
		PRINTM(MCMND, "GET Key not supported\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	} else {
		pkey_material->action = wlan_cpu_to_le16(cmd_action);
	}

	memset(pmpriv->adapter, &pkey_material->key_param_set, 0,
	       sizeof(MrvlIEtype_KeyParamSetV2_t));

	if (pkey->key_flags & KEY_FLAG_REMOVE_KEY) {
		pkey_material->action =
			wlan_cpu_to_le16(HostCmd_ACT_GEN_REMOVE);

		pkey_material->key_param_set.type =
			wlan_cpu_to_le16(TLV_TYPE_KEY_PARAM_V2);

		pkey_material->key_param_set.length =
			wlan_cpu_to_le16(KEY_PARAMS_FIXED_LEN);

		pkey_material->key_param_set.key_idx =
			pkey->key_index & KEY_INDEX_MASK;

		pkey_material->key_param_set.key_info =
			wlan_cpu_to_le16(KEY_INFO_MCAST_KEY |
					 KEY_INFO_UCAST_KEY);

		memcpy_ext(pmpriv->adapter,
			   pkey_material->key_param_set.mac_addr,
			   pkey->mac_addr, MLAN_MAC_ADDR_LENGTH,
			   MLAN_MAC_ADDR_LENGTH);

		cmd->size = wlan_cpu_to_le16(sizeof(MrvlIEtypesHeader_t) +
					     S_DS_GEN + KEY_PARAMS_FIXED_LEN +
					     sizeof(pkey_material->action));
		PRINTM(MCMND, "Remove Key\n");
		goto done;
	}

	pkey_material->action = wlan_cpu_to_le16(HostCmd_ACT_GEN_SET);
	pkey_material->key_param_set.key_idx = pkey->key_index & KEY_INDEX_MASK;
	pkey_material->key_param_set.type =
		wlan_cpu_to_le16(TLV_TYPE_KEY_PARAM_V2);
	pkey_material->key_param_set.key_info = KEY_INFO_ENABLE_KEY;
	memcpy_ext(pmpriv->adapter, pkey_material->key_param_set.mac_addr,
		   pkey->mac_addr, MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);

	if (pkey->key_len <= MAX_WEP_KEY_SIZE) {
		pkey_material->key_param_set.key_type = KEY_TYPE_ID_WEP;

		pkey_material->key_param_set.key_info |=
			KEY_INFO_MCAST_KEY | KEY_INFO_UCAST_KEY |
			KEY_INFO_TX_KEY | KEY_INFO_RX_KEY;

		pkey_material->key_param_set.key_params.wep.key_len =
			wlan_cpu_to_le16(pkey->key_len);

		memcpy_ext(pmpriv->adapter,
			   pkey_material->key_param_set.key_params.wep.key,
			   pkey->key_material, pkey->key_len, MAX_WEP_KEY_SIZE);

		pkey_material->key_param_set.length =
			wlan_cpu_to_le16(KEY_PARAMS_FIXED_LEN +
					 sizeof(wep_param_t));
		cmd->size =
			wlan_cpu_to_le16(sizeof(MrvlIEtypesHeader_t) +
					 S_DS_GEN + KEY_PARAMS_FIXED_LEN +
					 sizeof(wep_param_t) +
					 sizeof(pkey_material->action));

		PRINTM(MCMND, "Set WEP Key\n");
		goto done;
	}

	if (pkey->key_flags & KEY_FLAG_GROUP_KEY)
		pkey_material->key_param_set.key_info |= KEY_INFO_MCAST_KEY;
	else
		pkey_material->key_param_set.key_info |= KEY_INFO_UCAST_KEY;

	if (pkey->key_flags & KEY_FLAG_AES_MCAST_IGTK)
		pkey_material->key_param_set.key_info = KEY_INFO_CMAC_AES_KEY;

	if (pkey->key_flags & KEY_FLAG_SET_TX_KEY)
		pkey_material->key_param_set.key_info |=
			KEY_INFO_TX_KEY | KEY_INFO_RX_KEY;
#ifdef MAC80211_SUPPORT_MESH
	else if (pkey->key_flags & KEY_FLAG_SET_GRP_TX_KEY)
		pkey_material->key_param_set.key_info |= KEY_INFO_TX_KEY;
#endif
	else
		pkey_material->key_param_set.key_info |= KEY_INFO_RX_KEY;

	pkey_material->key_param_set.key_info |= KEY_INFO_DEFAULT_KEY;

	if (pkey->key_len == WPA_TKIP_KEY_LEN) {
		pkey_material->key_param_set.key_type = KEY_TYPE_ID_TKIP;

		pkey_material->key_param_set.key_params.tkip.key_len =
			wlan_cpu_to_le16(pkey->key_len);

		memcpy_ext(pmpriv->adapter,
			   pkey_material->key_param_set.key_params.tkip.key,
			   pkey->key_material, pkey->key_len, WPA_TKIP_KEY_LEN);

		if (pkey->key_flags &
		    (KEY_FLAG_RX_SEQ_VALID | KEY_FLAG_TX_SEQ_VALID))
			memcpy_ext(pmpriv->adapter,
				   pkey_material->key_param_set.key_params.tkip.
				   pn, pkey->pn, SEQ_MAX_SIZE, WPA_PN_SIZE);

		pkey_material->key_param_set.length =
			wlan_cpu_to_le16(KEY_PARAMS_FIXED_LEN +
					 sizeof(tkip_param));

		cmd->size = wlan_cpu_to_le16(sizeof(MrvlIEtypesHeader_t) +
					     S_DS_GEN + KEY_PARAMS_FIXED_LEN +
					     sizeof(tkip_param) +
					     sizeof(pkey_material->action));
		PRINTM(MCMND, "Set TKIP Key\n");
		goto done;
	}

	if (pkey->key_len == WPA_AES_KEY_LEN
	    && !(pkey->key_flags & KEY_FLAG_AES_MCAST_IGTK)
		) {

		pkey_material->key_param_set.key_type = KEY_TYPE_ID_AES;

		pkey_material->key_param_set.key_params.aes.key_len =
			wlan_cpu_to_le16(pkey->key_len);

		memcpy_ext(pmpriv->adapter,
			   pkey_material->key_param_set.key_params.aes.key,
			   pkey->key_material, pkey->key_len, WPA_AES_KEY_LEN);

		if (pkey->key_flags &
		    (KEY_FLAG_RX_SEQ_VALID | KEY_FLAG_TX_SEQ_VALID))
			memcpy_ext(pmpriv->adapter,
				   pkey_material->key_param_set.key_params.aes.
				   pn, pkey->pn, SEQ_MAX_SIZE, WPA_PN_SIZE);

		pkey_material->key_param_set.length =
			wlan_cpu_to_le16(KEY_PARAMS_FIXED_LEN +
					 sizeof(aes_param));

		cmd->size = wlan_cpu_to_le16(sizeof(MrvlIEtypesHeader_t) +
					     S_DS_GEN + KEY_PARAMS_FIXED_LEN +
					     sizeof(aes_param) +
					     sizeof(pkey_material->action));
		PRINTM(MCMND, "Set AES Key\n");
		goto done;
	}

	if (pkey->key_len == WPA_IGTK_KEY_LEN &&
	    (pkey->key_flags & KEY_FLAG_AES_MCAST_IGTK)) {
		pkey_material->key_param_set.key_type = KEY_TYPE_ID_AES_CMAC;

		pkey_material->key_param_set.key_info &=
			~(wlan_cpu_to_le16(KEY_INFO_MCAST_KEY));

		pkey_material->key_param_set.key_info |=
			wlan_cpu_to_le16(KEY_INFO_AES_MCAST_IGTK);

		pkey_material->key_param_set.key_params.cmac_aes.key_len =
			wlan_cpu_to_le16(pkey->key_len);

		memcpy_ext(pmpriv->adapter,
			   pkey_material->key_param_set.key_params.cmac_aes.key,
			   pkey->key_material, pkey->key_len, CMAC_AES_KEY_LEN);

		if (pkey->key_flags &
		    (KEY_FLAG_RX_SEQ_VALID | KEY_FLAG_TX_SEQ_VALID))
			memcpy_ext(pmpriv->adapter,
				   pkey_material->key_param_set.key_params.
				   cmac_aes.ipn, pkey->pn, SEQ_MAX_SIZE,
				   IGTK_PN_SIZE);

		pkey_material->key_param_set.length =
			wlan_cpu_to_le16(KEY_PARAMS_FIXED_LEN +
					 sizeof(cmac_aes_param));

		cmd->size = wlan_cpu_to_le16(sizeof(MrvlIEtypesHeader_t) +
					     S_DS_GEN + KEY_PARAMS_FIXED_LEN +
					     sizeof(cmac_aes_param) +
					     sizeof(pkey_material->action));
		PRINTM(MCMND, "Set CMAC AES Key\n");
		goto done;
	}

done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function prepares command for configuring AMPDU TX
 *
 *  @param priv        A pointer to mlan_private structure
 *  @param cmd         A pointer to HostCmd_DS_COMMAND structure
 *  @param pdata_buf   A pointer to data buffer
 *
 *  @return            MLAN_STATUS_SUCCESS
 */
static mlan_status
wlan_mac80211_cmd_ampdu_cfg_tx(mlan_private *priv,
			       HostCmd_DS_COMMAND *cmd, t_void *pdata_buf)
{
	HostCmd_DS_11N_ADDBA_REQ *padd_ba_req =
		(HostCmd_DS_11N_ADDBA_REQ *)&cmd->params.add_ba_req;
	mlan_ds_ampdu_cfg *ampdu_cfg = (mlan_ds_ampdu_cfg *) pdata_buf;
	ENTER();

	cmd->command = wlan_cpu_to_le16(HostCmd_CMD_11N_ADDBA_REQ);
	cmd->size =
		wlan_cpu_to_le16(sizeof(HostCmd_DS_11N_ADDBA_REQ) + S_DS_GEN);
	memcpy_ext(priv->adapter, &padd_ba_req->peer_mac_addr,
		   ampdu_cfg->peer_addr, MLAN_MAC_ADDR_LENGTH,
		   MLAN_MAC_ADDR_LENGTH);
	padd_ba_req->block_ack_param_set =
		(t_u16)((ampdu_cfg->tid << BLOCKACKPARAM_TID_POS) |
			(ampdu_cfg->buf_size << BLOCKACKPARAM_WINSIZE_POS) |
			IMMEDIATE_BLOCK_ACK | BLOCKACKPARAM_AMSDU_SUPP_MASK);
	padd_ba_req->block_ack_param_set =
		wlan_cpu_to_le16(padd_ba_req->block_ack_param_set);
	padd_ba_req->block_ack_tmo = wlan_cpu_to_le16(ampdu_cfg->timeout);
	padd_ba_req->ssn = wlan_cpu_to_le16(ampdu_cfg->seq_num);
	padd_ba_req->add_req_result = 0;
	padd_ba_req->dialog_token = 0;

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function prepares command for configuring AMPDU RX
 *
 *  @param priv        A pointer to mlan_private structure
 *  @param cmd         A pointer to HostCmd_DS_COMMAND structure
 *  @param pdata_buf   A pointer to data buffer
 *
 *  @return            MLAN_STATUS_SUCCESS
 */
static mlan_status
wlan_mac80211_cmd_ampdu_cfg_rx(mlan_private *priv,
			       HostCmd_DS_COMMAND *cmd, t_void *pdata_buf)
{
	HostCmd_DS_11N_ADDBA_RSP *padd_ba_resp =
		(HostCmd_DS_11N_ADDBA_RSP *)&cmd->params.add_ba_req;
	mlan_ds_ampdu_cfg *ampdu_cfg = (mlan_ds_ampdu_cfg *) pdata_buf;
	ENTER();

	cmd->command = wlan_cpu_to_le16(HostCmd_CMD_11N_ADDBA_RSP);
	cmd->size =
		wlan_cpu_to_le16(sizeof(HostCmd_DS_11N_ADDBA_RSP) + S_DS_GEN);
	memcpy_ext(priv->adapter, &padd_ba_resp->peer_mac_addr,
		   ampdu_cfg->peer_addr, MLAN_MAC_ADDR_LENGTH,
		   MLAN_MAC_ADDR_LENGTH);
	padd_ba_resp->block_ack_param_set =
		(t_u16)((ampdu_cfg->tid << BLOCKACKPARAM_TID_POS) |
			(ampdu_cfg->buf_size << BLOCKACKPARAM_WINSIZE_POS) |
			IMMEDIATE_BLOCK_ACK | BLOCKACKPARAM_AMSDU_SUPP_MASK);
	padd_ba_resp->block_ack_param_set =
		wlan_cpu_to_le16(padd_ba_resp->block_ack_param_set);
	padd_ba_resp->block_ack_tmo = wlan_cpu_to_le16(ampdu_cfg->timeout);
	padd_ba_resp->ssn = wlan_cpu_to_le16(ampdu_cfg->seq_num);
	padd_ba_resp->add_rsp_result = 0;
	padd_ba_resp->dialog_token = 0;
	padd_ba_resp->status_code = wlan_cpu_to_le16(ADDBA_RSP_STATUS_ACCEPT);

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function prepares command for configuring AMPDU RX
 *
 *  @param priv        A pointer to mlan_private structure
 *  @param cmd         A pointer to HostCmd_DS_COMMAND structure
 *  @param pdata_buf   A pointer to data buffer
 *
 *  @return            MLAN_STATUS_SUCCESS
 */

static mlan_status
wlan_mac80211_cmd_ampdu_delete(mlan_private *priv,
			       HostCmd_DS_COMMAND *cmd, t_void *pdata_buf)
{
	HostCmd_DS_11N_DELBA *pdel_ba =
		(HostCmd_DS_11N_DELBA *)&cmd->params.del_ba;
	mlan_ds_ampdu_cfg *ampdu_cfg = (mlan_ds_ampdu_cfg *) pdata_buf;
	ENTER();

	cmd->command = wlan_cpu_to_le16(HostCmd_CMD_11N_DELBA);
	cmd->size = wlan_cpu_to_le16(sizeof(HostCmd_DS_11N_DELBA) + S_DS_GEN);

	pdel_ba->del_ba_param_set = (ampdu_cfg->tid << DELBA_TID_POS);
	if (ampdu_cfg->is_tx)
		DELBA_INITIATOR(pdel_ba->del_ba_param_set);
	else
		DELBA_RECIPIENT(pdel_ba->del_ba_param_set);
	pdel_ba->del_ba_param_set = wlan_cpu_to_le16(pdel_ba->del_ba_param_set);

	memcpy_ext(priv->adapter, &pdel_ba->peer_mac_addr, ampdu_cfg->peer_addr,
		   MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);

	pdel_ba->reason_code = 0;
	pdel_ba->del_result = 0;

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function prepare the command before sending to firmware.
 *
 *  @param priv       A pointer to mlan_private structure
 *  @param cmd_no       Command number
 *  @param cmd_action   Command action: GET or SET
 *  @param cmd_oid      Cmd oid: treated as sub command
 *  @param pioctl_buf   A pointer to MLAN IOCTL Request buffer
 *  @param pdata_buf    A pointer to information buffer
 *  @param pcmd_buf      A pointer to cmd buf
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_ops_mac80211_prepare_cmd(t_void *priv, t_u16 cmd_no,
			      t_u16 cmd_action, t_u32 cmd_oid,
			      t_void *pioctl_buf,
			      t_void *pdata_buf, t_void *pcmd_buf)
{
	HostCmd_DS_COMMAND *cmd_ptr = (HostCmd_DS_COMMAND *)pcmd_buf;
	mlan_private *pmpriv = (mlan_private *)priv;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Prepare command */
	switch (cmd_no) {
#if defined(PCIE)
#if defined(PCIE8997) || defined(PCIE8897)
	case HostCmd_CMD_PCIE_HOST_BUF_DETAILS:
		ret = wlan_cmd_pcie_host_buf_cfg(pmpriv, cmd_ptr, cmd_action,
						 pdata_buf);
		break;
#endif
#endif
	case HostCmd_CMD_GET_HW_SPEC:
		ret = wlan_cmd_get_hw_spec(pmpriv, cmd_ptr);
		break;
	case HostCmd_CMD_MAC_CONTROL:
		ret = wlan_cmd_mac_control(pmpriv, cmd_ptr, cmd_action,
					   pdata_buf);
		break;
	case HostCmd_CMD_802_11_MAC_ADDRESS:
		ret = wlan_cmd_802_11_mac_address(pmpriv, cmd_ptr, cmd_action);
		break;
	case HostCmd_CMD_802_11_RF_ANTENNA:
		ret = wlan_cmd_802_11_rf_antenna(pmpriv, cmd_ptr, cmd_action,
						 pdata_buf);
		break;
	case HostCmd_CMD_FUNC_INIT:
		if (pmpriv->adapter->hw_status == WlanHardwareStatusReset)
			pmpriv->adapter->hw_status =
				WlanHardwareStatusInitializing;
		cmd_ptr->command = wlan_cpu_to_le16(cmd_no);
		cmd_ptr->size = wlan_cpu_to_le16(S_DS_GEN);
		break;
	case HostCmd_CMD_FUNC_SHUTDOWN:
		pmpriv->adapter->hw_status = WlanHardwareStatusReset;
		cmd_ptr->command = wlan_cpu_to_le16(cmd_no);
		cmd_ptr->size = wlan_cpu_to_le16(S_DS_GEN);
		break;
	case HostCmd_CMD_RECONFIGURE_TX_BUFF:
		ret = wlan_cmd_recfg_tx_buf(pmpriv, cmd_ptr, cmd_action,
					    pdata_buf);
		break;
	case HostCmd_CMD_AMSDU_AGGR_CTRL:
		ret = wlan_cmd_amsdu_aggr_ctrl(pmpriv, cmd_ptr, cmd_action,
					       pdata_buf);
		break;
	case HostCmd_CMD_11N_CFG:
		ret = wlan_cmd_11n_cfg(pmpriv, cmd_ptr, cmd_action, pdata_buf);
		break;
	case HostCmd_CMD_11AC_CFG:
		ret = wlan_cmd_11ac_cfg(pmpriv, cmd_ptr, cmd_action, pdata_buf);
		break;
#if defined(MAC80211_SUPPORT_UAP) || defined(MAC80211_SUPPORT_MESH)
	case Hostcmd_CMD_DOT11S_BEACON_CFG:
		ret = wlan_cmd_beacon_cfg(pmpriv, cmd_ptr, cmd_action,
					  pdata_buf);
		break;
#endif
	case Hostcmd_CMD_DOT11S_STA_STATE:
		ret = wlan_cmd_sta_state(pmpriv, cmd_ptr, cmd_action,
					 pdata_buf);
		break;
	case HostCmd_CMD_RX_MGMT_IND:
		cmd_ptr->command = wlan_cpu_to_le16(cmd_no);
		cmd_ptr->params.rx_mgmt_ind.action =
			wlan_cpu_to_le16(cmd_action);
		cmd_ptr->params.rx_mgmt_ind.mgmt_subtype_mask =
			wlan_cpu_to_le32((t_u32)(*((t_u32 *)pdata_buf)));
		cmd_ptr->size =
			wlan_cpu_to_le16(sizeof(HostCmd_DS_RX_MGMT_IND) +
					 S_DS_GEN);
		break;
	case HostCmd_CMD_802_11_REMAIN_ON_CHANNEL:
		ret = wlan_cmd_remain_on_channel(pmpriv, cmd_ptr, cmd_action,
						 pdata_buf);
		break;
	case HostCmd_CMD_802_11_SNMP_MIB:
		ret = wlan_mac80211_cmd_snmp_mib(pmpriv, cmd_ptr, cmd_action,
						 cmd_oid, pdata_buf);
		break;
	case HostCmd_CMD_HOST_CLOCK_CFG:
		ret = wlan_cmd_host_clock_cfg(cmd_ptr, cmd_action, pdata_buf);
		break;
	case HostCmd_CMD_CFG_DATA:
		ret = wlan_cmd_cfg_data(pmpriv, cmd_ptr, cmd_action, cmd_oid,
					pdata_buf);
		break;
	case HostCmd_CMD_802_11_KEY_MATERIAL:
		ret = wlan_mac80211_cmd_key_material(pmpriv, cmd_ptr,
						     cmd_action, cmd_oid,
						     pdata_buf);
		break;
	case HostCmd_CMD_11N_ADDBA_REQ:
		ret = wlan_mac80211_cmd_ampdu_cfg_tx(pmpriv, cmd_ptr,
						     pdata_buf);
		break;
	case HostCmd_CMD_11N_ADDBA_RSP:
		ret = wlan_mac80211_cmd_ampdu_cfg_rx(pmpriv, cmd_ptr,
						     pdata_buf);
		break;
	case HostCmd_CMD_11N_DELBA:
		ret = wlan_mac80211_cmd_ampdu_delete(pmpriv, cmd_ptr,
						     pdata_buf);
		break;
	case HostCmd_CMD_802_11_PS_MODE_ENH:
		ret = wlan_cmd_enh_power_mode(pmpriv, cmd_ptr, cmd_action,
					      (t_u16)cmd_oid, pdata_buf);
		break;
	case HostCmd_CMD_802_11_NET_MONITOR:
		ret = wlan_cmd_net_monitor(cmd_ptr, cmd_action, pdata_buf);
		break;
	default:
		PRINTM(MERROR, "PREP_CMD: unknown command- %#x\n", cmd_no);
		ret = MLAN_STATUS_FAILURE;
		break;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief This function handles the command response error
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param resp         A pointer to HostCmd_DS_COMMAND
 *  @param pioctl_buf   A pointer to command buffer
 *
 *  @return             N/A
 */
static mlan_status
mac80211_process_cmdresp_error(mlan_private *pmpriv,
			       HostCmd_DS_COMMAND *resp,
			       mlan_ioctl_req *pioctl_buf)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	mlan_status ret = MLAN_STATUS_FAILURE;

	ENTER();
	PRINTM(MERROR, "CMD_RESP: cmd %#x error, result=%#x\n", resp->command,
	       resp->result);
	if (pioctl_buf)
		pioctl_buf->status_code = MLAN_ERROR_FW_CMDRESP;

	switch (resp->command) {
	default:
		break;
	}
	/*
	 * Handling errors here
	 */
	wlan_request_cmd_lock(pmadapter);
	wlan_insert_cmd_to_free_q(pmadapter, pmadapter->curr_cmd);
	pmadapter->curr_cmd = MNULL;
	wlan_release_cmd_lock(pmadapter);

	LEAVE();
	return ret;
}

/**
 *  @brief This function handles the station command response
 *
 *  @param priv             A pointer to mlan_private structure
 *  @param cmdresp_no       cmd no
 *  @param pcmd_buf         cmdresp buf
 *  @param pioctl           A pointer to ioctl buf
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_ops_mac80211_process_cmdresp(t_void *priv,
				  t_u16 cmdresp_no,
				  t_void *pcmd_buf, t_void *pioctl)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_private *pmpriv = (mlan_private *)priv;
	HostCmd_DS_COMMAND *resp = (HostCmd_DS_COMMAND *)pcmd_buf;
	mlan_ioctl_req *pioctl_buf = (mlan_ioctl_req *)pioctl;
	mlan_adapter *pmadapter = pmpriv->adapter;

	ENTER();

	/* If the command is not successful, cleanup and return failure */
	if (resp->result != HostCmd_RESULT_OK) {
		ret = mac80211_process_cmdresp_error(pmpriv, resp, pioctl_buf);
		LEAVE();
		return ret;
	}

	/* Command successful, handle response */
	switch (cmdresp_no) {
#if defined(PCIE)
#if defined(PCIE8997) || defined(PCIE8897)
	case HostCmd_CMD_PCIE_HOST_BUF_DETAILS:
		PRINTM(MINFO, "PCIE host buffer configuration successful.\n");
		break;
#endif
#endif
	case HostCmd_CMD_GET_HW_SPEC:
		ret = wlan_ret_get_hw_spec(pmpriv, resp, pioctl_buf);
		break;
	case HostCmd_CMD_MAC_CONTROL:
		ret = wlan_ret_mac_control(pmpriv, resp, pioctl_buf);
		break;
	case HostCmd_CMD_802_11_MAC_ADDRESS:
		ret = wlan_ret_802_11_mac_address(pmpriv, resp, pioctl_buf);
		break;

	case HostCmd_CMD_802_11_RF_ANTENNA:
		ret = wlan_ret_802_11_rf_antenna(pmpriv, resp, pioctl_buf);
		break;

	case HostCmd_CMD_802_11_PS_MODE_ENH:
		ret = wlan_ret_enh_power_mode(pmpriv, resp, pioctl_buf);
		break;

	case HostCmd_CMD_FUNC_INIT:
	case HostCmd_CMD_FUNC_SHUTDOWN:
		break;
	case HostCmd_CMD_RECONFIGURE_TX_BUFF:
		wlan_set_tx_pause_flag(pmpriv, MFALSE);
		pmadapter->tx_buf_size =
			(t_u16)wlan_le16_to_cpu(resp->params.tx_buf.buff_size);
		pmadapter->curr_tx_buf_size = pmadapter->tx_buf_size;
		PRINTM(MCMND, "max_tx_buf_size=%d, tx_buf_size=%d\n",
		       pmadapter->max_tx_buf_size, pmadapter->tx_buf_size);
		break;
	case HostCmd_CMD_AMSDU_AGGR_CTRL:
		ret = wlan_ret_amsdu_aggr_ctrl(pmpriv, resp, pioctl_buf);
		break;
	case HostCmd_CMD_11N_CFG:
		ret = wlan_ret_11n_cfg(pmpriv, resp, pioctl_buf);
		break;
	case HostCmd_CMD_11AC_CFG:
		ret = wlan_ret_11ac_cfg(pmpriv, resp, pioctl_buf);
		break;
#if defined(MAC80211_SUPPORT_UAP) || defined(MAC80211_SUPPORT_MESH)
	case Hostcmd_CMD_DOT11S_BEACON_CFG:
		break;
#endif
	case Hostcmd_CMD_DOT11S_STA_STATE:
		break;
	case HostCmd_CMD_RX_MGMT_IND:
		ret = wlan_ret_rx_mgmt_ind(pmpriv, resp, pioctl_buf);
		break;
	case HostCmd_CMD_802_11_REMAIN_ON_CHANNEL:
		ret = wlan_ret_remain_on_channel(pmpriv, resp, pioctl_buf);
		break;
	case HostCmd_CMD_802_11_SNMP_MIB:
		break;
	case HostCmd_CMD_HOST_CLOCK_CFG:
		ret = wlan_ret_host_clock_cfg(pmpriv, resp, pioctl_buf);
		break;
	case HostCmd_CMD_CFG_DATA:
		ret = wlan_ret_cfg_data(pmpriv, resp, pioctl_buf);
		break;
	case HostCmd_CMD_802_11_KEY_MATERIAL:
		break;
	case HostCmd_CMD_11N_ADDBA_REQ:
		break;
	case HostCmd_CMD_11N_ADDBA_RSP:
		break;
	case HostCmd_CMD_11N_DELBA:
		break;
	case HostCmd_CMD_802_11_NET_MONITOR:
		ret = wlan_ret_net_monitor(pmpriv, resp, pioctl_buf);
		break;
	default:
		PRINTM(MERROR, "CMD_RESP: Unknown command response %#x\n",
		       resp->command);
		break;
	}

	LEAVE();
	return ret;
}
