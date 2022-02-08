/** @file moal_mac80211.c
 *
 * @brief This file contains the functions for mac80211.
 *
 *
 * Copyright 2018-2021 NXP
 *
 * This software file (the File) is distributed by NXP
 * under the terms of the GNU General Public License Version 2, June 1991
 * (the License).  You may use, redistribute and/or modify the File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 *
 */
#ifdef MAC80211_SUPPORT
#include <net/mac80211.h>
#include <linux/ieee80211.h>
#include "moal_main.h"
#include "moal_sta_cfg80211.h"
#include "moal_eth_ioctl.h"
#include "moal_mac80211.h"

#define MRVL_PKT_TYPE_MGMT_FRAME 0xE5
#define KEY_INDEX_CLEAR_ALL (0x0000000F)
#define CHANWIDTH_20_MHZ 0
#define CHANWIDTH_10_MHZ 1
#define HEADER_SIZE 8
#define PKT_LEN 2
#define IEEE_HEADER_ADDR4_POS 24
#define IEEE_HEADER_HAS_ADDR4 30
#define IEEE_HEADER_NO_ADDR4 24
#define MAC80211_MAX_TX_PENDING 400
#define MAC80211_MIN_TX_PENDING 380
enum MAC80211_STA_STATE { STA_REMOVE = 0, STA_ADD, STA_ASSOC, STA_DISSASSOC };

/** Supported rates to be advertised to the mac80211 */
static struct ieee80211_rate mac80211_rates[] = {
	{
	 .bitrate = 10,
	 .hw_value = 2,
	 },
	{
	 .bitrate = 20,
	 .hw_value = 4,
	 },
	{.bitrate = 55,.hw_value = 11},
	{
	 .bitrate = 110,
	 .hw_value = 22,
	 },
	{
	 .bitrate = 220,
	 .hw_value = 44,
	 },
	{
	 .bitrate = 60,
	 .hw_value = 12,
	 },
	{
	 .bitrate = 90,
	 .hw_value = 18,
	 },
	{
	 .bitrate = 120,
	 .hw_value = 24,
	 },
	{
	 .bitrate = 180,
	 .hw_value = 36,
	 },
	{
	 .bitrate = 240,
	 .hw_value = 48,
	 },
	{
	 .bitrate = 360,
	 .hw_value = 72,
	 },
	{
	 .bitrate = 480,
	 .hw_value = 96,
	 },
	{
	 .bitrate = 540,
	 .hw_value = 108,
	 },
};

/** Channel definitions for 2 GHz to be advertised to mac80211 */
static struct ieee80211_channel channels_2ghz[] = {
	{.center_freq = 2412,.hw_value = 1,.max_power = 20},
	{.center_freq = 2417,.hw_value = 2,.max_power = 20},
	{.center_freq = 2422,.hw_value = 3,.max_power = 20},
	{.center_freq = 2427,.hw_value = 4,.max_power = 20},
	{.center_freq = 2432,.hw_value = 5,.max_power = 20},
	{.center_freq = 2437,.hw_value = 6,.max_power = 20},
	{.center_freq = 2442,.hw_value = 7,.max_power = 20},
	{.center_freq = 2447,.hw_value = 8,.max_power = 20},
	{.center_freq = 2452,.hw_value = 9,.max_power = 20},
	{.center_freq = 2457,.hw_value = 10,.max_power = 20},
	{.center_freq = 2462,.hw_value = 11,.max_power = 20},
	{.center_freq = 2467,.hw_value = 12,.max_power = 20},
	{.center_freq = 2472,.hw_value = 13,.max_power = 20},
	{.center_freq = 2484,.hw_value = 14,.max_power = 20},
};

/** Channel definitions for 5 GHz to be advertised to mac80211 */
static struct ieee80211_channel channels_5ghz[] = {
	{.center_freq = 5180,.hw_value = 36,.max_power = 20},
	{.center_freq = 5200,.hw_value = 40,.max_power = 20},
	{.center_freq = 5220,.hw_value = 44,.max_power = 20},
	{.center_freq = 5240,.hw_value = 48,.max_power = 20},
	{.center_freq = 5260,.hw_value = 52,.max_power = 20},
	{.center_freq = 5280,.hw_value = 56,.max_power = 20},
	{.center_freq = 5300,.hw_value = 60,.max_power = 20},
	{.center_freq = 5320,.hw_value = 64,.max_power = 20},
	{.center_freq = 5500,.hw_value = 100,.max_power = 20},
	{.center_freq = 5520,.hw_value = 104,.max_power = 20},
	{.center_freq = 5540,.hw_value = 108,.max_power = 20},
	{.center_freq = 5560,.hw_value = 112,.max_power = 20},
	{.center_freq = 5580,.hw_value = 116,.max_power = 20},
	{.center_freq = 5600,.hw_value = 120,.max_power = 20},
	{.center_freq = 5620,.hw_value = 124,.max_power = 20},
	{.center_freq = 5640,.hw_value = 128,.max_power = 20},
	{.center_freq = 5660,.hw_value = 132,.max_power = 20},
	{.center_freq = 5680,.hw_value = 136,.max_power = 20},
	{.center_freq = 5700,.hw_value = 140,.max_power = 20},
	{.center_freq = 5720,.hw_value = 144,.max_power = 20},
	{.center_freq = 5745,.hw_value = 149,.max_power = 20},
	{.center_freq = 5765,.hw_value = 153,.max_power = 20},
	{.center_freq = 5785,.hw_value = 157,.max_power = 20},
	{.center_freq = 5805,.hw_value = 161,.max_power = 20},
	{.center_freq = 5825,.hw_value = 165,.max_power = 20},
};

static struct ieee80211_supported_band mac80211_band_2ghz = {
	.channels = channels_2ghz,
	.n_channels = ARRAY_SIZE(channels_2ghz),
	.bitrates = mac80211_rates,
	.n_bitrates = ARRAY_SIZE(mac80211_rates),
};

static struct ieee80211_supported_band mac80211_band_5ghz = {
	.channels = channels_5ghz,
	.n_channels = ARRAY_SIZE(channels_5ghz),
	.bitrates = mac80211_rates + 5,
	.n_bitrates = ARRAY_SIZE(mac80211_rates) - 5,
};

#if CFG80211_VERSION_CODE > KERNEL_VERSION(3, 0, 0)
/**
 * NOTE: types in all the sets must be equals to the
 * initial value of wiphy->interface_modes
 */
static const struct ieee80211_iface_limit mac80211_limits[] = {
	{.max = 4,
	 .types = BIT(NL80211_IFTYPE_STATION)
#if defined(MAC80211_SUPPORT_MESH) && defined(CONFIG_MAC80211_MESH)
	 | BIT(NL80211_IFTYPE_MESH_POINT)
#endif
#ifdef MAC80211_SUPPORT_UAP
	 | BIT(NL80211_IFTYPE_AP)
#endif
	 }
};

static struct ieee80211_iface_combination mac80211_iface_combination = {
	.limits = mac80211_limits,
	.num_different_channels = 1,
	.n_limits = ARRAY_SIZE(mac80211_limits),
	.max_interfaces = 3,
};
#endif

static struct nxp_rate_group nxp_rate_groups[] = {
	/*legacy */
	{.min_rate = RATEID_DBPSK1Mbps,
	 .max_rate = RATEID_OFDM72Mbps,
	 .flags = 0,
	 .nss = 0},
	/*HT40-20MHz */
	{.min_rate = RATEID_MCS0_6d5Mbps,
	 .max_rate = RATEID_MCS15_130Mbps,
	 .flags = IEEE80211_TX_RC_MCS,
	 .nss = 0},
	/*HT40-40MHz */
	{.min_rate = RATEID_MCS0BW40_13d5Mbps,
	 .max_rate = RATEID_MCS15BW40_270Mbps,
	 .flags = IEEE80211_TX_RC_MCS | IEEE80211_TX_RC_40_MHZ_WIDTH,
	 .nss = 0},
	/*VHT80-20MHz */
	{.min_rate = RATEID_VHT_MCS0_1SS_BW20,
	 .max_rate = RATEID_VHT_MCS9_1SS_BW20,
	 .flags = IEEE80211_TX_RC_VHT_MCS,
	 .nss = 0},
	/*VHT80-20MHz-2SS */
	{.min_rate = RATEID_VHT_MCS0_2SS_BW20,
	 .max_rate = RATEID_VHT_MCS9_2SS_BW20,
	 .flags = IEEE80211_TX_RC_VHT_MCS,
	 .nss = 1},
	/*VHT80-40MHz */
	{.min_rate = RATEID_VHT_MCS0_1SS_BW40,
	 .max_rate = RATEID_VHT_MCS9_1SS_BW40,
	 .flags = IEEE80211_TX_RC_VHT_MCS | IEEE80211_TX_RC_40_MHZ_WIDTH,
	 .nss = 0},
	/*VHT80-40MHz-2SS */
	{.min_rate = RATEID_VHT_MCS0_2SS_BW40,
	 .max_rate = RATEID_VHT_MCS9_2SS_BW40,
	 .flags = IEEE80211_TX_RC_VHT_MCS | IEEE80211_TX_RC_40_MHZ_WIDTH,
	 .nss = 1},
	/*VHT80-80MHz */
	{.min_rate = RATEID_VHT_MCS0_1SS_BW80,
	 .max_rate = RATEID_VHT_MCS9_1SS_BW80,
	 .flags = IEEE80211_TX_RC_VHT_MCS | IEEE80211_TX_RC_80_MHZ_WIDTH,
	 .nss = 0},
	/*VHT80-80MHz-2SS */
	{.min_rate = RATEID_VHT_MCS0_2SS_BW80,
	 .max_rate = RATEID_VHT_MCS9_2SS_BW80,
	 .flags = IEEE80211_TX_RC_VHT_MCS | IEEE80211_TX_RC_80_MHZ_WIDTH,
	 .nss = 1}
};

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
/* nxp MAC80211 vendor command and event */
#define MRVL_VENDOR_ID 0x005043
enum mac80211_vendor_event { mac80211_event_debug = 0, mac80211_event_max };
/* vendor events */
static const struct nl80211_vendor_cmd_info mac80211_vendor_events[] = {
	{.vendor_id = MRVL_VENDOR_ID,.subcmd = mac80211_event_debug},
	/**add vendor event here*/
};
#endif

#define MAX_TID 8
#define MAC80211_AMPDU_THRESHOLD 16
#define AMPDU_SET 1
#define AMPDU_CLR 0
struct mac80211_sta {
	struct list_head list;
	t_u32 pkts[MAX_TID];
	bool ampdu_status_tx[MAX_TID];
	bool ampdu_status_rx[MAX_TID];
};

/**
 * @brief get private from ieee80211_sta
 *
 * @param sta   struct ieee80211_sta
 *
 * @return      pointer to mac80211_sta
 */
static struct mac80211_sta *
woal_get_mac80211_sta(struct ieee80211_sta *sta)
{
	if (sta)
		return (struct mac80211_sta *)&sta->drv_priv;
	else
		return NULL;
}

#ifdef CONFIG_PM
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
static const struct wiphy_wowlan_support wowlan_support = {
	.flags = WIPHY_WOWLAN_ANY | WIPHY_WOWLAN_MAGIC_PKT,
	.n_patterns = MAX_NUM_FILTERS,
	.pattern_min_len = 1,
	.pattern_max_len = WOWLAN_MAX_PATTERN_LEN,
	.max_pkt_offset = WOWLAN_MAX_OFFSET_LEN,
};
#endif
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
/**
 * @brief get the event id of the events array
 *
 * @param event     vendor event
 *
 * @return    index of events array
 */
static int
woal_mac80211_get_event_id(int event)
{
	int i = 0;

	for (i = 0; i < (int)ARRAY_SIZE(mac80211_vendor_events); i++) {
		if ((int)mac80211_vendor_events[i].subcmd == event)
			return i;
	}

	return mac80211_event_max;
}

/**
 * @brief send vendor event to kernel
 *
 * @param priv     A pointer to moal_private
 * @param event    vendor event
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
static mlan_status
woal_mac80211_cfg80211_vendor_event(moal_private *priv,
				    int event, t_u8 *data, int len)
{
	struct wiphy *wiphy = NULL;
	struct sk_buff *skb = NULL;
	int event_idx = 0;
	t_u8 *pos = NULL;

	ENTER();

	if (!priv || !priv->hw || !priv->hw->wiphy) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	wiphy = priv->hw->wiphy;
	PRINTM(MEVENT, "vendor event :0x%x\n", event);
	event_idx = woal_mac80211_get_event_id(event);
	if (mac80211_event_max == event_idx) {
		PRINTM(MERROR, "unknown vendor event %d \n", event);
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	/**allocate skb*/
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
	skb = cfg80211_vendor_event_alloc(wiphy, NULL, len, event_idx,
					  GFP_ATOMIC);
#else
	skb = cfg80211_vendor_event_alloc(wiphy, len, event_idx, GFP_ATOMIC);
#endif

	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor event\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	pos = skb_put(skb, len);
	moal_memcpy_ext(priv->phandle, pos, data, len, len);
	/**send event*/
	cfg80211_vendor_event(skb, GFP_ATOMIC);

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 * @brief send vendor event to kernel
 *
 * @param priv     A pointer to moal_private
 * @param event    vendor event
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
static void
woal_mac80211_debug_event(moal_private *priv, t_u8 *data, int len)
{
	ENTER();

	woal_mac80211_cfg80211_vendor_event(priv, mac80211_event_debug, data,
					    len);

	LEAVE();
}
#endif

/**
 *  @brief mac80211 start
 *
 *  @param hw           A pointer to ieee80211_hw structure
 *
 *  @return             0->for success else failure
 */
static int
woal_mac80211_start(struct ieee80211_hw *hw)
{
	moal_private *priv = hw->priv;

	ENTER();

	priv->mac80211_started = 1;

	LEAVE();
	return 0;
}

/**
 *  @brief mac80211 stop
 *
 *  @param hw           A pointer to ieee80211_hw structure
 *
 *  @return             NONE
 */
static void
woal_mac80211_stop(struct ieee80211_hw *hw)
{
	moal_private *priv = hw->priv;

	ENTER();

	priv->mac80211_started = 0;

	LEAVE();
}

static void
woal_mac80211_set_tx_status(moal_private *priv, mlan_buffer *pmbuf,
			    struct sk_buff *skb)
{
	struct tx_status_info *tx_info = NULL;
	unsigned long flags;

	if (!skb)
		return;
	if (pmbuf->buf_type != MLAN_BUF_TYPE_DATA)
		return;

	/* TokenID can not be 0, Thus avoid rollover from 255 to 0 */
	if (!priv->tx_seq_num)
		priv->tx_seq_num++;
	pmbuf->tx_seq_num = priv->tx_seq_num++;

	tx_info = kzalloc(sizeof(struct tx_status_info), GFP_ATOMIC);
	if (tx_info) {
		spin_lock_irqsave(&priv->tx_stat_lock, flags);
		pmbuf->flags |= MLAN_BUF_FLAG_TX_STATUS;
		tx_info->tx_skb = skb;
		tx_info->tx_seq_num = pmbuf->tx_seq_num;
		INIT_LIST_HEAD(&tx_info->link);
		list_add_tail(&tx_info->link, &priv->tx_stat_queue);
		spin_unlock_irqrestore(&priv->tx_stat_lock, flags);
	}
}

static void
woal_mac80211_convert_nxp_rateid_to_ieee_rate(u16 rateid,
					      struct ieee80211_tx_info *info)
{
	int i;
	for (i = 0; i < MAX_NXP_RATE_GROUPS; i++) {
		if (rateid <= nxp_rate_groups[i].max_rate &&
		    rateid >= nxp_rate_groups[i].min_rate) {
			info->status.rates[0].idx =
				(rateid - nxp_rate_groups[i].min_rate) |
				(nxp_rate_groups[i].nss << 4);
			info->status.rates[0].flags = nxp_rate_groups[i].flags;
			break;
		}
	}
	if (i == MAX_NXP_RATE_GROUPS) {
		info->status.rates[0].idx = -1;
	}
}

static t_u16
woal_mac80211_convert_ieee_rate_to_nxp_rateid(s8 id, u16 flags)
{
	u16 rateid, idx;
	u16 ss_offset = 10;	/* 2SS rates start after 0-9 MCS entries of 1SS mode
				 */
	idx = id & 0xf;
	if (flags & IEEE80211_TX_RC_VHT_MCS) {
		if (flags & (IEEE80211_TX_RC_80_MHZ_WIDTH))
			rateid = idx + RATEID_VHT_MCS0_1SS_BW80;
		else if (flags & (IEEE80211_TX_RC_40_MHZ_WIDTH))
			rateid = idx + RATEID_VHT_MCS0_1SS_BW40;
		else
			rateid = idx + RATEID_VHT_MCS0_1SS_BW20;

		if ((id & 0xf0) >> 4 == 1) {	/* bits 7:4 in id , 0: 1SS mode ,
						   2: 2SS mode */
			rateid = rateid + ss_offset;
		}
	} else if (flags & IEEE80211_TX_RC_MCS) {
		if (flags & (IEEE80211_TX_RC_40_MHZ_WIDTH))
			rateid = idx + RATEID_MCS0BW40_13d5Mbps;
		else
			rateid = idx + RATEID_MCS0_6d5Mbps;
	} else
		rateid = idx;

	return rateid;
}

/**
 *  @brief Transmit the packet received from mac80211
 *
 *  @param hw           A pointer to ieee80211_hw structure
 *  @param control      A pointer to the ieee80211_tx_control structure
 *  @param skb          A pointer to skb_buf structure
 *
 *  @return             NONE
 */
static void
woal_mac80211_tx(struct ieee80211_hw *hw,
		 struct ieee80211_tx_control *control, struct sk_buff *skb)
{
	moal_private *priv = hw->priv;
	mlan_buffer *pmbuf;
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	struct sk_buff *new_skb;
	t_u8 bcast_addr[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	t_u16 len;
	t_u32 pkt_type = 0, tx_control = 0;
	int headroom, extra_headroom, total_headroom = 0;
	mlan_status status;
	t_u8 qos_ctl_offset;
	t_u8 tid;
	t_u16 qos;
	struct ieee80211_tx_info *tx_info = NULL;
	struct mac80211_sta *sta_info = NULL;
	t_u16 queue;

	ENTER();

	if (priv->phandle->surprise_removed == MTRUE) {
		dev_kfree_skb_any(skb);
		LEAVE();
		return;
	}
	if (ieee80211_is_mgmt(hdr->frame_control)) {
		headroom = MLAN_MIN_DATA_HEADER_LEN + sizeof(mlan_buffer) +
			HEADER_SIZE + PKT_LEN;

		if ((int)skb_headroom(skb) < headroom) {
			PRINTM(MWARN, "Tx: Insufficient skb headroom %d\n",
			       skb_headroom(skb));
			/* Insufficient skb headroom - allocate a new skb */
			new_skb = skb_realloc_headroom(skb, headroom);
			if (unlikely(!new_skb)) {
				PRINTM(MERROR, "Tx: Cannot reallocate skb\n");
				dev_kfree_skb_any(skb);
				goto done;
			}

			if (new_skb != skb)
				dev_kfree_skb_any(skb);
			skb = new_skb;
			total_headroom = skb_headroom(skb);
			PRINTM(MINFO, "new skb headroom %d\n",
			       skb_headroom(skb));
		}

		/** If skb_realloc_headroom allocates more headroom than
		 * required there will be extraheadroom */
		if (total_headroom > headroom) {
			extra_headroom = total_headroom - headroom;
			skb_push(skb, extra_headroom);
			memmove(skb->data, skb->data + extra_headroom,
				skb->len - extra_headroom);
			skb_trim(skb, skb->len - extra_headroom);
		}

		/*skb->data starts after the headroom.
		 * The headroom coniststs of
		 * |mlan_buffer|DMA|TXPD|pkt_type|tx_ctl|len|skb->data|
		 */
		pmbuf = (mlan_buffer *)skb->head;
		memset((t_u8 *)pmbuf, 0, sizeof(mlan_buffer));
		pmbuf->bss_index = priv->bss_index;
		woal_fill_mlan_buffer(priv, pmbuf, skb);
		pmbuf->data_offset -= (HEADER_SIZE + PKT_LEN);
		pmbuf->data_len = HEADER_SIZE + skb->len + PKT_LEN;
		len = skb->len;
		pmbuf->buf_type = MLAN_BUF_TYPE_RAW_DATA;
		pkt_type = MRVL_PKT_TYPE_MGMT_FRAME;

		// Add pkt type and tx control
		moal_memcpy_ext(priv->phandle, pmbuf->pbuf + pmbuf->data_offset,
				&pkt_type, sizeof(pkt_type), sizeof(pkt_type));
		moal_memcpy_ext(priv->phandle,
				pmbuf->pbuf + pmbuf->data_offset +
				sizeof(pkt_type), &tx_control,
				sizeof(tx_control), sizeof(tx_control));
		moal_memcpy_ext(priv->phandle,
				pmbuf->pbuf + pmbuf->data_offset + HEADER_SIZE,
				&len, sizeof(len), sizeof(len));

		skb_put(skb, MLAN_MAC_ADDR_LENGTH);
		memmove(skb->data + IEEE_HEADER_ADDR4_POS +
			MLAN_MAC_ADDR_LENGTH,
			skb->data + IEEE_HEADER_ADDR4_POS,
			skb->len -
			(IEEE_HEADER_ADDR4_POS + MLAN_MAC_ADDR_LENGTH));
		moal_memcpy_ext(priv->phandle,
				skb->data + IEEE_HEADER_ADDR4_POS, bcast_addr,
				MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);

		len = skb->len;
		pmbuf->data_len = HEADER_SIZE + skb->len + PKT_LEN;
		moal_memcpy_ext(priv->phandle,
				pmbuf->pbuf + MLAN_MIN_DATA_HEADER_LEN +
				HEADER_SIZE, &len, sizeof(len), sizeof(len));
	} else if (ieee80211_is_data(hdr->frame_control)) {
		headroom = MLAN_MIN_DATA_HEADER_LEN + sizeof(mlan_buffer) +
			priv->extra_tx_head_len;

		if ((int)skb_headroom(skb) < headroom) {
			PRINTM(MWARN, "Tx: Insufficient skb headroom %d\n",
			       skb_headroom(skb));
			/* Insufficient skb headroom - allocate a new skb */
			new_skb = skb_realloc_headroom(skb, headroom);
			if (unlikely(!new_skb)) {
				PRINTM(MERROR, "Tx: Cannot reallocate skb\n");
				dev_kfree_skb_any(skb);
				goto done;
			}
			if (new_skb != skb)
				dev_kfree_skb_any(skb);
			skb = new_skb;
			total_headroom = skb_headroom(skb);
			PRINTM(MINFO, "new skb headroom %d\n",
			       skb_headroom(skb));
		}

		/** If skb_realloc_headroom allocates more headroom than require
		 * there will be extraheadroom */
		if (total_headroom > headroom) {
			extra_headroom = total_headroom - headroom;
			skb_push(skb, extra_headroom);
			memmove(skb->data, skb->data + extra_headroom,
				skb->len - extra_headroom);
			skb_trim(skb, skb->len - extra_headroom);
		}

		pmbuf = (mlan_buffer *)skb->head;
		memset((t_u8 *)pmbuf, 0, sizeof(mlan_buffer));
		pmbuf->bss_index = priv->bss_index;
		woal_fill_mlan_buffer(priv, pmbuf, skb);
		pmbuf->buf_type = MLAN_BUF_TYPE_DATA;

		if (ieee80211_is_qos_nullfunc(hdr->frame_control) ||
		    ieee80211_is_nullfunc(hdr->frame_control)) {
			pmbuf->flags |= MLAN_BUF_FLAG_NULL_PKT;
		}

		if (ieee80211_is_data_qos(hdr->frame_control)) {
			/* TODO:Handle qos info in firmware */
			if (ieee80211_has_a4(hdr->frame_control))
				qos_ctl_offset = IEEE_HEADER_HAS_ADDR4;
			else
				qos_ctl_offset = IEEE_HEADER_NO_ADDR4;

			moal_memcpy_ext(priv->phandle, &pmbuf->priority,
					skb->data + qos_ctl_offset,
					IEEE80211_QOS_CTL_LEN,
					IEEE80211_QOS_CTL_LEN);

			memmove(skb->data + qos_ctl_offset,
				skb->data + qos_ctl_offset +
				IEEE80211_QOS_CTL_LEN,
				skb->len - (qos_ctl_offset +
					    IEEE80211_QOS_CTL_LEN));
			skb_trim(skb, skb->len - IEEE80211_QOS_CTL_LEN);
			pmbuf->data_len = skb->len;

			/* trigger BA session */
			sta_info = woal_get_mac80211_sta(control->sta);
			qos = le16_to_cpu(*
					  ((__le16 *)
					   ieee80211_get_qos_ctl(hdr)));
			tid = qos & 0xF;

			if (sta_info &&tid < MAX_TID) {
				sta_info->pkts[tid]++;
				PRINTM(MDATA, "sta_info_pkt = %d, tid %d\n",
				       sta_info->pkts[tid], tid);
				if (sta_info->ampdu_status_tx[tid] !=
				    AMPDU_SET &&
				    !(sta_info->pkts[tid] %
				      MAC80211_AMPDU_THRESHOLD)
				    &&!mac80211_rate_adapt) {
					ieee80211_start_tx_ba_session(control->
								      sta, tid,
								      0);
				}
			}
		}
	} else {
		PRINTM(MERROR, "%s unsupported TX frame control!\n", __func__);
		return;
	}

	if (mac80211_rate_adapt && pmbuf->buf_type == MLAN_BUF_TYPE_DATA) {
		pmbuf->flags |= MLAN_BUF_FLAG_TX_CTRL;
		tx_info = IEEE80211_SKB_CB(skb);
		if (!tx_info)
			PRINTM(MERROR,
			       "%s txinfo is null, send packet with basic rate\n",
			       __func__);
		else {
			pmbuf->u.tx_info.data_rate =
				woal_mac80211_convert_ieee_rate_to_nxp_rateid
				(tx_info->control.rates[0].idx,
				 tx_info->control.rates[0].flags);
			pmbuf->u.tx_info.retry_limit =
				tx_info->control.rates[0].count;
		}
	}
	if (tx_info && (tx_info->flags & IEEE80211_TX_CTL_RATE_CTRL_PROBE)) {
		pmbuf->flags |= MLAN_BUF_FLAG_RATE_ADAPT_PROBE_PKT;
	}
	woal_mac80211_set_tx_status(priv, pmbuf, skb);
	status = mlan_send_packet(priv->phandle->pmlan_adapter, pmbuf);
	switch (status) {
	case MLAN_STATUS_PENDING:
		atomic_inc(&priv->phandle->tx_pending);
		queue = skb_get_queue_mapping(skb);
		atomic_inc(&priv->wmm_tx_pending[queue]);
		if (atomic_read(&priv->wmm_tx_pending[queue]) >=
		    MAC80211_MAX_TX_PENDING)
			ieee80211_stop_queue(priv->hw, queue);
		queue_work(priv->phandle->workqueue, &priv->phandle->main_work);
		break;
	case MLAN_STATUS_SUCCESS:
		dev_kfree_skb_any(skb);
		break;
	case MLAN_STATUS_FAILURE:
	default:
		ieee80211_free_txskb(priv->hw, skb);
		break;
	}

done:
	/* TODO : inform mac80211 tx status */
	LEAVE();
}

/**
 *  @brief set mac address
 *
 *  @param priv         A pointer to moal_private structure
 *  @param wait_option  Wait option
 *
 *  @return             MLAN_STATUS_SUCCESS -- success, otherwise fail
 */

static mlan_status
woal_mac80211_mac_addr_ioctl(moal_private *priv, t_u8 wait_option)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_mac80211_cfg *mac80211_cfg = NULL;
	mlan_status status;
	ENTER();

	/* Allocate an IOCTL request buffer */
	req = (mlan_ioctl_req *)woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	mac80211_cfg = (mlan_ds_mac80211_cfg *) req->pbuf;
	mac80211_cfg->sub_command = MLAN_OID_MAC_ADDR;
	req->req_id = MLAN_IOCTL_MAC80211;
	req->action = MLAN_ACT_SET;
	moal_memcpy_ext(priv->phandle, &mac80211_cfg->params.mac_addr,
			priv->current_addr, sizeof(priv->current_addr),
			sizeof(mac80211_cfg->params.mac_addr));

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);
	if (status == MLAN_STATUS_SUCCESS) {
		HEXDUMP("priv->MacAddr:", priv->current_addr, ETH_ALEN);
	} else {
		PRINTM(MERROR,
		       "set mac address failed! status=%d, error_code=0x%x\n",
		       status, req->status_code);
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief This function adds interface
 *
 *  @param hw           A pointer to ieee80211_hw structure
 *  @param vif          A pointer to ieee80211_vif structure
 *
 *  @return             0->for success else failure
 */
static int
woal_mac80211_add_interface(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	moal_private *priv = hw->priv;

	ENTER();

	priv->vif = vif;
	moal_memcpy_ext(priv->phandle, priv->current_addr, vif->addr,
			sizeof(vif->addr), sizeof(priv->current_addr));

	if (MLAN_STATUS_SUCCESS !=
	    woal_mac80211_mac_addr_ioctl(priv, MOAL_IOCTL_WAIT)) {
		PRINTM(MERROR, "woal_mac80211_mac_addr_ioctl failed\n");
	}

	LEAVE();

	return 0;
}

/**
 *  @brief This function removes interface
 *
 *  @param hw           A pointer to ieee80211_hw structure
 *  @param vif          A pointer to ieee80211_vif structure
 *
 *  @return             NONE
 */
static void
woal_mac80211_remove_interface(struct ieee80211_hw *hw,
			       struct ieee80211_vif *vif)
{
	moal_private *priv = hw->priv;

	ENTER();

	priv->vif = NULL;

	LEAVE();
}

/**
 *  @brief This function Get the channe offset
 *
 *  @param chandef      A pointer to cfg80211_scan_def
 *
 *  @return             SEC_CHAN_NONE/SEC_CHAN_ABOVE/SEC_CHAN_BELOW
 */
static int
woal_mac80211_get_chan_offset(struct cfg80211_chan_def *chandef)
{
	switch (chandef->width) {
	case NL80211_CHAN_WIDTH_40:
		if (chandef->center_freq1 > chandef->chan->center_freq)
			return SEC_CHAN_ABOVE;
		else
			return SEC_CHAN_BELOW;
	case NL80211_CHAN_WIDTH_20_NOHT:
	case NL80211_CHAN_WIDTH_20:
	case NL80211_CHAN_WIDTH_80:
	default:
		return SEC_CHAN_NONE;
	}
	return SEC_CHAN_NONE;
}

/**
 *  @brief This function configures/changes the channel
 *
 *  @param priv         A pointer to moal_private structure
 *  @param conf         A pointer to ieee80211_conf structure
 *
 *  @return             0 on success; -EFAULT on failure
 */
static int
woal_mac80211_conf_change_channel(moal_private *priv,
				  struct ieee80211_conf *conf)
{
	mlan_ds_remain_chan chan_cfg;

	memset(&chan_cfg, 0, sizeof(mlan_ds_remain_chan));
	if (conf->chandef.chan->band == IEEE80211_BAND_2GHZ)
		chan_cfg.bandcfg.chanBand = BAND_2GHZ;
	else if (conf->chandef.chan->band == IEEE80211_BAND_5GHZ)
		chan_cfg.bandcfg.chanBand = BAND_5GHZ;

	switch (conf->chandef.width) {
	case NL80211_CHAN_WIDTH_20_NOHT:
	case NL80211_CHAN_WIDTH_20:
		chan_cfg.bandcfg.chanWidth = CHAN_BW_20MHZ;
		break;
	case NL80211_CHAN_WIDTH_40:
		chan_cfg.bandcfg.chanWidth = CHAN_BW_40MHZ;
		break;
	case NL80211_CHAN_WIDTH_80:
		chan_cfg.bandcfg.chanWidth = CHAN_BW_80MHZ;
		break;
	default:
		break;
	}

	chan_cfg.bandcfg.chan2Offset =
		woal_mac80211_get_chan_offset(&conf->chandef);
	chan_cfg.channel =
		ieee80211_frequency_to_channel(conf->chandef.chan->center_freq);
	if (MLAN_STATUS_SUCCESS !=
	    woal_set_remain_channel_ioctl(priv, MOAL_IOCTL_WAIT, &chan_cfg))
		return -EFAULT;

	return 0;
}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
/**
 *  @brief This function configures the monitor mode
 *
 *  @param priv         A pointer to moal_private structure
 *  @param conf         A pointer to ieee80211_conf structure
 *
 *  @return             0 on success; -EFAULT on failure
 */
static int
woal_mac80211_conf_mode_monitor(moal_private *priv, struct ieee80211_conf *conf)
{
	netmon_band_chan_cfg chan_cfg;
	int ret = MLAN_STATUS_SUCCESS;
	u8 en = MFALSE;

	if (conf->flags & IEEE80211_CONF_MONITOR) {
		/* enable monitor mode, return if already enabled */
		if (!priv->mac80211_mode_monitor)
			en = MTRUE;
		else
			return ret;
	} else {
		/* disable monitor mode if enabled earlier */
		if (!priv->mac80211_mode_monitor)
			return ret;
		else
			en = MFALSE;
	}

	chan_cfg.band = BAND_B | BAND_G | BAND_GN;
	chan_cfg.channel = 1;
	chan_cfg.chan_bandwidth = CHANNEL_BW_20MHZ;
	if (MLAN_STATUS_SUCCESS !=
	    woal_set_net_monitor(priv, MOAL_IOCTL_WAIT, en, 0x7, &chan_cfg))
		return -EFAULT;

	if (!ret)
		priv->mac80211_mode_monitor = en;
	return ret;
}
#endif
/**
 * @brief Request the driver to change the IEEE power save
 * mode
 *
 * @param priv            A pointer to moal_private structure
 * @param flags           ieee80211_conf flags
 *
 * @return                0 -- success, otherwise fail
 */
static int
woal_mac80211_set_power_mgmt(moal_private *priv, u32 flags)
{
	int enabled, disabled, ret = 0;

	ENTER();
	enabled = flags & IEEE80211_CONF_PS;

	if (enabled) {
		disabled = 0;
	} else {
		disabled = 1;
	}

	if (MLAN_STATUS_SUCCESS != woal_set_get_power_mgmt(priv, MLAN_ACT_SET,
							   &disabled, 0,
							   MOAL_NO_WAIT)) {
		ret = MLAN_STATUS_FAILURE;
	}
	LEAVE();
	return ret;
}

/**
 *  @brief mac80211 configuration
 *
 *  @param hw           A pointer to ieee80211_hw structure
 *  @param changed      Argument
 *
 *  @return             0->for success else failure
 */
static int
woal_mac80211_config(struct ieee80211_hw *hw, u32 changed)
{
	moal_private *priv = hw->priv;
	struct ieee80211_conf *conf = &hw->conf;
	int ret = 0;

	ENTER();
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	if (changed & IEEE80211_CONF_CHANGE_MONITOR) {
		ret = woal_mac80211_conf_mode_monitor(priv, conf);
	}
#endif
	if (changed & IEEE80211_CONF_CHANGE_CHANNEL) {
		ret = woal_mac80211_conf_change_channel(priv, conf);
	}

	if (changed & IEEE80211_CONF_CHANGE_PS) {
		ret = woal_mac80211_set_power_mgmt(priv, conf->flags);
	}

	LEAVE();
	return ret;
}

#if defined(MAC80211_SUPPORT_UAP) || defined(MAC80211_SUPPORT_MESH)
/**
 *  @brief set beacon
 *
 *  @param priv         A pointer to moal_private structure
 *  @param wait_option  Wait option
 *  @param pchan        A pointer to mlan_ds_beacon_cfg structure
 *
 *  @return             MLAN_STATUS_SUCCESS -- success, otherwise fail
 */

static mlan_status
woal_set_mac80211_beacon_ioctl(moal_private *priv,
			       t_u8 wait_option, mlan_ds_beacon_cfg * bcn_cfg)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_mac80211_cfg *mac80211_cfg = NULL;

	ENTER();
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_mac80211_cfg) +
					bcn_cfg->beacon_len);
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	mac80211_cfg = (mlan_ds_mac80211_cfg *) req->pbuf;
	mac80211_cfg->sub_command = MLAN_OID_SET_BEACON_CFG;
	req->req_id = MLAN_IOCTL_MAC80211;

	req->action = MLAN_ACT_SET;
	mac80211_cfg->params.bcn_cfg.beacon_len = bcn_cfg->beacon_len;
	mac80211_cfg->params.bcn_cfg.beacon_enable = bcn_cfg->beacon_enable;
	mac80211_cfg->params.bcn_cfg.beacon_period = bcn_cfg->beacon_period;
	mac80211_cfg->params.bcn_cfg.channel = bcn_cfg->channel;
	mac80211_cfg->params.bcn_cfg.bandcfg.chan2Offset =
		bcn_cfg->bandcfg.chan2Offset;
	mac80211_cfg->params.bcn_cfg.bandcfg.chanWidth =
		bcn_cfg->bandcfg.chanWidth;
	mac80211_cfg->params.bcn_cfg.bandcfg.chanBand =
		bcn_cfg->bandcfg.chanBand;
	moal_memcpy_ext(priv->phandle,
			&mac80211_cfg->params.bcn_cfg.beacon_data,
			bcn_cfg->beacon_data, bcn_cfg->beacon_len,
			bcn_cfg->beacon_len);
	ret = woal_request_ioctl(priv, req, wait_option);
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

static void
woal_mac80211_update_bcn(struct ieee80211_hw *hw,
			 struct ieee80211_vif *vif,
			 struct ieee80211_bss_conf *bss)
{
	moal_private *priv = hw->priv;
	struct sk_buff *beacon;
	mlan_ds_beacon_cfg *bcn_cfg;
	int bcn_length = 0;

	ENTER();

	beacon = ieee80211_beacon_get(hw, vif);

	if (beacon) {
		bcn_length = beacon->len;
	}

	bcn_cfg = kmalloc(sizeof(mlan_ds_beacon_cfg) + bcn_length, GFP_KERNEL);
	if (!bcn_cfg) {
		PRINTM(MERROR, "%s failed to allocate bcn_cfg\n", __func__);
		LEAVE();
		return;
	}
	memset(bcn_cfg, 0, sizeof(mlan_ds_beacon_cfg) + bcn_length);

	if (beacon) {
		moal_memcpy_ext(priv->phandle, bcn_cfg->beacon_data,
				beacon->data, beacon->len, bcn_length);
	}
	bcn_cfg->beacon_enable = bss->enable_beacon;
	bcn_cfg->beacon_period = bss->beacon_int;
	bcn_cfg->beacon_len = bcn_length;
	/** set channel and bandwidth */
	bcn_cfg->bandcfg.chan2Offset =
		woal_mac80211_get_chan_offset(&bss->chandef);
	bcn_cfg->channel =
		ieee80211_frequency_to_channel(bss->chandef.chan->center_freq);
	/** To match with FW ChanWidth_e enum */
	if (bss->chandef.width == NL80211_CHAN_WIDTH_20) {
		bcn_cfg->bandcfg.chanWidth = CHANWIDTH_20_MHZ;
	} else if (bss->chandef.width == NL80211_CHAN_WIDTH_10) {
		bcn_cfg->bandcfg.chanWidth = CHANWIDTH_10_MHZ;
	} else {
		bcn_cfg->bandcfg.chanWidth = bss->chandef.width;
	}
	bcn_cfg->bandcfg.chanBand = bss->chandef.chan->band;

	if (MLAN_STATUS_SUCCESS !=
	    woal_set_mac80211_beacon_ioctl(priv, MOAL_IOCTL_WAIT, bcn_cfg)) {
		PRINTM(MERROR, "woal_set_mac80211_beacon_ioctl failed\n");
	}

	kfree_skb(beacon);
	kfree(bcn_cfg);

	LEAVE();
	return;
}

/**
 *  @brief Set the beacon configuration to firmware
 *
 *  @param hw           A pointer to ieee80211_hw structure
 *  @param vif          A pointer to the ieee80211_vif structure
 *  @param bss          A pointer to bss structure
 *
 *  @return             NONE
 */
static void
woal_mac80211_bss_info_changed(struct ieee80211_hw *hw,
			       struct ieee80211_vif *vif,
			       struct ieee80211_bss_conf *bss, u32 changed)
{
	ENTER();

	if (vif->type != NL80211_IFTYPE_AP &&
	    vif->type != NL80211_IFTYPE_ADHOC && !ieee80211_vif_is_mesh(vif)) {
		PRINTM(MERROR, "%s not a valid interface!\n", __func__);
		LEAVE();
		return;
	}

	if (changed & (BSS_CHANGED_BEACON | BSS_CHANGED_BEACON_INT |
		       BSS_CHANGED_BEACON_ENABLED)) {
		woal_mac80211_update_bcn(hw, vif, bss);
	}

	LEAVE();
	return;
}
#endif

/**
 *  @brief Set the configure filter
 *
 *  @param hw            A pointer to ieee80211_hw structure
 *  @param changed_flags Changed_flags
 *  @param total_flags   Total_flags
 *  @param multicast     multicast
 *
 *  @return             NONE
 */
static void
woal_mac80211_configure_filter(struct ieee80211_hw *hw,
			       unsigned int changed_flags,
			       unsigned int *total_flags, u64 multicast)
{
	moal_private *priv = hw->priv;
	t_u32 mgmt_subtype_mask = 0;

	ENTER();

	if (changed_flags & FIF_BCN_PRBRESP_PROMISC) {
		if (*total_flags & FIF_BCN_PRBRESP_PROMISC) {
			mgmt_subtype_mask |= (BIT(5) | BIT(4) | BIT(7));
			woal_reg_rx_mgmt_ind(priv, MLAN_ACT_SET,
					     &mgmt_subtype_mask, MOAL_NO_WAIT);
		}
	}
	*total_flags = 0;

	LEAVE();
}

/**
 *  @brief Send delelte all BA command to MLAN
 *
 *  @param priv          A pointer to moal_private structure
 *  @param wait_option          Wait option
 *
 *  @return              MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
static mlan_status
woal_mac80211_delba_all(moal_private *priv, t_u8 wait_option, t_u8 *mac_addr)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_11n_cfg *cfg_11n = NULL;
	mlan_ds_11n_delba *del_ba = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	int ret = 0;

	ENTER();

	req = (mlan_ioctl_req *)
		woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	cfg_11n = (mlan_ds_11n_cfg *)req->pbuf;
	req->req_id = MLAN_IOCTL_11N_CFG;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_DELBA;

	del_ba = &cfg_11n->param.del_ba;
	memset(del_ba, 0, sizeof(mlan_ds_11n_delba));
	del_ba->direction = DELBA_RX | DELBA_TX;
	del_ba->tid = DELBA_ALL_TIDS;
	moal_memcpy_ext(priv->phandle, del_ba->peer_mac_addr, mac_addr,
			ETH_ALEN, ETH_ALEN);

	status = woal_request_ioctl(priv, req, wait_option);

	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Add/del mac80211 based STA
 *
 *  @param priv         A pointer to moal_private structure
 *  @param wait_option  Wait option
 *  @param pchan        A pointer to mlan_ds_beacon_cfg structure
 *
 *  @return             MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
static mlan_status
woal_mac80211_sta_state_ioctl(moal_private *priv,
			      t_u8 wait_option, mlan_ds_sta_state * state)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_mac80211_cfg *mac80211_cfg = NULL;

	ENTER();
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_mac80211_cfg) +
					state->tlv_len);
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	mac80211_cfg = (mlan_ds_mac80211_cfg *) req->pbuf;
	mac80211_cfg->sub_command = MLAN_OID_STA_STATE;
	req->req_id = MLAN_IOCTL_MAC80211;

	req->action = MLAN_ACT_SET;
	moal_memcpy_ext(priv->phandle, &mac80211_cfg->params.sta_state, state,
			sizeof(mlan_ds_sta_state) + state->tlv_len,
			sizeof(mlan_ds_sta_state) + state->tlv_len);
	ret = woal_request_ioctl(priv, req, wait_option);
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Sends peer information to firmware
 *
 *  @param hw            A pointer to ieee80211_hw structure
 *  @vif		 A pointer to ieee80211_vif structure
 *  @sta		 A pointer to ieee80211_sta structure
 *  @old_state 		 ieee80211_sta_state enum
 *  @new_state 		 ieee80211_sta_state enum
 *
 *  @return              0 for success else failure
 */
static int
woal_mac80211_sta_state(struct ieee80211_hw *hw,
			struct ieee80211_vif *vif,
			struct ieee80211_sta *sta,
			enum ieee80211_sta_state old_state,
			enum ieee80211_sta_state new_state)
{
	moal_private *priv = hw->priv;
	mlan_ds_sta_state state;
	struct ieee80211_conf *conf = &priv->hw->conf;
	struct ieee80211_supported_band *sband;
	struct mac80211_sta *sta_info;

	ENTER();

	if (priv->phandle->surprise_removed) {
		PRINTM(MERROR,
		       "sta_state change not allowed after card removal!\n");
		LEAVE();
		return 0;
	}

	sband = hw->wiphy->bands[conf->chandef.chan->band];
	memset(&state, 0, sizeof(mlan_ds_sta_state));

	sta_info = woal_get_mac80211_sta(sta);

	if (old_state == IEEE80211_STA_NOTEXIST &&
	    new_state == IEEE80211_STA_NONE) {
		state.add = STA_ADD;
		moal_memcpy_ext(priv->phandle, state.peer_mac, sta->addr,
				ETH_ALEN, ETH_ALEN);
		/** Add station */
		if (MLAN_STATUS_SUCCESS !=
		    woal_mac80211_sta_state_ioctl(priv, MOAL_IOCTL_WAIT,
						  &state)) {
			PRINTM(MERROR, "STA_ADD IOCTL failed\n");
			LEAVE();
			return -EFAULT;
		}
		PRINTM(MINFO, "STA_ADD IOCTL returned successfully\n");

		if (sta_info) {
			memset(sta_info, 0, sizeof(*sta_info));
			spin_lock_bh(&priv->sta_list_lock);
			list_add_tail(&sta_info->list, &priv->sta_list);
			spin_unlock_bh(&priv->sta_list_lock);
		}
		LEAVE();
		return 0;

	} else if ((old_state == IEEE80211_STA_NONE &&
		    new_state == IEEE80211_STA_NOTEXIST)) {
		state.add = STA_REMOVE;
		moal_memcpy_ext(priv->phandle, state.peer_mac, sta->addr,
				ETH_ALEN, ETH_ALEN);
		/** Delete existing station */
		if (MLAN_STATUS_SUCCESS !=
		    woal_mac80211_sta_state_ioctl(priv, MOAL_IOCTL_WAIT,
						  &state)) {
			PRINTM(MERROR, "STA_REMOVE IOCTL failed\n");
			LEAVE();
			return -EFAULT;
		}
		PRINTM(MINFO, "STA_REMOVE IOCTL returned successfully\n");

		spin_lock_bh(&priv->sta_list_lock);
		if (sta_info)
			list_del(&sta_info->list);
		spin_unlock_bh(&priv->sta_list_lock);

		LEAVE();
		return 0;

	} else if (old_state == IEEE80211_STA_AUTH &&
		   new_state == IEEE80211_STA_ASSOC) {
		t_u8 *pos;
		MrvlIEtypes_Data_t *tlv;
		mlan_ds_sta_state *new_sta = NULL;
		u16 tlv_len = 0;
		if (sta->ht_cap.ht_supported)
			tlv_len = sizeof(struct ieee80211_ht_cap) +
				sizeof(MrvlIEtypesHeader_t);
		if (sta->vht_cap.vht_supported)
			tlv_len += sizeof(struct ieee80211_vht_cap) +
				sizeof(MrvlIEtypesHeader_t);
		new_sta = kzalloc(sizeof(mlan_ds_sta_state) + tlv_len,
				  GFP_KERNEL);
		if (!new_sta) {
			PRINTM(MERROR,
			       "Fail to allocate memeory for new station\n");
			LEAVE();
			return -ENOMEM;
		}
		moal_memcpy_ext(priv->phandle, new_sta->peer_mac, sta->addr,
				ETH_ALEN, ETH_ALEN);
		new_sta->add = STA_ASSOC;
		new_sta->aid = sta->aid;
		new_sta->bitmap_supp_rates = sta->supp_rates[sband->band];
		new_sta->bw = sta->bandwidth;
		new_sta->wme = sta->wme;
		new_sta->uapsd_queues = sta->uapsd_queues;
		new_sta->bcn_interval = vif->bss_conf.beacon_int;
		new_sta->bcn_dtim_period = vif->bss_conf.dtim_period;
		new_sta->tlv_len = 0;
		pos = &new_sta->tlv_buf[0];
		if (sta->ht_cap.ht_supported) {
			struct ieee80211_ht_cap ht_cap;
			memset(&ht_cap, 0, sizeof(struct ieee80211_ht_cap));
			ht_cap.cap_info = (__force __le16) sta->ht_cap.cap;
			ht_cap.ampdu_params_info =
				(sta->ht_cap.ampdu_factor & 3) |
				((sta->ht_cap.ampdu_density & 7) << 2);
			moal_memcpy_ext(priv->phandle, &ht_cap.mcs,
					&sta->ht_cap.mcs,
					sizeof(struct ieee80211_mcs_info),
					sizeof(struct ieee80211_mcs_info));
			tlv = (MrvlIEtypes_Data_t *)pos;
			tlv->header.type = HT_CAPABILITY;
			tlv->header.len = sizeof(struct ieee80211_ht_cap);
			moal_memcpy_ext(priv->phandle, tlv->data, &ht_cap,
					tlv->header.len, tlv->header.len);
			pos += sizeof(MrvlIEtypesHeader_t) + tlv->header.len;
			new_sta->tlv_len +=
				sizeof(MrvlIEtypesHeader_t) + tlv->header.len;
		}

		if (sta->vht_cap.vht_supported) {
			struct ieee80211_vht_cap vht_cap;

			memset(&vht_cap, 0, sizeof(struct ieee80211_vht_cap));
			vht_cap.vht_cap_info =
				(__force __le32) sta->vht_cap.cap;
			moal_memcpy_ext(priv->phandle, &vht_cap.supp_mcs,
					&sta->vht_cap.vht_mcs,
					sizeof(struct ieee80211_vht_mcs_info),
					sizeof(struct ieee80211_vht_mcs_info));

			tlv = (MrvlIEtypes_Data_t *)pos;
			tlv->header.type = VHT_CAPABILITY;
			tlv->header.len = sizeof(struct ieee80211_vht_cap);
			moal_memcpy_ext(priv->phandle, tlv->data, &vht_cap,
					tlv->header.len, tlv->header.len);
			pos += sizeof(MrvlIEtypesHeader_t) + tlv->header.len;
			new_sta->tlv_len +=
				sizeof(MrvlIEtypesHeader_t) + tlv->header.len;
		}

		if (MLAN_STATUS_SUCCESS !=
		    woal_mac80211_sta_state_ioctl(priv, MOAL_IOCTL_WAIT,
						  new_sta)) {
			kfree(new_sta);
			PRINTM(MERROR, "STA_ASSOC IOCTL failed\n");
			LEAVE();
			return -EFAULT;
		}
		kfree(new_sta);
		PRINTM(MINFO, "STA_ASSOC IOCTL returned successfully\n");
		LEAVE();
		return 0;

	} else if (old_state == IEEE80211_STA_ASSOC &&
		   new_state == IEEE80211_STA_AUTH) {
		moal_memcpy_ext(priv->phandle, state.peer_mac, sta->addr,
				ETH_ALEN, ETH_ALEN);
		state.add = STA_DISSASSOC;
		/** Delete all the existing TX and RX BA stream with
		 * respect to the mac addr */
		woal_mac80211_delba_all(priv, MOAL_IOCTL_WAIT, sta->addr);
		if (MLAN_STATUS_SUCCESS !=
		    woal_mac80211_sta_state_ioctl(priv, MOAL_IOCTL_WAIT,
						  &state)) {
			PRINTM(MERROR, "STA_DISSASSOC IOCTL failed\n");
			LEAVE();
			return -EFAULT;
		}
		PRINTM(MINFO, "STA_DISSASSOC IOCTL returned successfully\n");
		LEAVE();
		return 0;
	}

	LEAVE();
	return 0;
}

/**
 *  @brief Function sets RTS threshold value
 *
 *  @param hw           A pointer to ieee80211_hw structure
 *  @param value        value
 *
 *  @return             NONE
 */
static int
woal_mac80211_set_rts_threshold(struct ieee80211_hw *hw, u32 value)
{
	int ret = MLAN_STATUS_SUCCESS;
	moal_private *priv = hw->priv;

	ENTER();

	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_rts(priv, MLAN_ACT_SET, MOAL_IOCTL_WAIT, &value))
		ret = -EFAULT;

	LEAVE();
	return ret;
}

/**
 *    @brief WOAL get time stamp from firmware
 *
 *    @param priv         moal private
 *    @param wait_option  wait option
 *    @param tsf          timestamp
 *    @return             Number of bytes written, negative for failure
 */
static int
woal_mac80211_get_tsf_ioctl(moal_private *priv, t_u8 wait_option, t_u64 *tsf)
{
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc_cfg = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	if (!priv) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_GET;
	misc_cfg = (mlan_ds_misc_cfg *)req->pbuf;
	misc_cfg->sub_command = MLAN_OID_MISC_GET_TSF;

	status = woal_request_ioctl(priv, req, wait_option);
	if (status != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "woal_request_ioctl return error!\n");
		ret = -EFAULT;
		goto done;
	}
	*tsf = misc_cfg->param.misc_tsf;
	ret = sizeof(*tsf);
done:
	if (req && (status != MLAN_STATUS_PENDING))
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Function gets tsf value
 *
 *  @param hw           A pointer to ieee80211_hw structure
 *  @param vif          A pointer to ieee80211_vif structure
 *
 *  @return             tsf value
 */
static u64
woal_mac80211_get_tsf(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	moal_private *priv = hw->priv;
	u64 tsf = 0;

	ENTER();

	if (MLAN_STATUS_SUCCESS !=
	    woal_mac80211_get_tsf_ioctl(priv, MOAL_IOCTL_WAIT, &tsf))
		PRINTM(MINFO, "IOCTL failed\n");

	PRINTM(MINFO, "IOCTL returned successfully\n");

	LEAVE();
	return tsf;
}

/**
 *  @brief Set/Enable encryption key
 *
 *  @param priv             A pointer to moal_private structure
 *  @param is_enable_wep    Enable WEP default key
 *  @param cipher           Cipher suite selector
 *  @param key              A pointer to key
 *  @param key_len          Key length
 *  @param seq              A pointer to sequence
 *  @param seq_len          Sequence length
 *  @param key_index        Key index
 *  @param addr             MAC for which key is to be set
 *  @param disable          Key disabled or not
 *  @param pairwise         Key pairwise or not
 *  @param local_addr       Self MAC address
 *  @param wait_option      wait option
 *
 *  @return                 MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
static mlan_status
woal_set_mac80211_key(moal_private *priv, t_u8 is_enable_wep,
		      t_u32 cipher, const t_u8 *key, int key_len,
		      const t_u8 *seq, int seq_len, t_u8 key_index,
		      const t_u8 *addr, int disable, bool pairwise,
		      struct ieee80211_vif *vif, t_u8 wait_option)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u8 bcast_addr[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		PRINTM(MERROR, "%s: fail to allocate memory\n", __func__);
		goto done;
	}

	/* Fill request buffer */
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_ENCRYPT_KEY;
	req->req_id = MLAN_IOCTL_SEC_CFG;
	req->action = MLAN_ACT_SET;

	if (!disable) {
		if (cipher != WLAN_CIPHER_SUITE_WEP40 &&
		    cipher != WLAN_CIPHER_SUITE_WEP104 &&
		    cipher != WLAN_CIPHER_SUITE_TKIP &&
		    cipher != WLAN_CIPHER_SUITE_CCMP
		    && cipher != WLAN_CIPHER_SUITE_AES_CMAC) {
			PRINTM(MERROR, "%s: invalid cipher suite specified\n",
			       __func__);
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}

		sec->param.encrypt_key.key_index = key_index;
		if (key && key_len) {
			moal_memcpy_ext(priv->phandle,
					sec->param.encrypt_key.key_material,
					key, key_len, MLAN_MAX_KEY_LENGTH);
			sec->param.encrypt_key.key_len = key_len;
		}

		if (addr) {
			moal_memcpy_ext(priv->phandle,
					sec->param.encrypt_key.mac_addr, addr,
					ETH_ALEN, ETH_ALEN);
			if (!memcmp(sec->param.encrypt_key.mac_addr, bcast_addr,
				    ETH_ALEN) || !pairwise)
				 sec->param.encrypt_key.key_flags =
					KEY_FLAG_GROUP_KEY;
			else
				sec->param.encrypt_key.key_flags =
					KEY_FLAG_SET_TX_KEY;
		} else {
			moal_memcpy_ext(priv->phandle,
					sec->param.encrypt_key.mac_addr,
					bcast_addr, ETH_ALEN, ETH_ALEN);
#ifdef MAC80211_SUPPORT_MESH
			if (vif->type == NL80211_IFTYPE_MESH_POINT)
				sec->param.encrypt_key.key_flags =
					KEY_FLAG_GROUP_KEY |
					KEY_FLAG_SET_GRP_TX_KEY;
			else
#endif
				sec->param.encrypt_key.key_flags =
					KEY_FLAG_GROUP_KEY |
					KEY_FLAG_SET_TX_KEY;
		}

		if (seq && seq_len) {
			moal_memcpy_ext(priv->phandle,
					sec->param.encrypt_key.pn, seq, seq_len,
					PN_SIZE);
			sec->param.encrypt_key.key_flags |=
				KEY_FLAG_RX_SEQ_VALID;
		}

	} else {
		if (key_index == KEY_INDEX_CLEAR_ALL)
			sec->param.encrypt_key.key_disable = MTRUE;
		else {
			sec->param.encrypt_key.key_index = key_index;
			sec->param.encrypt_key.key_remove = MTRUE;
		}
		sec->param.encrypt_key.key_flags = KEY_FLAG_REMOVE_KEY;
		if (addr)
			moal_memcpy_ext(priv->phandle,
					sec->param.encrypt_key.mac_addr, addr,
					ETH_ALEN, ETH_ALEN);
	}
	/* Send IOCTL request to MLAN */
	ret = woal_request_ioctl(priv, req, wait_option);

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Function sets the key
 *
 *  @param hw           A pointer to ieee80211_hw structure
 *  @param cmd          set_key_cmd enum
 *  @param vif          A pointer to ieee80211_vif structure
 *  @param sta          A pointer to ieee80211_sta structure
 *  @param key          A pointer to ieee80211_key_conf structure
 *
 *  @return             1 or 0
 */
static int
woal_mac80211_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
		      struct ieee80211_vif *vif,
		      struct ieee80211_sta *sta, struct ieee80211_key_conf *key)
{
	moal_private *priv = hw->priv;
	int disable;
	bool pairwise;
	int is_enable_wep = 0;

	ENTER();

	if (cmd == SET_KEY)
		disable = 0;
	else
		disable = 1;

	if ((key->flags & IEEE80211_KEY_FLAG_PAIRWISE))
		pairwise = 1;
	else
		pairwise = 0;

	if (key && key->keylen > MLAN_MAX_KEY_LENGTH) {
		PRINTM(MERROR, "%s: invalid key_len\n", __func__);
		LEAVE();
		return -EINVAL;
	}

	if (woal_set_mac80211_key
	    (priv, is_enable_wep, key->cipher, key->key, key->keylen, NULL, 0,
	     key->keyidx, sta ? sta->addr : NULL, disable, pairwise, vif,
	     MOAL_IOCTL_WAIT) != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "%s: error %s crypto keys\n", __func__,
		       disable ? "deleting" : "adding");
		LEAVE();
		return -EFAULT;
	}

	LEAVE();
	return 0;
}

#ifdef CONFIG_PM
static int
woal_mac80211_suspend(struct ieee80211_hw *hw, struct cfg80211_wowlan *wowlan)
{
	ENTER();
	LEAVE();
	return 0;
}

static int
woal_mac80211_resume(struct ieee80211_hw *hw)
{
	ENTER();
	LEAVE();
	return 0;
}
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 6, 0)
/**
 *  @brief mac80211 ampdu configuration
 *
 *  @param hw           A pointer to ieee80211_hw structure
 *
 *  @return             0->for success else failure
 */
static int
woal_mac80211_ampdu_cfg_txrx(struct ieee80211_hw *hw,
			     struct ieee80211_ampdu_params *params, bool is_tx)
{
	moal_private *priv = hw->priv;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_mac80211_cfg *mac80211_cfg = NULL;
	mlan_ds_ampdu_cfg ampdu_cfg;

	ENTER();

	moal_memcpy_ext(priv->phandle, ampdu_cfg.peer_addr, params->sta->addr,
			ETH_ALEN, ETH_ALEN);
	ampdu_cfg.tid = params->tid;
	ampdu_cfg.seq_num = params->ssn;
	ampdu_cfg.buf_size = params->buf_size;
	ampdu_cfg.timeout = params->timeout;
	ampdu_cfg.is_tx = is_tx;

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_mac80211_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	mac80211_cfg = (mlan_ds_mac80211_cfg *) req->pbuf;
	mac80211_cfg->sub_command = MLAN_OID_AMPDU_CFG_TXRX;
	req->req_id = MLAN_IOCTL_MAC80211;
	req->action = MLAN_ACT_SET;
	moal_memcpy_ext(priv->phandle, &mac80211_cfg->params.ampdu_cfg,
			&ampdu_cfg, sizeof(mlan_ds_ampdu_cfg),
			sizeof(mlan_ds_ampdu_cfg));

	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief mac80211 ampdu delete configuration
 *
 *  @param hw           A pointer to ieee80211_hw structure
 *
 *  @return             0->for success else failure
 */
static int
woal_mac80211_ampdu_delete(struct ieee80211_hw *hw,
			   struct ieee80211_ampdu_params *params, bool is_tx)
{
	moal_private *priv = hw->priv;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ioctl_req *req = NULL;
	mlan_ds_mac80211_cfg *mac80211_cfg = NULL;
	mlan_ds_ampdu_cfg ampdu_cfg;

	ENTER();

	moal_memcpy_ext(priv->phandle, ampdu_cfg.peer_addr, params->sta->addr,
			ETH_ALEN, ETH_ALEN);
	ampdu_cfg.tid = params->tid;
	ampdu_cfg.seq_num = params->ssn;
	ampdu_cfg.buf_size = params->buf_size;
	ampdu_cfg.timeout = params->timeout;
	ampdu_cfg.is_tx = is_tx;

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_mac80211_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	mac80211_cfg = (mlan_ds_mac80211_cfg *) req->pbuf;
	mac80211_cfg->sub_command = MLAN_OID_AMPDU_DELETE;
	req->req_id = MLAN_IOCTL_MAC80211;
	req->action = MLAN_ACT_SET;
	moal_memcpy_ext(priv->phandle, &mac80211_cfg->params.ampdu_cfg,
			&ampdu_cfg, sizeof(mlan_ds_ampdu_cfg),
			sizeof(mlan_ds_ampdu_cfg));

	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

static int
woal_mac80211_ampdu_action(struct ieee80211_hw *hw,
			   struct ieee80211_vif *vif,
			   struct ieee80211_ampdu_params *params)
{
	int ret = 0;
	struct mac80211_sta *sta_info;
	moal_private *priv = hw->priv;
	if (priv->phandle->surprise_removed == MTRUE)
		return ret;

	switch (params->action) {
	case IEEE80211_AMPDU_RX_START:
		ret = woal_mac80211_ampdu_cfg_txrx(hw, params, false);
		if (ret == MLAN_STATUS_SUCCESS) {
			sta_info = woal_get_mac80211_sta(params->sta);
			if (sta_info)
				sta_info->ampdu_status_rx[params->tid] =
					AMPDU_SET;
		}
		break;
	case IEEE80211_AMPDU_RX_STOP:
		sta_info = woal_get_mac80211_sta(params->sta);
		if (sta_info) {
			if (sta_info->ampdu_status_rx[params->tid] == AMPDU_SET) {
				ret = woal_mac80211_ampdu_delete(hw, params,
								 false);
				if (ret == MLAN_STATUS_SUCCESS) {
					sta_info->ampdu_status_rx[params->tid] =
						AMPDU_CLR;
				}
			}
		}
		break;
	case IEEE80211_AMPDU_TX_START:
		ieee80211_start_tx_ba_cb_irqsafe(vif, params->sta->addr,
						 params->tid);
		break;
	case IEEE80211_AMPDU_TX_OPERATIONAL:
		ret = woal_mac80211_ampdu_cfg_txrx(hw, params, true);
		if (ret == MLAN_STATUS_SUCCESS) {
			sta_info = woal_get_mac80211_sta(params->sta);
			if (sta_info)
				sta_info->ampdu_status_tx[params->tid] =
					AMPDU_SET;
		}
		break;
	case IEEE80211_AMPDU_TX_STOP_CONT:
	case IEEE80211_AMPDU_TX_STOP_FLUSH:
	case IEEE80211_AMPDU_TX_STOP_FLUSH_CONT:
		ieee80211_stop_tx_ba_cb_irqsafe(vif, params->sta->addr,
						params->tid);
		sta_info = woal_get_mac80211_sta(params->sta);
		if (sta_info) {
			if (sta_info->ampdu_status_tx[params->tid] == AMPDU_SET) {
				ret = woal_mac80211_ampdu_delete(hw, params,
								 true);
				if (ret == MLAN_STATUS_SUCCESS) {
					sta_info->ampdu_status_tx[params->tid] =
						AMPDU_CLR;
				}
			}
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}
#endif

static t_void
woal_beacon_update_work(struct work_struct *work)
{
	moal_private *priv =
		container_of(work, moal_private, beacon_update_work);
	struct ieee80211_hw *hw = priv->hw;
	struct ieee80211_vif *vif = priv->vif;
	struct ieee80211_bss_conf *bss = &vif->bss_conf;

	ENTER();

	if (!vif) {
		PRINTM(MERROR, "%s failed to get vif\n", __func__);
		LEAVE();
		return;
	}

	woal_mac80211_update_bcn(hw, vif, bss);

	LEAVE();
}

static int
woal_mac80211_set_tim(struct ieee80211_hw *hw, struct ieee80211_sta *sta,
		      bool set)
{
	moal_private *priv = hw->priv;

	ENTER();

	queue_work(priv->beacon_update_workqueue, &priv->beacon_update_work);

	LEAVE();
	return 0;
}

/** mac80211 operations **/
static const struct ieee80211_ops woal_mac80211_ops = {
	.start = woal_mac80211_start,
	.stop = woal_mac80211_stop,
	.tx = woal_mac80211_tx,
	.add_interface = woal_mac80211_add_interface,
	.remove_interface = woal_mac80211_remove_interface,
	.config = woal_mac80211_config,
#if defined(MAC80211_SUPPORT_UAP) || defined(MAC80211_SUPPORT_MESH)
	.bss_info_changed = woal_mac80211_bss_info_changed,
#endif
	.configure_filter = woal_mac80211_configure_filter,
	.sta_state = woal_mac80211_sta_state,
	.set_rts_threshold = woal_mac80211_set_rts_threshold,
	.get_tsf = woal_mac80211_get_tsf,
	.set_key = woal_mac80211_set_key,
	.set_tim = woal_mac80211_set_tim,
#ifdef CONFIG_PM
	.suspend = woal_mac80211_suspend,
	.resume = woal_mac80211_resume,
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 6, 0)
	.ampdu_action = woal_mac80211_ampdu_action,
#endif
};

/**
 *  @brief Sets up the CFG802.11 specific HT capability fields
 *  with default values
 *
 *  @param ht_info      A pointer to ieee80211_sta_ht_cap structure
 *  @param dev_cap      Device capability informations
 *  @param mcs_set      Device MCS sets
 *
 *  @return             N/A
 */
static void
woal_mac80211_setup_ht_cap(struct ieee80211_sta_ht_cap *ht_info,
			   t_u32 dev_cap, t_u8 *mcs_set)
{
	ENTER();

	ht_info->ht_supported = true;
	ht_info->ampdu_factor = 0x3;
	ht_info->ampdu_density = 0;

	memset(&ht_info->mcs, 0, sizeof(ht_info->mcs));
	ht_info->cap = 0;
	if (mcs_set)
		moal_memcpy_ext(NULL, ht_info->mcs.rx_mask, mcs_set,
				sizeof(ht_info->mcs.rx_mask),
				sizeof(ht_info->mcs.rx_mask));
	if (dev_cap & MBIT(8))	/* 40Mhz intolarance enabled */
		ht_info->cap |= IEEE80211_HT_CAP_40MHZ_INTOLERANT;
	if (dev_cap & MBIT(17))	/* Channel width 20/40Mhz support */
		ht_info->cap |= IEEE80211_HT_CAP_SUP_WIDTH_20_40;
	if ((dev_cap >> 20) & 0x03)	/* Delayed ACK supported */
		ht_info->cap |= IEEE80211_HT_CAP_DELAY_BA;
	if (dev_cap & MBIT(22))	/* Rx LDPC supported */
		ht_info->cap |= IEEE80211_HT_CAP_LDPC_CODING;
	if (dev_cap & MBIT(23))	/* Short GI @ 20Mhz supported */
		ht_info->cap |= IEEE80211_HT_CAP_SGI_20;
	if (dev_cap & MBIT(24))	/* Short GI @ 40Mhz supported */
		ht_info->cap |= IEEE80211_HT_CAP_SGI_40;
	if (dev_cap & MBIT(25))	/* Tx STBC supported */
		ht_info->cap |= IEEE80211_HT_CAP_TX_STBC;
	if (dev_cap & MBIT(26))	/* Rx STBC supported */
		ht_info->cap |= IEEE80211_HT_CAP_RX_STBC;
	if (dev_cap & MBIT(27))	/* MIMO PS supported */
		ht_info->cap |= 0;	/* WLAN_HT_CAP_SM_PS_STATIC */
	else			/* Disable HT SM PS */
		ht_info->cap |= IEEE80211_HT_CAP_SM_PS;
	if (dev_cap & MBIT(29))	/* Green field supported */
		ht_info->cap |= IEEE80211_HT_CAP_GRN_FLD;
	if (dev_cap & MBIT(31))	/* MAX AMSDU supported */
		ht_info->cap |= IEEE80211_HT_CAP_MAX_AMSDU;
	/* DSSS/CCK in 40Mhz supported */
	ht_info->cap |= IEEE80211_HT_CAP_DSSSCCK40;
	ht_info->mcs.tx_params = IEEE80211_HT_MCS_TX_DEFINED;

	LEAVE();
}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
/**
 *  @brief Sets up the CFG802.11 specific VHT capability fields
 *  with default values
 *
 * @param priv         A pointer to moal private structure
 *  @param vht_cap      A pointer to ieee80211_sta_vht_cap structure
 *
 *  @return             N/A
 */
static void
woal_mac80211_setup_vht_cap(moal_private *priv,
			    struct ieee80211_sta_vht_cap *vht_cap)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_11ac_cfg *cfg_11ac = NULL;
	mlan_status status;

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11ac_cfg));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		PRINTM(MERROR, "Fail to allocate buf for setup vht_cap\n");
		goto done;
	}
	cfg_11ac = (mlan_ds_11ac_cfg *)req->pbuf;
	cfg_11ac->sub_command = MLAN_OID_11AC_VHT_CFG;
	req->req_id = MLAN_IOCTL_11AC_CFG;
	req->action = MLAN_ACT_GET;
	cfg_11ac->param.vht_cfg.band = BAND_SELECT_A;
	cfg_11ac->param.vht_cfg.txrx = MLAN_RADIO_RX;
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (MLAN_STATUS_SUCCESS != status) {
		PRINTM(MERROR, "Fail to get vht_cfg\n");
		goto done;
	}
	vht_cap->vht_supported = true;
	vht_cap->cap = cfg_11ac->param.vht_cfg.vht_cap_info;
	vht_cap->vht_mcs.rx_mcs_map =
		(__force __le16) cfg_11ac->param.vht_cfg.vht_rx_mcs;
	vht_cap->vht_mcs.rx_highest =
		(__force __le16) cfg_11ac->param.vht_cfg.vht_rx_max_rate;
	vht_cap->vht_mcs.tx_mcs_map =
		(__force __le16) cfg_11ac->param.vht_cfg.vht_tx_mcs;
	vht_cap->vht_mcs.tx_highest =
		(__force __le16) cfg_11ac->param.vht_cfg.vht_tx_max_rate;
	PRINTM(MCMND,
	       "vht_cap=0x%x rx_mcs_map=0x%x rx_max=0x%x"
	       "tx_mcs_map=0x%x tx_max=0x%x\n", vht_cap->cap,
	       vht_cap->vht_mcs.rx_mcs_map, vht_cap->vht_mcs.rx_highest,
	       vht_cap->vht_mcs.tx_mcs_map, vht_cap->vht_mcs.tx_highest);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return;
}
#endif

/**
 * @brief This function register with the mac80211 subsystem
 and interface is created.This function is
 called only for bss_type = MAC80211
 *
 *  @param handle    A pointer to moal_handle structure
 *  @param bss_index BSS index number
 *  @param bss_type  BSS type
 *
 *  @return          A pointer to the new priv structure
 */

moal_private *
woal_add_mac80211_interface(moal_handle *handle, t_u8 bss_index, t_u8 bss_type)
{
	moal_private *priv;
	mlan_fw_info fw_info;
	struct ieee80211_hw *hw;
	mlan_ioctl_req *req;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_11n_cfg *cfg_11n;
	int status = 0;

	ENTER();

	hw = ieee80211_alloc_hw(sizeof(*priv), &woal_mac80211_ops);
	if (!hw) {
		PRINTM(MERROR, "allocation of hw failed!\n");
		goto err_alloc_hw;
	}
	SET_IEEE80211_DEV(hw, handle->hotplug_device);
	priv = hw->priv;
	priv->hw = hw;
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 1, 46)
	ieee80211_hw_set(hw, AMPDU_AGGREGATION);
	if (!mac80211_rate_adapt)
		ieee80211_hw_set(hw, HAS_RATE_CONTROL);
	ieee80211_hw_set(hw, SIGNAL_DBM);
	ieee80211_hw_set(hw, SUPPORTS_PER_STA_GTK);
	ieee80211_hw_set(hw, SUPPORTS_AMSDU_IN_AMPDU);
	ieee80211_hw_set(hw, MFP_CAPABLE);
	ieee80211_hw_set(hw, PS_NULLFUNC_STACK);
	ieee80211_hw_set(hw, SUPPORTS_PS);
#else
	hw->flags |= IEEE80211_HW_AMPDU_AGGREGATION |
		IEEE80211_HW_HAS_RATE_CONTROL | IEEE80211_HW_SIGNAL_DBM;
	hw->flags |= IEEE80211_HW_MFP_CAPABLE;
	hw->flags |= IEEE80211_HW_PS_NULLFUNC_STACK;
	hw->flags |= IEEE80211_HW_SUPPORTS_PS;
#endif
#define MAC80211_TX_WMM_QUEUES 4
	hw->queues = MAC80211_TX_WMM_QUEUES;
	hw->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION)
#if defined(MAC80211_SUPPORT_MESH) && defined(CONFIG_MAC80211_MESH)
		| BIT(NL80211_IFTYPE_MESH_POINT)
#endif
#ifdef MAC80211_SUPPORT_UAP
		| BIT(NL80211_IFTYPE_AP)
#endif
		;
	hw->wiphy->flags |= WIPHY_FLAG_IBSS_RSN;

	handle->priv[bss_index] = priv;
	priv->phandle = handle;
	priv->bss_index = bss_index;
	priv->bss_type = bss_type;
	priv->bss_role = MLAN_BSS_ROLE_STA;

	if (MLAN_STATUS_SUCCESS !=
	    woal_request_get_fw_info(priv, MOAL_IOCTL_WAIT, &fw_info)) {
		PRINTM(MERROR, "woal_request_get_fw_info failed!\n");
		goto err_get_fw_info;
	}

	if (fw_info.fw_bands & (BAND_B | BAND_G | BAND_GN | BAND_GAC))
		hw->wiphy->bands[IEEE80211_BAND_2GHZ] = &mac80211_band_2ghz;
	if (fw_info.fw_bands & BAND_A)
		hw->wiphy->bands[IEEE80211_BAND_5GHZ] = &mac80211_band_5ghz;

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (!req) {
		PRINTM(MERROR, "woal_alloc_mlan_ioctl_req failed!\n");
		goto error;
	}

	/** Get supported MCS rate */
	cfg_11n = (mlan_ds_11n_cfg *)req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_SUPPORTED_MCS_SET;
	req->req_id = MLAN_IOCTL_11N_CFG;
	req->action = MLAN_ACT_GET;
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret != MLAN_STATUS_SUCCESS)
		goto error;

	if (hw->wiphy->bands[IEEE80211_BAND_2GHZ])
		woal_mac80211_setup_ht_cap(&hw->wiphy->
					   bands[IEEE80211_BAND_2GHZ]->ht_cap,
					   fw_info.hw_dot_11n_dev_cap,
					   cfg_11n->param.supported_mcs_set);
	if (hw->wiphy->bands[IEEE80211_BAND_5GHZ]) {
		woal_mac80211_setup_ht_cap(&hw->wiphy->
					   bands[IEEE80211_BAND_5GHZ]->ht_cap,
					   fw_info.hw_dot_11n_dev_cap,
					   cfg_11n->param.supported_mcs_set);
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
		woal_mac80211_setup_vht_cap(priv,
					    &hw->wiphy->
					    bands[IEEE80211_BAND_5GHZ]->
					    vht_cap);
#endif
	}
	/* ENABLE_802_11AC */
#if CFG80211_VERSION_CODE > KERNEL_VERSION(3, 0, 0)
	hw->wiphy->iface_combinations = &mac80211_iface_combination;
	hw->wiphy->n_iface_combinations = 1;
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	hw->wiphy->vendor_events = mac80211_vendor_events;
	hw->wiphy->n_vendor_events = ARRAY_SIZE(mac80211_vendor_events);
#endif

#ifdef CONFIG_PM
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
	hw->wiphy->wowlan = &wowlan_support;
#else
	hw->wiphy->wowlan.flags = WIPHY_WOWLAN_ANY | WIPHY_WOWLAN_MAGIC_PKT;
	hw->wiphy->wowlan.n_patterns = MAX_NUM_FILTERS;
	hw->wiphy->wowlan.pattern_min_len = 1;
	hw->wiphy->wowlan.pattern_max_len = WOWLAN_MAX_PATTERN_LEN;
	hw->wiphy->wowlan.max_pkt_offset = WOWLAN_MAX_OFFSET_LEN;
#endif
#endif
#endif
	hw->wiphy->addr_mask[ETH_ALEN - 1] =
		hw->wiphy->iface_combinations->max_interfaces;
	status = ieee80211_register_hw(hw);
	if (status != 0) {
		PRINTM(MERROR, "mac80211 registration failed, error: %d\n",
		       status);
		goto error;
	}

	priv->mac80211_registered = 1;

	INIT_LIST_HEAD(&priv->tx_stat_queue);
	spin_lock_init(&priv->tx_stat_lock);

	hw->sta_data_size = sizeof(struct mac80211_sta);
	INIT_LIST_HEAD(&priv->sta_list);
	spin_lock_init(&priv->sta_list_lock);

#ifdef MAC80211_SUPPORT
	priv->beacon_update_workqueue =
		alloc_workqueue("MOAL_BEACON_UPDATE_WORK_QUEUE",
				WQ_HIGHPRI | WQ_MEM_RECLAIM | WQ_UNBOUND, 1);
	if (!priv->beacon_update_workqueue) {
		PRINTM(MERROR,
		       "%s failed to allocate beacon update workqueue\n",
		       __func__);
		goto error;
	}
	MLAN_INIT_WORK(&priv->beacon_update_work, woal_beacon_update_work);
#endif
	handle->priv_num++;

	kfree(req);
	LEAVE();
	return priv;
error:
	kfree(req);
err_get_fw_info:
	handle->priv[bss_index] = NULL;
	if (hw)
		ieee80211_free_hw(hw);
err_alloc_hw:
	LEAVE();
	return NULL;
}

static void
woal_mac80211_send_tx_status(moal_private *priv, struct sk_buff *skb,
			     bool ack, mac80211_tx_status_event * tx_status)
{
	int is_ampdu;
	struct ieee80211_tx_info *info;
	if (!skb)
		return;

	info = IEEE80211_SKB_CB(skb);
	is_ampdu = info->flags & IEEE80211_TX_CTL_AMPDU;
	ieee80211_tx_info_clear_status(info);

	if (!tx_status) {
		info->status.rates[0].idx = -1;
		info->status.rates[0].count = -1;
	} else {
		woal_mac80211_convert_nxp_rateid_to_ieee_rate(tx_status->
							      final_rate_id,
							      info);
		info->status.rates[0].count = tx_status->retry_count;
	}

	if (ack) {
		info->flags |= IEEE80211_TX_STAT_ACK;
		if (is_ampdu) {
			info->flags |= IEEE80211_TX_STAT_AMPDU;
			info->status.ampdu_ack_len = info->status.ampdu_len = 1;
		}
	} else {
		if (is_ampdu) {
			info->flags |= IEEE80211_TX_STAT_AMPDU;
			info->status.ampdu_ack_len = 0;
			info->status.ampdu_len = 1;
		}
	}

	/* ACK the frame to mac80211, which will free the skb */
	ieee80211_tx_status(priv->hw, skb);
}

/**
 *  @brief This function flush tx status queue for MAC80211 interface
 *
 *  @param priv      A pointer to moal_private structure
 *
 *  @return          N/A
 */
static void
woal_flush_mac80211_tx_stat_queue(moal_private *priv)
{
	struct tx_status_info *tx_info = NULL, *tmp_node;
	unsigned long flags;

	spin_lock_irqsave(&priv->tx_stat_lock, flags);
	list_for_each_entry_safe(tx_info, tmp_node, &priv->tx_stat_queue, link) {
		list_del(&tx_info->link);
		woal_mac80211_send_tx_status(priv, tx_info->tx_skb, 0, NULL);
		kfree(tx_info);
	}
	INIT_LIST_HEAD(&priv->tx_stat_queue);
	spin_unlock_irqrestore(&priv->tx_stat_lock, flags);
}

void
woal_remove_mac80211_interface(moal_private *priv)
{
	if (priv->hw) {
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
		if (priv->mac80211_mode_monitor) {
			woal_set_net_monitor(priv, MOAL_IOCTL_WAIT, MFALSE, 0,
					     NULL);
			priv->mac80211_mode_monitor = MFALSE;
		}
#endif
		ieee80211_stop_queues(priv->hw);
		woal_flush_mac80211_tx_stat_queue(priv);
		if (priv->mac80211_registered)
			ieee80211_unregister_hw(priv->hw);
		priv->mac80211_registered = 0;
		priv->phandle->priv[priv->bss_index] = NULL;
		ieee80211_free_hw(priv->hw);
#ifdef MAC80211_SUPPORT
		if (priv->beacon_update_workqueue) {
			flush_workqueue(priv->beacon_update_workqueue);
			destroy_workqueue(priv->beacon_update_workqueue);
			priv->beacon_update_workqueue = NULL;
		}
#endif
	}
}

/**
 *  @brief This function handles MAC80211 events from mlan
 *
 *  @param priv Pointer to the moal_private structure
 *  @param pmevent  Pointer to the mlan event structure
 *
 *  @return         MLAN_STATUS_SUCCESS
 */
mlan_status
moal_recv_event_mac80211(moal_private *priv, pmlan_event pmevent)
{
	ENTER();

	if (priv == NULL) {
		PRINTM(MERROR, "%s: priv is null\n", __func__);
		goto done;
	}

	switch (pmevent->event_id) {
	case MLAN_EVENT_ID_DRV_DEFER_RX_WORK:
		queue_work(priv->phandle->rx_workqueue,
			   &priv->phandle->rx_work);
		break;
	case MLAN_EVENT_ID_DRV_DEFER_HANDLING:
		queue_work(priv->phandle->workqueue, &priv->phandle->main_work);
		break;
	case MLAN_EVENT_ID_DRV_DBG_DUMP:
		priv->phandle->driver_status = MTRUE;
		woal_moal_debug_info(priv, NULL, MFALSE);
		woal_process_hang(priv->phandle);
		break;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	case MLAN_EVENT_ID_FW_DEBUG_INFO:
		woal_mac80211_debug_event(priv, pmevent->event_buf,
					  pmevent->event_len);
		break;
#endif
	case MLAN_EVENT_ID_FW_TX_STATUS:{
			unsigned long flag;
			mac80211_tx_status_event *tx_status =
				(mac80211_tx_status_event *) (pmevent->
							      event_buf + 4);
			struct tx_status_info *tx_info = NULL;

			PRINTM(MINFO,
			       "Receive Tx status: tx_token=%d, pkt_type=0x%x, status=%d tx_seq_num=%d\n",
			       tx_status->tx_token_id, tx_status->packet_type,
			       tx_status->status, priv->tx_seq_num);

			spin_lock_irqsave(&priv->tx_stat_lock, flag);
			tx_info =
				woal_get_tx_info(priv, tx_status->tx_token_id);
			if (tx_info) {
				bool ack;
				list_del(&tx_info->link);
				spin_unlock_irqrestore(&priv->tx_stat_lock,
						       flag);

				if (!tx_status->status)
					ack = true;
				else
					ack = false;
				PRINTM(MEVENT, "Wlan: Tx status=%d\n", ack);
				woal_mac80211_send_tx_status(priv,
							     tx_info->tx_skb,
							     ack, tx_status);
				kfree(tx_info);
			} else {
				spin_unlock_irqrestore(&priv->tx_stat_lock,
						       flag);
				PRINTM(MERROR, "%s: tx_info is missing!\n",
				       __func__);
			}
		}
		break;
	default:
		PRINTM(MEVENT, "%s:ignore events for mac80211\n", __func__);
		break;
	}
done:
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function is called when MLAN complete send data packet for
 * MAC80211 interface
 *
 *  @param pmoal Pointer to the MOAL context
 *  @param pmbuf    Pointer to the mlan buffer structure
 *  @param status   The status code for mlan_send_packet request
 *
 *  @return         MLAN_STATUS_SUCCESS
 */
mlan_status
moal_send_packet_complete_mac80211(t_void *pmoal,
				   pmlan_buffer pmbuf, mlan_status status)
{
	moal_handle *handle = (moal_handle *)pmoal;
	t_u16 queue;

	ENTER();

	if (pmbuf && pmbuf->pdesc) {
		moal_private *priv =
			woal_bss_index_to_priv(handle, pmbuf->bss_index);

		atomic_dec(&handle->tx_pending);
		queue = skb_get_queue_mapping((struct sk_buff *)pmbuf->pdesc);
		if (atomic_dec_return(&priv->wmm_tx_pending[queue]) ==
		    MAC80211_MIN_TX_PENDING) {
			if (ieee80211_queue_stopped(priv->hw, queue))
				ieee80211_wake_queue(priv->hw, queue);
		}

		if (!(pmbuf->flags & MLAN_BUF_FLAG_TX_STATUS))
			dev_kfree_skb_any((struct sk_buff *)pmbuf->pdesc);
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function uploads the packet to the network stack
 *
 *  @param pmoal Pointer to the MOAL context
 *  @param pmbuf    Pointer to the mlan buffer structure
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
moal_recv_packet_mac80211(t_void *pmoal, pmlan_buffer pmbuf)
{
	mlan_status status = MLAN_STATUS_SUCCESS;
	moal_private *priv = NULL;
	struct sk_buff *skb = NULL;
	moal_handle *handle = (moal_handle *)pmoal;
	struct ieee80211_rx_status rx_stats;
	struct ieee80211_hdr *hdr;

	ENTER();
	if (pmbuf) {
		priv = woal_bss_index_to_priv(pmoal, pmbuf->bss_index);
		if (!priv) {
			PRINTM(MERROR, "%s: drop RX packet, priv in NULL!\n",
			       __func__);
			status = MLAN_STATUS_FAILURE;
			goto done;
		}
		if (priv->mac80211_started) {
			skb = (struct sk_buff *)pmbuf->pdesc;
			if (skb) {
				skb_reserve(skb, pmbuf->data_offset);
				skb_put(skb, pmbuf->data_len);
				pmbuf->pdesc = NULL;
				pmbuf->pbuf = NULL;
				pmbuf->data_offset = pmbuf->data_len = 0;
				/* pkt been submit to kernel, no need to free by
				 * mlan*/
				status = MLAN_STATUS_PENDING;
				atomic_dec(&handle->mbufalloc_count);
			} else {
				PRINTM(MERROR, "%s without skb attach!!!\n",
				       __func__);
				skb = dev_alloc_skb(pmbuf->data_len +
						    MLAN_NET_IP_ALIGN);
				if (!skb) {
					PRINTM(MERROR, "%s fail to alloc skb\n",
					       __func__);
					status = MLAN_STATUS_FAILURE;
					priv->stats.rx_dropped++;
					goto done;
				}
				skb_reserve(skb, MLAN_NET_IP_ALIGN);
				moal_memcpy_ext(priv->phandle, skb->data,
						(t_u8 *)(pmbuf->pbuf +
							 pmbuf->data_offset),
						pmbuf->data_len,
						pmbuf->data_len);
				skb_put(skb, pmbuf->data_len);
			}

			memset(&rx_stats, 0, sizeof(rx_stats));
			rx_stats.band = priv->hw->conf.chandef.chan->band;
			rx_stats.freq =
				priv->hw->conf.chandef.chan->center_freq;

			hdr = (struct ieee80211_hdr *)skb->data;
			if (ieee80211_is_mgmt(hdr->frame_control)) {
#define PACKET_ADDR4_POS (2 + 2 + 6 + 6 + 6 + 2)
				/** Remove Add4 from mgmt frames */
				memmove(skb->data + ETH_ALEN, skb->data,
					PACKET_ADDR4_POS);
				skb_pull(skb, ETH_ALEN);
			}
			if (ieee80211_has_protected(hdr->frame_control)) {
				/* Frames are already decrypted in firmware, if
				 * frames are protected in the 802.11 header
				 * strip the flag else mac80211 will drop the
				 * frame
				 */
				hdr->frame_control &=
					~__cpu_to_le16
					(IEEE80211_FCTL_PROTECTED);
				rx_stats.flag |=
					RX_FLAG_DECRYPTED | RX_FLAG_IV_STRIPPED
					| RX_FLAG_MMIC_STRIPPED;
			}
			moal_memcpy_ext(priv->phandle, IEEE80211_SKB_RXCB(skb),
					&rx_stats, sizeof(rx_stats),
					sizeof(rx_stats));
			ieee80211_rx(priv->hw, skb);
			goto done;
		} else {
			PRINTM(MDATA, "%s: drop RX packet, priv not started!\n",
			       __func__);
			status = MLAN_STATUS_FAILURE;
			goto done;
		}
	}
done:
	LEAVE();
	return status;
}

#endif
