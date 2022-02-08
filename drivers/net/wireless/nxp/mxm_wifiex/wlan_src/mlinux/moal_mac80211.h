
/** @file moal_mac80211.h
 *
 * @brief This file contains mac80211 based driver specific defines etc.
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

/********************************************************
Change log:
    02/02/2021: initial version
********************************************************/

#ifndef _MOAL_MAC80211_H
#define _MOAL_MAC80211_H
enum { RATEID_DBPSK1Mbps,
	RATEID_OFDM72Mbps = 13,

	RATEID_MCS0_6d5Mbps = 14,
	RATEID_MCS7_65Mbps = 21,

	RATEID_MCS8_13Mbps = 22,
	RATEID_MCS15_130Mbps = 29,

	RATEID_MCS0BW40_13d5Mbps = 31,
	RATEID_MCS7BW40_135Mbps = 38,

	RATEID_MCS8BW40_27Mbps,
	RATEID_MCS15BW40_270Mbps = 46,

	RATEID_VHT_MCS0_1SS_BW20,
	RATEID_VHT_MCS9_1SS_BW20 = 56,

	RATEID_VHT_MCS0_2SS_BW20,
	RATEID_VHT_MCS9_2SS_BW20 = 66,

	RATEID_VHT_MCS0_1SS_BW40,
	RATEID_VHT_MCS9_1SS_BW40 = 76,

	RATEID_VHT_MCS0_2SS_BW40,
	RATEID_VHT_MCS9_2SS_BW40 = 86,

	RATEID_VHT_MCS0_1SS_BW80,
	RATEID_VHT_MCS9_1SS_BW80 = 96,

	RATEID_VHT_MCS0_2SS_BW80,
	RATEID_VHT_MCS9_2SS_BW80 = 106,
};

struct nxp_rate_group {
	u8 min_rate;
	u8 max_rate;
	u16 flags;
	u8 nss;
};

#define MAX_NXP_RATE_GROUPS 9
#endif /* _MOAL_MAC80211_H */
