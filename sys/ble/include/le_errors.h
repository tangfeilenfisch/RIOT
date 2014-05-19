/*
 * Copyright (C) 2014 Johann Fischer <j.fischer_at_fh-bingen_de>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 *
 * HCI command errors
 *
 */

#ifndef RIOT_BLECI_ERRORS_H
#define RIOT_BLECI_ERRORS_H

#define  UNK_CMD_ERR				0x01
#define  UNK_CON_ID_ERR				0x02
#define  HARD_FAIL_ERR				0x03
#define  PAGE_TIMEOUT_ERR			0x04
#define  AUT_FAIL_ERR				0x05
#define  PIN_KEY_MISSING_ERR			0x06
#define  MEM_CAP_EXC_ERR			0x07
#define  CON_TIMEOUT_ERR			0x08
#define  CON_LIMIT_EXC_ERR			0x09
#define  SYNC_CON_LD_EXC_ERR			0x0A
#define  ACL_CON_EXISTS_ERR			0x0B
#define  CMD_DISALLOWED_ERR			0x0C
#define  CON_REJ_LR_ERR				0x0D
#define  CON_REJ_SR_ERR				0x0E
#define  CON_REJ_BD_ADDR_ERR			0x0F
#define  CON_AT_EXC_ERR				0x10
#define  UNSUPP_VALUE_ERR			0x11
#define  INVALID_CMD_PARAM_ERR			0x12
#define  RUSER_TERM_CON_ERR			0x13
#define  RD_TERM_CON_LR_ERR			0x14
#define  RD_TERM_CON_POFF_ERR			0x15
#define  CON_TERM_BY_LHOST_ERR			0x16
#define  REPEATED_ATTEMPTS_ERR			0x17
#define  PAIRING_NALLOWED_ERR			0x18
#define  UNK_LMP_PDU_ERR			0x19
#define  UNSUPP_RFEATURE_ERR			0x1A
#define  SCO_OFFSET_REJ_ERR			0x1B
#define  SCO_INTERVAL_REJ_ERR			0x1C
#define  SCO_AIR_MODE_REJ_ERR			0x1D
#define  INVALID_LL_PARAM_ERR			0x1E
#define  UNSPECIFIED_ERROR_ERR			0x1F
#define  UNSUPP_LL_VALUE_ERR			0x20
#define  ROLE_CH_NALLOWED_ERR			0x21
#define  LL_RES_TIMEOUT_ERR			0x22
#define  LMP_ERROR_TC_ERR			0x23
#define  LMP_PDU_NALLOWED_ERR			0x24
#define  ENC_MODE_NACCEPT_ERR			0x25
#define  LK_CANNOT_CH_ERR			0x26
#define  REQ_QOS_UNSUPP_ERR			0x27
#define  INSTANT_PASSED_ERR			0x28
#define  PAIRING_WUK_UNSUPP_ERR			0x29
#define  DIFF_TRANSAC_COL_ERR			0x2A
#define  RESERVED1_ERR				0x2B
#define  QOS_UP_ERR				0x2C
#define  QOS_REJ_ERR				0x2D
#define  CCNS_ERR				0x2E
#define  INSUFF_SEC_ERR				0x2F
#define  PARAM_OUT_RANGE_ERR			0x30
#define  RESERVED2_ERR				0x31
#define  RSWITCH_PENDING_ERR			0x32
#define  RESERVED3_ERR				0x33
#define  RESERVED_SVIOLATION_ERR		0x34
#define  ROLE_SWITCH_FAILED_ERR			0x35
#define  EI_RES_TOO_LARGE_ERR			0x36
#define  SECURE_SP_UNSUPP_ERR			0x37
#define  HOST_BUSY_ERR				0x38
#define  CON_REJ_NC_FOUND_ERR			0x39
#define  CONTROLLER_BUSY_ERR			0x3A
#define  UNACCEPT_CON_PARAM_ERR			0x3B
#define  DIR_ADV_TIMEOUT_ERR			0x3C
#define  CON_TERM_MIC_FAIL_ERR			0x3D
#define  CON_FAILED_ERR				0x3E
#define  MAC_CON_FAILED_ERR			0x3F
#define  CCARBWTTAUCD_ERR			0x40

#endif
