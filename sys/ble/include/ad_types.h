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
 */

#ifndef RIOT_BLE_AD_TYPES_H
#define RIOT_BLE_AD_TYPES_H

#define AD_TYPE_FLAGS				0x01
#define FLAG_LE_LIMDISCMODE			(1 << 0)
#define FLAG_LE_GENDISCMODE			(1 << 1)
#define FLAG_BREDR_NSUPP			(1 << 2)
#define FLAG_SIMLE_BREDR_CNTR			(1 << 3)
#define FLAG_SIMLE_BREDR_HOST			(1 << 4)

#define AD_TYPE_IL_16UUID			0x02
#define AD_TYPE_CL_16UUID			0x03
#define AD_TYPE_IL_32UUID			0x04
#define AD_TYPE_CL_32UUID			0x05
#define AD_TYPE_IL_128UUID			0x06
#define AD_TYPE_CL_128UUID			0x07
#define AD_TYPE_SL_NAME				0x08
#define AD_TYPE_CL_NAME				0x09
#define AD_TYPE_TX_PWR_LEVEL			0x0A
#define AD_TYPE_DEV_CLASS			0x0D
#define AD_TYPE_SP_HASH_C			0x0E
#define AD_TYPE_SP_RND_R			0x0F
#define AD_TYPE_DEVICE_ID			0x10
#define AD_TYPE_SM_TK_VALUE			0x10
#define AD_TYPE_SM_OOB_FLAGS			0x11
#define AD_TYPE_SCON_IRANGE			0x12
#define AD_TYPE_SSL_16UUID			0x14
#define AD_TYPE_SSL_32UUID			0x1F
#define AD_TYPE_SSL_128UUID			0x15
#define AD_TYPE_SDATA				0x16
#define AD_TYPE_SDATA_16UUID			0x16
#define AD_TYPE_SDATA_32UUID			0x20
#define AD_TYPE_SDATA_128UUID			0x21
#define AD_TYPE_PUB_TADDRESS			0x17
#define AD_TYPE_RND_TADDRESS			0x18
#define AD_TYPE_APPEARANCE			0x19
#define AD_TYPE_ADV_INTERVAL			0x1A
#define AD_TYPE_LE_BT_DADDRESS			0x1B

#define AD_TYPE_LE_ROLE				0x1C
#define LE_ROLE_P				(1 << 0)
#define LE_ROLE_C				(1 << 1)
#define LE_ROLE_PC				(1 << 2)
#define LE_ROLE_CP				(1 << 3)

#define AD_TYPE_SP_HASH_C256			0x1D
#define AD_TYPE_SP_RND_R256			0x1E
#define AD_TYPE_3D_INFO_DATA			0x3D
#define AD_TYPE_MSPEC_DATA			0xFF

#endif
