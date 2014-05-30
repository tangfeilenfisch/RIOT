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
 */

#ifndef RIOT_BLE_L2CAP_H
#define RIOT_BLE_L2CAP_H

#include <stdint.h>
#include <stdbool.h>

#define L2CAP_FHEADER_SIZE		4

#define L2CAP_CHID_ATT			0x0004
#define L2CAP_CHID_LE_SIGNALING		0x0005
#define L2CAP_CHID_SMP			0x0006
#define L2CAP_CHID_6LOWPAN        	0x003e


struct l2cap_fheader {
	uint16_t f_length;
	uint16_t ch_id;
} __attribute__((packed));

#endif
