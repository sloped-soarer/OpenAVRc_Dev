 /*
 **************************************************************************
 *                                                                        *
 *                 ____                ___ _   _____                      *
 *                / __ \___  ___ ___  / _ | | / / _ \____                 *
 *               / /_/ / _ \/ -_) _ \/ __ | |/ / , _/ __/                 *
 *               \____/ .__/\__/_//_/_/ |_|___/_/|_|\__/                  *
 *                   /_/                                                  *
 *                                                                        *
 *              This file is part of the OpenAVRc project.                *
 *                                                                        *
 *                         Based on code(s) named :                       *
 *             OpenTx - https://github.com/opentx/opentx                  *
 *             Deviation - https://www.deviationtx.com/                   *
 *                                                                        *
 *                Only AVR code here for visibility ;-)                   *
 *                                                                        *
 *   OpenAVRc is free software: you can redistribute it and/or modify     *
 *   it under the terms of the GNU General Public License as published by *
 *   the Free Software Foundation, either version 2 of the License, or    *
 *   (at your option) any later version.                                  *
 *                                                                        *
 *   OpenAVRc is distributed in the hope that it will be useful,          *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of       *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the        *
 *   GNU General Public License for more details.                         *
 *                                                                        *
 *       License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html          *
 *                                                                        *
 **************************************************************************
*/


#ifndef misc_h
#define misc_h

#define RX_TX_ADDR_OFFSET       128
#define RF_ID_ADDR(x)           packet[(x)+RX_TX_ADDR_OFFSET]
#define CHANNEL_USED_OFFSET     50
#define CHANNEL_USED(x)         packet[(x)+RX_TX_ADDR_OFFSET]

static uint8_t rfState;
static uint8_t channel_index;
static uint8_t channel_offset;
static uint16_t packet_period;

extern void PROTO_Start_Callback(uint16_t us, uint16_t (*cb)());
extern void PROTO_Stop_Callback();
extern uint32_t CLOCK_getms();
extern void CLOCK_delayms(uint32_t delay_ms);
extern void PROTOCOL_SetBindState(tmr10ms_t t10ms);

#endif
