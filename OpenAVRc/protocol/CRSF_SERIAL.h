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

#ifndef CRSF_SERIAL_H
#define CRSF_SERIAL_H

#define CRSF_CHAN_PACKET_SIZE  26
#define CRSF_MAX_FRAME_SIZE    64
#define TELEMETRY_RX_PACKET_SIZE   64

typedef struct
{
  uint8_t crsf_tx_buffer[CRSF_MAX_FRAME_SIZE]; // Reserved aka Usart0TxBuffer_p2M
volatile  uint8_t crsf_rx_state;
  uint8_t crsf_freq_rate;
  uint8_t crsf_40ms_flag;
  uint8_t crsf_40ms_flipflop;
  uint16_t crsf_rate_period;
  uint8_t crsf_rx_buffer[15];
  int32_t crsf_timing_correct;
  int16_t crsf_timing_offset;
  uint8_t crsf_module_responding;
} crsfSt_t;


// Frame Type
#define CRSF_FRAMETYPE_GPS                 0x02
#define CRSF_FRAMETYPE_VARIO               0x07
#define CRSF_FRAMETYPE_BATTERY_SENSOR      0x08
#define CRSF_FRAMETYPE_BARO_ALTITUDE       0x09
#define CRSF_FRAMETYPE_HEARTBEAT           0x0B
//#define XF_TYPE_VTX                      0x0F
//#define XF_TYPE_VTX_TELEM                0x10
#define CRSF_FRAMETYPE_LINK_STATISTICS     0x14
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED  0x16
#define CRSF_FRAMETYPE_LINK_RX_ID          0x1C
#define CRSF_FRAMETYPE_LINK_TX_ID          0x1D
#define CRSF_FRAMETYPE_ATTITUDE            0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE         0x21

// CRSF_FRAMETYPE 0x28 and higher use Extended Packet Format.
#define CRSF_FRAMETYPE_DEVICE_PING         0x28
#define CRSF_FRAMETYPE_DEVICE_INFO         0x29
//#define XF_TYPE_REQUEST_SETTINGS         0x2A
#define CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY  0x2B
#define CRSF_FRAMETYPE_PARAMETER_READ            0x2C
#define CRSF_FRAMETYPE_PARAMETER_WRITE           0x2D
#define CRSF_FRAMETYPE_ELRS_STATUS               0x2E
#define CRSF_FRAMETYPE_COMMAND                   0x32
#define CRSF_FRAMETYPE_RADIO_ID                  0x3A // Sync mixer with OTA timing aka CRSFShot.
#define CRSF_FRAMETYPE_MSP_REQ                   0x7A
#define CRSF_FRAMETYPE_DISPLAYPORT_CMD           0x7D
// Frame Subtype
#define CRSF_UART_SYNC                      0xC8
#define CRSF_SUBCOMMAND                     0x10
#define CRSF_COMMAND_MODEL_SELECT_ID        0x05

#endif // CRSF_SERIAL_H
