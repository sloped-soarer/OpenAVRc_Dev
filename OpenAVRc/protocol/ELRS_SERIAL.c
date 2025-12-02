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
// Original file from https://github.com/phobos-/deviation/blob/elrs/src/protocol/crsf_uart.c
// For protocol information see https://github.com/crsf-wg/crsf/wiki
#include "../OpenAVRc.h"

#define CRSF_RX_STATE           pulses2MHz.crsf_st.crsf_rx_state
#define CRSF_FREQ_RATE_MEM      pulses2MHz.crsf_st.crsf_freq_rate
#define CRSF_40_MS_Flag         pulses2MHz.crsf_st.crsf_40ms_flag
#define CRSF_40_MS_Flipflop     pulses2MHz.crsf_st.crsf_40ms_flipflop
#define CRSF_RATE_PERIOD        pulses2MHz.crsf_st.crsf_rate_period
#define CRSF_TIMING_CORRECT     pulses2MHz.crsf_st.crsf_timing_correct
#define CRSF_TIMING_OFFSET      pulses2MHz.crsf_st.crsf_timing_offset
#define CRSF_MODULE_RESPONDING  pulses2MHz.crsf_st.crsf_module_responding

// Device addresses
#define CRSF_ADDR_BROADCAST  0x00  //  Broadcast address
#define CRSF_ADDR_USB        0x10  //  USB Device
#define CRSF_ADDR_BLUETOOTH  0x12  //  Bluetooth Module
#define CRSF_ADDR_PRO_CORE   0x80  //  TBS CORE PNP PRO
//  #define CRSF_ADDR_  0x8A       //  Reserved
#define CRSF_ADDR_PRO_CURR   0xC0  //  PNP PRO digital current sensor
#define CRSF_ADDR_PRO_GPS    0xC2  //  PNP PRO GPS
#define CRSF_ADDR_BLACKBOX   0xC4  //  TBS Blackbox
#define CRSF_ADDR_FC         0xC8  //  Flight controller
//  #define CRSF_ADDR_       0xCA  //  Reserved
#define CRSF_ADDR_RACETAG    0xCC  //  Race tag
#define CRSF_ADDR_RADIO      0xEA  //  Radio Transmitter
//  #define CRSF_ADDR_       0xEB  //  Reserved
#define CRSF_ADDR_RECEIVER   0xEC  //  Crossfire / UHF receiver
#define CRSF_ADDR_MODULE     0xEE  //  Crossfire transmitter

#define CRSF_DATARATE             115200
#define CRSF_FRAME_PERIOD         4000   // 4ms
#define CRSF_CHANNELS             16

#define CRSF_SET_PACKET_SIZE      8

// ELRS command
#define ELRS_ADDRESS               0xEE
#define ELRS_BIND_COMMAND          0xFF
#define ELRS_WIFI_COMMAND          0xFE
#define ELRS_PKT_RATE_COMMAND      0x01
#define ELRS_TLM_RATIO_COMMAND     0x02
#define ELRS_POWER_COMMAND         0x03

const static RfOptionSettingsvar_t RfOpt_CRSF_Ser[] PROGMEM =
{
  /*rfProtoNeed*/0, //can be PROTO_NEED_SPI | BOOL1USED | BOOL2USED | BOOL3USED
  /*rfSubTypeMax*/0,
  /*rfOptionValue1Min*/0,
  /*rfOptionValue1Max*/0x54,
  /*rfOptionValue2Min*/0,
  /*rfOptionValue2Max*/0,
  /*rfOptionValue3Max*/0,
};

const pm_char STR_CRSF_FREQ[] PROGMEM = "915AU""915FC""868EU""433AU""433EU""24ISM";
const uint8_t CRSF_RATE24[] PROGMEM =
{ 0xFF,250,150,50,25}; // 0xFF mean 500
const uint8_t CRSF_RATE900[] PROGMEM =
{ 200,100,50,25};
const uint8_t CRSF_POWER[] PROGMEM =
{ 10,25,50,100,250,0xFD,0xFE,0xFF}; // 0xFD mean 500 0xFE->1000 0xFF->2000
const uint8_t CRSF_TLMRATE[] PROGMEM =
{ 128,64,32,16,8,4,2};

#define READ_CRSF_FREQ      (g_model.rfOptionValue1>>4)
#define WRITE_CRSF_FREQ(x)  (g_model.rfOptionValue1 = (g_model.rfOptionValue1 & 0x0F) | (x<<4))
#define READ_CRSF_RATE      (g_model.rfOptionValue1&0x0F)
#define WRITE_CRSF_RATE(x)  (g_model.rfOptionValue1 = (g_model.rfOptionValue1 & 0xF0) | x)
#define IS_CRSF_24_FREQ     (READ_CRSF_FREQ == 5)
#define GET_CRSF_NUM_RATE   (IS_CRSF_24_FREQ ? sizeof(CRSF_RATE24) : sizeof(CRSF_RATE900))

static void CRSF_Reset()
{
  USART_DISABLE_TX(CRSF_USART);
  USART_DISABLE_RX(CRSF_USART);
#if defined(CPUXMEGA)
  S0_USART_TXD_PIN_CTRL_REG &= ~PORT_INVEN_bm;
  S0_USART_RXD_PIN_CTRL_REG &= ~PORT_INVEN_bm;
  CRSF_USART_PORT.PIN3CTRL = PORT_OPC_PULLUP_gc; // Pull up TXD.
  CRSF_USART_PORT.PIN2CTRL = PORT_OPC_PULLUP_gc; // Pull up RXD.
#endif
  parseTelemFunction = (p_parseTelemFunction) parseTelemFrskyByte;
}

// CRC8 implementation with polynom = x^8+x^7+x^6+x^4+x^2+1 (0xD5)
static const uint8_t ZZcrsf_crc8tab[] PROGMEM =
{
  0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
  0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
  0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
  0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
  0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
  0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
  0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
  0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
  0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
  0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
  0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
  0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
  0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
  0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
  0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
  0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9,
};

// CRC8 implementation with polynom = 0xBA
static const uint8_t ZZcrsf_crc8_BAtab[] PROGMEM =
{
  0x00, 0xBA, 0xCE, 0x74, 0x26, 0x9C, 0xE8, 0x52, 0x4C, 0xF6, 0x82, 0x38, 0x6A, 0xD0, 0xA4, 0x1E,
  0x98, 0x22, 0x56, 0xEC, 0xBE, 0x04, 0x70, 0xCA, 0xD4, 0x6E, 0x1A, 0xA0, 0xF2, 0x48, 0x3C, 0x86,
  0x8A, 0x30, 0x44, 0xFE, 0xAC, 0x16, 0x62, 0xD8, 0xC6, 0x7C, 0x08, 0xB2, 0xE0, 0x5A, 0x2E, 0x94,
  0x12, 0xA8, 0xDC, 0x66, 0x34, 0x8E, 0xFA, 0x40, 0x5E, 0xE4, 0x90, 0x2A, 0x78, 0xC2, 0xB6, 0x0C,
  0xAE, 0x14, 0x60, 0xDA, 0x88, 0x32, 0x46, 0xFC, 0xE2, 0x58, 0x2C, 0x96, 0xC4, 0x7E, 0x0A, 0xB0,
  0x36, 0x8C, 0xF8, 0x42, 0x10, 0xAA, 0xDE, 0x64, 0x7A, 0xC0, 0xB4, 0x0E, 0x5C, 0xE6, 0x92, 0x28,
  0x24, 0x9E, 0xEA, 0x50, 0x02, 0xB8, 0xCC, 0x76, 0x68, 0xD2, 0xA6, 0x1C, 0x4E, 0xF4, 0x80, 0x3A,
  0xBC, 0x06, 0x72, 0xC8, 0x9A, 0x20, 0x54, 0xEE, 0xF0, 0x4A, 0x3E, 0x84, 0xD6, 0x6C, 0x18, 0xA2,
  0xE6, 0x5C, 0x28, 0x92, 0xC0, 0x7A, 0x0E, 0xB4, 0xAA, 0x10, 0x64, 0xDE, 0x8C, 0x36, 0x42, 0xF8,
  0x7E, 0xC4, 0xB0, 0x0A, 0x58, 0xE2, 0x96, 0x2C, 0x32, 0x88, 0xFC, 0x46, 0x14, 0xAE, 0xDA, 0x60,
  0x6C, 0xD6, 0xA2, 0x18, 0x4A, 0xF0, 0x84, 0x3E, 0x20, 0x9A, 0xEE, 0x54, 0x06, 0xBC, 0xC8, 0x72,
  0xF4, 0x4E, 0x3A, 0x80, 0xD2, 0x68, 0x1C, 0xA6, 0xB8, 0x02, 0x76, 0xCC, 0x9E, 0x24, 0x50, 0xEA,
  0x48, 0xF2, 0x86, 0x3C, 0x6E, 0xD4, 0xA0, 0x1A, 0x04, 0xBE, 0xCA, 0x70, 0x22, 0x98, 0xEC, 0x56,
  0xD0, 0x6A, 0x1E, 0xA4, 0xF6, 0x4C, 0x38, 0x82, 0x9C, 0x26, 0x52, 0xE8, 0xBA, 0x00, 0x74, 0xCE,
  0xC2, 0x78, 0x0C, 0xB6, 0xE4, 0x5E, 0x2A, 0x90, 0x8E, 0x34, 0x40, 0xFA, 0xA8, 0x12, 0x66, 0xDC,
  0x5A, 0xE0, 0x94, 0x2E, 0x7C, 0xC6, 0xB2, 0x08, 0x16, 0xAC, 0xD8, 0x62, 0x30, 0x8A, 0xFE, 0x44
};

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len)
{
// crc implementation from CRSF protocol document rev7
  uint8_t crc = 0;
  uint_farptr_t crctab = pgm_get_far_address(ZZcrsf_crc8tab);
  for (uint8_t i = 0; i < len; i++)
  {
    crc = pgm_read_byte_far(crctab + (crc ^ *ptr--));
  }
  return crc;
}

uint8_t crsf_crc8_BA(const uint8_t *ptr, uint8_t len)
{
// crc implementation from CRSF protocol document rev7
  uint8_t crc = 0;
  uint_farptr_t crctab = pgm_get_far_address(ZZcrsf_crc8_BAtab);
  for (uint8_t i = 0; i < len; i++)
  {
    crc = pgm_read_byte_far(crctab + (crc ^ *ptr--));
  }
  return crc;
}

/*
 The CRSF channel values (0-1984).
 CRSF 172 represents 988us,
 CRSF 992 represents 1500us and
 CRSF 1811 represents 2012us.

 TICKS_TO_US(x) ((x - 992) * 5 / 8 + 1500)
 US_TO_TICKS(x) ((x - 1500) * 8 / 5 + 992)
 */

#define CSRF_MIN 172
#define CSRF_MAX 1811

enum CRSFRXSTATE
{
  IGNORE,
  LISTEN,
  RADIO_ADDRS_FOUND,
  LEN_FOUND,
};

#define l_buffer  pulses2MHz.crsf_st.crsf_rx_buffer

NOINLINE extern void parseCrossfireByte(uint8_t data)
{

  static uint8_t write_ptr;
  static uint8_t length;

  PORTB.OUTTGL = PIN6_bm;

  switch (CRSF_RX_STATE)
  {
    case IGNORE:
      (void) data;
      break;

    case LISTEN:

      write_ptr = 0;
      length = 0;
      if (data == CRSF_ADDR_RADIO) CRSF_RX_STATE = RADIO_ADDRS_FOUND;
      else CRSF_RX_STATE = IGNORE;
      break;

    case RADIO_ADDRS_FOUND:
      if (data == 0x0D)
      {
        CRSF_RX_STATE = LEN_FOUND;
        length = 0x0d;
      }
      else CRSF_RX_STATE = IGNORE;
      break;

    case LEN_FOUND:
      // store packet minus header.
      if (write_ptr < length)
      {
        l_buffer[write_ptr] = data;
        write_ptr++;
      }

      if (write_ptr == length)
      {

// decode the frame
        if (l_buffer[0] == CRSF_FRAMETYPE_RADIO_ID &&
        l_buffer[1] == CRSF_ADDR_RADIO &&
        l_buffer[2] == CRSF_ADDR_MODULE &&
        l_buffer[3] == 0x10 &&
        l_buffer[4] == 0x00 &&
        l_buffer[5] == 0x03 &&
        l_buffer[6] == 0x0d &&
        l_buffer[7] == 0x40) ;
        {
// crc check
          if (l_buffer[length -1] == crsf_crc8(l_buffer[0], length -2));
          // todo

          CRSF_TIMING_CORRECT = 0;
          CRSF_TIMING_CORRECT |= l_buffer[8];
          CRSF_TIMING_CORRECT <<= 8;
          CRSF_TIMING_CORRECT |= l_buffer[9];
          CRSF_TIMING_CORRECT <<= 8;
          CRSF_TIMING_CORRECT |= l_buffer[10];
          CRSF_TIMING_CORRECT <<= 8;
          CRSF_TIMING_CORRECT |= l_buffer[11];

          if (CRSF_TIMING_CORRECT > 0) CRSF_TIMING_OFFSET += 5;
          if (CRSF_TIMING_CORRECT < 0) CRSF_TIMING_OFFSET -= 5;
          if (CRSF_TIMING_OFFSET > +30000) CRSF_TIMING_OFFSET = 30000;
          if (CRSF_TIMING_OFFSET < -30000) CRSF_TIMING_OFFSET = -30000;

          CRSF_RX_STATE = IGNORE;
          s_anaFilt[AREFA] = CRSF_TIMING_CORRECT;
        }
      }
      break;
  }
}

static void build_CRSF_data_pkt()
{
#if defined(X_ANY)
 Xany_scheduleTx_AllInstance();
#endif

  Usart0TxBufferCount = CRSF_CHAN_PACKET_SIZE;
  uint8_t crsfTxBufferCount = Usart0TxBufferCount;

  Usart0TxBuffer_p2M[--crsfTxBufferCount] = CRSF_ADDR_MODULE;
  Usart0TxBuffer_p2M[--crsfTxBufferCount] = CRSF_CHAN_PACKET_SIZE - 2; // length of type + payload + crc
  Usart0TxBuffer_p2M[--crsfTxBufferCount] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;

  int16_t value;
  uint32_t bits = 0;
  uint8_t bitsavailable = 0;

  for (uint8_t i = 0; i < CRSF_CHANNELS; i++)
  {
    if (i < 16/*todoModel.num_channels*/)
    {
//     value = calcRESXto1000(FULL_CHANNEL_OUTPUTS(i))*8/10;
//     value += 992; // Add midpoint value
      value = 800;
    }
    else value = 992;  // midpoint

    bits |= (uint32_t) value << bitsavailable;
    bitsavailable += 11; // 11 bits per channel
    while (bitsavailable >= 8)
    {
      Usart0TxBuffer_p2M[--crsfTxBufferCount] = ((uint8_t)(bits & 0xff));
      bits >>= 8;
      bitsavailable -= 8;
    }
  }

  Usart0TxBuffer_p2M[0] = crsf_crc8(
      &Usart0TxBuffer_p2M[CRSF_CHAN_PACKET_SIZE - 3],
      CRSF_CHAN_PACKET_SIZE - 3);
}

#if 0
static void buildElrsPacket(uint8_t command, uint8_t value)
{
 Usart0TxBufferCount = CRSF_SET_PACKET_SIZE;
 uint8_t crsfTxBufferCount = Usart0TxBufferCount;

 Usart0TxBuffer_p2M[--crsfTxBufferCount] = CRSF_ADDR_MODULE;
 Usart0TxBuffer_p2M[--crsfTxBufferCount] = CRSF_SET_PACKET_SIZE-2;
 Usart0TxBuffer_p2M[--crsfTxBufferCount] = CRSF_FRAMETYPE_PARAMETER_WRITE;
 Usart0TxBuffer_p2M[--crsfTxBufferCount] = ELRS_ADDRESS;
 Usart0TxBuffer_p2M[--crsfTxBufferCount] = CRSF_ADDR_RADIO;
 Usart0TxBuffer_p2M[--crsfTxBufferCount] = command;
 Usart0TxBuffer_p2M[--crsfTxBufferCount] = value;
 Usart0TxBuffer_p2M[--crsfTxBufferCount] = crsf_crc8(&Usart0TxBuffer_p2M[CRSF_SET_PACKET_SIZE-3], CRSF_SET_PACKET_SIZE-3);
}
#endif

uint16_t convertPktRateToPeriod()
{
  uint8_t rate = READ_CRSF_RATE;
  uint16_t freq =
      IS_CRSF_24_FREQ ?
          pgm_read_byte_near(&CRSF_RATE24[rate]) :
          pgm_read_byte_near(&CRSF_RATE900[rate]);
  if (freq == 0xFF) freq = 500;
  freq = 1000 / freq;
  return freq;
}

static uint8_t convertPktRateToElrs(uint8_t rfFreqRate)
{
  uint8_t rate = (rfFreqRate & 0x0F);
  rfFreqRate >>= 4; // keep rfFreq value

  switch (rate)
  // rate
  {
    case 0:
      if (rfFreqRate == 5) return 0;
      return 2;
    case 1:
      if (rfFreqRate == 5) return 1;
      return 4;
    case 2:
      if (rfFreqRate == 5) return 3;
      return 5;
    case 3:
      if (rfFreqRate == 5) return 5;
      return 6;
    case 4:
      return 6;
  }
  return 6;
}


void build_Model_Id_frame(void)
{
#define MODEL_ID_FRAME_LEN 10
  Usart0TxBufferCount = MODEL_ID_FRAME_LEN;
  uint8_t crsfTxBufferCount = Usart0TxBufferCount;

  Usart0TxBuffer_p2M[--crsfTxBufferCount] = CRSF_UART_SYNC;
  Usart0TxBuffer_p2M[--crsfTxBufferCount] = MODEL_ID_FRAME_LEN - 2;
  Usart0TxBuffer_p2M[--crsfTxBufferCount] = CRSF_FRAMETYPE_COMMAND;
  Usart0TxBuffer_p2M[--crsfTxBufferCount] = CRSF_ADDR_MODULE;
  Usart0TxBuffer_p2M[--crsfTxBufferCount] = CRSF_ADDR_RADIO;
  Usart0TxBuffer_p2M[--crsfTxBufferCount] = CRSF_SUBCOMMAND;
  Usart0TxBuffer_p2M[--crsfTxBufferCount] = CRSF_COMMAND_MODEL_SELECT_ID;
  Usart0TxBuffer_p2M[--crsfTxBufferCount] = g_model.modelId;
  Usart0TxBuffer_p2M[--crsfTxBufferCount] = crsf_crc8_BA(
      &Usart0TxBuffer_p2M[MODEL_ID_FRAME_LEN - 3], MODEL_ID_FRAME_LEN - 4);
  Usart0TxBuffer_p2M[--crsfTxBufferCount] = crsf_crc8(
      &Usart0TxBuffer_p2M[MODEL_ID_FRAME_LEN - 3], MODEL_ID_FRAME_LEN - 3);
}

static uint8_t check_CRSF_ParamChange()
{
  if (g_model.rfOptionValue1 != CRSF_FREQ_RATE_MEM) // freq and/or rate change ?
  {
    uint8_t rate = convertPktRateToElrs(g_model.rfOptionValue1);
    //buildElrsPacket(ELRS_PKT_RATE_COMMAND, rate);
    CRSF_FREQ_RATE_MEM = g_model.rfOptionValue1;
    uint16_t period = convertPktRateToPeriod();
    if (period == 40)
    {
      CRSF_40_MS_Flag = 1;
      period /= 2; // use CRSF_40_MS_Flipflop
    }
    else
    {
      CRSF_40_MS_Flag = 0;
    }
    CRSF_RATE_PERIOD = period * 1000U;
    SCHEDULE_MIXER_END_IN_US(CRSF_RATE_PERIOD); // Schedule new next Mixer calculations.
    return 1;
  }
  return 0;
}

static uint16_t CRSF_SERIAL_cb()
{
  SCHEDULE_MIXER_END_IN_US(CRSF_RATE_PERIOD); // Schedule next Mixer calculations.
// if (!(CRSF_40_MS_Flag && (CRSF_40_MS_Flipflop ^= 0x1)))
// {
//  if (!check_CRSF_ParamChange())
  build_CRSF_data_pkt();

#if !defined(SIMU)
  USART_DISABLE_RX(CRSF_USART);
  RF_PORT.DIRSET = USART_TXD_PIN_bm;
  USART_TRANSMIT_BUFFER(CRSF_USART);
#endif
  heartbeat |= HEART_TIMER_PULSES;
//  }

  CALCULATE_LAT_JIT(); // Calculate latency and jitter.
  return ((20000U * 2) - (CRSF_TIMING_OFFSET / 5));
}

static void CRSF_initialize(uint8_t bind)
{
// 115K2 8N1
  parseTelemFunction = (p_parseTelemFunction) parseCrossfireByte;
  USART_SET_BAUD_115K2(CRSF_USART);
  USART_SET_MODE_8N1(CRSF_USART);
  USART_ENABLE_TX(CRSF_USART);
  USART_DISABLE_RX(CRSF_USART);
#if defined(CPUXMEGA)
  CRSF_USART_PORT.PIN3CTRL = PORT_OPC_PULLDOWN_gc; // Pulldown TXD.
  CRSF_USART_PORT.PIN2CTRL = PORT_OPC_PULLDOWN_gc;// Pulldown RXD.
  S0_USART_TXD_PIN_CTRL_REG |= PORT_INVEN_bm;// Invert TXD pin.
  S0_USART_RXD_PIN_CTRL_REG |= PORT_INVEN_bm;// Invert RXD pin.
  CRSF_USART_PORT.DIRCLR = USART_RXD_PIN_bm;// RXD pin as input.
#endif
  CRSF_MODULE_RESPONDING = 0;
  CRSF_RX_STATE = IGNORE;
  Usart0TxBufferCount = 0;
  CRSF_RATE_PERIOD = convertPktRateToPeriod() * 1000U;
  CRSF_TIMING_OFFSET = 0;
  PROTO_Start_Callback(CRSF_SERIAL_cb);
}

const void* CRSF_Cmds(enum ProtoCmds cmd)
{
  switch (cmd)
  {
    case PROTOCMD_INIT:
      CRSF_initialize(0);
      return 0;
    case PROTOCMD_RESET:
      PROTO_Stop_Callback();
      CRSF_Reset();
      return 0;
    case PROTOCMD_BIND:
      CRSF_initialize(1);
      return 0;
    case PROTOCMD_GETOPTIONS:
      SetRfOptionSettings(pgm_get_far_address(RfOpt_CRSF_Ser), STR_DUMMY, //Sub proto
          STR_DUMMY,      //Option 1 (int)
          STR_DUMMY,      //Option 2 (int)
          STR_DUMMY,      //Option 3 (uint 0 to 31)
          STR_DUMMY,      //OptionBool 1
          STR_DUMMY,      //OptionBool 2
          STR_DUMMY       //OptionBool 3
          );
      return 0;
    default:
      break;
  }
  return 0;
}

