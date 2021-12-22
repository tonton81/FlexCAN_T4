/*
  MIT License

  Copyright (c) 2018 Antonio Alexander Brewer (tonton81) - https://github.com/tonton81

  Designed and tested for PJRC Teensy 4.0.

  Forum link : https://forum.pjrc.com/threads/56035-isotp-FlexCAN-for-Teensy-4?highlight=isotp

  Thanks goes to skpang, mjs513, and collin for tech/testing support

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#if !defined(_ISOTP_H_)
#define _ISOTP_H_

#include "Arduino.h"
#include "circular_buffer.h"
#include "isotp.h"

#if defined(TEENSYDUINO) // Teensy
#include "FlexCAN_T4.h"
#elif defined(ARDUINO_ARCH_ESP32) //ESP32
#include "ESP32_CAN.h"
#endif

typedef struct ISOTP_data {
  uint32_t id = 0;                         /* can identifier */
  struct {
    bool extended = 0;                     /* identifier is extended (29-bit) */
    bool usePadding = 0;                   /* padd and use all 8 bytes instead of truncating len */
    bool separation_uS = 0;                /* separation time in uS (100-900uS only) */
  } flags;
  uint16_t len = 8;                        /* length of CAN message or callback payload */
  uint16_t blockSize = 0;                  /* used for flow control, specify how many frame blocks per frame control request */
  uint8_t flow_control_type = 0;           /* flow control type: 0: Clear to Send, 1: Wait, 2: Abort */
  uint16_t separation_time = 0;            /* time between frames */
} ISOTP_data;

typedef enum ISOTP_RXBANKS_TABLE {
  RX_BANKS_2 = (uint16_t)2,
  RX_BANKS_4 = (uint16_t)4,
  RX_BANKS_8 = (uint16_t)8,
  RX_BANKS_16 = (uint16_t)16,
  RX_BANKS_32 = (uint16_t)32,
  RX_BANKS_64 = (uint16_t)64,
  RX_BANKS_128 = (uint16_t)128,
  RX_BANKS_256 = (uint16_t)256,
  RX_BANKS_512 = (uint16_t)512,
  RX_BANKS_1024 = (uint16_t)1024
} ISOTP_RXBANKS_TABLE;

#define ISOTP_CLASS template<ISOTP_RXBANKS_TABLE _rxBanks = RX_BANKS_16, size_t _max_length = 32>
#define ISOTP_FUNC template<ISOTP_RXBANKS_TABLE _rxBanks, size_t _max_length>
#define ISOTP_OPT isotp<_rxBanks, _max_length>

typedef void (*_isotp_cb_ptr)(const ISOTP_data &config, const uint8_t *buf);

#if defined(TEENSYDUINO) // Teensy
static FlexCAN_T4_Base* _isotp_busToWrite = nullptr;
#elif defined(ARDUINO_ARCH_ESP32) //ESP32
static ESP32_CAN_Base* _isotp_busToWrite = nullptr;
#endif

class isotp_Base {
  public:
    virtual void _process_frame_data(const CAN_message_t &msg) = 0;
    virtual void write(const ISOTP_data &config, const uint8_t *buf, uint16_t size) = 0;
    _isotp_cb_ptr _isotp_handler = nullptr;
};

static isotp_Base* _ISOTP_OBJ = nullptr;

ISOTP_CLASS class isotp : public isotp_Base {
  public:
    isotp() { _ISOTP_OBJ = this; }

#if defined(TEENSYDUINO) // Teensy
    void setWriteBus(FlexCAN_T4_Base* _busWritePtr) { 
      _isotp_busToWrite = _busWritePtr; 
      #if defined(__IMXRT1062__)
        if ( _isotp_busToWrite == _CAN1 ) readBus = 1;    
        if ( _isotp_busToWrite == _CAN2 ) readBus = 2;
        if ( _isotp_busToWrite == _CAN3 ) readBus = 3;
      #endif
      #if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
        if ( _isotp_busToWrite == _CAN0 ) readBus = 0;    
        if ( _isotp_busToWrite == _CAN1 ) readBus = 1;
      #endif
    }
#elif defined(ARDUINO_ARCH_ESP32) //ESP32
    void setWriteBus(ESP32_CAN_Base* _busWritePtr) { _isotp_busToWrite = _busWritePtr; }
#endif
    void begin() { enable(); }
    void enable(bool yes = 1) { isotp_enabled = yes; }
    void setPadding(uint8_t _byte) { padding_value = _byte; }
    void onReceive(_isotp_cb_ptr handler) { _ISOTP_OBJ->_isotp_handler = handler; }
    void write(const ISOTP_data &config, const uint8_t *buf, uint16_t size);
    void write(const ISOTP_data &config, const char *buf, uint16_t size) { write(config, (const uint8_t*)buf, size); }
    void sendFlowControl(const ISOTP_data &config);

  private:
    void _process_frame_data(const CAN_message_t &msg);
    Circular_Buffer<uint8_t, _rxBanks, _max_length> _rx_slots;
    uint8_t padding_value = 0xA5;
    volatile bool isotp_enabled = 0;
    uint8_t readBus = 1;
};

#include "isotp.tpp"
#endif