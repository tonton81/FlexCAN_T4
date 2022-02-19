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

#if !defined(_ISOTP_SERVER_H_)
#define _ISOTP_SERVER_H_

#include "Arduino.h"
#include "circular_buffer.h"
#include "isotp_server.h"
#include "FlexCAN_T4.h"

typedef enum ISOTP_ID_TYPE {
  STANDARD_ID = 0,
  EXTENDED_ID = 1,
} ISOTP_ID_TYPE;

#define ISOTPSERVER_CLASS template<uint32_t canid, ISOTP_ID_TYPE extended, uint32_t request, uint8_t *buffer, uint16_t len>
#define ISOTPSERVER_FUNC template<uint32_t canid, ISOTP_ID_TYPE extended, uint32_t request, uint8_t *buffer, uint16_t len>
#define ISOTPSERVER_OPT isotp_server<canid, extended, request, buffer, len>


class isotp_server_Base {
  public:
    virtual void _process_frame_data(const CAN_message_t &msg) = 0;
    static int buffer_hosts;
    FlexCAN_T4_Base* _isotp_server_busToWrite = nullptr;
};

static isotp_server_Base* _ISOTPSERVER_OBJ[16] = { nullptr };

ISOTPSERVER_CLASS class isotp_server : public isotp_server_Base {
  public:
    isotp_server();
    void begin() { enable(); }
    void enable(bool yes = 1) { isotp_enabled = yes; }
    void setWriteBus(FlexCAN_T4_Base* _busWritePtr) { 
       _isotp_server_busToWrite = _busWritePtr; 
      #if defined(__IMXRT1062__)
        if ( _isotp_server_busToWrite == _CAN1 ) readBus = 1;    
        if ( _isotp_server_busToWrite == _CAN2 ) readBus = 2;
        if ( _isotp_server_busToWrite == _CAN3 ) readBus = 3;
      #endif
      #if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
        if ( _isotp_server_busToWrite == _CAN0 ) readBus = 0;    
        if ( _isotp_server_busToWrite == _CAN1 ) readBus = 1;
      #endif
    }   
    void setPadding(uint8_t _byte) { padding_value = _byte; }

  private:
    void _process_frame_data(const CAN_message_t &msg);
    void send_first_frame();
    bool send_next_frame();
    uint8_t header[2] = { 0 };
    volatile uint8_t request_size = 4;
    volatile uint16_t index_pos = 0;
    volatile uint8_t index_sequence = 1;
    volatile bool isotp_enabled = 0;
    uint8_t padding_value = 0xA5;
    uint8_t readBus = 1;
};


#include "isotp_server.tpp"
#endif