/*
  MIT License

  Copyright (c) 2018 Antonio Alexander Brewer (tonton81) - https://github.com/tonton81

  Designed and tested for PJRC Teensy 4.0.

  Forum link : https://forum.pjrc.com/threads/56035-FlexCAN_T4-FlexCAN-for-Teensy-4?highlight=flexcan_t4

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


#include <isotp_server.h>
#include "Arduino.h"


int isotp_server_Base::buffer_hosts = 0;


ISOTPSERVER_FUNC ISOTPSERVER_OPT::isotp_server() {
  _ISOTPSERVER_OBJ[isotp_server_Base::buffer_hosts] = this;
  isotp_server_Base::buffer_hosts++; 
  for ( int i = 3; i > -1; i-- ) {
    if ( ((request >> (i * 8)) & 0xFF) ) break;
    request_size--;
  }
  header[0] = (1U << 4) | len >> 8;
  header[1] = (uint8_t)len;
}


ISOTPSERVER_FUNC void ISOTPSERVER_OPT::send_first_frame() {
  index_sequence = 1;
  CAN_message_t msg;
  msg.id = canid;
  msg.flags.extended = extended;
  if ( len <= 7 ) {
    memset(&msg.buf[0], padding_value, 8);
    msg.buf[0] = len;
    memmove(&msg.buf[1], &buffer[0], len);
    if ( _isotp_server_busToWrite ) _isotp_server_busToWrite->write(msg);
    return;
  }
  msg.len = 8;
  memmove(&msg.buf[0], &header[0], 2);
  memmove(&msg.buf[2], &buffer[0], 6);
  if ( _isotp_server_busToWrite ) _isotp_server_busToWrite->write(msg);
}


ISOTPSERVER_FUNC bool ISOTPSERVER_OPT::send_next_frame() {
  if ( index_pos >= len ) return 0;
  CAN_message_t msg;
  msg.id = canid;
  msg.flags.extended = extended;
  msg.len = 8;
  msg.buf[0] = (2U << 4) | (index_sequence & 0xF);
  memmove(&msg.buf[1], &buffer[index_pos], constrain((len - index_pos), 1, 7));
  index_sequence++;
  if ((len - index_pos) < 7) for ( int i = (len - index_pos + 1); i < 8; i++ ) msg.buf[i] = 0xA5; 
  if ( _isotp_server_busToWrite ) _isotp_server_busToWrite->write(msg);
  index_pos += 7;
  return 1;
}


ISOTPSERVER_FUNC void ISOTPSERVER_OPT::_process_frame_data(const CAN_message_t &msg) {
  if ( !isotp_enabled ) return;
  
  #if defined(TEENSYDUINO)
    if ( msg.bus != readBus ) return;
  #endif

  if ( msg.id == canid ) {
    CAN_message_t msgCopy = msg;
    uint8_t request_array[4] = { (uint8_t)(request >> 24), (uint8_t)(request >> 16), (uint8_t)(request >> 8), (uint8_t)(request) };
    memmove(&msgCopy.buf[0], &request_array[4 - request_size], request_size);
    bool request_match = true;
    for ( int i = 0; i < 4; i++ ) {
      if ( msg.buf[i] != msgCopy.buf[i] ) {
        request_match = false;
        break;
      }
    }
    if ( request_match ) {
      send_first_frame();
      index_pos = 6;
      return;
    }
    if ( msg.buf[0] & (3U << 4) ) { /* flow control frame */
      if ( !msg.buf[1] ) { /* block size */
        bool sent = 1;
        while ( sent ) {
          sent = send_next_frame();
          if ( msg.buf[2] < 128 ) delay(msg.buf[2]);
          else if ( msg.buf[2] == constrain(msg.buf[2], 0xF1, 0xF9) ) {
            delayMicroseconds(map(msg.buf[2], 0xF1, 0xF9, 100, 900));
          }
        }
      }
      else {
        for ( int i = 0; i < msg.buf[1]; i++ ) {
          send_next_frame();
          if ( msg.buf[2] < 128 ) delay(msg.buf[2]);
          else if ( msg.buf[2] == constrain(msg.buf[2], 0xF1, 0xF9) ) {
            delayMicroseconds(map(msg.buf[2], 0xF1, 0xF9, 100, 900));
          }
        }
      }
    }
  }
}


void ext_output3(const CAN_message_t &msg) {
  for ( int i = 0; i < isotp_server_Base::buffer_hosts; i++ ) if ( _ISOTPSERVER_OBJ[i] ) _ISOTPSERVER_OBJ[i]->_process_frame_data(msg);
}











