/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "FlexCAN_T4.h"

/* Loopback test on Teensy 3.6 can0 transmitting to can1 */

FlexCAN_T4<CAN0, RX_SIZE_16, TX_SIZE_256> can0;
FlexCAN_T4<CAN1, RX_SIZE_16, TX_SIZE_256> can1;

CAN_message_t tx_msg, rx_msg;

void irq(const CAN_message_t &ref) {
  Serial.print("Received ID: ");
  Serial.println(ref.id);
}

int main() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("STARTING TEST");
  /* Enable the CAN transceivers */
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  digitalWriteFast(26, LOW);
  digitalWriteFast(27, LOW);
  /* Start the CAN bus and set the baud */
  can0.begin();
  can0.setBaudRate(1000000);
  can1.begin();
  can1.setBaudRate(1000000);
  can1.setRFFN(RFFN_32); // 32 filters
  can1.enableFIFO();  // enable FIFO
  can1.setMRP(0);  // prioritize FIFO
  can1.enableFIFOInterrupt();
  can1.onReceive(FIFO, irq);
  /* Set some random filters */
  can1.setFIFOFilter(REJECT_ALL);
  can1.setFIFOFilter(0, 1, STD);
  can1.setFIFOFilter(1, 2, STD);
  can1.setFIFOFilter(2, 3, STD);
  can1.setFIFOFilter(3, 4, STD);
  can1.setFIFOFilter(4, 5, STD);
  /* Transmit data sequentially */
  for (int i = 0; i < 24; i++) {
    tx_msg.id = i;
    tx_msg.seq = 1;
    can0.write(tx_msg);
  }
  while (1) {}
}
