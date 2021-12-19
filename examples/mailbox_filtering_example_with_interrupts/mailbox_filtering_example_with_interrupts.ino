#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

#define NUM_TX_MAILBOXES 2
#define NUM_RX_MAILBOXES 6
void setup(void) {
  Serial.begin(115200); delay(400);
  Can0.begin();
  Can0.setBaudRate(250000);
  Can0.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  for (int i = 0; i<NUM_RX_MAILBOXES; i++){
    Can0.setMB((FLEXCAN_MAILBOX)i,RX,EXT);
  }
  for (int i = NUM_RX_MAILBOXES; i<(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++){
    Can0.setMB((FLEXCAN_MAILBOX)i,TX,EXT);
  }
  Can0.setMBFilter(REJECT_ALL);
  Can0.enableMBInterrupts();
  Can0.onReceive(MB0,canSniff);
  Can0.onReceive(MB1,canSniff);
  Can0.onReceive(MB2,canSniff);
  Can0.setMBUserFilter(MB0,0x00,0xFF);
  Can0.setMBUserFilter(MB1,0x03,0xFF);
  Can0.setMBUserFilter(MB2,0x0B,0xFF);
  Can0.mailboxStatus();
}

void canSniff(const CAN_message_t &msg) {
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}

void loop() {
  Can0.events();
}