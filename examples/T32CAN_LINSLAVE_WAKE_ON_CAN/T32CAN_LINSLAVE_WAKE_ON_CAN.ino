#include <FlexCAN_T4.h>
FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> Can0;
volatile uint32_t can_activity = 0;
volatile uint32_t ign_output = 0;

void setup() {
  Serial.begin(115200);
  pinMode(23, OUTPUT); // ENABLE LIN SLAVE OUTPUT
  delay(1000);

  Can0.begin();
  Can0.setBaudRate(125000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(canSniff);
  Can0.mailboxStatus();
}

void loop() {
  if ( ign_output ) {
    if ( millis() - can_activity > 10000 ) {
      ign_output = 0;
      Serial.println("BUS INACTIVE, IGN OUTPUT DISABLED");
    }
  }
  else digitalWrite(23, HIGH); // DISABLE IGN OUTPUT
}

void canSniff(const CAN_message_t &msg) {
  can_activity = millis();
  if ( ign_output == 0 ) Serial.println("BUS ACTIVE, IGN OUTPUT ENABLED");
  ign_output = 1;
  digitalWrite(23, LOW); // ENABLE IGN OUTPUT
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
