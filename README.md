# FlexCAN_T4
FlexCAN Library for Teensy 4

https://forum.pjrc.com/threads/56035-FlexCAN_T4-FlexCAN-for-Teensy-4


Currently in early stages, designed for Teensy 4.
Based on a redesigned model of IFCT from the ground up.


Currently working:

1) The new interface allows a user to have a sketch based user assigned object to the library. Instead of the default Can0 and Can1 like in IFCT or FlexCAN_Library, the user can call his object of personal choice. The interface allows the user to also set the RX and TX queue system from within the sketch rather than from the source. The template constructor is:

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

CAN2 is the actual controlled on Teensy 4.0 being tested on pins 0 and 1. Can0 is the name the user gave to the object. The 2 others are buffer sizes for the bus interface to use.

2) Currently, the speed of the flexcan controller is set to maximum, which is 60Mhz, as opposed to the SDK which runs at 20 or 24 Mhz. It will probably have a user switchable function in the future to switch between speeds.
EDIT: getClock and setClock is added, the library now defaults to 24MHz oscillator, 60MHz can be activated in sketch during runtime.

3) 64 mailbox support

4) FIFO support, with up to 128 filters, and calculated offsets. FIFO tables A and B are only supported. C and D will not be used due to their excessive frame bleed-thru. You don't need that many partial masks in a table C or D if the bus is designed for ~60 nodes...

5) Transmit queue support. Sequential frames are targeted to a single mailbox while the rest of the frames can be written at any available transmit mailbox.

6) Receive queue support. Loop based, future will add threads/intervaltimer, should either be available.

7) Automatic Filtering system. This supports all 'up to' 128 filters of FIFO, and 64 mailboxes.

8) Currently running on Teensy Beta1 board using pins 0 and 1.

9) Up to 64 callbacks can be used. If FIFO is used, MB0 callback will be assigned and fired for reception, as FIFO occupies the first mailbox onwards.

10) CAN1 is now supported, on pins 22 and 23.
