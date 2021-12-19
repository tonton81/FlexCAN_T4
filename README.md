# FlexCAN_T4
FlexCAN Library for Teensy 4

https://forum.pjrc.com/threads/56035-FlexCAN_T4-FlexCAN-for-Teensy-4

Designed for Teensy 4.0, compatibility for Teensy 3.x.
Based on a redesigned model of IFCT from the ground up.

Your sketch should include the header to use it: 

`#include <FlexCAN_T4.h>`

There are 2 types of template constructors available. CAN2.0 and CANFD use a different constructor.


To use CAN2.0 mode, use:
```
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> myCan;
```
Where on Teensy4, CAN1,CAN2, and CAN3 are available.
On Teensy 3.x, CAN0 is available on all, but CAN1 exists only on Teensy 3.6.
The name of myCan can be anything of your choice.

To use CANFD mode, use:
```
FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> myFD;
```

Where, on Teensy 4.0, only CAN3 supports FD. Teensy3.x does not have an FD controller at all.
Again, the name of myFD can be anything of your choice.

Before using any of the commands of the library, myCan.begin() must absolutely be called, at a bare minimum to set up the clock and controller. Skipping this first step will hard fault teensy if the registers are not configured.

There are 2 different configurations for baudrates between CAN2.0 and CANFD.
To set baudrate in CAN2.0 mode, you need to call:
```
myCan.setBaudRate(1000000);
```
Where 1000000 is the bitrate of your choice.

To set baudrate in CANFD mode, you need to call:

```
CANFD_timings_t config;
config.clock = CLK_24MHz;
config.baudrate = 1000000;
config.baudrateFD = 2000000;
config.propdelay = 190;
config.bus_length = 1;
config.sample = 70;
FD.setBaudRate(config);
```

There are 2 different message structures for CAN2.0 and CANFD.

CANFD: `CANFD_message_t`

CAN2.0: `CAN_message_t`

Both are very similar, except CANFD structure has a 64 byte payload, edl, and brs switching.

brs by default is set to 1, unless changed in the message structure. This bit, when set, allows the higher bitrate for data to be used in CANFD mode, otherwise nominal rate is used.

edl by default is set to 1, unless changed in the message structure. This bit, when set, allows CANFD frames to be sent, otherwise, sends as a CAN2.0 frame, with a truncated payload to max 8 bytes.

Callbacks for interrupts can be used, both for individual mailboxes, or for all mailboxes.

```
myCan.onReceive(MB0, canSniff); // allows mailbox 0 messages to be received in the supplied callback.
myCan.onReceive(MB1, canSniff); // allows mailbox 1 messages to be received in the supplied callback.
myCan.onReceive(canSniff); // allows all FIFO/message box messages to be received in the supplied callback.
myCan.onReceive(FIFO, canSniff); // allows FIFO messages to be received in the supplied callback.
```

Note that there is no FIFO support in CANFD for Teensy 4.0. FIFO is only supported in CAN2.0 mode on Teensy 3.x and Teensy 4.0

To enable FIFO support in CAN2.0 mode, simply run myCAN.enableFIFO();

Each mailbox can be interrupt enabled or all at once. This allows frames to be interrupt driven rather than polling them in the loop.
```
myCan.enableMBInterrupts(); // enables all mailboxes to be interrupt enabled
myCan.enableMBInterrupt(MB4); // enables mailbox 4 to be interrupt enabled
myCan.enableMBInterrupt(FIFO); // enables FIFO to be interrupt enabled
```

A general callback printout for CAN2.0 and CANFD is the following (remove the FD from CANFD for the message structure if using CAN2.0 mode):
```
void canSniff(const CANFD_message_t &msg) {
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
```

To get an idea of a default initialization of the mailboxes, you may run:
```
myCan.mailboxStatus();
```
It will show you how many standard, extended, transmit mailboxes you have, and if fifo is enabled or not.

There is a complex filtering system in the FIFO region, which is not very user friendly. If you have FIFO enabled, running mailboxStatus will show you how many filters that you can set for it.
The library fully automates the masking capability in both hardware and software, taking the struggle away from the users. Usually, when you are setting filters, your first step would be to block all data first before setting the filters.
To do this, you would need to use:

For FIFO: `myCan.setFIFOFilter(REJECT_ALL);`

For mailboxes: `myCan.setMBFilter(REJECT_ALL);`

Once this is done, you may configure your filters to allow a specific ID, a group (up to 5 IDs), or even a range of IDs.
```
myCan.setFIFOFilter(0, 0x123, STD); // Set filter0 to allow STANDARD CAN ID 0x123 to be collected by FIFO. 
myCan.setFIFOFilter(1, 0x456, EXT); // Set filter1 to allow EXTENDED CAN ID 0x456 to be collected by FIFO. 
myCan.setMBFilter(MB6, 0x123); // Set mailbox 6 to allow CAN ID 0x123 to be collected. 
```
For the mailbox, there is no extended or standard selection. Those bits are set on the mailbox and referred to when applying the filter. To see what type of mailbox you have, `run mailboxStatus();`. 
```
myCan.setMBFilterRange(MB7, 0x1, 0x9); // Mailbox will attept to receive only frames 0x1 to 0x5.
```
(0x1,0x2,0x3,0x4,0x5, and other frames may bleed-thru, see enhanceFilter for sub-filtering)

### Enhanced filtering
Enhanced filtering is a feature of the library that can stop bleed-thru frames from a multi-ID filter to store messages of IDs that were not requested to the queue.
Lets say for example you want a mailbox to store IDs 0x1 and 0x5:
```
myCan.setMBFilter(MB6, 0x1, 0x5);
```
You will notice that not only frames 0x1 and 0x5 are received, but also 0x3 and 0x7. This is because the 2 out of 3 bits are different between both IDs.
Once you enable:
```
myCan.enhanceFilter(MB6);
```
The library will toss out those frames and keep only the ones you requested. So while the hardware does most of the filtering, the library is the final filtering system which allows what you actually wanted to be saved.
This can be applied to all FIFO filters as well.
```
myCan.enhanceFilter(FIFO);
```

Distribution, on all controllers, the first mailbox/FIFO to receive a frame is a first come, first serve, and if you have other mailboxes capturing the same type of frame, especially with individual callbacks, not all will fire, but only one. Distribution overcomes this limitation by copying the frame to the queue if it matches another mailbox filter, allowing all similar filters to receive the same message.
```
myCan.distribute(); // Enable distribution
```

The mailbox type can be changed to extended, standard, or transmit, using the setMB() function.
```
myCan.setMB(MB9,TX); // Set mailbox as transmit
myCan.setMB(MB10,RX,EXT); // Set mailbox as receiving extended frames.
myCan.setMB(MB11,RX,STD); // Set mailbox as receiving standard frames.
```

Events must be used in the loop(), IntervalTimer, or teensyThreads, for the callback system to push received interrupt frames from the queue to the callback. Sequential frames are pushed out from there as well, we'll talk about that soon.
```
myCan.events();
```

Sequential frames, if you want to send ordered frames out without bus arbitration making them go out in any order, the library supports sending your frames out from a queue to the absolute first transmit mailbox only. To enable sequential support, set your message structure to msg.seq = 1, then write it as normal, the library will only queue it for next cycle if the mailbox isn't available.

The recommended pins on the teensy cards for CAN are used by default. If you wish to switch to alternate pins, you can use `myCan.setRX(ALT);` or `myCAN.setTX(ALT)` as needed.

Polling frames is possible and won't touch interrupt driven mailboxes. 
You may use your correct message structure (CAN_message_t or CANFD_message_t), and simply call:
`myCan.read(myFrame);`
If FIFO is enabled, each read() call will read from FIFO or Mailbox randomly, provided they are not interrupt driven.

There are 2 functions to write messages to the bus:
```
myCan.write(myFrame); // Write to any available transmit mailbox, Note, sequential frames must use this function only.
myCan.write(MB15, myFrame); // Write to mailbox 15 (provided it's a transmit mailbox)
```
FIFO does NOT transmit, only mailboxes do.

In the background, the library has 3 weak functions used for interrupt driven frames, that can be used by other libraries to gather data for their own functionality.
An example of this is TeensyCAN, which uses one of the 3 weak functions, leaving 2 available for other libraries, if needed.

```
extern void ext_outputFD1(const CANFD_message_t &msg); // Interrupt data output, not filtered, for external libraries, FD
extern void ext_outputFD2(const CANFD_message_t &msg);
extern void ext_outputFD3(const CANFD_message_t &msg); // TeensyCAN uses this one.
extern void ext_output1(const CAN_message_t &msg); // Interrupt data output, not filtered, for external libraries, CAN2.0
extern void ext_output2(const CAN_message_t &msg);
extern void ext_output3(const CAN_message_t &msg); // TeensyCAN uses this one.
```

On Teensy 4, you have actually 64 mailboxes in CAN2.0 mode, but in CANFD mode, depending on the data size, can be much less. When setting the region support of the mailbox for FD mode (setRegion(x)), the function returns the count of mailboxes available to the user.
```
myFD.setRegion(8) // default, returns a value of 64 (mailboxes), each one supporting 8 bytes payload
myFD.setRegion(64) // returns a value of 14 (mailboxes), each one supporting 64 bytes payload
```
In CAN2.0 mode, setRegion doesn't exist, you have 64 mailboxes on Teensy 4.0, and 16 on Teensy 3.x.
