#if !defined(_FLEXCAN_T4_H_)
#define _FLEXCAN_T4_H_

#include "Arduino.h"
#include "circular_buffer.h"
#include "imxrt_flexcan.h"

typedef struct CAN_message_t {
  uint32_t id = 0;          // can identifier
  uint16_t timestamp = 0;   // FlexCAN time when message arrived
  struct {
    bool extended = 0; // identifier is extended (29-bit)
    bool remote = 0;  // remote transmission request packet type
    bool overrun = 0; // message overrun
    bool reserved = 0;
  } flags;
  uint8_t len = 8;      // length of data
  uint8_t buf[8] = { 0 };       // data
  uint8_t mb = 0;       // used to identify mailbox reception
  uint8_t bus = 0;      // used to identify where the message came from when events() is used.
  bool seq = 0;         // sequential frames
} CAN_message_t;

typedef void (*_MB_ptr)(const CAN_message_t &msg); /* mailbox / global callbacks */

typedef enum FLEXCAN_PINS {
  ALT = 0,
  DEF = 1,
} FLEXCAN_PINS;

typedef enum FLEXCAN_MAILBOX {
  MB0 = 0,
  MB1 = 1,
  MB2 = 2,
  MB3 = 3,
  MB4 = 4,
  MB5 = 5,
  MB6 = 6,
  MB7 = 7,
  MB8 = 8,
  MB9 = 9,
  MB10 = 10,
  MB11 = 11,
  MB12 = 12,
  MB13 = 13,
  MB14 = 14,
  MB15 = 15,
  MB16 = 16,
  MB17 = 17,
  MB18 = 18,
  MB19 = 19,
  MB20 = 20,
  MB21 = 21,
  MB22 = 22,
  MB23 = 23,
  MB24 = 24,
  MB25 = 25,
  MB26 = 26,
  MB27 = 27,
  MB28 = 28,
  MB29 = 29,
  MB30 = 30,
  MB31 = 31,
  MB32 = 32,
  MB33 = 33,
  MB34 = 34,
  MB35 = 35,
  MB36 = 36,
  MB37 = 37,
  MB38 = 38,
  MB39 = 39,
  MB40 = 40,
  MB41 = 41,
  MB42 = 42,
  MB43 = 43,
  MB44 = 44,
  MB45 = 45,
  MB46 = 46,
  MB47 = 47,
  MB48 = 48,
  MB49 = 49,
  MB50 = 50,
  MB51 = 51,
  MB52 = 52,
  MB53 = 53,
  MB54 = 54,
  MB55 = 55,
  MB56 = 56,
  MB57 = 57,
  MB58 = 58,
  MB59 = 59,
  MB60 = 60,
  MB61 = 61,
  MB62 = 62,
  MB63 = 63,
  FIFO = 99
} FLEXCAN_MAILBOX;

typedef enum FLEXCAN_RXTX {
  TX,
  RX
} FLEXCAN_RXTX;

typedef enum FLEXCAN_CLOCK {
  CLK_OFF,
  CLK_8MHz,
  CLK_16MHz,
  CLK_20MHz,
  CLK_24MHz,
  CLK_30MHz,
  CLK_60MHz
} FLEXCAN_CLOCK;

typedef enum FLEXCAN_IDE {
  NONE = 0,
  EXT = 1,
  RTR = 2,
  STD = 3,
  INACTIVE
} FLEXCAN_IDE;

typedef enum FLEXCAN_FLTEN {
  ACCEPT_ALL = 0,
  REJECT_ALL = 1
} FLEXCAN_FLTEN;

typedef enum FLEXCAN_FIFOTABLE {
  A = 0,
  B = 1,
  C = 2
} FLEXCAN_FIFOTABLE;

typedef enum FLEXCAN_RXQUEUE_TABLE {
  RX_SIZE_2 = (uint16_t)2,
  RX_SIZE_4 = (uint16_t)4,
  RX_SIZE_8 = (uint16_t)8,
  RX_SIZE_16 = (uint16_t)16,
  RX_SIZE_32 = (uint16_t)32,
  RX_SIZE_64 = (uint16_t)64,
  RX_SIZE_128 = (uint16_t)128,
  RX_SIZE_256 = (uint16_t)256,
  RX_SIZE_512 = (uint16_t)512,
  RX_SIZE_1024 = (uint16_t)1024
} FLEXCAN_RXQUEUE_TABLE;

typedef enum FLEXCAN_RFFN_TABLE {
  RFFN_8 = (uint8_t)0,
  RFFN_16 = (uint8_t)1,
  RFFN_24 = (uint8_t)2,
  RFFN_32 = (uint8_t)3,
  RFFN_40 = (uint8_t)4,
  RFFN_48 = (uint8_t)5,
  RFFN_56 = (uint8_t)6,
  RFFN_64 = (uint8_t)7,
  RFFN_72 = (uint8_t)8,
  RFFN_80 = (uint8_t)9,
  RFFN_88 = (uint8_t)10,
  RFFN_96 = (uint8_t)11,
  RFFN_104 = (uint8_t)12,
  RFFN_112 = (uint8_t)13,
  RFFN_120 = (uint8_t)14,
  RFFN_128 = (uint8_t)15
} FLEXCAN_RFFN_TABLE;

typedef enum FLEXCAN_TXQUEUE_TABLE {
  TX_SIZE_2 = (uint16_t)2,
  TX_SIZE_4 = (uint16_t)4,
  TX_SIZE_8 = (uint16_t)8,
  TX_SIZE_16 = (uint16_t)16,
  TX_SIZE_32 = (uint16_t)32,
  TX_SIZE_64 = (uint16_t)64,
  TX_SIZE_128 = (uint16_t)128,
  TX_SIZE_256 = (uint16_t)256,
  TX_SIZE_512 = (uint16_t)512,
  TX_SIZE_1024 = (uint16_t)1024
} FLEXCAN_TXQUEUE_TABLE;

typedef enum CAN_DEV_TABLE {
  CAN1 = (uint32_t)0x401D0000,
  CAN2 = (uint32_t)0x401D4000,
  CAN3 = (uint32_t)0x401D8000
} CAN_DEV_TABLE;

#define FCTP_CLASS template<CAN_DEV_TABLE _bus, FLEXCAN_RXQUEUE_TABLE _rxSize = RX_SIZE_16, FLEXCAN_TXQUEUE_TABLE _txSize = TX_SIZE_16>
#define FCTP_FUNC template<CAN_DEV_TABLE _bus, FLEXCAN_RXQUEUE_TABLE _rxSize, FLEXCAN_TXQUEUE_TABLE _txSize>
#define FCTP_OPT FlexCAN_T4<_bus, _rxSize, _txSize>

class FlexCAN_T4_Base {
  public:
    virtual void flexcan_interrupt() = 0;
    virtual void setBaudRate(uint32_t baud = 1000000) = 0;
    virtual void events() = 0;
};

FlexCAN_T4_Base* _CAN1 = nullptr;
FlexCAN_T4_Base* _CAN2 = nullptr;
FlexCAN_T4_Base* _CAN3 = nullptr;

FCTP_CLASS class FlexCAN_T4 : public FlexCAN_T4_Base {
  public:
    FlexCAN_T4();
    void begin();
    uint32_t getBaudRate() { return currentBitrate; }
    void setTx(FLEXCAN_PINS pin = DEF);
    void setRx(FLEXCAN_PINS pin = DEF);
    void setBaudRate(uint32_t baud = 1000000);
    void reset() { softReset(); } /* reset flexcan controller (needs register restore capabilities...) */
    void setMaxMB(uint8_t last);
    void enableFIFO(bool status = 1);
    void disableFIFO() { enableFIFO(0); }
    void enableFIFOInterrupt(bool status = 1);
    void disableFIFOInterrupt() { enableFIFOInterrupt(0); }
    void mailboxStatus();
    int read(CAN_message_t &msg);
    int readMB(CAN_message_t &msg);
    int readFIFO(CAN_message_t &msg);
    bool setMB(const FLEXCAN_MAILBOX &mb_num, const FLEXCAN_RXTX &mb_rx_tx, const FLEXCAN_IDE &ide = STD);
    void enableMBInterrupt(const FLEXCAN_MAILBOX &mb_num, bool status = 1);
    void disableMBInterrupt(const FLEXCAN_MAILBOX &mb_num) { enableMBInterrupt(mb_num, 0); }
    void setMRP(bool mrp = 1); /* mailbox(1)/fifo(0) priority */
    void setRRS(bool rrs = 1); /* store remote frames */
    void onReceive(const FLEXCAN_MAILBOX &mb_num, _MB_ptr handler); /* individual mailbox callback function */
    void onReceive(_MB_ptr handler); /* global callback function */
    void setMBFilter(FLEXCAN_FLTEN input); /* enable/disable traffic for all MBs (for individual masking) */
    void setMBFilter(FLEXCAN_MAILBOX mb_num, FLEXCAN_FLTEN input); /* set specific MB to accept/deny traffic */
    bool setMBFilter(FLEXCAN_MAILBOX mb_num, uint32_t id1); /* input 1 ID to be filtered */
    bool setMBFilter(FLEXCAN_MAILBOX mb_num, uint32_t id1, uint32_t id2); /* input 2 ID's to be filtered */
    bool setMBFilter(FLEXCAN_MAILBOX mb_num, uint32_t id1, uint32_t id2, uint32_t id3); /* input 3 ID's to be filtered */
    bool setMBFilter(FLEXCAN_MAILBOX mb_num, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4); /* input 4 ID's to be filtered */
    bool setMBFilter(FLEXCAN_MAILBOX mb_num, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4, uint32_t id5); /* input 5 ID's to be filtered */
    bool setMBFilterRange(FLEXCAN_MAILBOX mb_num, uint32_t id1, uint32_t id2); /* filter a range of ids */
    int write(const CAN_message_t &msg); /* use any available mailbox for transmitting */
    int write(FLEXCAN_MAILBOX mb_num, const CAN_message_t &msg); /* use a single mailbox for transmitting */
    uint64_t readIMASK() { return (((uint64_t)FLEXCANb_IMASK2(_bus) << 32) | FLEXCANb_IMASK1(_bus)); }
    void flexcan_interrupt();
    Circular_Buffer<uint8_t, (uint32_t)_rxSize, sizeof(CAN_message_t)> rxBuffer;
    Circular_Buffer<uint8_t, (uint32_t)_txSize, sizeof(CAN_message_t)> txBuffer;
    void events();
    void setRFFN(FLEXCAN_RFFN_TABLE rffn = RFFN_8); /* Number Of Rx FIFO Filters (0 == 8 filters, 1 == 16 filters, etc.. */
    void setRFFN(uint8_t rffn) { setRFFN((FLEXCAN_RFFN_TABLE)constrain(rffn, 0, 15)); }
    void setFIFOFilterTable(FLEXCAN_FIFOTABLE letter);
    void setFIFOFilter(const FLEXCAN_FLTEN &input);
    bool setFIFOFilter(uint8_t filter, uint32_t id1, const FLEXCAN_IDE &ide, const FLEXCAN_IDE &remote = NONE); /* single ID per filter */
    bool setFIFOFilter(uint8_t filter, uint32_t id1, uint32_t id2, const FLEXCAN_IDE &ide, const FLEXCAN_IDE &remote = NONE); /* 2 ID's per filter */
    bool setFIFOFilterRange(uint8_t filter, uint32_t id1, uint32_t id2, const FLEXCAN_IDE &ide, const FLEXCAN_IDE &remote = NONE); /* ID range per filter */
    bool setFIFOFilter(uint8_t filter, uint32_t id1, const FLEXCAN_IDE &ide1, const FLEXCAN_IDE &remote1, uint32_t id2, const FLEXCAN_IDE &ide2, const FLEXCAN_IDE &remote2); /* TableB 2 ID / filter */
    bool setFIFOFilter(uint8_t filter, uint32_t id1, uint32_t id2, const FLEXCAN_IDE &ide1, const FLEXCAN_IDE &remote1, uint32_t id3, uint32_t id4, const FLEXCAN_IDE &ide2, const FLEXCAN_IDE &remote2); /* TableB 4 minimum ID / filter */
    bool setFIFOFilterRange(uint8_t filter, uint32_t id1, uint32_t id2, const FLEXCAN_IDE &ide1, const FLEXCAN_IDE &remote1, uint32_t id3, uint32_t id4, const FLEXCAN_IDE &ide2, const FLEXCAN_IDE &remote2); /* TableB dual range based IDs */
    void writeTxMailbox(uint8_t mb_num, const CAN_message_t &msg);
    void struct2queueTx(const CAN_message_t &msg);
    void struct2queueRx(const CAN_message_t &msg);
    void setClock(FLEXCAN_CLOCK clock = CLK_24MHz);
    uint32_t getClock();
    void FLEXCAN_ExitFreezeMode();
    void FLEXCAN_EnterFreezeMode();
  
  private:
    uint8_t getNumMailBoxes() { return FLEXCANb_MAXMB_SIZE(_bus); }
    uint8_t mailboxOffset();
    void softReset();
    int getFirstTxBox();
    _MB_ptr _mbHandlers[64]; /* individual mailbox handlers */
    _MB_ptr _mainHandler; /* global mailbox handler */
    uint64_t readIFLAG() { return (((uint64_t)FLEXCANb_IFLAG2(_bus) << 32) | FLEXCANb_IFLAG1(_bus)); }
    void writeIFLAG(uint64_t value);
    void writeIFLAGBit(uint8_t mb_num);
    void writeIMASK(uint64_t value);
    void writeIMASKBit(uint8_t mb_num, bool set = 1);
    uint32_t nvicIrq = 0; 
    uint32_t currentBitrate = 0UL;
    uint8_t mailbox_reader_increment = 0;
    uint8_t busNumber;
    void setMBFilterProcessing(FLEXCAN_MAILBOX mb_num, uint32_t filter_id, uint32_t calculated_mask);
    void mbCallbacks(const FLEXCAN_MAILBOX &mb_num, const CAN_message_t &msg);
};

#include "FlexCAN_T4.tpp"

#endif
