#include <FlexCAN_T4.h>
#include "imxrt_flexcan.h"
#include "Arduino.h"

void flexcan_isr_can3();
void flexcan_isr_can2();
void flexcan_isr_can1();

FCTP_FUNC FCTP_OPT::FlexCAN_T4() {
  if ( _bus == CAN3 ) _CAN3 = this;
  if ( _bus == CAN2 ) _CAN2 = this;
  if ( _bus == CAN1 ) _CAN1 = this;
}

FCTP_FUNC void FCTP_OPT::setClock(FLEXCAN_CLOCK clock) {
  if ( clock == CLK_OFF ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(3) | CCM_CSCMR2_CAN_CLK_PODF(0);
  if ( clock == CLK_8MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(2) | CCM_CSCMR2_CAN_CLK_PODF(9);
  if ( clock == CLK_16MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(2) | CCM_CSCMR2_CAN_CLK_PODF(4);
  if ( clock == CLK_24MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(1) | CCM_CSCMR2_CAN_CLK_PODF(0);
  if ( clock == CLK_20MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(2) | CCM_CSCMR2_CAN_CLK_PODF(3);
  if ( clock == CLK_30MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(0) | CCM_CSCMR2_CAN_CLK_PODF(1);
  if ( clock == CLK_60MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(0) | CCM_CSCMR2_CAN_CLK_PODF(0);
  if ( _CAN1 ) _CAN1->setBaudRate(currentBitrate);
  if ( _CAN2 ) _CAN2->setBaudRate(currentBitrate);
  if ( _CAN3 ) _CAN3->setBaudRate(currentBitrate);
}

FCTP_FUNC uint32_t FCTP_OPT::getClock() {
  uint8_t clockSrc = 0;
  if ( ((CCM_CSCMR2 & 0x300) >> 8) == 0 ) clockSrc = 60;
  if ( ((CCM_CSCMR2 & 0x300) >> 8) == 1 ) clockSrc = 24;
  if ( ((CCM_CSCMR2 & 0x300) >> 8) == 2 ) clockSrc = 80;
  return (clockSrc / (((CCM_CSCMR2 & 0xFC) >> 2) +1));
}

FCTP_FUNC void FCTP_OPT::begin() {
  setClock(CLK_24MHz);
  if ( _bus == CAN3 ) {
    nvicIrq = IRQ_CAN3;
    _VectorsRam[16 + nvicIrq] = flexcan_isr_can3;
    CCM_CCGR7 |= 0x3C0;
    busNumber = 3;
  }
  if ( _bus == CAN2 ) {
    nvicIrq = IRQ_CAN2;
    _VectorsRam[16 + nvicIrq] = flexcan_isr_can2;
    CCM_CCGR0 |= 0x3C0000;
    busNumber = 2;
  }
  if ( _bus == CAN1 ) {
    nvicIrq = IRQ_CAN1;
    _VectorsRam[16 + nvicIrq] = flexcan_isr_can1;
    CCM_CCGR0 |= 0x3C000;
    busNumber = 1;
  }
  FLEXCANb_MCR(_bus) &= ~FLEXCAN_MCR_MDIS; /* enable module */
  FLEXCAN_EnterFreezeMode();
  FLEXCANb_CTRL1(_bus) |= FLEXCAN_CTRL_LOM; /* listen only mode */
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_FRZ; /* enable freeze bit */
  while (FLEXCANb_MCR(_bus) & FLEXCAN_MCR_LPM_ACK);
  softReset(); /* reset bus */
  while (!(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK));
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_SRX_DIS; /* Disable self-reception */
  FLEXCANb_MCR(_bus) &= ~FLEXCAN_MCR_SUPV; /* Supervisor (1:default) or User Mode (0) */
  disableFIFO(); /* clears all data and layout to legacy mailbox mode */
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_IRMQ; // individual mailbox masking
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_AEN; // TX ABORT FEATURE
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_LPRIO_EN; // TX PRIORITY FEATURE
  FLEXCANb_MCR(_bus) &= ~0x8800; // disable DMA and FD (valid bits are reserved in legacy controllers)
//
  FLEXCANb_MCR(_bus) |= 0x800; // FDEN 
//
  FLEXCANb_CTRL2(_bus) |= FLEXCAN_CTRL2_RRS | // store remote frames
                                  FLEXCAN_CTRL2_EACEN | /* handles the way filtering works. Library adjusts to whether you use this or not */ 
                                  FLEXCAN_CTRL2_MRP; // mailbox > FIFO priority.
  FLEXCANb_MCR(_bus) &= ~FLEXCAN_MCR_HALT; /* start the CAN */
  while (FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK);
  while (FLEXCANb_MCR(_bus) & FLEXCAN_MCR_NOT_RDY); /* wait until ready */
  NVIC_ENABLE_IRQ(nvicIrq);
  setTx(); setRx();
}

FCTP_FUNC void FCTP_OPT::enableFIFOInterrupt(bool status) {
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN) ) return; /* FIFO must be enabled first */
  if ( FLEXCANb_IMASK1(_bus) & FLEXCAN_IMASK1_BUF5M ) return; /* FIFO interrupts already enabled */
  FLEXCANb_IMASK1(_bus) &= ~0xFF; /* disable FIFO interrupt flags */
  if ( status ) FLEXCANb_IMASK1(_bus) |= FLEXCAN_IMASK1_BUF5M; /* enable FIFO interrupt */
}

FCTP_FUNC void FCTP_OPT::enableMBInterrupt(const FLEXCAN_MAILBOX &mb_num, bool status) {
  if ( mb_num < mailboxOffset() ) return; /* mailbox not available */
  if ( status ) writeIMASKBit(mb_num); /* enable mailbox interrupt */
  else writeIMASKBit(mb_num, 0); /* disable mailbox interrupt */
}

FCTP_FUNC bool FCTP_OPT::setMB(const FLEXCAN_MAILBOX &mb_num, const FLEXCAN_RXTX &mb_rx_tx, const FLEXCAN_IDE &ide) {
  if ( mb_num < mailboxOffset() ) return 0; /* mailbox not available */
  writeIMASKBit(mb_num, 0); /* immediately disable mailbox interrupt */
  FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, mb_num)); // Reading Control Status atomically locks mailbox (if it is RX mode).
  FLEXCANb_MBn_ID(_bus, mb_num) = 0UL;
  FLEXCANb_MBn_WORD0(_bus, mb_num) = 0UL;
  FLEXCANb_MBn_WORD1(_bus, mb_num) = 0UL;
  if ( mb_rx_tx == RX ) {
    if ( ide != EXT ) FLEXCANb_MBn_CS(_bus, mb_num) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY);
    else FLEXCANb_MBn_CS(_bus, mb_num) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE;
  }
  if ( mb_rx_tx == TX ) {
    FLEXCANb_MBn_CS(_bus, mb_num) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
    writeIMASKBit(mb_num, 1);
  }
  if ( ide == INACTIVE ) {
    FLEXCANb_MBn_CS(_bus, mb_num) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_INACTIVE);
  }
  writeIFLAGBit(mb_num); /* clear mailbox reception flag */
  FLEXCANb_TIMER(_bus); /* reading timer unlocks individual mailbox */
  return 1;
}

FCTP_FUNC void FCTP_OPT::mailboxStatus() {
  if ( FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN ) {
    Serial4.print("FIFO Enabled --> "); ( FLEXCANb_IMASK1(_bus) & FLEXCAN_IFLAG1_BUF5I ) ? Serial4.println("Interrupt Enabled") : Serial4.println("Interrupt Disabled");
    Serial4.print("\tFIFO Filters in use: ");
    Serial4.println((((FLEXCANb_CTRL2(_bus) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 8); // 8 filters per 2 mailboxes
    Serial4.print("\tRemaining Mailboxes: ");
    uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_bus) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_bus) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
    if ( FLEXCANb_MAXMB_SIZE(_bus) < (6 + ((((FLEXCANb_CTRL2(_bus) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
    Serial4.println(remaining_mailboxes); // 8 filters per 2 mailboxes
    for ( uint8_t i = FLEXCANb_MAXMB_SIZE(_bus) - remaining_mailboxes; i < FLEXCANb_MAXMB_SIZE(_bus); i++ ) {
      switch ( FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, i)) ) {
        case 0b0000: {
            Serial4.print("\t\tMB"); Serial4.print(i); Serial4.println(" code: RX_INACTIVE"); break;
          }
        case 0b0100: {
            Serial4.print("\t\tMB"); Serial4.print(i); Serial4.print(" code: RX_EMPTY");
            (FLEXCANb_MBn_CS(_bus, i) & FLEXCAN_MB_CS_IDE) ? Serial4.println("\t(Extended Frame)") : Serial4.println("\t(Standard Frame)");
            break;
          }
        case 0b0010: {
            Serial4.print("\t\tMB"); Serial4.print(i); Serial4.println(" code: RX_FULL"); break;
          }
        case 0b0110: {
            Serial4.print("\t\tMB"); Serial4.print(i); Serial4.println(" code: RX_OVERRUN"); break;
          }
        case 0b1010: {
            Serial4.print("\t\tMB"); Serial4.print(i); Serial4.println(" code: RX_RANSWER"); break;
          }
        case 0b0001: {
            Serial4.print("\t\tMB"); Serial4.print(i); Serial4.println(" code: RX_BUSY"); break;
          }
        case 0b1000: {
            Serial4.print("\t\tMB"); Serial4.print(i); Serial4.println(" code: TX_INACTIVE"); break;
          }
        case 0b1001: {
            Serial4.print("\t\tMB"); Serial4.print(i); Serial4.println(" code: TX_ABORT"); break;
          }
        case 0b1100: {
            Serial4.print("\t\tMB"); Serial4.print(i); Serial4.print(" code: TX_DATA (Transmitting)");
            uint32_t extid = (FLEXCANb_MBn_CS(_bus, i) & FLEXCAN_MB_CS_IDE);
            (extid) ? Serial4.print("(Extended Frame)") : Serial4.print("(Standard Frame)");
            uint32_t dataIn = FLEXCANb_MBn_WORD0(_bus, i);
            uint32_t id = (FLEXCANb_MBn_ID(_bus, i) & FLEXCAN_MB_ID_EXT_MASK);
            if (!extid) id >>= FLEXCAN_MB_ID_STD_BIT_NO;
            Serial4.print("(ID: 0x"); Serial4.print(id, HEX); Serial4.print(")");
            Serial4.print("(Payload: "); Serial4.print((uint8_t)(dataIn >> 24), HEX);
            Serial4.print(" "); Serial4.print((uint8_t)(dataIn >> 16), HEX);
            Serial4.print(" "); Serial4.print((uint8_t)(dataIn >> 8), HEX);
            Serial4.print(" "); Serial4.print((uint8_t)dataIn, HEX);
            dataIn = FLEXCANb_MBn_WORD1(_bus, i);
            Serial4.print(" "); Serial4.print((uint8_t)(dataIn >> 24), HEX);
            Serial4.print(" "); Serial4.print((uint8_t)(dataIn >> 16), HEX);
            Serial4.print(" "); Serial4.print((uint8_t)(dataIn >> 8), HEX);
            Serial4.print(" "); Serial4.print((uint8_t)dataIn, HEX);
            Serial4.println(")");
            break;
          }
        case 0b1110: {
            Serial4.print("\t\tMB"); Serial4.print(i); Serial4.println(" code: TX_TANSWER"); break;
          }
      }
    } // for loop
    return;
  } // fifo detected ends here
  Serial4.print("FIFO Disabled\n\tMailboxes:\n");
  for ( uint8_t i = 0; i < FLEXCANb_MAXMB_SIZE(_bus); i++ ) {
    switch ( FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, i)) ) {
      case 0b0000: {
          Serial4.print("\t\tMB"); Serial4.print(i); Serial4.println(" code: RX_INACTIVE"); break;
        }
      case 0b0100: {
          Serial4.print("\t\tMB"); Serial4.print(i); Serial4.print(" code: RX_EMPTY");
          (FLEXCANb_MBn_CS(_bus, i) & FLEXCAN_MB_CS_IDE) ? Serial4.println("\t(Extended Frame)") : Serial4.println("\t(Standard Frame)");
          break;
        }
      case 0b0010: {
          Serial4.print("\t\tMB"); Serial4.print(i); Serial4.println(" code: RX_FULL"); break;
        }
      case 0b0110: {
          Serial4.print("\t\tMB"); Serial4.print(i); Serial4.println(" code: RX_OVERRUN"); break;
        }
      case 0b1010: {
          Serial4.print("\t\tMB"); Serial4.print(i); Serial4.println(" code: RX_RANSWER"); break;
        }
      case 0b0001: {
          Serial4.print("\t\tMB"); Serial4.print(i); Serial4.println(" code: RX_BUSY"); break;
        }
      case 0b1000: {
          Serial4.print("\t\tMB"); Serial4.print(i); Serial4.println(" code: TX_INACTIVE"); break;
        }
      case 0b1001: {
          Serial4.print("\t\tMB"); Serial4.print(i); Serial4.println(" code: TX_ABORT"); break;
        }
      case 0b1100: {
          Serial4.print("\t\tMB"); Serial4.print(i); Serial4.print(" code: TX_DATA (Transmitting)");
          uint32_t extid = (FLEXCANb_MBn_CS(_bus, i) & FLEXCAN_MB_CS_IDE);
          (extid) ? Serial4.print("(Extended Frame)") : Serial4.print("(Standard Frame)");
          uint32_t dataIn = FLEXCANb_MBn_WORD0(_bus, i);
          uint32_t id = (FLEXCANb_MBn_ID(_bus, i) & FLEXCAN_MB_ID_EXT_MASK);
          if (!extid) id >>= FLEXCAN_MB_ID_STD_BIT_NO;
          Serial4.print("(ID: 0x"); Serial4.print(id, HEX); Serial4.print(")");
          Serial4.print("(Payload: "); Serial4.print((uint8_t)(dataIn >> 24), HEX);
          Serial4.print(" "); Serial4.print((uint8_t)(dataIn >> 16), HEX);
          Serial4.print(" "); Serial4.print((uint8_t)(dataIn >> 8), HEX);
          Serial4.print(" "); Serial4.print((uint8_t)dataIn, HEX);
          dataIn = FLEXCANb_MBn_WORD1(_bus, i);
          Serial4.print(" "); Serial4.print((uint8_t)(dataIn >> 24), HEX);
          Serial4.print(" "); Serial4.print((uint8_t)(dataIn >> 16), HEX);
          Serial4.print(" "); Serial4.print((uint8_t)(dataIn >> 8), HEX);
          Serial4.print(" "); Serial4.print((uint8_t)dataIn, HEX);
          Serial4.println(")");
          break;
        }
      case 0b1110: {
          Serial4.print("\t\tMB"); Serial4.print(i); Serial4.println(" code: TX_TANSWER"); break;
        }
    }
  } // for loop
}

FCTP_FUNC void FCTP_OPT::enableFIFO(bool status) {
  bool frz_flag_negate = 0;
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK) ) { // currently not in freeze mode
    frz_flag_negate = 1; FLEXCAN_EnterFreezeMode();
  }
  FLEXCANb_MCR(_bus) &= ~FLEXCAN_MCR_FEN; // Disable FIFO if already enabled for cleanup.
  writeIMASK(0ULL); // disable all FIFO/MB Interrupts
  for (uint8_t i = 0; i < FLEXCANb_MAXMB_SIZE(_bus); i++ ) { // clear all mailboxes
    FLEXCANb_MBn_ID(_bus, i) = 0UL;
    FLEXCANb_MBn_WORD0(_bus, i) = 0UL;
    FLEXCANb_MBn_WORD1(_bus, i) = 0UL;
    FLEXCANb_MBn_CS(_bus, i) = 0UL;
    FLEXCANb_RXIMR(_bus, i) = 0UL; // CLEAR MAILBOX MASKS (RXIMR)
  }
  /*
    RXMGMASK is provided for legacy application support.
    â€¢  When the MCR[IRMQ] bit is negated, RXMGMASK is always in effect.
    â€¢  When the MCR[IRMQ] bit is asserted, RXMGMASK has no effect.
    RXMGMASK is used to mask the filter fields of all Rx MBs, excluding MBs 14-15,
    which have individual mask registers
    RX14MASK/RX15MASK is provided for legacy application support. When the MCR[IRMQ] bit is
    asserted, RX14MASK/RX15MASK has no effect
  */
  FLEXCANb_RXMGMASK(_bus) = FLEXCANb_RXFGMASK(_bus) = 0;
  /*
      Enable RX FIFO
      Before enabling the RFEN, the CPU must service the IFLAG bits asserted in the Rx
      FIFO region; Otherwise, these IFLAG bits will mistakenly show
      the related MBs now belonging to FIFO as having contents to be serviced.
  */
  writeIFLAG(readIFLAG()); // (all bits reset when written back)
  if ( status ) {
    FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_FEN;
    /*
      Each group of eight filters occupies a memory space equivalent to two Message Buffers which
      means that the more filters are implemented the less Mailboxes will be available.
    */
    FLEXCAN_set_rffn(FLEXCANb_CTRL2(_bus), 0); // setup 8 Filters for FIFO, 0-5 = FIFO, 6-7 FILTERS, 8-64 MBs, max value 0xF which leaves MB14/15 free to use.
    // Setup TX mailboxes from 8 -> max, FIFO uses the first 8 (6MB for FIFO, 2MB for 8 filters for FIFO).

    for (uint8_t i = mailboxOffset(); i < FLEXCANb_MAXMB_SIZE(_bus); i++) FLEXCANb_MBn_CS(_bus,i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  } 
  else { // FIFO disabled default setup of mailboxes, 0-7 RX, 8-15 TX
    for (uint8_t i = 0; i < FLEXCANb_MAXMB_SIZE(_bus); i++ ) { // clear all mailboxes
      if ( i < (FLEXCANb_MAXMB_SIZE(_bus) / 2) ) {
        FLEXCANb_MBn_ID(_bus, i) = 0x00000000;
        FLEXCANb_MBn_WORD0(_bus, i) = 0x00000000;
        FLEXCANb_MBn_WORD1(_bus, i) = 0x00000000;
        if ( (i < (FLEXCANb_MAXMB_SIZE(_bus) / 4)) ) FLEXCANb_MBn_CS(_bus, i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY);
        else FLEXCANb_MBn_CS(_bus, i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | FLEXCAN_MB_CS_IDE | FLEXCAN_MB_CS_SRR;
      }
      else {
        FLEXCANb_MBn_ID(_bus, i) = 0x00000000;
        FLEXCANb_MBn_WORD0(_bus, i) = 0x00000000;
        FLEXCANb_MBn_WORD1(_bus, i) = 0x00000000;
        FLEXCANb_MBn_CS(_bus, i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
      }
    }
  }
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
}

FCTP_FUNC void FCTP_OPT::writeIFLAG(uint64_t value) {
  FLEXCANb_IFLAG2(_bus) = value >> 32;
  FLEXCANb_IFLAG1(_bus) = value;
}

FCTP_FUNC void FCTP_OPT::writeIFLAGBit(uint8_t mb_num) {
  if ( mb_num < 32 ) FLEXCANb_IFLAG1(_bus) |= (1UL << mb_num);
  else FLEXCANb_IFLAG2(_bus) |= (1UL << (mb_num - 32));
}

FCTP_FUNC void FCTP_OPT::writeIMASK(uint64_t value) {
  FLEXCANb_IMASK2(_bus) = value >> 32;
  FLEXCANb_IMASK1(_bus) = value;
}

FCTP_FUNC void FCTP_OPT::writeIMASKBit(uint8_t mb_num, bool set) {
  if ( mb_num < 32 ) (( set ) ? FLEXCANb_IMASK1(_bus) |= (1UL << mb_num) : FLEXCANb_IMASK1(_bus) &= ~(1UL << mb_num));
  else (( set ) ? FLEXCANb_IMASK2(_bus) |= (1UL << (mb_num - 32)) : FLEXCANb_IMASK2(_bus) &= ~(1UL << (mb_num - 32)));
}

FCTP_FUNC void FCTP_OPT::writeTxMailbox(uint8_t mb_num, const CAN_message_t &msg) {
  writeIFLAGBit(mb_num); // 1st step clear flag in case it's set as per datasheet
  ( msg.flags.remote ) ? FLEXCANb_MBn_CS(_bus, mb_num) |= FLEXCAN_MB_CS_RTR : FLEXCANb_MBn_CS(_bus, mb_num) &= ~FLEXCAN_MB_CS_RTR;
//FLEXCANb_MBn_CS(_bus, mb_num) = (1UL << 31); // FD frame
  FLEXCANb_MBn_CS(_bus, mb_num) |= FLEXCAN_MB_CS_LENGTH(msg.len+7);
FLEXCANb_MBn_WORD0(_bus, mb_num) |= 0x55555555;
//FLEXCANb_MBn_WORD1(_bus, mb_num) |= 0x55555555;
//FLEXCANb_MBn_WORD2(_bus, mb_num) |= 0x55555555;

//  for ( uint8_t i = 0; i < msg.len; i++ ) ( i < 4 ) ? (FLEXCANb_MBn_WORD0(_bus, mb_num) |= (*(msg.buf + i)) << ((3 - i) * 8)) : (FLEXCANb_MBn_WORD1(_bus, mb_num) |= (*(msg.buf + i)) << ((7 - i) * 8));

//for ( uint8_t i = 0; i < 8; i++ ) FLEXCANb_MBn_DATA(_bus, mb_num, i) = 0x55555555;


  FLEXCANb_MBn_ID(_bus, mb_num) = (( msg.flags.extended ) ? ( msg.id & FLEXCAN_MB_ID_EXT_MASK ) : FLEXCAN_MB_ID_IDSTD(msg.id));
  FLEXCANb_MBn_CS(_bus, mb_num) |= FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE);
  if ( msg.flags.remote ) {
    uint32_t timeout = millis();
    while ( !(readIFLAG() & (1ULL << mb_num)) && (millis() - timeout < 10) );
    writeIFLAGBit(mb_num);
    FLEXCANb_MBn_CS(_bus, mb_num) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  }
}

/*
FCTP_FUNC void FCTP_OPT::writeTxMailbox(uint8_t mb_num, const CAN_message_t &msg) {
  writeIFLAGBit(mb_num); // 1st step clear flag in case it's set as per datasheet
  ( msg.flags.remote ) ? FLEXCANb_MBn_CS(_bus, mb_num) |= FLEXCAN_MB_CS_RTR : FLEXCANb_MBn_CS(_bus, mb_num) &= ~FLEXCAN_MB_CS_RTR;
  FLEXCANb_MBn_CS(_bus, mb_num) |= FLEXCAN_MB_CS_LENGTH(msg.len);
  FLEXCANb_MBn_WORD0(_bus, mb_num) = FLEXCANb_MBn_WORD1(_bus, mb_num) = 0;
  for ( uint8_t i = 0; i < msg.len; i++ ) ( i < 4 ) ? (FLEXCANb_MBn_WORD0(_bus, mb_num) |= (*(msg.buf + i)) << ((3 - i) * 8)) : (FLEXCANb_MBn_WORD1(_bus, mb_num) |= (*(msg.buf + i)) << ((7 - i) * 8));
  FLEXCANb_MBn_ID(_bus, mb_num) = (( msg.flags.extended ) ? ( msg.id & FLEXCAN_MB_ID_EXT_MASK ) : FLEXCAN_MB_ID_IDSTD(msg.id));
  FLEXCANb_MBn_CS(_bus, mb_num) |= FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE);
  if ( msg.flags.remote ) {
    uint32_t timeout = millis();
    while ( !(readIFLAG() & (1ULL << mb_num)) && (millis() - timeout < 10) );
    writeIFLAGBit(mb_num);
    FLEXCANb_MBn_CS(_bus, mb_num) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  }
}
*/

FCTP_FUNC uint8_t FCTP_OPT::mailboxOffset() {
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN ) ) return 0; /* return offset 0 since FIFO is disabled */
  uint32_t remaining_mailboxes = FLEXCANb_MAXMB_SIZE(_bus) - 6 /* MAXMB - FIFO */ - ((((FLEXCANb_CTRL2(_bus) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2);
  if ( FLEXCANb_MAXMB_SIZE(_bus) < (6 + ((((FLEXCANb_CTRL2(_bus) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 2))) remaining_mailboxes = 0;
  return (FLEXCANb_MAXMB_SIZE(_bus) - remaining_mailboxes); /* otherwise return offset MB position after FIFO area */
}

FCTP_FUNC void FCTP_OPT::setMaxMB(uint8_t last) {
  last = constrain(last,1,64);
  last--;
  FLEXCAN_EnterFreezeMode();
  bool fifo_was_cleared = 0;
  if ( FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN ) fifo_was_cleared = 1; /* let fifo clear current mailboxes */
  disableFIFO();
  writeIFLAG(readIFLAG()); // (all bits reset when written back) (needed for MAXMB changes)
  FLEXCANb_MCR(_bus) &= ~0x7F; // clear current value
  FLEXCANb_MCR(_bus) |= last; // set mailbox max
  if ( fifo_was_cleared ) enableFIFO();
  FLEXCAN_ExitFreezeMode();
}

FCTP_FUNC void FCTP_OPT::softReset() {
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_SOFT_RST;
  while (FLEXCANb_MCR(_bus) & FLEXCAN_MCR_SOFT_RST);
}

FCTP_FUNC void FCTP_OPT::FLEXCAN_ExitFreezeMode() {
  FLEXCANb_MCR(_bus) &= ~FLEXCAN_MCR_HALT;
  while (FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK);
}

FCTP_FUNC void FCTP_OPT::FLEXCAN_EnterFreezeMode() {
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT;
  while (!(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK));
}

FCTP_FUNC void FCTP_OPT::setBaudRate(uint32_t baud) {
  if ( !getClock() ) return;
  unsigned mask;
  int clockXbits = 0, clockDivbits = 0, clockFreq = 0;
  currentBitrate = baud;

  // Get clock
  mask = ((1 << 2)-1) << 8;
  clockXbits = (CCM_CSCMR2 & mask) >> 8;
  //Get clock divider
  mask = ((1 << 6) - 1) << 2;
  clockDivbits = (CCM_CSCMR2 & mask) >> 2;

  if(clockXbits == 0x00){
    clockFreq = 60000000;
  } else if(clockXbits == 0x01){
    clockFreq = 24000000;
  } else if(clockXbits == 0x02){
    clockFreq = 80000000;
  }

  //CAN Clock Frequency
  clockFreq = clockFreq/(clockDivbits+1);

  uint32_t divisor = 0, bestDivisor = 0, result = clockFreq / baud / (divisor + 1);
  int error = baud - (clockFreq / (result * (divisor + 1))), bestError = error;
  bool frz_flag_negate = 0;

  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK) ) { // currently not in freeze mode
    frz_flag_negate = 1; FLEXCAN_EnterFreezeMode();
  }

  while (result > 5) {
    divisor++;
    result = clockFreq / baud / (divisor + 1);
    if (result <= 25) {
      error = baud - (clockFreq / (result * (divisor + 1)));
      if (error < 0) error *= -1;
      if (error < bestError) {
        bestError = error;
        bestDivisor = divisor;
      }
      if ((error == bestError) && (result > 11) && (result < 19)) {
        bestError = error;
        bestDivisor = divisor;
      }
    }
  }

  divisor = bestDivisor;
  result = clockFreq / baud / (divisor + 1);

  if ((result < 5) || (result > 25) || (bestError > 300)) {
    if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
    return;
  }

  result -= 5; // the bitTimingTable is offset by 5 since there was no reason to store bit timings for invalid numbers
  uint8_t bitTimingTable[21][3] = {
    {0, 0, 1}, //5
    {1, 0, 1}, //6
    {1, 1, 1}, //7
    {2, 1, 1}, //8
    {2, 2, 1}, //9
    {2, 3, 1}, //10
    {2, 3, 2}, //11
    {2, 4, 2}, //12
    {2, 5, 2}, //13
    {2, 5, 3}, //14
    {2, 6, 3}, //15
    {2, 7, 3}, //16
    {2, 7, 4}, //17
    {3, 7, 4}, //18
    {3, 7, 5}, //19
    {4, 7, 5}, //20
    {4, 7, 6}, //21
    {5, 7, 6}, //22
    {6, 7, 6}, //23
    {6, 7, 7}, //24
    {7, 7, 7}, //25
  }, propSeg = bitTimingTable[result][0], pSeg1 = bitTimingTable[result][1], pSeg2 = bitTimingTable[result][2];
  FLEXCANb_CTRL1(_bus) = (FLEXCAN_CTRL_PROPSEG(propSeg) | FLEXCAN_CTRL_RJW(1) | FLEXCAN_CTRL_PSEG1(pSeg1) |
                    FLEXCAN_CTRL_PSEG2(pSeg2) | FLEXCAN_CTRL_ERR_MSK | FLEXCAN_CTRL_PRESDIV(divisor));
  FLEXCANb_CTRL1(_bus) &= ~FLEXCAN_CTRL_LOM; /* disable listen-only mode */
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
}

FCTP_FUNC void FCTP_OPT::setMRP(bool mrp) { /* mailbox priority (1) or FIFO priority (0) */
  FLEXCAN_EnterFreezeMode();
  if ( mrp ) FLEXCANb_CTRL2(_bus) |= FLEXCAN_CTRL2_MRP;
  else FLEXCANb_CTRL2(_bus) &= ~FLEXCAN_CTRL2_MRP;
  FLEXCAN_ExitFreezeMode();
}

FCTP_FUNC void FCTP_OPT::setRRS(bool rrs) { /* store remote frames */
  FLEXCAN_EnterFreezeMode();
  if ( rrs ) FLEXCANb_CTRL2(_bus) |= FLEXCAN_CTRL2_RRS;
  else FLEXCANb_CTRL2(_bus) &= ~FLEXCAN_CTRL2_RRS;
  FLEXCAN_ExitFreezeMode();
}

FCTP_FUNC void FCTP_OPT::setTx(FLEXCAN_PINS pin) {
  if ( _bus == CAN3 ) {
    if ( pin == DEF ) {
      IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_36 = 0x19; // pin31 T3B2
      IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_36 = 0x10B0; // pin31 T3B2
    }
  }
  if ( _bus == CAN2 ) {
    if ( pin == DEF ) {
      IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02 = 0x10; // pin 1 T4B1+B2
      IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_02 = 0x10B0; // pin 1 T4B1+B2
    }
  }
  if ( _bus == CAN1 ) {
    if ( pin == DEF ) {
      IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_08 = 0x12; // pin 22 T4B1+B2
      IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_08 = 0x10B0; // pin 22 T4B1+B2
    }
  }
}

FCTP_FUNC void FCTP_OPT::setRx(FLEXCAN_PINS pin) {
  /* DAISY REGISTER CAN3
    00 GPIO_EMC_37_ALT9 — Selecting Pad: GPIO_EMC_37 for Mode: ALT9
    01 GPIO_AD_B0_15_ALT8 — Selecting Pad: GPIO_AD_B0_15 for Mode: ALT8
    10 GPIO_AD_B0_11_ALT8 — Selecting Pad: GPIO_AD_B0_11 for Mode: ALT8
  */
  /* DAISY REGISTER CAN2
    00 GPIO_EMC_10_ALT3 — Selecting Pad: GPIO_EMC_10 for Mode: ALT3
    01 GPIO_AD_B0_03_ALT0 — Selecting Pad: GPIO_AD_B0_03 for Mode: ALT0
    10 GPIO_AD_B0_15_ALT6 — Selecting Pad: GPIO_AD_B0_15 for Mode: ALT6
    11 GPIO_B1_09_ALT6 — Selecting Pad: GPIO_B1_09 for Mode: ALT6
  */
  /* DAISY REGISTER CAN1
    00 GPIO_SD_B1_03_ALT4 — Selecting Pad: GPIO_SD_B1_03 for Mode: ALT4
    01 GPIO_EMC_18_ALT3 — Selecting Pad: GPIO_EMC_18 for Mode: ALT3
    10 GPIO_AD_B1_09_ALT2 — Selecting Pad: GPIO_AD_B1_09 for Mode: ALT2
    11 GPIO_B0_03_ALT2 — Selecting Pad: GPIO_B0_03 for Mode: ALT2
  */
  if ( _bus == CAN3 ) {
    if ( pin == DEF ) {
      IOMUXC_CANFD_IPP_IND_CANRX_SELECT_INPUT = 0x00;
      IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_37 = 0x19; // pin30 T3B2
      IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_37 = 0x10B0; // pin30 T3B2
    }
  }
  if ( _bus == CAN2 ) {
    if ( pin == DEF ) {
      IOMUXC_FLEXCAN2_RX_SELECT_INPUT = 0x01;
      IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 = 0x10; // pin 0 T4B1+B2
      IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_03 = 0x10B0; // pin 0 T4B1+B2
    }
  }
  if ( _bus == CAN1 ) {
    if ( pin == DEF ) {
      IOMUXC_FLEXCAN1_RX_SELECT_INPUT = 0x02;
      IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_09 = 0x12; // pin 23 T4B1+B2
      IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_09 = 0x10B0; // pin 23 T4B1+B2
    }
  }
}

FCTP_FUNC void FCTP_OPT::setMBFilterProcessing(FLEXCAN_MAILBOX mb_num, uint32_t filter_id, uint32_t calculated_mask) {
  bool frz_flag_negate = 0;
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK) ) { // currently not in freeze mode
    frz_flag_negate = 1; FLEXCAN_EnterFreezeMode();
  }
  FLEXCANb_RXIMR(_bus, mb_num) = calculated_mask;
  if ( (FLEXCANb_CTRL2(_bus) & FLEXCAN_CTRL2_EACEN) ) {
    if ( (FLEXCANb_MBn_CS(_bus, mb_num) & FLEXCAN_MB_CS_IDE) ) FLEXCANb_RXIMR(_bus, mb_num) |= (1UL << 30);
  }
  else FLEXCANb_RXIMR(_bus, mb_num) |= (1UL << 30);
  if (!(FLEXCANb_MBn_CS(_bus, mb_num) & FLEXCAN_MB_CS_IDE)) FLEXCANb_MBn_ID(_bus, mb_num) = FLEXCAN_MB_ID_IDSTD(filter_id);
  else FLEXCANb_MBn_ID(_bus, mb_num) = FLEXCAN_MB_ID_IDEXT(filter_id);
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
}

FCTP_FUNC void FCTP_OPT::setMBFilter(FLEXCAN_FLTEN input) {
  bool frz_flag_negate = 0;
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK) ) { // currently not in freeze mode
    frz_flag_negate = 1; FLEXCAN_EnterFreezeMode();
  }
  for (uint8_t i = mailboxOffset(); i < FLEXCANb_MAXMB_SIZE(_bus); i++) {
    if ( (FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, i)) >> 3) ) continue; /* skip TX mailboxes */
    if ( input == ACCEPT_ALL ) FLEXCANb_RXIMR(_bus, i) = 0UL; // (RXIMR)
    if ( input == REJECT_ALL ) FLEXCANb_RXIMR(_bus, i) = ~0UL; // (RXIMR)
    FLEXCANb_MBn_ID(_bus, i) = 0UL;
  }
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
}

FCTP_FUNC void FCTP_OPT::setMBFilter(FLEXCAN_MAILBOX mb_num, FLEXCAN_FLTEN input) {
  if ( mb_num < mailboxOffset() || mb_num >= FLEXCANb_MAXMB_SIZE(_bus) ) return; /* mailbox not available */
  if ( (FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, mb_num)) >> 3) ) return; /* exit on TX mailbox */ 
  bool frz_flag_negate = 0;
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK) ) { // currently not in freeze mode
    frz_flag_negate = 1; FLEXCAN_EnterFreezeMode();
  }
  if ( input == ACCEPT_ALL ) FLEXCANb_RXIMR(_bus, mb_num) = 0UL; // (RXIMR)
  if ( input == REJECT_ALL ) FLEXCANb_RXIMR(_bus, mb_num) = ~0UL; // (RXIMR)
  FLEXCANb_MBn_ID(_bus, mb_num) = 0UL;
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
}

FCTP_FUNC bool FCTP_OPT::setMBFilter(FLEXCAN_MAILBOX mb_num, uint32_t id1) {
  if ( mb_num < mailboxOffset() || mb_num >= FLEXCANb_MAXMB_SIZE(_bus) ) return 0; /* mailbox not available */
  if ( (FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, mb_num)) >> 3) ) return 0; /* exit on TX mailbox */ 
  bool extbit = FLEXCANb_MBn_CS(_bus, mb_num) & FLEXCAN_MB_CS_IDE;
  if ( (id1 > 0x7FF) != extbit ) return 0;
  uint32_t mask = ( !extbit ) ? FLEXCAN_MB_ID_IDSTD(((id1) ^ (id1)) ^ 0x7FF) : FLEXCAN_MB_ID_IDEXT(((id1) ^ (id1)) ^ 0x1FFFFFFF);
  setMBFilterProcessing(mb_num,id1,mask);
  return 1;
}

FCTP_FUNC bool FCTP_OPT::setMBFilter(FLEXCAN_MAILBOX mb_num, uint32_t id1, uint32_t id2) {
  if ( mb_num < mailboxOffset() || mb_num >= FLEXCANb_MAXMB_SIZE(_bus) ) return 0; /* mailbox not available */
  if ( (FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, mb_num)) >> 3) ) return 0; /* exit on TX mailbox */ 
  bool extbit = FLEXCANb_MBn_CS(_bus, mb_num) & FLEXCAN_MB_CS_IDE;
  if ( (id1 > 0x7FF) != extbit || (id2 > 0x7FF) != extbit ) return 0;
  uint32_t mask = ( !extbit ) ? FLEXCAN_MB_ID_IDSTD(((id1 | id2) ^ (id1 & id2)) ^ 0x7FF) : FLEXCAN_MB_ID_IDEXT(((id1 | id2) ^ (id1 & id2)) ^ 0x1FFFFFFF);
  setMBFilterProcessing(mb_num,id1,mask);
  return 1;
}

FCTP_FUNC bool FCTP_OPT::setMBFilter(FLEXCAN_MAILBOX mb_num, uint32_t id1, uint32_t id2, uint32_t id3) {
  if ( mb_num < mailboxOffset() || mb_num >= FLEXCANb_MAXMB_SIZE(_bus) ) return 0; /* mailbox not available */
  if ( (FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, mb_num)) >> 3) ) return 0; /* exit on TX mailbox */ 
  bool extbit = FLEXCANb_MBn_CS(_bus, mb_num) & FLEXCAN_MB_CS_IDE;
  if ( (id1 > 0x7FF) != extbit || (id2 > 0x7FF) != extbit || (id3 > 0x7FF) != extbit ) return 0;
  uint32_t mask = ( !extbit ) ? FLEXCAN_MB_ID_IDSTD(((id1 | id2 | id3) ^ (id1 & id2 & id3)) ^ 0x7FF) : FLEXCAN_MB_ID_IDEXT(((id1 | id2 | id3) ^ (id1 & id2 & id3)) ^ 0x1FFFFFFF);
  setMBFilterProcessing(mb_num,id1,mask);
  return 1;
}

FCTP_FUNC bool FCTP_OPT::setMBFilter(FLEXCAN_MAILBOX mb_num, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4) {
  if ( mb_num < mailboxOffset() || mb_num >= FLEXCANb_MAXMB_SIZE(_bus) ) return 0; /* mailbox not available */
  if ( (FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, mb_num)) >> 3) ) return 0; /* exit on TX mailbox */ 
  bool extbit = FLEXCANb_MBn_CS(_bus, mb_num) & FLEXCAN_MB_CS_IDE;
  if ( (id1 > 0x7FF) != extbit || (id2 > 0x7FF) != extbit || (id3 > 0x7FF) != extbit || (id4 > 0x7FF) != extbit ) return 0;
  uint32_t mask = ( !extbit ) ? FLEXCAN_MB_ID_IDSTD(((id1 | id2 | id3 | id4) ^ (id1 & id2 & id3 & id4)) ^ 0x7FF) : FLEXCAN_MB_ID_IDEXT(((id1 | id2 | id3 | id4) ^ (id1 & id2 & id3 & id4)) ^ 0x1FFFFFFF);
  setMBFilterProcessing(mb_num,id1,mask);
  return 1;
}

FCTP_FUNC bool FCTP_OPT::setMBFilter(FLEXCAN_MAILBOX mb_num, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4, uint32_t id5) {
  if ( mb_num < mailboxOffset() || mb_num >= FLEXCANb_MAXMB_SIZE(_bus) ) return 0; /* mailbox not available */
  if ( (FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, mb_num)) >> 3) ) return 0; /* exit on TX mailbox */ 
  bool extbit = FLEXCANb_MBn_CS(_bus, mb_num) & FLEXCAN_MB_CS_IDE;
  if ( (id1 > 0x7FF) != extbit || (id2 > 0x7FF) != extbit || (id3 > 0x7FF) != extbit || (id4 > 0x7FF) != extbit || (id5 > 0x7FF) != extbit ) return 0;
  uint32_t mask = ( !extbit ) ? FLEXCAN_MB_ID_IDSTD(((id1 | id2 | id3 | id4 | id5) ^ (id1 & id2 & id3 & id4 & id5)) ^ 0x7FF) : FLEXCAN_MB_ID_IDEXT(((id1 | id2 | id3 | id4 | id5) ^ (id1 & id2 & id3 & id4 & id5)) ^ 0x1FFFFFFF);
  setMBFilterProcessing(mb_num,id1,mask);
  return 1;
}

FCTP_FUNC bool FCTP_OPT::setMBFilterRange(FLEXCAN_MAILBOX mb_num, uint32_t id1, uint32_t id2) {
  if ( mb_num < mailboxOffset() || mb_num >= FLEXCANb_MAXMB_SIZE(_bus) ) return 0; /* mailbox not available */
  if ( (FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, mb_num)) >> 3) ) return 0; /* exit on TX mailbox */ 
  bool extbit = FLEXCANb_MBn_CS(_bus, mb_num) & FLEXCAN_MB_CS_IDE;
  if ( (id1 > 0x7FF) != extbit || (id2 > 0x7FF) != extbit ) return 0;
  if ( id1 > id2 || ((id2 > id1) && (id2-id1>1000)) || !id1 || !id2 ) return 0; /* don't play around... */
  uint32_t stage1 = id1, stage2 = id1;
  for ( uint32_t i = id1 + 1; i <= id2; i++ ) {
    stage1 |= i; stage2 &= i;
  }
  uint32_t mask = ( !extbit ) ? FLEXCAN_MB_ID_IDSTD( (stage1 ^ stage2) ^ 0x1FFFFFFF ) : FLEXCAN_MB_ID_IDEXT( (stage1 ^ stage2) ^ 0x1FFFFFFF );
  setMBFilterProcessing(mb_num,id1,mask);
  return 1;
}

FCTP_FUNC int FCTP_OPT::readFIFO(CAN_message_t &msg) {
  if ( FLEXCANb_IMASK1(_bus) & FLEXCAN_IMASK1_BUF5M ) return 0; /* FIFO interrupt enabled, manual read blocked */
  if ( ( FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN ) && ( FLEXCANb_IFLAG1(_bus) & FLEXCAN_IFLAG1_BUF5I ) ) {
    msg.len = FLEXCAN_get_length(FLEXCANb_MBn_CS(_bus, 0));
    msg.flags.extended = (FLEXCANb_MBn_CS(_bus, 0) & FLEXCAN_MB_CS_IDE) ? 1 : 0;
    msg.flags.remote = (FLEXCANb_MBn_CS(_bus, 0) & FLEXCAN_MB_CS_RTR) ? 1 : 0;
    msg.timestamp = FLEXCAN_get_timestamp (FLEXCANb_MBn_CS(_bus, 0));
    msg.id = (FLEXCANb_MBn_ID(_bus, 0) & FLEXCAN_MB_ID_EXT_MASK);
    if (!msg.flags.extended) msg.id >>= FLEXCAN_MB_ID_STD_BIT_NO;
    uint32_t dataIn = FLEXCANb_MBn_WORD0(_bus, 0);
    msg.buf[0] = dataIn >> 24; msg.buf[1] = dataIn >> 16; msg.buf[2] = dataIn >> 8; msg.buf[3] = dataIn;
    dataIn = FLEXCANb_MBn_WORD1(_bus, 0);
    msg.buf[4] = dataIn >> 24; msg.buf[5] = dataIn >> 16; msg.buf[6] = dataIn >> 8; msg.buf[7] = dataIn;
    FLEXCANb_IFLAG1(_bus) = FLEXCAN_IFLAG1_BUF5I; /* clear FIFO bit only! */
    if ( FLEXCANb_IFLAG1(_bus) & FLEXCAN_IFLAG1_BUF6I ) FLEXCANb_IFLAG1(_bus) = FLEXCAN_IFLAG1_BUF6I;
    if ( FLEXCANb_IFLAG1(_bus) & FLEXCAN_IFLAG1_BUF7I ) FLEXCANb_IFLAG1(_bus) = FLEXCAN_IFLAG1_BUF7I;
    msg.bus = busNumber;
    msg.mb = 99; /* store the mailbox the message came from (for callback reference) */
    return 1;
  }
  return 0;
}

FCTP_FUNC int FCTP_OPT::getFirstTxBox() {
  for (uint8_t i = mailboxOffset(); i < FLEXCANb_MAXMB_SIZE(_bus); i++) {
    if ( (FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, i)) >> 3) ) return i; // if TX
  }
  return -1;
}

FCTP_FUNC int FCTP_OPT::read(CAN_message_t &msg) {
  bool _random = random(0, 2);
  if ( ( !_random ) && ( FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN ) &&
       !( FLEXCANb_IMASK1(_bus) & FLEXCAN_IMASK1_BUF5M ) &&
       ( FLEXCANb_IFLAG1(_bus) & FLEXCAN_IFLAG1_BUF5I ) ) return readFIFO(msg);
  return readMB(msg);
}

FCTP_FUNC int FCTP_OPT::readMB(CAN_message_t &msg) {
  uint8_t cycle_count = 0, mailboxes = mailboxOffset();

rescan_rx_mbs:

  if ( FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN ) {  /* FIFO is enabled, get only remaining RX (if any) */
    if ( mailbox_reader_increment < mailboxes ) mailbox_reader_increment = mailboxes - 1; /* go back to position end of fifo+filter region */
  }

  if ( ++mailbox_reader_increment >= FLEXCANb_MAXMB_SIZE(_bus) ) {
    mailbox_reader_increment = 0;
    if ( ++cycle_count > FLEXCANb_MAXMB_SIZE(_bus) ) return 0; /* if cycles are greater than number of mailboxes */
    if ( FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN ) goto rescan_rx_mbs; /* FIFO enabled? offset mailbox.. */
  }

  if (readIMASK() & (1ULL << mailbox_reader_increment)) { /* don't read interrupt enabled mailboxes */
    if ( ++cycle_count > FLEXCANb_MAXMB_SIZE(_bus) ) return 0; /* if cycles are greater than number of mailboxes */
    goto rescan_rx_mbs;
  }
  uint32_t code = FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, mailbox_reader_increment)); // Reading Control Status atomically locks mailbox.
  switch ( code ) {
    case FLEXCAN_MB_CODE_RX_FULL:           // rx full, Copy the frame to RX buffer
    case FLEXCAN_MB_CODE_RX_OVERRUN: {      // rx overrun. Incomming frame overwrote existing frame.
        FLEXCANb_MBn_CS(_bus, mailbox_reader_increment) |= FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_INACTIVE); /* deactivate mailbox */
        if ( FLEXCAN_MB_CODE_RX_OVERRUN == code ) msg.flags.overrun = 1;
        msg.len = FLEXCAN_get_length(FLEXCANb_MBn_CS(_bus, mailbox_reader_increment));
        msg.flags.extended = (FLEXCANb_MBn_CS(_bus, mailbox_reader_increment) & FLEXCAN_MB_CS_IDE) ? 1 : 0;
        msg.flags.remote = (FLEXCANb_MBn_CS(_bus, mailbox_reader_increment) & FLEXCAN_MB_CS_RTR) ? 1 : 0;
        msg.timestamp = FLEXCAN_get_timestamp(FLEXCANb_MBn_CS(_bus, mailbox_reader_increment));
        msg.id = (FLEXCANb_MBn_ID(_bus, mailbox_reader_increment) & FLEXCAN_MB_ID_EXT_MASK);
        if (!msg.flags.extended) msg.id >>= FLEXCAN_MB_ID_STD_BIT_NO;
        uint32_t dataIn = FLEXCANb_MBn_WORD0(_bus, mailbox_reader_increment);
        msg.buf[0] = dataIn >> 24; msg.buf[1] = dataIn >> 16; msg.buf[2] = dataIn >> 8; msg.buf[3] = dataIn;
        dataIn = FLEXCANb_MBn_WORD1(_bus, mailbox_reader_increment);
        msg.buf[4] = dataIn >> 24; msg.buf[5] = dataIn >> 16; msg.buf[6] = dataIn >> 8; msg.buf[7] = dataIn;
        msg.mb = mailbox_reader_increment; /* store the mailbox the message came from (for callback reference) */
        msg.bus = busNumber;
        if (!msg.flags.extended) FLEXCANb_MBn_CS(_bus, mailbox_reader_increment) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY);
        else FLEXCANb_MBn_CS(_bus, mailbox_reader_increment) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE;
        writeIFLAGBit(mailbox_reader_increment); /* immediately flush interrupt flag of current mailbox */
        FLEXCANb_TIMER(_bus); // reading timer unlocks individual mailbox
        return 1; /* we got a frame, exit */
      }
    case FLEXCAN_MB_CODE_TX_INACTIVE: {       // TX inactive. Just chillin' waiting for a message to send.
        goto rescan_rx_mbs;
      }
    case FLEXCAN_MB_CODE_RX_BUSY:           // mailbox is busy, check it later.
    case FLEXCAN_MB_CODE_RX_INACTIVE:       // inactive Receive box. Must be a false alarm!?
    case FLEXCAN_MB_CODE_RX_EMPTY:          // rx empty already. Why did it interrupt then?
    case FLEXCAN_MB_CODE_TX_ABORT:          // TX being aborted.
    case FLEXCAN_MB_CODE_TX_RESPONSE:       // remote request response (deprecated)
    case FLEXCAN_MB_CODE_TX_ONCE:           // TX mailbox is full and will be sent as soon as possible
    case FLEXCAN_MB_CODE_TX_RESPONSE_TEMPO: // remote request junk again. Go away.
    default:
      break;
  }
  return 0; /* no messages available */
}

FCTP_FUNC void FCTP_OPT::struct2queueTx(const CAN_message_t &msg) {
  uint8_t buf[sizeof(CAN_message_t)];
  memmove(buf, &msg, sizeof(msg));
  txBuffer.push_back(buf, sizeof(CAN_message_t));
}

FCTP_FUNC int FCTP_OPT::write(FLEXCAN_MAILBOX mb_num, const CAN_message_t &msg) {
  if ( mb_num < mailboxOffset() ) return 0; /* FIFO doesn't transmit */
  if ( !((FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, mb_num))) >> 3) ) return 0; /* not a transmit mailbox */
  uint32_t timeout = millis();
  while ( FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, mb_num)) != FLEXCAN_MB_CODE_TX_INACTIVE ) {
    if ( millis() - timeout > 100 ) return 0;
  }
  bool flag_set = 0;
  if ( (readIMASK() & (1ULL << mb_num)) ) {
    writeIMASKBit(mb_num, 0);
    flag_set = 1;
  }
  writeTxMailbox(mb_num, msg);
  if ( flag_set ) writeIMASKBit(mb_num, 1);
  return 1; // transmit entry accepted //
}

FCTP_FUNC int FCTP_OPT::write(const CAN_message_t &msg) {
  CAN_message_t frame = msg;
  frame.bus = busNumber;
  if ( frame.seq ) frame.mb = getFirstTxBox();
  else {
    for (uint8_t i = mailboxOffset(); i < FLEXCANb_MAXMB_SIZE(_bus); i++) {
      if ( FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, i)) == FLEXCAN_MB_CODE_TX_INACTIVE ) {
        bool flag_set = 0;
        if ( (readIMASK() & (1ULL << i)) ) {
          writeIMASKBit(i, 0);
          flag_set = 1;
        }
        writeTxMailbox(i, msg);
        if ( flag_set ) writeIMASKBit(i, 1);
        return 1; /* transmit entry accepted */
      }
    }
  }
  if ( txBuffer.size() < txBuffer.capacity() ) {
    struct2queueTx(frame);
    return 2;
  }
  return 0; /* transmit entry failed, no mailboxes or queues available */
}

FCTP_FUNC void FCTP_OPT::onReceive(const FLEXCAN_MAILBOX &mb_num, _MB_ptr handler) {
  if ( FIFO == mb_num ) {
    _mbHandlers[0] = handler;
    return;
  }
  _mbHandlers[mb_num] = handler;
}

FCTP_FUNC void FCTP_OPT::onReceive(_MB_ptr handler) {
  _mainHandler = handler;
}

FCTP_FUNC void FCTP_OPT::events() {
  if ( rxBuffer.size() ) {
    CAN_message_t frame;
    uint8_t buf[sizeof(CAN_message_t)];
    rxBuffer.pop_front(buf, sizeof(CAN_message_t));
    memmove(&frame, buf, sizeof(frame));
    mbCallbacks((FLEXCAN_MAILBOX)frame.mb, frame);
  }
}

void flexcan_isr_can1() {
  if ( _CAN1 ) _CAN1->flexcan_interrupt();
}

void flexcan_isr_can2() {
  if ( _CAN2 ) _CAN2->flexcan_interrupt();
}

void flexcan_isr_can3() {
  if ( _CAN3 ) _CAN3->flexcan_interrupt();
}

FCTP_FUNC void FCTP_OPT::mbCallbacks(const FLEXCAN_MAILBOX &mb_num, const CAN_message_t &msg) {
  if ( mb_num == FIFO ) {
    if ( _mbHandlers[0] ) _mbHandlers[0](msg);
    return;
  }
  if ( _mbHandlers[mb_num] ) _mbHandlers[mb_num](msg);
}

FCTP_FUNC void FCTP_OPT::struct2queueRx(const CAN_message_t &msg) {
  uint8_t buf[sizeof(CAN_message_t)];
  memmove(buf, &msg, sizeof(msg));
  rxBuffer.push_back(buf, sizeof(CAN_message_t));
}

FCTP_FUNC void FCTP_OPT::flexcan_interrupt() {
  CAN_message_t msg; // setup a temporary storage buffer
  uint64_t status = readIFLAG();

  if ( (FLEXCANb_IMASK1(_bus) & FLEXCAN_IMASK1_BUF5M) && (status & FLEXCAN_IFLAG1_BUF5I) && (FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN) ) { /* FIFO is enabled, capture frames if triggered */
    while ( FLEXCANb_IFLAG1(_bus) & FLEXCAN_IFLAG1_BUF5I ) {
      if ( FLEXCANb_IMASK1(_bus) & FLEXCAN_IMASK1_BUF5M ) {
        msg.len = FLEXCAN_get_length(FLEXCANb_MBn_CS(_bus, 0));
        msg.flags.extended = (FLEXCANb_MBn_CS(_bus, 0) & FLEXCAN_MB_CS_IDE) ? 1 : 0;
        msg.flags.remote = (FLEXCANb_MBn_CS(_bus, 0) & FLEXCAN_MB_CS_RTR) ? 1 : 0;
        msg.timestamp = FLEXCAN_get_timestamp (FLEXCANb_MBn_CS(_bus, 0));
        msg.id = (FLEXCANb_MBn_ID(_bus, 0) & FLEXCAN_MB_ID_EXT_MASK);
        if (!msg.flags.extended) msg.id >>= FLEXCAN_MB_ID_STD_BIT_NO;
        for ( uint8_t i = 0; i < 8; i++ ) (i < 4) ? msg.buf[3 - i] = FLEXCANb_MBn_WORD0(_bus, 0) >> (i * 8) : msg.buf[11 - i] = FLEXCANb_MBn_WORD1(_bus, 0) >> ((i - 4) * 8);
        msg.bus = busNumber;
        FLEXCANb_IFLAG1(_bus) = FLEXCAN_IFLAG1_BUF5I; /* clear FIFO bit only! */
        struct2queueRx(msg);
      }
      FLEXCANb_IFLAG1(_bus) = FLEXCAN_IFLAG1_BUF5I; /* clear FIFO bit only! */
    }
    if ( status & FLEXCAN_IFLAG1_BUF6I ) {
      FLEXCANb_IFLAG1(_bus) = FLEXCAN_IFLAG1_BUF6I;
      status &= ~FLEXCAN_IFLAG1_BUF6I; /* remove bit from initial flag lookup so it's not set at end when another frame is captured */
    }
    if ( status & FLEXCAN_IFLAG1_BUF7I ) {
      FLEXCANb_IFLAG1(_bus) = FLEXCAN_IFLAG1_BUF7I;
      status &= ~FLEXCAN_IFLAG1_BUF7I; /* remove bit from initial flag lookup so it's not set at end when another frame is captured */
    }
    status &= ~FLEXCAN_IFLAG1_BUF5I; /* remove bit from initial flag lookup so it's not set at end when another frame is captured */
  }


  /* mailbox handling routine */

  for (uint8_t i = mailboxOffset(); i < FLEXCANb_MAXMB_SIZE(_bus); i++) {

    // TRANSMISSIONS
    uint32_t code = FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, i)); // Reading Control Status atomically locks mailbox.
    if (!(status & (1ULL << i))) continue; // skip mailboxes that haven't triggered an interrupt
    switch ( code ) {
      case FLEXCAN_MB_CODE_TX_INACTIVE: {     // TX inactive. Dequeue if available
          if ( txBuffer.size() ) {
            CAN_message_t queueToSend;
            uint8_t buf[sizeof(CAN_message_t)];
            txBuffer.peek_front(buf, sizeof(CAN_message_t));
            memmove(&queueToSend, buf, sizeof(queueToSend));
            if ( queueToSend.seq ) {
              if ( queueToSend.mb == i ) {
                txBuffer.read();
                writeTxMailbox(i,queueToSend);
              }
            }
            else {
              txBuffer.pop_front(buf, sizeof(CAN_message_t));
              writeTxMailbox(i,queueToSend);
            }
          }
          continue;
        }
      case FLEXCAN_MB_CODE_TX_RESPONSE: {       // remote request response (deprecated)
          continue;
        }
      case FLEXCAN_MB_CODE_TX_RESPONSE_TEMPO: { // remote request junk again. Go away.
          continue;
        }
      case FLEXCAN_MB_CODE_TX_ABORT: {          // TX being aborted.
          continue;
        }
      case FLEXCAN_MB_CODE_TX_ONCE: {           // TX mailbox is full and will be sent as soon as possible
          continue;
        }
    }

    // RECEPTIONS
    if (!(readIMASK() & ( 1ULL << i ))) continue; // skip mailboxes that don't have interrupts enabled
    if (!(status & (1ULL << i))) continue; // skip mailboxes that haven't triggered an interrupt
    code = FLEXCAN_get_code(FLEXCANb_MBn_CS(_bus, i)); // Reading Control Status atomically locks mailbox.
    switch ( code ) {
      case FLEXCAN_MB_CODE_RX_FULL:           // rx full, Copy the frame to RX buffer
      case FLEXCAN_MB_CODE_RX_OVERRUN: {      // rx overrun. Incomming frame overwrote existing frame.
          if ( FLEXCAN_MB_CODE_RX_OVERRUN == code ) msg.flags.overrun = 1;
          msg.len = FLEXCAN_get_length(FLEXCANb_MBn_CS(_bus, i));
          msg.flags.extended = (FLEXCANb_MBn_CS(_bus, i) & FLEXCAN_MB_CS_IDE) ? 1 : 0;
          msg.flags.remote = (FLEXCANb_MBn_CS(_bus, i) & FLEXCAN_MB_CS_RTR) ? 1 : 0;
          msg.timestamp = FLEXCAN_get_timestamp (FLEXCANb_MBn_CS(_bus, i));
          msg.id = (FLEXCANb_MBn_ID(_bus, i) & FLEXCAN_MB_ID_EXT_MASK);
          if (!msg.flags.extended) msg.id >>= FLEXCAN_MB_ID_STD_BIT_NO;
          for ( uint8_t c = 0; c < 8; c++ ) (c < 4) ? msg.buf[3 - c] = FLEXCANb_MBn_WORD0(_bus, i) >> (c * 8) : msg.buf[11 - c] = FLEXCANb_MBn_WORD1(_bus, i) >> ((c - 4) * 8);
          msg.mb = i; /* store the mailbox the message came from (for callback reference) */
          msg.bus = busNumber;
          struct2queueRx(msg);
          if (!msg.flags.extended) FLEXCANb_MBn_CS(_bus, i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY);
          else FLEXCANb_MBn_CS(_bus, i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE;
          writeIFLAGBit(i); /* immediately flush interrupt flag of current mailbox */
          FLEXCANb_TIMER(_bus); // reading timer unlocks individual mailbox
          status &= ~(1ULL << i); /* remove bit from initial flag lookup so it's not set at end when another frame is captured */
          continue;
        }

      case FLEXCAN_MB_CODE_RX_BUSY:           // mailbox is busy, check it later.
      case FLEXCAN_MB_CODE_RX_INACTIVE:       // inactive Receive box. Must be a false alarm!?
      case FLEXCAN_MB_CODE_RX_EMPTY:          // rx empty already. Why did it interrupt then?
        continue;
    }
  }
  writeIFLAG(status);
  FLEXCANb_ESR1(_bus) |= FLEXCANb_ESR1(_bus);
}

FCTP_FUNC void FCTP_OPT::setRFFN(FLEXCAN_RFFN_TABLE rffn) {
  bool frz_flag_negate = 0;
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK) ) { // currently not in freeze mode
    frz_flag_negate = 1; FLEXCAN_EnterFreezeMode();
  }
  FLEXCAN_set_rffn(FLEXCANb_CTRL2(_bus), rffn); 
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
}

FCTP_FUNC void FCTP_OPT::setFIFOFilter(const FLEXCAN_FLTEN &input) {
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN )) return; /* FIFO not enabled. */
  uint8_t max_fifo_filters = (((FLEXCANb_CTRL2(_bus) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 8; // 8->128
  bool frz_flag_negate = 0;
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK) ) { // currently not in freeze mode
    frz_flag_negate = 1; FLEXCAN_EnterFreezeMode();
  }
  for (uint8_t i = 0; i < max_fifo_filters; i++) { /* block all ID's so filtering could be applied. */
    if ( input == REJECT_ALL ) {
      if ( ((FLEXCANb_MCR(_bus) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 0 ) { /* If Table A is chosen for FIFO */
        FLEXCANb_IDFLT_TAB(_bus, i) = 0xFFFFFFFF; /* reset id */
        /* individual masks (RXIMR) will just cover Rx FIFO filters in 0-31 range, and filters 32-127
           will use RXFGMASK. */ 
        if ( i < 32 ) FLEXCANb_RXIMR(_bus, i) = 0x3FFFFFFF; // (RXIMR) /* block all id's (0-31) */
        else {
          FLEXCANb_RXFGMASK(_bus) = 0x3FFFFFFF;
        }
      }
      else if ( ((FLEXCANb_MCR(_bus) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 1 ) { /* If Table B is chosen for FIFO */
        FLEXCANb_IDFLT_TAB(_bus, i) = 0xFFFFFFFF; /* reset id */
        if ( i < 32 ) FLEXCANb_RXIMR(_bus, i) = 0x7FFF7FFF; // (RXIMR) /* block all id's (0-31) */
        else FLEXCANb_RXFGMASK(_bus) = 0x7FFF7FFF;
      }
      else if ( ((FLEXCANb_MCR(_bus) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 2 ) { /* If Table C is chosen for FIFO */
       /* TO BE DONE */ //FLEXCANb_IDFLT_TAB(_bus, i) = 0x6E6E6E6E; /* reset id */
        //FLEXCANb_RXIMR(_bus, i) = 0xFFFFFFFF; // (RXIMR) /* block all id's */
      }
    }
    else if ( input == ACCEPT_ALL ) {
      FLEXCANb_IDFLT_TAB(_bus, i) = 0; /* reset id */
      if ( i < 32 ) FLEXCANb_RXIMR(_bus, i) = 0; // (RXIMR) /* allow all id's */
      else FLEXCANb_RXFGMASK(_bus) = 0; /* for masks above IDF 0->31, global is used for rest) */
    }
  }
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
}

FCTP_FUNC bool FCTP_OPT::setFIFOFilter(uint8_t filter, uint32_t id1, const FLEXCAN_IDE &ide, const FLEXCAN_IDE &remote) {
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN )) return 0; /* FIFO not enabled. */
  uint8_t max_fifo_filters = (((FLEXCANb_CTRL2(_bus) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 8; // 8->128
  if ( filter >= max_fifo_filters ) return 0;
  bool frz_flag_negate = 0;
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK) ) { // currently not in freeze mode
    frz_flag_negate = 1; FLEXCAN_EnterFreezeMode();
  }
  /* ##################################### TABLE A ###################################### */
  if ( ((FLEXCANb_MCR(_bus) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 0 ) {
    uint32_t mask = ( ide != EXT ) ? ((((id1) ^ (id1)) ^ 0x7FF) << 19 ) | 0xC0000001 : ((((id1) ^ (id1)) ^ 0x1FFFFFFF) << 1 ) | 0xC0000001;
    FLEXCANb_IDFLT_TAB(_bus, filter) = ((ide == EXT ? 1 : 0) << 30) | ((remote == RTR ? 1 : 0) << 31) |
        ((ide == EXT ? ((id1 & FLEXCAN_MB_ID_EXT_MASK) << 1) : (FLEXCAN_MB_ID_IDSTD(id1) << 1)));
    if ( filter < 32 ) FLEXCANb_RXIMR(_bus, filter) = mask; // (RXIMR)
    else FLEXCANb_RXFGMASK(_bus) = 0x3FFFFFFF; /* enforce it for blocks 32->127, single IDs */
  }
  /* #################################################################################### */
  /* ##################################### TABLE B ###################################### */
  if ( ((FLEXCANb_MCR(_bus) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 1 ) {
    FLEXCANb_IDFLT_TAB(_bus, filter) = ((ide == EXT ? 1 : 0) << 30) | ((ide == EXT ? 1 : 0) << 14) | /* STD IDs / EXT IDs */
        ((remote == RTR ? 1 : 0) << 31) | ((remote == RTR ? 1 : 0) << 15) | /* remote frames */
        (ide == EXT ? ((id1 >> (29 - 14)) << 16) : ((id1 & 0x7FF) << 19)) | /* first ID is EXT or STD? */
        (ide == EXT ? ((id1 >> (29 - 14)) <<  0) : ((id1 & 0x7FF) <<  3)) ; /* second ID is EXT or STD? */
    uint32_t mask = ( ide != EXT ) ? ((((id1) ^ (id1)) ^ 0x7FF) << 19 ) | ((remote == RTR)?(1UL<<31):0UL) : ((((id1) ^ (id1)) ^ 0x1FFFFFFF) << 16 ) | ((remote == RTR)?(1UL<<31):0UL);
    mask |= (( ide != EXT ) ? ((((id1) ^ (id1)) ^ 0x7FF) << 3 )  | ((remote == RTR)?(1UL<<15):0UL) : ((((id1) ^ (id1)) ^ 0x1FFFFFFF) << 0 )  | ((remote == RTR)?(1UL<<15):0UL) ) & 0xFFFF;
    mask |= (1UL<<30) | (1UL<<14);
    if ( filter < 32 ) FLEXCANb_RXIMR(_bus, filter) = mask; // (RXIMR)
    else FLEXCANb_RXFGMASK(_bus) = 0x7FFF7FFF; /* enforce it for blocks 32->127, single STD IDs / EXT partial matches */
  }
  /* #################################################################################### */
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
  return 1;
}

FCTP_FUNC bool FCTP_OPT::setFIFOFilter(uint8_t filter, uint32_t id1, uint32_t id2, const FLEXCAN_IDE &ide, const FLEXCAN_IDE &remote) {
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN )) return 0; /* FIFO not enabled. */
  if ( filter > 31 ) return 0; /* multi-id & ranges are not allowed */
  uint8_t max_fifo_filters = (((FLEXCANb_CTRL2(_bus) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 8; // 8->128
  if ( filter >= max_fifo_filters ) return 0;
  bool frz_flag_negate = 0;
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK) ) { // currently not in freeze mode
    frz_flag_negate = 1; FLEXCAN_EnterFreezeMode();
  }
  /* ##################################### TABLE A ###################################### */
  if ( ((FLEXCANb_MCR(_bus) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 0 ) {
    uint32_t mask = ( ide != EXT ) ? ((((id1 | id2) ^ (id1 & id2)) ^ 0x7FF) << 19 ) | 0xC0000001 : ((((id1 | id2) ^ (id1 & id2)) ^ 0x1FFFFFFF) << 1 ) | 0xC0000001;
    FLEXCANb_RXIMR(_bus, filter) = mask; // (RXIMR)
    FLEXCANb_IDFLT_TAB(_bus, filter) = ((ide == EXT ? 1 : 0) << 30) | ((remote == RTR ? 1 : 0) << 31) |
        ((ide == EXT ? ((id1 & FLEXCAN_MB_ID_EXT_MASK) << 1) : (FLEXCAN_MB_ID_IDSTD(id1) << 1)));
  }
  /* #################################################################################### */
  /* ##################################### TABLE B ###################################### */
  if ( ((FLEXCANb_MCR(_bus) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 1 ) {
    FLEXCANb_IDFLT_TAB(_bus, filter) = ((ide == EXT ? 1 : 0) << 30) | ((ide == EXT ? 1 : 0) << 14) | /* STD IDs / EXT IDs */
        ((remote == RTR ? 1 : 0) << 31) | ((remote == RTR ? 1 : 0) << 15) | /* remote frames */
        (ide == EXT ? ((id1 >> (29 - 14)) << 16) : ((id1 & 0x7FF) << 19)) | /* first ID is EXT or STD? */
        (ide == EXT ? ((id2 >> (29 - 14)) <<  0) : ((id2 & 0x7FF) <<  3)) ; /* second ID is EXT or STD? */
    uint32_t mask = ( ide != EXT ) ? ((((id1) ^ (id1)) ^ 0x7FF) << 19 ) | ((remote == RTR)?(1UL<<31):0UL) : ((((id1) ^ (id1)) ^ 0x1FFFFFFF) << 16 ) | ((remote == RTR)?(1UL<<31):0UL);
    mask |= (( ide != EXT ) ? ((((id2) ^ (id2)) ^ 0x7FF) << 3 )  | ((remote == RTR)?(1UL<<15):0UL) : ((((id2) ^ (id2)) ^ 0x1FFFFFFF) << 0 )  | ((remote == RTR)?(1UL<<15):0UL) ) & 0xFFFF;
    mask |= (1UL<<30) | (1UL<<14);
    FLEXCANb_RXIMR(_bus, filter) = mask; // (RXIMR)
  }
  /* #################################################################################### */
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
  return 1;
}

FCTP_FUNC bool FCTP_OPT::setFIFOFilterRange(uint8_t filter, uint32_t id1, uint32_t id2, const FLEXCAN_IDE &ide, const FLEXCAN_IDE &remote) {
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN )) return 0; /* FIFO not enabled. */
  if ( filter > 31 ) return 0; /* multi-id & ranges are not allowed */
  uint8_t max_fifo_filters = (((FLEXCANb_CTRL2(_bus) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 8; // 8->128
  if ( filter >= max_fifo_filters ) return 0;
  if ( id1 > id2 || ((id2 > id1) && (id2 - id1 > 1000)) || !id1 || !id2 ) return 0; /* don't play around... */
  bool frz_flag_negate = 0;
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK) ) { // currently not in freeze mode
    frz_flag_negate = 1; FLEXCAN_EnterFreezeMode();
  }
  /* ##################################### TABLE A ###################################### */
  if ( ((FLEXCANb_MCR(_bus) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 0 ) {
    uint32_t stage1 = id1, stage2 = id1;
    for ( uint32_t i = id1 + 1; i <= id2; i++ ) {
      stage1 |= i; stage2 &= i;
    }
    uint32_t mask = ( ide != EXT ) ? (((stage1 ^ stage2) ^ 0x7FF) << 19) | 0xC0000001 : (((stage1 ^ stage2) ^ 0x1FFFFFFF) << 1) | 0xC0000001;
    FLEXCANb_RXIMR(_bus, filter) = mask; // (RXIMR)
    FLEXCANb_IDFLT_TAB(_bus, filter) = ((ide == EXT ? 1 : 0) << 30) | ((remote == RTR ? 1 : 0) << 31) |
        ((ide == EXT ? ((id1 & FLEXCAN_MB_ID_EXT_MASK) << 1) : (FLEXCAN_MB_ID_IDSTD(id1) << 1)));
  }
  /* #################################################################################### */
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
  return 1;
}

FCTP_FUNC bool FCTP_OPT::setFIFOFilter(uint8_t filter, uint32_t id1, const FLEXCAN_IDE &ide1, const FLEXCAN_IDE &remote1, uint32_t id2, const FLEXCAN_IDE &ide2, const FLEXCAN_IDE &remote2) {
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN )) return 0; /* FIFO not enabled. */
  if ( filter > 31 ) return 0; /* multi-id & ranges are not allowed */
  uint8_t max_fifo_filters = (((FLEXCANb_CTRL2(_bus) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 8; // 8->128
  if ( filter >= max_fifo_filters ) return 0;
  bool frz_flag_negate = 0;
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK) ) { // currently not in freeze mode
    frz_flag_negate = 1; FLEXCAN_EnterFreezeMode();
  }
  /* ##################################### TABLE B ###################################### */
  if ( ((FLEXCANb_MCR(_bus) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 1 ) {
    FLEXCANb_IDFLT_TAB(_bus, filter) = ((ide1 == EXT ? 1 : 0) << 30) | ((ide2 == EXT ? 1 : 0) << 14) | /* STD IDs / EXT IDs */
        ((remote1 == RTR ? 1 : 0) << 31) | ((remote2 == RTR ? 1 : 0) << 15) | /* remote frames */
        (ide1 == EXT ? ((id1 >> (29 - 14)) << 16) : ((id1 & 0x7FF) << 19)) | /* first ID is EXT or STD? */
        (ide2 == EXT ? ((id2 >> (29 - 14)) <<  0) : ((id2 & 0x7FF) <<  3)) ; /* second ID is EXT or STD? */
    uint32_t mask = ( ide1 != EXT ) ? ((((id1) ^ (id1)) ^ 0x7FF) << 19 ) | ((remote1 == RTR)?(1UL<<31):0UL) : ((((id1) ^ (id1)) ^ 0x1FFFFFFF) << 16 ) | ((remote1 == RTR)?(1UL<<31):0UL);
    mask |= (( ide2 != EXT ) ? ((((id2) ^ (id2)) ^ 0x7FF) << 3 )  | ((remote2 == RTR)?(1UL<<15):0UL) : ((((id2) ^ (id2)) ^ 0x1FFFFFFF) << 0 )  | ((remote2 == RTR)?(1UL<<15):0UL) ) & 0xFFFF;
    mask |= (1UL<<30) | (1UL<<14);
    FLEXCANb_RXIMR(_bus, filter) = mask; // (RXIMR)
  }
  /* #################################################################################### */
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
  return 1;
}

FCTP_FUNC bool FCTP_OPT::setFIFOFilter(uint8_t filter, uint32_t id1, uint32_t id2, const FLEXCAN_IDE &ide1, const FLEXCAN_IDE &remote1, uint32_t id3, uint32_t id4, const FLEXCAN_IDE &ide2, const FLEXCAN_IDE &remote2) {
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN )) return 0; /* FIFO not enabled. */
  if ( filter > 31 ) return 0; /* multi-id & ranges are not allowed */
  uint8_t max_fifo_filters = (((FLEXCANb_CTRL2(_bus) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 8; // 8->128
  if ( filter >= max_fifo_filters ) return 0;
  bool frz_flag_negate = 0;
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK) ) { // currently not in freeze mode
    frz_flag_negate = 1; FLEXCAN_EnterFreezeMode();
  }
  /* ##################################### TABLE B ###################################### */
  if ( ((FLEXCANb_MCR(_bus) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 1 ) {
    uint32_t mask = ( ide1 != EXT ) ? ((((id1 | id2) ^ (id1 & id2)) ^ 0x7FF) << 19 ) | ((remote1 == RTR)?(1UL<<31):0UL) : ((((id1 | id2) ^ (id1 & id2)) ^ 0x1FFFFFFF) << 16 ) | ((remote1 == RTR)?(1UL<<31):0UL);
    mask |= (( ide2 != EXT ) ? ((((id3 | id4) ^ (id3 & id4)) ^ 0x7FF) << 3 )  | ((remote2 == RTR)?(1UL<<15):0UL) : ((((id3 | id4) ^ (id3 & id4)) ^ 0x1FFFFFFF) << 0 )  | ((remote2 == RTR)?(1UL<<15):0UL) ) & 0xFFFF;
    mask |= (1UL<<30) | (1UL<<14);
    FLEXCANb_IDFLT_TAB(_bus, filter) = ((ide1 == EXT ? 1 : 0) << 30) | ((ide2 == EXT ? 1 : 0) << 14) | /* STD IDs / EXT IDs */
        ((remote1 == RTR ? 1 : 0) << 31) | ((remote2 == RTR ? 1 : 0) << 15) | /* remote frames */
        (ide1 == EXT ? ((id1 >> (29 - 14)) << 16) : ((id1 & 0x7FF) << 19)) | /* first ID is EXT or STD? */
        (ide2 == EXT ? ((id3 >> (29 - 14)) << 0 ) : ((id3 & 0x7FF) << 3 ))  ; /* second ID is EXT or STD? */
    FLEXCANb_RXIMR(_bus, filter) = mask; // (RXIMR)
  }
  /* #################################################################################### */
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
  return 1;
}

FCTP_FUNC bool FCTP_OPT::setFIFOFilterRange(uint8_t filter, uint32_t id1, uint32_t id2, const FLEXCAN_IDE &ide1, const FLEXCAN_IDE &remote1, uint32_t id3, uint32_t id4, const FLEXCAN_IDE &ide2, const FLEXCAN_IDE &remote2) {
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN )) return 0; /* FIFO not enabled. */
  if ( filter > 31 ) return 0; /* multi-id & ranges are not allowed */
  uint8_t max_fifo_filters = (((FLEXCANb_CTRL2(_bus) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xF) + 1) * 8; // 8->128
  if ( filter >= max_fifo_filters ) return 0;
  if ( id1 > id2 || ((id2 > id1) && (id2 - id1 > 1000)) || !id1 || !id2 ) return 0; /* don't play around... */
  if ( id3 > id4 || ((id4 > id3) && (id4 - id3 > 1000)) || !id3 || !id4 ) return 0; /* don't play around... */
  bool frz_flag_negate = 0;
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK) ) { // currently not in freeze mode
    frz_flag_negate = 1; FLEXCAN_EnterFreezeMode();
  }
  /* ##################################### TABLE B ###################################### */
  if ( ((FLEXCANb_MCR(_bus) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 1 ) {
    uint32_t stage1 = id1, stage2 = id1;
    for ( uint32_t i = id1 + 1; i <= id2; i++ ) {
      stage1 |= i; stage2 &= i;
    }
    uint32_t mask = ( ide1 != EXT ) ? ((((stage1 | stage2) ^ (stage1 & stage2)) ^ 0x7FF) << 19 ) | ((remote1 == RTR)?(1UL<<31):0UL) : ((((stage1 | stage2) ^ (stage1 & stage2)) ^ 0x1FFFFFFF) << 16 ) | ((remote1 == RTR)?(1UL<<31):0UL);
    stage1 = stage2 = id3;
    for ( uint32_t i = id3 + 1; i <= id4; i++ ) {
      stage1 |= i; stage2 &= i;
    }
    mask |= (( ide2 != EXT ) ? ((((stage1 | stage2) ^ (stage1 & stage2)) ^ 0x7FF) << 3 )  | ((remote2 == RTR)?(1UL<<15):0UL) : ((((stage1 | stage2) ^ (stage1 & stage2)) ^ 0x1FFFFFFF) << 0 ) | ((remote2 == RTR)?(1UL<<15):0UL) ) & 0xFFFF;
    mask |= (1UL<<30) | (1UL<<14);
    FLEXCANb_IDFLT_TAB(_bus, filter) = ((ide1 == EXT ? 1 : 0) << 30) | ((ide2 == EXT ? 1 : 0) << 14) | /* STD IDs / EXT IDs */
        ((remote1 == RTR ? 1 : 0) << 31) | ((remote2 == RTR ? 1 : 0) << 15) | /* remote frames */
        (ide1 == EXT ? ((id1 >> (29 - 14)) << 16) : ((id1 & 0x7FF) << 19)) | /* first ID is EXT or STD? */
        (ide2 == EXT ? ((id3 >> (29 - 14)) << 0 ) : ((id3 & 0x7FF) << 3 ))  ; /* second ID is EXT or STD? */
    FLEXCANb_RXIMR(_bus, filter) = mask; // (RXIMR)
  }
  /* #################################################################################### */
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
  return 1;
}

FCTP_FUNC void FCTP_OPT::setFIFOFilterTable(FLEXCAN_FIFOTABLE letter) {
  bool frz_flag_negate = 0;
  if ( !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK) ) { // currently not in freeze mode
    frz_flag_negate = 1; FLEXCAN_EnterFreezeMode();
  }
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_IDAM(letter);
  if ( ((FLEXCANb_MCR(_bus) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 0 ) FLEXCANb_RXFGMASK(_bus) = 0x3FFFFFFF;
  else if ( ((FLEXCANb_MCR(_bus) & FLEXCAN_MCR_IDAM_MASK) >> FLEXCAN_MCR_IDAM_BIT_NO) == 1 ) FLEXCANb_RXFGMASK(_bus) = 0x7FFF7FFF;
  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
}
