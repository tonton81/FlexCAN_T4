#include <FlexCAN_T4.h>
#include "imxrt_flexcan.h"
#include "Arduino.h"

FCTPFD_FUNC static uint8_t flexcan_memory_regions = 2;
void flexcan_isr_can3fd();

FCTPFD_FUNC FCTPFD_OPT::FlexCAN_T4FD() {
  if ( _bus == CAN3 ) _CAN3 = this;
}

FCTPFD_FUNC void FCTPFD_OPT::setClock(FLEXCAN_CLOCK clock) {
  if ( clock == CLK_OFF ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(3) | CCM_CSCMR2_CAN_CLK_PODF(0);
  if ( clock == CLK_8MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(2) | CCM_CSCMR2_CAN_CLK_PODF(9);
  if ( clock == CLK_16MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(2) | CCM_CSCMR2_CAN_CLK_PODF(4);
  if ( clock == CLK_24MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(1) | CCM_CSCMR2_CAN_CLK_PODF(0);
  if ( clock == CLK_20MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(2) | CCM_CSCMR2_CAN_CLK_PODF(3);
  if ( clock == CLK_30MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(0) | CCM_CSCMR2_CAN_CLK_PODF(1);
  if ( clock == CLK_60MHz ) CCM_CSCMR2 = (CCM_CSCMR2 & 0xFFFFFC03) | CCM_CSCMR2_CAN_CLK_SEL(0) | CCM_CSCMR2_CAN_CLK_PODF(0);
  //if ( _CAN1 ) _CAN1->setBaudRate(currentBitrate);
  //if ( _CAN2 ) _CAN2->setBaudRate(currentBitrate);
  //if ( _CAN3 ) _CAN3->setBaudRate(currentBitrate);
}

FCTPFD_FUNC uint32_t FCTPFD_OPT::getClock() {
  uint8_t clockSrc = 0;
  if ( ((CCM_CSCMR2 & 0x300) >> 8) == 0 ) clockSrc = 60;
  if ( ((CCM_CSCMR2 & 0x300) >> 8) == 1 ) clockSrc = 24;
  if ( ((CCM_CSCMR2 & 0x300) >> 8) == 2 ) clockSrc = 80;
  return (clockSrc / (((CCM_CSCMR2 & 0xFC) >> 2) +1));
}

FCTPFD_FUNC void FCTPFD_OPT::begin() {
  setClock(CLK_20MHz);
  if ( _bus == CAN3 ) {
    nvicIrq = IRQ_CAN3;
    _VectorsRam[16 + nvicIrq] = flexcan_isr_can3fd;
    CCM_CCGR7 |= 0x3C0;
    busNumber = 3;
  }

  setTx(); setRx();

  FLEXCANb_MCR(_bus) &= ~FLEXCAN_MCR_MDIS; /* enable module */
  FLEXCAN_EnterFreezeMode();
  FLEXCANb_CTRL1(_bus) = /*FLEXCAN_CTRL_LOM |*/ (1UL << 5); /* listen only mode, TSYN */
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_FRZ; /* enable freeze bit */
  while (FLEXCANb_MCR(_bus) & FLEXCAN_MCR_LPM_ACK);
  softReset(); /* reset bus */
  while (!(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK));
  FLEXCANb_MCR(_bus) &= ~0x8000; // disable DMA
  FLEXCANb_MCR(_bus) |= (1UL << 16) | (1UL << 11) | (1UL << 17) | 0x3F; // IRMQ, FDEN, SRXDIS, 64MBs
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_AEN; // TX ABORT FEATURE
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_LPRIO_EN; // TX PRIORITY FEATURE
  FLEXCANb_CTRL2(_bus) |= FLEXCAN_CTRL2_RRS | // store remote frames
                                  FLEXCAN_CTRL2_EACEN | /* handles the way filtering works. Library adjusts to whether you use this or not */ 
                                  FLEXCAN_CTRL2_MRP; // mailbox > FIFO priority.
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_WRN_EN;
  FLEXCANb_MCR(_bus) |= (1UL << 26);

  // TEMPORARY SDK ENFORCED PARAMETERS FOR DEFAULT BITRATE. Needs Implementation....
  FLEXCANb_CBT(_bus) = 0x80210462;
  FLEXCANb_FDCBT(_bus) = 0x10463;
  FLEXCANb_FDCTRL(_bus) = 0x80008300; // MBDSR0-MBDSR1 = 0; 32MBs each as default, TDCEN, FDRATE(BRS)

  disableFIFO();

  FLEXCAN_ExitFreezeMode();
  NVIC_ENABLE_IRQ(nvicIrq);
}

FCTPFD_FUNC uint8_t FCTPFD_OPT::setRegions(uint8_t size) {
  FLEXCAN_EnterFreezeMode();
  FLEXCANb_FDCTRL(_bus) &= ~0x1B0000; /* clear both memory region bits */
  if ( constrain(size, 8, 64) == 8 ) {
    size = 64;
  }
  else if ( constrain(size, 16, 64) == 16 ) {
    FLEXCANb_FDCTRL(_bus) |= 0x90000;
    size = 42;
  }
  else if ( constrain(size, 32, 64) == 32 ) {
    FLEXCANb_FDCTRL(_bus) |= 0x120000;
    size = 24;
  }
  else {
    FLEXCANb_FDCTRL(_bus) |= 0x1B0000;
    size = 14;
  }
  disableFIFO();
  FLEXCAN_ExitFreezeMode();
  return size;
}





FCTPFD_FUNC uint32_t FCTPFD_OPT::mailbox_offset(uint8_t mailbox, uint8_t &maxsize) {
  const uint8_t shifts[4] = {0x10, 0x18, 0x28, 0x48};
  const uint8_t max_sizes[4] = {8, 16, 32, 64};
  uint8_t block0 = (FLEXCANb_FDCTRL(_bus) & (3UL << 16)) >> 16;
  uint8_t block1 = (FLEXCANb_FDCTRL(_bus) & (3UL << 19)) >> 19;
  if ( block0 == 0 ) {
    if ( mailbox <= 31 ) {
      maxsize = max_sizes[block0];
      return _bus + 0x80 + (0x10 * mailbox);
    }
    else {
      maxsize = max_sizes[block1];
      return _bus + 0x280 + (shifts[block1] * (mailbox - 32));
    }
  }
  if ( block0 == 1 ) {
    if ( mailbox <= 20 ) {
      maxsize = max_sizes[block0];
      return _bus + 0x80 + (0x18 * mailbox);
    }
    else {
      maxsize = max_sizes[block1];
      return _bus + 0x280 + (shifts[block1] * (mailbox - 21));
    }
  }
  if ( block0 == 2 ) {
    if ( mailbox <= 11 ) {
      maxsize = max_sizes[block0];
      return _bus + 0x80 + (0x28 * mailbox);
    }
    else {
      maxsize = max_sizes[block1];
      return _bus + 0x280 + (shifts[block1] * (mailbox - 12));
    }
  }
  if ( block0 == 3 ) {
    if ( mailbox <= 6 ) {
      maxsize = max_sizes[block0];
      return _bus + 0x80 + (0x48 * mailbox);
    }
    else {
      maxsize = max_sizes[block1];
      return _bus + 0x280 + (shifts[block1] * (mailbox - 7));
    }
  }
  return _bus + 0x80;
}

FCTPFD_FUNC int FCTPFD_OPT::getFirstTxBox() {
  int8_t block0 = (FLEXCANb_FDCTRL(_bus) & (3UL << 16)) >> 16;
  int8_t block1 = (FLEXCANb_FDCTRL(_bus) & (3UL << 19)) >> 19;
  uint8_t adjustment = 0;
  if ( block1 > block0 ) {
    if ( block0 == 0 ) adjustment = 32;
    else if ( block0 == 1 ) adjustment = 21;
    else if ( block0 == 2 ) adjustment = 12;
    else if ( block0 == 3 ) adjustment = 7;
  }
  for (uint8_t i = adjustment, mbsize = 0; i < max_mailboxes(); i++) {
    volatile uint32_t *mbxAddr = &(*(uint32_t*)(mailbox_offset(i, mbsize)));
    if ( (FLEXCAN_get_code(mbxAddr[0]) >> 3) ) return i; // if TX
  }
  return -1;
}

FCTPFD_FUNC uint8_t FCTPFD_OPT::max_mailboxes() {
  uint8_t mb_count = 0;
  uint8_t block0 = (FLEXCANb_FDCTRL(_bus) & (3UL << 16)) >> 16;
  uint8_t block1 = (FLEXCANb_FDCTRL(_bus) & (3UL << 19)) >> 19;
  if ( block0 == 0 ) mb_count = 32;
  else if ( block0 == 1 ) mb_count = 21;
  else if ( block0 == 2 ) mb_count = 12;
  else if ( block0 == 3 ) mb_count = 7;
  if ( block1 == 0 ) mb_count += 32;
  else if ( block1 == 1 ) mb_count += 21;
  else if ( block1 == 2 ) mb_count += 12;
  else if ( block1 == 3 ) mb_count += 7;
  if ( mb_count > ((FLEXCANb_MCR(_bus) & 0x3F) + 1) ) return ((FLEXCANb_MCR(_bus) & 0x3F) + 1);
  return mb_count;
}

FCTPFD_FUNC int FCTPFD_OPT::write(const CANFD_message_t &msg) {
  CANFD_message_t frame = msg;
  frame.bus = busNumber;
  if ( frame.seq ) frame.mb = getFirstTxBox();
  else {
    for (uint8_t i = 0, mbsize = 0; i < max_mailboxes(); i++) {
      volatile uint32_t *mbxAddr = &(*(uint32_t*)(mailbox_offset(i, mbsize)));
      if ( FLEXCAN_get_code(mbxAddr[0]) == FLEXCAN_MB_CODE_TX_INACTIVE ) {
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
    //struct2queueTx(frame);
    return 2;
  }
  return 0; /* transmit entry failed, no mailboxes or queues available */
}


FCTPFD_FUNC void FCTPFD_OPT::writeTxMailbox(uint8_t mb_num, const CANFD_message_t &frame) {
  CANFD_message_t msg = frame;
  writeIFLAGBit(mb_num);
  uint8_t mbsize = 0;
  volatile uint32_t *mbxAddr = &(*(uint32_t*)(mailbox_offset(mb_num, mbsize)));
  mbxAddr[0] = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  mbxAddr[1] = (( msg.flags.extended ) ? ( msg.id & FLEXCAN_MB_ID_EXT_MASK ) : FLEXCAN_MB_ID_IDSTD(msg.id));
  if ( msg.flags.extended ) mbxAddr[0] |= (3UL << 21);
  for ( uint8_t i = 0; i < (mbsize >> 2); i++ ) mbxAddr[2 + i] = (msg.buf[0 + i * 4] << 24) | (msg.buf[1 + i * 4] << 16) | (msg.buf[2 + i * 4] << 8) | msg.buf[3 + i * 4];
  if ( msg.len > mbsize ) msg.len = mbsize;
  if ( msg.len <= 8 ) mbxAddr[0] |= (uint32_t)msg.len << 16;
  else if ( msg.len <= 12 ) mbxAddr[0] |= 9UL << 16;
  else if ( msg.len <= 16 ) mbxAddr[0] |= 10UL << 16;
  else if ( msg.len <= 20 ) mbxAddr[0] |= 11UL << 16;
  else if ( msg.len <= 24 ) mbxAddr[0] |= 12UL << 16;
  else if ( msg.len <= 32 ) mbxAddr[0] |= 13UL << 16;
  else if ( msg.len <= 48 ) mbxAddr[0] |= 14UL << 16;
  else if ( msg.len <= 64 ) mbxAddr[0] |= 15UL << 16;
  mbxAddr[0] |= (1UL << 30); // BRS
  mbxAddr[0] |= (1UL << 31); // EDL
  mbxAddr[0] |= FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE);
}







FCTPFD_FUNC bool FCTPFD_OPT::setMB(const FLEXCAN_MAILBOX &mb_num, const FLEXCAN_RXTX &mb_rx_tx, const FLEXCAN_IDE &ide) {
  if ( mb_num >= max_mailboxes() ) return 0;
  writeIMASKBit(mb_num, 0); /* immediately disable mailbox interrupt */
  FLEXCAN_EnterFreezeMode();
  FLEXCANb_RXIMR(_bus, mb_num) = 0UL; // CLEAR MAILBOX MASK
  FLEXCAN_ExitFreezeMode();
  uint8_t mbsize = 0;
  volatile uint32_t *mbxAddr = &(*(uint32_t*)(mailbox_offset(mb_num , mbsize)));
  FLEXCAN_get_code(mbxAddr[0]); // Reading Control Status atomically locks mailbox (if it is RX mode).
  for ( uint8_t i = 0; i < (mbsize >> 2); i++ ) mbxAddr[2 + i] = 0x0; /* clear mailbox data */
  if ( ide == INACTIVE ) mbxAddr[0] = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_INACTIVE);
  else if ( mb_rx_tx == RX ) mbxAddr[0] = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | ((ide != EXT) ? 0 : FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE);
  else if ( mb_rx_tx == TX ) mbxAddr[0] = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  mbxAddr[1] = 0; // clear ID
  writeIFLAGBit(mb_num); /* clear mailbox flag */
  FLEXCANb_TIMER(_bus); /* reading timer unlocks individual mailbox */
  return 1;
}


















FCTPFD_FUNC int FCTPFD_OPT::read(CANFD_message_t &msg) {
  bool _random = random(0, 2);
//  if ( ( !_random ) && ( FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FEN ) &&
//       !( FLEXCANb_IMASK1(_bus) & FLEXCAN_IMASK1_BUF5M ) &&
//       ( FLEXCANb_IFLAG1(_bus) & FLEXCAN_IFLAG1_BUF5I ) ) return readFIFO(msg);
(void)_random;
  return readMB(msg);
}



FCTPFD_FUNC int FCTPFD_OPT::readMB(CANFD_message_t &msg) {
  uint8_t cycle_limit = 3;
  for ( uint8_t mbsize = 0; mailbox_reader_increment <= max_mailboxes(); ++mailbox_reader_increment ) {
    if ( mailbox_reader_increment >= max_mailboxes() ) {
      mailbox_reader_increment = 0;
      if ( !--cycle_limit ) return 0;
    }
    volatile uint32_t *mbxAddr = &(*(uint32_t*)(mailbox_offset(mailbox_reader_increment , mbsize)));
    if (readIMASK() & (1ULL << mailbox_reader_increment)) continue; /* don't read interrupt enabled mailboxes */
    if ( ( FLEXCAN_get_code(mbxAddr[0]) == FLEXCAN_MB_CODE_RX_FULL ) || ( FLEXCAN_get_code(mbxAddr[0]) == FLEXCAN_MB_CODE_RX_OVERRUN ) ) {
    msg.id = (mbxAddr[1] & 0x1FFFFFFF) >> ((msg.flags.extended) ? 0 : 18);
    msg.flags.extended = mbxAddr[0] & (1UL << 21);
    msg.len = (mbxAddr[0] & 0xF0000) >> 16;
    if ( msg.len <= 8 );
    else if ( msg.len == 9 ) msg.len = 12;
    else if ( msg.len == 10 ) msg.len = 16;
    else if ( msg.len == 11 ) msg.len = 20;
    else if ( msg.len == 12 ) msg.len = 24;
    else if ( msg.len == 13 ) msg.len = 32;
    else if ( msg.len == 14 ) msg.len = 48;
    else if ( msg.len == 15 ) msg.len = 64;
    msg.mb = mailbox_reader_increment++;
    msg.timestamp = mbxAddr[0] & 0xFFFF;
    msg.bus = busNumber;
    for ( uint8_t i = 0; i < (mbsize >> 2); i++ ) for ( int8_t d = 0; d < 4 ; d++ ) msg.buf[(4 * i) + 3 - d] = (uint8_t)(mbxAddr[2 + i] >> (8 * d));
    (void)FLEXCANb_TIMER(_bus); /* Read free-running timer to unlock Rx Message Buffer. */
    return 1;
    }
  } 
  return 0; /* no messages available */
}















FCTPFD_FUNC void FCTPFD_OPT::setBaudRate(uint32_t baud) {
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


FCTPFD_FUNC void FCTPFD_OPT::FLEXCAN_ExitFreezeMode() {
  FLEXCANb_MCR(_bus) &= ~FLEXCAN_MCR_HALT;
  while (FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK);
}

FCTPFD_FUNC void FCTPFD_OPT::FLEXCAN_EnterFreezeMode() {
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT;
  while (!(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK));
}

FCTPFD_FUNC void FCTPFD_OPT::softReset() {
  FLEXCANb_MCR(_bus) |= FLEXCAN_MCR_SOFT_RST;
  while (FLEXCANb_MCR(_bus) & FLEXCAN_MCR_SOFT_RST);
}

FCTPFD_FUNC void FCTPFD_OPT::setTx(FLEXCAN_PINS pin) {
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

FCTPFD_FUNC void FCTPFD_OPT::setRx(FLEXCAN_PINS pin) {
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












FCTPFD_FUNC void FCTPFD_OPT::writeIFLAG(uint64_t value) {
  FLEXCANb_IFLAG2(_bus) = value >> 32;
  FLEXCANb_IFLAG1(_bus) = value;
}

FCTPFD_FUNC void FCTPFD_OPT::writeIFLAGBit(uint8_t mb_num) {
  if ( mb_num < 32 ) FLEXCANb_IFLAG1(_bus) |= (1UL << mb_num);
  else FLEXCANb_IFLAG2(_bus) |= (1UL << (mb_num - 32));
}

FCTPFD_FUNC void FCTPFD_OPT::writeIMASK(uint64_t value) {
  FLEXCANb_IMASK2(_bus) = value >> 32;
  FLEXCANb_IMASK1(_bus) = value;
}

FCTPFD_FUNC void FCTPFD_OPT::writeIMASKBit(uint8_t mb_num, bool set) {
  if ( mb_num < 32 ) (( set ) ? FLEXCANb_IMASK1(_bus) |= (1UL << mb_num) : FLEXCANb_IMASK1(_bus) &= ~(1UL << mb_num));
  else (( set ) ? FLEXCANb_IMASK2(_bus) |= (1UL << (mb_num - 32)) : FLEXCANb_IMASK2(_bus) &= ~(1UL << (mb_num - 32)));
}

FCTPFD_FUNC void FCTPFD_OPT::enableFIFO(bool status) {
  bool frz_flag_negate = !(FLEXCANb_MCR(_bus) & FLEXCAN_MCR_FRZ_ACK);
  FLEXCAN_EnterFreezeMode();
  writeIMASK(0ULL); // disable all FIFO/MB Interrupts
  FLEXCANb_RXMGMASK(_bus) = FLEXCANb_RXFGMASK(_bus) = 0;
  writeIFLAG(readIFLAG()); // (all bits reset when written back)
  FLEXCANb_MCR(_bus) &= ~0x20008000; /* we disable DMA, Legacy FIFO (we never use this in FD mode!) */
  for (uint8_t i = 0; i < max_mailboxes(); i++ ) FLEXCANb_RXIMR(_bus, i) = 0; // CLEAR MAILBOX MASKS (RXIMR)
  for (uint8_t i = 0, mbsize = 0; i < max_mailboxes(); i++) { /* clear valid MB codes from memory regions */
    volatile uint32_t *mbxAddr = &(*(uint32_t*)(mailbox_offset(i, mbsize)));
    mbxAddr[0] = mbxAddr[1] = 0;
  }

  if ( status ) { /* enable FIFO */
    // TODO: FD FIFO
  }
  else { // FIFO disabled default setup of mailboxes, 0-7 RX, 8-15 TX
    for (uint8_t i = 0, mbsize = 0; i < max_mailboxes(); i++ ) { // clear all mailboxes
      volatile uint32_t *mbxAddr = &(*(uint32_t*)(mailbox_offset(i, mbsize)));
      if ( i < max_mailboxes() / 2 ) {
        mbxAddr[0] = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY) | ((i < (max_mailboxes() / 4)) ? 0 : FLEXCAN_MB_CS_IDE | FLEXCAN_MB_CS_SRR);
      }
      else {
        mbxAddr[0] = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
      }
    }
  }







// TODO: CONFIGURE FIFO DYNAMIC ADDRESSED MBs

#ifdef MEOWMIX
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
#endif


  if ( frz_flag_negate ) FLEXCAN_ExitFreezeMode();
}







void flexcan_isr_can3fd() {
  if ( _CAN3 ) _CAN3->flexcan_interrupt();
}

FCTPFD_FUNC void FCTPFD_OPT::flexcan_interrupt() {
  Serial.print("FD INTERRUPT! ");
  Serial.print(millis());
}
