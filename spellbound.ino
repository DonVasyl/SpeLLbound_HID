// >>> Spellbound_HID <<<
// - Amiga CD32, Atari 7800, MicroGenius/Dendy/Pegasus/Terminator, SEGA MegaDrive HID FW for Arduino Pro Micro boards -
//
// CD32: SHIFT/CLK/DATA/D-Pad
// 7800: Red(Fire1)/Blue(Fire2)/D-Pad
// PEGS: CLK/STROBE/D0, A/B/SELECT/START/D-Pad
// MDRV: SELECT, A/B/C/START/D-Pad
// MSYS: TBD, 1_START/2/D-Pad
//
// PCM audio replay code ("SPEECH CODE") for 32u4 based on speaker_pcm (PCMAudio) by Michael Smith
//
// written by DonVasyl/Chinchillah
// (c) Chinchillah/Hostile & Waters, 2016

#include <Arduino.h>
#include <HID.h>
#include <avr/wdt.h>

#include "common.h"
#include "speech.h"
#include "reports.h"

/**************************/
/*** USER CONFIGURATION ***/
/**************************/
#define SPELLBOUND_DONT_SPEAK 0x00

// button re-mappers for each protocol, feel free to edit
static const uint8_t spellbound_translator_CD32[0x07] = {0x01, 0x00, 0x03, 0x02, 0x05, 0x04, 0x06};
static const uint8_t spellbound_translator_PEGS[0x04] = {0x00, 0x01, 0x02, 0x03};
static const uint8_t spellbound_translator_7800[0x02] = {0x00, 0x01};
static const uint8_t spellbound_translator_MDRV[0x04] = {0x00, 0x01, 0x02, 0x03};

/*******************/
/*** SYSTEM DATA ***/
/*******************/

static uint8_t spellbound_state = SPELLBOUND_EMPTY_S;

static spellbound_state_t m_State_PEGS_1, m_State_PEGS_2;
static spellbound_state_t m_State_CD32_7800_SEGA_1, m_State_CD32_7800_SEGA_2;

/*****************************/
/*** PIN SETTING FUNCTIONS ***/
/*****************************/

static void SETPINS_PEGS_1(spellbound_state_t *pState){
  pState->pins.one_p   = -1;
  pState->pins.two_p   = SPELLBOUND_PEGS_CLOCK_P;
  pState->pins.three_p = SPELLBOUND_PEGS_STROBE_P;
  pState->pins.four_p  = SPELLBOUND_PEGS_1_DATA_P;
  pState->pins.five_p  = -1;
  pState->pins.six_p   = -1;
  pState->pins.seven_p = -1;

  pState->trans.hidID = m_hidReportPEGS_1[0x07];
}

static void SETPINS_PEGS_2(spellbound_state_t *pState){
  pState->pins.one_p   = -1;
  pState->pins.two_p   = SPELLBOUND_PEGS_CLOCK_P;
  pState->pins.three_p = SPELLBOUND_PEGS_STROBE_P;
  pState->pins.four_p  = SPELLBOUND_PEGS_2_DATA_P;
  pState->pins.five_p  = -1;
  pState->pins.six_p   = -1;
  pState->pins.seven_p = -1;

  pState->trans.hidID = m_hidReportPEGS_2[0x07];
}

static void SETPINS_CD32_7800_MDRV_1(spellbound_state_t *pState){
  pState->pins.one_p   = SPELLBOUND_CD32_7800_MDRV_1_DPAD_UP_P;
  pState->pins.two_p   = SPELLBOUND_CD32_7800_MDRV_1_DPAD_DOWN_P;
  pState->pins.three_p = SPELLBOUND_CD32_7800_MDRV_1_DPAD_LEFT_GROUND_P;
  pState->pins.four_p  = SPELLBOUND_CD32_7800_MDRV_1_DPAD_RIGHT_GROUND_P;
  pState->pins.five_p  = SPELLBOUND_CD32_7800_MDRV_1_LOAD_SHIFT_SELECT_P;
  pState->pins.six_p   = SPELLBOUND_CD32_7800_MDRV_1_RED_FIRE_CLK_A_B_P;
  pState->pins.seven_p = SPELLBOUND_CD32_7800_MDRV_1_BLUE_DATA_START_C_P;

  pState->trans.hidID = m_hidReportCD32_1[0x07]; // common value for 7800, CD32, MDRV & MSYS
}

static void SETPINS_CD32_7800_MDRV_2(spellbound_state_t *pState){
  pState->pins.one_p   = SPELLBOUND_CD32_7800_MDRV_2_DPAD_UP_P;
  pState->pins.two_p   = SPELLBOUND_CD32_7800_MDRV_2_DPAD_DOWN_P;
  pState->pins.three_p = SPELLBOUND_CD32_7800_MDRV_2_DPAD_LEFT_GROUND_P;
  pState->pins.four_p  = SPELLBOUND_CD32_7800_MDRV_2_DPAD_RIGHT_GROUND_P;
  pState->pins.five_p  = SPELLBOUND_CD32_7800_MDRV_2_LOAD_SHIFT_SELECT_P;
  pState->pins.six_p   = SPELLBOUND_CD32_7800_MDRV_2_RED_FIRE_CLK_A_B_P;
  pState->pins.seven_p = SPELLBOUND_CD32_7800_MDRV_2_BLUE_DATA_START_C_P;

  pState->trans.hidID = m_hidReportCD32_2[0x07]; // common value for 7800, CD32, MDRV & MSYS
}

/***************************/
/*** DETECTION FUNCTIONS ***/
/***************************/

static void DETECT_PEGS(spellbound_state_t *pState){
  INIT_PEGS(pState);
  READ_PEGS(pState);
  if(pState->buttons == ((0xffff & ~0x0f) | (0x01 << spellbound_translator_PEGS[0x00]))) {
    pState->trans.buttonsMaskA = 0x0f;
    pState->trans.buttonsMaskB = 0x00;
    pState->mode = SPELLBOUND_PEGS_M;
  }
}

static void DETECT_CD32(spellbound_state_t *pState){
  INIT_7800DPad(pState);
  INIT_CD32Buttons7(pState);
  READ_CD32Buttons7(pState);
  if(pState->buttons == ((0xffff & ~0x7f) | (0x01 << spellbound_translator_CD32[0x00]))) {
    pState->trans.buttonsMaskA = 0x7f;
    pState->trans.buttonsMaskB = 0x00;
    pState->mode = SPELLBOUND_CD32_M;
  }
}

static void DETECT_7800(spellbound_state_t *pState){
  INIT_7800DPad(pState);
  INIT_7800Buttons2(pState);
  READ_7800Buttons2(pState);
  if(pState->buttons == ((0xffff & ~0x03) | (0x01 << spellbound_translator_7800[0x00]))) {
    pState->trans.buttonsMaskA = 0x03;
    pState->trans.buttonsMaskB = 0x00;
    pState->mode = SPELLBOUND_7800_M;
  }
}

static void DETECT_MDRV(spellbound_state_t *pState){
  INIT_7800DPad(pState);
  INIT_MDRVButtons4(pState);
  READ_MDRV(pState);
  if(pState->buttons == ((0xffff & ~0x0f) | (0x01 << spellbound_translator_MDRV[0x00]))) {
    pState->trans.buttonsMaskA = 0x0f;
    pState->trans.buttonsMaskB = 0x00;
    pState->mode = SPELLBOUND_MDRV_M;
  }
}

/**********************/
/*** INIT FUNCTIONS ***/
/**********************/

static void INIT_7800Buttons2(spellbound_state_t *pState) {
  pinMode(pState->pins.five_p, OUTPUT);  
  digitalWrite(pState->pins.five_p, HIGH);
  pinMode(pState->pins.six_p, INPUT);  
  digitalWrite(pState->pins.six_p, HIGH);
  pinMode(pState->pins.seven_p, INPUT);  
  digitalWrite(pState->pins.seven_p, HIGH);  
}

static void INIT_CD32Buttons7(spellbound_state_t *pState) {
  pinMode(pState->pins.five_p, OUTPUT);  
  digitalWrite(pState->pins.five_p, HIGH);
  pinMode(pState->pins.six_p, OUTPUT);
  digitalWrite(pState->pins.six_p, LOW);
  pinMode(pState->pins.seven_p, INPUT);
  digitalWrite(pState->pins.seven_p, HIGH);
}

static void INIT_7800DPad(spellbound_state_t *pState) {
  pinMode(pState->pins.one_p, INPUT);
  digitalWrite(pState->pins.one_p, HIGH);
  pinMode(pState->pins.two_p, INPUT);
  digitalWrite(pState->pins.two_p, HIGH);
  pinMode(pState->pins.three_p, INPUT);
  digitalWrite(pState->pins.three_p, HIGH);
  pinMode(pState->pins.four_p, INPUT);  
  digitalWrite(pState->pins.four_p, HIGH);
}

static void INIT_PEGS(spellbound_state_t *pState) {
  pinMode(pState->pins.two_p, OUTPUT);
  digitalWrite(pState->pins.two_p, LOW);
  pinMode(pState->pins.three_p, OUTPUT);
  digitalWrite(pState->pins.three_p, LOW);
  pinMode(pState->pins.four_p, INPUT);
  digitalWrite(pState->pins.four_p, LOW);
}

static void INIT_MDRVButtons4(spellbound_state_t *pState){
  pinMode(pState->pins.five_p, OUTPUT);  
  digitalWrite(pState->pins.five_p, LOW);
  pinMode(pState->pins.six_p, INPUT);
  digitalWrite(pState->pins.six_p, HIGH);
  pinMode(pState->pins.seven_p, INPUT);
  digitalWrite(pState->pins.seven_p, HIGH);  
}

/*************************/
/*** READING FUNCTIONS ***/
/*************************/

static void READ_7800DPad(spellbound_state_t *pState) {
  pState->dpad = 0x00;
  pState->dpad |= digitalRead(pState->pins.one_p) << 0x00;
  pState->dpad |= digitalRead(pState->pins.two_p) << 0x01;
  pState->dpad |= digitalRead(pState->pins.three_p) << 0x02;
  pState->dpad |= digitalRead(pState->pins.four_p) << 0x03;
  pState->dpad = ~pState->dpad;
}

static void READ_7800Buttons2(spellbound_state_t *pState) {
  pState->buttons = 0x00;
  pState->buttons |= digitalRead(pState->pins.six_p) << spellbound_translator_7800[0x00];
  pState->buttons |= digitalRead(pState->pins.seven_p) << spellbound_translator_7800[0x01];
  pState->buttons = ~pState->buttons;
}

static void LOAD_CD32Buttons7(spellbound_state_t *pState) {
  pState->buttons = 0x00;
  digitalWrite(pState->pins.five_p,HIGH);
  delayMicroseconds(SPELLBOUND_CD32_SHIFT_HI_T);
  digitalWrite(pState->pins.five_p,LOW);
  delayMicroseconds(SPELLBOUND_CD32_SHIFT_LO_T);
}

static void TICK_CD32Buttons7(spellbound_state_t *pState) {
  digitalWrite(pState->pins.six_p,HIGH);
  delayMicroseconds(SPELLBOUND_CD32_CLOCK_HI_T);
  digitalWrite(pState->pins.six_p,LOW); 
  delayMicroseconds(SPELLBOUND_CD32_CLOCK_LO_T);
}

static void READ_CD32Buttons7(spellbound_state_t *pState) {
  pState->buttons = 0x00;
  LOAD_CD32Buttons7(pState);
  pState->buttons |= digitalRead(pState->pins.seven_p) << spellbound_translator_CD32[0x00];
  for(byte iA = 0x01; iA < 0x07; iA++) {
    TICK_CD32Buttons7(pState);
    pState->buttons |= digitalRead(pState->pins.seven_p) << spellbound_translator_CD32[iA];
  }
  TICK_CD32Buttons7(pState);
  TICK_CD32Buttons7(pState);
  pState->buttons = ~pState->buttons;
}

static void STROBE_PEGS(spellbound_state_t *pState) {
  digitalWrite(pState->pins.three_p,HIGH);
  delayMicroseconds(SPELLBOUND_PEGS_STROBE_T);
  digitalWrite(pState->pins.three_p,LOW);  
}

static void CLOCK_PEGS(spellbound_state_t *pState) {
  delayMicroseconds(SPELLBOUND_PEGS_CLOCK_HI_T);
  digitalWrite(pState->pins.two_p,HIGH);
  delayMicroseconds(SPELLBOUND_PEGS_CLOCK_LO_T);
  digitalWrite(pState->pins.two_p,LOW);
}

static void READ_PEGS(spellbound_state_t *pState) {
  pState->dpad = 0x00;
  pState->buttons = 0x00;
  
  STROBE_PEGS(pState);
  for (byte lI = 0x00; lI < 0x04; lI++) {
    pState->buttons |= digitalRead(pState->pins.four_p) << spellbound_translator_PEGS[lI];
    CLOCK_PEGS(pState);
  }
  for (byte lI = 0x00; lI < 0x04; lI++) {
    pState->dpad |= digitalRead(pState->pins.four_p) << lI;
    CLOCK_PEGS(pState);
  }

  pState->dpad = ~pState->dpad;
  pState->buttons = ~pState->buttons;
}

static void LOW_MDRV(spellbound_state_t *pState) {
  digitalWrite(pState->pins.five_p,LOW);
  delayMicroseconds(SPELLBOUND_MDRV_SELECT_DOWN_T);
}

static void HIGH_MDRV(spellbound_state_t *pState) {
  digitalWrite(pState->pins.five_p,HIGH);
  delayMicroseconds(SPELLBOUND_MDRV_SELECT_UP_T);  
}

static void READ_MDRV(spellbound_state_t *pState) {
  pState->buttons = 0x00;
  
  LOW_MDRV(pState);
  pState->buttons |= digitalRead(pState->pins.six_p) << spellbound_translator_MDRV[0x00];
  pState->buttons |= digitalRead(pState->pins.seven_p) << spellbound_translator_MDRV[0x03];
  HIGH_MDRV(pState);
  READ_7800DPad(pState);
  pState->buttons |= digitalRead(pState->pins.six_p) << spellbound_translator_MDRV[0x01];
  pState->buttons |= digitalRead(pState->pins.seven_p) << spellbound_translator_MDRV[0x02];
  
  pState->buttons = ~pState->buttons;
}

/*******************************/
/*** TRANS MISSION FUNCTIONS ***/
/*******************************/

/*** HID REPORTS ***/
static void HID_Report(const uint8_t *pReport, HIDSubDescriptor *pSubDescriptor) {
  pSubDescriptor = new HIDSubDescriptor(pReport, SPELLBOUND_HIDREPORT_S);
  HID().AppendDescriptor(pSubDescriptor);
}

/*** TRANSMITTERS ***/
static void TRANS_08(spellbound_state_t *pState) {
  int8_t lData[3];

  lData[0] = pState->buttons & pState->trans.buttonsMaskA;
  
  if(pState->dpad & 0x04) {
    lData[1] = -128;
  } else if(pState->dpad & 0x08) {
    lData[1] = +127;
  } else {
    lData[1] = 0;
  }

  if(pState->dpad & 0x01) {
    lData[2] = -128;
  } else if(pState->dpad & 0x02) {
    lData[2] = +127;
  } else {
    lData[2] = 0;
  }

  HID().SendReport(pState->trans.hidID, lData, 0x03);
}

/*******************/
/*** SPEECH CODE ***/
/*******************/
#if !(SPELLBOUND_DONT_SPEAK)
uint8_t m_lastSample;
const uint8_t *m_speechData;
volatile uint16_t m_speechLen, m_currentSample;
#endif

static void SPEECH_Stop(void) {
#if !(SPELLBOUND_DONT_SPEAK)
  TIMSK3 &= ~_BV(OCIE3A);
  TCCR3B &= ~_BV(CS30);
  TCCR1B &= ~_BV(CS10);
  digitalWrite(SPELLBOUND_SOUNDOUT_P, LOW);
#endif
}

#if !(SPELLBOUND_DONT_SPEAK)
ISR(TIMER3_COMPA_vect) {
  if (m_currentSample >= m_speechLen) {
    if (m_currentSample == m_speechLen + m_lastSample) {
        SPEECH_Stop();
    } else {
            OCR1B = m_speechLen + m_lastSample - m_currentSample;  
    }
  } else {
          OCR1B = pgm_read_byte(&m_speechData[m_currentSample]);
  }
  
  m_currentSample++;
}
#endif

static void SPEECH_Say(const uint8_t *pData, uint16_t pLen) {
#if !(SPELLBOUND_DONT_SPEAK)
  m_speechData = pData;
  m_speechLen = pLen;

  TCCR1A |= _BV(WGM10);
  TCCR1B &= ~_BV(WGM12);

  TCCR1A = (TCCR1A | _BV(COM1B1)) & ~_BV(COM1B0);
  TCCR1A &= ~(_BV(COM1A1) | _BV(COM1A0));
  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
  OCR1B = pgm_read_byte(&m_speechData[0]);

  cli();

  TCCR3B = (TCCR3B & ~_BV(WGM33)) | _BV(WGM32);
  TCCR3A = TCCR3A & ~(_BV(WGM31) | _BV(WGM30));
  TCCR3B = (TCCR3B & ~(_BV(CS32) | _BV(CS31))) | _BV(CS30);
  OCR3A = F_CPU / SPELLBOUND_SAMPLE_FREQ;
  TIMSK3 |= _BV(OCIE3A);

  m_lastSample = pgm_read_byte(&m_speechData[m_speechLen - 0x01]);
  m_currentSample = 0x00;
  
  sei();

  while(m_currentSample < m_speechLen + m_lastSample) { }
  delay(SPELLBOUND_SAMPLE_INTERVAL);
#endif
}

/*****************/
/*** MAIN PART ***/
/*****************/

static void STATE_Detection(void){
  // LEFT SIDE
  m_State_CD32_7800_SEGA_1.mode = SPELLBOUND_EMPTY_M;
  m_State_PEGS_1.mode = SPELLBOUND_EMPTY_M;
  m_State_PEGS_2.mode = SPELLBOUND_EMPTY_M;
  SETPINS_CD32_7800_MDRV_1(&m_State_CD32_7800_SEGA_1);
  DETECT_CD32(&m_State_CD32_7800_SEGA_1);

  if(m_State_CD32_7800_SEGA_1.mode == SPELLBOUND_EMPTY_M) {
    DETECT_7800(&m_State_CD32_7800_SEGA_1);
  }

  if(m_State_CD32_7800_SEGA_1.mode == SPELLBOUND_EMPTY_M) {
    DETECT_MDRV(&m_State_CD32_7800_SEGA_1);
  }

  if(m_State_CD32_7800_SEGA_1.mode == SPELLBOUND_EMPTY_M) {
    SETPINS_PEGS_1(&m_State_PEGS_1);
    SETPINS_PEGS_2(&m_State_PEGS_2);
    DETECT_PEGS(&m_State_PEGS_1);
    DETECT_PEGS(&m_State_PEGS_2);
  }

  // RIGHT SIDE
  m_State_CD32_7800_SEGA_2.mode = SPELLBOUND_EMPTY_M;
  SETPINS_CD32_7800_MDRV_2(&m_State_CD32_7800_SEGA_2);
  DETECT_CD32(&m_State_CD32_7800_SEGA_2);
  
  if(m_State_CD32_7800_SEGA_2.mode == SPELLBOUND_EMPTY_M) {
    DETECT_7800(&m_State_CD32_7800_SEGA_2);
  }

  if(m_State_CD32_7800_SEGA_2.mode == SPELLBOUND_EMPTY_M) {
    DETECT_MDRV(&m_State_CD32_7800_SEGA_2);
  }

  // HID REPORTING
  switch(m_State_CD32_7800_SEGA_1.mode) {
    case SPELLBOUND_CD32_M: HID_Report(m_hidReportCD32_1, m_hidSubDescriptorCD32_1); break;
    case SPELLBOUND_7800_M: HID_Report(m_hidReport7800_1, m_hidSubDescriptor7800_1); break;
    case SPELLBOUND_MDRV_M: HID_Report(m_hidReportMDRV_1, m_hidSubDescriptorMDRV_1); break;
  }
  
  switch(m_State_PEGS_1.mode) {
    case SPELLBOUND_PEGS_M: HID_Report(m_hidReportPEGS_1, m_hidSubDescriptorPEGS_1); break;
  }

  switch(m_State_PEGS_2.mode) {
    case SPELLBOUND_PEGS_M: HID_Report(m_hidReportPEGS_2, m_hidSubDescriptorPEGS_2); break;
  }

  switch(m_State_CD32_7800_SEGA_2.mode) {
    case SPELLBOUND_CD32_M: HID_Report(m_hidReportCD32_2, m_hidSubDescriptorCD32_2); break;
    case SPELLBOUND_7800_M: HID_Report(m_hidReport7800_2, m_hidSubDescriptor7800_2); break;
    case SPELLBOUND_MDRV_M: HID_Report(m_hidReportMDRV_2, m_hidSubDescriptorMDRV_2); break;
  }
      
  // SPEECH! SPEECH! SPEECH!
  switch(m_State_CD32_7800_SEGA_1.mode) {
    case SPELLBOUND_CD32_M: SPEECH_Say(cd32, CD32_LEN); SPEECH_Say(one, ONE_LEN); break;
    case SPELLBOUND_7800_M: SPEECH_Say(atari, ATARI_LEN); SPEECH_Say(one, ONE_LEN); break;
    case SPELLBOUND_MDRV_M: SPEECH_Say(megadrive, MEGADRIVE_LEN); SPEECH_Say(one, ONE_LEN); break;
  }
  
  switch(m_State_PEGS_1.mode) {
    case SPELLBOUND_PEGS_M: SPEECH_Say(pegasus, PEGASUS_LEN); SPEECH_Say(one, ONE_LEN); break;
  }

  switch(m_State_PEGS_2.mode) {
    case SPELLBOUND_PEGS_M: SPEECH_Say(pegasus, PEGASUS_LEN); SPEECH_Say(two, TWO_LEN); break;
  }

  switch(m_State_CD32_7800_SEGA_2.mode) {
    case SPELLBOUND_CD32_M: SPEECH_Say(cd32, CD32_LEN); SPEECH_Say(two, TWO_LEN); break;
    case SPELLBOUND_7800_M: SPEECH_Say(atari, ATARI_LEN); SPEECH_Say(two, TWO_LEN); break;
    case SPELLBOUND_MDRV_M: SPEECH_Say(megadrive, MEGADRIVE_LEN); SPEECH_Say(two, TWO_LEN); break;
  }
}

static void STATE_TransMission(void) { 
  switch(m_State_CD32_7800_SEGA_1.mode) {
    case SPELLBOUND_CD32_M:
      READ_7800DPad(&m_State_CD32_7800_SEGA_1); READ_CD32Buttons7(&m_State_CD32_7800_SEGA_1); TRANS_08(&m_State_CD32_7800_SEGA_1); break;
    case SPELLBOUND_7800_M:
      READ_7800DPad(&m_State_CD32_7800_SEGA_1); READ_7800Buttons2(&m_State_CD32_7800_SEGA_1); TRANS_08(&m_State_CD32_7800_SEGA_1); break;
    case SPELLBOUND_MDRV_M: READ_MDRV(&m_State_CD32_7800_SEGA_1); TRANS_08(&m_State_CD32_7800_SEGA_1); break;
    case SPELLBOUND_MSYS_M: break;
  }

  switch(m_State_PEGS_1.mode) {
    case SPELLBOUND_PEGS_M: READ_PEGS(&m_State_PEGS_1); TRANS_08(&m_State_PEGS_1); break;
  }

  switch(m_State_PEGS_2.mode) {
    case SPELLBOUND_PEGS_M: READ_PEGS(&m_State_PEGS_2); TRANS_08(&m_State_PEGS_2); break;
  }

  switch(m_State_CD32_7800_SEGA_2.mode) {
    case SPELLBOUND_CD32_M:
      READ_7800DPad(&m_State_CD32_7800_SEGA_2); READ_CD32Buttons7(&m_State_CD32_7800_SEGA_2); TRANS_08(&m_State_CD32_7800_SEGA_2); break;
    case SPELLBOUND_7800_M:
      READ_7800DPad(&m_State_CD32_7800_SEGA_2); READ_7800Buttons2(&m_State_CD32_7800_SEGA_2); TRANS_08(&m_State_CD32_7800_SEGA_2); break;
    case SPELLBOUND_MDRV_M: READ_MDRV(&m_State_CD32_7800_SEGA_2); TRANS_08(&m_State_CD32_7800_SEGA_2); break;
    case SPELLBOUND_MSYS_M: break;
  }
  delay(SPELLBOUND_MAINLOOP_T);    
}

void setup() {
  wdt_disable();
  pinMode(SPELLBOUND_DETECTBUTTON_P, INPUT);
  digitalWrite(SPELLBOUND_DETECTBUTTON_P, HIGH);

#if !(SPELLBOUND_DONT_SPEAK)
  pinMode(SPELLBOUND_SOUNDOUT_P, OUTPUT);
#endif
}

void loop() {
  switch(spellbound_state) {
    case SPELLBOUND_DETECTION_S:
      STATE_Detection();
      delay(SPELLBOUND_DETECTION_T);
      spellbound_state = SPELLBOUND_TRANS_MISSION_S;
      break;
    case SPELLBOUND_TRANS_MISSION_S:
      if(digitalRead(SPELLBOUND_DETECTBUTTON_P) == LOW) { // switch to detection
        spellbound_state = SPELLBOUND_EMPTY_S;
        wdt_enable(WDTO_120MS); // dirty WATCHDOG reset - the only way to re-configure HID :(
        while(2000) {}
      } else {
        STATE_TransMission();
      }
      break;
    default:
    case SPELLBOUND_EMPTY_S:
      spellbound_state = SPELLBOUND_DETECTION_S;
      break;  
  }
}

