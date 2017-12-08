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

/*** SPELLBOUND STATES ***/
#define SPELLBOUND_EMPTY_S         0x00
#define SPELLBOUND_DETECTION_S     0x01
#define SPELLBOUND_TRANS_MISSION_S 0x02

/*** PORT MODES ***/
#define SPELLBOUND_EMPTY_M         0x00
#define SPELLBOUND_PEGS_M          0x01
#define SPELLBOUND_CD32_M          0x02
#define SPELLBOUND_7800_M          0x03
#define SPELLBOUND_MDRV_M          0x04
#define SPELLBOUND_MSYS_M          0x05

/* DE-9 male connector pins, to be populated with MPU pin numbers (count from 1, skip GND & VCC) */
typedef struct {
  int8_t one_p, two_p, three_p, four_p, five_p, six_p, seven_p;
} spellbound_pins_t;

typedef struct {
  uint8_t hidID, buttonsMaskA, buttonsMaskB;
} spellbound_trans_t;

typedef struct {
  uint8_t mode : 4;
  uint8_t dpad : 4;
  uint16_t buttons;
  spellbound_pins_t pins;
  spellbound_trans_t trans;
} spellbound_state_t;

/**********************/
/*** TIME CONSTANTS ***/
/**********************/
#define SPELLBOUND_MAINLOOP_T            5
#define SPELLBOUND_DETECTION_T           200
#define SPELLBOUND_SAMPLE_FREQ           4800
#define SPELLBOUND_SAMPLE_INTERVAL       200

/*** AMIGA CD32 CONTROLLER SERIAL PROTOCOL TIMING ***/
#define SPELLBOUND_CD32_SHIFT_HI_T       5
#define SPELLBOUND_CD32_SHIFT_LO_T       10
#define SPELLBOUND_CD32_CLOCK_HI_T       10
#define SPELLBOUND_CD32_CLOCK_LO_T       10

/*** MICROGENIUS/PEGS/DENDY SERIAL PROTOCOL TIMING ***/
#define SPELLBOUND_PEGS_STROBE_T         12
#define SPELLBOUND_PEGS_CLOCK_HI_T       12
#define SPELLBOUND_PEGS_CLOCK_LO_T       12

/*** SEGA MEGADRIVE PROTOCOL TIMING ***/
#define SPELLBOUND_MDRV_SELECT_UP_T      10
#define SPELLBOUND_MDRV_SELECT_DOWN_T    10

/*********************/
/*** PIN CONSTANTS ***/
/*********************/
#define SPELLBOUND_DETECTBUTTON_P        0x09
#define SPELLBOUND_SOUNDOUT_P            0x0A

/*** AMIGA CD32/ATARI 7800/SEGA MEGADRIVE ATMEGA PINOUT ***/
#define SPELLBOUND_CD32_7800_MDRV_1_DPAD_UP_P           A0
#define SPELLBOUND_CD32_7800_MDRV_1_DPAD_DOWN_P         A1
#define SPELLBOUND_CD32_7800_MDRV_1_DPAD_LEFT_GROUND_P  A2
#define SPELLBOUND_CD32_7800_MDRV_1_DPAD_RIGHT_GROUND_P A3
#define SPELLBOUND_CD32_7800_MDRV_1_RED_FIRE_CLK_A_B_P  0x10
#define SPELLBOUND_CD32_7800_MDRV_1_LOAD_SHIFT_SELECT_P 0x0f
#define SPELLBOUND_CD32_7800_MDRV_1_BLUE_DATA_START_C_P 0x0e

#define SPELLBOUND_CD32_7800_MDRV_2_DPAD_UP_P           0x05
#define SPELLBOUND_CD32_7800_MDRV_2_DPAD_DOWN_P         0x04
#define SPELLBOUND_CD32_7800_MDRV_2_DPAD_LEFT_GROUND_P  0x03
#define SPELLBOUND_CD32_7800_MDRV_2_DPAD_RIGHT_GROUND_P 0x02
#define SPELLBOUND_CD32_7800_MDRV_2_RED_FIRE_CLK_A_B_P  0x08
#define SPELLBOUND_CD32_7800_MDRV_2_LOAD_SHIFT_SELECT_P 0x06
#define SPELLBOUND_CD32_7800_MDRV_2_BLUE_DATA_START_C_P 0x07

/*** MICROGENIUS/PEGS/DENDY ATMEGA PINOUT ***/
#define SPELLBOUND_PEGS_CLOCK_P          A3
#define SPELLBOUND_PEGS_STROBE_P         A2
#define SPELLBOUND_PEGS_1_DATA_P         A1
#define SPELLBOUND_PEGS_2_DATA_P         A0


