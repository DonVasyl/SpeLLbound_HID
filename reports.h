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

#define SPELLBOUND_HIDREPORT_S 50

typedef struct {
  const uint8_t report[SPELLBOUND_HIDREPORT_S];
  HIDSubDescriptor subDescriptor;
} spellbound_hidReport_t;

#define HID_REPORT(pName, pID, pButtons) static const uint8_t m_hidReport ## pName[] PROGMEM = { \
    0x05, 0x01,                       \
    0x09, 0x05,                       \
    0xa1, 0x01,                       \
    0x85, pID,                        \
                                      \
    0x05, 0x09,                       \
    0x19, 0x01,                       \
    0x29, pButtons,                   \
    0x15, 0x00,                       \
    0x25, 0x01,                       \
    0x75, 0x01,                       \
    0x95, ((pButtons + 8 - 1) / 8)*8, \
    0x55, 0x00,                       \
    0x65, 0x00,                       \
    0x81, 0x02,                       \
                                      \
    0x05, 0x01,                       \
    0x09, 0x01,                       \
    0x15, 0x81,                       \
    0x25, 0x7f,                       \
    0xA1, 0x00,                       \
    0x09, 0x30,                       \
    0x09, 0x31,                       \
    0x75, 0x08,                       \
    0x95, 0x02,                       \
    0x81, 0x02,                       \
    0xc0,                             \
    0xc0,                             \
};                                    \
HIDSubDescriptor *m_hidSubDescriptor ## pName;

HID_REPORT(CD32_1, 0x03, 0x07)
HID_REPORT(7800_1, 0x03, 0x02)
HID_REPORT(MDRV_1, 0x03, 0x04)

HID_REPORT(PEGS_1, 0x03, 0x04)
HID_REPORT(PEGS_2, 0x04, 0x04)

HID_REPORT(CD32_2, 0x05, 0x07)
HID_REPORT(7800_2, 0x05, 0x02)
HID_REPORT(MDRV_2, 0x05, 0x04)


