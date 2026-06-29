#pragma once

/**
 * ChannelPack - Shared channel packing/unpacking library for CRSF RC link
 *
 * Packs all RC remote inputs into 16 CRSF channels (11-bit each).
 * Used by both the TX (ESP32-S3 controller) and RX (QT Py receiver).
 *
 * Channel Assignment:
 *   CH1-4:  Gimbal axes (LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y) - proportional
 *   CH5-6:  Potentiometers (POT_1, POT_2) - proportional
 *   CH7-8:  Encoder positions (wrapped 11-bit)
 *   CH9:    8x 2-pos switches (SW_A-H) - bitfield
 *   CH10:   3 buttons + 2 three-pos toggles - bitfield
 *   CH11:   10 nav switch buttons (2x 5-way) - bitfield
 *   CH12-16: Reserved (center)
 */

#include <stdint.h>

// CRSF channel value constants
#define CPACK_CRSF_MIN  191   // 1000us equivalent
#define CPACK_CRSF_MID  992   // 1500us equivalent
#define CPACK_CRSF_MAX  1792  // 2000us equivalent

// Channel assignments (0-based)
#define CPACK_CH_LEFT_X      0
#define CPACK_CH_LEFT_Y      1
#define CPACK_CH_RIGHT_X     2
#define CPACK_CH_RIGHT_Y     3
#define CPACK_CH_POT1        4
#define CPACK_CH_POT2        5
#define CPACK_CH_ENC1        6
#define CPACK_CH_ENC2        7
#define CPACK_CH_SWITCHES    8   // 6x 2-pos switches (A,B,C,D,G,H) + 2 reserved
#define CPACK_CH_BTN_TOGGLE  9   // 4 buttons + 2 three-pos toggles (SW_E, SW_F)
#define CPACK_CH_NAV        10   // 10 nav switch buttons (2x5)
#define CPACK_NUM_CHANNELS  16

// Nav switch direction indices within the 5-element array
#define CPACK_NAV_UP     0
#define CPACK_NAV_DOWN   1
#define CPACK_NAV_LEFT   2
#define CPACK_NAV_RIGHT  3
#define CPACK_NAV_CENTER 4

// Input data structure
typedef struct {
    int16_t gimbal[4];      // -1000 to +1000 (LX, LY, RX, RY)
    int16_t pot[2];         // 0 to 1000
    int32_t encoder[2];     // raw encoder position
    bool    switches[8];    // SW_A,B,C,D,G,H (bits 0-5), bits 6-7 reserved
    bool    buttons[4];     // BTN_1, BTN_2, BTN_3, BTN_4
    uint8_t toggles[2];    // 3-pos: 0=UP, 1=CENTER, 2=DOWN
    bool    nav[2][5];      // nav[switch_idx][U/D/L/R/C]
} ChannelPackInputs_t;

//=============================================================================
// Pack functions (TX side)
//=============================================================================

namespace ChannelPack {

inline int16_t clampI16(int16_t val, int16_t lo, int16_t hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

// Map gimbal value (-1000..+1000) to CRSF channel value (191..1792)
inline uint16_t gimbalToCrsf(int16_t val) {
    val = clampI16(val, -1000, 1000);
    return (uint16_t)(((int32_t)(val + 1000) * (CPACK_CRSF_MAX - CPACK_CRSF_MIN)) / 2000 + CPACK_CRSF_MIN);
}

// Map pot value (0..1000) to CRSF channel value (191..1792)
inline uint16_t potToCrsf(int16_t val) {
    val = clampI16(val, 0, 1000);
    return (uint16_t)(((int32_t)val * (CPACK_CRSF_MAX - CPACK_CRSF_MIN)) / 1000 + CPACK_CRSF_MIN);
}

// Map encoder position to 11-bit value (wrapping)
inline uint16_t encoderToCrsf(int32_t pos) {
    return (uint16_t)(((pos % 2048) + 2048) % 2048);
}

// Pack 8 switches into 11-bit value (bits 0-7)
inline uint16_t packSwitches(const bool sw[8]) {
    uint16_t val = 0;
    for (int i = 0; i < 8; i++) {
        if (sw[i]) val |= (1 << i);
    }
    return val;
}

// Pack 4 buttons + 2 three-pos toggles into 11-bit value
// Bits 0-3: BTN_1, BTN_2, BTN_3, BTN_4
// Bits 4-5: SW_E 3-pos (0-2)
// Bits 6-7: SW_F 3-pos (0-2)
inline uint16_t packButtonsToggles(const bool btn[4], const uint8_t tog[2]) {
    uint16_t val = 0;
    for (int i = 0; i < 4; i++) {
        if (btn[i]) val |= (1 << i);
    }
    val |= ((uint16_t)(tog[0] & 0x03)) << 4;
    val |= ((uint16_t)(tog[1] & 0x03)) << 6;
    return val;
}

// Pack 10 nav switches (2x5-way) into 11-bit value
// Bits 0-4: NAV1 (U,D,L,R,C)
// Bits 5-9: NAV2 (U,D,L,R,C)
inline uint16_t packNavSwitches(const bool nav[2][5]) {
    uint16_t val = 0;
    for (int i = 0; i < 5; i++) {
        if (nav[0][i]) val |= (1 << i);
        if (nav[1][i]) val |= (1 << (i + 5));
    }
    return val;
}

// Pack all inputs into 16-channel array (CRSF 11-bit values)
inline void packInputs(const ChannelPackInputs_t* in, uint16_t ch[CPACK_NUM_CHANNELS]) {
    ch[CPACK_CH_LEFT_X]     = gimbalToCrsf(in->gimbal[0]);
    ch[CPACK_CH_LEFT_Y]     = gimbalToCrsf(in->gimbal[1]);
    ch[CPACK_CH_RIGHT_X]    = gimbalToCrsf(in->gimbal[2]);
    ch[CPACK_CH_RIGHT_Y]    = gimbalToCrsf(in->gimbal[3]);
    ch[CPACK_CH_POT1]       = potToCrsf(in->pot[0]);
    ch[CPACK_CH_POT2]       = potToCrsf(in->pot[1]);
    ch[CPACK_CH_ENC1]       = encoderToCrsf(in->encoder[0]);
    ch[CPACK_CH_ENC2]       = encoderToCrsf(in->encoder[1]);
    ch[CPACK_CH_SWITCHES]   = packSwitches(in->switches);
    ch[CPACK_CH_BTN_TOGGLE] = packButtonsToggles(in->buttons, in->toggles);
    ch[CPACK_CH_NAV]        = packNavSwitches(in->nav);
    for (int i = 11; i < CPACK_NUM_CHANNELS; i++) {
        ch[i] = CPACK_CRSF_MID;
    }
}

//=============================================================================
// Unpack functions (RX side)
//=============================================================================

inline int16_t crsfToGimbal(uint16_t crsf_val) {
    return (int16_t)(((int32_t)(crsf_val - CPACK_CRSF_MIN) * 2000) / (CPACK_CRSF_MAX - CPACK_CRSF_MIN) - 1000);
}

inline int16_t crsfToPot(uint16_t crsf_val) {
    return (int16_t)(((int32_t)(crsf_val - CPACK_CRSF_MIN) * 1000) / (CPACK_CRSF_MAX - CPACK_CRSF_MIN));
}

inline int32_t crsfToEncoder(uint16_t crsf_val) {
    return (int32_t)(crsf_val & 0x7FF);
}

inline void unpackSwitches(uint16_t val, bool sw[8]) {
    for (int i = 0; i < 8; i++) {
        sw[i] = (val >> i) & 1;
    }
}

inline void unpackButtonsToggles(uint16_t val, bool btn[4], uint8_t tog[2]) {
    for (int i = 0; i < 4; i++) {
        btn[i] = (val >> i) & 1;
    }
    tog[0] = (val >> 4) & 0x03;
    tog[1] = (val >> 6) & 0x03;
}

inline void unpackNavSwitches(uint16_t val, bool nav[2][5]) {
    for (int i = 0; i < 5; i++) {
        nav[0][i] = (val >> i) & 1;
        nav[1][i] = (val >> (i + 5)) & 1;
    }
}

// Unpack 16-channel array into input struct
inline void unpackChannels(const uint16_t ch[CPACK_NUM_CHANNELS], ChannelPackInputs_t* out) {
    out->gimbal[0] = crsfToGimbal(ch[CPACK_CH_LEFT_X]);
    out->gimbal[1] = crsfToGimbal(ch[CPACK_CH_LEFT_Y]);
    out->gimbal[2] = crsfToGimbal(ch[CPACK_CH_RIGHT_X]);
    out->gimbal[3] = crsfToGimbal(ch[CPACK_CH_RIGHT_Y]);
    out->pot[0]    = crsfToPot(ch[CPACK_CH_POT1]);
    out->pot[1]    = crsfToPot(ch[CPACK_CH_POT2]);
    out->encoder[0] = crsfToEncoder(ch[CPACK_CH_ENC1]);
    out->encoder[1] = crsfToEncoder(ch[CPACK_CH_ENC2]);
    unpackSwitches(ch[CPACK_CH_SWITCHES], out->switches);
    unpackButtonsToggles(ch[CPACK_CH_BTN_TOGGLE], out->buttons, out->toggles);
    unpackNavSwitches(ch[CPACK_CH_NAV], out->nav);
}

} // namespace ChannelPack
