/* Copyright 2020-2024 Lorenzo Leonini
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include QMK_KEYBOARD_H
#include <version.h>

#include <rgb_matrix.h>
extern void rgb_matrix_update_pwm_buffers(void);

// Base for timming computations
static uint8_t tapping_term = 200;

// Config in eeprom
static union {
    uint32_t raw;
    struct {
        int pending_win : 2;
        int pending_ctrl : 2;
        int pending_shift : 2;
        bool opposite_mods_as_tap : 1;
        int compose_ralt: 2; // F19, R_ALT, R_ALT + R_ALT => L_ALT, DISABLED
    };
} user_config;

enum custom_layers {
    _QWERTY,
    _COLEMAKDH,
    _COLEMAX,
    _DVORAK,
    _COLEMAK,
    _CUSTOM,

    _SPECIAL,
    _EXTRA,
    _ADM,
    _SETUP,
};

#define SETUP MO(_SETUP)

enum custom_keycodes {
    REFLASH = SAFE_RANGE,

    // Custom dual-function keys
    FIRST_DUAL, // do not remove
    LW_GRV,
    LW_DOT,
    LC_TAB,
    LS_BPC,
    RW_EQU,
    RC_QUT,
    RC_SCLN,
    RC_SLSH,
    RS_SPC,
    LC_CIRC,
    RC_DLR,
    RW_BS,
    LW_F11,
    RW_F12,
    LAST_DUAL, // do not remove

    // Custom layer keys
    FIRST_LAYER, // do not remove
    LLS_ESC,
    LLS_COMP,
    LLE_ENT,
    LLA_DEL,
    LAST_LAYER, // do not remove

    COMPOSE,
    LOR_ALT,
    CWD_TOG,

    KGB_WHT,
    KGB_RED,
    KGB_GRN,
    KGB_BLU,
    RGB_WPM,

    DF_QWER,
    DF_COMK,
    DF_CODH,
    DF_COMX,
    DF_DVRK,
    DF_CUST,
    ADM_INF,
    PEN_WIN,
    PEN_CTRL,
    PEN_SHFT,
    OPP_TOGG,
    COMP_SEL,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [_QWERTY] = LAYOUT_3x12_6(
            LW_GRV,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    RW_EQU,
            LC_TAB,  KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, RC_QUT,
            KC_LALT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, LOR_ALT,
                                       LLS_ESC, LS_BPC,  LLA_DEL, LLE_ENT, RS_SPC,  LLS_COMP
    ),
    [_COLEMAKDH] = LAYOUT_3x12_6(
            LW_GRV,  KC_Q,    KC_W,    KC_F,    KC_P,    KC_B,    KC_J,    KC_L,    KC_U,    KC_Y,    KC_SCLN, RW_EQU,
            LC_TAB,  KC_A,    KC_R,    KC_S,    KC_T,    KC_G,    KC_M,    KC_N,    KC_E,    KC_I,    KC_O,    RC_QUT,
            KC_LALT, KC_Z,    KC_X,    KC_C,    KC_D,    KC_V,    KC_K,    KC_H,    KC_COMM, KC_DOT,  KC_SLSH, LOR_ALT,
                                       LLS_ESC, LS_BPC,  LLA_DEL, LLE_ENT, RS_SPC,  LLS_COMP
    ),
    [_COLEMAX] = LAYOUT_3x12_6(
            LW_GRV,  KC_Q,    KC_W,    KC_F,    KC_P,    KC_G,    KC_K,    KC_L,    KC_U,    KC_Y,    KC_QUOT, RW_EQU,
            LC_TAB,  KC_A,    KC_R,    KC_S,    KC_T,    KC_D,    KC_H,    KC_N,    KC_E,    KC_I,    KC_O,    RC_SCLN,
            KC_LALT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_J,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, LOR_ALT,
                                       LLS_ESC, LS_BPC,  LLA_DEL, LLE_ENT, RS_SPC,  LLS_COMP
    ),
    [_COLEMAK] = LAYOUT_3x12_6(
            LW_GRV,  KC_Q,    KC_W,    KC_F,    KC_P,    KC_G,    KC_J,    KC_L,    KC_U,    KC_Y,    KC_SCLN, RW_EQU,
            LC_TAB,  KC_A,    KC_R,    KC_S,    KC_T,    KC_D,    KC_H,    KC_N,    KC_E,    KC_I,    KC_O,    RC_QUT,
            KC_LALT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_K,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, LOR_ALT,
                                       LLS_ESC, LS_BPC,  LLA_DEL, LLE_ENT, RS_SPC,  LLS_COMP
    ),
    // DVORAK has one additional key (UNDS) on its layout, that's why the 2
    // adaptions have been done in order to keep UNDS on the second layer:
    // - RC_SCLN instead of RC_UNDS
    // - LW_GRV instead of LW_SCLN
    [_DVORAK] = LAYOUT_3x12_6(
            LW_GRV,  KC_QUOT, KC_COMM, KC_DOT,  KC_P,    KC_Y,    KC_F,    KC_G,    KC_C,    KC_R,    KC_L,    RW_EQU,
            LC_TAB,  KC_A,    KC_O,    KC_E,    KC_U,    KC_I,    KC_D,    KC_H,    KC_T,    KC_N,    KC_S,    RC_SLSH,
            KC_LALT, KC_SCLN, KC_Q,    KC_J,    KC_K,    KC_X,    KC_B,    KC_M,    KC_W,    KC_V,    KC_Z,    LOR_ALT,
                                       LLS_ESC, LS_BPC,  LLA_DEL, LLE_ENT, RS_SPC,  LLS_COMP
    ),
    // Set your custom layout here!
    [_CUSTOM] = LAYOUT_3x12_6(
            LW_GRV,  KC_Q,    KC_W,    KC_D,    KC_F,    KC_P,    KC_K,    KC_L,    KC_U,    KC_Y,    KC_QUOT, RW_EQU,
            LC_TAB,  KC_A,    KC_R,    KC_S,    KC_T,    KC_G,    KC_H,    KC_N,    KC_E,    KC_O,    KC_I,    RC_SCLN,
            KC_LALT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_J,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, LOR_ALT,
                                       LLS_ESC, LS_BPC,  LLA_DEL, LLE_ENT, RS_SPC,  LLS_COMP
    ),
    [_SPECIAL] = LAYOUT_3x12_6(
            LW_DOT,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    RW_BS,
            LC_CIRC, KC_LBRC, KC_RBRC, KC_LPRN, KC_RPRN, KC_EXLM, KC_LEFT, KC_DOWN, KC_UP,   KC_RGHT, KC_MINS, RC_DLR,
            _______, KC_AMPR, KC_AT,   KC_LCBR, KC_RCBR, KC_PIPE, KC_UNDS, KC_ASTR, KC_HASH, KC_PERC, KC_TILD, _______,
                                       KC_ESC,  _______, KC_DEL,  KC_ENT,  _______, COMPOSE
    ),
    [_EXTRA] = LAYOUT_3x12_6(
            LW_F11,  KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  RW_F12,
            KC_LCTL, KC_PAUS, KC_INS,  KC_VOLD, KC_VOLU, KC_MUTE, KC_HOME, KC_PGDN, KC_PGUP, KC_END,  KC_APP,  KC_RCTL,
            _______, KC_SLEP, KC_PWR,  KC_MSTP, KC_MNXT, KC_MPLY, _______, KC_BRID, KC_BRIU, KC_PSCR, KC_WAKE, LOR_ALT,
                                       KC_CAPS, _______, KC_DEL,  _______, _______, CWD_TOG
    ),
    [_ADM] = LAYOUT_3x12_6(
            XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, RGB_M_B, RGB_VAD, RGB_VAI, RGB_SAD, RGB_SAI, RGB_WPM,
            XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, RGB_TOG, RGB_MOD, RGB_RMOD,RGB_HUD, RGB_HUI, KGB_WHT,
            XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, RGB_M_P, RGB_SPD, RGB_SPI, KGB_RED, KGB_GRN, KGB_BLU,
                                       XXXXXXX, XXXXXXX, XXXXXXX, SETUP,   XXXXXXX, XXXXXXX
    ),
    [_SETUP] = LAYOUT_3x12_6(
            REFLASH, XXXXXXX, DF_COMK, XXXXXXX, XXXXXXX, XXXXXXX, PEN_WIN, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, ADM_INF,
            XXXXXXX, DF_QWER, DF_CODH, DF_DVRK, DF_CUST, XXXXXXX, PEN_CTRL, OPP_TOGG, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,
            XXXXXXX, XXXXXXX, DF_COMX, XXXXXXX, XXXXXXX, XXXXXXX, PEN_SHFT, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, EE_CLR,
                                       XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,   XXXXXXX, COMP_SEL
    ),
};

static uint8_t mod_state;

#define PK(s) print_keyboard(s)
void print_keyboard(const char *msg) {
    mod_state = get_mods();
    unregister_mods(mod_state);
    send_string(msg);
    register_mods(mod_state);
}

/*
 * ADM42's dual-function keys
 * - Pending
 * - Context-aware
 * - Opposite MODS as TAP
 * - Rolling support
 */

typedef struct {
    // Config
    uint16_t mod;
    uint16_t tap;
    bool left; // Physical side on the keyboard, for Opposite MODs as TAP
    bool force_shift; // If true, SHIFT the tap key
    bool pending_mod; // If true, this mode is allowed to be pending

    // Contexts for immediate tap
    bool context_overlap; // A (tap) key is currently pressed
    bool context_previous; // Another tap has just been done

    // State
    uint16_t time; // Physical keypress time
    // If shift was enabled at physical press time, that means the user wants
    // the shifted version of the key
    bool shifted;
    uint16_t pressed; // kc pressed, in order to release the same one
    bool mod_used; // If the mod has been «used» by a tap key
} modtap;

// Keys configuration
static modtap modtaps[] = {

    // Base layer keys

    // LW_GRV
    {.mod = KC_LWIN, .tap = KC_GRV, .left = true, .context_overlap = true, .context_previous = true},
    // LW_DOT
    {.mod = KC_LWIN, .tap = KC_DOT, .left = true, .context_overlap = true, .context_previous = true},
    // LC_TAB
    {.mod = KC_LCTL, .tap = KC_TAB, .left = true, .context_overlap = true, .context_previous = true},
    // LS_BPC
    {.mod = KC_LSFT, .tap = KC_BSPC, .left = true, .context_overlap = false, .context_previous = false},
    // RW_EQU
    {.mod = KC_RWIN, .tap = KC_EQL, .context_overlap = true, .context_previous = true, },
    // RC_QUT
    {.mod = KC_RCTL, .tap = KC_QUOT, .context_overlap = true, .context_previous = true, },
    // RC_SCLN
    {.mod = KC_RCTL, .tap = KC_SCLN, .context_overlap = true, .context_previous = true, },
    // RC_SLSH
    {.mod = KC_RCTL, .tap = KC_SLSH, .context_overlap = true, .context_previous = true, },
    // RS_SPC
    {.mod = KC_RSFT, .tap = KC_SPC, .context_overlap = true, .context_previous = true, },

    // Special layer keys

    // LC_CIRC ^
    {.mod = KC_LCTL, .tap = KC_6, .left = true, .force_shift = true},
    // RC_DLR $
    {.mod = KC_RCTL, .tap = KC_4, .force_shift = true},
    // RW_BS
    {.mod = KC_RWIN, .tap = KC_BSLS},

    // Function layer keys

    // LW_F11
    {.mod = KC_LWIN, .tap = KC_F11, .left = true},
    // RW_F12
    {.mod = KC_RWIN, .tap = KC_F12},
};

#define MODTAP(X) modtaps[X - FIRST_DUAL - 1]

// For context/overlap logic
static uint8_t current_taps = 0;
static uint16_t last_tap_kc = 0;
static uint16_t last_tap_time = 0;
static uint16_t last_tap_release_time = 0;

static uint16_t mt_taphold_key = 0;
static uint16_t mt_taphold_time = 0;

// Store the current and previous keycode (press)
static uint16_t p_current_keycode;
static uint16_t p_current_time;
static uint16_t p_prev_keycode;
static uint16_t p_prev_time;

static uint16_t delayed_kc = 0;
static uint16_t delayed_time = 0;
// Smaller delay: favor MOD, larger delay: favor TAP
#define DELAYED_MOD_OVERLAP (tapping_term / 6)

#define TAP_IF_LAST_PREV_TAP (tapping_term / 3)

static uint16_t retained_mod_key = 0;

void config_pending(void) {
    for (uint8_t i = 0; i < sizeof(modtaps) / sizeof(modtaps[0]); i++) {
        modtaps[i].pending_mod = false;
    }

    if (user_config.pending_win == 1) {
        MODTAP(LW_GRV).pending_mod = true;
    }
    if (user_config.pending_ctrl == 1) {
        MODTAP(LC_TAB).pending_mod = true;
    }
    if (user_config.pending_shift == 1) {
        MODTAP(LS_BPC).pending_mod = true;
    }

    if (user_config.pending_win == 2) {
        MODTAP(RW_EQU).pending_mod = true;
    }
    if (user_config.pending_ctrl == 2) {
        MODTAP(RC_QUT).pending_mod = true;
        MODTAP(RC_SCLN).pending_mod = true;
        MODTAP(RC_SLSH).pending_mod = true;
    }
    if (user_config.pending_shift == 2) {
        MODTAP(RS_SPC).pending_mod = true;
    }
}

bool is_modtap(uint16_t key) {
    return (key > FIRST_DUAL && key < LAST_DUAL);
}

bool is_layer(uint16_t key) {
    return (key > FIRST_LAYER && key < LAST_LAYER);
}

bool is_controlable(uint16_t kc) {
    return (kc >= KC_A && kc <=KC_Z);
}

bool is_shiftable(uint16_t kc) {
    return is_controlable(kc);
}

void clean_modtap_state(uint16_t key) {
    MODTAP(key).time = 0;
    MODTAP(key).shifted = false;
    MODTAP(key).pressed = 0;
    MODTAP(key).mod_used = false;
}

void mod_used(bool super, bool ctrl, bool shift) {
    for (int i = 0; i < sizeof(modtaps) / sizeof(modtap); i++) {
        if (!modtaps[i].mod_used && modtaps[i].pressed == modtaps[i].mod) {
            uint16_t mod = modtaps[i].mod;
            if (
                    (super && (mod == KC_LWIN || mod == KC_RWIN)) ||
                    (ctrl && (mod == KC_LCTL || mod == KC_RCTL)) ||
                    (shift && (mod == KC_LSFT || mod == KC_RSFT))
                 ) {
                modtaps[i].mod_used = true;
            }
        }
    }
}

// All TAPs (=> no MODs or LAYERs)
// time: should be the time of physical keypress (even if the effect is delayed)
void last_tap_info(uint16_t kc, uint16_t time) {
    last_tap_kc = kc;
    last_tap_time = time;
    current_taps += 1;
}
void last_tap_release_info(void) {
    last_tap_release_time = timer_read();
    if (current_taps >= 1) {
        current_taps -= 1;
    }
}

bool check_already_mod(uint16_t key) {
    uint16_t mod = MODTAP(key).mod;
    mod_state = get_mods();
    if ((mod == KC_LWIN || mod == KC_RWIN) && (mod_state & MOD_MASK_GUI)) {
        return true;
    }
    if ((mod == KC_LCTL || mod == KC_RCTL) && (mod_state & MOD_MASK_CTRL)) {
        return true;
    }
    if ((mod == KC_LSFT || mod == KC_RSFT) && (mod_state & MOD_MASK_SHIFT)) {
        return true;
    }
    return false;
}

bool check_opposite_mods(uint16_t key) {
    if (get_mods()) {
        for (int i = 0; i < sizeof(modtaps) / sizeof(modtap); i++) {
            if (MODTAP(key).mod != modtaps[i].mod && modtaps[i].pressed == modtaps[i].mod) {
                if (MODTAP(key).left == !modtaps[i].left) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool check_tap_context(uint16_t key) {
    if (check_already_mod(key)) {
        return true;
    }

    if (user_config.opposite_mods_as_tap && check_opposite_mods(key)) {
        return true;
    }

    if (MODTAP(key).context_overlap && current_taps && timer_elapsed(last_tap_time) < tapping_term) {
        return true;
    }

    if (MODTAP(key).context_previous &&
            last_tap_kc != MODTAP(key).tap &&
            last_tap_kc != KC_BSPC &&
            timer_elapsed(last_tap_release_time) < TAP_IF_LAST_PREV_TAP) {
        return true;
    }

    return false;
}

void mt_register_mod(uint16_t key) {
    MODTAP(key).pressed = MODTAP(key).mod;
    register_code(MODTAP(key).mod);
    if (current_taps > 0) {
        MODTAP(key).mod_used = true;
    }
}

void mt_register_tap_kc(uint16_t key, uint16_t kc) {
    MODTAP(key).pressed = kc;
    register_code(kc);
    last_tap_info(kc, MODTAP(key).time);
    mod_used(true, true, false);
}

void mt_register_tap(uint16_t key) {
    if (retained_mod_key == key) {
        retained_mod_key = 0;
    }
    if (MODTAP(key).pressed == MODTAP(key).mod) {
        unregister_code(MODTAP(key).mod);
    }
    uint16_t kc = MODTAP(key).tap;

    if (MODTAP(key).pressed != kc) {
        mod_state = get_mods();
        bool shift_state = mod_state & MOD_MASK_SHIFT;
        if (MODTAP(key).force_shift && !shift_state) {
            add_weak_mods(MOD_BIT(KC_LSFT));
            mt_register_tap_kc(key, kc);
        } else if (MODTAP(key).shifted && !shift_state) {
            add_weak_mods(MOD_BIT(KC_LSFT));
            mt_register_tap_kc(key, kc);
        } else if (shift_state && !MODTAP(key).shifted && !MODTAP(key).force_shift) {
            del_mods(MOD_MASK_SHIFT);
            mt_register_tap_kc(key, kc);
            set_mods(mod_state);
        } else {
            mt_register_tap_kc(key, kc);
        }
    }
}

void mt_taphold_check(void) {
    if (mt_taphold_key && timer_elapsed(mt_taphold_time) > tapping_term) {
        mt_register_tap(mt_taphold_key);
        mt_taphold_key = 0;
    }
}
void mt_taphold_prepare(uint16_t key) {
    mt_taphold_key = key;
    mt_taphold_time = timer_read();
}
void mt_taphold_reset(void) {
    mt_taphold_key = 0;
}

void delayed_prepare(uint16_t kc) {
    delayed_kc = kc;
    delayed_time = timer_read();
}
void delayed_tap(void) {
    register_code(delayed_kc);
    last_tap_info(delayed_kc, delayed_time);
    mod_used(true, true, true);
    delayed_kc = 0;
}
void delayed_check(void) {
    if (delayed_kc && timer_elapsed(delayed_time) > DELAYED_MOD_OVERLAP) {
        delayed_tap();
    }
}

void retained_mod_check(void) {
    if (retained_mod_key && timer_elapsed(MODTAP(retained_mod_key).time) > tapping_term) {
        mt_register_mod(retained_mod_key);
        retained_mod_key = 0;
    }
}

// Called on physical press
void mt_press(uint16_t key) {
    clean_modtap_state(key);
    MODTAP(key).time = timer_read();

    if (get_mods() & MOD_MASK_SHIFT) {
        MODTAP(key).shifted = true;
        mod_used(false, false, true);
    }

    if (check_tap_context(key)) {
        mt_register_tap(key);
    } else {
        if (MODTAP(key).tap == last_tap_kc && timer_elapsed(last_tap_release_time) <= tapping_term) {
            // On dual keys the «hold» will start only after TAPPING_TERM if nothing else happen before
            mt_taphold_prepare(key);
        }

        if (current_taps == 0 &&
            retained_mod_key == 0 &&
            !get_mods()
        ) {
            if (MODTAP(key).pending_mod) {
                mt_register_mod(key);
            } else {
                retained_mod_key = key;
            }
        } else {
            mt_register_mod(key);
        }
    }
}

// Called on physical release
void mt_release(uint16_t key) {
    if (!MODTAP(key).mod_used && timer_elapsed(MODTAP(key).time) <= tapping_term) {
        mt_register_tap(key);
    }
    if (MODTAP(key).pressed) {
        unregister_code(MODTAP(key).pressed);
        if (MODTAP(key).pressed != MODTAP(key).mod) {
            last_tap_release_info();
        }
    }
    clean_modtap_state(key);
}

void adm_info(void) {
    PK("# ADM42\n\n");

    PK("* Layout: ");
    switch (get_highest_layer(default_layer_state)) {
     case _QWERTY:
        PK("QWERTY\n");
        break;
     case _COLEMAK:
        PK("Colemak\n");
        break;
     case _COLEMAKDH:
        PK("Colemak-DH\n");
        break;
     case _COLEMAX:
        PK("ColeMAX\n");
        break;
     case _DVORAK:
        PK("Dvorak\n");
        break;
     case _CUSTOM:
        PK("CUSTOM\n");
        break;
    }

    PK("* Pending MOD(s): ");
    if (!user_config.pending_win && !user_config.pending_ctrl && !user_config.pending_shift ) PK("DISABLED");
    if (user_config.pending_win) {
        if (user_config.pending_win == 1) PK("L_");
        if (user_config.pending_win == 2) PK("R_");
        PK("SUPER ");
    } 
    if (user_config.pending_ctrl) {
        if (user_config.pending_ctrl == 1) PK("L_");
        if (user_config.pending_ctrl == 2) PK("R_");
        PK("CTRL ");
    } 
    if (user_config.pending_shift) {
        if (user_config.pending_shift == 1) PK("L_");
        if (user_config.pending_shift == 2) PK("R_");
        PK("SHIFT ");
    } 
    PK("\n");

    PK("* Opposite MODs as TAP: ");
    if (user_config.opposite_mods_as_tap) {
        PK("ENABLED\n");
    } else {
        PK("DISABLED\n");
    }

    PK("* Compose key: ");
    if (user_config.compose_ralt == 3) {
        PK("DISABLED\n");
    } else if (user_config.compose_ralt == 2) {
        PK("R_ALT (+ R_ALT => L_ALT)\n");
    } else if (user_config.compose_ralt == 1) {
        PK("R_ALT\n");
    } else {
        PK("F19\n");
    }

    PK("* CAPS_WORD\n");

    PK("* RGB: +AUTOBRIGHTNESS +WPM\n");

#if defined(NKRO_ENABLE) && defined(FORCE_NKRO)
    PK("* NKRO");
#endif
    if (USB_POLLING_INTERVAL_MS == 1) {
        PK(" / 1000Hz");
    }
    PK("\n");

    PK("\n" QMK_KEYBOARD ":" QMK_KEYMAP " | " __DATE__ " - " __TIME__);
}

static uint8_t rgb_val = 255;

void rgb_set_val(uint8_t v) {
    rgb_matrix_sethsv_noeeprom(
        rgb_matrix_get_hue(),
        rgb_matrix_get_sat(),
        v);
}

static uint8_t rgb_mode = 0;
static uint16_t rgb_mode_check_time = 0;

void rgb_up(void) {
    rgb_mode = 1;
    rgb_set_val(1);
}

#define RGB_RAMP_UP_STEP 6

void rgb_mode_check(void) {

    if (over_ma) {
        over_ma = false;
        uint8_t v = rgb_matrix_get_val();
        if (v >= 4) {
            rgb_mode = 0;
            rgb_set_val(v - 4);
            return;
        }
    }

    if (timer_elapsed(rgb_mode_check_time) > RGB_MATRIX_LED_FLUSH_LIMIT) {
        rgb_mode_check_time = timer_read();

        uint8_t v = rgb_matrix_get_val();

        if (rgb_mode == 1) {
            if (v >= rgb_val) {
                rgb_mode = 0;
            } else {
                if (v < rgb_val - RGB_RAMP_UP_STEP) {
                    v += RGB_RAMP_UP_STEP;
                } else {
                    v = rgb_val;
                    rgb_mode = 0;
                }
                rgb_set_val(v);
                return;
            }
        }
    }
}

#define RGB_WPM_MIN 20
#define RGB_WPM_INC 5
#define RGB_WPM_DEC_TIME 30
bool rgb_wpm_enabled = false;
uint8_t rgb_wpm_val = RGB_WPM_MIN;
static uint16_t rgb_wpm_time = 0;

void rgb_val_decrease(void) {
    if (timer_elapsed(rgb_wpm_time) > RGB_WPM_DEC_TIME) {
        rgb_wpm_time = timer_read();
        if (rgb_wpm_val > RGB_WPM_MIN) {
            rgb_wpm_val -= 1;
            rgb_set_val(rgb_wpm_val);
        }
    }
}
void rgb_val_increase(void) {
    if (rgb_wpm_val < 255 - RGB_WPM_INC) {
        rgb_wpm_val += RGB_WPM_INC;
    }
}
void rgb_wpm_enable(void) {
    if (!rgb_wpm_enabled) {
        rgb_wpm_enabled = true;
        rgb_set_val(rgb_wpm_val);
    }
}
void rgb_wpm_disable(void) {
    if (rgb_wpm_enabled) {
            rgb_wpm_enabled = false;
            rgb_up();
    }
}
void rgb_wpm_toggle(void) {
    if (rgb_wpm_enabled) {
        rgb_wpm_disable();
    } else {
        rgb_wpm_enable();
    }
}

static bool caps_word = false;

void caps_word_stop(void) {
    if (caps_word) {
        caps_word = false;
        if (host_keyboard_leds() & 2) {
            tap_code(KC_CAPS);
        }
    }
}

/*
 * ADM42's layer-tap implementation
 */

typedef struct {
    // Config
    uint16_t layer;
    uint16_t tap;
    bool autorepeat;

    // State
    uint16_t tapped;
} layertap;

// Keys configuration
static layertap layertaps[] = {
    // LLS_ESC
    {.tap = KC_ESC, .layer = _SPECIAL},
    // LLS_COMP
    {.tap = KC_F19, .layer = _SPECIAL},
    // LLE_ENT
    {.tap = KC_ENT, .layer = _EXTRA, .autorepeat = true},
    // LLA_DEL
    {.tap = KC_DEL, .layer = _ADM, .autorepeat = true}
};
#define LAYERTAP(X) layertaps[X - FIRST_LAYER - 1]

// Called on physical press
void lt_press(uint16_t key) {
    if (
        (LAYERTAP(key).autorepeat &&
        p_prev_keycode == key &&
        timer_elapsed(last_tap_release_time) <= tapping_term)
    ) {
        uint16_t kc = LAYERTAP(key).tap;
        LAYERTAP(key).tapped = true;
        register_code(kc);
        caps_word_stop();
        last_tap_info(kc, timer_read());
        mod_used(true, true, true);

    } else {
        LAYERTAP(key).tapped = false;
        layer_on(LAYERTAP(key).layer);
    }
}

// Called on physical release
void lt_release(uint16_t key) {
    if (LAYERTAP(key).tapped) {
        uint16_t kc = LAYERTAP(key).tap;
        unregister_code(kc);
        last_tap_release_info();
    } else {
        layer_off(LAYERTAP(key).layer);
        if (key == p_current_keycode && timer_elapsed(p_current_time) <= tapping_term) {
            uint16_t kc = LAYERTAP(key).tap;
            tap_code(kc);
            caps_word_stop();
            last_tap_info(kc, p_current_time);
            mod_used(true, true, true);
            last_tap_release_info();
        }
    }
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {

    if (record->event.pressed) {
        if (rgb_wpm_enabled) rgb_val_increase();
        p_prev_keycode = p_current_keycode;
        p_current_keycode = keycode;
        p_prev_time = p_current_time;
        p_current_time = record->event.time;
    }

    // Any key press or release cancel the potential hold
    mt_taphold_reset();

    // A mod can only be retained until the next key press
    if (retained_mod_key && record->event.pressed) {
        mt_register_mod(retained_mod_key);
        retained_mod_key = 0;
    } else {
        if (retained_mod_key == keycode) {
            retained_mod_key = 0;
        }
    }

    // Important to react before any other key press or if the delayed key is released
    if (delayed_kc && (keycode == delayed_kc || record->event.pressed)) {
        delayed_tap();
    }

    if (keycode == CWD_TOG) {
        if (record->event.pressed) {
            if (caps_word) {
                caps_word = false;
                if (host_keyboard_leds() & 2) {
                    tap_code(KC_CAPS);
                }
            } else {
                if (!(host_keyboard_leds() & 2)) {
                    caps_word = true;
                }
                tap_code(KC_CAPS);
            }
        }
        return false;
    }
    if (caps_word && record->event.pressed) {
        if (!(
            (keycode >= KC_A && keycode <=KC_Z) ||
            (keycode >= KC_1 && keycode <= KC_0) ||
            keycode == KC_MINS || keycode == KC_UNDS ||
            keycode == LS_BPC ||
            keycode == LLS_ESC || keycode == LLS_COMP || keycode == LLE_ENT ||
            keycode == KC_CAPS
        )) {
            caps_word_stop();
        }
    }

    // Mod-tap keys management
    if (is_modtap(keycode)) {
        if (record->event.pressed) {
            mt_press(keycode);
        } else {
            mt_release(keycode);
        }
        return false;
    }

    // Delays keys that can be affected by (pending) MODs.
    if (record->event.pressed && last_tap_kc != KC_SPC &&
            is_shiftable(keycode) &&
            (get_mods() & MOD_MASK_SHIFT)) {
        delayed_prepare(keycode);
        return false;
    }
    if (record->event.pressed &&
            is_controlable(keycode) && (get_mods() & MOD_MASK_CTRL)) {
        delayed_prepare(keycode);
        return false;
    }

    // Layer-tap keys management
    if (is_layer(keycode)) {
        if (record->event.pressed) {
            lt_press(keycode);
        } else {
            lt_release(keycode);
        }
        return false;
    }

    /* Standard tap management */

    if (keycode != KC_LALT && keycode != KC_RALT && keycode != LOR_ALT && keycode != COMPOSE) { 
        if (record->event.pressed) {
            last_tap_info(keycode, record->event.time);
            mod_used(true, true, true);
        } else {
            last_tap_release_info();
        }
    }

    if (record->event.pressed) {
        switch (keycode) {
         case DF_QWER:
            set_single_persistent_default_layer(_QWERTY);
            return false;
         case DF_COMK:
            set_single_persistent_default_layer(_COLEMAK);
            return false;
         case DF_CODH:
            set_single_persistent_default_layer(_COLEMAKDH);
            return false;
         case DF_COMX:
            set_single_persistent_default_layer(_COLEMAX);
            return false;
         case DF_DVRK:
            set_single_persistent_default_layer(_DVORAK);
            return false;
         case DF_CUST:
            set_single_persistent_default_layer(_CUSTOM);
            return false;
         case REFLASH:
            writePinLow(QMK_LED);
            reset_keyboard();
            return false;

         case ADM_INF:
            adm_info();
            return false;

        case COMP_SEL:
            if (user_config.compose_ralt == 3) {
                user_config.compose_ralt = 0;
            } else {
                user_config.compose_ralt++;
            }
            if (user_config.compose_ralt == 3) {
                LAYERTAP(LLS_COMP).tap = KC_NO;
            } else if (user_config.compose_ralt == 1 || user_config.compose_ralt == 2) {
                LAYERTAP(LLS_COMP).tap = KC_RALT;
            } else {
                LAYERTAP(LLS_COMP).tap = KC_F19;
            }
            eeconfig_update_user(user_config.raw);
            return false;

        case PEN_WIN:
            if (user_config.pending_win == 2) {
                user_config.pending_win = 0;
            } else {
                user_config.pending_win++;
            }
            eeconfig_update_user(user_config.raw);
            config_pending();
            return false;
        case PEN_CTRL:
            if (user_config.pending_ctrl == 2) {
                user_config.pending_ctrl = 0;
            } else {
                user_config.pending_ctrl++;
            }
            eeconfig_update_user(user_config.raw);
            config_pending();
            return false;
        case PEN_SHFT:
            if (user_config.pending_shift == 2) {
                user_config.pending_shift = 0;
            } else {
                user_config.pending_shift++;
            }
            eeconfig_update_user(user_config.raw);
            config_pending();
            return false;

        case OPP_TOGG:
            user_config.opposite_mods_as_tap ^= 1;
            eeconfig_update_user(user_config.raw);
            return false;

         case RGB_WPM:
            rgb_wpm_toggle();
            return false;
         case RGB_VAD:
            if (rgb_val >= RGB_MATRIX_VAL_STEP) {
                rgb_val -= RGB_MATRIX_VAL_STEP;
            } else {
                rgb_val = 0;
            }
            rgb_wpm_disable();
            rgb_set_val(rgb_val);
            return false;
         case RGB_VAI:
            if (rgb_val < 255 - RGB_MATRIX_VAL_STEP) {
                rgb_val += RGB_MATRIX_VAL_STEP;
            } else {
                rgb_val = 255;
            }
            rgb_wpm_disable();
            rgb_set_val(rgb_val);
            return false;
         case KGB_WHT:
            rgb_matrix_sethsv(0, 0, rgb_val);
            return false;
         case KGB_RED:
            rgb_matrix_sethsv(0, 255, rgb_val);
            return false;
         case KGB_GRN:
            rgb_matrix_sethsv(85, 255, rgb_val);
            return false;
         case KGB_BLU:
            rgb_matrix_sethsv(169, 255, rgb_val);
            return false;
         case RGB_SAD:
         case RGB_SAI:
         case RGB_HUD:
         case RGB_HUI:
            return true;
         case RGB_M_B:
         case RGB_M_P:
         case RGB_MOD:
         case RGB_RMOD:
            rgb_wpm_disable();
            return true;
         case RGB_SPD:
         case RGB_SPI:
            rgb_wpm_disable();
            return true;
        }
    }

    if (record->event.pressed) {
        switch (keycode) {
         case COMPOSE:
            if (user_config.compose_ralt == 1 || user_config.compose_ralt == 2) {
                register_code(KC_RALT);
            } else if (user_config.compose_ralt == 0) {
                register_code(KC_F19);
            }
            return false;
        case LOR_ALT:
            register_code((user_config.compose_ralt == 2) ? KC_LALT : KC_RALT);
            return false;
        }
    } else {
        switch (keycode) {
         case COMPOSE:
            if (user_config.compose_ralt == 1 || user_config.compose_ralt == 2) {
                unregister_code(KC_RALT);
            } else if (user_config.compose_ralt == 0) {
                unregister_code(KC_F19);
            }
            return false;
         case LOR_ALT:
            unregister_code((user_config.compose_ralt == 2) ? KC_LALT : KC_RALT);
            return false;
        }
    }

    return true;
}

void matrix_scan_user(void) {
    mt_taphold_check();
    retained_mod_check();
    delayed_check();

    rgb_mode_check();
    if (rgb_wpm_enabled) {
        rgb_val_decrease();
    }
}

void keyboard_pre_init_kb(void) {
    setPinOutput(QMK_LED);
    writePinHigh(QMK_LED);
    rgb_set_val(0);
    rgb_matrix_update_pwm_buffers();
}

void keyboard_post_init_kb(void) {
    rgb_up();

    user_config.raw = eeconfig_read_user();
    if (user_config.compose_ralt == 3) {
        LAYERTAP(LLS_COMP).tap = KC_NO;
    } else if (user_config.compose_ralt == 1 || user_config.compose_ralt == 2) {
        LAYERTAP(LLS_COMP).tap = KC_RALT;
    } else {
        LAYERTAP(LLS_COMP).tap = KC_F19;
    }
    config_pending();
}

void eeconfig_init_user(void) {
    user_config.raw = eeconfig_read_user();
    user_config.pending_win = 0;
    user_config.pending_ctrl = 1;
    user_config.pending_shift = 1;
    user_config.opposite_mods_as_tap = 1;
    user_config.compose_ralt = 0;
    eeconfig_update_user(user_config.raw);
}

void suspend_power_down_kb(void) {
    writePinLow(QMK_LED);
}

void suspend_wakeup_init_kb(void) {
    writePinHigh(QMK_LED);
    rgb_up();
}
