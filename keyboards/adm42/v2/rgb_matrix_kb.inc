/* Copyright 2023-2026 Lorenzo Leonini
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

RGB_MATRIX_EFFECT(ADM42)

#ifdef RGB_MATRIX_CUSTOM_EFFECT_IMPLS

static const uint8_t positions[] = {
    3, 4, 7, 8, 11, 12, 13, 14, 15, 16, 17, 18,
    19, 20, 21, 22, 23,
    25, 26, 27, 28, 29, 30, 41, 40, 39, 38, 37, 36
};

#define POS_COUNT ((uint8_t)sizeof(positions))
//#define TRAIL_LENGTH POS_COUNT
#define TRAIL_LENGTH (POS_COUNT + 5)
#define PERIOD_MS_SLOW 2500UL
#define PERIOD_MS_FAST 400UL

static bool ADM42(effect_params_t *params) {
    RGB_MATRIX_USE_LIMITS(led_min, led_max);

    uint8_t speed = rgb_matrix_get_speed();

    uint16_t period_ms =
        PERIOD_MS_SLOW -
        ((PERIOD_MS_SLOW - PERIOD_MS_FAST) * (uint32_t)speed) / 255;

    // Head pos 8.8
    uint32_t head_pos_q8 =
        ((g_rgb_timer % period_ms) * ((uint32_t)POS_COUNT << 8)) / period_ms;

    uint8_t head_idx = (uint8_t)(head_pos_q8 >> 8);

    uint8_t max_val = rgb_matrix_get_val();

    HSV hsv = {
        .h = rgb_matrix_get_hue(),
        .s = rgb_matrix_get_sat(),
        .v = 0
    };

    for (uint8_t i = led_min; i < led_max; i++) {

        if (i == 0) {
            hsv.v = max_val;
            RGB rgb = rgb_matrix_hsv_to_rgb(hsv);
            rgb_matrix_set_color(i, rgb.r, rgb.g, rgb.b);
            continue;
        }

        uint8_t brightness = 0;

        for (uint8_t p = 0; p < POS_COUNT; p++) {
            if (positions[p] != i) continue;
            uint8_t dist = (head_idx >= p) ? (head_idx - p) : (POS_COUNT + head_idx - p);
            if (dist < TRAIL_LENGTH) {
                uint8_t fraction = 255 - ((uint16_t)dist * 255) / (TRAIL_LENGTH - 1);
                brightness = scale8(ease8InOutQuad(fraction), max_val);
            }
            break;
        }

        if (brightness) {
            hsv.v = brightness;
            RGB rgb = rgb_matrix_hsv_to_rgb(hsv);
            rgb_matrix_set_color(i, rgb.r, rgb.g, rgb.b);
        } else {
            rgb_matrix_set_color(i, 0, 0, 0);
        }
    }

    return rgb_matrix_check_finished_leds(led_max);
}

#endif // RGB_MATRIX_CUSTOM_EFFECT_IMPLS
