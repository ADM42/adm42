/* Copyright 2023-2024 Lorenzo Leonini
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

static uint8_t positions[] = {
    3, 4, 7, 8, 11, 12, 13, 14, 15, 16, 17, 18,
    19, 20, 21, 22, 23,
    25, 26, 27, 28, 29, 30, 41, 40, 39, 38, 37, 36};
static uint8_t position = 0;
static uint16_t position_timer;
static uint8_t trail_length = sizeof(positions) * 0.9;
extern bool rgb_wpm_enabled;
extern uint8_t rgb_wpm_val;

static bool ADM42(effect_params_t* params) {
    RGB_MATRIX_USE_LIMITS(led_min, led_max);

    uint8_t value;
    if (rgb_wpm_enabled) {
        value = 255;
    } else {
        value = rgb_matrix_get_val();
        if (value < 2 * trail_length) value = 2 * trail_length;
    }

    uint8_t increments = (value * 0.9) / trail_length;

    if (params->init) {
        rgb_matrix_set_color_all(0, 0, 0);
        position = 0;
        position_timer = timer_read();
    }

    uint8_t speed = 135 - scale8(rgb_matrix_get_speed(), 128);
    if (rgb_wpm_enabled) {
        speed = qsub8(150, scale8(rgb_wpm_val, 140));
    }
    if (timer_elapsed(position_timer) >= speed) {
        position_timer = timer_read();

        rgb_matrix_set_color_all(0x00, 0x00, 0x00);

        HSV hsv = {rgb_matrix_get_hue(), rgb_matrix_get_sat(), value};
        RGB rgb = rgb_matrix_hsv_to_rgb(hsv);
        rgb_matrix_set_color(0, rgb.r, rgb.g, rgb.b);

        position = (position + 1) % sizeof(positions);

        for (uint8_t l = 1; l <= trail_length; l++) {
            hsv.v = increments * l;
            rgb = rgb_matrix_hsv_to_rgb(hsv);
            rgb_matrix_set_color(positions[(position + l) % sizeof(positions)], rgb.r, rgb.g, rgb.b);
        }
    }
    return rgb_matrix_check_finished_leds(led_max);
}

#endif // RGB_MATRIX_CUSTOM_EFFECT_IMPLS
