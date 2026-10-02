/* SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Cogless-Commercial
 * Copyright (c) 2026 Ronan Mingon */
/*
 * rc_input.h - servo PWM (1000..2000 us) throttle input on J2.3 (PA1).
 *
 * EXTI interrupt on both edges of PA1, time-stamped by TIM2 running as a
 * free 1 MHz counter (no TIM2 remap: remaps 5/6 would steal PA0/PA2/PA3
 * from the TIM1 gate driver outputs). Anything outside 800..2300 us is ignored. A missing signal for
 * RC_TIMEOUT_MS invalidates the input (failsafe).
 */
#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <stdint.h>

#define RC_PULSE_MIN_US       800U
#define RC_PULSE_MAX_US       2300U
#define RC_THROTTLE_LOW_US    1050U    /* at or below: throttle 0, arming level */
#define RC_THROTTLE_HIGH_US   1950U    /* at or above: throttle 100 %           */
#define RC_TIMEOUT_MS         100U
#define RC_ARM_TIME_MS        1000U    /* zero throttle held this long to arm   */

void     rc_init(void);
uint8_t  rc_valid(void);              /* pulses arriving, not timed out */
uint16_t rc_pulse_us(void);           /* last accepted pulse width      */
uint16_t rc_throttle(void);           /* 0..1000, 0 when invalid        */

#endif /* RC_INPUT_H */
