/* SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Cogless-Commercial
 * Copyright (c) 2026 Ronan Mingon */
/*
 * motor_config.h - tunable parameters of the sensorless six-step controller.
 * Hardware facts (pins, dividers, shunt) live in board.h.
 */
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "board.h"

/* ------------------------------------------------------------------ PWM */
#define PWM_FREQ_HZ              32000U   /* 24000 or 48000 */
/* centre-aligned: 0 -> ARR -> 0 in one PWM period */
#define PWM_PERIOD_TICKS         (BOARD_TIM1_CLK_HZ / (2U * PWM_FREQ_HZ))
/* OFF time needed after the top for the BEMF sample: interrupt latency
 * ~47 ticks + sampling ~23 + margin. At 48 kHz it limits the duty to ~83 %. */
#define PWM_OFF_SAMPLE_TICKS     85U
#define PWM_DUTY_MAX_90          (PWM_PERIOD_TICKS * 90U / 100U)   /* bootstrap refresh */
#define PWM_DUTY_MAX             (((PWM_PERIOD_TICKS - PWM_OFF_SAMPLE_TICKS) < PWM_DUTY_MAX_90) \
                                  ? (PWM_PERIOD_TICKS - PWM_OFF_SAMPLE_TICKS) : PWM_DUTY_MAX_90)
#define PWM_DUTY_MIN             (PWM_PERIOD_TICKS * 3U / 100U)
#define PWM_DEAD_TIME_NS         400U
#define PWM_DEAD_TIME_TICKS      ((PWM_DEAD_TIME_NS * (BOARD_TIM1_CLK_HZ / 1000000U)) / 1000U)
/* hardware ADC trigger (ADC_TRIGGER_HW 1): TIM1 CH4 compare before the bottom */
#define PWM_ADC_TRIGGER_TICKS    17U
/* 1: TIM1 CC4 event triggers the ADC, control loop in the ADC interrupt
 * 0: TIM1 CC4 interrupt starts the ADC by software, then runs the loop */
#define ADC_TRIGGER_HW           0
/* software trigger: JSWSTART lands ~47 ticks after the CC4 match, the shunt
 * sample holds ~23 ticks later, i.e. ~15 ticks after the bottom, away from
 * the high-side turn-on transient */
#define PWM_ADC_TRIGGER_TICKS_SW 55U
/* interrupt waits are bounded by the timer, not by a loop count:
 * injected sequence: done ~160 ticks after the bottom, abandoned past this */
#define CC_WAIT_MAX_TICKS        320U
/* OFF-time conversion: done ~70 ticks after the top, abandoned past this */
#define UP_WAIT_MAX_TICKS        200U
/* ADC start later than this past the bottom: current sample ignored */
#define SAMPLE_LATE_TICKS        15U
/* this many late samples in a row with the bridge on: fault (the current
 * would otherwise go unmeasured) */
#define LATE_RUN_MAX             16U
/* 1: one motor_trace entry per closed-loop commutation (debug, costs CPU
 * time at 48 kHz); 0: open-loop start only, zc_stat counts in closed loop */
#define TRACE_IN_RUN             0
/* more than OVR_WINDOW_MAX lost control ticks within OVR_WINDOW_TICKS
 * (power of two, ~85 ms at 48 kHz) with the bridge on: overrun fault */
#define OVR_WINDOW_TICKS         4096U
#define OVR_WINDOW_MAX           32U

/* control loop tick = one PWM period */
#define CTRL_TICK_HZ             PWM_FREQ_HZ
/* Constants tuned at 24 kHz (1000 ticks per period) and written in control
 * ticks are scaled to the actual frequency, so any PWM_FREQ_HZ works
 * (24, 32, 48 kHz...). TICKS_24K(n) = n ticks of the 24 kHz loop. */
#define TICKS_24K(n)             (((uint32_t)(n) * CTRL_TICK_HZ + 12000U) / 24000U)
/* "one duty tick every N control ticks" rates: the loop runs CTRL_TICK_HZ /
 * 24000 times faster AND a duty tick is 1000 / PWM_PERIOD_TICKS times
 * larger. N is multiplied by this ratio, rounded to a power of two (1, 2,
 * 4: it is used as a bit mask). 24 kHz: 1, 32 kHz: 2, 48 kHz: 4. */
#define DUTY_RATE_RATIO_X10      ((CTRL_TICK_HZ * 10000U / PWM_PERIOD_TICKS) / 24000U)
#define DUTY_RATE_SCALE          ((DUTY_RATE_RATIO_X10 >= 30U) ? 4U : (DUTY_RATE_RATIO_X10 >= 15U) ? 2U : 1U)
#define MS_TO_TICKS(ms)          ((uint32_t)(ms) * CTRL_TICK_HZ / 1000U)
/* 100 us units keep multi-second values inside 32 bits */
#define US_TO_TICKS(us)          (((uint32_t)(us) / 100U) * (CTRL_TICK_HZ / 100U) / 100U)

/* ---------------------------------------------------------- bench tests
 * MOTOR_TEST_OPEN_LOOP 1: forced commutation forever after the alignment,
 * no zero-crossing detection. Validates the power stage and the step table. */
#define MOTOR_TEST_OPEN_LOOP     0
#define RAMP_TEST_STEP_US        20000U
#define RAMP_TEST_START_US       100000U
#define RAMP_TEST_DUTY_PCT       3U

/* DRIVER_PIN_TEST: probes run instead of the motor firmware (0 = off).
 *  1: low-side driver inputs as GPIO, one at a time (high sides held low)
 *  2: the six steps through TIM1, state machine bypassed
 *     (HOLD = 0: two revolutions, current per step in step_ma[];
 *      HOLD = N: entry N - 1 held DRIVER_PIN_TEST_STEP_MS, then off)
 *  3: PGA -> ADC path check, bridge never enabled (pga_probe[])
 *  4: short current sweep on one step (sweep_ma[]), current-limited supply
 *  5: hold two chosen steps (HS_CHECK_ENTRY_A/B) to check one phase each
 *  6: low sides forced one at a time through TIM1 (lsreg_* snapshots) */
#define DRIVER_PIN_TEST          0
#define DRIVER_PIN_TEST_STEP_MS  5000U
#define DRIVER_PIN_TEST_CYCLE_MS 700U
#define DRIVER_PIN_TEST_DUTY     50U    /* per mille */
#define DRIVER_PIN_TEST_HOLD     0U

#define SWEEP_POINTS             5
#define SWEEP_DUTIES             { 30, 45, 60, 80, 100 }    /* per mille */
#define SWEEP_POINT_MS           150U
#define SWEEP_ABORT_MA           3000
#define SWEEP_ABORT_SAG_MV       800    /* bus sag abort, independent of the shunt */

/* entries: 0 U+V-, 1 U+W-, 2 V+W-, 3 V+U-, 4 W+U-, 5 W+V-
 * 200 per mille = motor disconnected, 30 = motor connected */
#define HS_CHECK_DUTY            30U
#define HS_CHECK_MS              8000U
#define HS_CHECK_ENTRY_A         3U
#define HS_CHECK_ENTRY_B         5U

/* TEST_AUTORUN 1: no RC input. Arms by itself after TEST_AUTORUN_DELAY_MS
 * and plays the throttle staircase below (level_log[] per level). */
#define TEST_AUTORUN             1
#define TEST_AUTORUN_DELAY_MS    3000U
#define TEST_AUTORUN_THROTTLE    50U
#define TEST_AUTORUN_RAMP_MS     3000U
#define TEST_AUTORUN_ON_MS       10000U
#define TEST_LEVELS_N            5
/* LOAD_TEST 1: higher staircase for a real load, played once (no restart,
 * even after a fault) */
#define LOAD_TEST                1
#if LOAD_TEST
#define TEST_LEVELS              { 300, 500, 700, 850, 1000 }  /* per mille */
#define TEST_LEVEL_MS            8000U    /* long enough for the board to heat */
#define TEST_AUTORUN_ONCE        1
#else
#define TEST_LEVELS              { 50, 80, 110, 140, 170 }
#define TEST_LEVEL_MS            3000U
#define TEST_AUTORUN_ONCE        0
#endif
#define TEST_LEVEL0_EXTRA_MS     4000U    /* the start-up happens in the first level */
/* LONG_TEST 1 (replaces the staircase): endurance run. Starts at
 * LONG_TEST_START_THR, ramps to LONG_TEST_THROTTLE over LONG_TEST_RAMP_MS,
 * holds it LONG_TEST_S seconds, then stops for good. LONG_TEST_SAMPLES
 * interval averages go to run_now.lt[] (read run_prev.lt[] after a
 * reconnection). Current limits and thermal derating stay active. */
#define LONG_TEST                1
#define LONG_TEST_START_THR      100U     /* per mille, clean start */
#define LONG_TEST_THROTTLE       1000U    /* per mille, held (full throttle) */
#define LONG_TEST_RAMP_MS        5000U
#define LONG_TEST_S              600U     /* 10 minutes */
#define LONG_TEST_SAMPLES        20U      /* one every 30 s */
#define TEST_MOTOR_POLE_PAIRS    7U       /* rpm = erpm / pole pairs */
#define TEST_AUTORUN_OFF_MS      3000U

/* -------------------------------------------------------------- startup */
/* defaults until an auto-tune result is in flash (motor_tune.c) */
#define START_I_DEFAULT_MA       5000U
#define START_DUTY_MAX_OL        (PWM_PERIOD_TICKS * 8U / 100U)
#define START_ACCEL_SHIFT_DEFAULT 5U
#define START_DUTY_FLOOR         15U     /* PWM ticks, current regulator lower bound */

/* ---------------------------------------------------------------- beeps
 * The motor is used as a speaker. BEEP_DUTY_PM sets the loudness. */
#define BEEP_ENABLE              1
/* 40 per mille at 24 kHz; doubled at 48 kHz so the pulse left after the
 * dead time keeps the same width (same loudness) */
/* same pulse width left after the dead time as 40 per mille at 24 kHz
 * (61 ticks per 2000), whatever the frequency: same loudness */
#define BEEP_DUTY_PM             ((((61U * PWM_PERIOD_TICKS) / 1000U + PWM_DEAD_TIME_TICKS) * 1000U) \
                                  / (2U * PWM_PERIOD_TICKS))

/* ------------------------------------------------------------ auto-tune
 * AUTOTUNE_MODE 0: never (flash result or defaults)
 *               1: at power-up when no valid result is in flash
 *               2: at every power-up
 * The motor must turn freely, NO PROPELLER. */
#define AUTOTUNE_MODE            1
#define TUNE_I_TARGET_MA         5000U
#define TUNE_I_ABORT_MA          20000U
#define TUNE_OL_START_US         50000U  /* open-loop test, first step */
#define TUNE_OL_FINAL_US         1500U   /* open-loop test, final step */
#define TUNE_SYNC_STEPS          60U     /* steps held in sync at the final rate */
#define TUNE_HANDOFF_MIN_AMP     40      /* BEMF peak (LSB) at hand-off, at least */
#define TUNE_HANDOFF_SNR         4       /* and this times the parasitic floor */
/* closed-loop acceleration test: throttle step, fastest duty slew first */
#define TUNE_ACCEL_THROTTLE      150U    /* per mille */
#define TUNE_ACCEL_MS            2500U
#define TUNE_ACCEL_SLEWS         { 4, 8, 16, 32, 64 }   /* control ticks per duty tick */
#define TUNE_ACCEL_N             5

/* START_DIRECT 1: closed loop right after the alignment, blind accelerating
 * commutation until the first crossing. 0: open-loop ramp, then hand-off. */
#define START_DIRECT             0
#define START_STEP_US            10000U
#define START_BLIND_MIN_US       3300U
#define START_MISSED_MAX         96U
#define PRECHARGE_TICKS          MS_TO_TICKS(10)     /* all low sides on: bootstraps */
#define ALIGN_TICKS              MS_TO_TICKS(300)
#define ALIGN_DUTY               (PWM_PERIOD_TICKS * 3U / 100U)

/* open-loop ramp (defaults and MOTOR_TEST_OPEN_LOOP) */
#define RAMP_START_STEP_US       10000U
#define RAMP_END_STEP_US         1000U
#define RAMP_START_STEP_TICKS    US_TO_TICKS(RAMP_START_STEP_US)
#define RAMP_END_STEP_TICKS      US_TO_TICKS(RAMP_END_STEP_US)
#define RAMP_SHRINK_NUM          62U
#define RAMP_SHRINK_DEN          64U
#define RAMP_DUTY_START          (PWM_PERIOD_TICKS * 6U / 100U)
#define RAMP_DUTY_END            (PWM_PERIOD_TICKS * 7U / 100U)
#define RAMP_ZC_GOOD_NEEDED      12U   /* consecutive crossings before the hand-off */
#define RAMP_TIMEOUT_TICKS       MS_TO_TICKS(4500)

/* ----------------------------------------------------------- closed loop */
#define ZC_ADVANCE_SHIFT         3U    /* advance = period / 8 (7.5 deg) */
#define ZC_BLANK_DIV             8U    /* blanking after a commutation = period / 8 */
#define ZC_BLANK_MIN_TICKS       1U
#define ZC_HYST_LSB              4     /* ON-time method: hysteresis around Vbus/2 */
/* BEMF sampling: 1 = middle of the PWM OFF time (driven phases at 0 V,
 * floating phase = 1.5 x BEMF against ground); 0 = in the high-side pulse
 * against Vbus/2 */
#define BEMF_SAMPLE_OFFTIME      1
#define ZC_OFF_LOW_LSB           4     /* floating phase at 0 V */
#define ZC_OFF_HIGH_LSB          10    /* floating phase clearly positive (~60 mV) */
#define ZC_OFF_RAIL_MARGIN_LSB   150   /* within this of Vbus: demagnetisation clamp */
#define ZC_EARLY_MIN_LSB         30    /* ON-time method: "already crossed" margin */
#define ZC_RAIL_LSB              700   /* ON-time method: rail clamp threshold */
#define DESYNC_TIMEOUT_MS        250U  /* no crossing for this long -> desync */
#define ZC_PERIOD_MIN_TICKS      6U    /* shortest step: ~40 000 eRPM at 24 kHz, ~80 000 at 48 kHz */
#define ZC_MISSED_MAX            24U   /* consecutive blind steps -> desync */
/* speed governor at the ceiling: duty cap regulating the step period */
#define GOV_PERIOD_X16           ((ZC_PERIOD_MIN_TICKS + 1U) << 4)  /* target, 1/16 tick */
#define GOV_UPDATE_TICKS         (32U * DUTY_RATE_SCALE)   /* power of two, 750 Hz at 24 kHz */
#define GOV_STEP_MAX             2     /* duty ticks per update, at most */
#define GOV_RELEASE_MARGIN       8     /* cap released this far above the throttle duty */
#define ZC_EARLY_MAX_TICKS       TICKS_24K(48)
/* "already crossed" accepted on falling steps only when the blanking is far
 * longer than the demagnetisation, or when the ON-time gate below is valid */
#define ZC_EARLY_FALL_MIN_TICKS  TICKS_24K(32)
/* Falling steps: a demagnetising phase is held at 0 V by its low-side diode
 * even while the high side conducts, a real BEMF reads near Vbus/2 in the ON
 * time. The ON-time sample (injected rank 2, ~70 ticks after the bottom) is
 * inside the pulse from this duty up. */
#define ZC_FALL_GATE_MIN_DUTY    100U  /* PWM ticks */
#define ZC_FALL_GATE_DIV         8U    /* demagnetising while the ON sample < Vbus / 8 */
/* "already crossed": 1 = the crossing is taken at the detection, the step
 * ends half a period minus the advance later (the rotor is ahead, catch up);
 * 0 = the step keeps its nominal length (period minus advance) */
#define ZC_EARLY_AT_DETECT       1

/* duty slew: one duty tick every N control ticks */
#define DUTY_SLEW_UP_TICKS       32U   /* default, replaced by the auto-tune */
#define DUTY_SLEW_DOWN_TICKS     1U

/* ------------------------------------------------------------ protection */
/* PGA -> CMP2 -> TIM1 break. Keep it at 1: its trips are real current. */
#define HW_OC_BREAK_ENABLE       1

#define I_LIMIT_MA               25000  /* slow fold-back above this */
#define RUN_DUTY_FLOOR           PWM_DUTY_MIN  /* lowest duty the limiter and governor may set */
#define I_SW_FAULT_MA            35000  /* I_SW_FAULT_SAMPLES samples above -> fault */
/* must stay below I_SW_FAULT_MA, or the cut never acts before the fault */
#define I_PEAK_MA                30000  /* one sample above -> duty cut by 1/32 */
#define I_SW_FAULT_SAMPLES       3U
#define I_FILTER_SHIFT           4U     /* IIR 1/16 */

#define VBUS_FILTER_SHIFT        4U
#define VBUS_MIN_STARTUP_MV      6000   /* below this: no pack, no arming */
/* bridge active and the bus below this for VBUS_SAG_SAMPLES samples: the
 * supply is collapsing (current limit, long leads), stop before the gate
 * driver UVLO (5.1..6.4 V) and the chip reset */
#define VBUS_SAG_FAULT_MV        6000
#define VBUS_SAG_SAMPLES         4U
#define CELL_MV_WARN             3300   /* per cell: fold back */
#define CELL_MV_CUTOFF           3000   /* per cell: stop */

/* 1: TH1 and R24 fitted. 0: PD6 floats, thermal protection off */
#define NTC_PRESENT              1
#define TEMP_DERATE_START_C      80
#define TEMP_CUTOFF_C            100

/* desync/stall restart once the throttle is back to zero; hard faults
 * need a disarm */
#define FAULT_RETRY_DELAY_MS     500U

#if I_PEAK_MA >= I_SW_FAULT_MA
#error "I_PEAK_MA must be below I_SW_FAULT_MA"
#endif

#endif /* MOTOR_CONFIG_H */
