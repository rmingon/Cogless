/* SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Cogless-Commercial
 * Copyright (c) 2026 Ronan Mingon */
/*
 * motor_tune.h - motor auto-detection and start-up adaptation.
 *
 * The CH32M007 characterises the motor it is connected to, then derives the
 * start-up parameters (motor_start_cfg_t) and keeps them in flash:
 *   1. standstill current sweep on one step: duty that gives the start
 *      current, phase-to-phase resistance, parasitic BEMF floor;
 *   2. open-loop acceleration at regulated current, BEMF recorded at every
 *      step: BEMF constant, loss of synchronism, hand-off speed (BEMF well
 *      above the floor);
 *   3. retries with a gentler acceleration or more current if the rotor
 *      does not follow;
 *   4. result written to the last 256 B of flash, reloaded at power-up.
 * The motor must turn freely: NO PROPELLER.
 */
#ifndef MOTOR_TUNE_H
#define MOTOR_TUNE_H

#include <stdint.h>

typedef enum
{
    TUNE_IDLE = 0,
    TUNE_RUNNING,
    TUNE_OK,
    TUNE_ERR_NO_MOTOR,      /* no current even at the duty ceiling */
    TUNE_ERR_OVERCURRENT,   /* current limit or fault during the sweep */
    TUNE_ERR_NO_SYNC,       /* rotor never followed the open-loop ramp */
    TUNE_ERR_FLASH,
} tune_status_t;

typedef struct
{
    uint8_t  accel_shift;
    uint16_t i_start_ma;
    uint16_t last_period;   /* fastest step reached in sync, ticks */
    uint32_t k_ref;         /* BEMF peak x period, reference */
    uint8_t  result;        /* 1 = in sync at the final rate, 0 = lost sync */
    uint16_t steps;
} tune_attempt_t;

#define TUNE_MAX_ATTEMPTS 6

typedef struct
{
    uint8_t  slew;          /* control ticks per duty tick tried */
    uint8_t  result;        /* 0 = ok, else motor_fault_t (3 desync, 2 SW OC, 1 HW OC, 4 stall), 8 = did not accelerate, 9 = never reached closed loop */
    uint16_t t90_ms;        /* time from the throttle step to 90 % of the final speed */
    uint32_t erpm_final;    /* mean over the last 300 ms */
    int32_t  i_peak_ma;     /* largest filtered current seen */
    uint16_t duty_final;    /* per mille */
} tune_accel_t;

#define TUNE_ACCEL_MAX 5

/* Everything the tune measured, readable in the debugger as tune_res. */
typedef struct
{
    tune_status_t status;
    uint16_t vbus_mv;
    uint16_t duty_start;    /* PWM ticks for TUNE_I_TARGET_MA at standstill */
    int32_t  i_meas_ma;     /* current measured at duty_start */
    uint16_t r_mohm;        /* phase-to-phase resistance estimate */
    uint16_t floor_lsb;     /* parasitic OFF-time reading with current, LSB */
    uint32_t bemf_k;        /* BEMF peak (LSB) x step period (ticks) */
    uint16_t handoff_period;
    uint16_t stop_duty;     /* sweep duty when it stopped on an error */
    int32_t  stop_ma;       /* current measured there */
    uint8_t  stop_fault;    /* motor_fault_t at that moment (2 = software OC, 1 = CMP2 break) */
    uint8_t  n_attempts;
    tune_attempt_t att[TUNE_MAX_ATTEMPTS];
    uint8_t  slew_chosen;   /* stored acceleration setting, 0 = test failed (default kept) */
    uint8_t  n_accel;
    tune_accel_t acc[TUNE_ACCEL_MAX];
} tune_result_t;

extern volatile tune_result_t tune_res;

/* Load a stored result into the motor start config. Returns 1 if valid. */
uint8_t tune_load(void);
/* Run the full auto-tune (blocking, ~10..30 s), apply and store the result.
 * led(on) is called to show progress. Returns the final status. */
tune_status_t tune_run(void (*led)(uint8_t on));
/* Erase the stored result (next power-up re-tunes when AUTOTUNE_MODE = 1). */
void tune_forget(void);

#endif /* MOTOR_TUNE_H */
