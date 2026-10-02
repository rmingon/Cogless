/* SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Cogless-Commercial
 * Copyright (c) 2026 Ronan Mingon */
/*
 * motor.h - sensorless six-step (trapezoidal) BLDC controller for the
 * Cogless ESC (CH32M007G8R6 with integrated 3-phase gate driver).
 *
 * Fast loop at the PWM rate (PWM_FREQ_HZ, 48 kHz), synchronised with TIM1:
 *   - shunt current through the internal PGA and bus voltage, in the
 *     high-side pulse
 *   - floating-phase BEMF in the middle of the PWM OFF time
 *   - zero-crossing detection, commutation, current-regulated open-loop start
 *   - duty slew, current fold-back and peak limit, speed ceiling
 * Hardware over-current: PGA -> CMP2 -> TIM1 break.
 *
 * The control loop runs in interrupt context; the init, command and
 * debug functions are called from main().
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef enum
{
    MOTOR_STOPPED = 0,   /* all six FETs off, rotor coasting            */
    MOTOR_PRECHARGE,     /* all low-side on: bootstrap capacitors fill   */
    MOTOR_ALIGN,         /* fixed step energised, rotor parks            */
    MOTOR_RAMP,          /* forced commutation, waiting for BEMF lock    */
    MOTOR_RUN,           /* closed loop on zero crossings                */
    MOTOR_BRAKE,         /* all low-side on                              */
    MOTOR_BEEP,          /* tone: current reversed at audio rate, rotor still */
    MOTOR_FAULT,         /* outputs off, see motor_fault()               */
} motor_state_t;

typedef enum
{
    MOTOR_FAULT_NONE = 0,
    MOTOR_FAULT_HW_OVERCURRENT,  /* CMP2 tripped the TIM1 break input     */
    MOTOR_FAULT_SW_OVERCURRENT,  /* ADC current above I_SW_FAULT_MA       */
    MOTOR_FAULT_DESYNC,          /* lost zero crossings while running     */
    MOTOR_FAULT_STALL,           /* start ramp never locked               */
    MOTOR_FAULT_UNDERVOLTAGE,    /* raised by main() from the bus voltage */
    MOTOR_FAULT_OVERTEMP,        /* raised by main() from the NTC         */
    MOTOR_FAULT_OVERRUN,         /* control loop too late for too long    */
} motor_fault_t;

/* Start-up parameters, normally produced by the auto-tune (motor_tune.c)
 * and kept in flash. Periods are in control ticks (1 / PWM_FREQ_HZ), duty
 * in PWM ticks (PWM_PERIOD_TICKS = 100 %). */
typedef struct
{
    uint16_t i_start_ma;      /* current regulated during alignment and open-loop ramp */
    uint16_t duty_start;      /* duty that gives i_start at standstill */
    uint16_t duty_max_ol;     /* duty ceiling during the open-loop ramp */
    uint16_t ol_start_period; /* first open-loop step */
    uint16_t handoff_period;  /* ramp final step; zero crossings counted from here */
    uint8_t  accel_shift;     /* period -= period >> accel_shift at each step */
    uint8_t  ol_hold;         /* 1 = tuning: never hand off, hold the final rate */
    uint8_t  slew_up_ticks;   /* closed loop acceleration: +1 duty tick every N x DUTY_RATE_SCALE control ticks
                               * (N is the 24 kHz value, same percent per second at 48 kHz) */
    uint8_t  reserved;
} motor_start_cfg_t;

void          motor_set_start_cfg(const motor_start_cfg_t *c);
void          motor_get_start_cfg(motor_start_cfg_t *c);

/* One entry per commutation (see motor.c), the last MOTOR_TRACE_LEN kept. */
typedef struct
{
    uint16_t period;    /* step period, control ticks       */
    uint16_t duty;      /* applied duty, PWM ticks           */
    int16_t  i_lsb;     /* filtered shunt current, LSB       */
    uint8_t  fold;
    uint8_t  tag;       /* step that ENDED 0..5, + 0x10 if its crossing was "already crossed",
                         * + 0x20 if commutated blind (no crossing); 0x40 closed loop
                         * entered; 0x80 | fault */
    uint8_t  zc;
    int16_t  bmin;      /* floating phase over the step (OFF-time raw LSB) */
    int16_t  bmax;
    uint8_t  zc_at;
} motor_trace_t;
#define MOTOR_TRACE_LEN 96
uint16_t      motor_trace_count(void);
/* copy entry number n (0 = first since motor_start); returns 0 if overwritten */
uint8_t       motor_trace_get(uint16_t n, motor_trace_t *out);

/* Returns 0 when the control loop is running, 1 if it never ran
 * (ADC / TIM1 trigger problem): the caller must not start the motor. */
uint8_t       motor_init(void);

/* Commands (main context) */
void          motor_start(void);                 /* STOPPED -> PRECHARGE -> ... */
void          motor_stop(void);                  /* coast                      */
void          motor_brake(void);                 /* all low-side on            */
void          motor_set_throttle(uint16_t permille);   /* 0..1000              */
void          motor_set_direction(uint8_t reverse);    /* only while STOPPED   */
void          motor_set_duty_limit(uint16_t permille); /* thermal / voltage derating */
void          motor_raise_fault(motor_fault_t fault);
/* Tone through the windings (motor as a speaker), only from STOPPED.
 * Non-blocking: the bridge goes back to STOPPED after ms. */
void          motor_beep(uint16_t freq_hz, uint16_t ms, uint16_t duty_permille);
uint8_t       motor_beep_busy(void);
void          motor_clear_fault(void);           /* FAULT -> STOPPED           */

/* Status */
motor_state_t motor_state(void);
motor_fault_t motor_fault(void);
uint32_t      motor_fault_time_ms(void);
uint32_t      motor_erpm(void);                  /* electrical rpm             */
int32_t       motor_current_ma(void);            /* filtered shunt current     */
uint16_t      motor_vbus_mv(void);               /* filtered bus voltage       */
uint16_t      motor_duty(void);                  /* current duty, 0..1000      */
uint16_t      motor_peak_hits(void);             /* cycle-by-cycle limit count since start */
/* closed-loop commutations since start: crossing seen, "already crossed", blind */
void          motor_zc_counts(uint16_t *real, uint16_t *early, uint16_t *blind);
uint16_t      motor_fold_max_pm(void);           /* largest current fold-back since the last call, per mille */
typedef struct
{
    uint16_t cc_overrun;    /* control ticks lost since motor_start */
    uint16_t cc_isr_max;    /* longest control interrupt, timer ticks */
    uint16_t cc_late;       /* current samples ignored (late interrupt) */
    uint8_t  fault_state;   /* motor_state_t when the last fault was raised */
    uint16_t fault_duty;    /* duty (PWM ticks) at the last fault */
    int32_t  fault_i_ma;    /* shunt current at the last fault */
} motor_diag_t;
void          motor_get_diag(motor_diag_t *d);
uint16_t      motor_throttle(void);              /* last commanded, 0..1000    */
uint32_t      motor_millis(void);                /* ms since boot (control tick)   */

/* Probe: freeze the outputs in a fixed state, bypassing the state
 * machine. mode 0 = all off, 1 = all low-side on (brake), 2..7 = six-step
 * table entries 0..5 with the given PWM duty (per mille), 8/9/10 = only the
 * U/V/W low side forced on (no high side, no current). */
void          motor_debug_outputs(uint8_t mode, uint16_t duty_permille);

/* Probe: last raw ADC value of the current channel (PGA output, IN9). */
uint16_t      motor_debug_iraw(void);
/* Probe: last floating-phase samples (in the pulse, in the OFF time) and bus. */
void          motor_debug_bemf(uint16_t *on, uint16_t *off, uint16_t *vbus);

/* Slow ADC read of the NTC divider (regular group, blocking < 20 us). */
uint16_t      motor_read_ntc_raw(void);

#endif /* MOTOR_H */
