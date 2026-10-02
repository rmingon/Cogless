/* SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Cogless-Commercial
 * Copyright (c) 2026 Ronan Mingon */
/*
 * main.c - Cogless ESC firmware (CH32M007G8R6, integrated 3-phase gate
 * driver): sensorless six-step BLDC control.
 *
 *   board.h        pin map and hardware scaling
 *   motor_config.h tunables: PWM, start-up, limits, test modes
 *   motor.c        48 kHz control loop: PWM, current sense, BEMF commutation
 *   motor_tune.c   motor auto-detection, start-up parameters in flash
 *   rc_input.c     servo PWM throttle on J2 (PA1)
 *   telemetry.c    SDI text output (safe without a debugger)
 *
 * Power-up: beeps, auto-tune if no result is stored, then the 1 ms loop:
 * arming, voltage and temperature derating, LED, telemetry, watchdog.
 * Arming: valid throttle at minimum for 1 s (or TEST_AUTORUN), battery present.
 * Desync and stall retry after 500 ms at zero throttle; the other faults
 * need the throttle held low again.
 */

#include <string.h>
#include "debug.h"
#include "board.h"
#include "motor.h"
#include "motor_config.h"
#include "rc_input.h"
#include "telemetry.h"
#include "motor_tune.h"

#ifndef MOTOR_REVERSE
#define MOTOR_REVERSE        0
#endif

#define HARD_FAULT_CLEAR_MS  2000U
#define TELEMETRY_PERIOD_MS  200U
#define NTC_PERIOD_MS        100U
#define LED_SLOT_MS          125U      /* 16 slots = 2 s pattern */

/* ------------------------------------------------------------------ LED */
static void led_init(void)
{
    GPIO_InitTypeDef g = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC, ENABLE);
    g.GPIO_Pin   = BOARD_LED_PIN;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(BOARD_LED_PORT, &g);
}

void led_set(uint8_t on)
{
    if (on) GPIO_SetBits(BOARD_LED_PORT, BOARD_LED_PIN);
    else    GPIO_ResetBits(BOARD_LED_PORT, BOARD_LED_PIN);
}

/* 16-bit pattern, one bit per 125 ms, MSB first */
static uint16_t led_pattern(uint8_t armed, uint8_t rc_ok, motor_state_t st, motor_fault_t f)
{
    if (st == MOTOR_FAULT)
    {
        /* f SHORT (125 ms) blinks then a pause: 1010..00 */
        uint16_t p = 0;
        uint8_t i;
        for (i = 0; i < f && i < 7; i++)
        {
            p |= (uint16_t)(0x8000U >> (2U * i));
        }
        return p;
    }
    if (!rc_ok)  return 0xF000;                 /* one LONG (500 ms) blink per 2 s */
    if (!armed)  return 0xFF00;                 /* slow blink, 1 s on / 1 s off   */
    if (st == MOTOR_STOPPED || st == MOTOR_BRAKE) return 0xFFFF; /* solid */
    return 0xAAAA;                              /* running: 4 Hz            */
}

/* ---------------------------------------------------------------- beeps */
/* { frequency Hz (0 = silence), duration ms } */
typedef struct { uint16_t f; uint16_t ms; } note_t;

static const note_t mel_power[]   = { {1046, 120}, {0, 40}, {1318, 120}, {0, 40}, {1568, 180} };
static const note_t mel_tune[]    = { {1568, 80}, {0, 80}, {1568, 80} };
static const note_t mel_tune_ok[] = { {1318, 120}, {0, 40}, {2093, 220} };
static const note_t mel_fail[]    = { {523, 600} };
static const note_t mel_armed[]   = { {1568, 300} };

static void play(const note_t *mel, uint8_t n)
{
#if BEEP_ENABLE
    uint8_t i;
    for (i = 0; i < n; i++)
    {
        if (mel[i].f == 0U)
        {
            Delay_Ms(mel[i].ms);
            continue;
        }
        motor_beep(mel[i].f, mel[i].ms, BEEP_DUTY_PM);
        while (motor_beep_busy())
        {
            IWDG_ReloadCounter();          /* harmless before the watchdog is on */
        }
    }
#else
    (void)mel; (void)n;
#endif
}
#define PLAY(m) play((m), (uint8_t)(sizeof(m) / sizeof((m)[0])))

/* ------------------------------------------------------------------ NTC */
#if NTC_PRESENT
volatile int16_t board_temp_c = 25;   /* last NTC reading, for the debugger */

static int16_t ntc_to_celsius(uint16_t adc)
{
    static const uint16_t tab[] = BOARD_NTC_TABLE;
    const uint8_t n = sizeof(tab) / sizeof(tab[0]);
    uint8_t i;

    if (adc <= tab[0])     return BOARD_NTC_TABLE_T_MIN_C;
    if (adc >= tab[n - 1]) return (int16_t)(BOARD_NTC_TABLE_T_MIN_C + (n - 1) * BOARD_NTC_TABLE_STEP_C);
    for (i = 0; i < n - 1; i++)
    {
        if (adc < tab[i + 1])
        {
            uint16_t span = tab[i + 1] - tab[i];
            return (int16_t)(BOARD_NTC_TABLE_T_MIN_C + i * BOARD_NTC_TABLE_STEP_C
                             + ((uint32_t)(adc - tab[i]) * BOARD_NTC_TABLE_STEP_C) / span);
        }
    }
    return 0;
}
#endif

/* -------------------------------------------------------------- watchdog */
static void iwdg_init(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_32);      /* 128 kHz / 32 = 4 kHz */
    IWDG_SetReload(400);                       /* 100 ms               */
    IWDG_ReloadCounter();
    IWDG_Enable();
}

/* ------------------------------------------------------------ telemetry */
static void telemetry_line(uint8_t armed, uint8_t cells, int16_t temp_c)
{
    tel_puts("st=");   tel_put_uint(motor_state());
    tel_puts(" f=");   tel_put_uint(motor_fault());
    tel_puts(" arm="); tel_put_uint(armed);
    tel_puts(" rc=");  tel_put_uint(rc_pulse_us());
    tel_puts(" thr="); tel_put_uint(motor_throttle());
    tel_puts(" duty=");tel_put_uint(motor_duty());
    tel_puts(" erpm=");tel_put_uint(motor_erpm());
    tel_puts(" I=");   tel_put_int(motor_current_ma());
    tel_puts("mA V="); tel_put_uint(motor_vbus_mv());
    tel_puts("mV ");   tel_put_uint(cells);
    tel_puts("S T=");  tel_put_int(temp_c);
    tel_puts("C\r\n");
}

/* --------------------------------------------------------- driver probe */
#if DRIVER_PIN_TEST == 6
volatile uint8_t  lsreg_step;          /* 1..4, step being shown */
volatile uint16_t lsreg_ccer, lsreg_chctlr1, lsreg_chctlr2, lsreg_bdtr, lsreg_ctlr2;
volatile uint32_t lsreg_afio_pcfr1, lsreg_gpioa_cfglr, lsreg_gpiod_cfglr;

static void ls_tim1_test(void)
{
    static const uint8_t modes[4] = { 1, 8, 9, 10 };
    uint8_t step = 0;

    if (motor_init())
    {
        while (1) { led_set(1); Delay_Ms(50); led_set(0); Delay_Ms(50); }
    }
    while (1)
    {
        uint32_t t = 0;
        uint8_t  i;

        motor_debug_outputs(modes[step], 0);
        lsreg_step        = (uint8_t)(step + 1U);
        lsreg_ccer        = TIM1->CCER;
        lsreg_chctlr1     = TIM1->CHCTLR1;
        lsreg_chctlr2     = TIM1->CHCTLR2;
        lsreg_bdtr        = TIM1->BDTR;
        lsreg_ctlr2       = TIM1->CTLR2;
        lsreg_afio_pcfr1  = AFIO->PCFR1;
        lsreg_gpioa_cfglr = GPIOA->CFGLR;
        lsreg_gpiod_cfglr = GPIOD->CFGLR;
        for (i = 0; i <= step; i++)
        {
            led_set(1); Delay_Ms(100);
            led_set(0); Delay_Ms(200);
            t += 300;
        }
        Delay_Ms(4000U - t);
        step = (uint8_t)((step + 1U) % 4U);
    }
}
#elif DRIVER_PIN_TEST == 5
volatile uint8_t hs_check_phase;       /* 1 = V under test, 2 = W, 3 = done */

static void hs_check_hold(uint8_t mode, uint8_t blink)
{
    uint32_t t;
    motor_debug_outputs(mode, HS_CHECK_DUTY);
    for (t = 0; t < HS_CHECK_MS; t += 125)
    {
        if (motor_state() == MOTOR_FAULT)
        {
            return;
        }
        led_set(blink ? (uint8_t)((t / 125U) & 1U) : 1U);
        Delay_Ms(125);
    }
}

static void hs_check(void)
{
    if (motor_init())
    {
        while (1) { led_set(1); Delay_Ms(50); led_set(0); Delay_Ms(50); }
    }
    Delay_Ms(2000);
    hs_check_phase = 1;
    hs_check_hold((uint8_t)(2U + HS_CHECK_ENTRY_A), 0);
    hs_check_phase = 2;
    hs_check_hold((uint8_t)(2U + HS_CHECK_ENTRY_B), 1);
    hs_check_phase = 3;
    motor_debug_outputs(0, 0);
    while (1)
    {
        led_set(1); Delay_Ms(500);
        led_set(0); Delay_Ms(500);
    }
}
#elif DRIVER_PIN_TEST == 4
volatile int32_t  sweep_ma[SWEEP_POINTS];
volatile uint8_t  sweep_done;          /* number of points completed */
volatile uint8_t  sweep_aborted;
volatile int32_t  sweep_zero;          /* PGA output at zero current, LSB */
volatile uint16_t sweep_vbus_idle;     /* mV, bridge off */
volatile uint16_t sweep_vbus_min[SWEEP_POINTS]; /* lowest bus voltage per point, mV */
volatile uint8_t  sweep_abort_reason;  /* 1 = shunt reading, 2 = bus sag, 3 = fault */

static void current_sweep(void)
{
    static const uint16_t duty[SWEEP_POINTS] = SWEEP_DUTIES;
    uint8_t k;

    if (motor_init())
    {
        while (1) { led_set(1); Delay_Ms(50); led_set(0); Delay_Ms(50); }
    }
    Delay_Ms(2000);                        /* time to stand back */
    {
        uint32_t z = 0;                    /* zero-current baseline, bridge off */
        uint8_t  j;
        for (j = 0; j < 64; j++) { Delay_Ms(1); z += motor_debug_iraw(); }
        sweep_zero = (int32_t)(z / 64U);
        sweep_vbus_idle = motor_vbus_mv();
    }
    for (k = 0; k < SWEEP_POINTS; k++)
    {
        int32_t  sum = 0;
        uint16_t i, n = 0;
        uint16_t vmin = 0xFFFF;

        motor_debug_outputs(2, duty[k]);   /* entry 0: U PWM, V low */
        led_set(1);
        for (i = 0; i < SWEEP_POINT_MS; i++)
        {
            int32_t  ma;
            uint16_t v;
            Delay_Ms(1);
            v = motor_vbus_mv();
            if (v < vmin) vmin = v;
            if (v + SWEEP_ABORT_SAG_MV < sweep_vbus_idle)
            {
                sweep_abort_reason = 2;
                sweep_aborted = 1;
                break;
            }
            if (i < 30)
            {
                continue;                  /* let the current settle */
            }
            ma = ((int32_t)motor_debug_iraw() - sweep_zero)
                 * BOARD_I_MA_PER_LSB_NUM / BOARD_I_MA_PER_LSB_DEN;
            sum += ma;
            n++;
            if (ma > SWEEP_ABORT_MA || motor_state() == MOTOR_FAULT)
            {
                sweep_abort_reason = (motor_state() == MOTOR_FAULT) ? 3 : 1;
                sweep_aborted = 1;
                break;
            }
        }
        sweep_ma[k] = n ? sum / n : 0;
        sweep_vbus_min[k] = vmin;
        sweep_done = (uint8_t)(k + 1U);
        if (sweep_aborted)
        {
            break;
        }
    }
    motor_debug_outputs(0, 0);             /* bridge off for good */
    if (motor_state() == MOTOR_FAULT && !sweep_abort_reason)
    {
        sweep_abort_reason = 3;
    }
    while (1)
    {
        led_set(1); Delay_Ms(500);
        led_set(0); Delay_Ms(500);
    }
}
#elif DRIVER_PIN_TEST == 3
volatile uint16_t pga_probe[4];
volatile uint8_t  pga_probe_done;

static uint16_t pga_probe_read(uint32_t mode1, uint32_t vbsel)
{
    uint32_t sum = 0;
    uint8_t  i;

    OPA_Unlock();
    OPA->CTLR1 = (OPA->CTLR1 & ~((3UL << 1) | (1UL << 17))) | (mode1 << 1) | (vbsel << 17);
    Delay_Ms(20);                          /* settle */
    for (i = 0; i < 64; i++)
    {
        Delay_Ms(1);
        sum += motor_debug_iraw();
    }
    return (uint16_t)(sum / 64U);
}

static void pga_path_test(void)
{
    if (motor_init())                      /* bridge stays off (MOE = 0) */
    {
        while (1) { led_set(1); Delay_Ms(50); led_set(0); Delay_Ms(50); }
    }
    pga_probe[0] = pga_probe_read(3, 0);
    pga_probe[1] = pga_probe_read(3, 1);
    pga_probe[2] = pga_probe_read(1, 0);
    pga_probe[3] = pga_probe_read(1, 1);
    pga_probe_done = 1;
    while (1)                              /* slow blink: results ready */
    {
        led_set(1); Delay_Ms(500);
        led_set(0); Delay_Ms(500);
    }
}
#elif DRIVER_PIN_TEST == 2
/* Frozen six-step states through motor_debug_outputs(): the control loop
 * state machine is bypassed, only TIM1 registers are written. With
 * DRIVER_PIN_TEST_HOLD = 0 the six steps are applied in order, one every
 * DRIVER_PIN_TEST_STEP_MS; the LED blinks (step + 1) times at each change.
 * With HOLD = N the probe stays on six-step entry N - 1. */
volatile int32_t  step_lsb[6];          /* mean shunt reading per six-step entry, LSB */
volatile uint16_t step_boff[6];         /* floating phase, OFF time, raw LSB (0 V expected at standstill) */
volatile int16_t  step_bon[6];          /* floating phase, in the pulse, minus Vbus/2 (0 expected) */
volatile int32_t  step_ma[6];           /* same in mA */
volatile int32_t  probe_zero;
volatile uint8_t  probe_steps_done;

static void driver_tim1_test(void)
{
    uint8_t step = 0;

    if (motor_init())
    {
        while (1) { led_set(1); Delay_Ms(50); led_set(0); Delay_Ms(50); }
    }
    if (DRIVER_PIN_TEST_HOLD)
    {
        /* one-shot: hold one step for DRIVER_PIN_TEST_STEP_MS, then off for good */
        step = (uint8_t)((DRIVER_PIN_TEST_HOLD - 1U) % 6U);
        Delay_Ms(2000);                    /* time to stand back */
        motor_debug_outputs((uint8_t)(2U + step), DRIVER_PIN_TEST_DUTY);
        led_set(1);
        Delay_Ms(DRIVER_PIN_TEST_STEP_MS);
        motor_debug_outputs(0, 0);
        while (1)
        {
            led_set(1); Delay_Ms(500);
            led_set(0); Delay_Ms(500);
        }
    }
    /* cycle mode: two electrical revolutions, current recorded per step,
     * then the bridge is turned off for good */
    {
        uint32_t z = 0;
        uint8_t  j, n;

        Delay_Ms(2000);                    /* time to stand back */
        for (j = 0; j < 64; j++) { Delay_Ms(1); z += motor_debug_iraw(); }
        probe_zero = (int32_t)(z / 64U);

        for (n = 0; n < 12; n++)
        {
            int32_t  sum = 0, s_on = 0;
            uint32_t s_off = 0;
            uint16_t i, cnt = 0;

            step = (uint8_t)(n % 6U);
            motor_debug_outputs((uint8_t)(2U + step), DRIVER_PIN_TEST_DUTY);
            led_set((uint8_t)(n & 1U));
            for (i = 0; i < DRIVER_PIN_TEST_CYCLE_MS; i++)
            {
                Delay_Ms(1);
                if (motor_state() == MOTOR_FAULT)
                {
                    break;
                }
                if (i >= 300)                  /* rotor settled on the step */
                {
                    uint16_t bon, boff, vb;
                    motor_debug_bemf(&bon, &boff, &vb);
                    sum   += (int32_t)motor_debug_iraw() - probe_zero;
                    s_off += boff;
                    s_on  += (int32_t)bon - (int32_t)(vb / 2U);
                    cnt++;
                }
            }
            step_lsb[step]  = cnt ? sum / cnt : 0;
            step_boff[step] = cnt ? (uint16_t)(s_off / cnt) : 0;
            step_bon[step]  = cnt ? (int16_t)(s_on / (int32_t)cnt) : 0;
            step_ma[step]  = step_lsb[step] * BOARD_I_MA_PER_LSB_NUM / BOARD_I_MA_PER_LSB_DEN;
            probe_steps_done = (uint8_t)(n + 1U);
            if (motor_state() == MOTOR_FAULT)
            {
                break;
            }
        }
        motor_debug_outputs(0, 0);
        while (1)
        {
            led_set(1); Delay_Ms(500);
            led_set(0); Delay_Ms(500);
        }
    }
}
#elif DRIVER_PIN_TEST == 1
static void driver_pin_test(void)
{
    GPIO_InitTypeDef g = {0};
    uint8_t step = 0;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_AFIO | RCC_PB2Periph_GPIOA | RCC_PB2Periph_GPIOB
                          | RCC_PB2Periph_GPIOD, ENABLE);
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    g.GPIO_Pin = BOARD_LS_U_PIN; GPIO_Init(BOARD_LS_U_PORT, &g);
    g.GPIO_Pin = BOARD_LS_V_PIN; GPIO_Init(BOARD_LS_V_PORT, &g);
    g.GPIO_Pin = BOARD_LS_W_PIN; GPIO_Init(BOARD_LS_W_PORT, &g);
    g.GPIO_Pin = BOARD_HS_U_PIN; GPIO_Init(BOARD_HS_U_PORT, &g);
    g.GPIO_Pin = BOARD_HS_V_PIN; GPIO_Init(BOARD_HS_V_PORT, &g);
    g.GPIO_Pin = BOARD_HS_W_PIN; GPIO_Init(BOARD_HS_W_PORT, &g);

    while (1)
    {
        /* low-side inputs only: a high side is never switched on here, so no
         * current can flow whatever is connected (no shoot-through step). */
        uint8_t lu = 0, lv = 0, lw = 0, hu = 0;
        uint32_t t;

        switch (step)
        {
        case 0: break;                     /* 1 blink: all inputs low   */
        case 1: lu = 1; break;             /* 2 blinks: LIN1 (PA0) high */
        case 2: lv = 1; break;             /* 3 blinks: LIN2 (PA2) high */
        case 3: lw = 1; break;             /* 4 blinks: LIN3 (PD0) high */
        case 4: lu = lv = lw = 1; break;   /* 5 blinks: all LIN high    */
        default: break;
        }
        GPIO_WriteBit(BOARD_HS_U_PORT, BOARD_HS_U_PIN, Bit_RESET);
        GPIO_WriteBit(BOARD_HS_V_PORT, BOARD_HS_V_PIN, Bit_RESET);
        GPIO_WriteBit(BOARD_HS_W_PORT, BOARD_HS_W_PIN, Bit_RESET);
        GPIO_WriteBit(BOARD_LS_U_PORT, BOARD_LS_U_PIN, lu ? Bit_SET : Bit_RESET);
        GPIO_WriteBit(BOARD_LS_V_PORT, BOARD_LS_V_PIN, lv ? Bit_SET : Bit_RESET);
        GPIO_WriteBit(BOARD_LS_W_PORT, BOARD_LS_W_PIN, lw ? Bit_SET : Bit_RESET);
        GPIO_WriteBit(BOARD_HS_U_PORT, BOARD_HS_U_PIN, hu ? Bit_SET : Bit_RESET);

        /* LED: (step + 1) short blinks, then hold for the rest of the step */
        for (t = 0; t < DRIVER_PIN_TEST_STEP_MS; )
        {
            uint8_t i;
            for (i = 0; i <= step; i++)
            {
                led_set(1); Delay_Ms(100);
                led_set(0); Delay_Ms(200);
                t += 300;
            }
            Delay_Ms(1000);
            t += 1000;
        }
        step = (uint8_t)((step + 1U) % 5U);
    }
}
#endif

#if TEST_AUTORUN
typedef struct
{
    uint16_t thr;       /* throttle of the level, per mille */
    uint16_t duty_pm;   /* applied duty at the end of the level, per mille */
    uint32_t erpm;
    uint32_t rpm;       /* mechanical, erpm / TEST_MOTOR_POLE_PAIRS */
    int32_t  i_ma;      /* filtered motor current at the end of the level */
    int32_t  i_avg_ma;  /* mean motor current over the level */
    int32_t  i_peak_ma; /* largest filtered motor current in the level */
    int32_t  ibat_ma;   /* mean battery current estimate (duty x motor current) */
    uint16_t vbus_mv;
    uint16_t vbus_min_mv; /* lowest bus voltage in the level (battery sag) */
    uint16_t peak_hits; /* cycle-by-cycle current limit hits during the level */
    int16_t  temp_max_c; /* hottest board temperature seen in the level (NTC) */
    uint16_t fold_max_pm; /* largest current fold-back in the level, per mille of duty */
    uint16_t zc_real;   /* commutations on a crossing seen happening */
    uint16_t zc_early;  /* commutations on an "already crossed" sample */
    uint16_t zc_blind;  /* commutations without any crossing */
    uint8_t  state;     /* motor_state_t, 4 = RUN */
    uint8_t  fault;     /* motor_fault_t, 0 = none */
} level_log_t;
#if !LONG_TEST
static int8_t level_prev = -1;
static int64_t lv_sum_i, lv_sum_ibat;
static uint32_t lv_n;
static int32_t lv_peak;
static uint16_t lv_vmin = 0xFFFF, lv_hits0;
static int16_t lv_tmax = -100;
static uint16_t lv_zr0, lv_ze0, lv_zb0;
#endif

/* one endurance-test interval (LONG_TEST) */
typedef struct
{
    uint16_t t_s;          /* end of the interval, seconds since full throttle */
    int16_t  temp_max_c;   /* hottest board temperature in the interval */
    uint16_t i_avg_ma;     /* motor (shunt) current, mean */
    uint16_t i_max_ma;     /* filtered motor current, max */
    uint16_t ibat_ma;      /* battery current estimate (duty x motor current), mean */
    uint16_t vbus_mv;      /* mean */
    uint16_t vbus_min_mv;
    uint16_t rpm;          /* mechanical, mean */
    uint16_t duty_pm;      /* applied duty, mean */
    uint8_t  state;        /* motor_state_t at the end, 4 = RUN */
    uint8_t  fault;        /* motor_fault_t at the end */
} lt_sample_t;
#endif

/* ------------------------------------------------------------- run log
 * Kept in .noinit RAM, which the start-up code does not clear: it survives
 * a reset (brown-out, watchdog, debugger). At each boot the previous run is
 * copied to run_prev, so it can be read with the debugger after the fact,
 * even when the WCH-Link dropped during the run. */
#define RUN_LOG_MAGIC  0xC0661E55UL
typedef struct
{
    uint32_t magic;
    uint16_t boot_count;    /* boots since the log was created */
    uint8_t  reset_cause;   /* RSTSCKR >> 24 at this boot, see main() */
    uint8_t  phase;         /* 1 boot, 2 auto-tune, 3 main loop, 4 test running, 5 test done */
    uint32_t ms_last;       /* last time stamp written (ms since this boot) */
    uint8_t  tune_status;   /* tune_status_t */
    uint8_t  tune_stop_fault;
    uint8_t  fault;         /* last fault of the run, motor_fault_t */
    uint8_t  fault_state;   /* motor_state_t when it was raised */
    uint16_t fault_duty;    /* PWM ticks */
    int32_t  fault_i_ma;
    uint32_t fault_ms;      /* motor_millis() when it was raised */
    uint8_t  fault_first;   /* first fault of the run */
    uint8_t  fault_count;   /* faults during the run */
    uint16_t vbus_min_mv;   /* lowest bus voltage with the bridge on */
    int16_t  temp_max_c;
    uint16_t cc_overrun;
    uint16_t cc_isr_max;
    uint16_t cc_late;
    uint16_t zc_real, zc_early, zc_blind;
#if TEST_AUTORUN && LONG_TEST
    uint8_t     lt_n;              /* intervals written */
    lt_sample_t lt[LONG_TEST_SAMPLES];
#elif TEST_AUTORUN
    level_log_t level[TEST_LEVELS_N];
#endif
} run_log_t;
__attribute__((section(".noinit"))) volatile run_log_t run_now;
volatile run_log_t run_prev;      /* the previous run, read this one */

/* The last interesting run (a test ran, a fault, or a reset during the
 * auto-tune) is also written to flash at the next boot, so it survives a
 * power-off. Watch expression: *run_flash */
#define RUN_FLASH_ADDR  (0x08000000UL + 0xF500UL)   /* excluded in Ld/Link.ld */
#define RUN_FLASH_SIZE  512U
__attribute__((used)) const volatile run_log_t *const run_flash =
    (const volatile run_log_t *)RUN_FLASH_ADDR;
typedef char run_log_fits_flash[(sizeof(run_log_t) <= RUN_FLASH_SIZE) ? 1 : -1];

/* bridge off only: the flash stalls the CPU (and the control loop) for a
 * few ms; 512 B are written from src, the tail past the struct is don't-care */
static void run_log_save(const volatile run_log_t *src)
{
    __disable_irq();
    if (FLASH_ROM_ERASE(RUN_FLASH_ADDR, RUN_FLASH_SIZE) == FLASH_COMPLETE)
    {
        (void)FLASH_ROM_WRITE(RUN_FLASH_ADDR, (uint32_t *)src, RUN_FLASH_SIZE);
    }
    __enable_irq();
}

static void run_log_boot(uint8_t cause)
{
    uint16_t boots = 1;
    if (run_now.magic == RUN_LOG_MAGIC)
    {
        memcpy((void *)&run_prev, (const void *)&run_now, sizeof(run_log_t));
        boots = (uint16_t)(run_now.boot_count + 1U);
        /* a run cut short by a reset (test or auto-tune in progress): the
         * fault and end-of-test saves below did not happen, save it now */
        if (run_prev.phase == 4U || run_prev.phase == 2U)
        {
            run_log_save(&run_prev);       /* bridge not started yet */
        }
    }
    (void)run_flash->magic;            /* keeps the debugger pointer in the image */
    memset((void *)&run_now, 0, sizeof(run_log_t));
    run_now.magic       = RUN_LOG_MAGIC;
    run_now.boot_count  = boots;
    run_now.reset_cause = cause;
    run_now.phase       = 1;
    run_now.vbus_min_mv = 0xFFFF;
    run_now.temp_max_c  = -100;
}

static void run_log_counters(void)
{
    motor_diag_t d;
    uint16_t zr, ze, zb;
    motor_get_diag(&d);
    motor_zc_counts(&zr, &ze, &zb);
    run_now.cc_overrun = d.cc_overrun;
    run_now.cc_isr_max = d.cc_isr_max;
    run_now.cc_late    = d.cc_late;
    run_now.zc_real    = zr;
    run_now.zc_early   = ze;
    run_now.zc_blind   = zb;
}

/* LED callback of the auto-tune: also time-stamps the run log */
static void tune_led(uint8_t on)
{
    led_set(on);
    run_now.ms_last = motor_millis();
}

#if TEST_AUTORUN && LONG_TEST
/* t_ms: time since the test start; returns the throttle (per mille) */
static uint16_t long_test_step(uint32_t t_ms, int16_t temp_c)
{
    static uint32_t n, sum_i, sum_ibat, sum_v, sum_rpm, sum_duty;
    static uint16_t i_max, v_min = 0xFFFF;
    static int16_t  t_max = -100;
    static uint8_t  idx;
    const uint32_t  hold_ms  = (uint32_t)LONG_TEST_S * 1000U;
    const uint32_t  slice_ms = hold_ms / LONG_TEST_SAMPLES;
    uint16_t thr;

    if (t_ms < LONG_TEST_RAMP_MS)
    {
        run_now.phase = 4;
        return (uint16_t)(LONG_TEST_START_THR
               + ((LONG_TEST_THROTTLE - LONG_TEST_START_THR) * t_ms) / LONG_TEST_RAMP_MS);
    }
    t_ms -= LONG_TEST_RAMP_MS;
    if (t_ms >= hold_ms || idx >= LONG_TEST_SAMPLES)
    {
        if (run_now.phase == 4 && motor_state() != MOTOR_RUN && motor_state() != MOTOR_RAMP) { run_now.phase = 5; run_log_counters(); run_log_save(&run_now); }
        return 0;                          /* done: stays stopped */
    }
    thr = LONG_TEST_THROTTLE;
    run_now.phase = 4;

    /* statistics once per ms (also after a fault: the cooling is logged) */
    {
        int32_t  i = motor_current_ma();
        uint16_t v = motor_vbus_mv();
        uint16_t d = motor_duty();
        if (i < 0) i = 0;
        sum_i    += (uint32_t)i;
        sum_ibat += (uint32_t)i * d / 1000U;
        sum_v    += v;
        sum_rpm  += motor_erpm() / TEST_MOTOR_POLE_PAIRS;
        sum_duty += d;
        n++;
        if ((uint32_t)i > i_max) i_max = (uint16_t)i;
        if (v < v_min) v_min = v;
        if (temp_c > t_max) t_max = temp_c;
    }
    if (t_ms >= (uint32_t)(idx + 1U) * slice_ms)
    {
        volatile lt_sample_t *e = &run_now.lt[idx];
        e->t_s         = (uint16_t)(t_ms / 1000U);
        e->temp_max_c  = t_max;
        e->i_avg_ma    = (uint16_t)(sum_i / n);
        e->i_max_ma    = i_max;
        e->ibat_ma     = (uint16_t)(sum_ibat / n);
        e->vbus_mv     = (uint16_t)(sum_v / n);
        e->vbus_min_mv = v_min;
        e->rpm         = (uint16_t)(sum_rpm / n);
        e->duty_pm     = (uint16_t)(sum_duty / n);
        e->state       = (uint8_t)motor_state();
        e->fault       = (uint8_t)motor_fault();
        idx++;
        run_now.lt_n = idx;
        n = 0; sum_i = 0; sum_ibat = 0; sum_v = 0; sum_rpm = 0; sum_duty = 0;
        i_max = 0; v_min = 0xFFFF; t_max = -100;
    }
    return thr;
}
#endif

/* ------------------------------------------------------------------ main */
volatile uint8_t reset_cause;

int main(void)
{
    uint8_t  armed = 0;
    uint8_t  cells = 0;
    uint32_t arm_start_ms = 0;
    (void)arm_start_ms;
    uint32_t last_ms = 0, last_tel_ms = 0, last_ntc_ms = 0;
    int16_t  temp_c = 25;
    uint16_t vlimit = 1000, tlimit = 1000;

    /* why the chip restarted (debugger): RCC RSTSCKR bits 31..24,
     * 0x04 reset pin / debugger, 0x08 power-on or brown-out, 0x10 software,
     * 0x20 independent watchdog, 0x40 window watchdog */
    reset_cause = (uint8_t)(RCC->RSTSCKR >> 24);
    RCC_ClearFlag();
    run_log_boot(reset_cause);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

    led_init();
    led_set(1);
    tel_init();
#if DRIVER_PIN_TEST == 6
    ls_tim1_test();               /* never returns, low sides only */
#elif DRIVER_PIN_TEST == 5
    hs_check();                   /* never returns, bridge off after 32 s */
#elif DRIVER_PIN_TEST == 4
    current_sweep();              /* never returns, bridge off after ~1.5 s */
#elif DRIVER_PIN_TEST == 3
    pga_path_test();              /* never returns, bridge never enabled */
#elif DRIVER_PIN_TEST == 2
    driver_tim1_test();           /* never returns */
#elif DRIVER_PIN_TEST == 1
    driver_pin_test();            /* never returns */
#endif
    if (motor_init())             /* starts the control loop, calibrates the shunt offset */
    {
        /* Control loop never ran: blink fast forever, never arm. */
        tel_puts("\r\nFATAL: control loop not running (ADC/TIM1)\r\n");
        while (1)
        {
            led_set(1); Delay_Ms(50);
            led_set(0); Delay_Ms(50);
        }
    }
    motor_set_direction(MOTOR_REVERSE);

    PLAY(mel_power);

    /* Auto-detection of the connected motor (see motor_tune.h). */
#if AUTOTUNE_MODE == 2
    PLAY(mel_tune);
    run_now.phase = 2;
        (void)tune_run(tune_led);
    if (tune_res.status == TUNE_OK) PLAY(mel_tune_ok); else PLAY(mel_fail);
#elif AUTOTUNE_MODE == 1
    if (!tune_load())
    {
        PLAY(mel_tune);
        run_now.phase = 2;
        (void)tune_run(tune_led);
        if (tune_res.status == TUNE_OK) PLAY(mel_tune_ok); else PLAY(mel_fail);
    }
#else
    (void)tune_load();
#endif
    if (tune_res.status != TUNE_OK)
    {
        /* tune failed or absent: defaults stay in use; show the code for 3 s
         * (status number = short blinks) */
        uint8_t k, b;
        for (k = 0; k < 2U && tune_res.status > TUNE_OK; k++)
        {
            for (b = 0; b < (uint8_t)tune_res.status; b++)
            {
                led_set(1); Delay_Ms(150);
                led_set(0); Delay_Ms(250);
            }
            Delay_Ms(1000);
        }
    }
    run_now.tune_status     = (uint8_t)tune_res.status;
    run_now.tune_stop_fault = tune_res.stop_fault;
    run_now.phase           = 3;
    rc_init();
    iwdg_init();
    led_set(0);

    tel_puts("\r\nCogless ESC, CH32M007G8R6, sysclk=");
    tel_put_uint(SystemCoreClock);
    tel_puts("\r\n");

#if TEST_AUTORUN
    /* the test schedule starts here, after the melodies and the auto-tune,
     * not at power-up */
    const uint32_t run_t0 = motor_millis();
    /* the test only starts after a real power-up: a debugger (re)connection
     * resets the chip without power-cycling it, the motor then stays still
     * and run_prev keeps the results of the last run */
    const uint8_t autorun_ok = (uint8_t)((reset_cause & 0x08U) != 0U);
#endif
    while (1)
    {
        uint32_t      now = motor_millis();
        uint16_t      thr;
        uint8_t       rc_ok;
        motor_state_t st;
        motor_fault_t f;

        if (now == last_ms)
        {
            continue;             /* 1 ms scheduler */
        }
        last_ms = now;
        IWDG_ReloadCounter();

#if TEST_AUTORUN
        /* Self-running test: throttle schedule instead of the RC input. */
        rc_ok = 1;
        if (!autorun_ok)
        {
            rc_ok = 0;                 /* hold: one long blink every 2 s */
            thr   = 0;
        }
        else if (now - run_t0 < TEST_AUTORUN_DELAY_MS)
        {
            thr = 0;
        }
        else
        {
#if LONG_TEST
            thr = long_test_step(now - run_t0 - TEST_AUTORUN_DELAY_MS, temp_c);
#else
            /* Staircase: first level lasts longer to cover the start-up, then
             * one level every TEST_LEVEL_MS, then TEST_AUTORUN_OFF_MS stopped.
             * The state at the end of each level is stored in run_now.level[]. */
            static const uint16_t levels[TEST_LEVELS_N] = TEST_LEVELS;
            const uint32_t on_ms = TEST_LEVEL0_EXTRA_MS + TEST_LEVELS_N * TEST_LEVEL_MS;
            uint32_t t = (now - run_t0 - TEST_AUTORUN_DELAY_MS) % (on_ms + TEST_AUTORUN_OFF_MS);
            int8_t   li;

            if (TEST_AUTORUN_ONCE && (now - run_t0 - TEST_AUTORUN_DELAY_MS) >= on_ms)
            {
                t = on_ms;                     /* played once: stay stopped */
            }
            if (t >= on_ms)
            {
                li  = -1;
                thr = 0;
                if (run_now.phase == 4 && motor_state() != MOTOR_RUN && motor_state() != MOTOR_RAMP) { run_now.phase = 5; run_log_counters(); run_log_save(&run_now); }
            }
            else
            {
                li = (t < TEST_LEVEL0_EXTRA_MS) ? 0
                     : (int8_t)((t - TEST_LEVEL0_EXTRA_MS) / TEST_LEVEL_MS);
                thr = levels[li];
                run_now.phase = 4;
            }
            if (li != level_prev)
            {
                if (level_prev >= 0)
                {
                    volatile level_log_t *e = &run_now.level[level_prev];
                    e->thr     = levels[level_prev];
                    e->duty_pm = motor_duty();
                    e->erpm    = motor_erpm();
                    e->rpm     = motor_erpm() / TEST_MOTOR_POLE_PAIRS;
                    e->i_ma    = motor_current_ma();
                    e->vbus_mv = motor_vbus_mv();
                    e->state   = (uint8_t)motor_state();
                    e->fault   = (uint8_t)motor_fault();
                    e->i_avg_ma    = lv_n ? (int32_t)(lv_sum_i / (int64_t)lv_n) : 0;
                    e->ibat_ma     = lv_n ? (int32_t)(lv_sum_ibat / (int64_t)lv_n) : 0;
                    e->i_peak_ma   = lv_peak;
                    e->vbus_min_mv = lv_vmin;
                    e->peak_hits   = (uint16_t)(motor_peak_hits() - lv_hits0);
                    e->temp_max_c  = lv_tmax;
                    e->fold_max_pm = motor_fold_max_pm();
                    {
                        uint16_t zr, ze, zb;
                        motor_zc_counts(&zr, &ze, &zb);
                        e->zc_real  = (uint16_t)(zr - lv_zr0);
                        e->zc_early = (uint16_t)(ze - lv_ze0);
                        e->zc_blind = (uint16_t)(zb - lv_zb0);
                    }
                }
                lv_sum_i = 0; lv_sum_ibat = 0; lv_n = 0; lv_peak = 0;
                lv_vmin  = 0xFFFF;
                lv_tmax  = -100;
                lv_hits0 = motor_peak_hits();
                motor_zc_counts(&lv_zr0, &lv_ze0, &lv_zb0);
                (void)motor_fold_max_pm();
                if (li == 0)
                {
                    uint8_t k;
                    for (k = 0; k < TEST_LEVELS_N; k++) run_now.level[k].thr = 0;
                }
                level_prev = li;
            }
            if (li >= 0)
            {
                /* per-level statistics, once per ms */
                int32_t  i = motor_current_ma();
                uint16_t v = motor_vbus_mv();
                lv_sum_i    += i;
                lv_sum_ibat += (int64_t)i * motor_duty() / 1000;
                lv_n++;
                if (i > lv_peak) lv_peak = i;
                if (v < lv_vmin) lv_vmin = v;
                if (temp_c > lv_tmax) lv_tmax = temp_c;
            }
#endif
        }
#else
        thr   = rc_throttle();
        rc_ok = rc_valid();
#endif
        st    = motor_state();
        f     = motor_fault();

        /* ---- run log (survives a reset) ---- */
        run_now.ms_last = now;
        if (st != MOTOR_STOPPED && st != MOTOR_FAULT)
        {
            uint16_t v = motor_vbus_mv();
            if (v < run_now.vbus_min_mv) run_now.vbus_min_mv = v;
        }
        /* every new fault event (its time stamp changes) is logged and saved */
        if (f != MOTOR_FAULT_NONE
            && (run_now.fault_count == 0U || motor_fault_time_ms() != run_now.fault_ms))
        {
            motor_diag_t d;
            motor_get_diag(&d);
            if (run_now.fault_count == 0U) run_now.fault_first = (uint8_t)f;
            if (run_now.fault_count < 255U) run_now.fault_count++;
            run_now.fault       = (uint8_t)f;
            run_now.fault_state = d.fault_state;
            run_now.fault_duty  = d.fault_duty;
            run_now.fault_i_ma  = d.fault_i_ma;
            run_now.fault_ms    = motor_fault_time_ms();
            run_log_counters();
            if (st == MOTOR_FAULT)
            {
                run_log_save(&run_now);    /* outputs already off */
            }
        }

        /* ---- slow measurements and derating ---- */
        if (now - last_ntc_ms >= NTC_PERIOD_MS)
        {
            uint16_t vbus = motor_vbus_mv();

            last_ntc_ms = now;
            if (run_now.fault_count == 0U) run_log_counters();
#if NTC_PRESENT
            temp_c = ntc_to_celsius(motor_read_ntc_raw());
            board_temp_c = temp_c;
            if (temp_c > run_now.temp_max_c) run_now.temp_max_c = temp_c;
#else
            temp_c = 25;                      /* no sensor fitted */
#endif

            if (temp_c >= TEMP_CUTOFF_C)
            {
                motor_raise_fault(MOTOR_FAULT_OVERTEMP);
                tlimit = 0;
            }
            else if (temp_c > TEMP_DERATE_START_C)
            {
                tlimit = (uint16_t)(1000 - ((temp_c - TEMP_DERATE_START_C) * 800)
                                            / (TEMP_CUTOFF_C - TEMP_DERATE_START_C));
            }
            else
            {
                tlimit = 1000;
            }

            if (armed && cells)
            {
                uint16_t vcell = vbus / cells;
                if (vcell < CELL_MV_CUTOFF && st != MOTOR_STOPPED)
                {
                    motor_raise_fault(MOTOR_FAULT_UNDERVOLTAGE);
                    vlimit = 0;
                }
                else if (vcell < CELL_MV_WARN)
                {
                    vlimit = (uint16_t)(200 + ((uint32_t)(vcell - CELL_MV_CUTOFF) * 800)
                                              / (CELL_MV_WARN - CELL_MV_CUTOFF));
                }
                else
                {
                    vlimit = 1000;
                }
            }
            motor_set_duty_limit((tlimit < vlimit) ? tlimit : vlimit);
        }

        /* ---- arming and throttle ---- */
        if (!armed)
        {
            if (rc_ok && thr == 0 && motor_vbus_mv() >= VBUS_MIN_STARTUP_MV)
            {
#if TEST_AUTORUN
                if (now - run_t0 >= TEST_AUTORUN_DELAY_MS / 2U)
#else
                if (now - arm_start_ms >= RC_ARM_TIME_MS)
#endif
                {
                    uint16_t v = motor_vbus_mv();
                    cells = (v > 12900) ? 4 : (v > 8600) ? 3 : 2;
                    armed = 1;
                    PLAY(mel_armed);
                    tel_puts("armed\r\n");
                }
            }
            else
            {
                arm_start_ms = now;
            }
        }
        else if (!rc_ok)
        {
            motor_stop();                     /* failsafe: signal lost */
            armed = 0;
            arm_start_ms = now;
            tel_puts("rc lost, disarmed\r\n");
        }
        else
        {
            motor_set_throttle(thr);

            if (st == MOTOR_FAULT)
            {
                uint8_t  soft   = (f == MOTOR_FAULT_DESYNC || f == MOTOR_FAULT_STALL);
                uint32_t held   = now - motor_fault_time_ms();

                if (thr == 0 && soft && held >= FAULT_RETRY_DELAY_MS)
                {
                    motor_clear_fault();
                }
                else if (thr == 0 && !soft && held >= HARD_FAULT_CLEAR_MS)
                {
                    motor_clear_fault();
#if !TEST_AUTORUN
                    armed = 0;                /* hard fault: re-arm required */
                    arm_start_ms = now;
#endif
                }
            }
            else if (thr > 0 && st == MOTOR_STOPPED)
            {
                motor_start();
            }
            else if (thr == 0 && st != MOTOR_STOPPED)
            {
                motor_stop();
            }
        }

        /* ---- LED ---- */
        {
            uint16_t p    = led_pattern(armed, rc_ok, st, f);
            uint8_t  slot = (uint8_t)((now / LED_SLOT_MS) & 15U);
            led_set((p >> (15U - slot)) & 1U);
        }

        /* ---- telemetry ---- */
        if (tel_active() && now - last_tel_ms >= TELEMETRY_PERIOD_MS)
        {
            last_tel_ms = now;
            telemetry_line(armed, cells, temp_c);
        }
    }
}
