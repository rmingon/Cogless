/* SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Cogless-Commercial
 * Copyright (c) 2026 Ronan Mingon */
/*
 * motor_tune.c - motor auto-detection and start-up adaptation, see
 * motor_tune.h.
 */
#include <string.h>
#include "motor_tune.h"
#include "motor.h"
#include "motor_config.h"
#include "board.h"
#include "debug.h"

/* last 256 B of the 62 KB flash, excluded from the program in Ld/Link.ld */
#define TUNE_REC_ADDR      (0x08000000UL + 0xF700UL)
#define TUNE_REC_SIZE      256U
#define TUNE_MAGIC         0x31474F43UL          /* "COG1" */
#define TUNE_VERSION       7U          /* 7: record tied to the PWM frequency */
/* periods and duties are stored in ticks: a record made at another PWM
 * frequency is invalid */
#define TUNE_REC_VERSION   ((uint16_t)(TUNE_VERSION * 100U + PWM_FREQ_HZ / 1000U))

#define TUNE_SWEEP_FIRST   16U                   /* PWM ticks */
#define TUNE_SWEEP_LAST    150U
#define TUNE_SWEEP_STEP    1U
#define TUNE_I_VALID_MA    1500                  /* first point used for R */
#define TUNE_K_POINTS      6U                    /* BEMF samples averaged for k_ref */
#define TUNE_BAD_MAX       4U                    /* consecutive out-of-sync steps */

typedef struct
{
    uint32_t          magic;
    uint16_t          version;
    uint16_t          size;
    motor_start_cfg_t cfg;
    uint16_t          r_mohm;
    uint16_t          floor_lsb;
    uint32_t          bemf_k;
    uint32_t          check;
} tune_rec_t;

volatile tune_result_t tune_res;

static uint32_t tune_buf[TUNE_REC_SIZE / 4U];

/* ------------------------------------------------------------- flash */
static uint32_t rec_check(const tune_rec_t *r)
{
    const uint8_t *b = (const uint8_t *)r;
    uint32_t c = 0x5A5A1234UL;
    uint32_t i;
    for (i = 0; i < (uint32_t)((const uint8_t *)&r->check - b); i++)
    {
        c = ((c << 5) | (c >> 27)) ^ b[i];
    }
    return c;
}

static uint8_t rec_valid(const tune_rec_t *r)
{
    return (uint8_t)(r->magic == TUNE_MAGIC && r->version == TUNE_REC_VERSION
                     && r->size == sizeof(tune_rec_t) && r->check == rec_check(r));
}

static uint8_t rec_write(const tune_rec_t *src)
{
    FLASH_Status st;

    memset(tune_buf, 0xFF, sizeof(tune_buf));
    memcpy(tune_buf, src, sizeof(tune_rec_t));
    /* the bridge is off here; the flash stalls the CPU while it works */
    __disable_irq();
    st = FLASH_ROM_ERASE(TUNE_REC_ADDR, TUNE_REC_SIZE);
    if (st == FLASH_COMPLETE)
    {
        st = FLASH_ROM_WRITE(TUNE_REC_ADDR, tune_buf, TUNE_REC_SIZE);
    }
    __enable_irq();
    return (uint8_t)(st == FLASH_COMPLETE && rec_valid((const tune_rec_t *)TUNE_REC_ADDR));
}

void tune_forget(void)
{
    __disable_irq();
    (void)FLASH_ROM_ERASE(TUNE_REC_ADDR, TUNE_REC_SIZE);
    __enable_irq();
}

uint8_t tune_load(void)
{
    const tune_rec_t *r = (const tune_rec_t *)TUNE_REC_ADDR;
    motor_start_cfg_t c;

    if (!rec_valid(r))
    {
        return 0;
    }
    c = r->cfg;
    c.ol_hold = 0;
    motor_set_start_cfg(&c);
    tune_res.status         = TUNE_OK;
    tune_res.duty_start     = c.duty_start;
    tune_res.handoff_period = c.handoff_period;
    tune_res.r_mohm         = r->r_mohm;
    tune_res.floor_lsb      = r->floor_lsb;
    tune_res.bemf_k         = r->bemf_k;
    tune_res.slew_chosen    = c.slew_up_ticks;
    return 1;
}

/* ------------------------------------------------------ measurements */
static int32_t raw_zero(void)
{
    uint32_t z = 0;
    uint8_t  i;
    for (i = 0; i < 64U; i++)
    {
        Delay_Ms(1);
        z += motor_debug_iraw();
    }
    return (int32_t)(z / 64U);
}

static int32_t avg_ma(int32_t zero, uint16_t n)
{
    int32_t  sum = 0;
    uint16_t i;
    for (i = 0; i < n; i++)
    {
        Delay_Ms(1);
        sum += (int32_t)motor_debug_iraw() - zero;
    }
    return (sum / (int32_t)n) * BOARD_I_MA_PER_LSB_NUM / BOARD_I_MA_PER_LSB_DEN;
}

/* hold one six-step entry at duty d, return the largest OFF-time reading */
static uint16_t hold_floor(uint8_t entry, uint16_t d, uint16_t ms)
{
    uint16_t i, mx = 0;
    motor_debug_outputs((uint8_t)(2U + entry), d);
    for (i = 0; i < ms; i++)
    {
        uint16_t on, off, vb;
        Delay_Ms(1);
        if (i < ms / 3U)
        {
            continue;                       /* rotor settles on the step */
        }
        motor_debug_bemf(&on, &off, &vb);
        if (off > mx) mx = off;
    }
    return mx;
}

static void bridge_off(void)
{
    motor_debug_outputs(0, 0);
    motor_stop();
}

/* --------------------------------------------- open-loop sync attempt */
static uint8_t try_open_loop(motor_start_cfg_t *c, volatile tune_attempt_t *a,
                             int16_t amp_valid, void (*led)(uint8_t on))
{
    uint16_t n_read = 0, at_final = 0, bad = 0, k_n = 0;
    uint32_t k_sum = 0, k_ref = 0;
    uint32_t ramp_ms, t0, t_end;
    uint8_t  done = 0, ok = 0;

    motor_set_start_cfg(c);
    motor_get_start_cfg(c);                 /* clamped values */
    motor_set_throttle(0);
    motor_clear_fault();
    motor_start();

    /* alignment + ramp + hold at the final rate, with margin */
    ramp_ms = ((uint32_t)(1UL << c->accel_shift) * (c->ol_start_period - c->handoff_period))
              / (CTRL_TICK_HZ / 1000U);
    t0    = motor_millis();
    t_end = t0 + 1500U + ramp_ms + 1000U;

    a->accel_shift = c->accel_shift;
    a->i_start_ma  = c->i_start_ma;
    a->steps       = 0;
    a->last_period = 0;
    a->k_ref       = 0;
    a->result      = 0;

    while (!done && motor_millis() < t_end)
    {
        uint16_t cnt;

        Delay_Ms(1);
        led((uint8_t)((motor_millis() / 100U) & 1U));
        if (motor_state() == MOTOR_FAULT)
        {
            break;
        }
        cnt = motor_trace_count();
        while (n_read < cnt && !done)
        {
            motor_trace_t e;
            int16_t       amp;
            uint32_t      kv;

            if (!motor_trace_get(n_read, &e))
            {
                n_read = (uint16_t)(cnt - MOTOR_TRACE_LEN + 1U);   /* fell behind */
                continue;
            }
            n_read++;
            if (e.tag > 5U)
            {
                continue;
            }
            a->steps++;
            amp = e.bmax;
            if (amp >= amp_valid && (k_n >= TUNE_K_POINTS || e.period <= 4U * c->handoff_period))
            {
                /* reference taken only once the BEMF dominates the parasitics */
                kv = (uint32_t)amp * e.period;
                if (k_n < TUNE_K_POINTS)
                {
                    k_sum += kv;
                    if (++k_n == TUNE_K_POINTS)
                    {
                        k_ref    = k_sum / TUNE_K_POINTS;
                        a->k_ref = k_ref;
                    }
                }
                else if (kv * 2U < k_ref || kv > k_ref * 2U)
                {
                    bad++;                  /* BEMF no longer follows the speed */
                }
                else
                {
                    bad = 0;
                    a->last_period = e.period;
                }
            }
            else if (k_n >= TUNE_K_POINTS)
            {
                bad++;                      /* BEMF vanished after being valid */
            }
            if (bad >= TUNE_BAD_MAX)
            {
                done = 1;                   /* lost synchronism */
            }
            else if (k_n >= TUNE_K_POINTS && bad == 0 && e.period <= c->handoff_period)
            {
                if (++at_final >= TUNE_SYNC_STEPS)
                {
                    done = 1;
                    ok   = 1;
                }
            }
        }
    }
    bridge_off();
    motor_clear_fault();
    a->result = ok;
    Delay_Ms(1500);                         /* let the rotor stop */
    return ok;
}

/* ------------------------------------------- closed-loop acceleration */
#define ACC_SAMPLE_MS 20U
#define ACC_SAMPLES   (TUNE_ACCEL_MS / ACC_SAMPLE_MS)
static uint32_t acc_erpm[ACC_SAMPLES];

static void try_accel(motor_start_cfg_t *c, volatile tune_accel_t *a, void (*led)(uint8_t on))
{
    uint32_t t_end, t_step, now;
    uint16_t n = 0, k, n_tail;
    uint32_t tail = 0, fin;

    a->slew       = c->slew_up_ticks;
    a->result     = 9;
    a->t90_ms     = 0;
    a->erpm_final = 0;
    a->i_peak_ma  = 0;
    a->duty_final = 0;

    motor_set_start_cfg(c);
    motor_set_throttle(TUNE_ACCEL_THROTTLE / 10U);   /* small throttle while starting */
    motor_clear_fault();
    motor_start();

    /* wait for the hand-off to closed loop */
    t_end = motor_millis() + 2000U
            + (((uint32_t)1U << c->accel_shift) * (c->ol_start_period - c->handoff_period))
              / (CTRL_TICK_HZ / 1000U);
    while (motor_state() != MOTOR_RUN)
    {
        Delay_Ms(1);
        led((uint8_t)((motor_millis() / 250U) & 1U));
        if (motor_state() == MOTOR_FAULT || motor_millis() > t_end)
        {
            a->result = (motor_state() == MOTOR_FAULT) ? (uint8_t)motor_fault() : 9U;
            bridge_off();
            motor_clear_fault();
            Delay_Ms(1500);
            return;
        }
    }
    Delay_Ms(200);                          /* settle at the hand-off speed */

    /* throttle step, observe */
    motor_set_throttle(TUNE_ACCEL_THROTTLE);
    t_step = motor_millis();
    t_end  = t_step + TUNE_ACCEL_MS;
    while ((now = motor_millis()) < t_end)
    {
        int32_t i;
        Delay_Ms(1);
        led((uint8_t)((now / 50U) & 1U));
        if (motor_state() != MOTOR_RUN)
        {
            a->result = (uint8_t)motor_fault();
            break;
        }
        i = motor_current_ma();
        if (i > a->i_peak_ma) a->i_peak_ma = i;
        if ((now - t_step) >= (uint32_t)n * ACC_SAMPLE_MS && n < ACC_SAMPLES)
        {
            acc_erpm[n++] = motor_erpm();
        }
    }
    if (motor_state() == MOTOR_RUN && n == ACC_SAMPLES)
    {
        a->result     = 0;
        a->duty_final = motor_duty();
        n_tail = 300U / ACC_SAMPLE_MS;
        for (k = (uint16_t)(n - n_tail); k < n; k++) tail += acc_erpm[k];
        fin = tail / n_tail;
        a->erpm_final = fin;
        /* must clearly exceed the hand-off speed, else it only crawled */
        if (fin * 2U < 3U * ((10U * CTRL_TICK_HZ) / c->handoff_period))
        {
            a->result = 8;
        }
        for (k = 0; k < n; k++)
        {
            if (acc_erpm[k] * 10U >= fin * 9U)
            {
                a->t90_ms = (uint16_t)(k * ACC_SAMPLE_MS);
                break;
            }
        }
    }
    motor_set_throttle(0);
    bridge_off();
    motor_clear_fault();
    Delay_Ms(2500);                         /* let the rotor stop */
}

/* -------------------------------------------------------------- run */
tune_status_t tune_run(void (*led)(uint8_t on))
{
    static const uint8_t  shifts[TUNE_MAX_ATTEMPTS]  = { 5, 6, 7, 5, 6, 7 };
    static const uint8_t  i_x10[TUNE_MAX_ATTEMPTS]   = { 10, 10, 10, 15, 15, 15 };
    motor_start_cfg_t c;
    tune_rec_t        rec;
    int32_t  zero, i, i_lo = 0;
    uint16_t d, d_lo = 0, duty_start = 0, f0, f3, floor_lsb;
    uint32_t vbus;
    int16_t  amp_valid, amp_handoff;
    uint8_t  k;

    memset((void *)&tune_res, 0, sizeof(tune_res));
    tune_res.status = TUNE_RUNNING;
    led(1);

    /* ---- 1. standstill sweep on entry 0 (U+ V-) ---- */
    bridge_off();
    Delay_Ms(200);
    zero = raw_zero();
    vbus = motor_vbus_mv();
    tune_res.vbus_mv = (uint16_t)vbus;

    for (d = TUNE_SWEEP_FIRST; d <= TUNE_SWEEP_LAST; d = (uint16_t)(d + TUNE_SWEEP_STEP))
    {
        motor_debug_outputs(2, d);
        Delay_Ms(25);
        i = avg_ma(zero, 10);
        if (motor_state() == MOTOR_FAULT || i > (int32_t)TUNE_I_ABORT_MA)
        {
            tune_res.stop_duty  = d;
            tune_res.stop_ma    = i;
            tune_res.stop_fault = (uint8_t)motor_fault();
            bridge_off();
            motor_clear_fault();
            tune_res.status = TUNE_ERR_OVERCURRENT;
            return tune_res.status;
        }
        if (!d_lo && i >= TUNE_I_VALID_MA)
        {
            d_lo = d;
            i_lo = i;
        }
        if (i >= (int32_t)TUNE_I_TARGET_MA)
        {
            duty_start = d;
            tune_res.i_meas_ma = i;
            break;
        }
    }
    if (!duty_start)
    {
        bridge_off();
        tune_res.status = TUNE_ERR_NO_MOTOR;
        return tune_res.status;
    }
    tune_res.duty_start = duty_start;

    /* resistance from two points: V = Vbus * (2 d - dead time) / (2 ARR) */
    if (d_lo && duty_start > d_lo && tune_res.i_meas_ma > i_lo)
    {
        int32_t v1 = (int32_t)vbus * (int32_t)(2U * d_lo - PWM_DEAD_TIME_TICKS) / (int32_t)(2U * PWM_PERIOD_TICKS);
        int32_t v2 = (int32_t)vbus * (int32_t)(2U * duty_start - PWM_DEAD_TIME_TICKS) / (int32_t)(2U * PWM_PERIOD_TICKS);
        tune_res.r_mohm = (uint16_t)(((v2 - v1) * 1000) / (tune_res.i_meas_ma - i_lo));
    }

    /* ---- 2. parasitic OFF-time floor with the start current flowing ---- */
    f0 = hold_floor(0, duty_start, 150);
    f3 = hold_floor(3, duty_start, 150);
    bridge_off();
    floor_lsb = (f0 > f3) ? f0 : f3;
    if (floor_lsb < 8U) floor_lsb = 8U;
    tune_res.floor_lsb = floor_lsb;
    Delay_Ms(500);

    amp_valid   = (int16_t)((3U * floor_lsb > 20U) ? 3U * floor_lsb : 20U);
    amp_handoff = (int16_t)((TUNE_HANDOFF_SNR * floor_lsb > TUNE_HANDOFF_MIN_AMP)
                            ? TUNE_HANDOFF_SNR * floor_lsb : TUNE_HANDOFF_MIN_AMP);

    /* ---- 3. open-loop acceleration attempts ---- */
    for (k = 0; k < TUNE_MAX_ATTEMPTS; k++)
    {
        c.i_start_ma      = (uint16_t)((TUNE_I_TARGET_MA * i_x10[k]) / 10U);
        c.duty_start      = duty_start;
        c.duty_max_ol     = (uint16_t)(duty_start * 4U);
        c.ol_start_period = (uint16_t)US_TO_TICKS(TUNE_OL_START_US);
        c.handoff_period  = (uint16_t)US_TO_TICKS(TUNE_OL_FINAL_US);
        c.accel_shift     = shifts[k];
        c.ol_hold         = 1;
        tune_res.n_attempts = (uint8_t)(k + 1U);
        if (try_open_loop(&c, &tune_res.att[k], amp_valid, led))
        {
            break;
        }
    }
    if (k == TUNE_MAX_ATTEMPTS)
    {
        tune_res.status = TUNE_ERR_NO_SYNC;
        led(0);
        return tune_res.status;
    }

    /* ---- 4. hand-off where the BEMF peak reaches amp_handoff ---- */
    {
        uint32_t kref = tune_res.att[k].k_ref;
        uint32_t ph   = kref / (uint32_t)amp_handoff;
        if (ph < c.handoff_period)        ph = c.handoff_period;     /* tested final rate */
        if (ph > c.ol_start_period / 4U)  ph = c.ol_start_period / 4U;
        c.handoff_period = (uint16_t)ph;
        c.ol_hold        = 0;
        tune_res.bemf_k         = kref;
        tune_res.handoff_period = (uint16_t)ph;
    }
    motor_set_start_cfg(&c);
    motor_get_start_cfg(&c);

    /* ---- 5. closed-loop acceleration: fastest duty slew that passes ---- */
    {
        static const uint8_t slews[TUNE_ACCEL_N] = TUNE_ACCEL_SLEWS;
        uint8_t j;
        tune_res.slew_chosen = 0;
        for (j = 0; j < TUNE_ACCEL_N && j < TUNE_ACCEL_MAX; j++)
        {
            c.slew_up_ticks = slews[j];
            tune_res.n_accel = (uint8_t)(j + 1U);
            try_accel(&c, &tune_res.acc[j], led);
            if (tune_res.acc[j].result == 0U)
            {
                /* keep a 2x margin on the fastest slew that worked */
                uint16_t sl = (uint16_t)slews[j] * 2U;
                tune_res.slew_chosen = (uint8_t)(sl > 128U ? 128U : sl);
                break;
            }
            if (tune_res.acc[j].result == 9U)
            {
                break;                      /* start itself failed: no point going on */
            }
        }
        c.slew_up_ticks = tune_res.slew_chosen ? tune_res.slew_chosen : (uint8_t)DUTY_SLEW_UP_TICKS;
        motor_set_start_cfg(&c);
        motor_get_start_cfg(&c);
    }

    /* ---- 6. store ---- */
    memset(&rec, 0, sizeof(rec));
    rec.magic     = TUNE_MAGIC;
    rec.version   = TUNE_REC_VERSION;
    rec.size      = sizeof(tune_rec_t);
    rec.cfg       = c;
    rec.r_mohm    = tune_res.r_mohm;
    rec.floor_lsb = tune_res.floor_lsb;
    rec.bemf_k    = tune_res.bemf_k;
    rec.check     = rec_check(&rec);
    tune_res.status = rec_write(&rec) ? TUNE_OK : TUNE_ERR_FLASH;

    /* three long blinks: done */
    for (k = 0; k < 3U; k++)
    {
        led(1); Delay_Ms(400);
        led(0); Delay_Ms(300);
    }
    return tune_res.status;
}
