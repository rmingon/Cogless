/* SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Cogless-Commercial
 * Copyright (c) 2026 Ronan Mingon */
/*
 * motor.c - sensorless six-step BLDC controller, see motor.h.
 */
#include "motor.h"

/* Interrupt-path code runs from RAM: the flash needs 2 wait states at
 * 48 MHz, which the 48 kHz loop cannot afford. The startup code copies
 * .data (and so .data.ramfunc) from flash to RAM. */
#define RAMFUNC __attribute__((section(".data.ramfunc")))
#include "motor_config.h"
#include "debug.h"

/* ------------------------------------------------------------------ TIM1
 * Register images written on every commutation. Writing the three registers
 * directly is much faster than the library helpers and avoids intermediate
 * states. Channels: U = CH1, V = CH2, W = CH3, CH4 = ADC trigger only.
 */
#define OCM_PWM1          0x0060U   /* OCxM = 110b, PWM mode 1                  */
#define OCM_FORCED_LOW    0x0040U   /* OCxM = 100b, OCx inactive -> OCxN active */
#define OCM_FORCED_HIGH   0x0050U   /* OCxM = 101b, only used for a sub-dead-time edge */
#define OCM_MASK          0x0070U
#define OC_PRELOAD        0x0008U

typedef struct
{
    uint16_t chctlr1;       /* CH1 / CH2 mode  */
    uint16_t chctlr2;       /* CH3 / CH4 mode  */
    uint16_t ccer;          /* output enables  */
    uint16_t chctlr1_edge;  /* same, low-side channels forced ACTIVE (edge maker) */
    uint16_t chctlr2_edge;
} out_regs_t;

enum { PH_U = 0, PH_V = 1, PH_W = 2, PH_NONE = 3 };

/* step -> { PWM (high side) phase, low-side phase, floating phase } */
static const uint8_t step_table[6][3] =
{
    { PH_U, PH_V, PH_W },
    { PH_U, PH_W, PH_V },
    { PH_V, PH_W, PH_U },
    { PH_V, PH_U, PH_W },
    { PH_W, PH_U, PH_V },
    { PH_W, PH_V, PH_U },
};

static const uint16_t ccer_phase[3] =
{
    TIM_CC1E | TIM_CC1NE,
    TIM_CC2E | TIM_CC2NE,
    TIM_CC3E | TIM_CC3NE,
};

/* A phase that is off must still have its two driver inputs DRIVEN low.
 * With CCxE = CCxNE = 0 TIM1 releases both pins (not driven, RM OSSR note)
 * and the gate driver sees floating HIN/LIN inputs. Keeping CCxE = 1 with
 * OCxREF forced inactive drives OCx = 0, and OSSR = 1 drives OCxN at its
 * inactive level (0). */
static const uint16_t ccer_off[3] =
{
    TIM_CC1E,
    TIM_CC2E,
    TIM_CC3E,
};

static const uint8_t bemf_adc_ch[3] =
{
    BOARD_BEMF_U_ADC_CH, BOARD_BEMF_V_ADC_CH, BOARD_BEMF_W_ADC_CH,
};

static out_regs_t step_regs[6];
static out_regs_t regs_brake;
static out_regs_t regs_low_only[3];   /* one low side forced on, nothing else */
static out_regs_t regs_off;
static uint32_t   isqr_for_phase[3];

/* ---------------------------------------------------------------- state */
typedef struct
{
    volatile motor_state_t state;
    volatile motor_fault_t fault;
    volatile uint32_t fault_ms;

    /* time */
    volatile uint32_t tick;
    volatile uint32_t ms;
    uint16_t          ms_acc;
    uint32_t          state_ticks;

    /* commutation */
    uint8_t  step;
    uint8_t  reverse;
    uint32_t step_ticks;      /* ticks since last commutation             */
    uint32_t zc_ticks;        /* ticks since last zero crossing           */
    uint32_t zc_period;       /* filtered 60 deg period, ticks            */
    uint32_t zc_period_x16;   /* same with 4 fractional bits (closed loop) */
    uint8_t  zc_missed;       /* consecutive steps without a crossing      */
    uint8_t  speed_capped;    /* 1 = at the speed ceiling                  */
    uint16_t cap_duty;        /* duty ceiling from the speed governor, 0 = none */
    uint16_t peak_hits;       /* cycle-by-cycle limit activations (diagnostic) */
    uint16_t zc_real_cnt;     /* closed loop: crossings seen happening (diagnostic) */
    uint16_t zc_early_cnt;    /* closed loop: "already crossed" crossings */
    uint16_t zc_blind_cnt;    /* closed loop: blind commutations */
    uint8_t  zc_kind;         /* trace flag of the crossing of this step: 0 real, 0x10 early */
    uint16_t cc_isr_max;      /* longest control interrupt, timer ticks from the CC4 match */
    uint16_t cc_overrun;      /* control interrupts still running at the next CC4 (tick lost) */
    uint16_t cc_late;         /* control interrupts entered late: current sample ignored */
    uint8_t  sample_late;     /* this tick's injected samples were not taken at the bottom */
    uint8_t  late_run;        /* consecutive late samples */
    volatile uint8_t ovr_win; /* lost ticks in the current OVR_WINDOW_TICKS window */
    uint8_t  sag_count;       /* consecutive bus samples below VBUS_SAG_FAULT_MV */
    int32_t  fault_i_raw;     /* shunt sample (LSB, 39.4 mA) when the last fault was raised */
    uint8_t  fault_state;     /* motor_state_t when the last fault was raised */
    uint16_t fault_duty;      /* duty (PWM ticks) when the last fault was raised */
    uint16_t i_fold_max;      /* largest fold-back since the last read (diagnostic) */
    uint8_t  start_phase;     /* 1 = direct start, no zero crossing seen yet */
    uint16_t beep_half;       /* ticks per half tone period */
    uint16_t beep_cnt;
    uint32_t beep_left;       /* ticks of tone remaining */
    uint8_t  beep_pol;
    uint16_t beep_pre;        /* bootstrap precharge ticks left before the tone */
    motor_start_cfg_t cfg;    /* start-up parameters (auto-tune or defaults) */
    uint32_t ramp_timeout;    /* ticks, from the configured ramp */
    int32_t  i_start_lsb;
    uint32_t commutate_at;    /* step_ticks value of the next commutation */
    uint8_t  zc_found;
    uint8_t  zc_armed;        /* sample seen on the "wrong" side first    */
    uint8_t  zc_first;        /* next ZC check is the first after blanking */
    volatile uint16_t bemf_off; /* floating phase sampled mid OFF time (raw LSB) */
    volatile uint16_t bemf_on;  /* floating phase sampled in the high-side pulse (raw LSB) */
    volatile uint8_t  bemf_ch_ok; /* regular group points at a BEMF channel (a step is applied) */
    uint16_t ntc_last;
    volatile uint8_t  ntc_req;  /* main asks for an NTC conversion in the OFF-time interrupt */
    uint8_t  good_zc;
    uint32_t ramp_period;

    /* duty */
    uint16_t throttle;        /* 0..1000 from main                        */
    uint16_t duty_limit;      /* 0..1000 from main (derating)             */
    uint16_t duty_target;     /* ticks                                    */
    uint16_t duty;            /* ticks, after slew                        */
    uint16_t i_fold;          /* ticks removed by the current limiter     */
    uint16_t slew_acc;

    /* measurements */
    int32_t  i_offset;        /* PGA output at zero current, LSB          */
    int32_t  i_raw;           /* last sample - offset, LSB                */
    int32_t  i_filt_x16;
    uint8_t  oc_count;
    volatile uint16_t cal_count;
    volatile uint8_t  loop_alive;
    volatile uint8_t  debug_hold;   /* motor_debug_outputs() active: ISR leaves TIM1 alone */
    /* sampling-instant diagnostics (software trigger mode) */
    volatile uint16_t dbg_cnt_start; /* TIM1->CNT right after JSWSTART      */
    volatile uint16_t dbg_cnt_done;  /* TIM1->CNT when the sequence is done */
    volatile uint8_t  dbg_dir_start; /* 1 = counting down at JSWSTART       */
    uint32_t cal_sum;
    uint16_t vbus_raw;
    uint32_t vbus_filt_x16;
} motor_t;

static motor_t m;

/* Start-up trace: one entry per commutation after the alignment, plus one
 * entry when a fault is entered. Read it in the debugger (motor_trace). */
typedef motor_trace_t trace_t;

/* circular: motor_trace_n counts every entry, the last TRACE_LEN are kept;
 * the newest entry is at index motor_trace_i - 1 (debugger) */
#define TRACE_LEN MOTOR_TRACE_LEN
volatile trace_t  motor_trace[TRACE_LEN];
volatile uint16_t motor_trace_n;

static int16_t bemf_min, bemf_max;
static uint8_t zc_at = 0xFF;
static uint16_t motor_trace_i;     /* write index, motor_trace_n % TRACE_LEN */

/* Closed-loop statistics per step index (debugger): how each step ended and
 * what its floating phase looked like (OFF-time raw LSB, after blanking).
 * rising steps are the odd ones (forward direction). */
typedef struct
{
    uint16_t real, early, blind;
    int16_t  vmin;      /* filtered minimum of the floating phase over the step */
    int16_t  vmax;      /* filtered maximum */
} zc_stat_t;
volatile zc_stat_t zc_stat[6];

RAMFUNC static void trace_add(uint8_t tag)
{
    volatile trace_t *t = &motor_trace[motor_trace_i];
    motor_trace_n++;
    if (++motor_trace_i >= TRACE_LEN) motor_trace_i = 0;   /* = motor_trace_n % TRACE_LEN */
    t->period = (uint16_t)((m.state == MOTOR_RUN) ? m.zc_period : m.ramp_period);
    t->duty   = m.duty;
    t->i_lsb  = (int16_t)(m.i_filt_x16 >> I_FILTER_SHIFT);
    t->fold   = (uint8_t)(m.i_fold > 255U ? 255U : m.i_fold);
    t->tag    = tag;
    t->zc     = m.good_zc;
    t->bmin   = bemf_min;
    t->bmax   = bemf_max;
    t->zc_at  = zc_at;
    if (m.state == MOTOR_RUN && (tag & 0xC0U) == 0U && (tag & 0x0FU) < 6U
        && bemf_min != 32767)
    {
        volatile zc_stat_t *s = &zc_stat[tag & 0x0FU];
        if (tag & 0x20U)      s->blind++;
        else if (tag & 0x10U) s->early++;
        else                  s->real++;
        s->vmin = (int16_t)(s->vmin + (bemf_min - s->vmin) / 8);
        s->vmax = (int16_t)(s->vmax + (bemf_max - s->vmax) / 8);
    }
    zc_at     = 0xFF;
    bemf_min  = 32767;
    bemf_max  = -32767;
}

/* constants derived once */
#define I_LIMIT_LSB       ((int32_t)I_LIMIT_MA * BOARD_I_MA_PER_LSB_DEN / BOARD_I_MA_PER_LSB_NUM)
#define I_SW_FAULT_LSB    ((int32_t)I_SW_FAULT_MA * BOARD_I_MA_PER_LSB_DEN / BOARD_I_MA_PER_LSB_NUM)
#define I_PEAK_LSB        ((int32_t)I_PEAK_MA * BOARD_I_MA_PER_LSB_DEN / BOARD_I_MA_PER_LSB_NUM)
#define DESYNC_MIN_TICKS  MS_TO_TICKS(DESYNC_TIMEOUT_MS)
#define VBUS_SAG_FAULT_LSB ((uint32_t)VBUS_SAG_FAULT_MV * BOARD_VBUS_MV_DEN / BOARD_VBUS_MV_NUM)

/* ------------------------------------------------------------ helpers */
/*
 * pwm_ph: phase driven with complementary PWM (PH_NONE for none).
 * low_mask: bit n set = phase n low side forced on.
 * Every other phase is off with both driver inputs driven low.
 */
static void build_regs(uint8_t pwm_ph, uint8_t low_mask, out_regs_t *r)
{
    uint16_t mode[3] = { OCM_FORCED_LOW, OCM_FORCED_LOW, OCM_FORCED_LOW };
    uint16_t edge[3] = { OCM_FORCED_LOW, OCM_FORCED_LOW, OCM_FORCED_LOW };
    uint16_t ccer = 0;
    uint8_t  ph;

    for (ph = 0; ph < 3; ph++)
    {
        if (ph == pwm_ph)
        {
            mode[ph] = edge[ph] = OCM_PWM1;
            ccer |= ccer_phase[ph];        /* complementary PWM          */
        }
        else if (low_mask & (1U << ph))
        {
            edge[ph] = OCM_FORCED_HIGH;    /* see apply_regs()           */
            ccer |= ccer_phase[ph];        /* OCx = 0, OCxN = 1: low on  */
        }
        else
        {
            ccer |= ccer_off[ph];          /* OCx = 0, OCxN = 0, driven  */
        }
    }

    r->chctlr1 = (uint16_t)(mode[PH_U] | OC_PRELOAD | (uint16_t)((mode[PH_V] | OC_PRELOAD) << 8));
    /* CH4: PWM1, no preload, output not enabled on the pin (it is SWIO). */
    r->chctlr2 = (uint16_t)(mode[PH_W] | OC_PRELOAD | (uint16_t)(OCM_PWM1 << 8));
    r->chctlr1_edge = (uint16_t)(edge[PH_U] | OC_PRELOAD | (uint16_t)((edge[PH_V] | OC_PRELOAD) << 8));
    r->chctlr2_edge = (uint16_t)(edge[PH_W] | OC_PRELOAD | (uint16_t)(OCM_PWM1 << 8));
    r->ccer    = ccer;
}

/*
 * On this TIM1, OCxN only follows OCxREF on an OCxREF edge: a channel set
 * straight to "forced inactive" keeps OCxN at its previous level. Low-side
 * channels are pulsed to "forced active" for a few bus cycles and back; the
 * pulse is far shorter than the dead time, so the high side never turns on.
 * Call with interrupts masked or from the TIM1 interrupt.
 */
RAMFUNC static inline void apply_regs(const out_regs_t *r)
{
    /* load everything first so the four mode writes are back to back */
    register uint32_t e1 = r->chctlr1_edge, e2 = r->chctlr2_edge;
    register uint32_t f1 = r->chctlr1,      f2 = r->chctlr2;
    volatile uint16_t *c1 = &TIM1->CHCTLR1;
    volatile uint16_t *c2 = &TIM1->CHCTLR2;

    TIM1->CCER = r->ccer;
    __asm__ volatile ("" : : "r"(e1), "r"(e2), "r"(f1), "r"(f2), "r"(c1), "r"(c2) : "memory");
    *c1 = (uint16_t)e1;
    *c2 = (uint16_t)e2;
    *c1 = (uint16_t)f1;
    *c2 = (uint16_t)f2;
}

static inline void outputs_enable(void)
{
    TIM1->BDTR |= TIM_MOE;
}

static inline void outputs_disable(void)
{
    TIM1->BDTR &= (uint16_t)~TIM_MOE;
}

RAMFUNC static inline void set_duty_regs(uint16_t d)
{
    TIM1->CH1CVR = d;
    TIM1->CH2CVR = d;
    TIM1->CH3CVR = d;
}

RAMFUNC static inline void commutate_to(uint8_t step)
{
    m.step = step;
    apply_regs(&step_regs[step]);
    ADC1->ISQR = isqr_for_phase[step_table[step][2]];
#if BEMF_SAMPLE_OFFTIME
    ADC1->RSQR3  = bemf_adc_ch[step_table[step][2]];   /* OFF-time sample, regular group */
    m.bemf_ch_ok = 1;
#endif
    m.step_ticks = 0;
    m.zc_found   = 0;
    m.zc_armed   = 0;
    m.zc_first   = 1;
}

RAMFUNC static inline void next_step(void)
{
    uint8_t s = m.reverse ? (uint8_t)(m.step ? m.step - 1U : 5U)
                          : (uint8_t)(m.step < 5U ? m.step + 1U : 0U);
    commutate_to(s);
}

RAMFUNC static void enter_fault(motor_fault_t f)
{
    m.fault_i_raw = m.i_raw;
    m.fault_state = (uint8_t)m.state;
    m.fault_duty  = m.duty;
    trace_add((uint8_t)(0x80U | (uint8_t)f));
    m.debug_hold = 0;
    outputs_disable();
    apply_regs(&regs_off);
    m.duty = 0;
    m.duty_target = 0;
    set_duty_regs(0);
    m.state    = MOTOR_FAULT;
    m.fault    = f;
    m.fault_ms = m.ms;
}

RAMFUNC static void go_stopped(void)
{
    outputs_disable();
    apply_regs(&regs_off);
    m.duty = 0;
    m.duty_target = 0;
    m.i_fold = 0;
    set_duty_regs(0);
    m.state = MOTOR_STOPPED;
}

/*
 * Zero-crossing test on one sample of the floating phase.
 * Returns 1 for a crossing seen happening, 2 for "already crossed" (the
 * first sample after blanking is already on the far side), else 0.
 * allow_early: bit 0 = rising steps, bit 1 = falling steps.
 */
#if BEMF_SAMPLE_OFFTIME
/*
 * OFF-time sample: the two driven phases sit at 0 V (low sides on), the
 * floating phase reads 1.5 x its BEMF against ground; the negative half is
 * clamped to ~0 by its low-side body diode. Rising step: 0 -> positive.
 * Falling step: positive -> 0.
 */
RAMFUNC static inline uint8_t zc_detect(uint16_t v, uint8_t allow_early)
{
    uint8_t rising = (uint8_t)((m.step & 1U) ^ m.reverse);

    /* clamped to Vbus through the high-side diode: still demagnetising */
    if ((uint32_t)v + ZC_OFF_RAIL_MARGIN_LSB >= m.vbus_raw)
    {
        return 0;
    }
    /* falling step still demagnetising: 0 V in the ON time too */
    if (!rising && m.duty >= ZC_FALL_GATE_MIN_DUTY
        && (uint32_t)m.bemf_on * ZC_FALL_GATE_DIV < m.vbus_raw)
    {
        return 0;
    }
    m.zc_first = 0;
    /* Reaching the far side without having seen the near side means the
     * crossing happened before (or within the hysteresis band of) the first
     * samples: "already crossed". Testing only the very first sample left a
     * hole: a first sample inside the band never armed, and the step ended
     * blind. */
    if (rising)
    {
        if (v <= ZC_OFF_LOW_LSB)
        {
            m.zc_armed = 1;
        }
        else if (v >= ZC_OFF_HIGH_LSB)
        {
            if (m.zc_armed) return 1;
            if (allow_early & 1U) return 2;
        }
    }
    else
    {
        /* falling: only allowed when demagnetisation (also 0 V) cannot pass
         * for a crossing, see early_mask() */
        if (v >= ZC_OFF_HIGH_LSB)
        {
            m.zc_armed = 1;
        }
        else if (v <= ZC_OFF_LOW_LSB)
        {
            if (m.zc_armed) return 1;
            if (allow_early & 2U) return 2;
        }
    }
    return 0;
}
#else
/* ON-time sample: the floating phase is compared with Vbus/2 (both through
 * identical dividers, raw ADC counts compare directly) */
RAMFUNC static inline uint8_t zc_detect(uint16_t bemf, uint8_t allow_early)
{
    int32_t neutral = (int32_t)(m.vbus_filt_x16 >> (4 + 1));
    uint8_t rising  = (uint8_t)((m.step & 1U) ^ m.reverse);
    int32_t v       = (int32_t)bemf;
    int32_t d       = v - neutral;
    uint8_t first;

    /* clamped to a rail: still demagnetising, ignore it and keep the
     * first-sample status for the next one */
    if (d >= ZC_RAIL_LSB || d <= -ZC_RAIL_LSB)
    {
        return 0;
    }
    first      = m.zc_first;
    m.zc_first = 0;
    /* rotor ahead: the crossing happened while the phase was still driven */
    if (first && allow_early)
    {
        int32_t far = rising ? d : -d;
        if (far > ZC_EARLY_MIN_LSB)
        {
            return 1;
        }
    }
    /* a crossing only counts once the phase was seen on the other side */
    if (rising)
    {
        if (v < neutral - ZC_HYST_LSB)
        {
            m.zc_armed = 1;
        }
        else if (v > neutral + ZC_HYST_LSB)
        {
            return m.zc_armed;
        }
    }
    else
    {
        if (v > neutral + ZC_HYST_LSB)
        {
            m.zc_armed = 1;
        }
        else if (v < neutral - ZC_HYST_LSB)
        {
            return m.zc_armed;
        }
    }
    return 0;
}
#endif

/* "already crossed" mask for zc_detect: rising steps always, falling steps
 * once demagnetisation cannot pass for a crossing */
/* end of a closed-loop step: full trace entry only with TRACE_IN_RUN (it
 * costs ~130 instructions per commutation), otherwise the per-step counts */
RAMFUNC static inline void run_step_done(uint8_t tag)
{
#if TRACE_IN_RUN
    trace_add(tag);
#else
    volatile zc_stat_t *s = &zc_stat[tag & 0x0FU];
    if (tag & 0x20U)      s->blind++;
    else if (tag & 0x10U) s->early++;
    else                  s->real++;
    zc_at = 0xFF;
#endif
}

RAMFUNC static inline uint8_t early_mask(uint32_t period)
{
#if BEMF_SAMPLE_OFFTIME
    return (period >= ZC_EARLY_FALL_MIN_TICKS || m.duty >= ZC_FALL_GATE_MIN_DUTY) ? 3U : 1U;
#else
    return (period >= ZC_EARLY_FALL_MIN_TICKS) ? 3U : 1U;
#endif
}

RAMFUNC static inline uint32_t blank_ticks(uint32_t period)
{
    uint32_t b = period / ZC_BLANK_DIV;
    return (b < ZC_BLANK_MIN_TICKS) ? ZC_BLANK_MIN_TICKS : b;
}

RAMFUNC static inline uint16_t throttle_to_duty(void)
{
    /* x / 1000 as (x * 1049) >> 20: no hardware divider on RV32EC, and the
     * loop budget halves at 48 kHz */
    uint32_t d   = PWM_DUTY_MIN + (((uint32_t)(PWM_DUTY_MAX - PWM_DUTY_MIN) * m.throttle * 1049U) >> 20);
    uint32_t lim = ((uint32_t)PWM_PERIOD_TICKS * m.duty_limit * 1049U) >> 20;

    if (d > lim) d = lim;
    /* current fold-back, taken off the target */
    d = (d > m.i_fold) ? d - m.i_fold : 0U;
    if (d < RUN_DUTY_FLOOR) d = RUN_DUTY_FLOOR;
    return (uint16_t)d;
}

/* Alignment and open-loop ramp: regulate the shunt current to
 * cfg.i_start_ma. The duty is set directly (no slew) so the loop stays fast.
 * Below ~25 PWM ticks the shunt sample falls outside the pulse and reads
 * low; the regulator then simply raises the duty back into range. */
RAMFUNC static inline void start_regulate_current(void)
{
    int32_t i = m.i_filt_x16 >> I_FILTER_SHIFT;

    /* not scaled by DUTY_RATE_SCALE: this current loop must follow the
     * open-loop ramp; slowed down 4x at 48 kHz the start lost sync */
    if ((m.tick & 3U) == 0U)
    {
        if (i > m.i_start_lsb)
        {
            if (m.duty_target > START_DUTY_FLOOR) m.duty_target--;
        }
        else if (i < m.i_start_lsb - (m.i_start_lsb >> 3))
        {
            if (m.duty_target < m.cfg.duty_max_ol) m.duty_target++;
        }
        m.duty = m.duty_target;
    }
}

void motor_set_start_cfg(const motor_start_cfg_t *c)
{
    motor_start_cfg_t k = *c;
    if (k.accel_shift < 3U)  k.accel_shift = 3U;
    if (k.accel_shift > 10U) k.accel_shift = 10U;
    if (k.handoff_period < ZC_PERIOD_MIN_TICKS * 2U) k.handoff_period = ZC_PERIOD_MIN_TICKS * 2U;
    if (k.ol_start_period < k.handoff_period) k.ol_start_period = k.handoff_period;
    if (k.duty_max_ol > PWM_DUTY_MAX / 2U) k.duty_max_ol = PWM_DUTY_MAX / 2U;
    if (k.duty_start > k.duty_max_ol) k.duty_start = k.duty_max_ol;
    if (k.slew_up_ticks == 0U) k.slew_up_ticks = DUTY_SLEW_UP_TICKS;
    __disable_irq();
    m.cfg = k;
    m.i_start_lsb = (int32_t)k.i_start_ma * BOARD_I_MA_PER_LSB_DEN / BOARD_I_MA_PER_LSB_NUM;
    __enable_irq();
}

void motor_get_start_cfg(motor_start_cfg_t *c)
{
    *c = m.cfg;
}

uint16_t motor_trace_count(void)
{
    return motor_trace_n;
}

uint8_t motor_trace_get(uint16_t n, motor_trace_t *out)
{
    uint8_t ok;
    __disable_irq();
    ok = (uint8_t)(n < motor_trace_n && (uint16_t)(motor_trace_n - n) <= TRACE_LEN);
    if (ok)
    {
        *out = *(const trace_t *)&motor_trace[n % TRACE_LEN];
    }
    __enable_irq();
    return ok;
}

/* --------------------------------------------------------- fast loop ISR */
void ADC1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_BRK_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/* One control step on the freshly converted injected group. Called from the
 * ADC JEOC interrupt (hardware trigger) or from the TIM1 CC4 interrupt
 * (software trigger), see ADC_TRIGGER_HW in motor_config.h. */
RAMFUNC static void control_loop(void)
{
    uint16_t bemf, iraw, vb;

    m.loop_alive = 1;
    iraw = (uint16_t)ADC1->IDATAR1;       /* rank 1: shunt, closest to the trigger */
    bemf = (uint16_t)ADC1->IDATAR2;       /* rank 2: floating phase */
    m.bemf_on = bemf;
    vb   = (uint16_t)ADC1->IDATAR3;       /* rank 3: bus voltage */

    /* ---- time base ---- */
    m.tick++;
    if ((m.tick & (OVR_WINDOW_TICKS - 1U)) == 0U) m.ovr_win = 0;
    if (++m.ms_acc >= (CTRL_TICK_HZ / 1000U))
    {
        m.ms_acc = 0;
        m.ms++;
    }

    /* ---- bus voltage ---- */
    m.vbus_raw = vb;
    m.vbus_filt_x16 += (int32_t)vb - (int32_t)(m.vbus_filt_x16 >> VBUS_FILTER_SHIFT);

    /* ---- current ---- */
    if (m.cal_count)
    {
        m.cal_sum += iraw;
        if (--m.cal_count == 0)
        {
            m.i_offset = (int32_t)(m.cal_sum >> 8);
        }
    }
    /* bridge on: supply collapse and loss of current measurement are faults */
    if ((m.state >= MOTOR_PRECHARGE && m.state <= MOTOR_BEEP) || m.debug_hold)
    {
        if (vb < VBUS_SAG_FAULT_LSB)
        {
            if (++m.sag_count >= VBUS_SAG_SAMPLES)
            {
                enter_fault(MOTOR_FAULT_UNDERVOLTAGE);
                return;
            }
        }
        else
        {
            m.sag_count = 0;
        }
        /* lost ticks break the commutation timing: stop rather than run
         * a badly timed bridge */
        if (m.ovr_win > OVR_WINDOW_MAX)
        {
            m.ovr_win = 0;
            enter_fault(MOTOR_FAULT_OVERRUN);
            return;
        }
        if (m.sample_late)
        {
            if (++m.late_run >= LATE_RUN_MAX)
            {
                enter_fault(MOTOR_FAULT_OVERRUN);
                return;
            }
        }
        else
        {
            m.late_run = 0;
        }
    }

    /* a late interrupt sampled the shunt at a random point of the period,
     * possibly on a switching edge: keep the previous value (bounded by
     * LATE_RUN_MAX above) */
    if (!m.sample_late)
    {
        m.i_raw = (int32_t)iraw - m.i_offset;
        m.i_filt_x16 += m.i_raw - (m.i_filt_x16 >> I_FILTER_SHIFT);
    }

    if (!m.sample_late
        && ((m.state >= MOTOR_PRECHARGE && m.state <= MOTOR_BEEP) || m.debug_hold))
    {
        int32_t ia = (m.i_raw < 0) ? -m.i_raw : m.i_raw;
        if (ia > I_SW_FAULT_LSB)
        {
            if (++m.oc_count >= I_SW_FAULT_SAMPLES)
            {
                enter_fault(MOTOR_FAULT_SW_OVERCURRENT);
                return;
            }
        }
        else
        {
            m.oc_count = 0;
        }
    }

    if (m.debug_hold)
    {
        return;                            /* probe mode: outputs frozen by main() */
    }

    /* ---- BEMF extremes for the trace (after the blanking window) ---- */
    if ((m.state == MOTOR_RAMP || (TRACE_IN_RUN && m.state == MOTOR_RUN))
        && m.step_ticks + 1U > blank_ticks(m.state == MOTOR_RUN ? m.zc_period : m.ramp_period))
    {
        /* same samples as the detector (it runs after step_ticks++) */
#if BEMF_SAMPLE_OFFTIME
        int16_t d = (int16_t)m.bemf_off;   /* raw, against ground */
        /* clamped to Vbus by demagnetisation: not BEMF, keep it out */
        if ((uint32_t)m.bemf_off + ZC_OFF_RAIL_MARGIN_LSB < m.vbus_raw)
#else
        int16_t d = (int16_t)((int32_t)bemf - (int32_t)(m.vbus_filt_x16 >> (4 + 1)));
#endif
        {
            if (d < bemf_min) bemf_min = d;
            if (d > bemf_max) bemf_max = d;
        }
    }

#if BEMF_SAMPLE_OFFTIME
    bemf = m.bemf_off;                     /* detector input: OFF-time sample */
#endif

    /* ---- state machine ---- */
    m.state_ticks++;
    switch (m.state)
    {
    case MOTOR_PRECHARGE:
        if (m.state_ticks >= PRECHARGE_TICKS)
        {
            commutate_to(0);
#if MOTOR_TEST_OPEN_LOOP
            m.duty_target = (uint16_t)(PWM_PERIOD_TICKS * RAMP_TEST_DUTY_PCT / 100U);
#else
            m.duty_target = m.cfg.duty_start;
            m.duty        = m.cfg.duty_start;
#endif
            m.state_ticks = 0;
            m.state = MOTOR_ALIGN;
        }
        break;

    case MOTOR_ALIGN:
#if START_DIRECT && !MOTOR_TEST_OPEN_LOOP
        if (m.state_ticks >= ALIGN_TICKS)
        {
            m.zc_period     = US_TO_TICKS(START_STEP_US);
            m.zc_period_x16 = m.zc_period << 4;
            m.zc_ticks      = 0;
            m.zc_missed     = 0;
            m.cap_duty      = 0;
            m.speed_capped  = 0;
            m.i_fold        = 0;
            m.good_zc       = 0;
            m.start_phase   = 1;
            next_step();
            m.state_ticks   = 0;
            m.state         = MOTOR_RUN;
            trace_add(0x40);
        }
        break;
#endif
#if !MOTOR_TEST_OPEN_LOOP
        start_regulate_current();
#endif
        if (m.state_ticks >= ALIGN_TICKS)
        {
#if MOTOR_TEST_OPEN_LOOP
            m.ramp_period = US_TO_TICKS(RAMP_TEST_START_US);
#else
            m.ramp_period  = m.cfg.ol_start_period;
            /* ramp time ~ 2^shift x (start - end) ticks, plus 2 s */
            m.ramp_timeout = ((uint32_t)1U << m.cfg.accel_shift)
                             * (uint32_t)(m.cfg.ol_start_period - m.cfg.handoff_period)
                             + MS_TO_TICKS(2000);
#endif
            m.zc_ticks    = 0;
            m.good_zc     = 0;
            next_step();
#if MOTOR_TEST_OPEN_LOOP
            m.duty_target = RAMP_DUTY_START;
#endif
            m.state_ticks = 0;
            m.state = MOTOR_RAMP;
        }
        break;

    case MOTOR_RAMP:
        m.step_ticks++;
        m.zc_ticks++;
#if MOTOR_TEST_OPEN_LOOP
        if (m.step_ticks >= m.ramp_period)
        {
            trace_add(m.step);
            next_step();
            /* accelerate by 1/32 per step down to the final rate */
            if (m.ramp_period > US_TO_TICKS(RAMP_TEST_STEP_US))
            {
                m.ramp_period -= (m.ramp_period / 32U) + 1U;
                if (m.ramp_period < US_TO_TICKS(RAMP_TEST_STEP_US))
                {
                    m.ramp_period = US_TO_TICKS(RAMP_TEST_STEP_US);
                }
            }
        }
        /* test mode: fixed duty minus the current fold-back */
        {
            uint16_t d = (uint16_t)(PWM_PERIOD_TICKS * RAMP_TEST_DUTY_PCT / 100U);
            m.duty_target = (d > m.i_fold) ? (uint16_t)(d - m.i_fold) : 0U;
        }
        goto fold_and_slew;
#endif
        if (m.step_ticks >= m.ramp_period)
        {
            if (!m.zc_found)
            {
                m.good_zc = 0;
            }
            trace_add(m.step);
            next_step();
            if (m.ramp_period > m.cfg.handoff_period)
            {
                m.ramp_period -= (m.ramp_period >> m.cfg.accel_shift) + 1U;
                if (m.ramp_period < m.cfg.handoff_period)
                {
                    m.ramp_period = m.cfg.handoff_period;
                }
            }
        }
        else if (!m.zc_found && m.step_ticks > blank_ticks(m.ramp_period)
                 && zc_detect(bemf, (uint8_t)((m.ramp_period <= m.cfg.handoff_period)
                                              ? early_mask(m.ramp_period) : 0U)))
        {
            m.zc_found = 1;
            m.zc_ticks = 0;
            zc_at = (uint8_t)(m.step_ticks > 254U ? 254U : m.step_ticks);
            /* crossings only count at the hand-off rate, where the tune found
             * the BEMF well above the parasitic floor */
            if (m.ramp_period <= m.cfg.handoff_period)
            {
                m.good_zc++;
            }
            if (!m.cfg.ol_hold && m.good_zc >= RAMP_ZC_GOOD_NEEDED)
            {
                m.zc_period    = m.ramp_period;
                m.commutate_at = m.step_ticks + (m.zc_period / 2U) - (m.zc_period >> ZC_ADVANCE_SHIFT);
                m.state_ticks  = 0;
                m.zc_period_x16 = m.zc_period << 4;
                m.zc_missed    = 0;
                m.cap_duty     = 0;
                m.speed_capped = 0;
                m.start_phase  = 0;
                m.i_fold       = 0;
                m.duty_target  = m.duty;   /* start from the ramp duty, slew from there */
                m.state        = MOTOR_RUN;
                trace_add(0x40);
            }
        }
        if (m.state == MOTOR_RAMP)
        {
            start_regulate_current();
            if (!m.cfg.ol_hold && m.state_ticks >= m.ramp_timeout)
            {
                enter_fault(MOTOR_FAULT_STALL);
                return;
            }
        }
        break;

    case MOTOR_RUN:
    {
        m.step_ticks++;
        m.zc_ticks++;
        if (!m.zc_found)
        {
            uint8_t zr = 0;
            if (m.step_ticks > blank_ticks(m.zc_period))
            {
                zr = zc_detect(bemf, early_mask(m.zc_period));
            }
            if (zr)
            {
                /* new period from crossing to crossing, change limited per
                 * step; after a direct start the first crossing keeps the
                 * blind period (zc_ticks spans the whole blind start) */
                uint32_t p16 = m.start_phase ? m.zc_period_x16 : (m.zc_ticks << 4);
                /* early crossing: the step may shorten by 1/16, a real
                 * crossing by 1/8 at low speed */
                uint32_t lo  = m.zc_period_x16 - (m.zc_period_x16 >> ((zr == 2U) ? 4
                                                   : ((m.zc_period > ZC_EARLY_MAX_TICKS) ? 3 : 5)));
                uint32_t hi  = m.zc_period_x16 + (m.zc_period_x16 >> 2);
                if (p16 < lo) p16 = lo;
                if (p16 > hi) p16 = hi;
                m.zc_period_x16 = (m.zc_period_x16 * 3U + p16) >> 2;
                if (m.zc_period_x16 < ((uint32_t)ZC_PERIOD_MIN_TICKS << 4))
                {
                    m.zc_period_x16 = (uint32_t)ZC_PERIOD_MIN_TICKS << 4;
                }
                m.zc_period    = m.zc_period_x16 >> 4;
                m.zc_ticks     = 0;
                m.zc_found     = 1;
                m.zc_missed    = 0;
                m.start_phase  = 0;
                zc_at          = (uint8_t)(m.step_ticks > 254U ? 254U : m.step_ticks);
                if (zr == 2U)
                {
                    /* rotor ahead: the crossing happened at or before this
                     * sample */
#if ZC_EARLY_AT_DETECT
                    uint32_t ca = m.step_ticks + (m.zc_period / 2U) - (m.zc_period >> ZC_ADVANCE_SHIFT);
#else
                    uint32_t ca = m.zc_period - (m.zc_period >> ZC_ADVANCE_SHIFT);
#endif
                    m.commutate_at = (ca > m.step_ticks) ? ca : m.step_ticks + 1U;
                    m.zc_early_cnt++;
                    m.zc_kind = 0x10U;
                }
                else
                {
                    m.commutate_at = m.step_ticks + (m.zc_period / 2U) - (m.zc_period >> ZC_ADVANCE_SHIFT);
                    m.zc_real_cnt++;
                    m.zc_kind = 0U;
                }
            }
            else if (m.start_phase ? (m.step_ticks >= m.zc_period)
                                   : (m.step_ticks >= m.zc_period + (m.zc_period >> 1)))
            {
                /* no crossing seen: commutate blind. During the start the
                 * blind steps accelerate until the BEMF becomes visible. */
                m.zc_blind_cnt++;
                run_step_done((uint8_t)(m.step | 0x20U));
                next_step();
                if (m.start_phase && m.zc_period > US_TO_TICKS(START_BLIND_MIN_US))
                {
                    m.zc_period_x16 -= (m.zc_period_x16 >> 5);
                    m.zc_period      = m.zc_period_x16 >> 4;
                }
                if (++m.zc_missed >= (m.start_phase ? START_MISSED_MAX : ZC_MISSED_MAX))
                {
                    enter_fault(m.start_phase ? MOTOR_FAULT_STALL : MOTOR_FAULT_DESYNC);
                    return;
                }
            }
        }
        else if (m.step_ticks >= m.commutate_at)
        {
            run_step_done((uint8_t)(m.step | m.zc_kind));
            next_step();
        }
        if (!m.start_phase && m.zc_ticks > DESYNC_MIN_TICKS)
        {
            enter_fault(MOTOR_FAULT_DESYNC);
            return;
        }
        m.duty_target = throttle_to_duty();
        /* speed ceiling: at the shortest step the sampling allows, more
         * duty would only raise the current. A duty cap regulates the step
         * period on GOV_PERIOD_X16, proportionally and without a hysteresis
         * band (an on/off cap swept the whole 7..8 tick band: audible surging). */
        if ((m.tick & (GOV_UPDATE_TICKS - 1U)) == 0U)
        {
            int32_t e = (int32_t)m.zc_period_x16 - (int32_t)GOV_PERIOD_X16;  /* > 0: below the ceiling */

            if (m.cap_duty == 0U)
            {
                if (e < 0)
                {
                    m.cap_duty = m.duty;         /* engage at the present duty */
                }
            }
            else
            {
                int32_t step = e / 2;
                int32_t cd;
                if (step >  GOV_STEP_MAX) step =  GOV_STEP_MAX;
                if (step < -GOV_STEP_MAX) step = -GOV_STEP_MAX;
                cd = (int32_t)m.cap_duty + step;
                if (cd >= (int32_t)m.duty_target + GOV_RELEASE_MARGIN)
                {
                    cd = 0;                      /* throttle below the ceiling again */
                }
                else if (cd < (int32_t)RUN_DUTY_FLOOR)
                {
                    cd = RUN_DUTY_FLOOR;
                }
                m.cap_duty = (uint16_t)cd;
            }
            m.speed_capped = (uint8_t)(m.cap_duty != 0U);
        }
        if (m.cap_duty && m.duty_target > m.cap_duty)
        {
            m.duty_target = m.cap_duty;
        }
        break;
    }

    case MOTOR_BEEP:
        /* entry 0 (U+ V-) and entry 3 (V+ U-) alternate: the current reverses
         * every half period, the torque averages out, the windings sing */
        if (m.beep_pre)
        {
            if (--m.beep_pre == 0U)
            {
                apply_regs(&step_regs[0]);
            }
            break;
        }
        if (m.beep_left == 0U)
        {
            go_stopped();
            break;
        }
        m.beep_left--;
        if (++m.beep_cnt >= m.beep_half)
        {
            m.beep_cnt = 0;
            m.beep_pol ^= 1U;
            apply_regs(&step_regs[m.beep_pol ? 3U : 0U]);
        }
        break;

    case MOTOR_STOPPED:
    case MOTOR_BRAKE:
    case MOTOR_FAULT:
    default:
        break;
    }

#if MOTOR_TEST_OPEN_LOOP
fold_and_slew:
#endif
    /* ---- current fold-back ---- */
    if (m.state == MOTOR_RUN || m.state == MOTOR_RAMP || m.state == MOTOR_ALIGN)
    {
        if ((m.i_filt_x16 >> I_FILTER_SHIFT) > I_LIMIT_LSB)
        {
            if ((m.tick & (DUTY_RATE_SCALE - 1U)) == 0U && m.i_fold < (PWM_DUTY_MAX - PWM_DUTY_MIN)) m.i_fold++;
        }
        else if (m.i_fold && (m.tick & (8U * DUTY_RATE_SCALE - 1U)) == 0U)
        {
            m.i_fold--;
        }
        if (m.i_fold > m.i_fold_max) m.i_fold_max = m.i_fold;
    }

    /* ---- cycle-by-cycle peak limit (closed loop only) ----
     * one raw sample above I_PEAK cuts the duty by 1/32 and adds a small
     * step to the slow fold-back */
    if (m.state == MOTOR_RUN && m.i_raw > I_PEAK_LSB)
    {
        uint16_t floor_d = (uint16_t)RUN_DUTY_FLOOR;
        m.duty -= (uint16_t)(m.duty >> 5);
        if (m.duty < floor_d) m.duty = floor_d;
        if (m.i_fold < (PWM_DUTY_MAX - PWM_DUTY_MIN - 2U)) m.i_fold += 2U;
        m.peak_hits++;
    }

    /* ---- duty slew ---- */
    if (m.duty < m.duty_target)
    {
        if (++m.slew_acc >= (uint16_t)(m.cfg.slew_up_ticks * DUTY_RATE_SCALE))
        {
            m.slew_acc = 0;
            m.duty++;
        }
    }
    else if (m.duty > m.duty_target)
    {
        if (++m.slew_acc >= DUTY_SLEW_DOWN_TICKS * DUTY_RATE_SCALE)
        {
            m.slew_acc = 0;
            m.duty--;
        }
    }
    set_duty_regs(m.duty);
}

#if ADC_TRIGGER_HW
void ADC1_IRQHandler(void)
{
    ADC1->STATR = (uint32_t)~ADC_JEOC;
    control_loop();
}

void TIM1_CC_IRQHandler(void)
{
    TIM1->INTFR = (uint16_t)~TIM_IT_CC4;
}
#else
void ADC1_IRQHandler(void)
{
    ADC1->STATR = 0;
}

/* CC4 match while counting down, just before the counter bottom: start the
 * injected sequence by software and wait for it (3 conversions, about 3 us
 * with the ADC at 24 MHz), then run the control step. */
RAMFUNC void TIM1_CC_IRQHandler(void)
{
    /* the three conversions end ~160 ticks after the bottom; give up at
     * CC_WAIT_MAX_TICKS so a missing conversion never eats the period */
    uint32_t guard = 400;

    TIM1->INTFR = (uint16_t)~TIM_IT_CC4;
    ADC1->STATR = (uint32_t)~(ADC_JEOC | ADC_JSTRT);
    /* start and timestamp together: the OFF-time interrupt (higher
     * priority, also due at the bottom) must not slip in between */
    __disable_irq();
    ADC1->CTLR2 |= ADC_JEXTTRIG | ADC_JSWSTART;
    m.dbg_cnt_start = TIM1->CNT;
    m.dbg_dir_start = (TIM1->CTLR1 & TIM_DIR) ? 1U : 0U;
    __enable_irq();
    /* on time: between the CC4 match and a few ticks past the bottom, the
     * shunt sample (held ~23 ticks later) stays near the pulse centre */
    m.sample_late = (uint8_t)!(m.dbg_dir_start ? (m.dbg_cnt_start <= PWM_ADC_TRIGGER_TICKS_SW)
                                               : (m.dbg_cnt_start <= SAMPLE_LATE_TICKS));
    if (m.sample_late) m.cc_late++;
    while ((ADC1->STATR & ADC_JEOC) == 0 && --guard)
    {
        if (!(TIM1->CTLR1 & TIM_DIR) && TIM1->CNT > CC_WAIT_MAX_TICKS)
        {
            guard = 0;
            break;
        }
    }
    m.dbg_cnt_done = TIM1->CNT;
    ADC1->STATR = (uint32_t)~(ADC_JEOC | ADC_JSTRT);
    if (guard)
    {
        control_loop();
    }
    /* CPU budget: the OFF-time BEMF interrupt preempts this one, so the loop
     * only has to end before the next CC4 match (one PWM period) */
    {
        uint16_t cnt = TIM1->CNT;
        uint16_t el  = (TIM1->CTLR1 & TIM_DIR)
                       ? (uint16_t)(PWM_ADC_TRIGGER_TICKS_SW + 2U * PWM_PERIOD_TICKS - cnt)  /* past the top */
                       : (uint16_t)(PWM_ADC_TRIGGER_TICKS_SW + cnt);
        if (el > m.cc_isr_max) m.cc_isr_max = el;
        if (TIM1->INTFR & TIM_IT_CC4)
        {
            m.cc_overrun++;                /* next match already passed: a tick is lost */
            if (m.ovr_win < 255U) m.ovr_win++;
        }
    }
}
#endif

#if BEMF_SAMPLE_OFFTIME
void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
/* Update events at the top and at the bottom of the centre-aligned count.
 * Counting down = the top was just passed = middle of the OFF time: sample
 * the floating phase (regular group, channel set at each commutation). */
RAMFUNC void TIM1_UP_IRQHandler(void)
{
    uint32_t guard = 200;

    TIM1->INTFR = (uint16_t)~TIM_IT_Update;
    /* STRT/JSTRT are sticky, gate on the channel only: a BEMF conversion is
     * short and cannot overlap the injected one */
    if ((TIM1->CTLR1 & TIM_DIR) && m.bemf_ch_ok)
    {
        ADC1->STATR  = (uint32_t)~ADC_EOC;
        ADC1->CTLR2 |= ADC_EXTTRIG | ADC_SWSTART;
        while ((ADC1->STATR & ADC_EOC) == 0 && --guard)
        {
            if (TIM1->CNT < PWM_PERIOD_TICKS - UP_WAIT_MAX_TICKS)
            {
                guard = 0;                 /* far past the top: give up */
                break;
            }
        }
        if (guard)
        {
            m.bemf_off = (uint16_t)ADC1->RDATAR;
        }
        ADC1->STATR = (uint32_t)~(ADC_EOC | ADC_STRT);
    }
    /* NTC while the bridge runs: one conversion right after the BEMF sample,
     * then the regular group goes back to the floating phase. No other TIM1
     * interrupt can preempt this one, so the channel cannot change meanwhile. */
    if ((TIM1->CTLR1 & TIM_DIR) && m.ntc_req)
    {
        uint32_t ch = ADC1->RSQR3;

        guard = 400;                       /* longer NTC sample time */
        ADC1->RSQR3  = BOARD_NTC_ADC_CH;
        ADC1->STATR  = (uint32_t)~ADC_EOC;
        ADC1->CTLR2 |= ADC_EXTTRIG | ADC_SWSTART;
        while ((ADC1->STATR & ADC_EOC) == 0 && --guard)
        {
            if (!(TIM1->CTLR1 & TIM_DIR))
            {
                guard = 0;                 /* bottom reached: leave the loop its slot */
                break;
            }
        }
        if (guard)
        {
            m.ntc_last = (uint16_t)ADC1->RDATAR;
        }
        ADC1->STATR = (uint32_t)~(ADC_EOC | ADC_STRT);
        ADC1->RSQR3 = ch;
        m.ntc_req   = 0;
    }
}
#endif

void TIM1_BRK_IRQHandler(void)
{
    TIM1->INTFR = (uint16_t)~TIM_FLAG_Break;
    /* MOE is already cleared by hardware; record it and park the outputs. */
    enter_fault(MOTOR_FAULT_HW_OVERCURRENT);
}

/* ---------------------------------------------------------- init blocks */
static void gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_AFIO | RCC_PB2Periph_GPIOA | RCC_PB2Periph_GPIOB
                          | RCC_PB2Periph_GPIOC | RCC_PB2Periph_GPIOD, ENABLE);

    /* Gate driver: the internal pre-driver inputs are the GPIO pads
     * PA0/PA2/PD0 (LIN) and PA3/PB0/PB1 (HIN): TIM1 alternate-function
     * push-pull outputs, remap 0100b. */
    GPIO_PinRemapConfig(BOARD_TIM1_REMAP, ENABLE);
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    g.GPIO_Pin = BOARD_HS_U_PIN; GPIO_Init(BOARD_HS_U_PORT, &g);
    g.GPIO_Pin = BOARD_HS_V_PIN; GPIO_Init(BOARD_HS_V_PORT, &g);
    g.GPIO_Pin = BOARD_HS_W_PIN; GPIO_Init(BOARD_HS_W_PORT, &g);
    g.GPIO_Pin = BOARD_LS_U_PIN; GPIO_Init(BOARD_LS_U_PORT, &g);
    g.GPIO_Pin = BOARD_LS_V_PIN; GPIO_Init(BOARD_LS_V_PORT, &g);
    g.GPIO_Pin = BOARD_LS_W_PIN; GPIO_Init(BOARD_LS_W_PORT, &g);

    /* Break pin PB3 is unconnected; the brake source is internal (CMP2). */
    g.GPIO_Mode = GPIO_Mode_IPD;
    g.GPIO_Pin  = BOARD_BKIN_PIN; GPIO_Init(BOARD_BKIN_PORT, &g);

    /* Analog inputs */
    g.GPIO_Mode = GPIO_Mode_AIN;
    g.GPIO_Pin = BOARD_BEMF_U_PIN;  GPIO_Init(BOARD_BEMF_U_PORT, &g);
    g.GPIO_Pin = BOARD_BEMF_V_PIN;  GPIO_Init(BOARD_BEMF_V_PORT, &g);
    g.GPIO_Pin = BOARD_BEMF_W_PIN;  GPIO_Init(BOARD_BEMF_W_PORT, &g);
    g.GPIO_Pin = BOARD_VBAT_PIN;    GPIO_Init(BOARD_VBAT_PORT, &g);
    g.GPIO_Pin = BOARD_NTC_PIN;     GPIO_Init(BOARD_NTC_PORT, &g);
    g.GPIO_Pin = BOARD_SHUNT_P_PIN; GPIO_Init(BOARD_SHUNT_P_PORT, &g);
    g.GPIO_Pin = BOARD_SHUNT_N_PIN; GPIO_Init(BOARD_SHUNT_N_PORT, &g);
}

static void pga_init(void)
{
    OPA_InitTypeDef o;

    OPA_StructInit(&o);
    o.Mode      = OUT_CMP2_ONLY;        /* MODE1 = 11b: output stays internal (ADC IN9 + CMP2),
                                           PD4 remains a free ADC input        */
    o.PSEL      = CHP1;                 /* OPA_P1 = PD7 = shunt +              */
    o.NSEL      = CHN_PGA_16xIN;        /* differential PGA, gain 16           */
    o.FB        = FB_ON;                /* internal feedback network           */
    o.PGADIF    = PGADIF_ON;            /* N input = OPA_N2 = PA4 = shunt -    */
    o.PGA_VBEN  = PGA_VBEN_ON;          /* output biased to VDD/2: bipolar     */
    o.PGA_VBSEL = PGA_VBSEL_VDD_DIV2;
    o.VBCMPSEL  = VBCMPSEL_Mode_0;      /* CMP2 threshold about VDD*30/33      */
    o.OPA_HS    = HS_ON;                /* 40 V/us slew for PWM-rate sampling  */

    OPA_Unlock();
    OPA_Init(&o);
    OPA_Cmd(ENABLE);

    /* CMP2 compares the PGA output with its internal reference and feeds the
     * TIM1 break input directly: hardware over-current, about 66 A. */
    OPA_CMP_Unlock();
#if HW_OC_BREAK_ENABLE
    OPA_CMP_FILT_LEN_Config(CMP_FILT_Len_1);
    OPA_CMP_FILT_Cmd(ENABLE);
    OPA_CMP_TIM1_BKINConfig(TIM1_Brake_Source_CMP2);
    OPA_CMP_Cmd(CMP2, ENABLE);
#else
    OPA_CMP_TIM1_BKINConfig(TIM1_Brake_Source_IO);   /* PB3, pulled down: never active */
    OPA_CMP_Cmd(CMP2, DISABLE);
#endif
}

static void tim1_init(void)
{
    TIM_TimeBaseInitTypeDef tb  = {0};
    TIM_OCInitTypeDef       oc  = {0};
    TIM_BDTRInitTypeDef     bd  = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_TIM1, ENABLE);

    tb.TIM_Prescaler         = 0;
    tb.TIM_CounterMode       = TIM_CounterMode_CenterAligned1;  /* CC events on down-count only */
    tb.TIM_Period            = PWM_PERIOD_TICKS;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &tb);

    oc.TIM_OCMode       = TIM_OCMode_PWM1;
    oc.TIM_OutputState  = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Enable;
    oc.TIM_Pulse        = 0;
    oc.TIM_OCPolarity   = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM1, &oc);
    TIM_OC2Init(TIM1, &oc);
    TIM_OC3Init(TIM1, &oc);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* CH4: ADC injected trigger, no pin output (PD1 is SWIO with remap 4). */
    oc.TIM_OutputState  = TIM_OutputState_Disable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
#if ADC_TRIGGER_HW
    oc.TIM_Pulse        = PWM_ADC_TRIGGER_TICKS;
#else
    oc.TIM_Pulse        = PWM_ADC_TRIGGER_TICKS_SW;
#endif
    TIM_OC4Init(TIM1, &oc);

    bd.TIM_OSSRState       = TIM_OSSRState_Enable;
    bd.TIM_OSSIState       = TIM_OSSIState_Enable;
    bd.TIM_LOCKLevel       = TIM_LOCKLevel_OFF;
    bd.TIM_DeadTime        = PWM_DEAD_TIME_TICKS;
#if HW_OC_BREAK_ENABLE
    bd.TIM_Break           = TIM_Break_Enable;
#else
    bd.TIM_Break           = TIM_Break_Disable;
#endif
    bd.TIM_BreakPolarity   = TIM_BreakPolarity_High;   /* CMP2 output high = over-current */
    bd.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
    TIM_BDTRConfig(TIM1, &bd);

    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_ClearFlag(TIM1, TIM_FLAG_Break);
    TIM_ITConfig(TIM1, TIM_IT_Break, ENABLE);
    /* priorities (bit 7 = preemption, nesting enabled in the startup code):
     * the short OFF-time BEMF interrupt preempts the control loop, so a long
     * loop never delays the BEMF sample; break and control do not preempt
     * each other (no state race) */
    NVIC_SetPriority(TIM1_BRK_IRQn, 0x80);
    NVIC_EnableIRQ(TIM1_BRK_IRQn);
#if BEMF_SAMPLE_OFFTIME
    TIM_ClearFlag(TIM1, TIM_IT_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    NVIC_SetPriority(TIM1_UP_IRQn, 0x00);
    NVIC_EnableIRQ(TIM1_UP_IRQn);
#endif
#if !ADC_TRIGGER_HW
    TIM_ClearFlag(TIM1, TIM_IT_CC4);
    TIM_ITConfig(TIM1, TIM_IT_CC4, ENABLE);
    NVIC_SetPriority(TIM1_CC_IRQn, 0x80);
    NVIC_EnableIRQ(TIM1_CC_IRQn);
#endif

    apply_regs(&regs_off);
    set_duty_regs(0);
    outputs_disable();
    /* the counter is started by motor_init() once the ADC is powered: a CC4
     * interrupt before that would wait for a conversion that never comes */
}

static void adc_init(void)
{
    ADC_InitTypeDef a = {0};
    uint8_t p;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div2);     /* 24 MHz */

    a.ADC_Mode               = ADC_Mode_Independent;
    a.ADC_ScanConvMode       = ENABLE;
    a.ADC_ContinuousConvMode = DISABLE;
    a.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;   /* regular: software */
    a.ADC_DataAlign          = ADC_DataAlign_Right;
    a.ADC_NbrOfChannel       = 1;
    ADC_Init(ADC1, &a);

    /* Injected group, once per PWM period at the TIM1 CC4 point:
     *   1: PGA output (shunt current)
     *   2: floating phase (ON-time method, channel set at each commutation)
     *   3: bus voltage */
    ADC_InjectedSequencerLengthConfig(ADC1, 3);
    ADC_InjectedChannelConfig(ADC1, BOARD_SHUNT_ADC_CH,  1, ADC_SampleTime_CyclesMode2);
    ADC_InjectedChannelConfig(ADC1, BOARD_BEMF_W_ADC_CH, 2, ADC_SampleTime_CyclesMode2);
    ADC_InjectedChannelConfig(ADC1, BOARD_VBAT_ADC_CH,   3, ADC_SampleTime_CyclesMode2);
#if ADC_TRIGGER_HW
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4);
#else
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);  /* JSWSTART */
#endif
    ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);

    /* sample time of all three BEMF channels: a channel left at its reset
     * value keeps part of the previous conversion */
    for (p = 0; p < 3; p++)
    {
        uint32_t sh = 3U * bemf_adc_ch[p];
        ADC1->SAMPTR2 = (ADC1->SAMPTR2 & ~(7UL << sh)) | ((uint32_t)ADC_SampleTime_CyclesMode2 << sh);
    }

    for (p = 0; p < 3; p++)
    {
        /* JL = 2: rank 1 sits in JSQ2 (bits 9:5), rank 2 in JSQ3 (bits 14:10) */
        isqr_for_phase[p] = (ADC1->ISQR & ~(0x1FUL << 10)) | ((uint32_t)bemf_adc_ch[p] << 10);
    }

    /* regular group: OFF-time BEMF sample (TIM1 update) or NTC (main) */
    ADC_RegularChannelConfig(ADC1, BOARD_NTC_ADC_CH, 1, ADC_SampleTime_CyclesMode5);

    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
#if ADC_TRIGGER_HW
    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
    NVIC_SetPriority(ADC_IRQn, 0x00);
    NVIC_EnableIRQ(ADC_IRQn);
#endif
    /* normal mode: the reset default (low power) is meant for < 1 Msps */
    ADC1->CTLR3 &= ~ADC_LP;
    ADC_Cmd(ADC1, ENABLE);
    Delay_Us(20);                          /* tSTAB after power-up */
}

/* ---------------------------------------------------------------- API */
uint8_t motor_init(void)
{
    uint8_t s;
    uint16_t wait_ms;

    for (s = 0; s < 6; s++)
    {
        build_regs(step_table[s][0], (uint8_t)(1U << step_table[s][1]), &step_regs[s]);
    }
    build_regs(PH_NONE, 0x07, &regs_brake);   /* every low side on */
    build_regs(PH_NONE, 0x00, &regs_off);
    for (s = 0; s < 3; s++)
    {
        build_regs(PH_NONE, (uint8_t)(1U << s), &regs_low_only[s]);
    }

    {
        motor_start_cfg_t d;
        d.i_start_ma      = START_I_DEFAULT_MA;
        d.duty_start      = ALIGN_DUTY;
        d.duty_max_ol     = START_DUTY_MAX_OL;
        d.ol_start_period = RAMP_START_STEP_TICKS;
        d.handoff_period  = RAMP_END_STEP_TICKS;
        d.accel_shift     = START_ACCEL_SHIFT_DEFAULT;
        d.ol_hold         = 0;
        d.slew_up_ticks   = DUTY_SLEW_UP_TICKS;
        d.reserved        = 0;
        motor_set_start_cfg(&d);
    }
    m.state      = MOTOR_STOPPED;
    m.fault      = MOTOR_FAULT_NONE;
    m.duty_limit = 1000;
    m.i_offset   = BOARD_I_ZERO_EXPECTED;
    m.zc_period  = RAMP_END_STEP_TICKS;

    gpio_init();
    pga_init();
    tim1_init();
    adc_init();
    TIM_Cmd(TIM1, ENABLE);                 /* control loop starts here */

    /* Zero-current calibration of the PGA output: 256 samples with the
     * bridge off. The fast loop is already running at this point. */
    Delay_Ms(20);
    m.cal_sum   = 0;
    m.cal_count = 256;
    for (wait_ms = 0; wait_ms < 200 && m.cal_count; wait_ms++)
    {
        Delay_Ms(1);                       /* normally done in about 11 ms */
    }
    if (m.cal_count || !m.loop_alive)
    {
        outputs_disable();
        return 1;                          /* control loop is not running */
    }
    /* Start the voltage filter from a sane value */
    m.vbus_filt_x16 = (uint32_t)m.vbus_raw << VBUS_FILTER_SHIFT;
    return 0;
}

void motor_start(void)
{
    __disable_irq();
    if (m.state == MOTOR_STOPPED)
    {
        m.debug_hold  = 0;                 /* leave any probe mode */
        motor_trace_n = 0;                 /* new trace for every start */
        motor_trace_i = 0;
        m.peak_hits   = 0;
        m.zc_real_cnt = 0;
        m.zc_early_cnt = 0;
        m.zc_blind_cnt = 0;
        m.cc_isr_max   = 0;
        m.cc_overrun   = 0;
        m.cc_late      = 0;
        {
            uint8_t k;
            for (k = 0; k < 6U; k++)
            {
                zc_stat[k].real = 0; zc_stat[k].early = 0; zc_stat[k].blind = 0;
                zc_stat[k].vmin = 0; zc_stat[k].vmax = 0;
            }
        }
        bemf_min = 32767;
        bemf_max = -32767;
        m.duty        = 0;
        m.duty_target = 0;
        m.i_fold      = 0;
        m.oc_count    = 0;
        set_duty_regs(0);
        apply_regs(&regs_brake);
        m.state_ticks = 0;
        m.state       = MOTOR_PRECHARGE;
        outputs_enable();
    }
    __enable_irq();
}

void motor_stop(void)
{
    __disable_irq();
    if (m.state != MOTOR_FAULT)
    {
        go_stopped();
    }
    __enable_irq();
}

void motor_brake(void)
{
    __disable_irq();
    if (m.state != MOTOR_FAULT)
    {
        m.duty = 0;
        m.duty_target = 0;
        set_duty_regs(0);
        apply_regs(&regs_brake);
        m.state = MOTOR_BRAKE;
        outputs_enable();
    }
    __enable_irq();
}

void motor_set_throttle(uint16_t permille)
{
    m.throttle = (permille > 1000U) ? 1000U : permille;
}

void motor_set_direction(uint8_t reverse)
{
    if (m.state == MOTOR_STOPPED)
    {
        m.reverse = reverse ? 1U : 0U;
    }
}

void motor_set_duty_limit(uint16_t permille)
{
    m.duty_limit = (permille > 1000U) ? 1000U : permille;
}

void motor_raise_fault(motor_fault_t fault)
{
    __disable_irq();
    if (m.state != MOTOR_FAULT)
    {
        enter_fault(fault);
    }
    __enable_irq();
}

void motor_clear_fault(void)
{
    __disable_irq();
    if (m.state == MOTOR_FAULT)
    {
        TIM_ClearFlag(TIM1, TIM_FLAG_Break);
        m.fault = MOTOR_FAULT_NONE;
        go_stopped();
    }
    __enable_irq();
}

void motor_debug_outputs(uint8_t mode, uint16_t duty_permille)
{
    uint16_t d = (uint16_t)(((uint32_t)PWM_PERIOD_TICKS * (duty_permille > 1000U ? 1000U : duty_permille)) / 1000U);

    __disable_irq();
    if (m.state == MOTOR_FAULT)
    {
        __enable_irq();
        return;                            /* a fault latched: never re-enable */
    }
    m.debug_hold = 1;
    m.state = MOTOR_STOPPED;
    set_duty_regs(d);
    if (mode == 0)
    {
        apply_regs(&regs_off);
        outputs_disable();
    }
    else
    {
        if (mode == 1)
        {
            apply_regs(&regs_brake);
        }
        else if (mode >= 8 && mode <= 10)
        {
            apply_regs(&regs_low_only[mode - 8U]);   /* 8 = U, 9 = V, 10 = W */
        }
        else
        {
            uint8_t st = (uint8_t)((mode - 2U) % 6U);
            m.step = st;
            apply_regs(&step_regs[st]);
            ADC1->ISQR = isqr_for_phase[step_table[st][2]];
#if BEMF_SAMPLE_OFFTIME
            ADC1->RSQR3  = bemf_adc_ch[step_table[st][2]];
            m.bemf_ch_ok = 1;
#endif
        }
        outputs_enable();
    }
    __enable_irq();
}

void motor_debug_bemf(uint16_t *on, uint16_t *off, uint16_t *vbus)
{
    *on   = m.bemf_on;
    *off  = m.bemf_off;
    *vbus = m.vbus_raw;
}

uint16_t motor_debug_iraw(void)
{
    return (uint16_t)(m.i_raw + m.i_offset);
}

void motor_beep(uint16_t freq_hz, uint16_t ms, uint16_t duty_permille)
{
    uint16_t d;

    if (freq_hz == 0U || ms == 0U)
    {
        return;
    }
    d = (uint16_t)(((uint32_t)PWM_PERIOD_TICKS * (duty_permille > 100U ? 100U : duty_permille)) / 1000U);
    __disable_irq();
    if (m.state == MOTOR_STOPPED)
    {
        m.debug_hold  = 0;
        m.beep_half   = (uint16_t)(CTRL_TICK_HZ / (2U * freq_hz));
        if (m.beep_half == 0U) m.beep_half = 1U;
        m.beep_cnt    = 0;
        m.beep_pol    = 0;
        m.beep_left   = MS_TO_TICKS(ms);
        m.oc_count    = 0;
        m.duty        = d;
        m.duty_target = d;
        set_duty_regs(d);
        /* charge the bootstrap capacitors first, as a motor start does */
        apply_regs(&regs_brake);
        m.beep_pre    = PRECHARGE_TICKS;
        m.state       = MOTOR_BEEP;
        outputs_enable();
    }
    __enable_irq();
}

uint8_t motor_beep_busy(void)
{
    return (uint8_t)(m.state == MOTOR_BEEP);
}

motor_state_t motor_state(void)         { return m.state; }
motor_fault_t motor_fault(void)         { return m.fault; }
uint32_t      motor_fault_time_ms(void) { return m.fault_ms; }
uint32_t      motor_millis(void)        { return m.ms; }
uint16_t      motor_peak_hits(void)     { return m.peak_hits; }

void motor_zc_counts(uint16_t *real, uint16_t *early, uint16_t *blind)
{
    *real  = m.zc_real_cnt;
    *early = m.zc_early_cnt;
    *blind = m.zc_blind_cnt;
}

void motor_get_diag(motor_diag_t *d)
{
    d->cc_overrun  = m.cc_overrun;
    d->cc_isr_max  = m.cc_isr_max;
    d->cc_late     = m.cc_late;
    d->fault_state = m.fault_state;
    d->fault_duty  = m.fault_duty;
    d->fault_i_ma  = m.fault_i_raw * BOARD_I_MA_PER_LSB_NUM / BOARD_I_MA_PER_LSB_DEN;
}

uint16_t motor_fold_max_pm(void)
{
    uint16_t f = m.i_fold_max;
    m.i_fold_max = 0;
    return (uint16_t)(((uint32_t)f * 1000U) / PWM_PERIOD_TICKS);
}
uint16_t      motor_duty(void)          { return (uint16_t)(((uint32_t)m.duty * 1000U) / PWM_PERIOD_TICKS); }
uint16_t      motor_throttle(void)      { return m.throttle; }

uint32_t motor_erpm(void)
{
    uint32_t p16 = m.zc_period_x16;
    if (m.state != MOTOR_RUN || p16 == 0)
    {
        return 0;
    }
    /* one electrical turn = 6 steps of p ticks: erpm = 60 * f_tick / (6 p),
     * p = p16 / 16 */
    return (160U * CTRL_TICK_HZ) / p16;
}

int32_t motor_current_ma(void)
{
    int32_t lsb = m.i_filt_x16 >> I_FILTER_SHIFT;
    return (lsb * BOARD_I_MA_PER_LSB_NUM) / BOARD_I_MA_PER_LSB_DEN;
}

uint16_t motor_vbus_mv(void)
{
    uint32_t lsb = m.vbus_filt_x16 >> VBUS_FILTER_SHIFT;
    return (uint16_t)((lsb * BOARD_VBUS_MV_NUM) / BOARD_VBUS_MV_DEN);
}

uint16_t motor_read_ntc_raw(void)
{
    uint32_t guard = 20000;

#if BEMF_SAMPLE_OFFTIME
    /* the regular group samples the floating phase while the bridge runs:
     * the OFF-time interrupt converts the NTC on request, the value returned
     * here is from the previous call (NTC_PERIOD_MS old at most) */
    if (m.state != MOTOR_STOPPED && m.state != MOTOR_FAULT)
    {
        m.ntc_req = 1;
        return m.ntc_last;
    }
    __disable_irq();
    m.bemf_ch_ok = 0;
    ADC1->RSQR3  = BOARD_NTC_ADC_CH;
#endif
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET && --guard)
    {
    }
    m.ntc_last = (uint16_t)ADC1->RDATAR;
#if BEMF_SAMPLE_OFFTIME
    __enable_irq();
#endif
    return m.ntc_last;
}
