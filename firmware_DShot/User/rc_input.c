/* SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Cogless-Commercial
 * Copyright (c) 2026 Ronan Mingon */
/*
 * rc_input.c - servo PWM capture on PA1, see rc_input.h.
 * EXTI on both edges, time-stamped by TIM2 as a free 1 MHz counter (no TIM2
 * remap: it would take PA0/PA2/PA3 from the gate driver).
 */
#include "rc_input.h"
#include "board.h"
#include "motor.h"

static volatile uint16_t rc_pulse;
static volatile uint32_t rc_last_ms;
static volatile uint8_t  rc_seen;
static volatile uint16_t rc_rise_ts;

void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void EXTI7_0_IRQHandler(void)
{
    if (EXTI->INTFR & EXTI_Line1)
    {
        uint16_t now = (uint16_t)TIM2->CNT;
        EXTI->INTFR = EXTI_Line1;                       /* write 1 clears */
        if (GPIO_ReadInputDataBit(BOARD_RC_PORT, BOARD_RC_PIN))
        {
            rc_rise_ts = now;                           /* rising edge */
        }
        else
        {
            uint16_t width = (uint16_t)(now - rc_rise_ts);  /* falling edge */
            if (width >= RC_PULSE_MIN_US && width <= RC_PULSE_MAX_US)
            {
                rc_pulse   = width;
                rc_last_ms = motor_millis();
                rc_seen    = 1;
            }
        }
    }
    else
    {
        EXTI->INTFR = 0xFF;
    }
}

void rc_init(void)
{
    GPIO_InitTypeDef        g  = {0};
    TIM_TimeBaseInitTypeDef tb = {0};
    EXTI_InitTypeDef        e  = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_AFIO | RCC_PB2Periph_GPIOA, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_TIM2, ENABLE);

    g.GPIO_Pin   = BOARD_RC_PIN;
    g.GPIO_Mode  = GPIO_Mode_IPD;                          /* open input reads low */
    g.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(BOARD_RC_PORT, &g);

    /* TIM2: free-running 1 us counter, no remap, no channel, no output. */
    tb.TIM_Prescaler     = (uint16_t)(BOARD_SYSCLK_HZ / 1000000U - 1U);
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    tb.TIM_Period        = 0xFFFF;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &tb);
    TIM_Cmd(TIM2, ENABLE);

    /* EXTI line 1 on PA1, both edges. */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
    e.EXTI_Line    = EXTI_Line1;
    e.EXTI_Mode    = EXTI_Mode_Interrupt;
    e.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    e.EXTI_LineCmd = ENABLE;
    EXTI_Init(&e);
    EXTI_ClearITPendingBit(EXTI_Line1);
    NVIC_SetPriority(EXTI7_0_IRQn, 0x80);
    NVIC_EnableIRQ(EXTI7_0_IRQn);
}

uint8_t rc_valid(void)
{
    if (!rc_seen)
    {
        return 0;
    }
    return (motor_millis() - rc_last_ms) <= RC_TIMEOUT_MS;
}

uint16_t rc_pulse_us(void)
{
    return rc_pulse;
}

uint16_t rc_throttle(void)
{
    uint16_t p;

    if (!rc_valid())
    {
        return 0;
    }
    p = rc_pulse;
    if (p <= RC_THROTTLE_LOW_US)  return 0;
    if (p >= RC_THROTTLE_HIGH_US) return 1000;
    return (uint16_t)(((uint32_t)(p - RC_THROTTLE_LOW_US) * 1000U)
                      / (RC_THROTTLE_HIGH_US - RC_THROTTLE_LOW_US));
}
