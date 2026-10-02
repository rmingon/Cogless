/* SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Cogless-Commercial
 * Copyright (c) 2026 Ronan Mingon */
/*
 * board.h - Cogless ESC hardware definition (CH32M007G8R6, SSOP-28)
 *
 * Every pin below comes from hardware/hardware.kicad_sch. Change this file
 * (and only this file) if the board is re-spun.
 *
 *  Pin  Signal            Net         Function used by the firmware
 *  ---  ----------------  ----------  --------------------------------------------
 *   1   PD7/OPA_P1        OP1         Shunt + (PGA positive input)
 *  28   PA4/OPA_N2        A-          Shunt - Kelvin (PGA negative input, PGADIF)
 *   2   PD2/ADC_IN3       PD2         Phase U back-EMF divider (33k/8.2k)
 *   3   PD3/ADC_IN4       PD3         Phase V back-EMF divider
 *   4   PD4/ADC_IN7       PD4         Phase W back-EMF divider
 *   5   PD5/ADC_IN5       BATT_VOLT   Battery divider (33k/8.2k)
 *   6   PD6/ADC_IN6       NTC         10k B3500 NTC to +5V, 10k to GND
 *  10   LO1 (PA0/T1C1N)   LGU         Low-side gate U
 *  11   LO2 (PA2/T1C2N)   LGV         Low-side gate V
 *  12   LO3 (PD0/T1C3N)   LGW         Low-side gate W
 *  15   HO1 (PA3/T1C1)    HGU         High-side gate U
 *  18   HO2 (PB0/T1C2)    HGV         High-side gate V
 *  21   HO3 (PB1/T1C3)    HGW         High-side gate W
 *  22   PB3/T1BK          n.c.        TIM1 break pin (unused, internal CMP2 used)
 *  23   PC2/PD1/SWIO      SWIO        Debug (TIM1_CH4 lands here with remap 4: keep CC4 output off)
 *  24   PA1/ADC_IN1       PA1         Control input header J2.3 (servo PWM, EXTI1)
 *  26   PC4               LED         Status LED, active high (D1 + 1.1k to GND)
 *  27   PC5/RST           -           Reset, 10k pull-up
 */
#ifndef BOARD_H
#define BOARD_H

#include "ch32v00X.h"

/* ---------------------------------------------------------------- clocks */
#define BOARD_SYSCLK_HZ          48000000UL
#define BOARD_TIM1_CLK_HZ        BOARD_SYSCLK_HZ

/* ------------------------------------------------------------ gate driver
 * TIM1 remap 0100b routes CH1..3 / CH1N..3N to the internal pre-driver
 * (datasheet 1.4.19). GPIO_PartialRemap4_TIM1 == TIM1_RM = 0100b.
 */
#define BOARD_TIM1_REMAP         GPIO_PartialRemap4_TIM1

#define BOARD_HS_U_PORT          GPIOA
#define BOARD_HS_U_PIN           GPIO_Pin_3
#define BOARD_HS_V_PORT          GPIOB
#define BOARD_HS_V_PIN           GPIO_Pin_0
#define BOARD_HS_W_PORT          GPIOB
#define BOARD_HS_W_PIN           GPIO_Pin_1
#define BOARD_LS_U_PORT          GPIOA
#define BOARD_LS_U_PIN           GPIO_Pin_0
#define BOARD_LS_V_PORT          GPIOA
#define BOARD_LS_V_PIN           GPIO_Pin_2
#define BOARD_LS_W_PORT          GPIOD
#define BOARD_LS_W_PIN           GPIO_Pin_0

#define BOARD_BKIN_PORT          GPIOB
#define BOARD_BKIN_PIN           GPIO_Pin_3

/* ---------------------------------------------------------------- analog */
#define BOARD_BEMF_U_PORT        GPIOD
#define BOARD_BEMF_U_PIN         GPIO_Pin_2
#define BOARD_BEMF_U_ADC_CH      ADC_Channel_3
#define BOARD_BEMF_V_PORT        GPIOD
#define BOARD_BEMF_V_PIN         GPIO_Pin_3
#define BOARD_BEMF_V_ADC_CH      ADC_Channel_4
#define BOARD_BEMF_W_PORT        GPIOD
#define BOARD_BEMF_W_PIN         GPIO_Pin_4
#define BOARD_BEMF_W_ADC_CH      ADC_Channel_7

#define BOARD_VBAT_PORT          GPIOD
#define BOARD_VBAT_PIN           GPIO_Pin_5
#define BOARD_VBAT_ADC_CH        ADC_Channel_5

#define BOARD_NTC_PORT           GPIOD
#define BOARD_NTC_PIN            GPIO_Pin_6
#define BOARD_NTC_ADC_CH         ADC_Channel_6

#define BOARD_SHUNT_P_PORT       GPIOD
#define BOARD_SHUNT_P_PIN        GPIO_Pin_7      /* OPA_P1  -> PSEL = CHP1 */
#define BOARD_SHUNT_N_PORT       GPIOA
#define BOARD_SHUNT_N_PIN        GPIO_Pin_4      /* OPA_N2  -> PGADIF = 1 */
#define BOARD_SHUNT_ADC_CH       ADC_Channel_OPA /* internal IN9 */

/* ---------------------------------------------------------- control input */
#define BOARD_RC_PORT            GPIOA
#define BOARD_RC_PIN             GPIO_Pin_1
/* PA1 uses EXTI line 1, not a TIM2 channel: TIM2 remaps 5/6 would take
 * PA0/PA2/PA3 from TIM1 (gate driver). */
#define BOARD_RC_EXTI_LINE       EXTI_Line1

/* ------------------------------------------------------------------- LED */
#define BOARD_LED_PORT           GPIOC
#define BOARD_LED_PIN            GPIO_Pin_4

/* ------------------------------------------------------- scaling factors
 * ADC: 12 bit, VREF = VDD = 5.0 V  -> 1.2207 mV / LSB
 * Phase and battery dividers: 33k / 8.2k -> Vin = Vadc * 41.2 / 8.2
 * Full-scale on the divider: 4096 LSB = 25.1 V (16.8 V pack = 2744 LSB)
 */
#define BOARD_ADC_FULL_SCALE     4096
#define BOARD_VDD_MV             5000
#define BOARD_DIV_TOP_OHM        33000
#define BOARD_DIV_BOT_OHM        8200
/* millivolts = adc * BOARD_VBUS_MV_NUM / BOARD_VBUS_MV_DEN  (6.1333 mV/LSB) */
#define BOARD_VBUS_MV_NUM        6133
#define BOARD_VBUS_MV_DEN        1000

/* Current sense: 2 mOhm shunt, differential PGA x16 with VDD/2 bias.
 * Reference manual table 17-2: AC gain in biased mode = 16 - 0.5 = 15.5.
 * 1 LSB = 1.2207 mV / 15.5 / 2 mOhm = 39.4 mA.  Full scale about +/- 80 A.
 * Zero-current output sits at VDD/2 (about 2048 LSB), calibrated at boot.
 */
#define BOARD_SHUNT_MOHM         2
#define BOARD_PGA_GAIN_X10       155          /* 15.5 */
#define BOARD_I_MA_PER_LSB_NUM   3938         /* 39.38 mA / LSB */
#define BOARD_I_MA_PER_LSB_DEN   100
#define BOARD_I_ZERO_EXPECTED    2048

/* Hardware over-current: CMP2 threshold ~ VDD*30/33 = 4.55 V on the PGA output
 * -> (4.55 - 2.5) / 15.5 / 2 mOhm = about 66 A. Trips TIM1 break directly. */

/* NTC TH1 = Cantherm CMFA103J3500HANT (10k at 25 C, 5 %, B 3500 K) on top,
 * R24 10k to GND: ADC counts rise with temperature.
 * adc = 4096 * 10k / (10k + 10k * exp(B * (1/T - 1/298.15))) */
#define BOARD_NTC_B_K            3500
#define BOARD_NTC_TABLE_T_MIN_C  (-20)
#define BOARD_NTC_TABLE_STEP_C   10
#define BOARD_NTC_TABLE                                                         \
    { 452, 710, 1043, 1431, 1844, 2246, 2609, 2919, 3172, 3373, 3529, 3650, \
      3742, 3814, 3869, 3912, 3946 }                       /* -20 .. 140 C */

#endif /* BOARD_H */
