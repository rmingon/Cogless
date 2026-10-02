/* SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Cogless-Commercial
 * Copyright (c) 2026 Ronan Mingon */
/*
 * telemetry.c - non-blocking SDI text output, see telemetry.h.
 *
 * Mailbox protocol (same as Debug/debug.c): DATA0 low byte = length (1..7),
 * DATA0 bytes 1..3 and DATA1 bytes 0..3 = payload. The debugger clears
 * DATA0 once it has read the packet.
 */
#include "telemetry.h"
#include "ch32v00X.h"

#define SDI_DATA0   (*(volatile uint32_t *)0xE00000F4)
#define SDI_DATA1   (*(volatile uint32_t *)0xE00000F8)
#define SDI_WAIT_LOOPS  20000U      /* about 1 ms at 48 MHz */

static uint8_t active;

void tel_init(void)
{
    SDI_DATA0 = 0;
    active = 1;
}

uint8_t tel_active(void)
{
    return active;
}

static uint8_t sdi_wait_free(void)
{
    uint32_t n = SDI_WAIT_LOOPS;
    while (SDI_DATA0 != 0U)
    {
        if (--n == 0U)
        {
            active = 0;         /* nobody is listening: go quiet for good */
            return 0;
        }
    }
    return 1;
}

static void sdi_write(const char *buf, uint8_t len)
{
    uint32_t d0 = len, d1 = 0;
    uint8_t i;

    for (i = 0; i < len && i < 3; i++)
    {
        d0 |= (uint32_t)(uint8_t)buf[i] << (8U * (i + 1U));
    }
    for (i = 3; i < len; i++)
    {
        d1 |= (uint32_t)(uint8_t)buf[i] << (8U * (i - 3U));
    }
    SDI_DATA1 = d1;
    SDI_DATA0 = d0;
}

void tel_puts(const char *s)
{
    uint8_t len;

    if (!active)
    {
        return;
    }
    while (*s)
    {
        for (len = 0; len < 7 && s[len]; len++)
        {
        }
        if (!sdi_wait_free())
        {
            return;
        }
        sdi_write(s, len);
        s += len;
    }
}

void tel_put_uint(uint32_t v)
{
    char buf[11];
    uint8_t i = 10;

    buf[i] = '\0';
    do
    {
        buf[--i] = (char)('0' + (v % 10U));
        v /= 10U;
    } while (v);
    tel_puts(&buf[i]);
}

void tel_put_int(int32_t v)
{
    if (v < 0)
    {
        tel_puts("-");
        tel_put_uint((uint32_t)(-v));
    }
    else
    {
        tel_put_uint((uint32_t)v);
    }
}
