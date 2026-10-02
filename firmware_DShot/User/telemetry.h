/* SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Cogless-Commercial
 * Copyright (c) 2026 Ronan Mingon */
/*
 * telemetry.h - text output over the 1-wire SDI debug link (WCH-LinkE).
 *
 * The stock printf in Debug/debug.c blocks forever when no debugger reads
 * the mailbox, which would freeze the ESC. This writer gives up after a
 * short timeout and then stays silent until reboot, so the firmware runs
 * the same with or without a WCH-Link attached.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

void tel_init(void);
void tel_puts(const char *s);
void tel_put_int(int32_t v);
void tel_put_uint(uint32_t v);
uint8_t tel_active(void);

#endif /* TELEMETRY_H */
