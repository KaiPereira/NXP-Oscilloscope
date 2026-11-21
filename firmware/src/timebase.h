/*
 * MCXScope firmware: millisecond / microsecond time since boot (SysTick).
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef MCXSCOPE_TIMEBASE_H
#define MCXSCOPE_TIMEBASE_H

#include <stdint.h>

void timebase_init(void);
uint32_t timebase_ms(void);
uint64_t timebase_us(void);

#endif
