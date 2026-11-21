/*
 * MCXScope firmware: SCPI line parser and command set (PROTOCOL.md §2-§8).
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef MCXSCOPE_SCPI_H
#define MCXSCOPE_SCPI_H

#include <stddef.h>
#include <stdint.h>

void scpi_init(void);

/* Feed bytes received from the host. Complete lines are executed immediately and their
 * replies written with cdc_write()/cdc_flush(). */
void scpi_input(const uint8_t *data, size_t len);

#endif
