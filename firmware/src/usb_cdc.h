/*
 * MCXScope firmware: USB CDC-ACM byte stream.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef MCXSCOPE_USB_CDC_H
#define MCXSCOPE_USB_CDC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void cdc_init(void);

/* Copy up to `max` received bytes into `dst`. Returns the count (0 if none). Non-blocking. */
size_t cdc_read(uint8_t *dst, size_t max);

/* Queue bytes for the host. Blocks while the staging buffer is being sent. */
void cdc_write(const void *data, size_t len);

/* Send everything queued and end the transfer (adds a zero-length packet when needed). */
void cdc_flush(void);

#endif
