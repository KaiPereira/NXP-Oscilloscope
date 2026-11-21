/*
 * MCXScope firmware: ADC sampling hardware.
 *
 *   CTIMER0 match 0 ──INPUTMUX──┬─> ADC1 trigger 0 ──FIFO──DMA ch0──> ring (CH1, P1_10 / J4-2)
 *                               └─> ADC0 trigger 0 ──FIFO──DMA ch1──> ring (CH2, P2_1  / J2-7)
 *
 * One timer edge starts a conversion on both ADCs, so the channels are sampled
 * simultaneously. Each DMA channel writes into its own circular buffer ("ring")
 * indefinitely; the CPU only reads the rings and the DMA position.
 *
 * Samples are numbered from 0 at adc_capture_start(): sample k of a channel lives at
 * ring[k % ring_len] until it is overwritten ring_len samples later.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef MCXSCOPE_ADC_CAPTURE_H
#define MCXSCOPE_ADC_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"

void adc_capture_init(void);

/* Start sampling the channels in `mask` (bit n = CHANn+1) at `rate` Hz.
 * `count_ch` is the channel whose DMA position defines adc_capture_count(). */
void adc_capture_start(uint32_t rate, uint16_t mask, unsigned count_ch);

/* Stop the sample clock as soon as `count` samples have been taken (checked from the
 * 10 kHz tick, so it may overshoot by up to ~100 us of samples). */
void adc_capture_stop_at(uint64_t count);
void adc_capture_stop(void);
bool adc_capture_stopped(void); /* true once stopped and the last samples have landed */

uint64_t adc_capture_count(void);   /* samples taken so far on `count_ch` */
uint32_t adc_capture_ring_len(void);
const int16_t *adc_capture_ring(unsigned ch); /* NULL if ch is not being captured */
bool adc_capture_overrun(void);     /* an ADC FIFO overflowed: samples were lost */

/* Self-test: square waves on the (unconnected) inputs via the pins' pull resistors.
 * Don't enable with a real signal connected: the ~50 kOhm pulls would load it. */
void adc_capture_test_signal(bool on);
bool adc_capture_test_signal_on(void);

/* Call from the 10 kHz tick interrupt. */
void adc_capture_tick(void);

/* Raw codes: 12-bit conversions left-justified in D[14:3], so 0..32760 for 0..VREF. */
#define ADC_CODE_FULL 32768.0f
#define ADC_CODE_MAX  32760

#endif
