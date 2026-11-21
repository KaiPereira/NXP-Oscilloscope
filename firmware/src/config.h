/*
 * MCXScope firmware: device capabilities (PROTOCOL.md §3).
 *
 * Everything the host can discover about the hardware lives here.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef MCXSCOPE_CONFIG_H
#define MCXSCOPE_CONFIG_H

#define FW_VENDOR   "OpenHW"
#define FW_MODEL    "MCXScope-2CH"
#define FW_SERIAL   "PROTO001"
#define FW_VERSION  "0.2.2"

#define NUM_CHANNELS 2
#define ADC_BITS     12

#define RATE_LIST    {1000u, 10000u, 100000u, 1000000u, 2000000u} /* 2 MS/s = ADC conversion limit at 48 MHz */
/* 30000 only fits one captured channel (DMA ring limit 32000 samples per channel) */
#define DEPTH_LIST   {1000u, 2000u, 5000u, 10000u, 20000u, 30000u}
#define MAX_POINTS   40000u /* depth x enabled channels */

/* Sample memory shared by the capture rings (int16). The part beyond MAX_POINTS is slack
 * that lets the sample clock run on briefly after the record is complete. */
#define SAMPLE_BUF_TOTAL 50000u

/*
 * Analog input: ADC pins driven directly, 0 V .. VREF (VDDA). Voltages outside 0..3.3 V
 * can damage the MCU.
 *
 * There is no gain or offset hardware, so range and offset only set the display window
 * the host draws (and clamp the trigger level). The ADC always converts the full
 * 0..VREF, records always carry absolute volts, and CLIPPED means the ADC limit, not the
 * window edge. A smaller range zooms in; it does not add resolution (1 code = 0.8 mV).
 */
#define ADC_VREF       3.3f
#define RANGE_LIST     {0.1f, 0.2f, 0.5f, 1.0f, 2.0f, ADC_VREF}
#define DEFAULT_OFFSET_V (ADC_VREF / 2.0f) /* centre of the ADC span: window = 0..3.3 V */
#define OFFSET_LIMIT_V 10.0f               /* window centre may pan within +/- this */

#define MAX_LINE 128 /* bytes, including the terminator (§2) */

#endif
