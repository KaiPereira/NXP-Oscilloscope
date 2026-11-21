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
#define FW_VERSION  "0.2.0"

#define NUM_CHANNELS 2
#define ADC_BITS     12

#define RATE_LIST    {1000u, 10000u, 100000u, 1000000u, 2000000u} /* 2 MS/s = ADC conversion limit at 48 MHz */
#define DEPTH_LIST   {1000u, 2000u, 5000u, 10000u, 20000u}
#define MAX_POINTS   40000u /* depth x enabled channels */

/* Sample memory shared by the capture rings (int16). The part beyond MAX_POINTS is slack
 * that lets the sample clock run on briefly after the record is complete. */
#define SAMPLE_BUF_TOTAL 50000u

/*
 * Analog input: ADC pins driven directly, 0 V .. VREF (VDDA). One fixed range, centred
 * at VREF/2, as PROTOCOL.md §4 describes for a 0..Vref input without an offset DAC.
 * Voltages outside 0..3.3 V can damage the MCU.
 */
#define ADC_VREF       3.3f
#define RANGE_LIST     {ADC_VREF}
#define FIXED_OFFSET_V (ADC_VREF / 2.0f)

#define MAX_LINE 128 /* bytes, including the terminator (§2) */

#endif
