/*
 * MCXScope firmware: settings, run control and capture (PROTOCOL.md §4-§9).
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef MCXSCOPE_SCOPE_H
#define MCXSCOPE_SCOPE_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"

typedef enum { EDGE_RIS, EDGE_FALL, EDGE_ANY } trig_edge_t;
typedef enum { MODE_AUTO, MODE_NORM } trig_mode_t;
typedef enum { ST_STOP, ST_ARMED, ST_TRIGGERED } scope_state_t;

typedef struct
{
    bool enabled;
    float range;  /* full-scale span, V */
    float offset; /* V at the centre of the range */
} channel_t;

typedef struct
{
    channel_t ch[NUM_CHANNELS];
    uint32_t rate;      /* Hz */
    uint32_t depth_req; /* requested depth, before the MAXPoints limit */
    trig_mode_t trig_mode;
    uint8_t trig_src;   /* 0-based channel */
    trig_edge_t trig_edge;
    float trig_level;   /* V */
    float trig_pos;     /* 0..1, fraction of the record before the trigger */
    float trig_hyst;    /* V, < 0 = default (2 % of the source range) */
} scope_settings_t;

extern scope_settings_t g_set;

extern const uint32_t g_rates[];
extern const uint32_t g_depths[];
extern const float g_ranges[];
extern const unsigned g_numRates, g_numDepths, g_numRanges;

void scope_init(void);
void scope_reset(void);           /* *RST: defaults, STOP, record discarded */

uint32_t scope_effective_depth(void);
uint32_t scope_pretrig(void);     /* pre-trigger samples for the effective depth */
float scope_hyst(void);

void scope_run(bool continuous);  /* :RUN / :SINGle */
void scope_stop(void);
void scope_force(void);
void scope_settings_changed(void); /* restart an armed capture with the new settings */
void scope_poll(void);            /* call often from the main loop */
scope_state_t scope_state(void);

/* :DIAGnostic:RATE?  Runs the sample clock for ~200 ms and returns the measured rate. */
uint32_t scope_diag_measured_rate(bool *overrun);

/* :WAVeform:DATA?  Sends the record block (or #10 and returns false if none is ready). */
bool scope_send_record(void);

#endif
