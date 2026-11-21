/*
 * MCXScope firmware: settings, run control and capture (PROTOCOL.md §4-§9).
 *
 * Arming starts the ADCs; DMA fills one ring per captured channel continuously (see
 * adc_capture.h). The main loop scans the trigger channel's new samples for an edge,
 * starting at sample `pre` so the pre-trigger part of the record is real data. Once a
 * trigger is found, the sample clock is told to stop after the last post-trigger sample;
 * the record is then read straight out of the rings, with no copy.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "adc_capture.h"
#include "scope.h"
#include "timebase.h"
#include "usb_cdc.h"

#define MAX_SCAN_PER_POLL 16384U /* trigger-search samples per scope_poll() call */
#define RING_MARGIN       256U   /* stay this far behind the DMA's overwrite point */

const uint32_t g_rates[] = RATE_LIST;
const uint32_t g_depths[] = DEPTH_LIST;
const float g_ranges[] = RANGE_LIST;
const unsigned g_numRates = sizeof(g_rates) / sizeof(g_rates[0]);
const unsigned g_numDepths = sizeof(g_depths) / sizeof(g_depths[0]);
const unsigned g_numRanges = sizeof(g_ranges) / sizeof(g_ranges[0]);

scope_settings_t g_set;

/* volts per raw code; codes are 0..ADC_CODE_MAX for 0..VREF (offset 0 V) */
#define VOLTS_PER_CODE (ADC_VREF / ADC_CODE_FULL)

/* ---- capture in progress (parameters fixed at arm time) ---- */
static struct
{
    scope_state_t state;
    bool continuous;
    uint32_t rate, depth, pre;
    uint16_t rec_mask; /* channels in the record (enabled ones) */
    unsigned src;
    float lvl_c;           /* trigger level in codes */
    int32_t fire_r, arm_r_below; /* rising: fire if c >= fire_r, arm if c < arm_r_below */
    int32_t fire_f, arm_f_above; /* falling: fire if c <= fire_f, arm if c > arm_f_above */
    trig_edge_t edge;
    uint64_t arm_us;
    uint64_t search_k; /* next sample to examine for a trigger */
    bool arm_r, arm_f;
    bool have_prev;
    int32_t prev_c;
    bool force_req;
    bool pending; /* trigger found, waiting for the post-trigger samples */
    bool forced;
    uint64_t trig_k; /* first sample at or after the crossing */
    float frac;
} cap;

/* ---- completed record (its samples stay in the capture rings) ---- */
static struct
{
    uint16_t mask;
    uint8_t nch;
    uint8_t flags;
    uint32_t depth;
    uint32_t rate;
    uint32_t trig_index;
    float frac;
    uint32_t sequence;
    uint64_t start_k;
} rec;

static uint32_t s_sequence;

/* ------------------------------------------------------------------------------------ */

void scope_reset(void)
{
    adc_capture_stop();
    for (unsigned i = 0; i < NUM_CHANNELS; i++)
    {
        g_set.ch[i].enabled = (i == 0U);
        g_set.ch[i].range = g_ranges[g_numRanges - 1U];
        g_set.ch[i].offset = DEFAULT_OFFSET_V;
    }
    g_set.rate = g_rates[g_numRates - 1U];
    g_set.depth_req = g_depths[0];
    for (unsigned i = 0; i < g_numDepths; i++)
    {
        if (g_depths[i] <= 10000U)
        {
            g_set.depth_req = g_depths[i];
        }
    }
    g_set.trig_mode = MODE_AUTO;
    g_set.trig_src = 0;
    g_set.trig_edge = EDGE_RIS;
    g_set.trig_level = g_set.ch[0].offset;
    g_set.trig_pos = 0.5f;
    g_set.trig_hyst = -1.0f;

    cap.state = ST_STOP;
    cap.continuous = false;
}

void scope_init(void)
{
    s_sequence = 0;
    adc_capture_init();
    scope_reset();
}

uint32_t scope_effective_depth(void)
{
    /* count every channel that will be captured: the enabled ones plus the trigger source,
     * which is sampled even when hidden and takes its own share of sample memory */
    unsigned n = 0;
    for (unsigned i = 0; i < NUM_CHANNELS; i++)
    {
        n += (g_set.ch[i].enabled || i == g_set.trig_src) ? 1U : 0U;
    }
    uint32_t best = 0, smallest_fit = 0;
    for (unsigned i = 0; i < g_numDepths; i++)
    {
        uint32_t d = g_depths[i];
        if (d * n > MAX_POINTS)
        {
            continue;
        }
        if (smallest_fit == 0U)
        {
            smallest_fit = d;
        }
        if (d <= g_set.depth_req)
        {
            best = d;
        }
    }
    if (best != 0U)
    {
        return best;
    }
    return smallest_fit ? smallest_fit : g_depths[0];
}

uint32_t scope_pretrig(void)
{
    return (uint32_t)lroundf(g_set.trig_pos * (float)scope_effective_depth());
}

float scope_hyst(void)
{
    if (g_set.trig_hyst >= 0.0f)
    {
        return g_set.trig_hyst;
    }
    return 0.02f * g_set.ch[g_set.trig_src].range;
}

static void arm(bool continuous)
{
    memset(&cap, 0, sizeof(cap));
    cap.state = ST_ARMED;
    cap.continuous = continuous;
    cap.rate = g_set.rate;
    cap.depth = scope_effective_depth();
    cap.pre = scope_pretrig();
    cap.src = g_set.trig_src;
    cap.edge = g_set.trig_edge;

    for (unsigned i = 0; i < NUM_CHANNELS; i++)
    {
        if (g_set.ch[i].enabled)
        {
            cap.rec_mask |= (uint16_t)(1U << i);
        }
    }

    /* thresholds in integer codes: c < x  <=>  c < ceil(x);  c > x  <=>  c > floor(x) */
    float lvl = g_set.trig_level / VOLTS_PER_CODE;
    float hh = scope_hyst() / VOLTS_PER_CODE / 2.0f;
    cap.lvl_c = lvl;
    cap.fire_r = (int32_t)ceilf(lvl);
    cap.arm_r_below = (int32_t)ceilf(lvl - hh);
    cap.fire_f = (int32_t)floorf(lvl);
    cap.arm_f_above = (int32_t)floorf(lvl + hh);

    /* the earliest trigger is sample `pre`, so pre-trigger data is always real */
    cap.search_k = cap.pre;
    cap.arm_us = timebase_us();
    /* the trigger source is always sampled, even if it isn't displayed */
    adc_capture_start(cap.rate, (uint16_t)(cap.rec_mask | (1U << cap.src)), cap.src);
}

void scope_run(bool continuous)
{
    arm(continuous);
}

void scope_stop(void)
{
    if (cap.state == ST_ARMED)
    {
        adc_capture_stop();
        cap.state = ST_STOP;
    }
    else if (cap.state == ST_TRIGGERED)
    {
        cap.continuous = false; /* record stays readable; don't re-arm after it */
    }
}

void scope_force(void)
{
    if (cap.state == ST_ARMED)
    {
        cap.force_req = true;
        scope_poll();
    }
}

void scope_settings_changed(void)
{
    if (cap.state == ST_ARMED)
    {
        arm(cap.continuous);
    }
}

scope_state_t scope_state(void)
{
    return cap.state;
}

/* Search samples [search_k, limit) of the trigger channel. Returns true and sets
 * cap.trig_k / cap.frac on a trigger. */
static bool search(uint64_t limit)
{
    const int16_t *ring = adc_capture_ring(cap.src);
    const uint32_t len = adc_capture_ring_len();

    if (limit <= cap.search_k)
    {
        return false; /* e.g. still filling the pre-trigger part: nothing new to scan */
    }
    if (limit > cap.search_k + (len - RING_MARGIN))
    {
        /* Fell too far behind: the samples have been overwritten. Skip ahead; the
         * hysteresis state starts over. */
        cap.search_k = limit - len / 2U;
        cap.have_prev = false;
        cap.arm_r = cap.arm_f = false;
    }
    if (limit > cap.search_k + MAX_SCAN_PER_POLL)
    {
        limit = cap.search_k + MAX_SCAN_PER_POLL;
    }

    const bool ris = (cap.edge != EDGE_FALL), fall = (cap.edge != EDGE_RIS);
    uint32_t idx = (uint32_t)(cap.search_k % len);
    bool arm_r = cap.arm_r, arm_f = cap.arm_f;
    int32_t prev = cap.prev_c;
    bool have_prev = cap.have_prev;

    for (uint64_t k = cap.search_k; k < limit; k++)
    {
        int32_t c = ring[idx];
        if (++idx == len)
        {
            idx = 0;
        }
        bool fire = false;
        if (ris)
        {
            if (c >= cap.fire_r)
            {
                fire = arm_r;
                arm_r = false;
            }
            else if (c < cap.arm_r_below)
            {
                arm_r = true;
            }
        }
        if (fall)
        {
            if (c <= cap.fire_f)
            {
                fire = fire || arm_f;
                arm_f = false;
            }
            else if (c > cap.arm_f_above)
            {
                arm_f = true;
            }
        }
        if (fire)
        {
            cap.trig_k = k;
            cap.frac = 0.0f;
            if (have_prev && c != prev)
            {
                /* crossing at (k - 1) + f; trigger_frac is measured back from k */
                float f = (cap.lvl_c - (float)prev) / (float)(c - prev);
                f = fminf(fmaxf(f, 0.0f), 1.0f);
                cap.frac = fminf(1.0f - f, 0.999999f);
            }
            cap.search_k = k + 1U;
            return true;
        }
        prev = c;
        have_prev = true;
    }
    cap.search_k = limit;
    cap.arm_r = arm_r;
    cap.arm_f = arm_f;
    cap.prev_c = prev;
    cap.have_prev = have_prev;
    return false;
}

static bool clipped_in(const int16_t *ring, uint32_t len, uint64_t start, uint32_t n)
{
    uint32_t idx = (uint32_t)(start % len);
    for (uint32_t i = 0; i < n; i++)
    {
        int16_t c = ring[idx];
        if (c <= 0 || c >= ADC_CODE_MAX)
        {
            return true;
        }
        if (++idx == len)
        {
            idx = 0;
        }
    }
    return false;
}

static void make_record(void)
{
    const uint32_t len = adc_capture_ring_len();
    uint64_t end = cap.trig_k - cap.pre + cap.depth;
    uint64_t taken = adc_capture_count();
    bool overrun = adc_capture_overrun() || (taken - (end - cap.depth) > len) || taken < end;
    bool clipped = false;

    rec.mask = cap.rec_mask;
    rec.nch = 0;
    for (unsigned ch = 0; ch < NUM_CHANNELS; ch++)
    {
        if (rec.mask & (1U << ch))
        {
            rec.nch++;
            clipped = clipped || clipped_in(adc_capture_ring(ch), len, end - cap.depth, cap.depth);
        }
    }
    rec.depth = cap.depth;
    rec.rate = cap.rate;
    rec.trig_index = cap.pre;
    rec.frac = cap.forced ? 0.0f : cap.frac;
    rec.flags = (uint8_t)((cap.forced ? 1U : 0U) | (overrun ? 2U : 0U) | (clipped ? 4U : 0U));
    rec.sequence = ++s_sequence;
    rec.start_k = end - cap.depth;
    cap.state = ST_TRIGGERED;
}

void scope_poll(void)
{
    if (cap.state != ST_ARMED)
    {
        return;
    }

    if (!cap.pending)
    {
        uint64_t n = adc_capture_count();
        if (cap.force_req)
        {
            cap.pending = true;
            cap.forced = true;
            cap.trig_k = (n > cap.search_k) ? n : cap.search_k;
        }
        else if (search(n))
        {
            cap.pending = true;
            cap.forced = false;
        }
        else if (g_set.trig_mode == MODE_AUTO)
        {
            uint64_t timeout_us = 2ULL * cap.depth * 1000000ULL / cap.rate;
            if (timeout_us < 100000U)
            {
                timeout_us = 100000U;
            }
            if (timebase_us() - cap.arm_us > timeout_us)
            {
                /* re-read: the search above took time, and a stale count would put the
                 * end of the record in the past */
                n = adc_capture_count();
                cap.pending = true;
                cap.forced = true;
                cap.trig_k = (n > cap.search_k) ? n : cap.search_k;
            }
        }
        if (cap.pending)
        {
            adc_capture_stop_at(cap.trig_k - cap.pre + cap.depth);
        }
    }

    if (cap.pending && adc_capture_stopped())
    {
        make_record();
    }
}

/* Measure the sample clock: samples per second over ~200 ms at the current rate. */
uint32_t scope_diag_measured_rate(bool *overrun)
{
    bool was_armed = (cap.state == ST_ARMED);
    adc_capture_start(g_set.rate, 3U, 0U); /* both ADCs: worst case */
    uint64_t t0 = timebase_us(), n0 = adc_capture_count();
    while (timebase_us() - t0 < 200000U)
    {
    }
    uint64_t t1 = timebase_us(), n1 = adc_capture_count();
    *overrun = adc_capture_overrun();
    adc_capture_stop();
    if (was_armed)
    {
        arm(cap.continuous);
    }
    else if (cap.state == ST_TRIGGERED)
    {
        cap.state = ST_STOP; /* the rings were overwritten */
    }
    return (uint32_t)((n1 - n0) * 1000000ULL / (t1 - t0));
}

/* ---- readout ---- */

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
    put16(p, (uint16_t)v);
    put16(p + 2, (uint16_t)(v >> 16));
}

static void putf(uint8_t *p, float f)
{
    uint32_t v;
    memcpy(&v, &f, sizeof(v));
    put32(p, v);
}

bool scope_send_record(void)
{
    if (cap.state != ST_TRIGGERED)
    {
        cdc_write("#10\n", 4);
        return false;
    }

    uint8_t hdr[40 + 8 * NUM_CHANNELS];
    uint32_t header_len = 40U + 8U * rec.nch;
    uint32_t data_len = 2U * rec.depth * rec.nch;
    uint64_t period_fs = (1000000000000000ULL + rec.rate / 2U) / rec.rate;

    memset(hdr, 0, sizeof(hdr));
    memcpy(hdr, "MCXW", 4);
    put16(hdr + 4, 1);
    put16(hdr + 6, (uint16_t)header_len);
    put16(hdr + 8, rec.mask);
    hdr[10] = ADC_BITS;
    hdr[11] = rec.flags;
    put32(hdr + 12, rec.depth);
    put32(hdr + 16, (uint32_t)period_fs);
    put32(hdr + 20, (uint32_t)(period_fs >> 32));
    put32(hdr + 24, rec.trig_index);
    putf(hdr + 28, rec.frac);
    put32(hdr + 32, rec.sequence);
    put32(hdr + 36, 0);
    for (unsigned i = 0; i < rec.nch; i++)
    {
        putf(hdr + 40 + 8 * i, VOLTS_PER_CODE); /* volts = code * vpc + 0 */
        putf(hdr + 44 + 8 * i, 0.0f);
    }

    char len_str[12], prefix[16];
    int nd = snprintf(len_str, sizeof(len_str), "%lu", (unsigned long)(header_len + data_len));
    int np = snprintf(prefix, sizeof(prefix), "#%d%s", nd, len_str);
    cdc_write(prefix, (size_t)np);
    cdc_write(hdr, header_len);

    /* each channel's record is a (possibly wrapped) slice of its ring; int16 samples are
     * already little-endian on Cortex-M */
    const uint32_t len = adc_capture_ring_len();
    for (unsigned ch = 0; ch < NUM_CHANNELS; ch++)
    {
        if ((rec.mask & (1U << ch)) == 0U)
        {
            continue;
        }
        const int16_t *ring = adc_capture_ring(ch);
        uint32_t idx = (uint32_t)(rec.start_k % len);
        uint32_t first = len - idx;
        if (first > rec.depth)
        {
            first = rec.depth;
        }
        cdc_write(&ring[idx], first * 2U);
        cdc_write(&ring[0], (rec.depth - first) * 2U);
    }
    cdc_write("\n", 1);

    if (cap.continuous)
    {
        arm(true);
    }
    else
    {
        cap.state = ST_STOP;
    }
    return true;
}
