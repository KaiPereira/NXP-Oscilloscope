/*
 * MCXScope firmware: SCPI line parser and command set (PROTOCOL.md §2-§8).
 *
 * The command handling mirrors host/tools/fake_scope.py, the reference implementation.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "adc_capture.h"
#include "config.h"
#include "scope.h"
#include "scpi.h"
#include "usb_cdc.h"

/* ---- error queue (§2.2) ---- */

#define ERRQ_SIZE 8

enum
{
    E_SYNTAX = -102,
    E_PARAM_NOT_ALLOWED = -108,
    E_MISSING_PARAM = -109,
    E_UNDEFINED_HEADER = -113,
    E_SUFFIX_RANGE = -114,
    E_SETTINGS_CONFLICT = -221,
    E_DATA_RANGE = -222,
    E_TOO_MUCH_DATA = -223,
    E_QUEUE_OVERFLOW = -350,
};

static int s_errq[ERRQ_SIZE];
static unsigned s_errCount;

static void err_push(int code)
{
    if (s_errCount >= ERRQ_SIZE)
    {
        s_errq[ERRQ_SIZE - 1] = E_QUEUE_OVERFLOW;
        return;
    }
    s_errq[s_errCount++] = code;
}

static int err_pop(void)
{
    if (s_errCount == 0U)
    {
        return 0;
    }
    int code = s_errq[0];
    memmove(&s_errq[0], &s_errq[1], (s_errCount - 1U) * sizeof(s_errq[0]));
    s_errCount--;
    return code;
}

static const char *err_msg(int code)
{
    switch (code)
    {
        case 0: return "No error";
        case E_SYNTAX: return "Syntax error";
        case E_PARAM_NOT_ALLOWED: return "Parameter not allowed";
        case E_MISSING_PARAM: return "Missing parameter";
        case E_UNDEFINED_HEADER: return "Undefined header";
        case E_SUFFIX_RANGE: return "Header suffix out of range";
        case E_SETTINGS_CONFLICT: return "Settings conflict";
        case E_DATA_RANGE: return "Data out of range";
        case E_TOO_MUCH_DATA: return "Too much data";
        case E_QUEUE_OVERFLOW: return "Queue overflow";
        default: return "Unknown error";
    }
}

/* ---- parsed line ---- */

#define MAX_NODES 5

static char *s_nodes[MAX_NODES];
static unsigned s_numNodes;
static const char *s_arg; /* "" if none */
static bool s_query;
static char s_resp[200];  /* text reply to a query */

/*
 * Match the parsed header against a pattern such as "CHANnel#:RANGe:LIST".
 * Each node may be given in short form (the uppercase letters) or long form.
 * '#' marks a numeric suffix, which defaults to 1. Returns false if no match.
 */
static bool match(const char *pattern, unsigned *suffix)
{
    char pat[48];
    strncpy(pat, pattern, sizeof(pat) - 1U);
    pat[sizeof(pat) - 1U] = '\0';

    unsigned i = 0;
    char *save = NULL;
    for (char *p = strtok_r(pat, ":", &save); p != NULL; p = strtok_r(NULL, ":", &save), i++)
    {
        if (i >= s_numNodes)
        {
            return false;
        }
        size_t plen = strlen(p);
        bool want_suffix = (plen > 0U && p[plen - 1U] == '#');
        if (want_suffix)
        {
            p[--plen] = '\0';
        }
        char short_form[16], long_form[24];
        size_t ns = 0;
        for (size_t j = 0; j < plen && j < sizeof(long_form) - 1U; j++)
        {
            if (isupper((unsigned char)p[j]) || p[j] == '*')
            {
                short_form[ns++] = p[j];
            }
            long_form[j] = (char)toupper((unsigned char)p[j]);
        }
        short_form[ns] = '\0';
        long_form[plen] = '\0';

        /* split the node into word + digits */
        const char *node = s_nodes[i];
        size_t wlen = 0;
        while (node[wlen] != '\0' && !isdigit((unsigned char)node[wlen]))
        {
            wlen++;
        }
        const char *digits = &node[wlen];
        for (const char *d = digits; *d != '\0'; d++)
        {
            if (!isdigit((unsigned char)*d))
            {
                return false;
            }
        }
        bool word_ok = (wlen == strlen(short_form) && strncmp(node, short_form, wlen) == 0) ||
                       (wlen == strlen(long_form) && strncmp(node, long_form, wlen) == 0);
        if (!word_ok)
        {
            return false;
        }
        if (*digits != '\0' && !want_suffix)
        {
            return false;
        }
        if (want_suffix && suffix != NULL)
        {
            *suffix = (*digits != '\0') ? (unsigned)strtoul(digits, NULL, 10) : 1U;
        }
    }
    return i == s_numNodes;
}

/* ---- argument parsing; each returns 0 or a negative error code ---- */

static int need_arg(void)
{
    return (s_arg[0] == '\0') ? E_MISSING_PARAM : 0;
}

static int need_noarg(void)
{
    return (s_arg[0] != '\0') ? E_PARAM_NOT_ALLOWED : 0;
}

static int parse_real(float *out)
{
    int e = need_arg();
    if (e)
    {
        return e;
    }
    char *end;
    double v = strtod(s_arg, &end);
    if (end == s_arg || *end != '\0')
    {
        return E_SYNTAX;
    }
    if (!isfinite(v))
    {
        return E_DATA_RANGE;
    }
    *out = (float)v;
    return 0;
}

static int parse_uint(uint32_t *out)
{
    int e = need_arg();
    if (e)
    {
        return e;
    }
    char *end;
    double v = strtod(s_arg, &end);
    if (end == s_arg || *end != '\0')
    {
        return E_SYNTAX;
    }
    if (!isfinite(v) || v < 0.0 || v > 4294967295.0)
    {
        return E_DATA_RANGE;
    }
    *out = (uint32_t)llround(v);
    return 0;
}

static bool arg_is(const char *a, const char *b)
{
    return strcasecmp(a, b) == 0;
}

static int parse_bool(bool *out)
{
    int e = need_arg();
    if (e)
    {
        return e;
    }
    if (arg_is(s_arg, "1") || arg_is(s_arg, "ON"))
    {
        *out = true;
        return 0;
    }
    if (arg_is(s_arg, "0") || arg_is(s_arg, "OFF"))
    {
        *out = false;
        return 0;
    }
    return E_DATA_RANGE;
}

/* nearest supported value; ties go to the lower one */
static uint32_t nearest(const uint32_t *opts, unsigned n, uint32_t v)
{
    uint32_t best = opts[0];
    uint32_t best_d = (v > best) ? v - best : best - v;
    for (unsigned i = 1; i < n; i++)
    {
        uint32_t d = (v > opts[i]) ? v - opts[i] : opts[i] - v;
        if (d < best_d)
        {
            best = opts[i];
            best_d = d;
        }
    }
    return best;
}

/* ---- reply helpers ---- */

static void reply(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void reply(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_resp, sizeof(s_resp), fmt, ap);
    va_end(ap);
}

static void reply_real(float v)
{
    reply("%.6g", (double)v);
}

static void reply_list_u32(const uint32_t *v, unsigned n)
{
    size_t pos = 0;
    s_resp[0] = '\0';
    for (unsigned i = 0; i < n && pos < sizeof(s_resp); i++)
    {
        pos += (size_t)snprintf(&s_resp[pos], sizeof(s_resp) - pos, i ? ",%lu" : "%lu", (unsigned long)v[i]);
    }
}

/* ---- command handlers ---- */

static int channel(unsigned suffix, channel_t **out)
{
    if (suffix < 1U || suffix > NUM_CHANNELS)
    {
        return E_SUFFIX_RANGE;
    }
    *out = &g_set.ch[suffix - 1U];
    return 0;
}

static const char *edge_name(trig_edge_t e)
{
    return (e == EDGE_RIS) ? "RIS" : (e == EDGE_FALL) ? "FALL" : "ANY";
}

static void clamp_trigger_level(void)
{
    const channel_t *c = &g_set.ch[g_set.trig_src];
    float lo = c->offset - c->range / 2.0f, hi = c->offset + c->range / 2.0f;
    g_set.trig_level = fminf(fmaxf(g_set.trig_level, lo), hi);
}

/* Returns 0 on success or a negative error code. Query replies go to s_resp, except
 * :WAVeform:DATA?, which streams its own block (*streamed is set). */
static int dispatch(bool *streamed)
{
    const bool q = s_query;
    unsigned sfx = 1;
    channel_t *c;
    int e;

    /* --- common --- */
    if (q && match("*IDN", NULL))
    {
        reply("%s,%s,%s,%s", FW_VENDOR, FW_MODEL, FW_SERIAL, FW_VERSION);
        return 0;
    }
    if (!q && match("*RST", NULL))
    {
        scope_reset();
        return 0;
    }
    if (!q && match("*CLS", NULL))
    {
        s_errCount = 0;
        return 0;
    }
    if (q && match("*OPC", NULL))
    {
        reply("1");
        return 0;
    }
    if (q && match("SYSTem:ERRor", NULL))
    {
        int code = err_pop();
        reply("%d,\"%s\"", code, err_msg(code));
        return 0;
    }
    if (q && match("SYSTem:PROTocol", NULL))
    {
        reply("1");
        return 0;
    }
    if (q && match("SYSTem:CHANnels", NULL))
    {
        reply("%d", NUM_CHANNELS);
        return 0;
    }
    if (q && match("SYSTem:BITS", NULL))
    {
        reply("%d", ADC_BITS);
        return 0;
    }

    /* --- acquisition --- */
    if (q && match("ACQuire:RATE:LIST", NULL))
    {
        reply_list_u32(g_rates, g_numRates);
        return 0;
    }
    if (q && match("ACQuire:DEPTh:LIST", NULL))
    {
        reply_list_u32(g_depths, g_numDepths);
        return 0;
    }
    if (q && match("ACQuire:MAXPoints", NULL))
    {
        reply("%lu", (unsigned long)MAX_POINTS);
        return 0;
    }
    if (match("ACQuire:RATE", NULL))
    {
        if (q)
        {
            reply("%lu", (unsigned long)g_set.rate);
            return 0;
        }
        uint32_t v;
        if ((e = parse_uint(&v)) != 0)
        {
            return e;
        }
        g_set.rate = nearest(g_rates, g_numRates, v);
        scope_settings_changed();
        return 0;
    }
    if (match("ACQuire:DEPTh", NULL))
    {
        if (q)
        {
            reply("%lu", (unsigned long)scope_effective_depth());
            return 0;
        }
        uint32_t v;
        if ((e = parse_uint(&v)) != 0)
        {
            return e;
        }
        g_set.depth_req = nearest(g_depths, g_numDepths, v);
        scope_settings_changed();
        return 0;
    }

    /* --- channels --- */
    if (q && match("CHANnel#:RANGe:LIST", &sfx))
    {
        if ((e = channel(sfx, &c)) != 0)
        {
            return e;
        }
        size_t pos = 0;
        for (unsigned i = 0; i < g_numRanges; i++)
        {
            pos += (size_t)snprintf(&s_resp[pos], sizeof(s_resp) - pos, i ? ",%g" : "%g", (double)g_ranges[i]);
        }
        return 0;
    }
    if (q && match("CHANnel#:COUPling:LIST", &sfx))
    {
        if ((e = channel(sfx, &c)) != 0)
        {
            return e;
        }
        reply("DC");
        return 0;
    }
    if (match("CHANnel#:ENABle", &sfx))
    {
        if ((e = channel(sfx, &c)) != 0)
        {
            return e;
        }
        if (q)
        {
            reply("%d", c->enabled ? 1 : 0);
            return 0;
        }
        bool v;
        if ((e = parse_bool(&v)) != 0)
        {
            return e;
        }
        c->enabled = v;
        scope_settings_changed();
        return 0;
    }
    if (match("CHANnel#:RANGe", &sfx))
    {
        if ((e = channel(sfx, &c)) != 0)
        {
            return e;
        }
        if (q)
        {
            reply_real(c->range);
            return 0;
        }
        float v;
        if ((e = parse_real(&v)) != 0)
        {
            return e;
        }
        if (v <= 0.0f)
        {
            return E_DATA_RANGE;
        }
        /* smallest supported range >= request, else the largest */
        c->range = g_ranges[g_numRanges - 1U];
        for (unsigned i = 0; i < g_numRanges; i++)
        {
            if (g_ranges[i] >= v * (1.0f - 1e-6f))
            {
                c->range = g_ranges[i];
                break;
            }
        }
        clamp_trigger_level();
        scope_settings_changed();
        return 0;
    }
    if (match("CHANnel#:OFFSet", &sfx))
    {
        if ((e = channel(sfx, &c)) != 0)
        {
            return e;
        }
        if (q)
        {
            reply_real(c->offset);
            return 0;
        }
        float v;
        if ((e = parse_real(&v)) != 0)
        {
            return e;
        }
        /* display window only (see config.h): the ADC data is unaffected */
        c->offset = fminf(fmaxf(v, -OFFSET_LIMIT_V), OFFSET_LIMIT_V);
        clamp_trigger_level();
        scope_settings_changed();
        return 0;
    }
    if (match("CHANnel#:COUPling", &sfx))
    {
        if ((e = channel(sfx, &c)) != 0)
        {
            return e;
        }
        if (q)
        {
            reply("DC");
            return 0;
        }
        if ((e = need_arg()) != 0)
        {
            return e;
        }
        if (arg_is(s_arg, "DC"))
        {
            return 0;
        }
        return arg_is(s_arg, "AC") ? E_SETTINGS_CONFLICT : E_DATA_RANGE;
    }

    /* --- trigger --- */
    if (match("TRIGger:MODE", NULL))
    {
        if (q)
        {
            reply(g_set.trig_mode == MODE_AUTO ? "AUTO" : "NORM");
            return 0;
        }
        if ((e = need_arg()) != 0)
        {
            return e;
        }
        if (arg_is(s_arg, "AUTO"))
        {
            g_set.trig_mode = MODE_AUTO;
        }
        else if (arg_is(s_arg, "NORM") || arg_is(s_arg, "NORMAL"))
        {
            g_set.trig_mode = MODE_NORM;
        }
        else
        {
            return E_DATA_RANGE;
        }
        return 0;
    }
    if (match("TRIGger:SOURce", NULL))
    {
        if (q)
        {
            reply("CHAN%u", g_set.trig_src + 1U);
            return 0;
        }
        if ((e = need_arg()) != 0)
        {
            return e;
        }
        const char *digits;
        if (strncasecmp(s_arg, "CHANNEL", 7) == 0)
        {
            digits = s_arg + 7;
        }
        else if (strncasecmp(s_arg, "CHAN", 4) == 0)
        {
            digits = s_arg + 4;
        }
        else
        {
            return E_DATA_RANGE;
        }
        unsigned n = 1;
        if (*digits != '\0')
        {
            char *end;
            n = (unsigned)strtoul(digits, &end, 10);
            if (*end != '\0')
            {
                return E_DATA_RANGE;
            }
        }
        if ((e = channel(n, &c)) != 0)
        {
            return e;
        }
        g_set.trig_src = (uint8_t)(n - 1U);
        clamp_trigger_level();
        scope_settings_changed();
        return 0;
    }
    if (match("TRIGger:EDGE", NULL))
    {
        if (q)
        {
            reply("%s", edge_name(g_set.trig_edge));
            return 0;
        }
        if ((e = need_arg()) != 0)
        {
            return e;
        }
        if (arg_is(s_arg, "RIS") || arg_is(s_arg, "RISING"))
        {
            g_set.trig_edge = EDGE_RIS;
        }
        else if (arg_is(s_arg, "FALL") || arg_is(s_arg, "FALLING"))
        {
            g_set.trig_edge = EDGE_FALL;
        }
        else if (arg_is(s_arg, "ANY"))
        {
            g_set.trig_edge = EDGE_ANY;
        }
        else
        {
            return E_DATA_RANGE;
        }
        scope_settings_changed();
        return 0;
    }
    if (match("TRIGger:LEVel", NULL))
    {
        if (q)
        {
            reply_real(g_set.trig_level);
            return 0;
        }
        float v;
        if ((e = parse_real(&v)) != 0)
        {
            return e;
        }
        g_set.trig_level = v;
        clamp_trigger_level();
        scope_settings_changed();
        return 0;
    }
    if (match("TRIGger:POSition", NULL))
    {
        if (q)
        {
            reply_real((float)scope_pretrig() / (float)scope_effective_depth());
            return 0;
        }
        float v;
        if ((e = parse_real(&v)) != 0)
        {
            return e;
        }
        if (v < 0.0f || v > 1.0f)
        {
            return E_DATA_RANGE;
        }
        g_set.trig_pos = v;
        scope_settings_changed();
        return 0;
    }
    if (match("TRIGger:HYSTeresis", NULL))
    {
        if (q)
        {
            reply_real(scope_hyst());
            return 0;
        }
        float v;
        if ((e = parse_real(&v)) != 0)
        {
            return e;
        }
        if (v < 0.0f)
        {
            return E_DATA_RANGE;
        }
        g_set.trig_hyst = v;
        scope_settings_changed();
        return 0;
    }
    if (!q && match("TRIGger:FORCe", NULL))
    {
        if ((e = need_noarg()) != 0)
        {
            return e;
        }
        scope_force();
        return 0;
    }

    /* --- run control --- */
    if (!q && match("RUN", NULL))
    {
        if ((e = need_noarg()) != 0)
        {
            return e;
        }
        scope_run(true);
        return 0;
    }
    if (!q && match("SINGle", NULL))
    {
        if ((e = need_noarg()) != 0)
        {
            return e;
        }
        scope_run(false);
        return 0;
    }
    if (!q && match("STOP", NULL))
    {
        if ((e = need_noarg()) != 0)
        {
            return e;
        }
        scope_stop();
        return 0;
    }
    if (q && match("STATus", NULL))
    {
        scope_poll();
        scope_state_t st = scope_state();
        reply(st == ST_STOP ? "STOP" : st == ST_ARMED ? "ARMED" : "TRIGGERED");
        return 0;
    }

    /* --- development diagnostics (not part of PROTOCOL.md) --- */
    if (q && match("DIAGnostic:RATE", NULL))
    {
        bool overrun;
        uint32_t r = scope_diag_measured_rate(&overrun);
        reply("%lu,%d", (unsigned long)r, overrun ? 1 : 0);
        return 0;
    }

    if (match("DIAGnostic:TESTsignal", NULL))
    {
        if (q)
        {
            reply("%d", adc_capture_test_signal_on() ? 1 : 0);
            return 0;
        }
        bool v;
        if ((e = parse_bool(&v)) != 0)
        {
            return e;
        }
        adc_capture_test_signal(v);
        return 0;
    }

    /* --- waveform --- */
    if (q && match("WAVeform:DATA", NULL))
    {
        scope_poll();
        *streamed = true;
        return scope_send_record() ? 0 : E_SETTINGS_CONFLICT;
    }

    return E_UNDEFINED_HEADER;
}

static void execute(char *line)
{
    /* trim */
    while (*line == ' ' || *line == '\t')
    {
        line++;
    }
    size_t len = strlen(line);
    while (len > 0U && (line[len - 1U] == ' ' || line[len - 1U] == '\t' || line[len - 1U] == '\r'))
    {
        line[--len] = '\0';
    }
    if (len == 0U)
    {
        return;
    }

    /* header and argument */
    char *header = line;
    char *arg = line;
    while (*arg != '\0' && *arg != ' ' && *arg != '\t')
    {
        arg++;
    }
    if (*arg != '\0')
    {
        *arg++ = '\0';
        while (*arg == ' ' || *arg == '\t')
        {
            arg++;
        }
    }
    s_arg = arg;

    size_t hlen = strlen(header);
    s_query = (hlen > 0U && header[hlen - 1U] == '?');
    if (s_query)
    {
        header[--hlen] = '\0';
    }
    if (*header == ':')
    {
        header++;
    }
    for (char *p = header; *p != '\0'; p++)
    {
        *p = (char)toupper((unsigned char)*p);
    }

    /* split into nodes */
    int err = 0;
    s_numNodes = 0;
    char *p = header;
    while (err == 0)
    {
        char *colon = strchr(p, ':');
        if (colon != NULL)
        {
            *colon = '\0';
        }
        if (*p == '\0' || s_numNodes >= MAX_NODES)
        {
            err = E_SYNTAX;
            break;
        }
        s_nodes[s_numNodes++] = p;
        if (colon == NULL)
        {
            break;
        }
        p = colon + 1;
    }

    bool streamed = false;
    s_resp[0] = '\0';
    if (err == 0)
    {
        err = dispatch(&streamed);
    }
    if (err != 0)
    {
        err_push(err);
    }
    if (s_query)
    {
        if (!streamed)
        {
            if (err != 0)
            {
                s_resp[0] = '\0'; /* a failed query still gets exactly one (empty) reply */
            }
            cdc_write(s_resp, strlen(s_resp));
            cdc_write("\n", 1);
        }
        cdc_flush();
    }
}

/* ---- line assembly ---- */

static char s_line[MAX_LINE];
static size_t s_lineLen;
static bool s_overflow;

void scpi_init(void)
{
    s_errCount = 0;
    s_lineLen = 0;
    s_overflow = false;
}

void scpi_input(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        char ch = (char)data[i];
        if (ch == '\n')
        {
            if (s_overflow)
            {
                err_push(E_TOO_MUCH_DATA); /* the whole line is discarded */
            }
            else
            {
                s_line[s_lineLen] = '\0';
                execute(s_line);
            }
            s_lineLen = 0;
            s_overflow = false;
        }
        else if (s_overflow)
        {
            /* keep discarding until the end of the line */
        }
        else if (s_lineLen < MAX_LINE - 1U) /* leave room for the terminator */
        {
            s_line[s_lineLen++] = ch;
        }
        else
        {
            s_overflow = true;
        }
    }
}
