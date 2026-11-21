# MCXScope Host Protocol, version 1

This is the contract between the MCXScope device firmware (NXP MCXA156) and host software
(the `mcxscope` ngscopeclient driver and the tools in `host/tools/`). A firmware that does
everything in this document, and passes `host/tools/conformance.py`, will work with the
host side without any further coordination.

The key words MUST, SHOULD and MAY are used as in RFC 2119.

---

## 1. Transport

| Transport | Details |
|---|---|
| USB CDC-ACM (primary) | Enumerates as a virtual serial port (`/dev/ttyACM*`, `COMx`). Baud rate, parity and stop bits set by the host MUST be ignored. No hardware flow control. The device MUST NOT wait for DTR/RTS (`SET_CONTROL_LINE_STATE`) before replying: ngscopeclient's serial transport does not assert DTR by default. The MCUXpresso `usb_device_cdc_vcom` example only starts transferring after DTR is set (its `startTransactions` flag), so drop that gate. `conformance.py` keeps DTR deasserted to catch this. |
| TCP (optional) | Port **5025**, raw socket, byte-for-byte identical to the CDC stream. |

- The device MUST NOT drop or reorder bytes in either direction. If its receive buffer is
  full it MUST NAK USB OUT packets (normal CDC behaviour) instead of discarding data.
- The device MUST NOT send anything that was not requested: no banners, debug prints or
  unsolicited status.
- The host talks to one device per connection and never pipelines more than one query
  without reading its reply. It MAY send several *commands* (non-queries) back to back.

## 2. Message syntax

- Host → device: ASCII lines terminated by `\n` (LF). The device MUST also accept `\r\n`
  and ignore the `\r`. Maximum line length is **128 bytes** including the terminator. A longer line MUST
  be discarded and error `-223` queued.
- Headers are SCPI style: case-insensitive, with a short form (the uppercase part in this
  document) and a long form. For example, `:CHAN1:ENAB 1`, `:chan1:enable 1` and
  `:CHANnel1:ENABle 1` are all equivalent. The leading `:` is optional.
- Numeric suffixes: `CHANnel<n>` takes the channel number directly (`CHAN2`). If the suffix is omitted, it defaults to 1 (standard SCPI).
- One command per line. The device does not need to support `;`-separated compound commands.
- A header and its argument are separated by one or more spaces.
- **Query** = header ending in `?`. Every query produces **exactly one** response: either a
  single ASCII line terminated by `\n`, or a binary block (§6). Even a failed query MUST produce a
  response (an empty line `\n` for text queries, `#10\n` for `:WAVeform:DATA?`) and
  queue an error.
- **Commands** (no `?`) produce **no response**, success or failure. Failures queue an error.

### 2.1 Argument formats

| Type | Host sends | Device MUST accept |
|---|---|---|
| `<bool>` | `0` / `1` | `0`, `1`, `OFF`, `ON` |
| `<int>` | decimal integer, e.g. `1000000` | decimal integers up to 2^32−1 |
| `<real>` | e.g. `0.5`, `-1.250000e-01` | optional sign, decimal point, **and exponent notation** (`e`/`E`) |
| `<enum>` | long form, e.g. `RISing` → `RISING` | short or long form, any case |

Device responses use: `<bool>` → `0`/`1`; `<int>` → decimal; `<real>` → `%g`-style
with at least 6 significant digits (e.g. `0.125`, `-2.5e-03`); `<enum>` → the **short
form in uppercase** (e.g. `RIS`). Lists are comma-separated with no spaces.

### 2.2 Errors

The device keeps a FIFO error queue of at least 8 entries. If it overflows, the newest
entry MUST be replaced with `-350,"Queue overflow"`.

| Code | Message | When |
|---|---|---|
| 0 | `No error` | queue empty |
| −102 | `Syntax error` | malformed line |
| −108 | `Parameter not allowed` | argument given to a header that takes none |
| −109 | `Missing parameter` | |
| −113 | `Undefined header` | unknown command |
| −114 | `Header suffix out of range` | e.g. `CHAN9` on a 2-channel unit |
| −222 | `Data out of range` | value outside the supported range and not coercible |
| −221 | `Settings conflict` | e.g. `:WAV:DATA?` with no capture ready, or unsupported coupling |
| −223 | `Too much data` | line longer than 128 bytes |
| −350 | `Queue overflow` | |

`:SYSTem:ERRor?` → `<code>,"<message>"`, e.g. `-113,"Undefined header"`. It pops one
entry, and returns `0,"No error"` when empty.

## 3. Identity and capabilities

| Header | Response / effect |
|---|---|
| `*IDN?` | `<vendor>,<model>,<serial>,<firmware>`. The model MUST start with `MCXScope`. Example: `OpenHW,MCXScope-2CH,A1B2C3D4,1.0.0` |
| `*RST` | Restore the defaults in §8. Stops acquisition and clears any captured record. Does not clear the error queue. |
| `*CLS` | Clear the error queue. |
| `*OPC?` | Returns `1` once every previous command has taken effect. The host uses it as a barrier. |
| `:SYSTem:PROTocol?` | Protocol version as an int: `1`. |
| `:SYSTem:CHANnels?` | Number of analog channels N (1…16). Channels are numbered 1…N. |
| `:SYSTem:BITS?` | ADC resolution in bits (informational, e.g. `12` or `16`). |
| `:ACQuire:RATE:LIST?` | Supported sample rates in Hz, ascending, e.g. `1000,10000,100000,1000000,4000000` |
| `:ACQuire:DEPTh:LIST?` | Supported record lengths in samples per channel, ascending, e.g. `1000,2000,5000,10000,20000` |
| `:ACQuire:MAXPoints?` | Total sample memory across all enabled channels (see §5). |
| `:CHANnel<n>:RANGe:LIST?` | Supported full-scale ranges for channel n in volts, ascending, e.g. `0.5,2,10`. A unit with no gain switching returns a single value. |
| `:CHANnel<n>:COUPling:LIST?` | Supported couplings, e.g. `DC` or `DC,AC`. |

## 4. Channel settings

`<n>` is 1…N. Every setter has a matching query (`:CHAN1:RANG?` etc.) that returns the
**actual** value in effect.

| Header | Argument | Notes |
|---|---|---|
| `:CHANnel<n>:ENABle` | `<bool>` | Disabled channels are not captured or sent. |
| `:CHANnel<n>:RANGe` | `<real>` V | Full-scale span (max − min). The device picks the **smallest supported range ≥ the request**, or the largest if none is. |
| `:CHANnel<n>:OFFSet` | `<real>` V | Voltage at the **centre** of the range. The channel then covers `[OFFS − RANG/2, OFFS + RANG/2]`. A device without an offset DAC MUST accept the command and either keep its fixed value (normally 0 for a bipolar front end, or RANG/2 for a 0…Vref input) and return it from the query, or treat range and offset as a display window only (see below). Otherwise it clamps to the hardware limit. |
| `:CHANnel<n>:COUPling` | `DC` \| `AC` | Unsupported value → error −221, setting unchanged. |

**Display-window ranges.** A device with no gain or offset hardware MAY accept any offset
(within a limit it chooses) and list extra ranges that only define the window the host
draws and clamp the trigger level. The ADC still converts its full input span, samples
are scaled in absolute volts exactly as in §8.2, and `CLIPPED` refers to the ADC limit,
not the window edge. This lets the host pan and zoom; it does not add resolution.

All voltages are **at the probe tip / BNC**: the device applies its own front-end gain,
attenuation and calibration. The host never needs to know about the analog front end.

## 5. Acquisition settings

| Header | Argument | Notes |
|---|---|---|
| `:ACQuire:RATE` | `<int>` Hz | The device selects the **nearest** supported rate (ties go to the lower rate). |
| `:ACQuire:DEPTh` | `<int>` samples | The device selects the **nearest** supported depth (ties go to the lower depth). |

**Memory limit.** If `DEPTh × (number of enabled channels) > MAXPoints`, the device MUST
reduce the effective depth to the largest supported depth that fits. The `:ACQ:DEPT?`
query always returns the depth that the next capture will actually use. The host re-reads it after
enabling channels.

## 6. Trigger

| Header | Argument | Notes |
|---|---|---|
| `:TRIGger:MODE` | `AUTO` \| `NORMal` | AUTO: if no trigger occurs within the auto timeout, capture anyway (flag `FORCED`). Timeout = max(100 ms, 2 × record duration). NORMal: wait forever. |
| `:TRIGger:SOURce` | `CHAN<n>` | Query returns e.g. `CHAN1`. |
| `:TRIGger:EDGE` | `RISing` \| `FALLing` \| `ANY` | |
| `:TRIGger:LEVel` | `<real>` V | In probe-tip volts like everything else. Clamped to the source channel's range. |
| `:TRIGger:POSition` | `<real>` 0.0…1.0 | Fraction of the record **before** the trigger (0 = trigger at the first sample, 0.5 = centred). The device quantises to whole samples. The query returns the quantised value. |
| `:TRIGger:HYSTeresis` | `<real>` V | OPTIONAL. Noise rejection band. Default is device-chosen (suggested: 2 % of range). |
| `:TRIGger:FORCe` | — | If ARMED, trigger immediately (the record is flagged `FORCED`). Otherwise ignored. |

**Edge definition.** A rising edge occurs between samples k−1 and k when
`v[k−1] < LEVel − HYST/2` has been seen since the last trigger and `v[k] ≥ LEVel`. Falling is the mirror image.
The device MUST collect the full pre-trigger portion before accepting a trigger, so that
the pre-trigger data is real data.

## 7. Run control and status

State machine:

```
             :RUN / :SINGle                trigger + record complete
   STOP ───────────────────────► ARMED ─────────────────────────────► TRIGGERED
    ▲  ◄──────── :STOP ────────────┘                                     │
    │                                                                    │ full :WAV:DATA? readout
    │◄────────────── (SINGLE mode) ──────────────────────────────────────┤
                                     ARMED ◄──── (RUN mode) ─────────────┘
```

| Header | Effect |
|---|---|
| `:RUN` | Arm in continuous mode. After each complete `:WAV:DATA?` readout the device re-arms **by itself**. |
| `:SINGle` | Arm for one capture. After readout the device goes to STOP. |
| `:STOP` | Abort. Goes to STOP. A record that is already TRIGGERED stays readable until the next `:RUN`/`:SINGle`/`*RST`. |
| `:STATus?` | `STOP`, `ARMED` or `TRIGGERED`. ARMED covers "filling pre-trigger", "waiting for trigger" and "capturing post-trigger". TRIGGERED means **a complete record is ready to read**. While a record is TRIGGERED and unread, no new capture starts. |

Changing any channel, acquisition or trigger setting while ARMED MUST take effect from
the next capture. The device MAY restart the current capture to apply it.

## 8. Waveform readout: `:WAVeform:DATA?`

Valid only in state TRIGGERED. Otherwise the response is the empty block `#10\n` and
error −221 is queued.

### 8.1 Framing (IEEE 488.2 definite-length block)

```
'#'  <d>  <len: d ASCII digits>  <payload: len bytes>  '\n'
```

`<d>` is a single digit 1–9 giving the number of digits in `<len>`. Example: a 40,048-byte
payload is sent as `#540048` followed by the payload and then `\n`.

### 8.2 Payload

All fields are **little-endian**, packed with no padding.

| Offset | Type | Field | Meaning |
|---:|---|---|---|
| 0 | u8[4] | `magic` | ASCII `M`,`C`,`X`,`W` (0x4D 0x43 0x58 0x57) |
| 4 | u16 | `version` | `1` |
| 6 | u16 | `header_len` | Total header bytes including the channel table = 40 + 8 × (enabled channels). The host MUST skip to this offset to find the samples, so later versions can append fields. |
| 8 | u16 | `channel_mask` | Bit (n−1) set = CHANn present. Channels appear in ascending order. |
| 10 | u8 | `bits` | ADC resolution (informational) |
| 11 | u8 | `flags` | bit0 `FORCED` (auto timeout or `:TRIG:FORC`, no real trigger); bit1 `OVERRUN` (DMA overrun or dropped samples, data suspect); bit2 `CLIPPED` (some sample hit the ADC limit); other bits 0 |
| 12 | u32 | `depth` | Samples per channel in this record |
| 16 | u64 | `sample_period_fs` | Sample period in femtoseconds (1 MS/s → 1,000,000,000) |
| 24 | u32 | `trigger_index` | Index of the first sample **at or after** the trigger crossing |
| 28 | f32 | `trigger_frac` | Sub-sample position of the crossing, in [0, 1): the crossing happened at `(trigger_index − trigger_frac) × period`. Linear interpolation between samples trigger_index−1 and trigger_index. 0 if unknown or FORCED. |
| 32 | u32 | `sequence` | Increments by 1 on every record captured since power-up (wraps). Lets the host detect missed or repeated records. |
| 36 | u32 | `reserved` | 0 |
| 40 | per enabled channel: f32 `volts_per_code`, f32 `offset_volts` | | Calibration for that channel and record |
| header_len | i16[depth] × channels | samples | Channel-major: all samples of the lowest enabled channel, then the next, and so on. |

`payload_len = header_len + 2 × depth × popcount(channel_mask)`

**Scaling:** `volts = code × volts_per_code + offset_volts`. Codes are signed 16-bit. A
12-bit unipolar ADC can just send its raw 0…4095 values with a suitable `offset_volts`, or
left-justify them. Either is fine as long as the scaling describes the result.

The readout counts as complete once the final `\n` has been queued for sending. The
device then re-arms (RUN) or stops (SINGLE).

### 8.3 Bandwidth note

USB Full-Speed CDC gives roughly 0.8–1 MB/s in practice. A 2-channel × 20k-sample record
is 80 kB, or about 0.1 s of transfer. This is why the protocol is block-based (capture to SRAM,
then send) rather than streaming.

## 9. Defaults (power-up and `*RST`)

| Setting | Default |
|---|---|
| Channels | CHAN1 enabled, all others disabled |
| Range | largest supported |
| Offset | device's natural centre |
| Coupling | DC |
| Rate | highest supported |
| Depth | largest supported depth ≤ 10,000 |
| Trigger | MODE AUTO, SOURce CHAN1, EDGE RIS, LEVel = CHAN1 offset, POSition 0.5 |
| State | STOP |

## 10. Example session

`>` = host to device, `<` = device to host.

```
> *IDN?
< OpenHW,MCXScope-2CH,A1B2C3D4,1.0.0
> :SYST:PROT?
< 1
> :SYST:CHAN?
< 2
> :ACQ:RATE:LIST?
< 1000,10000,100000,1000000,4000000
> :ACQ:DEPT:LIST?
< 1000,2000,5000,10000,20000
> :CHAN2:ENAB 1
> :CHAN1:RANG 3
> :CHAN1:RANG?
< 10
> :ACQ:RATE 1000000
> :ACQ:DEPT 10000
> :TRIG:SOUR CHAN1
> :TRIG:EDGE RISING
> :TRIG:LEV 1.250000e+00
> :TRIG:POS 0.5
> :SINGLE
> :STAT?
< ARMED
> :STAT?
< TRIGGERED
> :WAV:DATA?
< #540056<40056 bytes>\n          (header 56 = 40 + 2×8, then 2 × 10000 × 2 bytes)
> :STAT?
< STOP
> :SYST:ERR?
< 0,"No error"
```

## 11. Firmware conformance checklist

- [ ] Accepts `\n` and `\r\n` line endings, both short and long header forms, any case, with or without a leading `:`
- [ ] Parses reals in exponent notation (`1.250000e+00`)
- [ ] Commands never reply. Queries always reply exactly once, even on error
- [ ] Error queue with `:SYST:ERR?`, codes as in §2.2
- [ ] Capability queries (§3) reflect the real hardware
- [ ] Rate and depth coerce to the nearest supported value. Depth respects MAXPoints
- [ ] Range picks the smallest supported range ≥ the request
- [ ] All voltages (range, offset, trigger level, scaling) are probe-tip volts
- [ ] Pre-trigger data is real. `trigger_index ≈ POSition × depth` for real triggers
- [ ] AUTO mode delivers FORCED records when no trigger arrives
- [ ] RUN re-arms after readout. SINGLE goes to STOP after readout
- [ ] `:WAV:DATA?` when not TRIGGERED → `#10\n` + error −221
- [ ] Block framing and payload layout exactly as in §8. `sequence` increments
- [ ] No unsolicited output, ever

Run `python3 host/tools/conformance.py /dev/ttyACM0 --signal` with a 1 kHz, ~1 Vpp sine wave
on CHAN1 for the full check.
