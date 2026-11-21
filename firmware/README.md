# MCXScope firmware (FRDM-MCXA156)

Implements [`PROTOCOL.md`](../PROTOCOL.md): SCPI over USB CDC-ACM on the MCU's own USB
port (not the MCU-Link debug port). The board enumerates as `MCXScope`, normally
`/dev/ttyACM1` next to the debug probe's `/dev/ttyACM0`; the stable path is
`/dev/serial/by-id/usb-NXP_SEMICONDUCTORS_MCXScope-if00`.

## Inputs

| Channel | Header pin | MCU pin | ADC |
|---|---|---|---|
| CH1 | J4 pin 2 | P1_10 | ADC1 A8 |
| CH2 | J2 pin 7 | P2_1 | ADC0 A1 |
| Ground | any GND pin | | |

**Input range is 0 to 3.3 V, straight into the MCU.** There is no front end: negative
voltages or anything above 3.3 V can damage the chip.

Range (0.1 to 3.3 V) and offset (±10 V) only move the window ngscopeclient draws, so you
can pan and zoom; the ADC always converts the full 0..3.3 V and records carry absolute
volts. Default: 3.3 V range centred at 1.65 V, i.e. the whole ADC span, trigger at 1.65 V.
Zooming in doesn't add resolution (one ADC step is 0.8 mV), and anything outside 0..3.3 V
shows as a flat clipped line however you pan.

## Specs

- 2 channels sampled simultaneously (one ADC each, same timer edge), 12-bit
- 1 k, 10 k, 100 k, 1 M, 2 MS/s. 2 MS/s is the ADC's conversion limit at its 48 MHz clock
- Record length 1000 to 20000 samples per channel, or 30000 when only one channel is captured
  (the trigger source counts as captured even if hidden)
- Rising / falling / any edge trigger with hysteresis, sub-sample trigger position, AUTO and NORMAL
- Readout about 800 kB/s (an 80 KB two-channel 20k record takes 0.1 s)
- Lower sample rates use a longer ADC sampling window automatically, so high-impedance
  sources read accurately (at 2 MS/s the window is 62 ns; keep the source impedance low)

## How capture works

CTIMER0 triggers both ADCs; each ADC's result FIFO is drained by its own DMA channel into
a circular buffer, continuously, with no CPU involvement. The main loop scans new samples
of the trigger channel for an edge (starting after the pre-trigger part has been
captured); a 10 kHz tick stops the sample clock once the post-trigger samples are in. The
record is sent straight from the circular buffers.

## Build and flash

Needs the MCUXpresso SDK in `../sdks/frdm_mcxa156_blank_sdk`, Arm GNU toolchain 14.2 in
`~/.mcuxpressotools`, CMake, Ninja and LinkServer.

```sh
cmake --preset debug
cmake --build build
LinkServer flash MCXA156 load build/mcxscope.elf
```

The debug preset is built with `-O2 -g`: the software trigger search must keep up with
2 MS/s, which unoptimised code can't.

## Test

```sh
python3 ../host/tools/conformance.py /dev/serial/by-id/usb-NXP_SEMICONDUCTORS_MCXScope-if00
```

`--signal` and the emulator-specific checks in `host/scopehal/test/driver_test` assume a
bipolar ±1 V signal and fail on this 0..3.3 V input by design.

### Built-in test signal (no wiring)

`:DIAGnostic:TESTsignal 1` toggles each input pin's internal pull-up/pull-down from the
10 kHz tick, giving a 0..3.3 V square wave on unconnected inputs: 1 kHz on CH1, 500 Hz on
CH2. **Turn it off (`:DIAG:TEST 0`, or power-cycle) before connecting a real signal**: the
~50 kOhm pulls load it. It stays on across `*RST`, so you can enable it and then connect
ngscopeclient.

`:DIAGnostic:RATE?` runs the sample clock for 200 ms and returns `<measured S/s>,<overrun>`.

Neither command is part of PROTOCOL.md.

## Layout

| File | Role |
|---|---|
| `src/config.h` | Capabilities the host sees: rates, depths, range, channel count, `*IDN?` |
| `src/adc_capture.c` | Pins, ADCs, CTIMER, INPUTMUX, DMA rings, stop-at-sample, test signal |
| `src/scope.c` | Settings, state machine, trigger search, record and `:WAV:DATA?` block |
| `src/scpi.c` | Line assembly, header matching, error queue, every command |
| `src/usb_cdc.c` | CDC-ACM glue from the SDK example: no DTR gate, NAK-on-busy receive, stall recovery when the host stops reading |
| `src/timebase.c` | SysTick at 10 kHz: ms/us clock and the capture tick |

USB VID/PID are NXP's example IDs (`1FC9:0094`), fine for development only.
