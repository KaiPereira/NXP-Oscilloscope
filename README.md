# MCXScope: host side

A headless oscilloscope built on an NXP MCXA156, with [ngscopeclient](https://www.ngscopeclient.org/) as the GUI.
This repository holds the **protocol spec** and the **computer-side code**. The MCU firmware
is developed separately against `PROTOCOL.md`.

```
PROTOCOL.md                  wire protocol v1 (SCPI over USB CDC / TCP 5025): the firmware contract
host/scopehal/               ngscopeclient driver "mcxscope", integrate.sh, INTEGRATION.md, test/ (headless driver test)
host/tools/fake_scope.py     device emulator (reference implementation of PROTOCOL.md); needs numpy
host/tools/conformance.py    protocol test suite for the emulator or real hardware; pyserial for serial ports
```

## Quick start (no hardware)

```sh
pip install -r host/tools/requirements.txt
python3 host/tools/fake_scope.py --tcp 5025 &
python3 host/tools/conformance.py tcp:127.0.0.1:5025 --signal     # expect "69 passed, 0 failed"
```

Then build ngscopeclient with the driver (`host/scopehal/INTEGRATION.md`) and connect with driver
`mcxscope`, transport `lan`, path `127.0.0.1:5025`.

## For the firmware author

Implement `PROTOCOL.md`, then run:

```sh
python3 host/tools/conformance.py /dev/ttyACM0                 # protocol checks
python3 host/tools/conformance.py /dev/ttyACM0 --signal        # + trigger accuracy (1 kHz ~1 Vpp sine on CHAN1)
```

`fake_scope.py` is a working reference for every command, including trigger search, AUTO
timeout, RUN/SINGLE re-arm and record framing. Its `-v` flag logs the traffic.

Why not PicoLog? PicoLog 6 checks a SHA-256 hash of each vendor driver it loads, so a custom
driver can't be dropped in without patching the app. It is also a slow data logger with no
triggering, not a scope UI.

## License

BSD 3-Clause, see `LICENSE`. This matches libscopehal, so the driver can be contributed upstream.
