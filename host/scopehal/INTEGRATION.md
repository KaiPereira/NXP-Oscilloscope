# Adding the MCXScope driver to ngscopeclient

The driver is two files (`MCXScopeOscilloscope.h/.cpp`) written against current libscopehal
(`ngscopeclient/scopehal` main). ngscopeclient has no out-of-tree driver mechanism, so the files are
compiled into its copy of libscopehal.

## 1. Get ngscopeclient source and build dependencies

The ngscopeclient GUI lives in the `scopehal-apps` repository. Clone it with full history,
because its CMake reads the version from git tags and fails on a shallow clone without tags:

```sh
git clone --recursive https://github.com/ngscopeclient/scopehal-apps.git ngscopeclient
```

Fedora 44 packages (on top of `gcc-c++ cmake make git`), as used for the tested build:

```sh
sudo dnf install zlib-devel libpng-devel yaml-cpp-devel libsigc++30-devel gtk3-devel glfw-devel \
    spirv-tools-devel glslang-devel libshaderc-devel glslc vulkan-headers vulkan-loader-devel hidapi-devel
```

For other distros, see the build instructions in the ngscopeclient manual (linked from <https://www.ngscopeclient.org/>).

## 2. Patch in the driver

```sh
host/scopehal/integrate.sh path/to/ngscopeclient
```

This copies the two files into `lib/scopehal/`, adds the `.cpp` to `lib/scopehal/CMakeLists.txt`,
and adds `#include` plus `AddDriverClass(MCXScopeOscilloscope);` to `lib/scopehal/scopehal.cpp`.
It is idempotent. Run it again after `git pull` or after editing the driver here.

## 3. Build

```sh
cd path/to/ngscopeclient && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
make -j$(nproc)
./src/ngscopeclient/ngscopeclient
```

## 4. Connect

*Add → Oscilloscope → Connect…*, then:

| Field | Real hardware | Emulator (`fake_scope.py --tcp 5025`) | Emulator (`--pty`) |
|---|---|---|---|
| Driver | `mcxscope` | `mcxscope` | `mcxscope` |
| Transport | `uart` | `lan` | `uart` |
| Path | `/dev/ttyACM0` | `127.0.0.1:5025` | the `/dev/pts/N` it prints |

On Linux your user needs access to `/dev/ttyACM*`, normally via the `dialout` group.

You can also connect from the command line (`name:driver:transport:args`):

```sh
./src/ngscopeclient/ngscopeclient mcx:mcxscope:lan:127.0.0.1:5025
```

ngscopeclient connects and reads the settings but doesn't start acquiring until you press Start (or Single).

## 5. Headless driver test

`test/driver_test.cpp` links libscopehal directly and checks the driver against the emulator
without the GUI. It covers settings round-trips, trigger push and pull, single capture, RUN mode,
forced triggers, and whether the trigger crossing lands on the trigger marker to within 0.1 sample.

```sh
host/scopehal/test/build.sh path/to/ngscopeclient     # after building ngscopeclient
python3 host/tools/fake_scope.py --tcp 5025 &
host/scopehal/test/driver_test                         # expect "32 passed, 0 failed"
```

## Behaviour notes

- **Offset sign:** scopehal offsets are added to the signal, while the protocol's `OFFSet` is the
  voltage at the centre of the screen. The driver negates between the two.
- **Attenuation and bandwidth limit** are fixed at 1x and none. The device reports probe-tip
  volts and owns the front-end scaling.
- **Trigger** supports edge triggers only (rising, falling, any). The driver sets the device to
  `AUTO` trigger mode on connect so a trace appears with no signal. Change
  `kDefaultTriggerMode` in the `.cpp` to `"NORM"` for strict normal mode. *Force trigger*
  arms a single capture if needed, then forces.
- **Trigger position:** ngscopeclient's trigger offset (fs from record start) maps to
  `:TRIG:POS` as a fraction of the record. After each capture the driver sets the offset to
  exactly `trigger_index × period` and shifts the waveform by `trigger_frac` samples
  (`m_triggerPhase`), so the interpolated crossing lands on the trigger marker.
- **Memory limit:** enabling a channel re-reads `:ACQ:DEPT?`, because the device may shrink
  the depth to fit `MAXPoints`.
- **Errors:** the device error queue is drained into the ngscopeclient log on connect and
  whenever a readout comes back empty.
