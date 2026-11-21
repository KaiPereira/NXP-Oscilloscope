#!/usr/bin/env bash
# Adds the MCXScope driver to an ngscopeclient (or bare scopehal) source checkout.
#
#   ./integrate.sh ~/src/ngscopeclient        # ngscopeclient checkout (scopehal lives in lib/)
#   ./integrate.sh ~/src/scopehal             # bare scopehal checkout
#
# Safe to run more than once. Touches only files inside the given checkout.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="${1:?usage: $0 <ngscopeclient-or-scopehal-checkout>}"

if   [[ -f "$root/lib/scopehal/scopehal.cpp" ]]; then dir="$root/lib/scopehal"
elif [[ -f "$root/scopehal/scopehal.cpp"     ]]; then dir="$root/scopehal"
else echo "error: can't find scopehal/scopehal.cpp under $root (did you clone with --recursive?)" >&2; exit 1
fi

cp "$here/MCXScopeOscilloscope.h" "$here/MCXScopeOscilloscope.cpp" "$dir/"
echo "copied driver into $dir"

# CMakeLists.txt: add the source next to RohdeSchwarzOscilloscope.cpp
if ! grep -q 'MCXScopeOscilloscope.cpp' "$dir/CMakeLists.txt"; then
	sed -i 's/^\(\s*\)RohdeSchwarzOscilloscope\.cpp$/&\n\1MCXScopeOscilloscope.cpp/' "$dir/CMakeLists.txt"
	grep -q 'MCXScopeOscilloscope.cpp' "$dir/CMakeLists.txt" || { echo "error: failed to patch CMakeLists.txt" >&2; exit 1; }
	echo "patched CMakeLists.txt"
fi

# scopehal.cpp: include + registration
if ! grep -q 'MCXScopeOscilloscope.h' "$dir/scopehal.cpp"; then
	sed -i 's/^#include "RohdeSchwarzOscilloscope.h"$/&\n#include "MCXScopeOscilloscope.h"/' "$dir/scopehal.cpp"
	grep -q 'MCXScopeOscilloscope.h' "$dir/scopehal.cpp" || { echo "error: failed to add #include to scopehal.cpp" >&2; exit 1; }
fi
if ! grep -q 'AddDriverClass(MCXScopeOscilloscope)' "$dir/scopehal.cpp"; then
	sed -i 's/^\(\s*\)AddDriverClass(RohdeSchwarzOscilloscope);$/&\n\1AddDriverClass(MCXScopeOscilloscope);/' "$dir/scopehal.cpp"
	grep -q 'AddDriverClass(MCXScopeOscilloscope)' "$dir/scopehal.cpp" || { echo "error: failed to register driver in scopehal.cpp" >&2; exit 1; }
	echo "patched scopehal.cpp"
fi

echo "done. Rebuild ngscopeclient; the driver is listed as \"mcxscope\"."
