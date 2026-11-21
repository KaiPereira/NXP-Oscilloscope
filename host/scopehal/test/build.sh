#!/usr/bin/env bash
# Builds driver_test against an ngscopeclient tree that has the driver integrated and is already built.
#   ./build.sh /path/to/ngscopeclient        -> produces ./driver_test next to this script
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ng="${1:?usage: $0 <ngscopeclient checkout with build/>}"
b="$ng/build"
g++ -std=c++20 -O1 -Wall -Wextra \
	-I"$b/lib/scopehal" -I"$ng/lib/scopehal" -I"$ng/lib" -I"$ng/lib/log" -I"$ng/lib/xptools" \
	$(pkg-config --cflags sigc++-3.0 yaml-cpp) \
	"$here/driver_test.cpp" -o "$here/driver_test" \
	-L"$b/lib/scopehal" -Wl,-rpath,"$b/lib/scopehal" -lscopehal \
	"$b/lib/xptools/libxptools.a" "$b/lib/log/liblog.a" \
	$(pkg-config --libs sigc++-3.0 yaml-cpp hidapi-hidraw 2>/dev/null || pkg-config --libs sigc++-3.0 yaml-cpp hidapi) \
	-lglslang -lglslang-default-resource-limits -lvulkan -lpthread -fopenmp
echo "built $here/driver_test"
