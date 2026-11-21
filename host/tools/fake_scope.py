#!/usr/bin/env python3
"""
fake_scope.py: software emulator of an MCXScope device, implementing PROTOCOL.md (v1).

Use it to develop and test the host side (the ngscopeclient driver and conformance.py)
without hardware.

    python3 fake_scope.py --tcp 5025          # ngscopeclient: transport "lan", 127.0.0.1:5025
    python3 fake_scope.py --pty               # prints /dev/pts/N; transport "uart", path /dev/pts/N

Requires numpy.

Simulated signals (probe-tip volts):
    CHAN1  1 kHz sine, 1 Vpp, +0.1 V DC
    CHAN2  250 Hz square, +/-1 V, with 2 us edges
    CHAN3  2 kHz triangle, 2 Vpp
    CHAN4  10 kHz sine, 0.4 Vpp
plus a little Gaussian noise on every channel.
"""

import argparse
import math
import os
import re
import selectors
import socket
import struct
import sys
import time
import tty

import numpy as np

# --------------------------------------------------------------------------------------
# Device model


RATES = [1000, 10000, 100000, 1000000, 4000000]
DEPTHS = [1000, 2000, 5000, 10000, 20000]
MAX_POINTS = 40000
RANGES = [0.5, 2.0, 10.0]
COUPLINGS = ["DC", "AC"]
BITS = 12
OFFSET_LIMIT = 5.0
NOISE_V = 0.004

ERRORS = {
    -102: "Syntax error",
    -108: "Parameter not allowed",
    -109: "Missing parameter",
    -113: "Undefined header",
    -114: "Header suffix out of range",
    -221: "Settings conflict",
    -222: "Data out of range",
    -223: "Too much data",
    -350: "Queue overflow",
}

ENUMS = {
    "EDGE": [("RIS", "RISING"), ("FALL", "FALLING"), ("ANY", "ANY")],
    "MODE": [("AUTO", "AUTO"), ("NORM", "NORMAL")],
    "COUP": [("DC", "DC"), ("AC", "AC")],
}


def signal(ch, t, coupling):
    """Clean simulated signal for channel ch (1-based) at times t (numpy array, seconds)."""
    if ch == 1:
        v = 0.5 * np.sin(2 * np.pi * 1000 * t) + 0.1
        dc = 0.1
    elif ch == 2:
        # band-limited-ish square: tanh edges ~2 us
        v = np.tanh(np.sin(2 * np.pi * 250 * t) * 2000)
        dc = 0.0
    elif ch == 3:
        ph = (t * 2000) % 1.0
        v = 2 * np.abs(2 * ph - 1) - 1
        dc = 0.0
    else:
        v = 0.2 * np.sin(2 * np.pi * 10000 * t + ch)
        dc = 0.0
    if coupling == "AC":
        v = v - dc
    return v


def fmt_real(x):
    return "%.6g" % x


def nearest(options, value):
    # nearest, ties -> lower
    return min(options, key=lambda o: (abs(o - value), o))


class Device:
    def __init__(self, nchan, serial="FAKE0001", seed=None):
        self.n = nchan
        self.serial = serial
        self.rng = np.random.default_rng(seed)
        self.errors = []
        self.sequence = 0
        self.t0 = time.monotonic()
        self.reset()

    # ---- helpers ------------------------------------------------------------------

    def now(self):
        return time.monotonic() - self.t0

    def err(self, code):
        if len(self.errors) >= 8:
            self.errors[-1] = -350
        else:
            self.errors.append(code)

    def n_enabled(self):
        return sum(1 for c in self.ch if c["en"])

    def effective_depth(self):
        """Requested depth, reduced to the largest supported depth that fits MAXPoints."""
        n = max(1, self.n_enabled())
        fits = [d for d in DEPTHS if d * n <= MAX_POINTS] or [DEPTHS[0]]
        cands = [d for d in fits if d <= self.depth_req]
        return max(cands) if cands else fits[0]

    def pretrig(self):
        return int(round(self.trig_pos * self.effective_depth()))

    def reset(self):
        self.ch = [
            {"en": i == 0, "range": RANGES[-1], "offset": 0.0, "coup": "DC"}
            for i in range(self.n)
        ]
        self.rate = RATES[-1]
        self.depth_req = max(d for d in DEPTHS if d <= 10000)
        self.trig_mode = "AUTO"
        self.trig_src = 1
        self.trig_edge = "RIS"
        self.trig_level = 0.0
        self.trig_pos = 0.5
        self.trig_hyst = None  # None = 2% of range
        self.state = "STOP"
        self.run_mode = None  # "RUN" or "SINGLE"
        self.record = None
        self.force_pending = False

    # ---- acquisition simulation ----------------------------------------------------

    def arm(self, mode):
        self.run_mode = mode
        self.state = "ARMED"
        self.record = None
        self.force_pending = False
        self.arm_time = self.now()
        # sim time of the next sample examined for a trigger
        depth = self.effective_depth()
        # all sampling happens on one global clock grid t = k / rate (like a real ADC);
        # search_k is the index of the next sample to examine for a trigger
        self.search_k = math.ceil((self.arm_time + self.pretrig() / self.rate) * self.rate)
        self.cap_rate = self.rate
        self.cap_depth = depth
        self.hyst_armed = {}
        self.pending = None

    def hyst(self):
        if self.trig_hyst is not None:
            return self.trig_hyst
        return 0.02 * self.ch[self.trig_src - 1]["range"]

    def sample_channel(self, ch, t):
        c = self.ch[ch - 1]
        v = signal(ch, t, c["coup"]) + self.rng.normal(0, NOISE_V, size=t.shape)
        return v

    def find_trigger(self, t_end):
        """Search samples from index self.search_k up to time t_end for a trigger. Returns crossing time or None.

        A rising trigger fires on the first sample >= level after a sample < level - hyst/2
        has been seen (falling is mirrored). The armed state carries across calls in
        self.hyst_armed (a dict per edge kind).
        """
        rate = self.cap_rate
        dt = 1.0 / rate
        n = int(t_end * rate) - self.search_k
        if n <= 0:
            return None
        n = min(n, 2_000_000)
        t = (self.search_k + np.arange(n)) / rate
        v = self.sample_channel(self.trig_src, t)
        lvl, h = self.trig_level, self.hyst() / 2
        idx = np.arange(n)
        best = None
        kinds = []
        if self.trig_edge in ("RIS", "ANY"):
            kinds.append(("R", v < lvl - h, v >= lvl))
        if self.trig_edge in ("FALL", "ANY"):
            kinds.append(("F", v > lvl + h, v <= lvl))
        for kind, arm, fire in kinds:
            armed0 = self.hyst_armed.get(kind, False)
            last_arm = np.maximum.accumulate(np.where(arm, idx, -1 if armed0 else -3))
            last_fire = np.maximum.accumulate(np.where(fire, idx, -2))
            # fire at k if fire[k] and the last arm sample before k is newer than the
            # last fire sample before k
            prev_arm = np.concatenate(([-1 if armed0 else -3], last_arm[:-1]))
            prev_fire = np.concatenate(([-2], last_fire[:-1]))
            hits = np.nonzero(fire & (prev_arm > prev_fire))[0]
            if len(hits):
                k = int(hits[0])
                if best is None or k < best[0]:
                    best = (k, kind)
            else:
                self.hyst_armed[kind] = bool(last_arm[-1] > last_fire[-1])
        if best is None:
            self.search_k += n
            return None
        k, kind = best
        if k == 0:
            return t[0]
        v0, v1 = v[k - 1], v[k]
        f = (lvl - v0) / (v1 - v0) if v1 != v0 else 1.0
        return t[k - 1] + min(max(f, 0.0), 1.0) * dt

    def advance(self):
        if self.state != "ARMED":
            return
        now = self.now()
        rate, depth = self.cap_rate, self.cap_depth
        pre = int(round(self.trig_pos * depth))
        if self.pending is None:
            if self.force_pending:
                self.pending = (max(self.search_k / rate, now), True)
            else:
                t_trig = self.find_trigger(now)
                if t_trig is not None:
                    self.pending = (t_trig, False)
                elif self.trig_mode == "AUTO" and now - self.arm_time > max(0.1, 2 * depth / rate):
                    self.pending = (now, True)
        if self.pending is None:
            return
        t_trig, forced = self.pending
        # the record is complete only once the post-trigger part has "happened"
        if now < t_trig + (depth - pre) / rate:
            return
        self.make_record(t_trig, forced)

    def make_record(self, t_trig, forced):
        rate, depth = self.cap_rate, self.cap_depth
        pre = int(round(self.trig_pos * depth))
        dt = 1.0 / rate
        # first sample at/after trigger is index `pre`
        if forced:
            k_after = math.ceil(t_trig * rate)
            frac = 0.0
        else:
            k_after = math.ceil(t_trig * rate - 1e-6)
            frac = min(max(k_after - t_trig * rate, 0.0), 0.999999)
        t = (k_after - pre + np.arange(depth)) / rate
        chans = []
        clipped = False
        full = 2 ** (BITS - 1)
        for i, c in enumerate(self.ch):
            if not c["en"]:
                continue
            v = self.sample_channel(i + 1, t)
            vpc = c["range"] / (2 ** BITS)
            codes = np.round((v - c["offset"]) / vpc)
            if np.any(codes < -full) or np.any(codes > full - 1):
                clipped = True
            codes = np.clip(codes, -full, full - 1).astype("<i2")
            chans.append((i, vpc, c["offset"], codes))
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        flags = (1 if forced else 0) | (4 if clipped else 0)
        mask = 0
        for i, *_ in chans:
            mask |= 1 << i
        header_len = 40 + 8 * len(chans)
        hdr = struct.pack(
            "<4sHHHBBIQIfII",
            b"MCXW", 1, header_len, mask, BITS, flags, depth,
            int(round(1e15 / rate)), pre, float(frac), self.sequence, 0,
        )
        assert len(hdr) == 40
        cal = b"".join(struct.pack("<ff", vpc, off) for _, vpc, off, _ in chans)
        data = b"".join(c.tobytes() for *_, c in chans)
        self.record = hdr + cal + data
        self.state = "TRIGGERED"

    # ---- SCPI ----------------------------------------------------------------------

    def handle_line(self, raw):
        """Returns bytes to send (possibly b'') for one input line."""
        line = raw.strip()
        if not line:
            return b""
        if len(raw) > 128:
            self.err(-223)
            return b""
        m = re.match(r"^(\S+)(?:\s+(.*))?$", line)
        header, arg = m.group(1), (m.group(2) or "").strip()
        query = header.endswith("?")
        if query:
            header = header[:-1]
        try:
            resp = self.dispatch(header.upper().lstrip(":"), arg, query)
        except ScpiError as e:
            self.err(e.code)
            resp = None
            if query:
                resp = b"#10\n" if self._is_wav(header) else b"\n"
            return resp or b""
        if query:
            if isinstance(resp, bytes):
                return resp
            return (str(resp) + "\n").encode()
        return b""

    @staticmethod
    def _is_wav(header):
        h = header.upper().lstrip(":")
        return re.match(r"^WAV(EFORM)?:DATA$", h) is not None

    def dispatch(self, h, arg, q):
        nodes = h.split(":")

        def match(pattern):
            """pattern like 'CHANnel#:RANGe:LIST'. Returns list of suffixes or None."""
            pn = pattern.split(":")
            if len(pn) != len(nodes):
                return None
            sufs = []
            for p, n in zip(pn, nodes):
                want_suffix = p.endswith("#")
                p = p.rstrip("#")
                short = "".join(ch for ch in p if ch.isupper() or ch == "*" or ch.isdigit())
                long_ = p.upper()
                mm = re.match(r"^([A-Z*]+?)(\d*)$", n)
                if not mm:
                    return None
                word, digits = mm.group(1), mm.group(2)
                if word not in (short, long_):
                    return None
                if digits and not want_suffix:
                    return None
                if want_suffix:
                    sufs.append(int(digits) if digits else 1)
            return sufs

        def need_noarg():
            if arg:
                raise ScpiError(-108)

        def need_arg():
            if not arg:
                raise ScpiError(-109)
            return arg

        def chan(s):
            if not 1 <= s <= self.n:
                raise ScpiError(-114)
            return self.ch[s - 1]

        # --- common ---
        if match("*IDN") is not None and q:
            return "FakeVendor,MCXScope-%dCH,%s,0.1.0-fake" % (self.n, self.serial)
        if match("*RST") is not None and not q:
            self.reset()
            return None
        if match("*CLS") is not None and not q:
            self.errors.clear()
            return None
        if match("*OPC") is not None and q:
            return "1"
        if match("SYSTem:ERRor") is not None and q:
            if not self.errors:
                return '0,"No error"'
            c = self.errors.pop(0)
            return '%d,"%s"' % (c, ERRORS[c])
        if match("SYSTem:PROTocol") is not None and q:
            return "1"
        if match("SYSTem:CHANnels") is not None and q:
            return str(self.n)
        if match("SYSTem:BITS") is not None and q:
            return str(BITS)

        # --- acquisition ---
        if match("ACQuire:RATE:LIST") is not None and q:
            return ",".join(str(r) for r in RATES)
        if match("ACQuire:DEPTh:LIST") is not None and q:
            return ",".join(str(d) for d in DEPTHS)
        if match("ACQuire:MAXPoints") is not None and q:
            return str(MAX_POINTS)
        if match("ACQuire:RATE") is not None:
            if q:
                return str(self.rate)
            self.rate = nearest(RATES, parse_int(need_arg()))
            self.restart_if_armed()
            return None
        if match("ACQuire:DEPTh") is not None:
            if q:
                return str(self.effective_depth())
            self.depth_req = nearest(DEPTHS, parse_int(need_arg()))
            self.restart_if_armed()
            return None

        # --- channels ---
        s = match("CHANnel#:RANGe:LIST")
        if s is not None and q:
            chan(s[0])
            return ",".join(fmt_real(r) for r in RANGES)
        s = match("CHANnel#:COUPling:LIST")
        if s is not None and q:
            chan(s[0])
            return ",".join(COUPLINGS)
        s = match("CHANnel#:ENABle")
        if s is not None:
            c = chan(s[0])
            if q:
                return "1" if c["en"] else "0"
            c["en"] = parse_bool(need_arg())
            self.restart_if_armed()
            return None
        s = match("CHANnel#:RANGe")
        if s is not None:
            c = chan(s[0])
            if q:
                return fmt_real(c["range"])
            v = parse_real(need_arg())
            if v <= 0:
                raise ScpiError(-222)
            ok = [r for r in RANGES if r >= v - 1e-9]
            c["range"] = ok[0] if ok else RANGES[-1]
            self.restart_if_armed()
            return None
        s = match("CHANnel#:OFFSet")
        if s is not None:
            c = chan(s[0])
            if q:
                return fmt_real(c["offset"])
            v = parse_real(need_arg())
            c["offset"] = max(-OFFSET_LIMIT, min(OFFSET_LIMIT, v))
            self.restart_if_armed()
            return None
        s = match("CHANnel#:COUPling")
        if s is not None:
            c = chan(s[0])
            if q:
                return c["coup"]
            c["coup"] = parse_enum("COUP", need_arg())
            self.restart_if_armed()
            return None

        # --- trigger ---
        if match("TRIGger:MODE") is not None:
            if q:
                return self.trig_mode
            self.trig_mode = parse_enum("MODE", need_arg())
            return None
        if match("TRIGger:SOURce") is not None:
            if q:
                return "CHAN%d" % self.trig_src
            mm = re.match(r"^CHAN(?:NEL)?(\d*)$", need_arg().upper())
            if not mm:
                raise ScpiError(-222)
            n = int(mm.group(1) or 1)
            chan(n)
            self.trig_src = n
            self.restart_if_armed()
            return None
        if match("TRIGger:EDGE") is not None:
            if q:
                return self.trig_edge
            self.trig_edge = parse_enum("EDGE", need_arg())
            self.restart_if_armed()
            return None
        if match("TRIGger:LEVel") is not None:
            if q:
                return fmt_real(self.trig_level)
            v = parse_real(need_arg())
            c = self.ch[self.trig_src - 1]
            lo, hi = c["offset"] - c["range"] / 2, c["offset"] + c["range"] / 2
            self.trig_level = max(lo, min(hi, v))
            self.restart_if_armed()
            return None
        if match("TRIGger:POSition") is not None:
            if q:
                d = self.effective_depth()
                return fmt_real(self.pretrig() / d)
            v = parse_real(need_arg())
            if not 0.0 <= v <= 1.0:
                raise ScpiError(-222)
            self.trig_pos = v
            self.restart_if_armed()
            return None
        if match("TRIGger:HYSTeresis") is not None:
            if q:
                return fmt_real(self.hyst())
            v = parse_real(need_arg())
            if v < 0:
                raise ScpiError(-222)
            self.trig_hyst = v
            return None
        if match("TRIGger:FORCe") is not None and not q:
            need_noarg()
            if self.state == "ARMED":
                self.force_pending = True
                self.advance()
            return None

        # --- run control ---
        if match("RUN") is not None and not q:
            need_noarg()
            self.arm("RUN")
            return None
        if match("SINGle") is not None and not q:
            need_noarg()
            self.arm("SINGLE")
            return None
        if match("STOP") is not None and not q:
            need_noarg()
            if self.state == "ARMED":
                self.state = "STOP"
            elif self.state == "TRIGGERED":
                self.run_mode = "SINGLE"  # record stays readable; don't re-arm after it
            return None
        if match("STATus") is not None and q:
            self.advance()
            return self.state

        # --- waveform ---
        if match("WAVeform:DATA") is not None and q:
            self.advance()
            if self.state != "TRIGGERED" or self.record is None:
                raise ScpiError(-221)
            payload = self.record
            ln = str(len(payload))
            out = b"#" + str(len(ln)).encode() + ln.encode() + payload + b"\n"
            self.record = None
            if self.run_mode == "RUN":
                self.arm("RUN")
            else:
                self.state = "STOP"
            return out

        raise ScpiError(-113)

    def restart_if_armed(self):
        if self.state == "ARMED":
            self.arm(self.run_mode)


class ScpiError(Exception):
    def __init__(self, code):
        super().__init__(code)
        self.code = code


def parse_bool(s):
    s = s.upper()
    if s in ("1", "ON"):
        return True
    if s in ("0", "OFF"):
        return False
    raise ScpiError(-222)


def parse_int(s):
    try:
        return int(round(float(s)))
    except ValueError:
        raise ScpiError(-102)


def parse_real(s):
    try:
        v = float(s)
    except ValueError:
        raise ScpiError(-102)
    if not math.isfinite(v):
        raise ScpiError(-222)
    return v


def parse_enum(kind, s):
    s = s.upper()
    for short, long_ in ENUMS[kind]:
        if s in (short, long_):
            return short
    raise ScpiError(-222)


# --------------------------------------------------------------------------------------
# Transports


class LineBuffer:
    def __init__(self, dev, verbose):
        self.dev = dev
        self.buf = b""
        self.verbose = verbose

    def feed(self, data):
        self.buf += data
        out = b""
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            text = line.decode("ascii", "replace").rstrip("\r")
            resp = self.dev.handle_line(text)
            if self.verbose:
                shown = resp if len(resp) < 100 else resp[:40] + b"...(%d bytes)" % len(resp)
                print(">", text, "\n<", shown, file=sys.stderr)
            out += resp
        if len(self.buf) > 4096:
            self.buf = b""
            self.dev.err(-223)
        return out


def serve_tcp(dev, port, verbose):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(1)
    print("fake MCXScope listening on 127.0.0.1:%d" % port, flush=True)
    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print("client connected:", addr, flush=True)
        lb = LineBuffer(dev, verbose)
        try:
            while True:
                data = conn.recv(4096)
                if not data:
                    break
                out = lb.feed(data)
                if out:
                    conn.sendall(out)
        except (ConnectionResetError, BrokenPipeError):
            pass
        conn.close()
        print("client disconnected", flush=True)


def serve_pty(dev, verbose, link=None):
    master, slave = os.openpty()
    tty.setraw(slave)
    path = os.ttyname(slave)
    if link:
        try:
            os.unlink(link)
        except FileNotFoundError:
            pass
        os.symlink(path, link)
        print("fake MCXScope on %s (symlink %s)" % (path, link), flush=True)
    else:
        print("fake MCXScope on %s" % path, flush=True)
    # keep `slave` open so the pty survives client disconnects
    lb = LineBuffer(dev, verbose)
    while True:
        try:
            data = os.read(master, 4096)
        except OSError:
            time.sleep(0.05)
            continue
        out = lb.feed(data)
        while out:
            n = os.write(master, out)
            out = out[n:]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--tcp", type=int, metavar="PORT", help="listen on 127.0.0.1:PORT (use 5025)")
    g.add_argument("--pty", action="store_true", help="create a pseudo-terminal")
    ap.add_argument("--link", help="with --pty: also create this symlink to the pty")
    ap.add_argument("--channels", type=int, default=2, choices=range(1, 5))
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("-v", "--verbose", action="store_true", help="log traffic to stderr")
    a = ap.parse_args()
    dev = Device(a.channels, seed=a.seed)
    try:
        if a.tcp:
            serve_tcp(dev, a.tcp, a.verbose)
        else:
            serve_pty(dev, a.verbose, a.link)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
