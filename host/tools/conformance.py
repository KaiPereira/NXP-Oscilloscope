#!/usr/bin/env python3
"""
conformance.py: checks that a device (real or fake) implements PROTOCOL.md v1.

    python3 conformance.py /dev/ttyACM0                 # real hardware over USB CDC
    python3 conformance.py tcp:127.0.0.1:5025          # fake_scope.py --tcp 5025
    python3 conformance.py /dev/ttyACM0 --signal        # also check trigger accuracy;
                                                        # needs a ~1 kHz, >=0.5 Vpp periodic
                                                        # signal centred near 0 V on CHAN1

Serial ports need pyserial (`pip install pyserial`); tcp: needs only the standard library.
Exit status is 0 if every check passes.
"""

import argparse
import socket
import struct
import sys
import time

# --------------------------------------------------------------------------------------
# Link


class Link:
    def __init__(self, target, timeout=3.0):
        self.timeout = timeout
        self.buf = b""
        if target.startswith("tcp:"):
            host, port = target[4:].rsplit(":", 1)
            self.sock = socket.create_connection((host, int(port)), timeout=timeout)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._rx = lambda n: self.sock.recv(n)
            self._tx = self.sock.sendall
        else:
            import serial  # pyserial

            # Keep DTR/RTS deasserted, as ngscopeclient's UART transport does by default
            self.ser = serial.Serial()
            self.ser.port = target
            self.ser.baudrate = 115200
            self.ser.timeout = 0.05
            self.ser.dtr = False
            self.ser.rts = False
            self.ser.open()
            self.ser.reset_input_buffer()
            self._rx = lambda n: self.ser.read(n)
            self._tx = self.ser.write

    def send(self, line):
        self._tx((line + "\n").encode())

    def _fill(self, deadline):
        while time.monotonic() < deadline:
            try:
                d = self._rx(65536)
            except socket.timeout:
                d = b""
            if d:
                self.buf += d
                return True
        return False

    def read_line(self):
        deadline = time.monotonic() + self.timeout
        while b"\n" not in self.buf:
            if not self._fill(deadline):
                raise TimeoutError("no reply line")
        line, self.buf = self.buf.split(b"\n", 1)
        return line.decode("ascii", "replace").rstrip("\r")

    def read_exact(self, n):
        deadline = time.monotonic() + self.timeout + n / 500e3
        while len(self.buf) < n:
            if not self._fill(deadline):
                raise TimeoutError("short binary read (%d of %d)" % (len(self.buf), n))
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def query(self, line):
        self.send(line)
        return self.read_line()

    def query_block(self, line):
        self.send(line)
        h = self.read_exact(2)
        if h[0:1] != b"#":
            raise ValueError("block does not start with '#': %r" % h)
        nd = int(h[1:2])
        ln = int(self.read_exact(nd)) if nd else 0
        payload = self.read_exact(ln)
        term = self.read_exact(1)
        if term != b"\n":
            raise ValueError("block not terminated by LF (got %r)" % term)
        return payload

    def drain(self, wait=0.2):
        """Return anything unsolicited that arrives within `wait` seconds."""
        deadline = time.monotonic() + wait
        while self._fill(deadline):
            pass
        out, self.buf = self.buf, b""
        return out


# --------------------------------------------------------------------------------------
# Record parsing (reference decoder for §8.2)

HDR = struct.Struct("<4sHHHBBIQIfII")


class Record:
    def __init__(self, payload):
        if len(payload) < HDR.size:
            raise ValueError("payload shorter than fixed header (%d)" % len(payload))
        (self.magic, self.version, self.header_len, self.mask, self.bits, self.flags,
         self.depth, self.period_fs, self.trigger_index, self.trigger_frac,
         self.sequence, self.reserved) = HDR.unpack_from(payload)
        self.channels = [i + 1 for i in range(16) if self.mask & (1 << i)]
        self.cal = {}
        for k, ch in enumerate(self.channels):
            self.cal[ch] = struct.unpack_from("<ff", payload, 40 + 8 * k)
        self.payload = payload

    def expected_len(self):
        return self.header_len + 2 * self.depth * len(self.channels)

    def volts(self, ch):
        k = self.channels.index(ch)
        start = self.header_len + 2 * self.depth * k
        vpc, off = self.cal[ch]
        n = self.depth
        codes = struct.unpack_from("<%dh" % n, self.payload, start)
        return [c * vpc + off for c in codes]


# --------------------------------------------------------------------------------------
# Checks


class Runner:
    def __init__(self, link):
        self.l = link
        self.passed = 0
        self.failed = []

    def check(self, name, cond, detail=""):
        if cond:
            self.passed += 1
            print("  PASS  " + name)
        else:
            self.failed.append(name)
            print("  FAIL  " + name + ("  -- " + detail if detail else ""))
        return cond

    def err(self):
        return self.l.query(":SYST:ERR?")

    def clear_errors(self):
        for _ in range(20):
            if self.err().startswith("0,"):
                return

    def capture(self, cmd=":SINGLE", timeout=5.0):
        self.l.send(cmd)
        deadline = time.monotonic() + timeout
        st = ""
        while time.monotonic() < deadline:
            st = self.l.query(":STAT?")
            if st == "TRIGGERED":
                return Record(self.l.query_block(":WAV:DATA?"))
            time.sleep(0.01)
        raise TimeoutError("no capture within %.1fs (last status %r)" % (timeout, st))


def floats(s):
    return [float(x) for x in s.split(",") if x]


def run(r, signal):
    l = r.l
    print("== Identity")
    l.send("*RST")
    l.send("*CLS")
    r.check("*OPC? returns 1", l.query("*OPC?") == "1")
    idn = l.query("*IDN?")
    parts = idn.split(",")
    r.check("*IDN? has 4 fields, model starts with MCXScope",
            len(parts) == 4 and parts[1].startswith("MCXScope"), idn)
    r.check("protocol version 1", l.query(":SYST:PROT?") == "1")
    n = int(l.query(":SYST:CHAN?"))
    r.check("channel count 1..16", 1 <= n <= 16, str(n))
    bits = int(l.query(":SYST:BITS?"))
    r.check("bits plausible", 8 <= bits <= 16, str(bits))
    r.check("no unsolicited output", l.drain() == b"")

    print("== Syntax")
    r.check("long form + lower case", l.query(":system:protocol?") == "1")
    r.check("no leading colon", l.query("SYST:PROT?") == "1")
    l._tx(b":SYST:PROT?\r\n")
    r.check("CRLF accepted", l.read_line() == "1")
    l.send(":NOPE:NOTHING")
    r.check("unknown command -> no reply", l.drain(0.2) == b"")
    r.check("unknown command -> -113", r.err().startswith("-113,"))
    r.check("queue empty -> 0,\"No error\"", r.err() == '0,"No error"')
    resp = l.query(":NOPE?")
    r.check("unknown query still replies (empty line)", resp == "", repr(resp))
    r.clear_errors()
    l.send(":CHAN%d:ENAB 1" % (n + 1))
    r.check("bad channel suffix -> -114", r.err().startswith("-114,"))
    l.send(":SYST:PROT 5")
    r.clear_errors()
    l.send(":" + "X" * 200)
    r.check("over-long line -> -223", r.err().startswith("-223,"))
    r.clear_errors()

    print("== Capabilities")
    rates = [int(x) for x in l.query(":ACQ:RATE:LIST?").split(",")]
    depths = [int(x) for x in l.query(":ACQ:DEPT:LIST?").split(",")]
    maxp = int(l.query(":ACQ:MAXP?"))
    r.check("rate list ascending, non-empty", rates == sorted(rates) and rates[0] > 0, str(rates))
    r.check("depth list ascending, non-empty", depths == sorted(depths) and depths[0] > 0, str(depths))
    r.check("MAXPoints >= smallest depth", maxp >= depths[0])
    ranges = floats(l.query(":CHAN1:RANG:LIST?"))
    r.check("range list ascending", ranges == sorted(ranges) and ranges[0] > 0, str(ranges))
    coups = l.query(":CHAN1:COUP:LIST?").split(",")
    r.check("coupling list contains DC", "DC" in coups, str(coups))

    print("== Defaults (*RST)")
    r.check("CHAN1 enabled", l.query(":CHAN1:ENAB?") == "1")
    if n > 1:
        r.check("CHAN2 disabled", l.query(":CHAN2:ENAB?") == "0")
    r.check("state STOP", l.query(":STAT?") == "STOP")
    r.check("trigger mode AUTO", l.query(":TRIG:MODE?") == "AUTO")
    r.check("trigger source CHAN1", l.query(":TRIG:SOUR?") == "CHAN1")
    r.check("trigger edge RIS", l.query(":TRIG:EDGE?") == "RIS")
    r.check("rate = highest", int(l.query(":ACQ:RATE?")) == rates[-1])

    print("== Setters / coercion")
    mid = rates[len(rates) // 2]
    l.send(":ACQ:RATE %d" % (mid + 1))
    r.check("rate coerces to nearest", int(l.query(":ACQ:RATE?")) == mid)
    l.send(":ACQ:RATE %.6e" % rates[0])
    r.check("rate accepts exponent notation", int(l.query(":ACQ:RATE?")) == rates[0])
    l.send(":ACQ:DEPT %d" % depths[0])
    r.check("depth set", int(l.query(":ACQ:DEPT?")) == depths[0])
    want = ranges[0] * 0.9
    l.send(":CHAN1:RANG %.6e" % want)
    r.check("range -> smallest >= request", abs(float(l.query(":CHAN1:RANG?")) - ranges[0]) < 1e-6)
    l.send(":CHAN1:RANG %g" % (ranges[-1] * 10))
    r.check("range above max -> largest", abs(float(l.query(":CHAN1:RANG?")) - ranges[-1]) < 1e-6)
    l.send(":CHAN1:ENAB OFF")
    r.check("bool OFF", l.query(":CHAN1:ENAB?") == "0")
    l.send(":CHAN1:ENAB ON")
    r.check("bool ON", l.query(":CHAN1:ENAB?") == "1")
    l.send(":CHAN1:OFFS 0")
    float(l.query(":CHAN1:OFFS?"))
    r.check("offset query is a real", True)
    l.send(":TRIG:EDGE FALLING")
    r.check("edge long form -> short reply", l.query(":TRIG:EDGE?") == "FALL")
    l.send(":TRIG:EDGE ris")
    l.send(":TRIG:POS 0.25")
    p = float(l.query(":TRIG:POS?"))
    r.check("trigger position ~0.25", abs(p - 0.25) < 0.01, str(p))
    l.send(":TRIG:POS 1.5")
    r.check("trigger position out of range -> -222", r.err().startswith("-222,"))
    l.send(":CHAN1:COUP XYZ")
    e = r.err()
    r.check("bad enum -> error", not e.startswith("0,"), e)
    r.clear_errors()

    if n > 1:
        print("== Memory limit")
        for c in range(1, n + 1):
            l.send(":CHAN%d:ENAB 1" % c)
        l.send(":ACQ:DEPT %d" % depths[-1])
        d = int(l.query(":ACQ:DEPT?"))
        r.check("depth x channels <= MAXPoints", d * n <= maxp or d == depths[0], "%d x %d > %d" % (d, n, maxp))
        for c in range(2, n + 1):
            l.send(":CHAN%d:ENAB 0" % c)

    print("== Run control and readout")
    l.send("*RST")
    l.send(":ACQ:RATE %d" % rates[-1])
    l.send(":ACQ:DEPT %d" % depths[0])
    l.send(":TRIG:MODE AUTO")
    payload = l.query_block(":WAV:DATA?")
    r.check(":WAV:DATA? while STOP -> empty block", payload == b"")
    r.check("... and error -221", r.err().startswith("-221,"))

    # AUTO mode with an unreachable level -> FORCED record
    rng1 = float(l.query(":CHAN1:RANG?"))
    off1 = float(l.query(":CHAN1:OFFS?"))
    l.send(":TRIG:LEV %g" % (off1 + rng1 / 2))
    l.send(":TRIG:EDGE RIS")
    try:
        rec = r.capture(":SINGLE")
        ok = True
    except TimeoutError as e:
        ok = False
        r.check("AUTO single capture completes", False, str(e))
    if ok:
        r.check("magic MCXW", rec.magic == b"MCXW")
        r.check("version 1", rec.version == 1)
        r.check("header_len = 40 + 8*nch", rec.header_len == 40 + 8 * len(rec.channels))
        r.check("payload length matches header", len(rec.payload) == rec.expected_len(),
                "%d vs %d" % (len(rec.payload), rec.expected_len()))
        r.check("channel mask = CHAN1 only", rec.channels == [1], str(rec.channels))
        r.check("depth matches :ACQ:DEPT?", rec.depth == int(l.query(":ACQ:DEPT?")))
        r.check("period matches rate", abs(rec.period_fs - 1e15 / rates[-1]) <= 1, str(rec.period_fs))
        r.check("reserved = 0", rec.reserved == 0)
        r.check("bits matches :SYST:BITS?", rec.bits == bits)
        r.check("trigger_frac in [0,1)", 0 <= rec.trigger_frac < 1)
        r.check("unreachable level in AUTO -> FORCED flag", rec.flags & 1 == 1, "flags=%d" % rec.flags)
        r.check("SINGLE -> STOP after readout", l.query(":STAT?") == "STOP")
        vpc, offv = rec.cal[1]
        r.check("volts_per_code positive", vpc > 0)
        seq1 = rec.sequence

    # RUN mode re-arms; sequence increments
    l.send(":TRIG:LEV %g" % off1)
    try:
        a = r.capture(":RUN")
        st = l.query(":STAT?")
        r.check("RUN -> re-armed after readout", st in ("ARMED", "TRIGGERED"), st)
        b = None
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if l.query(":STAT?") == "TRIGGERED":
                b = Record(l.query_block(":WAV:DATA?"))
                break
            time.sleep(0.01)
        r.check("RUN delivers a second record", b is not None)
        if b is not None:
            r.check("sequence increments by 1", b.sequence == (a.sequence + 1) & 0xFFFFFFFF,
                    "%d -> %d" % (a.sequence, b.sequence))
        l.send(":STOP")
        time.sleep(0.05)
        st = l.query(":STAT?")
        if st == "TRIGGERED":  # a record completed just before STOP: still readable
            l.query_block(":WAV:DATA?")
            st = l.query(":STAT?")
        r.check("STOP -> STOP", st == "STOP", st)
    except TimeoutError as e:
        r.check("RUN capture", False, str(e))

    # Force trigger in NORMAL mode
    l.send(":TRIG:MODE NORM")
    l.send(":TRIG:LEV %g" % (off1 + rng1 / 2))
    l.send(":SINGLE")
    time.sleep(0.15)
    r.check("NORMAL + unreachable level stays ARMED", l.query(":STAT?") == "ARMED")
    l.send(":TRIG:FORC")
    try:
        rec = r.capture(":TRIG:FORC", timeout=3)  # second FORC harmless if already triggered
        r.check(":TRIG:FORC produces FORCED record", rec.flags & 1 == 1)
    except TimeoutError as e:
        r.check(":TRIG:FORC produces record", False, str(e))

    if n > 1:
        l.send(":CHAN2:ENAB 1")
        l.send(":TRIG:MODE AUTO")
        rec = r.capture(":SINGLE")
        r.check("two channels -> mask 0b11, channel-major data", rec.channels == [1, 2]
                and len(rec.payload) == rec.expected_len())
        l.send(":CHAN2:ENAB 0")

    if signal:
        print("== Signal / trigger accuracy (needs a signal on CHAN1)")
        l.send("*RST")
        l.send(":ACQ:RATE %d" % min(rates, key=lambda x: abs(x - 1_000_000)))
        l.send(":ACQ:DEPT %d" % min(depths, key=lambda x: abs(x - 10000)))
        l.send(":CHAN1:RANG 2")
        l.send(":TRIG:MODE NORM")
        l.send(":TRIG:POS 0.5")
        for edge, lvl in (("RIS", 0.1), ("FALL", -0.1)):
            l.send(":TRIG:EDGE " + edge)
            l.send(":TRIG:LEV %g" % lvl)
            try:
                rec = r.capture(":SINGLE", timeout=3)
            except TimeoutError as e:
                r.check("%s trigger fires" % edge, False, str(e))
                continue
            v = rec.volts(1)
            ti = rec.trigger_index
            r.check("%s: not FORCED" % edge, rec.flags & 1 == 0)
            r.check("%s: trigger_index ~ POS x depth" % edge, abs(ti - rec.depth // 2) <= 1,
                    "%d vs %d" % (ti, rec.depth // 2))
            # look a few samples around the trigger, average out noise
            w = 3
            before = sum(v[ti - w - 1:ti - 1]) / w
            after = sum(v[ti + 1:ti + w + 1]) / w
            if edge == "RIS":
                ok = before <= lvl + 0.05 and after >= lvl - 0.05 and after > before
            else:
                ok = before >= lvl - 0.05 and after <= lvl + 0.05 and after < before
            r.check("%s: data crosses level at trigger_index" % edge, ok,
                    "before=%.3f after=%.3f level=%.3f" % (before, after, lvl))

    l.send("*RST")
    r.check("error queue clean at end", r.err() == '0,"No error"')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", help="serial port path, or tcp:HOST:PORT")
    ap.add_argument("--signal", action="store_true", help="also run trigger-accuracy checks on CHAN1")
    ap.add_argument("--timeout", type=float, default=3.0)
    a = ap.parse_args()
    r = Runner(Link(a.target, a.timeout))
    try:
        run(r, a.signal)
    except (TimeoutError, ValueError) as e:
        r.check("protocol exchange", False, "aborted: %s" % e)
    print("\n%d passed, %d failed" % (r.passed, len(r.failed)))
    for f in r.failed:
        print("  failed:", f)
    sys.exit(1 if r.failed else 0)


if __name__ == "__main__":
    main()
