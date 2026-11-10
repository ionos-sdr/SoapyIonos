#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# spyserver_test.py - SpyServer client test for tools/ionos_spyserver
#   Copyright (c) 2026 Zoltan Doczi HA7DCD
#
# Talks to ionos_spyserver the way SDR++'s spyserver_source does (HELLO, then
# IQ_FORMAT, IQ_DECIMATION, IQ_FREQUENCY, STREAMING_MODE, GAIN, DIGITAL_GAIN,
# STREAMING_ENABLED) and checks:
#   - DeviceInfo: the rate list (MaximumSampleRate >> i) and the frequency range,
#   - ClientSync after tuning,
#   - the delivered sample rate for each requested decimation stage,
#   - the mock's test tone is found at the expected offset in every format.
#
#   spyserver_test.py --port 5555 --secs 4 --tone 13370

import argparse, socket, struct, sys, time

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--port", type=int, default=5555)
ap.add_argument("--secs", type=float, default=4.0)
ap.add_argument("--tone", type=float, default=13370.0)
ap.add_argument("--freq", type=int, default=433920000)
args = ap.parse_args()

PROTO = (2 << 24) | (0 << 16) | 1700
HDR = struct.Struct("<5I")
fails = 0


def check(ok, what):
    global fails
    print(("PASS  " if ok else "FAIL  ") + what)
    if not ok:
        fails += 1


s = socket.create_connection((args.host, args.port), timeout=5)
s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)


def cmd(t, body=b""):
    s.sendall(struct.pack("<2I", t, len(body)) + body)


def setting(k, v):
    cmd(2, struct.pack("<2I", k, v))


def recv_exact(n):
    b = bytearray()
    while len(b) < n:
        c = s.recv(n - len(b))
        if not c:
            raise EOFError("server closed")
        b += c
    return bytes(b)


def read_msg():
    h = HDR.unpack(recv_exact(HDR.size))
    return h, recv_exact(h[4]) if h[4] else b""


cmd(0, struct.pack("<I", PROTO) + b"SDR++ test")
info = sync = None
while info is None or sync is None:
    h, b = read_msg()
    if h[1] & 0xFFFF == 0:
        info = struct.unpack("<12I", b)
    elif h[1] & 0xFFFF == 1:
        sync = struct.unpack("<9I", b)

dev_type, serial, max_sr, max_bw, stages, gstages, gmax, fmin, fmax, res, min_dec, forced = info
rates = [max_sr / (1 << i) for i in range(min_dec, stages + 1)]
print(f"device type {dev_type} serial {serial:08X}, rates {[round(r) for r in rates]}, {fmin}..{fmax} Hz")
check(max_sr == 200000 and stages == 5, "DeviceInfo: 200 ksps max, 5 halving stages (200k..6.25k)")
check(fmin <= args.freq <= fmax and fmax > fmin, "DeviceInfo: frequency range covers the test frequency")
check(gmax == 0, "DeviceInfo: no gain slider (device AGC)")


def run(stage, fmt, label):
    global fails
    setting(100, fmt)                    # IQ_FORMAT
    setting(102, stage)                  # IQ_DECIMATION
    setting(101, args.freq)              # IQ_FREQUENCY
    setting(0, 1)                        # STREAMING_MODE = IQ only
    setting(2, 0)                        # GAIN
    setting(103, stage * 3)              # IQ_DIGITAL_GAIN
    setting(1, 1)                        # STREAMING_ENABLED
    want = max_sr / (1 << stage)
    chunks, n, t0, seq_prev, seq_gaps, synced = [], 0, None, None, 0, None
    while True:
        h, b = read_msg()
        mt = h[1] & 0xFFFF
        if mt == 1:
            synced = struct.unpack("<9I", b)
            continue
        if mt not in (100, 101, 103):
            continue
        if t0 is None:
            t0 = time.monotonic()            # start counting after the first IQ message
            seq_prev = h[3]
            continue
        if h[3] != seq_prev + 1:
            seq_gaps += 1
        seq_prev = h[3]
        if mt == 101:
            x = np.frombuffer(b, "<i2").astype(np.float32) / 32768.0
        elif mt == 103:
            x = np.frombuffer(b, "<f4")
        else:
            x = (np.frombuffer(b, np.uint8).astype(np.float32) - 128.0) / 128.0
        chunks.append(x)
        n += len(x) // 2
        if time.monotonic() - t0 >= args.secs:
            break
    dt = time.monotonic() - t0
    setting(1, 0)                        # STREAMING_ENABLED = 0
    # drain what is in flight
    s.settimeout(0.5)
    try:
        while True:
            read_msg()
    except (socket.timeout, EOFError):
        pass
    s.settimeout(5)

    sps = n / dt
    x = np.concatenate(chunks)
    z = x[0::2] + 1j * x[1::2]
    z = z[-min(len(z), 65536):]
    spec = np.abs(np.fft.fft(z * np.hanning(len(z))))
    f = np.fft.fftfreq(len(z), 1.0 / want)
    peak = f[int(np.argmax(spec))]
    print(f"  {label}: {n} samples in {dt:.2f} s = {sps:.0f} sps (want {want:.0f}), tone at {peak:.1f} Hz, "
          f"seq gaps {seq_gaps}")
    check(abs(sps - want) / want < 0.03, f"{label}: sample rate")
    check(abs(peak - args.tone) < max(2 * want / len(z), 5.0) if abs(args.tone) < want / 2 else True,
          f"{label}: test tone at its offset")
    check(synced is not None and synced[3] == args.freq, f"{label}: ClientSync reports the tuned frequency")


run(0, 2, "200 ksps INT16")
run(1, 4, "100 ksps FLOAT")
run(2, 1, "50 ksps UINT8")
run(1, 2, "100 ksps INT16 (rate change back up)")

cmd(3)
h, _ = read_msg()
while h[1] & 0xFFFF != 2:
    h, _ = read_msg()
check(True, "PING -> PONG")
s.close()
print("ALL SPYSERVER TESTS PASSED" if fails == 0 else f"{fails} SPYSERVER TEST(S) FAILED")
sys.exit(1 if fails else 0)
