#!/usr/bin/env python3
"""Automatic PI tuning for the motor PID demo (MOTOR_PID_DEMO in src/main.c).

Steps, all over the same USART5 link motor_pid_uart.py uses:
  1. Direction check: `open 20` / `open -20` must give positive / negative
     speed. A reversed encoder makes the closed loop run away, so nothing
     closed-loop runs until this passes.
  2. Plant identification: open-loop duty steps from rest, each fitted to a
     first-order-plus-dead-time model (gain, time constant, delay).
  3. Gains from that model (SIMC / lambda tuning for PI), applied live with
     the `kp` / `ki` / `kd` commands.
  4. Verification: closed-loop steps (default 30, 60, -30, 0 RPM), scored for
     overshoot, settling time, steady-state error and jitter. Gains are scaled
     down and re-verified if a step overshoots too much, or up if it's slow.

Safety: aborts on FAULT:STALL, on speed far above target, or on speed in the
wrong direction. Aborting sends `cap 0` (forces zero output even in a
runaway, where `stop` would not help), then `stop`, and leaves the cap at 0;
raise it again with `cap 100` once you've looked at what happened. Ctrl-C does
the same. Keep a hand on the 12 V supply switch anyway.

The serial port can only be open in one program at a time: close
motor_pid_uart.py and `pio device monitor` first.

--motor 2 tunes the second motor: every per-motor command gets the firmware's
"m2 " prefix and the motor-2 telemetry fields (tgt2/meas2/..., flags open2 /
stall2) are read instead. The other motor is left at target 0.
"""
import argparse
import csv
import math
import queue
import re
import sys
import threading
import time

import serial

TEL_RE = re.compile(
    r"tgt=(?P<tgt>-?[\d.]+) meas=(?P<meas>-?[\d.]+) duty=(?P<duty>-?[\d.]+)% "
    r"cap=(?P<cap>-?[\d.]+)% cnt=(?P<cnt>-?\d+)(?P<rest>.*)")
TEL2_RE = re.compile(
    r" tgt2=(?P<tgt>-?[\d.]+) meas2=(?P<meas>-?[\d.]+) duty2=(?P<duty>-?[\d.]+)% "
    r"cap2=(?P<cap>-?[\d.]+)% cnt2=(?P<cnt>-?\d+)")
GLOBAL_CMDS = ("rate", "stop", "reset")      # act on both motors; never prefixed


class Abort(Exception):
    pass


class Link:
    """Serial link: a reader thread splits telemetry from command replies."""

    def __init__(self, port, baud, log_path, motor=1):
        self.motor = motor
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.tel = queue.Queue()
        self.replies = queue.Queue()
        self.stop_evt = threading.Event()
        self.log = open(log_path, "w", newline="") if log_path else None
        self.csv = csv.writer(self.log) if self.log else None
        if self.csv:
            self.csv.writerow(["host_t", "dev_ms", "tgt", "meas", "duty", "cap", "cnt", "flags", "phase"])
        self.phase = ""
        self.cap = None               # duty cap to restore after clearing the integral
        self.th = threading.Thread(target=self._reader, daemon=True)
        self.th.start()

    def _reader(self):
        while not self.stop_evt.is_set():
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                break
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            m = TEL_RE.match(line)
            if not m:
                if line.startswith(("OK", "ERR")):
                    self.replies.put(line)
                continue
            rest = m.group("rest")
            tm = re.search(r"\bt=(\d+)", rest)
            open_flag, fault_flag = " OPEN" in rest, "FAULT:STALL" in rest
            if self.motor == 2:
                m = TEL2_RE.match(rest)
                if not m:
                    raise SystemExit("--motor 2: this firmware sends no motor-2 telemetry")
                open_flag, fault_flag = " open2" in rest, " stall2" in rest
            row = {
                "host_t": time.monotonic(),
                "t": int(tm.group(1)) / 1000.0 if tm else None,
                "tgt": float(m.group("tgt")), "meas": float(m.group("meas")),
                "duty": float(m.group("duty")), "cap": float(m.group("cap")),
                "cnt": int(m.group("cnt")),
                "open": open_flag, "fault": fault_flag,
            }
            self.tel.put(row)
            if self.csv:
                flags = ("OPEN " if row["open"] else "") + ("FAULT" if row["fault"] else "")
                self.csv.writerow([f"{row['host_t']:.4f}", tm.group(1) if tm else "", row["tgt"],
                                   row["meas"], row["duty"], row["cap"], row["cnt"], flags.strip(),
                                   self.phase])

    def cmd(self, text, timeout=1.0):
        """Send one command and return its OK/ERR reply."""
        while not self.replies.empty():
            self.replies.get_nowait()
        self.ser.write((self.wire(text) + "\n").encode())
        try:
            reply = self.replies.get(timeout=timeout)
        except queue.Empty:
            raise Abort(f"no reply to '{text}'")
        if reply.startswith("ERR"):
            raise Abort(f"'{text}' -> {reply}")
        return reply

    def wire(self, text):
        """Per-motor commands go to motor 2 with the firmware's "m2 " prefix."""
        if self.motor == 2 and not text.startswith(GLOBAL_CMDS):
            return "m2 " + text
        return text

    def drain(self):
        while not self.tel.empty():
            self.tel.get_nowait()

    def collect(self, seconds, monitor=None):
        """Gather telemetry rows for `seconds`, calling monitor(row) on each."""
        rows, end = [], time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                row = self.tel.get(timeout=0.3)
            except queue.Empty:
                raise Abort("telemetry stopped arriving")
            if row["fault"]:
                raise Abort("FAULT:STALL reported by the firmware")
            if monitor:
                monitor(row)
            rows.append(row)
        return rows

    def close(self):
        self.stop_evt.set()
        self.th.join(timeout=1)
        self.ser.close()
        if self.log:
            self.log.close()


def mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def wait_stopped(link, limit=4.0):
    """Wait until the motor has been still (|meas| < 1) for 0.3 s."""
    still_since, end = None, time.monotonic() + limit
    while time.monotonic() < end:
        row = link.collect(0.05)[-1:] or None
        if row is None:
            continue
        now = time.monotonic()
        if abs(row[0]["meas"]) < 1.0:
            still_since = still_since or now
            if now - still_since >= 0.3:
                # Firmware built before the "target 0 means off" fix keeps the
                # integral from the spin-down and holds a few % duty into the
                # stopped motor. Cycling the cap through 0 clears it.
                if link.cap is not None:
                    link.cmd("cap 0")
                    link.cmd(f"cap {link.cap:g}")
                return
        else:
            still_since = None
    raise Abort("motor didn't come to rest")


# ---------------------------------------------------------------- 1. direction

def direction_check(link, duty):
    out = {}
    for sign in (1, -1):
        link.phase = f"dircheck{sign:+d}"
        wait_stopped(link)
        link.drain()
        link.cmd(f"open {sign * duty:g}")
        rows = link.collect(1.5)
        link.cmd("stop")
        tail = [r["meas"] for r in rows[len(rows) // 2:]]
        avg = mean(tail)
        out[sign] = avg
        print(f"  open {sign * duty:+g}%: meas {avg:+.1f} RPM")
        if abs(avg) < 3:
            raise Abort(f"open {sign * duty:g} gave ~0 RPM: the motor isn't turning or the "
                        "encoder isn't counting (check wiring before tuning)")
        if avg * sign < 0:
            raise Abort("encoder counts opposite to the drive direction: set "
                        "MOTOR_ENC_INVERT 1 in src/main.c, reflash with 12 V off, rerun")
    return out


# ------------------------------------------------------- 2. plant identification

def fit_fopdt(t, y, ss):
    """Grid-search delay theta and time constant tau for y = ss*(1-exp(-(t-theta)/tau))."""
    best = None
    for theta in [i * 0.005 for i in range(0, 41)]:            # 0 .. 0.2 s
        for tau in [0.01 * 1.08 ** k for k in range(0, 60)]:   # 0.01 .. ~1 s
            sse = 0.0
            for ti, yi in zip(t, y):
                model = ss * (1 - math.exp(-(ti - theta) / tau)) if ti > theta else 0.0
                sse += (yi - model) ** 2
            if best is None or sse < best[0]:
                best = (sse, theta, tau)
    rms = math.sqrt(best[0] / len(t))
    return best[1], best[2], rms


def step_open(link, duty, seconds):
    link.phase = f"open{duty:g}"
    wait_stopped(link)
    link.drain()
    link.cmd(f"open {duty:g}")
    rows = link.collect(seconds)
    link.cmd("stop")
    # Step applied somewhere between the last non-OPEN line and the first OPEN one;
    # take the midpoint of that interval (one telemetry period wide).
    i0 = next(i for i, r in enumerate(rows) if r["open"])
    t_first = rows[i0]["t"]
    t_prev = rows[i0 - 1]["t"] if i0 > 0 else t_first - 0.02
    t0 = (t_first + t_prev) / 2
    data = [(r["t"] - t0, r["meas"]) for r in rows[i0:] if r["open"]]
    ss = mean([y for tt, y in data if tt >= seconds - 0.8])
    theta, tau, rms = fit_fopdt([d[0] for d in data], [d[1] for d in data], ss)
    print(f"  open {duty:g}%: steady {ss:6.1f} RPM, delay {theta * 1000:3.0f} ms, "
          f"time constant {tau * 1000:4.0f} ms, fit error {rms:.1f} RPM rms")
    return ss, theta, tau


def identify(link, duties, seconds):
    res = [step_open(link, d, seconds) for d in duties]
    (d1, d2), (s1, s2) = duties, (res[0][0], res[1][0])
    gain = (s2 - s1) / (d2 - d1)                 # RPM per % duty
    if gain <= 0:
        raise Abort("speed didn't increase with duty; can't model the motor")
    offset = d1 - s1 / gain                      # duty where speed extrapolates to 0
    theta = mean([r[1] for r in res])
    tau = mean([r[2] for r in res])
    print(f"  model: {gain:.2f} RPM per % duty above ~{offset:.1f}% duty, "
          f"delay {theta * 1000:.0f} ms, time constant {tau * 1000:.0f} ms")
    return gain, offset, theta, tau


# ------------------------------------------------------------- 3. gain design

def simc_pi(gain, theta, tau, tau_c, quant_rpm):
    """SIMC PI: Kc = tau / (K (tau_c + theta)), Ti = min(tau, 4 (tau_c + theta))."""
    kp = tau / (gain * (tau_c + theta))
    ti = min(tau, 4 * (tau_c + theta))
    # Keep one count of speed quantization from moving the duty more than ~2%.
    kp_noise_max = 2.0 / quant_rpm
    if kp > kp_noise_max:
        ti *= kp_noise_max / kp                  # keep the same integral action
        kp = kp_noise_max
    return kp, kp / ti


# ------------------------------------------------------------- 4. verification

RAMP_RPM_PER_S = 200.0   # MOTOR_RAMP_RPM_PER_S in src/main.c


def make_monitor(prev, target):
    """Abort on speed far above target or in the wrong direction, sustained 0.3 s.

    Coming down from `prev` (e.g. 60 -> -30), the motor legitimately keeps
    turning the old way while it slows, so the wrong-direction check only
    starts once the ramp has had time to get there, plus 0.6 s of slack."""
    state = {"bad_since": None, "t0": None}
    top = max(abs(prev), abs(target))
    limit = top + max(30.0, 0.5 * top)
    grace = abs(target - prev) / RAMP_RPM_PER_S + 0.6

    def monitor(row):
        meas = row["meas"]
        state["t0"] = state["t0"] or row["host_t"]
        settled_in = row["host_t"] - state["t0"] > grace
        wrong_way = (settled_in and row["tgt"] != 0 and meas * row["tgt"] < 0
                     and abs(meas) > 10)
        too_fast = abs(meas) > limit
        if wrong_way or too_fast:
            state["bad_since"] = state["bad_since"] or row["host_t"]
            if row["host_t"] - state["bad_since"] > 0.3:
                why = "turning the wrong way" if wrong_way else f"{meas:.0f} RPM, far above target"
                raise Abort(f"motor {why}")
        else:
            state["bad_since"] = None
    return monitor


def score(rows, start, target, t_cmd):
    """Metrics for one closed-loop step from `start` to `target` RPM."""
    t = [r["t"] - t_cmd for r in rows]
    y = [r["meas"] for r in rows]
    step = target - start
    band = 0.05 * abs(target) + 2.0
    settle = None
    for i in range(len(y)):
        if all(abs(v - target) <= band for v in y[i:]):
            settle = t[i]
            break
    tail = [v for tt, v in zip(t, y) if tt >= t[-1] - 0.5]
    if step > 0:
        over = max(0.0, max(y) - target)
    else:
        over = max(0.0, target - min(y))
    duty_tail = [r["duty"] for tt, r in zip(t, rows) if tt >= t[-1] - 0.5]
    sd = math.sqrt(mean([(v - mean(tail)) ** 2 for v in tail]))
    return {
        "target": target, "overshoot_pct": 100 * over / abs(step) if step else 0.0,
        "overshoot_rpm": over, "settle_s": settle,
        # the setpoint ramp alone takes |step| / RAMP_RPM_PER_S; judge settling after it
        "settle_after_ramp_s": None if settle is None else max(0.0, settle - abs(step) / RAMP_RPM_PER_S), "err": mean(tail) - target, "jitter_sd": sd,
        "duty": mean(duty_tail), "cap": rows[-1]["cap"],
    }


def verify(link, targets, hold):
    link.phase = "verify"
    wait_stopped(link)
    link.drain()
    results, prev = [], 0.0
    for tgt in targets:
        link.phase = f"verify{tgt:g}"
        link.cmd(f"{tgt:g}")
        rows = link.collect(hold, monitor=make_monitor(prev, tgt))
        t_cmd = next((r["t"] for r in rows if r["tgt"] == tgt), rows[0]["t"])
        rows = [r for r in rows if r["t"] >= t_cmd]
        s = score(rows, prev, tgt, t_cmd)
        results.append(s)
        settle = (f"{s['settle_s']:.2f} s ({s['settle_after_ramp_s']:.2f} after ramp)"
                  if s["settle_s"] is not None else "not settled")
        print(f"  {prev:+5.0f} -> {tgt:+5.0f} RPM: overshoot {s['overshoot_pct']:4.1f}%, "
              f"settle {settle:>24}, error {s['err']:+5.1f}, jitter sd {s['jitter_sd']:.1f}, "
              f"duty {s['duty']:+5.1f}%")
        prev = tgt
    link.cmd("0")
    wait_stopped(link)
    return results


QUANT_RPM = 60.0 / (751.8 * 5 * 0.01)   # one count of the 50 ms speed window


def judge(results, max_over, slow_s):
    """Overshoot smaller than ~one count of speed resolution isn't the controller's
    doing (a 1.2 RPM blip on a 10 RPM step reads as 12%), so it's ignored; settling
    is timed from the end of the setpoint ramp, which the controller can't beat."""
    nonzero = [r for r in results if r["target"] != 0]
    worst_over = max((r["overshoot_pct"] if r["overshoot_rpm"] > 1.25 * QUANT_RPM else 0.0)
                     for r in nonzero)
    unsettled = any(r["settle_s"] is None for r in nonzero)
    capped = any(abs(r["duty"]) >= r["cap"] - 1 for r in nonzero)
    slowest = max((r["settle_after_ramp_s"] if r["settle_after_ramp_s"] is not None else 99)
                  for r in nonzero)
    if capped:
        return "capped", worst_over, slowest
    if worst_over > max_over or unsettled and worst_over > max_over / 2:
        return "too_aggressive", worst_over, slowest
    if slowest > slow_s:
        return "too_slow", worst_over, slowest
    return "ok", worst_over, slowest


# ----------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--motor", type=int, choices=(1, 2), default=1, help="which motor to tune (default 1)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dir-duty", type=float, default=20, help="duty %% for the direction check")
    ap.add_argument("--id-duties", type=float, nargs=2, default=[15, 30],
                    help="two open-loop duty %% steps for identification")
    ap.add_argument("--id-seconds", type=float, default=2.5)
    ap.add_argument("--targets", type=float, nargs="+", default=[30, 60, -30, 0],
                    help="closed-loop verification targets in RPM, in order")
    ap.add_argument("--hold", type=float, default=2.5, help="seconds per verification step")
    ap.add_argument("--tau-c", type=float, default=0.2,
                    help="desired closed-loop time constant in s (smaller = more aggressive)")
    ap.add_argument("--max-overshoot", type=float, default=10.0, help="%% overshoot allowed")
    ap.add_argument("--slow", type=float, default=1.0,
                    help="settling time after the setpoint ramp (s) counted as too slow")
    ap.add_argument("--rounds", type=int, default=3, help="max verify/adjust rounds")
    ap.add_argument("--log", default=f"autotune_{time.strftime('%Y%m%d_%H%M%S')}.csv",
                    help="CSV log of every telemetry line ('' to disable)")
    a = ap.parse_args()

    try:
        link = Link(a.port, a.baud, a.log, a.motor)
    except serial.SerialException as e:
        sys.exit(f"Could not open {a.port}: {e}")

    cap_before = None
    ok = False
    try:
        link.cmd("rate 50")
        first = link.collect(0.3)
        if first[-1]["t"] is None:
            raise Abort("firmware doesn't send timestamps: flash the build with the 'rate' command")
        cap_before = first[-1]["cap"]
        link.cap = cap_before
        link.cmd("stop")
        print(f"Connected to motor {a.motor}. Duty cap {cap_before:g}%.")

        print("1. Direction check")
        direction_check(link, a.dir_duty)

        print("2. Measuring the motor (open-loop steps)")
        gain, offset, theta, tau = identify(link, a.id_duties, a.id_seconds)

        quant = 60.0 / (751.8 * 5 * 0.01)        # RPM per count of the 50 ms speed window
        kp, ki = simc_pi(gain, theta, tau, a.tau_c, quant)
        print(f"3. Gains from the model: Kp {kp:.3f}, Ki {ki:.3f} (tau_c {a.tau_c:g} s)")

        history = []
        for rnd in range(1, a.rounds + 1):
            link.cmd(f"kp {kp:.4f}")
            link.cmd(f"ki {ki:.4f}")
            link.cmd("kd 0")
            print(f"4.{rnd} Verifying Kp {kp:.3f}, Ki {ki:.3f}")
            res = verify(link, a.targets, a.hold)
            verdict, over, slowest = judge(res, a.max_overshoot, a.slow)
            history.append((kp, ki, verdict, over, slowest))
            if verdict == "ok":
                break
            if verdict == "capped":
                print("   a step needed more duty than the cap allows; lower the targets or raise "
                      "the cap. Keeping these gains.")
                break
            f = 0.7 if verdict == "too_aggressive" else 1.3
            print(f"   {verdict.replace('_', ' ')} (worst overshoot {over:.1f}%, slowest settle "
                  f"{slowest:.2f} s): scaling gains by {f}")
            kp, ki = kp * f, ki * f
        ok = True

        print("\nResult")
        for kpv, kiv, verdict, over, slowest in history:
            print(f"  Kp {kpv:.3f}  Ki {kiv:.3f}: {verdict}, worst overshoot {over:.1f}%, "
                  f"slowest settle after ramp {slowest:.2f} s")
        kp_f, ki_f = history[-1][0], history[-1][1]
        print(f"\nThese gains are live now on motor {a.motor} and are lost on reset. To keep them, set in "
              f"src/main.c:\n  #define MOTOR{a.motor}_BOOT_KP {kp_f:.3f}f\n  #define MOTOR{a.motor}_BOOT_KI {ki_f:.3f}f")
    except (Abort, KeyboardInterrupt, StopIteration) as e:
        reason = "interrupted" if isinstance(e, KeyboardInterrupt) else str(e) or "unexpected telemetry"
        print(f"\nABORTED: {reason}")
        for c in ("cap 0", "stop"):
            try:
                link.ser.write((link.wire(c) + "\n").encode())
                time.sleep(0.1)
            except serial.SerialException:
                pass
        print("Sent 'cap 0' and 'stop'. The cap stays at 0 until you raise it (e.g. 'cap 100').")
    finally:
        try:
            link.ser.write(b"rate 10\n")
            if ok:
                link.ser.write(b"stop\n")
            time.sleep(0.2)
        except serial.SerialException:
            pass
        link.close()
        if a.log:
            print(f"Telemetry log: {a.log}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
