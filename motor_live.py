#!/usr/bin/env python3
"""Live graph + control panel for the motor PID demo (MOTOR_PID_DEMO in src/main.c).

Holds the USART5 serial link, switches telemetry to 50 Hz with device
timestamps (`rate 50`), and serves a page with live charts of target vs.
measured speed and duty, plus buttons and a command box that send the same
commands as motor_pid_uart.py.

    python3 motor_live.py                    # then open http://127.0.0.1:8090
    python3 motor_live.py --host 0.0.0.0     # reachable from other machines

Only one program can hold the serial port: close motor_pid_uart.py,
motor_autotune.py and `pio device monitor` first. On exit (Ctrl-C) it sends
`stop` and puts telemetry back to 10 Hz.
"""
import argparse
import collections
import json
import re
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import serial

HERE = Path(__file__).parent
TEL_RE = re.compile(
    r"tgt=(?P<tgt>-?[\d.]+) meas=(?P<meas>-?[\d.]+) duty=(?P<duty>-?[\d.]+)% "
    r"cap=(?P<cap>-?[\d.]+)% cnt=(?P<cnt>-?\d+)(?P<rest>.*)")
CMD_RE = re.compile(r"^[A-Za-z0-9 .+\-]{1,22}$")   # firmware lines are < 24 chars


class Board:
    def __init__(self, port, baud, keep_s):
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.lock = threading.Lock()          # guards samples/messages/seq
        self.write_lock = threading.Lock()    # one command at a time
        self.samples = collections.deque(maxlen=int(keep_s * 50))
        self.messages = collections.deque(maxlen=50)
        self.seq = 0                          # increments per stored sample
        self.msg_seq = 0
        self.epoch = 0                        # bumps when the board restarts
        self.last_t = None
        self.last_rate_fix = 0.0
        self.last_line = 0.0
        self.replies = collections.deque()
        self.reply_evt = threading.Event()
        threading.Thread(target=self._reader, daemon=True).start()
        self.write("rate 50")

    def write(self, text):
        self.ser.write((text + "\n").encode())

    def _reader(self):
        while True:
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                time.sleep(0.5)
                continue
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            self.last_line = time.monotonic()
            m = TEL_RE.match(line)
            if not m:
                if line:
                    with self.lock:
                        self.msg_seq += 1
                        self.messages.append((self.msg_seq, line))
                    if line.startswith(("OK", "ERR")):
                        self.replies.append(line)
                        self.reply_evt.set()
                continue
            rest = m.group("rest")
            tm = re.search(r"\bt=(\d+)", rest)
            if not tm:
                # 10 Hz telemetry without timestamps: the board was reset or someone
                # sent `rate 10`. Ask for 50 Hz again (at most every 2 s).
                now = time.monotonic()
                if now - self.last_rate_fix > 2.0:
                    self.last_rate_fix = now
                    self.write("rate 50")
                continue
            t = int(tm.group(1)) / 1000.0
            with self.lock:
                if self.last_t is not None and t < self.last_t - 0.5:
                    self.epoch += 1            # board restarted: its clock went back
                    self.samples.clear()
                self.last_t = t
                self.seq += 1
                self.samples.append((self.seq, round(t, 3), float(m.group("tgt")),
                                     float(m.group("meas")), float(m.group("duty")),
                                     float(m.group("cap")), int(m.group("cnt")),
                                     int(" OPEN" in rest), int("FAULT:STALL" in rest)))

    def command(self, text, timeout=1.0):
        with self.write_lock:
            self.replies.clear()
            self.reply_evt.clear()
            self.write(text)
            if not self.reply_evt.wait(timeout):
                return "ERR no reply from the board"
            return self.replies.popleft()

    def since(self, seq, msg_seq):
        with self.lock:
            rows = [s for s in self.samples if s[0] > seq]
            msgs = [m for m in self.messages if m[0] > msg_seq]
            return rows, msgs, self.epoch, self.seq, self.msg_seq

    def link_ok(self):
        return time.monotonic() - self.last_line < 1.0


def make_handler(board):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlsplit(self.path).path       # ignore ?theme=... etc.
            if path == "/":
                self._send(200, "text/html; charset=utf-8", (HERE / "motor_live.html").read_bytes())
            elif path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                seq = msg_seq = 0
                epoch = None
                try:
                    while True:
                        rows, msgs, ep, seq_now, msg_now = board.since(seq, msg_seq)
                        if ep != epoch:            # first frame, or board restarted
                            rows, msgs, ep, seq_now, msg_now = board.since(0, msg_seq)
                        payload = {"epoch": ep, "reset": ep != epoch, "rows": [r[1:] for r in rows],
                                   "msgs": [m[1] for m in msgs], "link": board.link_ok()}
                        epoch, seq, msg_seq = ep, seq_now, msg_now
                        self.wfile.write(f"data: {json.dumps(payload, separators=(',', ':'))}\n\n".encode())
                        self.wfile.flush()
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self._send(404, "text/plain", b"not found")

        def do_POST(self):
            if urlsplit(self.path).path != "/cmd":
                self._send(404, "text/plain", b"not found")
                return
            n = min(int(self.headers.get("Content-Length", 0) or 0), 64)
            text = self.rfile.read(n).decode(errors="replace").strip()
            if not CMD_RE.match(text):
                reply = "ERR not a valid command"
            else:
                reply = board.command(text)
            self._send(200, "application/json", json.dumps({"cmd": text, "reply": reply}).encode())

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--http-port", type=int, default=8090)
    ap.add_argument("--keep", type=float, default=120, help="seconds of history kept for the charts")
    a = ap.parse_args()

    try:
        board = Board(a.port, a.baud, a.keep)
    except serial.SerialException as e:
        raise SystemExit(f"Could not open {a.port}: {e}")
    srv = ThreadingHTTPServer((a.host, a.http_port), make_handler(board))
    srv.daemon_threads = True

    def on_term(*_):
        raise KeyboardInterrupt           # same cleanup as Ctrl-C (also when backgrounded)
    signal.signal(signal.SIGTERM, on_term)
    print(f"Serving on http://{a.host}:{a.http_port}  (Ctrl-C stops the motor and exits)", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        for c in ("stop", "rate 10"):
            try:
                board.write(c)
                time.sleep(0.1)
            except serial.SerialException:
                pass
        print("Sent 'stop' and 'rate 10'.")


if __name__ == "__main__":
    main()
