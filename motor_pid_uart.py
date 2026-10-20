#!/usr/bin/env python3
"""Command-line front end for the closed-loop motor PID demo (see main.c,
MOTOR_PID_DEMO / motor_uart_loop()).

Type a target RPM (signed, e.g. "40" or "-40") and Enter to command speed,
or one of:
  open <pct>  open-loop test: fixed duty in % (signed), no PID, no stall
              watchdog, auto-stops after 15 s -- for diagnosing wiring and demos
  cap <pct>   set the runtime duty-cycle cap in % (hard-clamped in firmware)
  reset       clear a latched stall fault
  stop        equivalent to "0", also ends an open-loop test
  q           quit

The firmware streams a telemetry line (~10 Hz) on its own; this script prints
it from a background thread only when it changes, and keeps what you have
typed visible on the prompt line.

The serial port can only be open in one program at a time, so close
`pio device monitor` before running this.
"""
import argparse
import readline  # input() line editing; lets the reader thread redraw typed text
import sys
import threading

import serial

PROMPT = "RPM (or cap/reset/stop/q) > "


def reader_thread(ser, stop_event):
    last = None
    while not stop_event.is_set():
        try:
            line = ser.readline().decode(errors="replace").strip()
        except serial.SerialException:
            break
        # Telemetry repeats ~10x/s; only show it when something changed, so a
        # stopped or faulted motor doesn't flood the screen while you type.
        if not line or line == last:
            continue
        last = line
        # Erase the prompt line, print the new line, then redraw the prompt
        # together with whatever has been typed so far, so typing stays visible.
        typed = readline.get_line_buffer()
        print(f"\r\033[K{line}\n{PROMPT}{typed}", end="", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", default="/dev/ttyUSB0", help="serial port (default: %(default)s)")
    ap.add_argument("--baud", type=int, default=115200, help="baud rate (default: %(default)s)")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"Could not open {args.port}: {e}")

    stop_event = threading.Event()
    t = threading.Thread(target=reader_thread, args=(ser, stop_event), daemon=True)
    t.start()

    print(f"Connected to {args.port} at {args.baud} baud.")
    with ser:
        while True:
            try:
                text = input(PROMPT).strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not text:
                continue
            if text.lower() in ("q", "quit", "exit"):
                break
            ser.write((text + "\n").encode())

    stop_event.set()


if __name__ == "__main__":
    main()
