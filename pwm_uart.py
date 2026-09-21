#!/usr/bin/env python3
"""Set the STM32 PC9 PWM duty cycle (TIM3 CH4) over the USB-UART link.

Each duty cycle (percent) you type is sent as a text line (e.g. "40.000\\n")
to the STM32's USART5 RX (PD2). The firmware updates the PWM on PC9 and
replies with the duty cycle it actually produced. Type q (or Ctrl-C / Ctrl-D)
to quit.

The serial port can only be open in one program at a time, so close
`pio device monitor` before running this.
"""
import argparse
import sys

import serial

MIN_DUTY = 0.0    # keep in sync with the range check in pwm_uart_loop() in src/main.c
MAX_DUTY = 100.0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default="/dev/ttyUSB0", help="serial port (default: %(default)s)")
    ap.add_argument("--baud", type=int, default=115200, help="baud rate (default: %(default)s)")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"Could not open {args.port}: {e}")

    with ser:
        print(f"Connected to {args.port} at {args.baud} baud. "
              f"Duty cycle range {MIN_DUTY:g}-{MAX_DUTY:g} %, q to quit.")
        while True:
            try:
                text = input("Enter PWM duty cycle (%): ").strip().rstrip("%").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if text.lower() in ("q", "quit", "exit"):
                break
            if not text:
                continue

            try:
                duty = float(text)
            except ValueError:
                print(f"'{text}' is not a number.")
                continue
            if not (MIN_DUTY <= duty <= MAX_DUTY):  # also rejects nan
                print(f"Duty cycle must be between {MIN_DUTY:g} and {MAX_DUTY:g} %.")
                continue

            ser.reset_input_buffer()
            ser.write(f"{duty:.3f}\n".encode())
            reply = ser.readline().decode(errors="replace").strip()
            print(reply if reply else "No reply from the STM32 (check wiring, port and baud rate).")


if __name__ == "__main__":
    main()
