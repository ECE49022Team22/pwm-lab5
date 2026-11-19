#!/usr/bin/env python3
"""Build the motor firmware and flash it onto the team PCB's STM32, from the Raspberry Pi.

    python3 flash_pcb.py

There is no ST-LINK programmer: the Pi itself talks to the STM32's debug port (SWD) through three of its GPIO pins
(pin 36 = SWDIO, pin 38 = SWCLK, pin 40 = NRST, GND on pin 34). OpenOCD drives those pins.
"""
import os
import subprocess
import time

import serial   # pyserial: talks to the STM32 over the Pi's UART

HOME = os.path.expanduser("~")
FIRMWARE_DIR = f"{HOME}/pwm-lab5"                                      # this repo
ELF = f"{FIRMWARE_DIR}/.pio/build/nucleo_f091rc/firmware.elf"         # what PlatformIO builds
PIO = f"{HOME}/.platformio/penv/bin/pio"                              # PlatformIO
SWD_CFG = f"{HOME}/testing/pcb_bringup/pi_swd.cfg"                    # OpenOCD: SWD on Pi GPIO16 / GPIO20
RESET_CFG = f"{HOME}/testing/pcb_bringup/pi_srst.cfg"                 # OpenOCD: NRST on Pi GPIO21 (open-drain)
UART = "/dev/ttyAMA1"                                                 # Pi UART1 -> STM32 USART2 (pins 27/28)


# Step 1: stop the robot program, if it's running. Flashing restarts the STM32, and only one program can use it.
print("1. Stopping the robot program (if running)", flush=True)
subprocess.run(["pkill", "-INT", "-f", "nav/follow.py"])               # same as pressing Ctrl-C in its terminal
time.sleep(2)

# Step 2: build the firmware with PlatformIO (compiles src/main.c into firmware.elf).
print("2. Building the firmware", flush=True)
subprocess.run([PIO, "run"], cwd=FIRMWARE_DIR, check=True)            # check=True: stop here if the build fails

# Step 3: flash it with OpenOCD over SWD.
print("3. Flashing over SWD", flush=True)
subprocess.run([
    "openocd",
    "-f", SWD_CFG,                    # how the Pi's GPIO pins are wired to the STM32
    "-f", RESET_CFG,                  # which pin is the reset line
    "-f", "target/stm32f0x.cfg",      # the chip: STM32F0 family
    # The stock reset steps try to change the SWD clock speed, which the Pi's GPIO driver can't do: skip them.
    "-c", "stm32f0x.cpu configure -event reset-start {} -event reset-init {}",
    "-c", "init",                     # connect to the chip
    "-c", "reset halt",               # reset it and stop the CPU
    "-c", f"flash write_image erase {ELF}",   # erase and write the new program
    "-c", f"verify_image {ELF}",      # read it back and compare
    # Restart into the new program. A plain reset isn't enough on a chip that was ever blank (it stays in its
    # ROM bootloader), so trigger an option-byte reload, which restarts it properly (no settings are changed):
    "-c", "mww 0x40022004 0x45670123", "-c", "mww 0x40022004 0xCDEF89AB",   # unlock the flash controller
    "-c", "mww 0x40022008 0x45670123", "-c", "mww 0x40022008 0xCDEF89AB",   # unlock the option bytes
    "-c", "catch {mww 0x40022010 0x2200}",    # OBL_LAUNCH: reload option bytes = restart the chip
    "-c", "sleep 100",
    "-c", "shutdown",
], check=False)                       # OpenOCD reports an error when the chip restarts under it: that's expected

# Step 4: check the new firmware answers over the UART.
print("4. Checking the firmware answers", flush=True)
time.sleep(1.5)                                                       # give it time to boot
with serial.Serial(UART, 115200, timeout=0.5) as uart:
    uart.write(b"stop\nservo\n")                                      # motors to 0, ask for the latch status
    time.sleep(0.5)
    replies = [uart.readline().decode(errors="replace").strip() for _ in range(6)]
for line in replies:
    if line.startswith(("OK", "ERR")):
        print("   ", line)
if any(line.startswith("OK target") for line in replies):
    print("Done: the new firmware is running.")
else:
    print("No reply from the STM32: check the flash output above and the UART wiring (Pi pins 27/28 -> J13).")
