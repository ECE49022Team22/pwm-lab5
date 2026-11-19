#!/bin/bash
# Build the motor firmware and flash it onto the team PCB's STM32, from the Raspberry Pi.
#   bash ~/pwm-lab5/flash_pcb.sh
# There is no ST-LINK: the Pi talks to the STM32's debug port (SWD) through its own GPIO pins
# (pin 36 = SWDIO, pin 38 = SWCLK, pin 40 = reset, GND on pin 34), driven by OpenOCD.

set -e   # stop at the first command that fails

echo "1. Stopping the robot program (if running)"
pkill -INT -f nav/follow.py || true      # same as Ctrl-C; "|| true" = fine if it wasn't running
sleep 2

echo "2. Building the firmware"
cd ~/pwm-lab5
~/.platformio/penv/bin/pio run           # compiles src/main.c into .pio/build/nucleo_f091rc/firmware.elf

echo "3. Flashing over SWD"
# - pi_swd.cfg / pi_srst.cfg: which Pi GPIO pins are wired to the STM32's SWD and reset pins
# - target/stm32f0x.cfg: the chip (STM32F0 family)
# - "configure -event ...": skip the stock reset steps (they change the SWD speed, which the Pi's GPIO can't)
# - reset halt, write, verify: erase and write the program, then read it back and compare
# - the mww lines: unlock the flash controller and trigger an option-byte reload, which restarts the chip
#   into the new program (a plain reset leaves a once-blank chip in its ROM bootloader). No settings change.
# OpenOCD prints an error at the end when the chip restarts under it: that's expected ("|| true").
openocd -f ~/testing/pcb_bringup/pi_swd.cfg -f ~/testing/pcb_bringup/pi_srst.cfg -f target/stm32f0x.cfg \
  -c "stm32f0x.cpu configure -event reset-start {} -event reset-init {}" \
  -c "init" \
  -c "reset halt" \
  -c "flash write_image erase .pio/build/nucleo_f091rc/firmware.elf" \
  -c "verify_image .pio/build/nucleo_f091rc/firmware.elf" \
  -c "mww 0x40022004 0x45670123" -c "mww 0x40022004 0xCDEF89AB" \
  -c "mww 0x40022008 0x45670123" -c "mww 0x40022008 0xCDEF89AB" \
  -c "catch {mww 0x40022010 0x2200}" \
  -c "sleep 100" \
  -c "shutdown" || true

echo "4. Checking the firmware answers (expect 'OK target 0 RPM' and 'OK servo ...')"
sleep 1.5                                                  # let it boot
stty -F /dev/ttyAMA1 115200 raw -echo                      # Pi UART1 -> STM32, 115200 baud
timeout 1.5 grep -a --line-buffered "^OK" /dev/ttyAMA1 &   # listen for the replies...
sleep 0.3
printf 'stop\nservo\n' > /dev/ttyAMA1                      # ...to: motors to 0, latch status
wait || true
echo "Done."
