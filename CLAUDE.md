# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A single-file C tool (`servo.c`): an interactive REPL for poking Feetech SCS/STS bus servos over USB serial. The servo actually in use is a Waveshare SC09, which is almost certainly a rebadged Feetech SCS0009: SCS protocol, big-endian 16-bit registers (the default here), 1 Mbps default baud, position range 0..1023. When looking up register behaviour, use the Feetech SCS-series docs (SCS0009/SCS15), not the STS-series docs. It runs on a Raspberry Pi (Linux, termios). There are no tests and no dependencies.

## Hardware

- **Host:** Raspberry Pi 5, where Claude Code itself runs, so commands can be tried against the real servo.
- **Board:** Waveshare Serial Bus Servo Driver Board (https://thepihut.com/products/serial-bus-servo-driver-board). It connects over USB through a CH343 USB-serial chip (`1a86:55d3`) and appears as `/dev/ttyACM0` (`/dev/serial/by-id/usb-1a86_USB_Single_Serial_*`).
- **Servo:** Waveshare SC09 (see above).

## Build / run

```
make                               # cc -O2 -Wall -Wextra -o servo servo.c
./servo [/dev/ttyACM0] [baud]      # defaults: /dev/ttyACM0, 1000000
echo "scan" | ./servo              # non-tty stdin works for scripted commands (no prompt printed)
```

Keep the build warning-free under `-Wall -Wextra`. You can only really verify a change with hardware attached. If you can't test on a servo, say so.

## Architecture

- **Protocol layer:** `txrx()` builds a Feetech packet (`FF FF ID LEN INSTR PARAMS CHK`) and reads the status reply. The adapter is half-duplex and may echo TX back, so `txrx()` first discards an exact echo, then resyncs on the `FF FF` header. Broadcast (0xFE) writes other than ping return immediately without waiting for a reply.
- **Register helpers:** `read_regs` / `write_regs` / `read_u16` / `write_u16` / `put16`. All 16-bit values go through the global `big_endian` flag. It's big-endian for the SCS series (the default) and little-endian for STS/SMS, and the `endian` command toggles it. Any new 16-bit register access must respect this flag.
- **EPROM writes:** ID, angle limits and mode must be wrapped in unlock/lock via `REG_LOCK` (0x30): write 0, change the value, write 1. `setid`, `limits`, `motor` and `servomode` follow this pattern.
- **Command dispatch:** `run()` takes a tokenised line and matches it with an `if/else strcmp` chain. `main()` only opens the port, reads lines, tokenises them (max 16 tokens) and calls `run()`. To add a command, add a branch in `run()` and a line in `help()`.
- **Argument parsing:** `arg(tok, i, nt, dflt, &ok)` parses with `strtol` base 0, so hex like `0x2A` works. It clears `ok` when an argument is missing, and commands use this to print their usage text.
- **Motor mode (SC series):** both angle limits are 0. Speed is written to the goal-time register (0x2C), with bit 10 as the direction bit (see `spin`).

Commands take explicit servo ids only. An earlier "all" (scan-and-apply) option was removed on purpose; don't reintroduce it unless asked.
