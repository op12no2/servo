# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A single-file C tool (`servo.c`): an interactive REPL for Waveshare SC09 bus servos over USB serial. It supports the SC09 only; don't add support for other servos (such as Feetech or Waveshare ST-series), and the SC09 isn't a rebadged Feetech. Use Waveshare's SC09 docs (wiki below) for register behaviour. It runs on a Raspberry Pi (Linux, termios). There are no tests and no dependencies.

## Hardware

- **Host:** Raspberry Pi 5, where Claude Code itself runs, so commands can be tried against the real servo.
- **Board:** Waveshare Bus Servo Adapter (A), sold by The Pi Hut as the "Serial Bus Servo Driver Board" ([wiki](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A)), [shop](https://thepihut.com/products/serial-bus-servo-driver-board)). It connects over USB through a CH343 USB-serial chip (`1a86:55d3`) and appears as `/dev/ttyACM0` (`/dev/serial/by-id/usb-1a86_USB_Single_Serial_*`). Its jumper must be in position **B** for USB control (A = UART). The board passes its DC input straight to the servos, so the supply voltage must match the servo's rating.
- **Servo:** Waveshare SC09 ([wiki](https://www.waveshare.com/wiki/SC09_Servo)). Rated input **4.8–8.4 V** (2S LiPo is fine). The Waveshare wiki says "4-6V", but 6 V is only the voltage its figures are optimised for and quoted at, not the maximum. 300° over 0..1023 (0.293°/step, centre 511); 2.3 kg·cm and 0.1 s/60° at 6 V; 38400 bps to 1 Mbps; factory default ID 1. The wiki links the protocol manual and memory table PDFs.
- **Observed on this bench:** servos at ids 2 and 3; dead zone (0x1A/0x1B) = 1; voltage limits 4.5..9.0 V (0x0F/0x0E); supply is a 2S LiPo, reading about 8.4–8.6 V at the servo.

## Build / run

```
make                               # cc -O2 -Wall -Wextra -o servo servo.c
./servo [/dev/ttyACM0] [baud]      # defaults: /dev/ttyACM0, 1000000
echo "scan" | ./servo              # non-tty stdin works for scripted commands (no prompt printed)
```

Keep the build warning-free under `-Wall -Wextra`. You can only really verify a change with hardware attached. If you can't test on a servo, say so.

## Architecture

- **Protocol layer:** `txrx()` builds a packet (`FF FF ID LEN INSTR PARAMS CHK`) and reads the status reply. The adapter is half-duplex and may echo TX back, so `txrx()` first discards an exact echo, then resyncs on the `FF FF` header. Broadcast (0xFE) writes other than ping return immediately without waiting for a reply.
- **Register helpers:** `read_regs` / `write_regs` / `read_u16` / `write_u16`. 16-bit registers are high byte first; convert through `get16()` / `put16()` rather than open-coding the byte order.
- **EPROM writes:** ID, angle limits and mode must be wrapped in unlock/lock via `REG_LOCK` (0x30): write 0, change the value, write 1. `setid`, `limits`, `motor` and `servomode` follow this pattern.
- **Command dispatch:** `main()` reads and tokenises lines (max 16 tokens) and calls `dispatch()`. For commands in `per_id_cmds`, `dispatch()` expands the `<id>` token with `parse_ids()` (`3`, `1-12`, `2,6`, `1-6,9`; no spaces) and calls `run()` once per id with that token replaced. Everything else goes straight to `run()`, an `if/else strcmp` chain that only ever sees a single id. To add a command, add a branch in `run()`, a line in `help()`, and add it to `per_id_cmds` if its first argument is an id.
- **`move` is the exception:** it's not in `per_id_cmds`. It parses its own id list so that several ids go out as one `SYNC_WRITE` (`sync_move()`, pos/time/speed, split into packets of 35 ids), so all the servos start together. It then blocks in `verify_move()`, polling pos and the moving flag (read together with pos, 0x38..0x42) every 20 ms. It reports ids that stop outside goal ± their dead zone, or are still moving after time + 3 s.
- **Argument parsing:** `arg(tok, i, nt, dflt, &ok)` parses with `strtol` base 0, so hex like `0x2A` works. It clears `ok` when an argument is missing, and commands use this to print their usage text.
- **Motor mode:** both angle limits are 0. Speed is written to the goal-time register (0x2C), with bit 10 as the direction bit (see `spin`).

An earlier "all" (scan-and-apply) option and a separate `sync` command were both removed on purpose. Id lists replace them, so don't reintroduce either unless asked. The repo is kept generic; hexapod-specific code will live in a separate copy.
