# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A single-file C tool (`servo.c`): an interactive REPL for Waveshare SC09 bus servos over USB serial. It supports the SC09 only; don't add support for other servos (such as Feetech or Waveshare ST-series), and the SC09 isn't a rebadged Feetech. Use Waveshare's SC09 docs (wiki below) for register behaviour. It runs on a Raspberry Pi (Linux, termios). There are no tests and no dependencies.

## Hardware

- **Host:** Raspberry Pi 5, where Claude Code itself runs, so commands can be tried against the real servo.
- **Board:** Waveshare Bus Servo Adapter (A), sold by The Pi Hut as the "Serial Bus Servo Driver Board" ([wiki](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A)), [shop](https://thepihut.com/products/serial-bus-servo-driver-board)). It connects over USB through a CH343 USB-serial chip (`1a86:55d3`) and appears as `/dev/ttyACM0` (`/dev/serial/by-id/usb-1a86_USB_Single_Serial_*`). Its jumper must be in position **B** for USB control (A = UART). The board passes its DC input straight to the servos, so the supply voltage must match the servo's rating.
- **Servo:** Waveshare SC09 ([wiki](https://www.waveshare.com/wiki/SC09_Servo)). Rated input **4.8–8.4 V** (2S LiPo is fine). The Waveshare wiki says "4-6V", but 6 V is only the voltage its figures are optimised for and quoted at, not the maximum. 300° over 0..1023 (0.293°/step, centre 511); 2.3 kg·cm and 0.1 s/60° at 6 V; 38400 bps to 1 Mbps; factory default ID 1. The wiki links the protocol manual (PDF) and the SCS-series memory table (XLS).
- **Observed on this bench:** dead zone (0x1A/0x1B) = 1; voltage limits 4.5..9.0 V (0x0F/0x0E); supply is a 2S LiPo, reading about 8.4–8.6 V at the servo.
- **Stopping short under friction:** the earlier 1–6-step misses (short on the approach side) happened while the servo had tipped over with its axle rubbing on the desk. Unloaded, 12 back-and-forth moves at time 500 all settled within ±2 (mostly ±1) with no creep a second later. Settings are unchanged: P = 15 (0x15), D = 15 (0x16), **I = 0** (0x17), minimum starting force 45 = 4.5% (0x18, u16). If it stops short under real load, try (EPROM writes, so unlock/lock) a small I (e.g. 2) or a higher minimum starting force (e.g. ~80). Separately, `verify_move` once reported "stopped" too early (moving flag read 0 for 100 ms, then the servo crept from 475 to 482); not yet rechecked without the friction.
- **Angle limits clamp goals:** servo 1's limits are 20..1003 (the `servomode` default), so `move 1 0` goes to 20; `move` warns about this and checks the servo against the clamped goal.
- **Device path:** the adapter sometimes re-enumerates as `/dev/ttyACM1`; the `/dev/serial/by-id/` path is stable.

## Build / run

```
make                               # cc -O2 -Wall -Wextra -o servo servo.c
./servo [/dev/ttyACM0] [baud]      # defaults: /dev/ttyACM0, 1000000
echo "ping" | ./servo              # non-tty stdin works for scripted commands (no prompt printed)
```

Keep the build warning-free under `-Wall -Wextra`. You can only really verify a change with hardware attached. If you can't test on a servo, say so.

## Git

Commit and push to `origin main` whenever a change is done and tested; no need to ask first. Keep README, `help()` and this file in step with the code in the same commit.

## Architecture

- **Protocol layer:** `txrx()` builds a packet (`FF FF ID LEN INSTR PARAMS CHK`) and reads the status reply. The adapter is half-duplex and may echo TX back, so `txrx()` first discards an exact echo, then resyncs on the `FF FF` header. Broadcast (0xFE) writes other than ping return immediately without waiting for a reply.
- **Register helpers:** `read_regs` / `write_regs` / `read_u16` / `write_u16`. 16-bit registers are high byte first; convert through `get16()` / `put16()` rather than open-coding the byte order.
- **EPROM writes:** ID, angle limits and mode must be wrapped in unlock/lock via `REG_LOCK` (0x30): write 0, change the value, write 1. `setid`, `limits`, `motor` and `servomode` follow this pattern.
- **Command dispatch:** `main()` reads lines (via `edit_line()`, a small termios line editor with arrow-key history, when stdin is a tty; plain `fgets` otherwise), splits each on `;` into commands, tokenises each (max 16 tokens) and calls `dispatch()` in order; a command returning 1 (quit) skips the rest. For commands in `per_id_cmds`, `dispatch()` expands the `<id>` token with `parse_ids()` (`3`, `1-12`, `2,6`, `1-6,9`; no spaces) and calls `run()` once per id with that token replaced. Everything else goes straight to `run()`, an `if/else strcmp` chain that only ever sees a single id. To add a command, add a branch in `run()`, a line in `help()`, and add it to `per_id_cmds` if its first argument is an id.
- **`move` is the exception:** it's not in `per_id_cmds`. It parses its own id list so that several ids go out as one `SYNC_WRITE` (`sync_move()`, pos/time/speed, split into packets of 35 ids), so all the servos start together. It then blocks in `verify_move()`, which first reads each id's angle limits and dead zone (one read, 0x09..0x1B) and warns if the goal is clamped by the limits, then polls pos and the moving flag (read together with pos, 0x38..0x42) every 20 ms. It reports ids that stop outside goal ± their dead zone, or are still moving after time + 3 s.
- **Argument parsing:** `arg(tok, i, nt, dflt, &ok)` parses with `strtol` base 0, so hex like `0x2A` works. It clears `ok` when an argument is missing, and commands use this to print their usage text.
- **Motor mode:** both angle limits are 0. Speed is written to the goal-time register (0x2C), with bit 10 as the direction bit (see `spin`).

An earlier "all" (scan-and-apply) option, a separate `sync` command and a separate `scan` command (now bare `ping`) were all removed on purpose. Id lists replace them, so don't reintroduce any of them unless asked. The repo is kept generic; hexapod-specific code will live in a separate copy.
