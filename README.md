# Servo

Interactive tester for a [Waveshare SC09](https://thepihut.com/products/serial-bus-servo-2-3kg) servo via a [Waveshare Serial Bus Servo Driver Board](https://thepihut.com/products/serial-bus-servo-driver-board) from a Raspberry Pi 5.

## Build

```
make
./servo [/dev/ttyACM0] [baud]      # default 1000000
```

## Use

Numbers accept hex (`0x2A`). `<id>` can be a list with no spaces, e.g. `3`, `1-12`, `2,6`, `1-6,9` (every command that takes one, except `setid`).

Several commands can go on one line, separated by `;`, and run in order when you press enter, e.g. `move 1 200; move 1 800 500; pos 1`. Each command finishes before the next starts (so a `move` waits until the servo stops). `quit` part-way through a line skips the rest.

At the prompt, up/down arrows step through the last 100 lines entered (a whole `;` line comes back as one entry), with the cursor at the end, so up + enter repeats the last line. Left/right, Home/End (or ctrl-a/ctrl-e), backspace, Delete and ctrl-u (clear line) edit it; ctrl-d on an empty line exits. History isn't saved between runs. Piped input (`echo "ping" | ./servo`) is read as plain lines.

| Command | Description |
|---|---|
| `ping [id]` | ping servo(s); no id pings all 0..253 |
| `pos <id>` | report position |
| `stat <id>` | pos/speed/load/volt/temp/moving, then mode: `servo <min>..<max>` (angle limits) or `motor` |
| `move <id> <pos> [time] [speed]` | goal pos 0..1023; time in ms (0 = asap); speed cap in steps/s (0 = full); an id list is sent as one sync write, so they all start together; warns if a goal is outside an id's angle limits (the servo clamps it); waits for the move to finish and reports ids off the (clamped) goal by more than their dead zone (0x1A/0x1B) |
| `torque <id> 0\|1` | torque enable |
| `rb <id> <addr>` / `rw <id> <addr>` | read byte / 16-bit |
| `wb <id> <addr> <val>` / `ww <id> <addr> <val>` | write byte / 16-bit |
| `dump <id>` | dump registers 0x00..0x45 |
| `setid <old> <new>` | change ID (EPROM) |
| `lock <id> 0\|1` | EPROM lock (0x30) |
| `limits <id> [min max]` | read/set angle limits |
| `motor <id>` | wheel mode (limits 0/0) |
| `spin <id> <speed>` | wheel speed -1000..1000 |
| `servomode <id> [min max]` | back to position mode (default 20..1003) |
| `raw <hexbytes...>` | send raw bytes, print reply |
| `verbose 0\|1` | hex dump packets |
| `timeout [ms]` | show/set reply timeout |
| `help`, `?` | list commands |
| `quit`, `q`, `exit` | exit |

## Links

- [SC09 Servo wiki](https://www.waveshare.com/wiki/SC09_Servo)
- [Bus Servo Adapter (A) wiki](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A))
