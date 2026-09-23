# Servo

Interactive tester for a Waveshare SC09 via a [Waveshare Serial Bus Servo Driver Board](https://thepihut.com/products/serial-bus-servo-driver-board) from a Raspberry Pi 5.

## Build

```
make
./servo [/dev/ttyACM0] [baud]      # default 1000000
```

## Use

Numbers accept hex (`0x2A`). `<id>` can be a list with no spaces, e.g. `3`, `1-12`, `2,6`, `1-6,9` (all commands except `setid`).

| Command | Description |
|---|---|
| `ping <id>` | ping one servo |
| `scan [lo] [hi]` | ping a range (default 0..253) |
| `pos <id>` | present position |
| `stat <id>` | pos/speed/load/volt/temp/moving |
| `move <id> <pos> [time] [speed]` | goal pos 0..1023; time in ms (0 = asap); speed cap in steps/s (0 = full); an id list is sent as one sync write, so they all start together; waits for the move to finish and reports ids off goal by more than their dead zone (0x1A/0x1B) |
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
| `timeout <ms>` | reply timeout |
| `help`, `?` | list commands |
| `quit`, `q`, `exit` | exit |

## Links

- [SC09 Servo wiki](https://www.waveshare.com/wiki/SC09_Servo)
- [Bus Servo Adapter (A) wiki](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A))
