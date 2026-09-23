# Servo

Interactive tester for a Waveshare SC09 via a [Waveshare Serial Bus Servo Driver Board](https://thepihut.com/products/serial-bus-servo-driver-board) from a Raspberry Pi 5. Probably also works for Feetech SCS/STS bus servos in general.

## Build

```
make
./servo [/dev/ttyACM0] [baud]      # default 1000000
```

## Use

Numbers accept hex (`0x2A`).

| Command | Description |
|---|---|
| `ping <id>` | ping one servo |
| `scan [lo] [hi]` | ping a range (default 0..253) |
| `pos <id>` | present position |
| `stat <id>` | pos/speed/load/volt/temp/moving |
| `move <id> <pos> [time] [speed]` | goal pos 0..1023; time in ms (0 = asap); speed cap in steps/s (0 = full) |
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
| `sync <pos> <id>...` | same goal pos to many ids at once |
| `raw <hexbytes...>` | send raw bytes, print reply |
| `endian big\|little` | 16-bit byte order (big = SCS, little = STS) |
| `verbose 0\|1` | hex dump packets |
| `timeout <ms>` | reply timeout |
| `help`, `?` | list commands |
| `quit`, `q`, `exit` | exit |
