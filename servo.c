/*
 * servo.c - tiny interactive tester for Feetech SCS/STS bus servos
 * (e.g. SC09) via Waveshare Serial Bus Servo Driver Board / USB serial.
 *
 * Build:  make
 * Run:    ./servo [/dev/ttyACM0] [baud]
 * Then type commands on stdin ("help" lists them).
 *
 * Protocol (Feetech, Dynamixel-1.0-like):
 *   TX: FF FF ID LEN INSTR PARAM... CHK     LEN = nparams + 2
 *   RX: FF FF ID LEN ERR   PARAM... CHK
 *   CHK = ~(ID + LEN + INSTR/ERR + sum(PARAM)) & 0xFF
 * 16-bit registers: SCS series (SC09, SCS15...) = high byte first,
 *                   STS/SMS series              = low byte first.
 */
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/time.h>

#define INST_PING       0x01
#define INST_READ       0x02
#define INST_WRITE      0x03
#define INST_REG_WRITE  0x04
#define INST_ACTION     0x05
#define INST_RESET      0x06
#define INST_SYNC_WRITE 0x83
#define BROADCAST       0xFE

/* SCS register map (addresses shared with STS, meanings mostly same) */
#define REG_ID            0x05
#define REG_BAUD          0x06
#define REG_MIN_ANGLE     0x09  /* u16 */
#define REG_MAX_ANGLE     0x0B  /* u16 */
#define REG_TORQUE_ENABLE 0x28
#define REG_GOAL_POS      0x2A  /* u16 */
#define REG_GOAL_TIME     0x2C  /* u16 */
#define REG_GOAL_SPEED    0x2E  /* u16 */
#define REG_LOCK          0x30  /* EPROM lock: 1=locked, 0=unlocked */
#define REG_PRESENT_POS   0x38  /* u16 */
#define REG_PRESENT_SPEED 0x3A  /* u16 */
#define REG_PRESENT_LOAD  0x3C  /* u16 */
#define REG_VOLTAGE       0x3E
#define REG_TEMP          0x3F
#define REG_MOVING        0x42

static int fd = -1;
static int big_endian = 1;      /* 1 = SCS (SC09), 0 = STS/SMS */
static int verbose = 0;
static int timeout_ms = 50;

static void die(const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt); vfprintf(stderr, fmt, ap); va_end(ap);
    fputc('\n', stderr); exit(1);
}

static speed_t baud_const(int baud)
{
    switch (baud) {
    case 9600: return B9600;     case 19200: return B19200;
    case 38400: return B38400;   case 57600: return B57600;
    case 115200: return B115200; case 230400: return B230400;
    case 500000: return B500000; case 1000000: return B1000000;
    default: return 0;
    }
}

static void open_port(const char *dev, int baud)
{
    speed_t sp = baud_const(baud);
    if (!sp) die("unsupported baud %d", baud);
    fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) die("open %s: %s", dev, strerror(errno));
    struct termios t;
    if (tcgetattr(fd, &t) < 0) die("tcgetattr: %s", strerror(errno));
    cfmakeraw(&t);
    cfsetispeed(&t, sp); cfsetospeed(&t, sp);
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    t.c_cflag = (t.c_cflag & ~CSIZE) | CS8;
    t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &t) < 0) die("tcsetattr: %s", strerror(errno));
    tcflush(fd, TCIOFLUSH);
}

static void hexdump(const char *tag, const unsigned char *b, int n)
{
    printf("%s", tag);
    for (int i = 0; i < n; i++) printf(" %02X", b[i]);
    printf("\n");
}

/* read exactly n bytes or time out; returns bytes read */
static int read_bytes(unsigned char *buf, int n, int ms)
{
    int got = 0;
    while (got < n) {
        fd_set rf; FD_ZERO(&rf); FD_SET(fd, &rf);
        struct timeval tv = { ms / 1000, (ms % 1000) * 1000 };
        int r = select(fd + 1, &rf, NULL, NULL, &tv);
        if (r <= 0) break;
        r = read(fd, buf + got, n - got);
        if (r <= 0) break;
        got += r;
    }
    return got;
}

/*
 * Send instruction packet, receive status packet.
 * Returns number of params in reply (>=0), -1 on timeout/no reply,
 * -2 on bad checksum. *err gets the error byte. The adapter is half
 * duplex so we read back (and discard) our own TX echo first.
 */
static int txrx(int id, int instr, const unsigned char *p, int np,
                unsigned char *err, unsigned char *out, int outmax)
{
    unsigned char tx[260];
    int n = 0;
    tx[n++] = 0xFF; tx[n++] = 0xFF; tx[n++] = id; tx[n++] = np + 2; tx[n++] = instr;
    unsigned sum = id + (np + 2) + instr;
    for (int i = 0; i < np; i++) { tx[n++] = p[i]; sum += p[i]; }
    tx[n++] = ~sum & 0xFF;

    tcflush(fd, TCIFLUSH);
    if (write(fd, tx, n) != n) die("write: %s", strerror(errno));
    tcdrain(fd);
    if (verbose) hexdump("  tx:", tx, n);
    if (id == BROADCAST && instr != INST_PING) return 0;   /* no reply, don't wait for one */

    /* swallow echo if the adapter loops TX back to RX */
    unsigned char rx[64];
    int got = read_bytes(rx, n, timeout_ms);
    if (got == n && memcmp(rx, tx, n) == 0) {
        if (verbose) printf("  (echo)\n");
        got = 0;
    }

    /* find header */
    int hdr = 0;
    while (1) {
        if (got < 4) {
            int r = read_bytes(rx + got, 4 - got, timeout_ms);
            if (r <= 0) { if (verbose && got) hexdump("  rx(partial):", rx, got); return -1; }
            got += r;
        }
        for (hdr = 0; hdr + 1 < got; hdr++)
            if (rx[hdr] == 0xFF && rx[hdr + 1] == 0xFF && (hdr + 2 >= got || rx[hdr + 2] != 0xFF)) break;
        if (hdr + 3 < got) break;
        memmove(rx, rx + hdr, got - hdr); got -= hdr;
        if (got >= 4) { hdr = 0; break; }
    }
    if (hdr) { memmove(rx, rx + hdr, got - hdr); got -= hdr; }
    int len = rx[3];
    int total = 4 + len;
    if (total > (int)sizeof rx) return -2;
    if (got < total) got += read_bytes(rx + got, total - got, timeout_ms);
    if (got < total) { if (verbose) hexdump("  rx(short):", rx, got); return -1; }
    if (verbose) hexdump("  rx:", rx, total);
    unsigned s = 0;
    for (int i = 2; i < total - 1; i++) s += rx[i];
    if ((~s & 0xFF) != rx[total - 1]) { printf("  bad checksum\n"); return -2; }
    if (err) *err = rx[4];
    int npar = len - 2;
    for (int i = 0; i < npar && i < outmax; i++) out[i] = rx[5 + i];
    return npar;
}

static const char *errstr(unsigned char e)
{
    static char buf[128];
    if (!e) return "ok";
    buf[0] = 0;
    if (e & 0x01) strcat(buf, "voltage ");
    if (e & 0x02) strcat(buf, "angle ");
    if (e & 0x04) strcat(buf, "overheat ");
    if (e & 0x08) strcat(buf, "overcurrent ");   /* STS: overele */
    if (e & 0x20) strcat(buf, "overload ");
    return buf;
}

static int ping(int id)
{
    unsigned char e = 0, out[8];
    int r = txrx(id, INST_PING, NULL, 0, &e, out, sizeof out);
    if (r < 0) return r;
    unsigned char m[2]; int model = -1;
    if (txrx(id, INST_READ, (unsigned char[]){0x03, 2}, 2, &e, m, 2) == 2)
        model = big_endian ? (m[0] << 8 | m[1]) : (m[1] << 8 | m[0]);
    printf("id %d: alive, model %d (status %s)\n", id, model, errstr(e));
    return 0;
}

/*
 * Ping ids lo..hi with a short timeout, collect responders into ids[].
 * Returns number found.
 */
static int scan_ids(int lo, int hi, int *ids, int max)
{
    int save = timeout_ms, found = 0;
    timeout_ms = 15;
    for (int id = lo; id <= hi; id++) {
        int r = ping(id);
        if (r >= 0 && found < max) ids[found++] = id;
    }
    timeout_ms = save;
    return found;
}

static int read_regs(int id, int addr, int n, unsigned char *out)
{
    unsigned char p[2] = { addr, n }, e = 0;
    int r = txrx(id, INST_READ, p, 2, &e, out, n);
    if (r < 0) { printf("id %d: no reply\n", id); return -1; }
    if (e) printf("  status: %s\n", errstr(e));
    return r;
}

static int write_regs(int id, int addr, const unsigned char *v, int n)
{
    unsigned char p[32], e = 0, out[8];
    p[0] = addr; memcpy(p + 1, v, n);
    int r = txrx(id, INST_WRITE, p, n + 1, &e, out, sizeof out);
    if (id != BROADCAST && r < 0) { printf("id %d: no reply\n", id); return -1; }
    if (e) printf("  status: %s\n", errstr(e));
    return 0;
}

static int read_u16(int id, int addr, int *val)
{
    unsigned char b[2];
    if (read_regs(id, addr, 2, b) < 2) return -1;
    *val = big_endian ? (b[0] << 8 | b[1]) : (b[1] << 8 | b[0]);
    return 0;
}

static int write_u8(int id, int addr, int v)
{
    unsigned char b = v; return write_regs(id, addr, &b, 1);
}

static int write_u16(int id, int addr, int v)
{
    unsigned char b[2];
    if (big_endian) { b[0] = v >> 8; b[1] = v; } else { b[0] = v; b[1] = v >> 8; }
    return write_regs(id, addr, b, 2);
}

static void put16(unsigned char *b, int v)
{
    if (big_endian) { b[0] = v >> 8; b[1] = v; } else { b[0] = v; b[1] = v >> 8; }
}

static void help(void)
{
    puts(
    "ping <id>                 ping one servo\n"
    "scan [lo] [hi]            ping a range (default 0..253)\n"
    "pos <id>                  present position\n"
    "stat <id>                 pos/speed/load/volt/temp/moving\n"
    "move <id> <pos> [time] [speed]   goal pos (0..1023); time = ms to reach it (0 = asap);\n"
    "                          speed = max speed in steps/s, 0..1023 (0 = full speed, reg 0x2E); both default 0\n"
    "torque <id> 0|1           torque enable\n"
    "rb <id> <addr>            read byte\n"
    "rw <id> <addr>            read 16-bit\n"
    "wb <id> <addr> <val>      write byte\n"
    "ww <id> <addr> <val>      write 16-bit\n"
    "dump <id>                 dump registers 0x00..0x45\n"
    "setid <old> <new>         change ID (unlock EPROM, write, lock)\n"
    "lock <id> 0|1             EPROM lock register (0x30)\n"
    "limits <id> [min max]     read/set angle limits (0x09/0x0B)\n"
    "motor <id>                enter motor/wheel mode (angle limits -> 0/0, EPROM)\n"
    "spin <id> <speed>         motor mode speed -1000..1000 (0 = stop)\n"
    "servomode <id> [min max]  back to position mode (limits default 20..1003)\n"
    "sync <pos> <id>...        sync-write same goal pos to many ids\n"
    "raw <hexbytes...>         send raw bytes, print reply\n"
    "endian big|little         16-bit byte order (big=SCS/SC09, little=STS)\n"
    "verbose 0|1               hex dump packets\n"
    "timeout <ms>              reply timeout\n"
    "help | quit");
}

static int arg(char **tok, int i, int ntok, int dflt, int *ok)
{
    if (i >= ntok) { if (ok) *ok = 0; return dflt; }
    return (int)strtol(tok[i], NULL, 0);
}

/* run one tokenised command; returns 1 on quit */
static int run(char **tok, int nt)
{
    const char *c = tok[0];
    int ok = 1;

    if (!strcmp(c, "help") || !strcmp(c, "?")) help();
    else if (!strcmp(c, "quit") || !strcmp(c, "q") || !strcmp(c, "exit")) return 1;
    else if (!strcmp(c, "verbose")) verbose = arg(tok, 1, nt, 1, NULL);
    else if (!strcmp(c, "timeout")) timeout_ms = arg(tok, 1, nt, 50, NULL);
    else if (!strcmp(c, "endian")) {
        if (nt > 1) big_endian = !strcmp(tok[1], "big");
        printf("%s-endian\n", big_endian ? "big" : "little");
    }
    else if (!strcmp(c, "ping")) {
        int id = arg(tok, 1, nt, 1, NULL);
        if (ping(id) < 0) printf("id %d: no reply\n", id);
    }
    else if (!strcmp(c, "scan")) {
        int lo = arg(tok, 1, nt, 0, NULL), hi = arg(tok, 2, nt, 253, NULL), ids[254];
        printf("%d servo(s) found\n", scan_ids(lo, hi, ids, 254));
    }
    else if (!strcmp(c, "pos")) {
        int id = arg(tok, 1, nt, 1, NULL), v;
        if (read_u16(id, REG_PRESENT_POS, &v) == 0) printf("id %d pos %d\n", id, v);
    }
    else if (!strcmp(c, "stat")) {
        int id = arg(tok, 1, nt, 1, NULL);
        unsigned char b[11];
        if (read_regs(id, REG_PRESENT_POS, 11, b) == 11) {
            int p = big_endian ? b[0]<<8|b[1] : b[1]<<8|b[0];
            int s = big_endian ? b[2]<<8|b[3] : b[3]<<8|b[2];
            int l = big_endian ? b[4]<<8|b[5] : b[5]<<8|b[4];
            printf("id %d pos %d speed %d load %d volt %.1fV temp %dC moving %d\n",
                   id, p, s, l, b[6] / 10.0, b[7], b[10]);
        }
    }
    else if (!strcmp(c, "move")) {
        int id = arg(tok, 1, nt, 1, &ok), pos = arg(tok, 2, nt, 512, &ok);
        int tm = arg(tok, 3, nt, 0, NULL), sp = arg(tok, 4, nt, 0, NULL);
        if (!ok) { puts("usage: move <id> <pos> [time] [speed]"); return 0; }
        if (pos < 0 || pos > 1023) printf("warning: pos %d outside 0..1023, servo will clamp to its limits\n", pos);
        unsigned char b[6]; put16(b, pos); put16(b + 2, tm); put16(b + 4, sp);
        write_regs(id, REG_GOAL_POS, b, 6);
    }
    else if (!strcmp(c, "torque")) write_u8(arg(tok, 1, nt, 1, NULL), REG_TORQUE_ENABLE, arg(tok, 2, nt, 1, NULL));
    else if (!strcmp(c, "rb")) {
        int id = arg(tok, 1, nt, 1, &ok), a = arg(tok, 2, nt, 0, &ok); unsigned char b;
        if (!ok) { puts("usage: rb <id> <addr>"); return 0; }
        if (read_regs(id, a, 1, &b) == 1) printf("id %d [0x%02X] = %d (0x%02X)\n", id, a, b, b);
    }
    else if (!strcmp(c, "rw")) {
        int id = arg(tok, 1, nt, 1, &ok), a = arg(tok, 2, nt, 0, &ok), v;
        if (!ok) { puts("usage: rw <id> <addr>"); return 0; }
        if (read_u16(id, a, &v) == 0) printf("id %d [0x%02X] = %d (0x%04X)\n", id, a, v, v);
    }
    else if (!strcmp(c, "wb")) {
        int id = arg(tok, 1, nt, 1, &ok), a = arg(tok, 2, nt, 0, &ok), v = arg(tok, 3, nt, 0, &ok);
        if (!ok) { puts("usage: wb <id> <addr> <val>"); return 0; }
        write_u8(id, a, v);
    }
    else if (!strcmp(c, "ww")) {
        int id = arg(tok, 1, nt, 1, &ok), a = arg(tok, 2, nt, 0, &ok), v = arg(tok, 3, nt, 0, &ok);
        if (!ok) { puts("usage: ww <id> <addr> <val>"); return 0; }
        write_u16(id, a, v);
    }
    else if (!strcmp(c, "dump")) {
        int id = arg(tok, 1, nt, 1, NULL);
        unsigned char b[0x46];
        int n = 0;
        for (int a = 0; a < 0x46 && n >= 0; a += 8) {
            int k = a + 8 > 0x46 ? 0x46 - a : 8;
            if (read_regs(id, a, k, b + a) != k) { n = -1; break; }
        }
        if (n >= 0) printf("id %d:\n", id);
        if (n >= 0) for (int a = 0; a < 0x46; a++) {
            if (a % 8 == 0) printf("%02X:", a);
            printf(" %02X", b[a]);
            if (a % 8 == 7 || a == 0x45) putchar('\n');
        }
    }
    else if (!strcmp(c, "setid")) {
        int o = arg(tok, 1, nt, 1, &ok), n = arg(tok, 2, nt, 1, &ok);
        if (!ok || n < 0 || n > 253) { puts("usage: setid <old> <new>   (new 0..253)"); return 0; }
        if (write_u8(o, REG_LOCK, 0)) return 0;
        if (write_u8(o, REG_ID, n)) return 0;
        write_u8(n, REG_LOCK, 1);
        if (ping(n) == 0) printf("id changed %d -> %d\n", o, n);
    }
    else if (!strcmp(c, "lock")) write_u8(arg(tok, 1, nt, 1, NULL), REG_LOCK, arg(tok, 2, nt, 1, NULL));
    else if (!strcmp(c, "limits")) {
        int id = arg(tok, 1, nt, 1, NULL), mn, mx;
        if (nt >= 4) {
            write_u8(id, REG_LOCK, 0);
            write_u16(id, REG_MIN_ANGLE, arg(tok, 2, nt, 0, NULL));
            write_u16(id, REG_MAX_ANGLE, arg(tok, 3, nt, 1023, NULL));
            write_u8(id, REG_LOCK, 1);
        }
        if (read_u16(id, REG_MIN_ANGLE, &mn) == 0 && read_u16(id, REG_MAX_ANGLE, &mx) == 0)
            printf("id %d limits %d..%d\n", id, mn, mx);
    }
    else if (!strcmp(c, "motor")) {            /* SC "PWM/wheel" mode: both angle limits = 0 */
        int id = arg(tok, 1, nt, 1, NULL);
        unsigned char z[4] = {0, 0, 0, 0};
        write_u8(id, REG_LOCK, 0);
        write_regs(id, REG_MIN_ANGLE, z, 4);
        write_u8(id, REG_LOCK, 1);
        int m = 0; unsigned char mb;
        if (read_regs(id, 0x21, 1, &mb) == 1) m = mb;
        printf("id %d: motor mode (limits 0/0, mode reg 0x21 = %d). 'spin %d <-1000..1000>'\n", id, m, id);
    }
    else if (!strcmp(c, "spin")) {             /* speed via goal-time reg, bit 10 = reverse */
        int id = arg(tok, 1, nt, 1, &ok), sp = arg(tok, 2, nt, 0, &ok);
        if (!ok) { puts("usage: spin <id> <speed -1000..1000>"); return 0; }
        int v = sp < 0 ? (-sp & 0x3FF) | 0x400 : (sp & 0x3FF);
        write_u16(id, REG_GOAL_TIME, v);
    }
    else if (!strcmp(c, "servomode")) {        /* back to position mode: limits 20..1003 */
        int id = arg(tok, 1, nt, 1, NULL);
        write_u16(id, REG_GOAL_TIME, 0);
        write_u8(id, REG_LOCK, 0);
        write_u8(id, 0x21, 0);
        write_u16(id, REG_MIN_ANGLE, arg(tok, 2, nt, 20, NULL));
        write_u16(id, REG_MAX_ANGLE, arg(tok, 3, nt, 1003, NULL));
        write_u8(id, REG_LOCK, 1);
        int mn, mx;
        if (read_u16(id, REG_MIN_ANGLE, &mn) == 0 && read_u16(id, REG_MAX_ANGLE, &mx) == 0)
            printf("id %d: servo mode, limits %d..%d\n", id, mn, mx);
    }
    else if (!strcmp(c, "sync")) {
        if (nt < 3) { puts("usage: sync <pos> <id>..."); return 0; }
        int pos = arg(tok, 1, nt, 512, NULL);
        int ids[254], nid = 0;
        for (int i = 2; i < nt; i++) ids[nid++] = arg(tok, i, nt, 1, NULL);
        if (nid > 83) nid = 83;                 /* LEN byte limit: 2 + 3*n + 2 <= 255 */
        unsigned char p[256]; int n = 0;
        p[n++] = REG_GOAL_POS; p[n++] = 2;
        for (int i = 0; i < nid; i++) { p[n++] = ids[i]; put16(p + n, pos); n += 2; }
        txrx(BROADCAST, INST_SYNC_WRITE, p, n, NULL, NULL, 0);
        printf("sync pos %d -> id(s):", pos);
        for (int i = 0; i < nid; i++) printf(" %d", ids[i]);
        putchar('\n');
    }
    else if (!strcmp(c, "raw")) {
        unsigned char b[64]; int n = 0;
        for (int i = 1; i < nt && n < 64; i++) b[n++] = strtol(tok[i], NULL, 16);
        tcflush(fd, TCIFLUSH);
        if (write(fd, b, n) != n) perror("write");
        tcdrain(fd);
        hexdump("tx:", b, n);
        unsigned char r[64]; int got = read_bytes(r, sizeof r, timeout_ms);
        hexdump("rx:", r, got);
    }
    else printf("unknown command '%s' (help)\n", c);
    return 0;
}

int main(int argc, char **argv)
{
    const char *dev = argc > 1 ? argv[1] : "/dev/ttyACM0";
    int baud = argc > 2 ? atoi(argv[2]) : 1000000;
    open_port(dev, baud);
    printf("opened %s @ %d, %s-endian 16-bit (type help)\n", dev, baud,
           big_endian ? "big" : "little");

    char line[256];
    int interactive = isatty(0);
    for (;;) {
        if (interactive) { fputs("> ", stdout); fflush(stdout); }
        if (!fgets(line, sizeof line, stdin)) break;
        char *tok[16]; int nt = 0;
        for (char *s = strtok(line, " \t\r\n"); s && nt < 16; s = strtok(NULL, " \t\r\n")) tok[nt++] = s;
        if (!nt) continue;
        if (run(tok, nt)) break;
    }
    close(fd);
    return 0;
}
