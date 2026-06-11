#define _GNU_SOURCE
#include "lens_bus.h"
#include "lens_proto.h"

#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <sched.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int sig) {
    (void)sig;
    g_stop = 1;
}

static void usage(FILE *f) {
    fprintf(f,
        "Canon EF-like lens GPIO bit-bang test tool\n"
        "\n"
        "Default pins, BCM numbering:\n"
        "  MOSI/DCL GPIO17  physical pin 11\n"
        "  MISO/DLC GPIO27  physical pin 13\n"
        "  CLK      GPIO22  physical pin 15, open-drain by direction switching\n"
        "\n"
        "Usage:\n"
        "  lensctl [options] <command> [args...]\n"
        "\n"
        "Options:\n"
        "  --mosi N              BCM GPIO for DCL/MOSI, default 17\n"
        "  --miso N              BCM GPIO for DLC/MISO, default 27\n"
        "  --clk N               BCM GPIO for CLK, default 22\n"
        "  --slow-ns N           slow full clock period, default 12500 ns\n"
        "  --fast-ns N           fast full clock period, default 2000 ns\n"
        "  --sample-delay-ns N   delay after CLK rising before MISO sample, default 0\n"
        "  --mosi-setup-ns N     MOSI setup before falling edge; 0=auto mid-high\n"
        "  --legacy-mosi-after-fall\n"
        "                        old diagnostic mode: change MOSI just after CLK falling\n"
        "  --timeout-us N        timeout waiting for CLK high, default 100000 us\n"
        "  --no-clk-pullup       do not enable Pi's weak internal pull-up on CLK\n"
        "  --no-wait-clk-high    do not wait when lens holds CLK low\n"
        "  --no-rt               do not request SCHED_FIFO/mlockall\n"
        "  -h, --help            show this help\n"
        "\n"
        "Commands:\n"
        "  ready                 slow 0x0A/0x00 ready-alive sequence\n"
        "  init                  captured init/probe sequence, prints replies\n"
        "  status                fast 0x90 0x00 0x00\n"
        "  focus-read            fast 0xC0 0x00 0x00\n"
        "  focus-set VALUE       fast 0x44 hi lo, VALUE 0..4234\n"
        "  focus-inc CURRENT D   set clamp(CURRENT + D, 0..4234)\n"
        "  focus-endpoint        fast 0x06 raw endpoint command seen in capture\n"
        "  aperture-info         fast 0xB0 0x00 0x00 0x00\n"
        "  aperture-reopen       fast 0x13 0x80\n"
        "  aperture-move CODE    fast 0x13 CODE, CODE decimal or hex, e.g. 0x0E\n"
        "  aperture-set-code C   reopen, status poll, then 0x13 C\n"
        "  focal-length          fast 0xA0 0x00 0x00\n"
        "  name                  fast 0x82 then repeated 0x83 reads\n"
        "  xfer-slow BYTES...    raw slow transfer, prints RX\n"
        "  xfer-fast BYTES...    raw fast transfer, prints RX\n"
        "  clk-release           release CLK to input/high-Z and print level\n"
        "  clk-low               pull CLK low until the process exits; Ctrl+C releases\n"
        "\n"
        "Examples:\n"
        "  sudo ./lensctl ready\n"
        "  sudo ./lensctl xfer-fast 0x13 0x80\n"
        "  sudo ./lensctl focus-set 1000\n"
        "  sudo ./lensctl aperture-set-code 0x1A\n");
}

static int parse_u32(const char *s, uint32_t *out) {
    if (!s || !*s || !out) return -EINVAL;
    char *end = NULL;
    errno = 0;
    unsigned long v = strtoul(s, &end, 0);
    if (errno || end == s || *end != '\0' || v > UINT32_MAX) return -EINVAL;
    *out = (uint32_t)v;
    return 0;
}

static int parse_u16(const char *s, uint16_t *out) {
    uint32_t v;
    int rc = parse_u32(s, &v);
    if (rc < 0 || v > UINT16_MAX) return -EINVAL;
    *out = (uint16_t)v;
    return 0;
}

static int parse_i32(const char *s, int *out) {
    if (!s || !*s || !out) return -EINVAL;
    char *end = NULL;
    errno = 0;
    long v = strtol(s, &end, 0);
    if (errno || end == s || *end != '\0' || v < INT_MIN || v > INT_MAX) return -EINVAL;
    *out = (int)v;
    return 0;
}

static int parse_u8(const char *s, uint8_t *out) {
    uint32_t v;
    int rc = parse_u32(s, &v);
    if (rc < 0 || v > 0xFFu) return -EINVAL;
    *out = (uint8_t)v;
    return 0;
}

static void print_bytes(const char *label, const uint8_t *b, size_t n) {
    printf("%s", label);
    for (size_t i = 0; i < n; ++i) printf(" 0x%02X", b[i]);
    printf("\n");
}

static int enable_realtime(void) {
    int ok = 0;
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        fprintf(stderr, "warning: mlockall failed: %s\n", strerror(errno));
        ok = -1;
    }

    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        fprintf(stderr, "warning: sched_setscheduler(SCHED_FIFO) failed: %s\n", strerror(errno));
        ok = -1;
    }

    if (setpriority(PRIO_PROCESS, 0, -20) != 0) {
        fprintf(stderr, "warning: setpriority failed: %s\n", strerror(errno));
        ok = -1;
    }
    return ok;
}

static int raw_xfer(lens_bus_t *bus, lens_speed_t speed, int argc, char **argv) {
    if (argc <= 0) return -EINVAL;
    uint8_t tx[256];
    uint8_t rx[256];
    if (argc > (int)sizeof(tx)) return -EINVAL;
    for (int i = 0; i < argc; ++i) {
        int rc = parse_u8(argv[i], &tx[i]);
        if (rc < 0) return rc;
    }
    int rc = lens_bus_transfer(bus, speed, tx, rx, (size_t)argc);
    if (rc < 0) return rc;
    print_bytes("TX:", tx, (size_t)argc);
    print_bytes("RX:", rx, (size_t)argc);
    return 0;
}

static int do_init(lens_bus_t *bus) {
    int rc;
    uint8_t rx4[4];
    lens_basic_info_t bi;
    char name[64];
    uint8_t raw_name[18];
    uint8_t rx3[3];
    uint8_t rx5[5];
    lens_aperture_info_t ai;
    uint16_t v;

    printf("ready/alive slow sequence...\n");
    rc = lens_ready_alive(bus, rx4);
    if (rc < 0) return rc;
    print_bytes("RX:", rx4, sizeof(rx4));

    printf("basic numeric lens info...\n");
    rc = lens_read_basic_info(bus, &bi);
    if (rc < 0) return rc;
    print_bytes("RAW:", bi.raw, sizeof(bi.raw));
    printf("type=0x%02X lens_id=0x%02X max_focal=%u min_focal=%u c1=0x%02X c2=0x%02X\n",
           bi.type, bi.lens_id, bi.max_focal_length, bi.min_focal_length, bi.c1, bi.c2);

    printf("three fast dummy clocks from capture...\n");
    const uint8_t dummy3[3] = {0x00, 0x00, 0x00};
    rc = lens_bus_transfer(bus, LENS_SPEED_FAST, dummy3, rx3, sizeof(dummy3));
    if (rc < 0) return rc;
    print_bytes("RX:", rx3, sizeof(rx3));

    printf("lens name...\n");
    rc = lens_read_name(bus, name, sizeof(name), raw_name, sizeof(raw_name));
    if (rc < 0) return rc;
    print_bytes("RAW:", raw_name, sizeof(raw_name));
    printf("name='%s'\n", name);

    printf("focus position...\n");
    rc = lens_read_focus_position(bus, &v, rx3);
    if (rc < 0) return rc;
    print_bytes("RX:", rx3, sizeof(rx3));
    printf("focus_raw=%u / 0x%04X\n", v, v);

    printf("C2 unknown 4-byte reply...\n");
    rc = lens_read_c2_unknown(bus, rx5);
    if (rc < 0) return rc;
    print_bytes("RX:", rx5, sizeof(rx5));

    printf("focal length...\n");
    rc = lens_read_focal_length(bus, &v, rx3);
    if (rc < 0) return rc;
    print_bytes("RX:", rx3, sizeof(rx3));
    printf("focal_length=%u\n", v);

    printf("aperture info...\n");
    rc = lens_read_aperture_info(bus, &ai);
    if (rc < 0) return rc;
    print_bytes("RX:", ai.raw, sizeof(ai.raw));
    printf("aperture raw max/current/min = 0x%02X 0x%02X 0x%02X\n",
           ai.max_aperture, ai.current_aperture, ai.min_aperture);

    printf("captured 0x50 0x6E tail...\n");
    const uint8_t tail[2] = {0x50, 0x6E};
    uint8_t tail_rx[2];
    rc = lens_bus_transfer(bus, LENS_SPEED_FAST, tail, tail_rx, sizeof(tail));
    if (rc < 0) return rc;
    print_bytes("RX:", tail_rx, sizeof(tail_rx));

    return 0;
}

int main(int argc, char **argv) {
    lens_bus_config_t cfg;
    lens_bus_default_config(&cfg);
    bool want_rt = true;

    int argi = 1;
    while (argi < argc) {
        const char *a = argv[argi];
        if (strcmp(a, "-h") == 0 || strcmp(a, "--help") == 0) {
            usage(stdout);
            return 0;
        } else if (strcmp(a, "--mosi") == 0 && argi + 1 < argc) {
            uint32_t v; if (parse_u32(argv[++argi], &v) < 0) goto badopt; cfg.mosi_gpio = v;
        } else if (strcmp(a, "--miso") == 0 && argi + 1 < argc) {
            uint32_t v; if (parse_u32(argv[++argi], &v) < 0) goto badopt; cfg.miso_gpio = v;
        } else if (strcmp(a, "--clk") == 0 && argi + 1 < argc) {
            uint32_t v; if (parse_u32(argv[++argi], &v) < 0) goto badopt; cfg.clk_gpio = v;
        } else if (strcmp(a, "--slow-ns") == 0 && argi + 1 < argc) {
            if (parse_u32(argv[++argi], &cfg.slow_period_ns) < 0) goto badopt;
        } else if (strcmp(a, "--fast-ns") == 0 && argi + 1 < argc) {
            if (parse_u32(argv[++argi], &cfg.fast_period_ns) < 0) goto badopt;
        } else if (strcmp(a, "--sample-delay-ns") == 0 && argi + 1 < argc) {
            if (parse_u32(argv[++argi], &cfg.sample_delay_ns) < 0) goto badopt;
        } else if (strcmp(a, "--mosi-setup-ns") == 0 && argi + 1 < argc) {
            if (parse_u32(argv[++argi], &cfg.mosi_setup_ns) < 0) goto badopt;
        } else if (strcmp(a, "--legacy-mosi-after-fall") == 0) {
            cfg.legacy_mosi_after_fall = true;
        } else if (strcmp(a, "--timeout-us") == 0 && argi + 1 < argc) {
            if (parse_u32(argv[++argi], &cfg.clk_high_timeout_us) < 0) goto badopt;
        } else if (strcmp(a, "--no-clk-pullup") == 0) {
            cfg.clk_internal_pullup = false;
        } else if (strcmp(a, "--no-wait-clk-high") == 0) {
            cfg.wait_clk_high = false;
        } else if (strcmp(a, "--no-rt") == 0) {
            want_rt = false;
        } else if (a[0] == '-') {
            goto badopt;
        } else {
            break;
        }
        ++argi;
    }

    if (argi >= argc) {
        usage(stderr);
        return 2;
    }

    const char *cmd = argv[argi++];

    if (want_rt) (void)enable_realtime();

    lens_bus_t bus;
    int rc = lens_bus_open(&bus, &cfg);
    if (rc < 0) {
        fprintf(stderr, "lens_bus_open failed: %s (%d)\n", lens_strerror(rc), rc);
        return 1;
    }

    if (strcmp(cmd, "ready") == 0) {
        uint8_t rx[4];
        rc = lens_ready_alive(&bus, rx);
        if (rc >= 0) print_bytes("RX:", rx, sizeof(rx));
    } else if (strcmp(cmd, "init") == 0) {
        rc = do_init(&bus);
    } else if (strcmp(cmd, "status") == 0) {
        uint8_t status = 0, rx[3];
        rc = lens_read_status(&bus, &status, rx);
        if (rc >= 0) { print_bytes("RX:", rx, sizeof(rx)); printf("status=0x%02X\n", status); }
    } else if (strcmp(cmd, "focus-read") == 0) {
        uint16_t pos = 0; uint8_t rx[3];
        rc = lens_read_focus_position(&bus, &pos, rx);
        if (rc >= 0) { print_bytes("RX:", rx, sizeof(rx)); printf("focus=%u / 0x%04X\n", pos, pos); }
    } else if (strcmp(cmd, "focus-set") == 0) {
        if (argi >= argc) { rc = -EINVAL; }
        else {
            uint16_t pos; rc = parse_u16(argv[argi], &pos);
            if (rc >= 0) { uint8_t rx[3]; rc = lens_focus_set_position(&bus, pos, rx); if (rc >= 0) print_bytes("RX:", rx, sizeof(rx)); }
        }
    } else if (strcmp(cmd, "focus-inc") == 0) {
        if (argi + 1 >= argc) { rc = -EINVAL; }
        else {
            uint16_t cur; int delta; rc = parse_u16(argv[argi], &cur);
            if (rc >= 0) rc = parse_i32(argv[argi + 1], &delta);
            if (rc >= 0) { uint16_t next; rc = lens_focus_increase_from(&bus, cur, delta, &next); if (rc >= 0) printf("new_focus=%u / 0x%04X\n", next, next); }
        }
    } else if (strcmp(cmd, "focus-endpoint") == 0) {
        uint8_t rx;
        rc = lens_focus_endpoint(&bus, &rx);
        if (rc >= 0) print_bytes("RX:", &rx, 1);
    } else if (strcmp(cmd, "aperture-info") == 0) {
        lens_aperture_info_t ai;
        rc = lens_read_aperture_info(&bus, &ai);
        if (rc >= 0) {
            print_bytes("RX:", ai.raw, sizeof(ai.raw));
            printf("aperture raw max/current/min = 0x%02X 0x%02X 0x%02X\n",
                   ai.max_aperture, ai.current_aperture, ai.min_aperture);
        }
    } else if (strcmp(cmd, "aperture-reopen") == 0) {
        uint8_t rx[2];
        rc = lens_aperture_reopen(&bus, rx);
        if (rc >= 0) print_bytes("RX:", rx, sizeof(rx));
    } else if (strcmp(cmd, "aperture-move") == 0) {
        if (argi >= argc) { rc = -EINVAL; }
        else { uint8_t code; rc = parse_u8(argv[argi], &code); if (rc >= 0) { uint8_t rx[2]; rc = lens_aperture_move_code(&bus, code, rx); if (rc >= 0) print_bytes("RX:", rx, sizeof(rx)); } }
    } else if (strcmp(cmd, "aperture-set-code") == 0) {
        if (argi >= argc) { rc = -EINVAL; }
        else { uint8_t code; rc = parse_u8(argv[argi], &code); if (rc >= 0) rc = lens_aperture_set_from_open_code(&bus, code); }
    } else if (strcmp(cmd, "focal-length") == 0) {
        uint16_t fl = 0; uint8_t rx[3];
        rc = lens_read_focal_length(&bus, &fl, rx);
        if (rc >= 0) { print_bytes("RX:", rx, sizeof(rx)); printf("focal_length=%u\n", fl); }
    } else if (strcmp(cmd, "name") == 0) {
        char name[64]; uint8_t raw[18];
        rc = lens_read_name(&bus, name, sizeof(name), raw, sizeof(raw));
        if (rc >= 0) { print_bytes("RX:", raw, sizeof(raw)); printf("name='%s'\n", name); }
    } else if (strcmp(cmd, "xfer-slow") == 0) {
        rc = raw_xfer(&bus, LENS_SPEED_SLOW, argc - argi, &argv[argi]);
    } else if (strcmp(cmd, "xfer-fast") == 0) {
        rc = raw_xfer(&bus, LENS_SPEED_FAST, argc - argi, &argv[argi]);
    } else if (strcmp(cmd, "clk-release") == 0) {
        lens_clk_release(&bus);
        rc = lens_wait_clk_high(&bus, cfg.clk_high_timeout_us);
        printf("CLK level=%d\n", lens_clk_read(&bus));
    } else if (strcmp(cmd, "clk-low") == 0) {
        struct sigaction sa;
        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = on_signal;
        sigaction(SIGINT, &sa, NULL);
        sigaction(SIGTERM, &sa, NULL);
        printf("Pulling CLK low. Terminate with Ctrl+C; cleanup will release it.\n");
        lens_clk_drive_low(&bus);
        while (!g_stop) pause();
        lens_clk_release(&bus);
        rc = 0;
    } else {
        fprintf(stderr, "unknown command: %s\n", cmd);
        rc = -EINVAL;
    }

    if (rc < 0) {
        fprintf(stderr, "command failed: %s (%d)\n", lens_strerror(rc), rc);
        lens_bus_close(&bus);
        return 1;
    }

    lens_bus_close(&bus);
    return 0;

badopt:
    fprintf(stderr, "bad option or missing value near '%s'\n", argv[argi]);
    usage(stderr);
    return 2;
}
