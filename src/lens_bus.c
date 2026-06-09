#define _GNU_SOURCE
#include "lens_bus.h"

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* Same GPIO register offsets as gpio_rpi.c, word-indexed. Kept here for fast
 * inlined access to the hot-path SET/CLR/LEV/FSEL registers. */
enum {
    GPFSEL0 = 0x00 / 4,
    GPSET0  = 0x1C / 4,
    GPCLR0  = 0x28 / 4,
    GPLEV0  = 0x34 / 4,
};

static inline void cpu_relax(void) {
#if defined(__aarch64__) || defined(__arm__)
    __asm__ volatile("yield" ::: "memory");
#else
    __asm__ volatile("pause" ::: "memory");
#endif
}

static inline void compiler_barrier(void) {
    __asm__ volatile("" ::: "memory");
}

#if defined(__aarch64__)
static inline uint64_t read_cntvct_el0(void) {
    uint64_t v;
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(v));
    return v;
}

static inline uint64_t read_cntfrq_el0(void) {
    uint64_t v;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(v));
    return v;
}
#endif

static uint64_t now_ns_fallback(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static inline uint64_t ns_to_ticks(uint64_t ns, uint64_t freq_hz) {
    return (ns * freq_hz + 999999999ull) / 1000000000ull;
}

static inline void delay_ns(lens_bus_t *bus, uint32_t ns) {
    if (ns == 0) return;

#if defined(__aarch64__)
    if (bus->has_arch_timer && bus->arch_timer_freq_hz) {
        const uint64_t ticks = ns_to_ticks(ns, bus->arch_timer_freq_hz);
        const uint64_t start = read_cntvct_el0();
        while ((read_cntvct_el0() - start) < ticks) {
            cpu_relax();
        }
        return;
    }
#else
    (void)bus;
#endif

    const uint64_t start = now_ns_fallback();
    while ((now_ns_fallback() - start) < ns) {
        cpu_relax();
    }
}

static inline void mosi_write(lens_bus_t *bus, bool high) {
    if (high) {
        bus->gpio.regs[GPSET0 + bus->mosi_bank] = bus->mosi_mask;
    } else {
        bus->gpio.regs[GPCLR0 + bus->mosi_bank] = bus->mosi_mask;
    }
    compiler_barrier();
}

static inline int miso_read(lens_bus_t *bus) {
    return (bus->gpio.regs[GPLEV0 + bus->miso_bank] & bus->miso_mask) ? 1 : 0;
}

void lens_bus_default_config(lens_bus_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->mosi_gpio = 17;          /* physical pin 11 */
    cfg->miso_gpio = 27;          /* physical pin 13 */
    cfg->clk_gpio  = 22;          /* physical pin 15 */
    cfg->slow_period_ns = 12500;  /* ~80 kHz, close to your slow captures */
    cfg->fast_period_ns = 2000;   /* 500 kHz */
    cfg->sample_delay_ns = 0;
    cfg->clk_internal_pullup = true;
    cfg->wait_clk_high = true;
    cfg->clk_high_timeout_us = 100000; /* 100 ms, enough for observed aperture busy times. */
}

static int validate_config(const lens_bus_config_t *cfg) {
    if (rpi_gpio_validate_pin(cfg->mosi_gpio) ||
        rpi_gpio_validate_pin(cfg->miso_gpio) ||
        rpi_gpio_validate_pin(cfg->clk_gpio)) {
        return -EINVAL;
    }
    if (cfg->mosi_gpio == cfg->miso_gpio || cfg->mosi_gpio == cfg->clk_gpio ||
        cfg->miso_gpio == cfg->clk_gpio) {
        return -EINVAL;
    }
    if (cfg->slow_period_ns < 1000 || cfg->fast_period_ns < 1000) {
        return -EINVAL;
    }
    if (cfg->clk_high_timeout_us == 0 && cfg->wait_clk_high) {
        return -EINVAL;
    }
    return 0;
}

int lens_bus_open(lens_bus_t *bus, const lens_bus_config_t *cfg) {
    if (!bus || !cfg) return -EINVAL;
    memset(bus, 0, sizeof(*bus));
    bus->cfg = *cfg;

    int rc = validate_config(cfg);
    if (rc < 0) return rc;

    rc = rpi_gpio_open(&bus->gpio);
    if (rc < 0) return rc;

#if defined(__aarch64__)
    bus->arch_timer_freq_hz = read_cntfrq_el0();
    bus->has_arch_timer = bus->arch_timer_freq_hz > 0;
#else
    bus->has_arch_timer = false;
    bus->arch_timer_freq_hz = 0;
#endif

    bus->mosi_bank = cfg->mosi_gpio / 32;
    bus->miso_bank = cfg->miso_gpio / 32;
    bus->clk_bank  = cfg->clk_gpio  / 32;
    bus->mosi_mask = 1u << (cfg->mosi_gpio % 32);
    bus->miso_mask = 1u << (cfg->miso_gpio % 32);
    bus->clk_mask  = 1u << (cfg->clk_gpio  % 32);

    bus->clk_fsel_index = GPFSEL0 + cfg->clk_gpio / 10;
    const unsigned clk_shift = (cfg->clk_gpio % 10) * 3;
    bus->clk_fsel_clear_mask = ~(7u << clk_shift);
    bus->clk_fsel_output_bits = 1u << clk_shift;

    /* Safe initial state: MOSI low, MISO input, CLK input/released. */
    rpi_gpio_write(&bus->gpio, cfg->mosi_gpio, false);
    rpi_gpio_set_output(&bus->gpio, cfg->mosi_gpio);
    rpi_gpio_set_input(&bus->gpio, cfg->miso_gpio);
    rpi_gpio_set_pull_legacy(&bus->gpio, cfg->miso_gpio, RPI_GPIO_PULL_OFF);

    rpi_gpio_write(&bus->gpio, cfg->clk_gpio, false); /* output latch is low for future open-drain pulls */
    rpi_gpio_set_input(&bus->gpio, cfg->clk_gpio);
    rpi_gpio_set_pull_legacy(&bus->gpio, cfg->clk_gpio,
                             cfg->clk_internal_pullup ? RPI_GPIO_PULL_UP : RPI_GPIO_PULL_OFF);

    return 0;
}

void lens_bus_close(lens_bus_t *bus) {
    if (!bus) return;
    if (bus->gpio.regs) {
        /* Release the bus. Do not leave CLK driven. */
        lens_clk_release(bus);
        rpi_gpio_write(&bus->gpio, bus->cfg.mosi_gpio, false);
        rpi_gpio_set_input(&bus->gpio, bus->cfg.mosi_gpio);
        rpi_gpio_set_input(&bus->gpio, bus->cfg.miso_gpio);
    }
    rpi_gpio_close(&bus->gpio);
}

void lens_clk_drive_low(lens_bus_t *bus) {
    /* Open-drain low: ensure output latch is low, then switch function to output. */
    bus->gpio.regs[GPCLR0 + bus->clk_bank] = bus->clk_mask;
    volatile uint32_t *fsel = &bus->gpio.regs[bus->clk_fsel_index];
    uint32_t v = *fsel;
    v &= bus->clk_fsel_clear_mask;
    v |= bus->clk_fsel_output_bits;
    *fsel = v;
    compiler_barrier();
}

void lens_clk_release(lens_bus_t *bus) {
    /* Open-drain high: switch to input/high-Z; pull-up or external circuit raises it. */
    volatile uint32_t *fsel = &bus->gpio.regs[bus->clk_fsel_index];
    uint32_t v = *fsel;
    v &= bus->clk_fsel_clear_mask;
    *fsel = v;
    compiler_barrier();
}

int lens_clk_read(const lens_bus_t *bus) {
    return (bus->gpio.regs[GPLEV0 + bus->clk_bank] & bus->clk_mask) ? 1 : 0;
}

int lens_wait_clk_high(lens_bus_t *bus, uint32_t timeout_us) {
    if (lens_clk_read(bus)) return 0;

#if defined(__aarch64__)
    if (bus->has_arch_timer && bus->arch_timer_freq_hz) {
        const uint64_t timeout_ticks = ns_to_ticks((uint64_t)timeout_us * 1000ull,
                                                   bus->arch_timer_freq_hz);
        const uint64_t start = read_cntvct_el0();
        while (!lens_clk_read(bus)) {
            if ((read_cntvct_el0() - start) >= timeout_ticks) return -ETIMEDOUT;
            cpu_relax();
        }
        return 0;
    }
#endif

    const uint64_t timeout_ns = (uint64_t)timeout_us * 1000ull;
    const uint64_t start = now_ns_fallback();
    while (!lens_clk_read(bus)) {
        if ((now_ns_fallback() - start) >= timeout_ns) return -ETIMEDOUT;
        cpu_relax();
    }
    return 0;
}

static int transfer_byte(lens_bus_t *bus, uint32_t period_ns, uint8_t tx, uint8_t *rx) {
    const uint32_t half_ns = period_ns / 2;
    uint8_t in = 0;

    for (int bit = 7; bit >= 0; --bit) {
        if (bus->cfg.wait_clk_high) {
            int rc = lens_wait_clk_high(bus, bus->cfg.clk_high_timeout_us);
            if (rc < 0) return rc;
        }

        /* Falling edge. In this EF-like mode the transmitter changes data on
         * the falling edge and the receiver samples on the rising edge. */
        lens_clk_drive_low(bus);
        mosi_write(bus, (tx & (uint8_t)(1u << bit)) != 0);
        delay_ns(bus, half_ns);

        /* Rising edge is by release/high-Z, not push-pull high. */
        lens_clk_release(bus);
        if (bus->cfg.wait_clk_high) {
            int rc = lens_wait_clk_high(bus, bus->cfg.clk_high_timeout_us);
            if (rc < 0) return rc;
        }
        if (bus->cfg.sample_delay_ns) delay_ns(bus, bus->cfg.sample_delay_ns);

        in = (uint8_t)((in << 1) | (miso_read(bus) ? 1u : 0u));
        delay_ns(bus, half_ns);
    }

    if (rx) *rx = in;
    return 0;
}

int lens_bus_transfer_period(lens_bus_t *bus,
                             uint32_t period_ns,
                             const uint8_t *tx,
                             uint8_t *rx,
                             size_t len) {
    if (!bus || !tx || len == 0) return -EINVAL;
    if (!bus->gpio.regs) return -ENODEV;

    lens_clk_release(bus);
    if (bus->cfg.wait_clk_high) {
        int rc = lens_wait_clk_high(bus, bus->cfg.clk_high_timeout_us);
        if (rc < 0) return rc;
    }

    for (size_t i = 0; i < len; ++i) {
        uint8_t b = 0;
        int rc = transfer_byte(bus, period_ns, tx[i], &b);
        if (rc < 0) {
            lens_clk_release(bus);
            return rc;
        }
        if (rx) rx[i] = b;
    }

    lens_clk_release(bus);
    return 0;
}

int lens_bus_transfer(lens_bus_t *bus,
                      lens_speed_t speed,
                      const uint8_t *tx,
                      uint8_t *rx,
                      size_t len) {
    const uint32_t period = (speed == LENS_SPEED_FAST) ? bus->cfg.fast_period_ns
                                                        : bus->cfg.slow_period_ns;
    return lens_bus_transfer_period(bus, period, tx, rx, len);
}

const char *lens_strerror(int rc) {
    if (rc >= 0) return "ok";
    switch (-rc) {
        case EINVAL: return "invalid argument/configuration";
        case ENODEV: return "GPIO bus is not open";
        case ETIMEDOUT: return "timeout waiting for CLK high; lens may be holding CLK low";
        case EACCES: return "permission denied opening /dev/gpiomem";
        case ENOENT: return "/dev/gpiomem not found";
        default: return strerror(-rc);
    }
}
