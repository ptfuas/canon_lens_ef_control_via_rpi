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

static inline uint64_t bus_now_ticks(lens_bus_t *bus) {
    (void)bus;
#if defined(__aarch64__)
    if (bus->has_arch_timer && bus->arch_timer_freq_hz) {
        return read_cntvct_el0();
    }
#endif
    return now_ns_fallback();
}

static inline uint64_t bus_ns_to_ticks(lens_bus_t *bus, uint64_t ns) {
    (void)bus;
#if defined(__aarch64__)
    if (bus->has_arch_timer && bus->arch_timer_freq_hz) {
        return ns_to_ticks(ns, bus->arch_timer_freq_hz);
    }
#endif
    return ns;
}

static inline int64_t tick_delta(uint64_t a, uint64_t b) {
    return (int64_t)(a - b);
}

static inline void wait_until_ticks(lens_bus_t *bus, uint64_t deadline) {
    while (tick_delta(bus_now_ticks(bus), deadline) < 0) {
        cpu_relax();
    }
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
    cfg->slow_period_ns = 12835;  /* 77.91 kHz, from your Pinefeat slow-clock measurement */
    cfg->fast_period_ns = 2000;   /* 500 kHz */
    cfg->sample_delay_ns = 0;
    cfg->mosi_setup_ns = 0;       /* auto: update MOSI halfway through CLK-high */
    cfg->legacy_mosi_after_fall = false;
    cfg->additive_timing = false;
    cfg->clk_internal_pullup = true;
    cfg->wait_clk_high = false;
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
    bus->arch_timer_freq_hz = read_cntfrq_el0();  // 19.2Mhz
    bus->has_arch_timer = bus->arch_timer_freq_hz > 0; // True
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
                             cfg->clk_internal_pullup ? RPI_GPIO_PULL_UP : RPI_GPIO_PULL_OFF);  // activates physical pull-up in clk pin

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

static int transfer_byte_legacy_after_fall(lens_bus_t *bus,
                                               uint32_t period_ns,
                                               uint8_t tx,
                                               uint8_t *rx) {
    const uint32_t half_ns = period_ns / 2;
    uint8_t in = 0;

    for (int bit = 7; bit >= 0; --bit) {
        if (bus->cfg.wait_clk_high) {
            int rc = lens_wait_clk_high(bus, bus->cfg.clk_high_timeout_us);
            if (rc < 0) return rc;
        }

        /* Diagnostic/old mode. This is intentionally not the default anymore:
         * it changes MOSI immediately after CLK falling, which can make the
         * first falling edge appear as if it did not carry a valid MOSI bit. */
        lens_clk_drive_low(bus);
        mosi_write(bus, (tx & (uint8_t)(1u << bit)) != 0);
        delay_ns(bus, half_ns);

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

static int transfer_stream_legacy_after_fall(lens_bus_t *bus,
                                             uint32_t period_ns,
                                             const uint8_t *tx,
                                             uint8_t *rx,
                                             size_t len) {
    for (size_t i = 0; i < len; ++i) {
        uint8_t b = 0;
        int rc = transfer_byte_legacy_after_fall(bus, period_ns, tx[i], &b);
        if (rc < 0) return rc;
        if (rx) rx[i] = b;
    }
    return 0;
}

static inline bool tx_bit_at(const uint8_t *tx, size_t bit_index) {
    const size_t byte_index = bit_index / 8;
    const unsigned bit_in_byte = 7u - (unsigned)(bit_index % 8u);
    return (tx[byte_index] & (uint8_t)(1u << bit_in_byte)) != 0;
}

static inline void rx_store_bit(uint8_t *rx, size_t bit_index, bool bit) {
    if (!rx || !bit) return;
    const size_t byte_index = bit_index / 8;
    const unsigned bit_in_byte = 7u - (unsigned)(bit_index % 8u);
    rx[byte_index] |= (uint8_t)(1u << bit_in_byte);
}


static int lens_wait_clk_low_then_high(lens_bus_t *bus,
                                      uint32_t low_start_timeout_us,
                                      uint32_t high_release_timeout_us) {
#if defined(__aarch64__)
    if (bus->has_arch_timer && bus->arch_timer_freq_hz) {
        const uint64_t timeout_ticks = ns_to_ticks((uint64_t)low_start_timeout_us * 1000ull,
                                                   bus->arch_timer_freq_hz);
        const uint64_t start = read_cntvct_el0();

        /* Wait until the lens pulls the real CLK line low. */
        while (lens_clk_read(bus)) {
            if ((read_cntvct_el0() - start) >= timeout_ticks) return -ETIMEDOUT;
            cpu_relax();
        }

        /* Now wait until the lens releases CLK high again. */
        return lens_wait_clk_high(bus, high_release_timeout_us);
    }
#endif

    const uint64_t timeout_ns = (uint64_t)low_start_timeout_us * 1000ull;
    const uint64_t start = now_ns_fallback();

    while (lens_clk_read(bus)) {
        if ((now_ns_fallback() - start) >= timeout_ns) return -ETIMEDOUT;
        cpu_relax();
    }

    return lens_wait_clk_high(bus, high_release_timeout_us);
}

static int transfer_byte_mosi_mid_high_absolute(lens_bus_t *bus,
                                                uint32_t period_ns,
                                                uint8_t tx,
                                                uint8_t *rx) {
    const uint32_t correction_factor = (uint32_t)((uint64_t)period_ns * 8u / 100u);
    const uint32_t low_ns  = (period_ns - correction_factor) / 2u;
    const uint32_t high_ns = period_ns - low_ns;

    uint32_t setup_before_fall_ns = bus->cfg.mosi_setup_ns;
    if (setup_before_fall_ns == 0) setup_before_fall_ns = high_ns / 2u;
    if (setup_before_fall_ns > high_ns) setup_before_fall_ns = high_ns;

    const uint64_t period_ticks = bus_ns_to_ticks(bus, period_ns);
    const uint64_t low_ticks    = bus_ns_to_ticks(bus, low_ns);
    const uint64_t setup_ticks  = bus_ns_to_ticks(bus, setup_before_fall_ns);
    const uint64_t sample_ticks = bus_ns_to_ticks(bus, bus->cfg.sample_delay_ns);

    uint8_t in = 0;

    /* CLK idles/released high. Prepare bit 7 before the first falling edge. */
    lens_clk_release(bus);
    if (bus->cfg.wait_clk_high) {
        int rc = lens_wait_clk_high(bus, bus->cfg.clk_high_timeout_us);
        if (rc < 0) return rc;
    }

    mosi_write(bus, (tx & 0x80u) != 0);

    uint64_t fall_time = bus_now_ticks(bus) + setup_ticks;
    wait_until_ticks(bus, fall_time);

    for (int bit = 7; bit >= 0; --bit) {
        const bool last_bit = (bit == 0);
        const uint64_t rise_release_time = fall_time + low_ticks;
        uint64_t next_fall_time = fall_time + period_ticks;

        /* Falling edge for this bit. MOSI is already stable. */
        lens_clk_drive_low(bus);

        wait_until_ticks(bus, rise_release_time);
        lens_clk_release(bus);

        if (bus->cfg.wait_clk_high) {
            int rc = lens_wait_clk_high(bus, bus->cfg.clk_high_timeout_us);
            if (rc < 0) return rc;
        }

        if (sample_ticks) {
            wait_until_ticks(bus, bus_now_ticks(bus) + sample_ticks);
        }

        in = (uint8_t)((in << 1) | (miso_read(bus) ? 1u : 0u));

        if (last_bit) break;

        /* Prepare next MOSI bit during the high phase. */
        {
            uint64_t mosi_time = next_fall_time - setup_ticks;
            uint64_t now = bus_now_ticks(bus);

            if (tick_delta(now, mosi_time) < 0) {
                wait_until_ticks(bus, mosi_time);
            }

            mosi_write(bus, (tx & (uint8_t)(1u << (bit - 1))) != 0);

            now = bus_now_ticks(bus);
            if (tick_delta(now + setup_ticks, next_fall_time) > 0) {
                next_fall_time = now + setup_ticks;
            }

            wait_until_ticks(bus, next_fall_time);
            fall_time = next_fall_time;
        }
    }

    lens_clk_release(bus);
    if (rx) *rx = in;
    return 0;
}

/* The real function that transfers the bytes. */
static int transfer_stream_mosi_mid_high_absolute(lens_bus_t *bus,
                                                  uint32_t period_ns,
                                                  const uint8_t *tx,
                                                  uint8_t *rx,
                                                  size_t len) {
    if (rx) memset(rx, 0, len);
    if (len == 0) return 0;

    /*
     * Current PCB/debug rule: there is no Pi GPIO that senses the real
     * lens-side DCLK_5V node. Therefore this code must NOT wait for
     * lens_clk_read(), because that reads only the Pi-side drive GPIO.
     *
     * For the ready/alive init sequence we want to observe four frames:
     *
     *   byte 0: 0x0A  command/check
     *           then wait 50 us before clocking the first reply byte.
     *           Scope measurements showed about 33 us for the lens CLK low/release cycle, so 50 us gives margin.
     *
     *   byte 1: 0x00  dummy/read byte, expected lens reply 0xAA
     *           then wait 50 us before sending the next 0x0A.
     *           Your scope showed the lens clock activity after this frame
     *           takes about 33 us, so 50 us gives margin.
     *
     *   byte 2: 0x0A  second command/check
     *           then wait 50 us before clocking the second reply byte.
     *           This lets us see the fourth frame without waiting another 30 ms.
     *
     *   byte 3: 0x00  dummy/read byte, expected lens reply 0xAA
     *
     * For non-ready/alive transfers this function falls back to a conservative
     * simple rule: after 0x0A wait 50 us, and before a following 0x0A after
     * 0x00 wait 50 us. This is temporary until a real DCLK_5V sense pin exists.
     */
    const useconds_t first_after_0a_wait_us = 50u;
    const useconds_t normal_after_0a_wait_us = 50u;
    const useconds_t after_00_before_0a_wait_us = 50u;

    const bool is_ready_alive_4 =
        tx && len == 4u && tx[0] == 0x0A && tx[1] == 0x00 &&
        tx[2] == 0x0A && tx[3] == 0x00;

    for (size_t i = 0; i < len; ++i) {
        uint8_t in = 0;
        const uint8_t out = tx ? tx[i] : 0x00;

        int rc = transfer_byte_mosi_mid_high_absolute(bus, period_ns, out, &in);
        if (rc < 0) return rc;
        if (rx) rx[i] = in;

        if (i + 1u < len) {
            const uint8_t next_out = tx ? tx[i + 1u] : 0x00;
            useconds_t wait_us = 0;

            lens_clk_release(bus);

            if (is_ready_alive_4) {
                if (i == 0u) {
                    wait_us = first_after_0a_wait_us;      /* 0x0A -> first 0x00 */
                } else if (i == 1u) {
                    wait_us = after_00_before_0a_wait_us;  /* first 0x00 -> second 0x0A */
                } else if (i == 2u) {
                    wait_us = normal_after_0a_wait_us;     /* second 0x0A -> second 0x00 */
                }
            } else {
                if (out == 0x0A) {
                    wait_us = normal_after_0a_wait_us;
                } else if (out == 0x00 && next_out == 0x0A) {
                    wait_us = after_00_before_0a_wait_us;
                }
            }

            if (wait_us > 0u) {
                usleep(wait_us);
            }
        }
    }

    lens_clk_release(bus);
    return 0;
}

static int transfer_stream_mosi_mid_high_additive(lens_bus_t *bus,
                                         uint32_t period_ns,
                                         const uint8_t *tx,
                                         uint8_t *rx,
                                         size_t len) {
    const size_t total_bits = len * 8u;
    const uint32_t low_ns  = period_ns / 2u;
    const uint32_t high_ns = period_ns - low_ns;

    /* Default: update MOSI at the middle of the high phase.
     * Then keep it stable until the next falling edge. */
    uint32_t setup_before_fall_ns = bus->cfg.mosi_setup_ns;
    if (setup_before_fall_ns == 0) setup_before_fall_ns = high_ns / 2u;
    if (setup_before_fall_ns > high_ns) setup_before_fall_ns = high_ns;

    const uint32_t after_rise_before_mosi_ns = high_ns - setup_before_fall_ns;
    const uint32_t sample_delay_ns = bus->cfg.sample_delay_ns;

    if (rx) memset(rx, 0, len);

    if (total_bits == 0) return 0;

    /* CLK is idle/released high. Prepare the first MOSI bit BEFORE the first
     * falling edge so that falling edge #1 already belongs to bit 7. */
    lens_clk_release(bus);
    if (bus->cfg.wait_clk_high) {
        int rc = lens_wait_clk_high(bus, bus->cfg.clk_high_timeout_us);
        if (rc < 0) return rc;
    }

    mosi_write(bus, tx_bit_at(tx, 0));
    delay_ns(bus, setup_before_fall_ns);

    for (size_t bit_index = 0; bit_index < total_bits; ++bit_index) {
        /* Current MOSI bit is already valid here. This falling edge is the bit
         * boundary visible in your Pinefeat captures. */
        lens_clk_drive_low(bus);
        delay_ns(bus, low_ns);

        /* Rising edge is generated by releasing the open-drain CLK. MISO is
         * sampled on/just after this positive edge. */
        lens_clk_release(bus);
        if (bus->cfg.wait_clk_high) {
            int rc = lens_wait_clk_high(bus, bus->cfg.clk_high_timeout_us);
            if (rc < 0) return rc;
        }

        if (sample_delay_ns) delay_ns(bus, sample_delay_ns);
        rx_store_bit(rx, bit_index, miso_read(bus) != 0);

        if (bit_index + 1u < total_bits) {
            /* Keep MOSI unchanged for the first part of CLK-high, then update
             * it halfway through the high phase so the next falling edge is
             * not racing the data transition. Account for any explicit sample
             * delay already spent after the rising edge. */
            if (after_rise_before_mosi_ns > sample_delay_ns) {
                delay_ns(bus, after_rise_before_mosi_ns - sample_delay_ns);
            }
            mosi_write(bus, tx_bit_at(tx, bit_index + 1u));
            delay_ns(bus, setup_before_fall_ns);
        } else {
            /* Finish the last high phase. */
            if (high_ns > sample_delay_ns) {
                delay_ns(bus, high_ns - sample_delay_ns);
            }
        }
    }

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

    /* No DCLK_5V sense pin exists on the current PCB, so do not wait here. */
    int rc;
    if (bus->cfg.legacy_mosi_after_fall) {
        rc = transfer_stream_legacy_after_fall(bus, period_ns, tx, rx, len);
    } else {
        rc = bus->cfg.additive_timing
             ? transfer_stream_mosi_mid_high_additive(bus, period_ns, tx, rx, len)
             : transfer_stream_mosi_mid_high_absolute(bus, period_ns, tx, rx, len);
    }

    lens_clk_release(bus);
    return rc;
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