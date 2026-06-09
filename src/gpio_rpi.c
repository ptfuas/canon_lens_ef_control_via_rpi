#define _GNU_SOURCE
#include "gpio_rpi.h"

#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

/* GPIO register offsets from the GPIO peripheral base, word-indexed. */
enum {
    GPIO_MAP_SIZE = 0x1000,

    GPFSEL0    = 0x00 / 4,
    GPSET0     = 0x1C / 4,
    GPCLR0     = 0x28 / 4,
    GPLEV0     = 0x34 / 4,
    GPPUD      = 0x94 / 4,
    GPPUDCLK0  = 0x98 / 4,
};

static inline void compiler_barrier(void) {
    __asm__ volatile("" ::: "memory");
}

static inline void tiny_wait_cycles(unsigned cycles) {
    /* The Broadcom sequence asks for roughly 150 cycles. Use more; this is not
     * timing-critical and avoids depending on the CPU frequency. */
    for (volatile unsigned i = 0; i < cycles; ++i) {
        compiler_barrier();
    }
}

int rpi_gpio_validate_pin(unsigned gpio) {
    return gpio <= 53 ? 0 : -EINVAL;
}

int rpi_gpio_open(rpi_gpio_t *g) {
    if (!g) return -EINVAL;
    memset(g, 0, sizeof(*g));
    g->fd = -1;
    g->map_size = GPIO_MAP_SIZE;

    int fd = open("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC);
    if (fd < 0) {
        return -errno;
    }

    void *map = mmap(NULL, g->map_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (map == MAP_FAILED) {
        int saved = errno;
        close(fd);
        return -saved;
    }

    g->fd = fd;
    g->regs = (volatile uint32_t *)map;
    return 0;
}

void rpi_gpio_close(rpi_gpio_t *g) {
    if (!g) return;
    if (g->regs) {
        munmap((void *)g->regs, g->map_size);
        g->regs = NULL;
    }
    if (g->fd >= 0) {
        close(g->fd);
        g->fd = -1;
    }
}

void rpi_gpio_set_input(rpi_gpio_t *g, unsigned gpio) {
    volatile uint32_t *fsel = &g->regs[GPFSEL0 + gpio / 10];
    const unsigned shift = (gpio % 10) * 3;
    uint32_t v = *fsel;
    v &= ~(7u << shift);
    *fsel = v;
    compiler_barrier();
}

void rpi_gpio_set_output(rpi_gpio_t *g, unsigned gpio) {
    volatile uint32_t *fsel = &g->regs[GPFSEL0 + gpio / 10];
    const unsigned shift = (gpio % 10) * 3;
    uint32_t v = *fsel;
    v &= ~(7u << shift);
    v |=  (1u << shift); /* function 001 = output */
    *fsel = v;
    compiler_barrier();
}

void rpi_gpio_write(rpi_gpio_t *g, unsigned gpio, bool high) {
    const unsigned bank = gpio / 32;
    const uint32_t mask = 1u << (gpio % 32);
    if (high) {
        g->regs[GPSET0 + bank] = mask;
    } else {
        g->regs[GPCLR0 + bank] = mask;
    }
    compiler_barrier();
}

int rpi_gpio_read(const rpi_gpio_t *g, unsigned gpio) {
    const unsigned bank = gpio / 32;
    const uint32_t mask = 1u << (gpio % 32);
    return (g->regs[GPLEV0 + bank] & mask) ? 1 : 0;
}

void rpi_gpio_set_pull_legacy(rpi_gpio_t *g, unsigned gpio, rpi_gpio_pull_t pull) {
    const unsigned bank = gpio / 32;
    const uint32_t mask = 1u << (gpio % 32);

    g->regs[GPPUD] = (uint32_t)pull & 0x3u;
    tiny_wait_cycles(1000);
    g->regs[GPPUDCLK0 + bank] = mask;
    tiny_wait_cycles(1000);
    g->regs[GPPUD] = 0;
    g->regs[GPPUDCLK0 + bank] = 0;
    compiler_barrier();
}
