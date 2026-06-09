#ifndef GPIO_RPI_H
#define GPIO_RPI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RPI_GPIO_PULL_OFF  = 0,
    RPI_GPIO_PULL_DOWN = 1,
    RPI_GPIO_PULL_UP   = 2,
} rpi_gpio_pull_t;

typedef struct {
    int fd;
    volatile uint32_t *regs;
    size_t map_size;
} rpi_gpio_t;

int  rpi_gpio_open(rpi_gpio_t *g);
void rpi_gpio_close(rpi_gpio_t *g);
int  rpi_gpio_validate_pin(unsigned gpio);

void rpi_gpio_set_input(rpi_gpio_t *g, unsigned gpio);
void rpi_gpio_set_output(rpi_gpio_t *g, unsigned gpio);
void rpi_gpio_write(rpi_gpio_t *g, unsigned gpio, bool high);
int  rpi_gpio_read(const rpi_gpio_t *g, unsigned gpio);

/*
 * Raspberry Pi Zero/Zero 2/1/2/3 generation pull control.
 * This uses the BCM2835-style GPPUD/GPPUDCLK sequence.
 * It is intentionally targeted at the user's Pi Zero 2 W, not Pi 5.
 */
void rpi_gpio_set_pull_legacy(rpi_gpio_t *g, unsigned gpio, rpi_gpio_pull_t pull);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_RPI_H */
