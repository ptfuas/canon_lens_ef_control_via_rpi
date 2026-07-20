#ifndef LENS_POWER_H
#define LENS_POWER_H

#include "gpio_rpi.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LENS_POWER_DEFAULT_EN_VBUS_GPIO 23u
#define LENS_POWER_DEFAULT_EN_VBAT_GPIO 24u

typedef struct {
    rpi_gpio_t *gpio;
    unsigned en_vbus_gpio;
    unsigned en_vbat_gpio;
    bool initialized;
    bool vbus_on;
    bool vbat_on;
} lens_power_t;

/* Configure both enable GPIOs as outputs in their safe OFF state. */
int lens_power_init(lens_power_t *power,
                    rpi_gpio_t *gpio,
                    unsigned en_vbus_gpio,
                    unsigned en_vbat_gpio);

/* Lens electronics supply, connected to TPS22968 ON2 / EN_VBUS. */
void lens_power_set_vbus(lens_power_t *power, bool on);

/* Lens motor supply, connected to TPS22968 ON1 / EN_VBAT. */
void lens_power_set_vbat(lens_power_t *power, bool on);

/* Disable motor supply first, then lens electronics supply. */
void lens_power_all_off(lens_power_t *power);

#ifdef __cplusplus
}
#endif

#endif /* LENS_POWER_H */
