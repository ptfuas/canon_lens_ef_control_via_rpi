#include "lens_power.h"

#include <errno.h>
#include <string.h>

int lens_power_init(lens_power_t *power,
                    rpi_gpio_t *gpio,
                    unsigned en_vbus_gpio,
                    unsigned en_vbat_gpio) {
    if (!power || !gpio || !gpio->regs) return -EINVAL;
    if (rpi_gpio_validate_pin(en_vbus_gpio) < 0 ||
        rpi_gpio_validate_pin(en_vbat_gpio) < 0 ||
        en_vbus_gpio == en_vbat_gpio) {
        return -EINVAL;
    }

    memset(power, 0, sizeof(*power));
    power->gpio = gpio;
    power->en_vbus_gpio = en_vbus_gpio;
    power->en_vbat_gpio = en_vbat_gpio;

    /* Program the output latch LOW before changing the pin direction. This
     * prevents a short HIGH pulse when the GPIO becomes an output. */
    rpi_gpio_write(gpio, en_vbus_gpio, false);
    rpi_gpio_write(gpio, en_vbat_gpio, false);
    rpi_gpio_set_output(gpio, en_vbus_gpio);
    rpi_gpio_set_output(gpio, en_vbat_gpio);

    power->initialized = true;
    power->vbus_on = false;
    power->vbat_on = false;
    return 0;
}

void lens_power_set_vbus(lens_power_t *power, bool on) {
    if (!power || !power->initialized || !power->gpio) return;
    rpi_gpio_write(power->gpio, power->en_vbus_gpio, on);
    power->vbus_on = on;
}

void lens_power_set_vbat(lens_power_t *power, bool on) {
    if (!power || !power->initialized || !power->gpio) return;
    rpi_gpio_write(power->gpio, power->en_vbat_gpio, on);
    power->vbat_on = on;
}

void lens_power_all_off(lens_power_t *power) {
    if (!power || !power->initialized || !power->gpio) return;

    /* Remove motor power before removing power from the lens electronics. */
    rpi_gpio_write(power->gpio, power->en_vbat_gpio, false);
    rpi_gpio_write(power->gpio, power->en_vbus_gpio, false);
    power->vbat_on = false;
    power->vbus_on = false;
}
