#include "lens_proto.h"

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>


int lens_ready_alive(lens_bus_t *bus, uint8_t rx[4])
{
    const uint8_t tx[4] = {0x0A, 0x00, 0x0A, 0x00};
    return lens_bus_transfer(bus, LENS_SPEED_SLOW, tx, rx, sizeof(tx));
}

int lens_read_basic_info(lens_bus_t *bus, lens_basic_info_t *info) {
    if (!info) return -EINVAL;
    const uint8_t tx[9] = {0x80, 0x0A, 0xA4, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx[9] = {0};
    int rc = lens_bus_transfer(bus, LENS_SPEED_SLOW, tx, rx, sizeof(tx));
    if (rc < 0) return rc;

    memcpy(info->raw, rx, sizeof(rx));
    info->type = rx[1];
    info->lens_id = rx[2];
    info->max_focal_length = ((uint16_t)rx[3] << 8) | rx[4];
    info->min_focal_length = ((uint16_t)rx[5] << 8) | rx[6];
    info->c1 = rx[7];
    info->c2 = rx[8];
    return 0;
}

int lens_read_name(lens_bus_t *bus, char *name, size_t name_size, uint8_t *raw_rx, size_t raw_rx_size) {
    if (!name || name_size == 0) return -EINVAL;

    /* From the capture: 0x82 starts lens-name request; repeated 0x83 reads chars. */
    enum { N = 18 };
    uint8_t tx[N];
    uint8_t rx[N] = {0};
    tx[0] = 0x82;
    for (size_t i = 1; i < N; ++i) tx[i] = 0x83;

    int rc = lens_bus_transfer(bus, LENS_SPEED_FAST, tx, rx, N);
    if (rc < 0) return rc;

    if (raw_rx && raw_rx_size) {
        size_t n = raw_rx_size < N ? raw_rx_size : N;
        memcpy(raw_rx, rx, n);
    }

    size_t out = 0;
    for (size_t i = 1; i < N && out + 1 < name_size; ++i) {
        if (rx[i] == 0x00) break;
        name[out++] = (char)rx[i];
    }
    name[out] = '\0';
    return 0;
}

int lens_read_status(lens_bus_t *bus, uint8_t *status, uint8_t raw_rx[3]) {
    const uint8_t tx[3] = {0x90, 0x00, 0x00};
    uint8_t rx[3] = {0};
    int rc = lens_bus_transfer(bus, LENS_SPEED_FAST, tx, rx, sizeof(tx));
    if (rc < 0) return rc;
    if (raw_rx) memcpy(raw_rx, rx, sizeof(rx));
    if (status) *status = rx[2];
    return 0;
}

int lens_read_focus_position(lens_bus_t *bus, uint16_t *position, uint8_t raw_rx[3]) {
    const uint8_t tx[3] = {0xC0, 0x00, 0x00};
    uint8_t rx[3] = {0};
    int rc = lens_bus_transfer(bus, LENS_SPEED_FAST, tx, rx, sizeof(tx));
    if (rc < 0) return rc;
    if (raw_rx) memcpy(raw_rx, rx, sizeof(rx));
    if (position) *position = ((uint16_t)rx[1] << 8) | rx[2];
    return 0;
}

int lens_read_focal_length(lens_bus_t *bus, uint16_t *focal_length, uint8_t raw_rx[3]) {
    const uint8_t tx[3] = {0xA0, 0x00, 0x00};
    uint8_t rx[3] = {0};
    int rc = lens_bus_transfer(bus, LENS_SPEED_FAST, tx, rx, sizeof(tx));
    if (rc < 0) return rc;
    if (raw_rx) memcpy(raw_rx, rx, sizeof(rx));
    if (focal_length) *focal_length = ((uint16_t)rx[1] << 8) | rx[2];
    return 0;
}

int lens_read_aperture_info(lens_bus_t *bus, lens_aperture_info_t *info) {
    if (!info) return -EINVAL;
    const uint8_t tx[4] = {0xB0, 0x00, 0x00, 0x00};
    uint8_t rx[4] = {0};
    int rc = lens_bus_transfer(bus, LENS_SPEED_FAST, tx, rx, sizeof(tx));
    if (rc < 0) return rc;
    memcpy(info->raw, rx, sizeof(rx));
    info->max_aperture = rx[1];
    info->current_aperture = rx[2];
    info->min_aperture = rx[3];
    return 0;
}

int lens_read_c2_unknown(lens_bus_t *bus, uint8_t raw_rx[5]) {
    const uint8_t tx[5] = {0xC2, 0x00, 0x00, 0x00, 0x00};
    return lens_bus_transfer(bus, LENS_SPEED_FAST, tx, raw_rx, sizeof(tx));
}

int lens_focus_set_position(lens_bus_t *bus, uint16_t position, uint8_t raw_rx[3]) {
    if (position > LENS_FOCUS_MAX) return -ERANGE;
    const uint8_t tx[3] = {0x44, (uint8_t)(position >> 8), (uint8_t)(position & 0xFF)};
    return lens_bus_transfer(bus, LENS_SPEED_FAST, tx, raw_rx, sizeof(tx));
}

int lens_focus_endpoint(lens_bus_t *bus, uint8_t *raw_rx) {
    const uint8_t tx[1] = {0x06};
    return lens_bus_transfer(bus, LENS_SPEED_FAST, tx, raw_rx, sizeof(tx));
}

int lens_focus_increase_from(lens_bus_t *bus, uint16_t current, int delta, uint16_t *new_position) {
    int next = (int)current + delta;
    if (next < (int)LENS_FOCUS_MIN) next = (int)LENS_FOCUS_MIN;
    if (next > (int)LENS_FOCUS_MAX) next = (int)LENS_FOCUS_MAX;
    if (new_position) *new_position = (uint16_t)next;
    return lens_focus_set_position(bus, (uint16_t)next, NULL);
}

int lens_aperture_reopen(lens_bus_t *bus, uint8_t raw_rx[2]) {
    const uint8_t tx[2] = {0x13, 0x80};
    return lens_bus_transfer(bus, LENS_SPEED_FAST, tx, raw_rx, sizeof(tx));
}

int lens_aperture_move_code(lens_bus_t *bus, uint8_t code, uint8_t raw_rx[2]) {
    const uint8_t tx[2] = {0x13, code};
    return lens_bus_transfer(bus, LENS_SPEED_FAST, tx, raw_rx, sizeof(tx));
}

int lens_aperture_set_from_open_code(lens_bus_t *bus, uint8_t code) {
    int rc;
    uint8_t tmp2[2];
    uint8_t tmp3[3];

    /* Captures show aperture set as: 0x13 0x80 (reopen), status poll,
     * then 0x13 <relative-from-open code>. This function intentionally does
     * not try to translate f-number to code yet. */
    rc = lens_aperture_reopen(bus, tmp2);
    if (rc < 0) return rc;
    rc = lens_read_status(bus, NULL, tmp3);
    if (rc < 0) return rc;
    rc = lens_aperture_move_code(bus, code, tmp2);
    if (rc < 0) return rc;
    return 0;
}
