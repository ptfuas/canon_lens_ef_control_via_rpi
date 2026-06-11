#ifndef LENS_PROTO_H
#define LENS_PROTO_H

#include "lens_bus.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LENS_FOCUS_MIN 0u
#define LENS_FOCUS_MAX 4234u

typedef struct {
    uint8_t type;
    uint8_t lens_id;
    uint16_t max_focal_length;
    uint16_t min_focal_length;
    uint8_t c1;
    uint8_t c2;
    uint8_t raw[9];
} lens_basic_info_t;

typedef struct {
    uint8_t max_aperture;
    uint8_t current_aperture;
    uint8_t min_aperture;
    uint8_t raw[4];
} lens_aperture_info_t;

int lens_ready_alive(lens_bus_t *bus, uint8_t rx[4]);
int lens_read_basic_info(lens_bus_t *bus, lens_basic_info_t *info);
int lens_read_name(lens_bus_t *bus, char *name, size_t name_size, uint8_t *raw_rx, size_t raw_rx_size);
int lens_read_status(lens_bus_t *bus, uint8_t *status, uint8_t raw_rx[3]);
int lens_read_focus_position(lens_bus_t *bus, uint16_t *position, uint8_t raw_rx[3]);
int lens_read_focal_length(lens_bus_t *bus, uint16_t *focal_length, uint8_t raw_rx[3]);
int lens_read_aperture_info(lens_bus_t *bus, lens_aperture_info_t *info);
int lens_read_c2_unknown(lens_bus_t *bus, uint8_t raw_rx[5]);

int lens_focus_set_position(lens_bus_t *bus, uint16_t position, uint8_t raw_rx[3]);
int lens_focus_endpoint(lens_bus_t *bus, uint8_t *raw_rx);
int lens_focus_increase_from(lens_bus_t *bus, uint16_t current, int delta, uint16_t *new_position);

int lens_aperture_reopen(lens_bus_t *bus, uint8_t raw_rx[2]);
int lens_aperture_move_code(lens_bus_t *bus, uint8_t code, uint8_t raw_rx[2]);
int lens_aperture_set_from_open_code(lens_bus_t *bus, uint8_t code);

#ifdef __cplusplus
}
#endif

#endif /* LENS_PROTO_H */
#ifndef LENS_PROTO_H
#define LENS_PROTO_H

#include "lens_bus.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LENS_FOCUS_MIN 0u
#define LENS_FOCUS_MAX 4234u

typedef struct {
    uint8_t type;
    uint8_t lens_id;
    uint16_t max_focal_length;
    uint16_t min_focal_length;
    uint8_t c1;
    uint8_t c2;
    uint8_t raw[9];
} lens_basic_info_t;

typedef struct {
    uint8_t max_aperture;
    uint8_t current_aperture;
    uint8_t min_aperture;
    uint8_t raw[4];
} lens_aperture_info_t;

int lens_ready_alive(lens_bus_t *bus, uint8_t rx[4]);
int lens_read_basic_info(lens_bus_t *bus, lens_basic_info_t *info);
int lens_read_name(lens_bus_t *bus, char *name, size_t name_size, uint8_t *raw_rx, size_t raw_rx_size);
int lens_read_status(lens_bus_t *bus, uint8_t *status, uint8_t raw_rx[3]);
int lens_read_focus_position(lens_bus_t *bus, uint16_t *position, uint8_t raw_rx[3]);
int lens_read_focal_length(lens_bus_t *bus, uint16_t *focal_length, uint8_t raw_rx[3]);
int lens_read_aperture_info(lens_bus_t *bus, lens_aperture_info_t *info);
int lens_read_c2_unknown(lens_bus_t *bus, uint8_t raw_rx[5]);

int lens_focus_set_position(lens_bus_t *bus, uint16_t position, uint8_t raw_rx[3]);
int lens_focus_endpoint(lens_bus_t *bus, uint8_t *raw_rx);
int lens_focus_increase_from(lens_bus_t *bus, uint16_t current, int delta, uint16_t *new_position);

int lens_aperture_reopen(lens_bus_t *bus, uint8_t raw_rx[2]);
int lens_aperture_move_code(lens_bus_t *bus, uint8_t code, uint8_t raw_rx[2]);
int lens_aperture_set_from_open_code(lens_bus_t *bus, uint8_t code);

#ifdef __cplusplus
}
#endif

#endif /* LENS_PROTO_H */
