# Raspberry Pi Zero 2 W bit-banged Canon EF-like lens bus

This is a fast userspace C test driver for the Canon EF-like three-wire bus you decoded:

- `DCL / MOSI`: Raspberry Pi to lens
- `DLC / MISO`: lens to Raspberry Pi
- `CLK`: open-drain/open-collector style, because the lens may pull it low
- no chip select
- 8-bit MSB-first transfers
- MOSI changes during/after the falling edge; MISO is sampled on the rising edge

The implementation uses `/dev/gpiomem` and direct GPIO registers. It is intentionally not portable.

## Default pins

BCM GPIO numbering:

| Lens signal | Pi GPIO | Header pin |
|---|---:|---:|
| MOSI / DCL | GPIO17 | pin 11 |
| MISO / DLC | GPIO27 | pin 13 |
| CLK | GPIO22 | pin 15 |

CLK is never driven high. The code pulls it low by switching GPIO22 to output-low, and releases it high by switching GPIO22 back to input/high-Z. It also enables the Pi's weak internal pull-up on CLK by default. For a real 500 kHz clock, use an external/intermediate pull-up in your level circuit; the internal pull-up is mainly a safe idle helper.

## Build

```bash
make
```

## Basic tests with a logic analyzer, no lens connected

```bash
sudo ./lensctl --no-wait-clk-high xfer-slow 0x0A 0x00 0x0A 0x00
sudo ./lensctl --no-wait-clk-high xfer-fast 0x13 0x80
sudo ./lensctl --no-wait-clk-high focus-set 1000
```

With no external pull-up, leave the internal CLK pull-up enabled. With your level-shifter/pull-up circuit attached, you can try:

```bash
sudo ./lensctl --no-clk-pullup xfer-fast 0x13 0x80
```

## Commands

```bash
sudo ./lensctl ready
sudo ./lensctl init
sudo ./lensctl status
sudo ./lensctl focus-read
sudo ./lensctl focus-set 1000
sudo ./lensctl focus-inc 1000 50
sudo ./lensctl focus-endpoint
sudo ./lensctl aperture-info
sudo ./lensctl aperture-reopen
sudo ./lensctl aperture-move 0x0E
sudo ./lensctl aperture-set-code 0x1A
sudo ./lensctl xfer-fast 0x90 0x00 0x00
sudo ./lensctl xfer-slow 0x0A 0x00
```

## Timing defaults

- Slow period: `12500 ns` full period, about 80 kHz.
- Fast period: `2000 ns` full period, 500 kHz.

Adjust with:

```bash
sudo ./lensctl --slow-ns 12800 --fast-ns 2500 ready
```

## ACK / busy handling

This first version does not decode an ACK byte or ACK state. It does, however, release CLK and wait for the physical CLK line to be high before continuing, unless you pass `--no-wait-clk-high`. That avoids fighting the lens when it pulls CLK low for busy/clock stretching.

## Captured command assumptions encoded here

From the attached workbook and notes:

- ready/alive: `0x0A 0x00 0x0A 0x00`, slow
- basic lens info: `0x80 0x0A 0xA4 0x03 0x00 0x00 0x00 0x00 0x00`, slow
- lens name: `0x82`, then repeated `0x83`, fast
- status: `0x90 0x00 0x00`, fast
- focus position read: `0xC0 0x00 0x00`, fast
- focus set exact position: `0x44 hi lo`, fast
- aperture info: `0xB0 0x00 0x00 0x00`, fast
- aperture reopen: `0x13 0x80`, fast
- aperture relative/from-open movement code: `0x13 code`, fast

Aperture code-to-f-number mapping is not finalized. The code exposes raw `aperture-move` and `aperture-set-code` commands instead of pretending the mapping is known.
