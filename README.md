# LCD graphic display by raw SPI commands
# SPI Host Driver Example

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This example aims to show how to use SPI Host driver API, like `spi_transaction_t` and spi_device_queue.

If you are looking for code to drive LCDs in general, rather than code that uses the SPI master, that may be a better example to look at as it uses ESP-IDFs built-in LCD support rather than doing all the low-level work itself, which can be found at `examples/peripherals/lcd/tjpgd/`

## How to Use Example

### Hardware Required

* An ESP development board, with SPI LCD
* Rectangular LCD display with 128x160 TFT driven by ST7735S
** Chip Select on GPI 26 for rectangular
* Round LCD display with 240x240 TFT driven by GC9A01
** Chip select on GPIO 13 for round
* SPI SCLK on GPIO 32
* MOSI data out GPIO 33
* DC data/command register select on 25
* Reset on 14
* Backlight on 27
* 3.3v VCC to the displays
 

Connection :

Depends on boards. Refer to `spi_master_example_main.c` No wiring is required on ESP-WROVER-KIT

### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Example Output

On ESP-WROVER-KIT there will be:

```
LCD ID: 00000000
ILI9341 detected.
LCD ILI9341 initialization.
```

At the meantime `ESP32` will be displayed on the connected LCD screen.

## Troubleshooting

For any technical queries, please open an [issue] (https://github.com/espressif/esp-idf/issues) on GitHub. We will get back to you soon.
