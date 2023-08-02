static void send_lines(spi_device_handle_t spi, int ypos, uint16_t *linedata, int chip);
static void send_line_finish(spi_device_handle_t spi);
static void rgb_stripe(spi_device_handle_t spi, int chip, int frame);

typedef struct display_device_t {
    spi_device_handle_t spi;
    int chip;
    int width;
    int height;
    spi_transaction_t** transactions;
} display_device_t;
