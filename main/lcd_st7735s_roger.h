/* Definitions for Roger's LCD project on ST7735 display, 128x160px, SPI interface */

#define PIN_NUM_MISO   25 // not used by ST7735S, would be receive data 
#define PIN_NUM_MOSI   33 // SPI data send
#define PIN_NUM_CLK    32 // SPI SCLK clock
#define PIN_CS_ST7735S 26 // SPI chip select

#define PIN_NUM_DC   25 // SPI data/command (register address)
#define PIN_NUM_RST  14 // Reset pin
#define PIN_NUM_BCKL 27 // backlight

#define LCD_WIDTH_ST7735S  128
#define LCD_HEIGHT_ST7735S 160
#define MAX_DISPLAY_DIMENSION 320


#define IMAGE_W 128
#define IMAGE_H 160
