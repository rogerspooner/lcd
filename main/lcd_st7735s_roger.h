/* Definitions for Roger's LCD project on ST7735 display, 128x160px, SPI interface */

#define PIN_NUM_MISO 25 // not used by ST7735S, would be receive data 
#define PIN_NUM_MOSI 33 // SPI data send
#define PIN_NUM_CLK  32 // SPI SCLK clock
#define PIN_NUM_CS   26 // SPI chip select

#define PIN_NUM_DC   25 // SPI data/command (register address)
#define PIN_NUM_RST  14 // Reset pin
#define PIN_NUM_BCKL 27 // backlight

#define LCD_WIDTH  128
#define LCD_HEIGHT 160
