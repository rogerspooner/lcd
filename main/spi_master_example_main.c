/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lcd_st7735s_roger.h"
#include "lcd_gc9a01_roger.h"

#include "pretty_effect.h"

/*
 This code displays some fancy graphics on the 128x160 LCD on an ESPWROOM board.
 This example demonstrates the use of both spi_device_transmit as well as
 spi_device_queue_trans/spi_device_get_trans_result and pre-transmit callbacks.

 Some info about the ILI9341/ST7789V: It has an C/D line, which is connected to a GPIO here. It expects this
 line to be low for a command and high for data. We use a pre-transmit callback here to control that
 line: every transaction has as the user-definable argument the needed state of the D/C line and just
 before the transaction is sent, the callback will set this line to the correct state.
*/

#define LCD_HOST    HSPI_HOST
#define INIT_CMD_DELAY_AFTER 0x80 // long delay after sending SPI command. This should be ORed with databytes field in lcd_init_cmd_t struct
#define INIT_CMD_END_SEQUENCE 0xFF


/* RIWS notes
Proposed connections to ESP32
3 SCL = Clock = GPIO_32
4 SDA = Data in (maybe bidirectional) = GPIO_33 / data input. Any output? Connect to MOSI
5 RST = GPIO_14
6 DC = GPIO_25 / what is this?
7 CS = GPIO_26
8 BLK = GPIO_27
*/

//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES 16

// The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 INIT_CMD_DELAY_AFTER = delay after set; 0xFF INIT_CMD_END_SEQUENCE = end of cmds.
} lcd_init_cmd_t;


DRAM_ATTR static const lcd_init_cmd_t st7735s_InitSequence[] = {
    { 0x01, { 0x80 }, 1 | INIT_CMD_DELAY_AFTER }, // Software reset
    { 0x11, { 0x80 }, 1 | INIT_CMD_DELAY_AFTER }, // Exit sleep mode
    { 0xB1, { 0x03, 0x03 }, 2 }, // Frame rate control - normal mode
    { 0xB2, { 0x03, 0x03, 0x00, 0x33, 0x33 }, 5 }, // Frame rate control - idle mode
    { 0xB3, { 0x03, 0x03, 0x00, 0x33, 0x33 }, 5 }, // Frame rate control - partial mode
    { 0xB4, { 0x07 }, 1 }, // Display inversion control
    { 0xC0, { 0xA2, 0x02, 0x84, 0x08 },4  }, // Power control 1
    { 0xC1, { 0xC5 }, 1 }, // Power control 2
    { 0xC2, { 0x0A, 0x00 }, 2 }, // Power control 3
    { 0xC3, { 0x8A, 0x2A }, 2 }, // Power control 4
    { 0xC4, { 0x8A, 0xEE }, 2 }, // Power control 5
    { 0xC5, { 0x0E }, 1 }, // VCOM control 1
    // display resolution 128x160. GM[1:0] = 11 should have been set by hard-wiring pins
    { 0x36, { 0xC0 }, 1 }, // Memory data access control (row address/col address, top to bottom refresh, left-to-right refresh, RGB order)
    { 0x3A, { 0x05 }, 1 }, // Interface pixel format (16-bit/pixel RGB565)
    { 0xE0, { 0x1F, 0x24, 0x0C, 0x10, 0x06, 0x09, 0x05, 0x00, 0x08, 0x03, 0x0E, 0x09, 0x00 }, 13  }, // Gamma curve
    { 0xE1, { 0x1F, 0x24, 0x0C, 0x10, 0x06, 0x09, 0x05, 0x00, 0x08, 0x03, 0x0E, 0x09, 0x00 }, 13 }, // Gamma curve
    { 0x2A, { 0x00, 0x00, 0x00, 0x7F }, 4 }, // Column address set. XS = start row at offset 0x0000. XE = 0x007F for 128 pixels
    { 0x2B, { 0x00, 0x00, 0x00, 0x9F }, 4 }, // Row address set. YS = start row at offset 0x0000. YE = 0x009F for 160 pixels
    { 0x29, { }, 0 | INIT_CMD_DELAY_AFTER }, // Display on
    // 0x2C, // Memory write
    {0, {0}, INIT_CMD_END_SEQUENCE } // terminate this list. This is not sent.
};

/* Initialise GC9A01. Based on manufacturer's recommendation but copied from AdaFruit */
DRAM_ATTR static const lcd_init_cmd_t gc9a01_InitSequence[] = {
  { GC9A01A_INREGEN2, { }, 0 },
  { 0xEB, { 0x14 }, 1 },// ?
  { GC9A01A_INREGEN1, { }, 0} ,
  { GC9A01A_INREGEN2, { }, 0} ,
  { 0xEB, { 0x14 }, 1 }, // ?
  { 0x84, { 0x40 }, 1 }, // ?
  { 0x85, { 0xFF }, 1 }, // ?
  { 0x86, { 0xFF }, 1 }, // ?
  { 0x87, { 0xFF }, 1 }, // ?
  { 0x88, { 0x0A }, 1 }, // ?
  { 0x89, { 0x21 }, 1 }, // ?
  { 0x8A, { 0x00 }, 1 }, // ?
  { 0x8B, { 0x80 }, 1 }, // ?
  { 0x8C, { 0x01 }, 1 }, // ?
  { 0x8D, { 0x01 }, 1 }, // ?
  { 0x8E, { 0xFF }, 1 }, // ?
  { 0x8F, { 0xFF }, 1 }, // ?
  { 0xB6, { 0x00, 0x00 }, 2 }, // ?
  { GC9A01A_MADCTL, { MADCTL_MX | MADCTL_BGR }, 1 },
  { GC9A01A_COLMOD, { 0x05 }, 1 },
  { 0x90, { 0x08, 0x08, 0x08, 0x08}, 4}, // ?
  { 0xBD, { 0x06 }, 1 }, // ?
  { 0xBC, { 0x00 }, 1 }, // ?
  { 0xFF, { 0x60, 0x01, 0x04 }, 3 }, // ?
  { GC9A01A1_POWER2, { 0x13 }, 1 },
  { GC9A01A1_POWER3, { 0x13 }, 1 },
  { GC9A01A1_POWER4, { 0x22 }, 1 },
  { 0xBE, { 0x11 }, 1 }, // ?
  { 0xE1, { 0x10, 0x0E }, 2 }, // ?
  { 0xDF, { 0x21, 0x0c, 0x02 }, 3 }, // ?
  { GC9A01A_GAMMA1, { 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A }, 6 },
  { GC9A01A_GAMMA2, { 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F }, 6 },
  { GC9A01A_GAMMA3, { 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A }, 6 },
  { GC9A01A_GAMMA4, { 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F }, 6 },
  { 0xED, { 0x1B, 0x0B }, 2}, // ?
  { 0xAE, { 0x77 }, 1 }, // ?
  { 0xCD, { 0x63 }, 1 }, // ?
  // Unsure what this line (from manufacturer's boilerplate code) is
  // meant to do, but users reported issues, seems to work OK without:
  //0x70, 9, 0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0x08, 0x03, // ?
  { GC9A01A_FRAMERATE, { 0x34 }, 1 },
  { 0x62, { 0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, // ?
            0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70 }, 12 },
  { 0x63, { 0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, // ?
            0x18, 0x13, 0x71, 0xF3, 0x70, 0x70} , 12 },
  { 0x64, { 0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07 }, 7 }, // ?
  { 0x66, { 0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00 }, 10 }, // ?
  { 0x67, { 0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98 }, 10 }, // ?
  { 0x74, { 0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00 }, 7 }, // ?
  { 0x98, { 0x3e, 0x07 }, 2 }, // ?
  { GC9A01A_TEON, { }, 0 },
  { GC9A01A_INVON, { }, 0 },
  { GC9A01A_SLPOUT, { }, 0 | INIT_CMD_DELAY_AFTER }, // Exit sleep
  { GC9A01A_DISPON, { }, 0 | INIT_CMD_DELAY_AFTER }, // Display on
  { 0, { }, 0 | INIT_CMD_END_SEQUENCE }                 // End of list
};


/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd, bool keep_cs_active)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    if (keep_cs_active) {
      t.flags = SPI_TRANS_CS_KEEP_ACTIVE;   //Keep CS active after data transfer
    }
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

//Initialize the display
void lcd_init(spi_device_handle_t spi, int chip)
{
    int cmd=0;
    const lcd_init_cmd_t* lcd_init_cmds;

    //Initialize non-SPI GPIOs
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = ((1ULL<<PIN_NUM_DC) | (1ULL<<PIN_NUM_RST) | (1ULL<<PIN_NUM_BCKL));
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = true;
    gpio_config(&io_conf);

    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Backlight on
    gpio_set_level(PIN_NUM_BCKL, 1);
    
    //Can't detect LCD type so assume st7735s
    printf("Initialising LCD on CS=%d\n",chip);
    
    switch (chip)
    { case PIN_CS_ST7735S:
        printf("Initialising ST7735S LCD\n");
        lcd_init_cmds = st7735s_InitSequence;
        break;
      case PIN_CS_GC9A01:
        printf("Initialising GC9A01 LCD\n");
        lcd_init_cmds = gc9a01_InitSequence;
        break;
      default:
        printf("Unknown LCD type\n");
        lcd_init_cmds = st7735s_InitSequence; // wrong answer
        break;
    }
    
    while (lcd_init_cmds[cmd].databytes != INIT_CMD_END_SEQUENCE ) {
        lcd_cmd(spi, lcd_init_cmds[cmd].cmd, false);
        lcd_data(spi, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes&0x1F);
        if (lcd_init_cmds[cmd].databytes & INIT_CMD_DELAY_AFTER ) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        cmd++;
    }
}

/* To send a set of lines we have to send a command, 2 data bytes, another command, 2 more data bytes and another command
 * before sending the line data itself; a total of 6 transactions. (We can't put all of this in just one transaction
 * because the D/C line needs to be toggled in the middle.)
 * This routine queues these commands up as interrupt transactions so they get
 * sent faster (compared to calling spi_device_transmit several times), and at
 * the mean while the lines for next transactions can get calculated.
 */
static void send_lines_st7735s(spi_device_handle_t spi, int ypos, uint16_t *linedata)
{
    esp_err_t ret;
    int x;
    //Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this
    //function is finished because the SPI driver needs access to it even while we're already calculating the next line.
    static spi_transaction_t trans[6];

    //In theory, it's better to initialize trans and data only once and hang on to the initialized
    //variables. We allocate them on the stack, so we need to re-init them each call.
    for (x=0; x<6; x++) {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if ((x&1)==0) {
            //Even transfers are commands
            trans[x].length=8;
            trans[x].user=(void*)0;
        } else {
            //Odd transfers are data
            trans[x].length=8*4;
            trans[x].user=(void*)1;
        }
        trans[x].flags=SPI_TRANS_USE_TXDATA;
    }
    trans[0].tx_data[0]=0x2A;           //Column Address Set
    trans[1].tx_data[0]=0;              //Start Col High
    trans[1].tx_data[1]=0;              //Start Col Low
    trans[1].tx_data[2]=(LCD_WIDTH)>>8;       //End Col High
    trans[1].tx_data[3]=(LCD_WIDTH)&0xff;     //End Col Low
    trans[2].tx_data[0]=0x2B;           //Page address set
    trans[3].tx_data[0]=ypos>>8;        //Start page high
    trans[3].tx_data[1]=ypos&0xff;      //start page low
    trans[3].tx_data[2]=(ypos+PARALLEL_LINES)>>8;    //end page high
    trans[3].tx_data[3]=(ypos+PARALLEL_LINES)&0xff;  //end page low
    trans[4].tx_data[0]=0x2C;           //memory write
    trans[5].tx_buffer=linedata;        //finally send the line data
    trans[5].length=LCD_WIDTH*2*8*PARALLEL_LINES;          //Data length, in bits
    trans[5].flags=0; //undo SPI_TRANS_USE_TXDATA flag

    //Queue all transactions.
    for (x=0; x<6; x++) {
        ret=spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
        assert(ret==ESP_OK);
    }

    //When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens
    //mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to
    //finish because we may as well spend the time calculating the next line. When that is done, we can call
    //send_line_finish, which will wait for the transfers to be done and check their status.
}


static void send_line_finish(spi_device_handle_t spi)
{
    spi_transaction_t *rtrans;
    esp_err_t ret;
    //Wait for all 6 transactions to be done and get back the results.
    for (int x=0; x<6; x++) {
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
        //We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}


//Simple routine to generate some patterns and send them to the LCD. Don't expect anything too
//impressive. Because the SPI driver handles transactions in the background, we can calculate the next line
//while the previous one is being sent.
static void display_pretty_colors(spi_device_handle_t spi, int chip)
{
    uint16_t *lines[2];
    //Allocate memory for the pixel buffers
    for (int i=0; i<2; i++) {
        lines[i]=heap_caps_malloc(LCD_WIDTH * PARALLEL_LINES*sizeof(uint16_t), MALLOC_CAP_DMA);
        assert(lines[i]!=NULL);
    }
    int frame=0;
    //Indexes of the line currently being sent to the LCD and the line we're calculating.
    int sending_line=-1;
    int calc_line=0;

    while(1) {
        frame++;
        for (int y=0; y < LCD_HEIGHT; y+=PARALLEL_LINES) {
            //Calculate a line.
            pretty_effect_calc_lines(lines[calc_line], y, frame, PARALLEL_LINES);
            //Finish up the sending process of the previous line, if any
            if (sending_line!=-1) send_line_finish(spi);
            //Swap sending_line and calc_line
            sending_line=calc_line;
            calc_line=(calc_line==1)?0:1;
            //Send the line we currently calculated.
            send_lines_st7735s(spi, y, lines[sending_line]);
            //The line set is queued up for sending now; the actual sending happens in the
            //background. We can go on to calculate the next line set as long as we do not
            //touch line[sending_line]; the SPI sending process is still reading from that.
        }
    }
}

static void rgb_stripe(spi_device_handle_t spi, int chip)
{
    // one pixel buffer
    uint16_t *lines = heap_caps_malloc(LCD_WIDTH * PARALLEL_LINES * sizeof(uint16_t), MALLOC_CAP_DMA);
    uint16_t *dest;
    int y;
    dest = lines;
    int stripeMode = 0;
    /* | RGB565 | primary colours | Wrong-endian |
       | 0xf800 | red    | 0x00f8 |
       | 0x07e0 | green  | 0xe007 |
       | 0x001f | blue   | 0x1f00 |
       | 0xffe0 | yellow | 0xe0ff |
       | 0x87ff | cyan   | 0xff87 |
       | 0xf81f | magenta| 0x1ff8 |
    */
    for (int yb=0; yb<LCD_HEIGHT; yb+= PARALLEL_LINES) {
        dest = lines;
        for (y = 0; y < PARALLEL_LINES; y++) {
            switch (stripeMode)
            {
                case 0:
                    for (int x=0; x < LCD_WIDTH; x++) {
                        *dest++= 0xffff; // white
                    }
                    break;
                case 1:
                    for (int x=0; x < LCD_WIDTH; x++) {
                        *dest++= 0x0000; // black
                    }
                    break;
                case 2:
                    for (int x=0; x < LCD_WIDTH; x++) {
                        *dest++= 0x00f8; // red, wrong-endian RGB565
                    }
                    break;
                case 3:
                    for (int x=0; x < LCD_WIDTH; x++) {
                        *dest++= 0xe007; // green
                    }
                    break;
                case 4:
                    for (int x=0; x < LCD_WIDTH; x++) {
                        *dest++= 0x1f00; // blue
                    }
                    break;
                case 5:
                    for (int x=0; x < LCD_WIDTH; x++) {
                        *dest++= 0x0eff; // yellow
                    }
                    break;
                case 6:
                    for (int x=0; x < LCD_WIDTH; x++) {
                        *dest++= 0xff87; // cyan
                    }
                    break;
                case 7:
                    for (int x=0; x < LCD_WIDTH; x++) {
                        *dest++= 0x1ff8; // magenta
                    }
                    break;
                case 8:
                    for (int x=0; x < LCD_WIDTH; x++) {
                        int shade = (x<<5) / LCD_WIDTH;
                        *dest++= (shade<<8) | shade << 3; // fade magenta
                    }
                    break;
                default:
                    for (int x=0; x < LCD_WIDTH; x++) {
                        *dest = 0xffff;
                    }
                    break;
            }
        }
        send_lines_st7735s(spi, yb, lines);
        send_line_finish(spi); // could do process next block of data while this is happening
        stripeMode++;
        if (stripeMode >= 9) 
            stripeMode = 0;
    }
}

void app_main(void)
{
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=PARALLEL_LINES*320*2+8
    };
    spi_device_interface_config_t devcfg_st={
        .clock_speed_hz=10*1000*1000,           //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_CS_ST7735S,               //CS pin for rectangular LCD
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(LCD_HOST, &devcfg_st, &spi);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t devcfg_gc={
        .clock_speed_hz=10*1000*1000,           //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_CS_GC9A01,               //CS pin for round LCD
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    ESP_LOGI("lcd", "Adding second spi_bus_add_device()");
    ret=spi_bus_add_device(LCD_HOST, &devcfg_gc, &spi); // not sure how to tell the other functions which device we want to use.
    ESP_ERROR_CHECK(ret);

    //Initialize the LCD
    lcd_init(spi, PIN_CS_ST7735S);
    lcd_init(spi, PIN_CS_GC9A01);

    ESP_LOGE("lcd", "Drawing RGB stripes");
    rgb_stripe(spi, PIN_CS_ST7735S);
    vTaskDelay(500);

    //Initialize the effect displayed
    ESP_LOGE("lcd", "Animating pretty effect");

    ret=pretty_effect_init();
    ESP_ERROR_CHECK(ret);

    //Go do nice stuff.
    display_pretty_colors(spi, PIN_CS_ST7735S); // infinite loop animation
}
