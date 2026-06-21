#ifndef __GC9A01_H_
#define __GC9A01_H_
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include "fonts.h"
#include "nrf_drv_spi.h"
// Command codes:
#define COL_ADDR_SET        0x2A
#define ROW_ADDR_SET        0x2B
#define MEM_WR              0x2C
#define MEM_WR_CONT         0x3C
#define COLOR_MODE          0x3A
#define COLOR_MODE__12_BIT  0x03
#define COLOR_MODE__16_BIT  0x05
#define COLOR_MODE__18_BIT  0x06
#define SLPIN               0x10
#define SLPOUT              0x11

// Pins — board pin map (LCD_*_Pin) centralized in main.h.

// Colors
#define	BLACK               0x0000
#define	BLUE                0x001F
#define	RED                 0xF800
#define	GREEN               0x07E0
#define CYAN                0x07FF
#define MAGENTA             0xF81F
#define YELLOW              0xFFE0
#define WHITE               0xFFFF
// Display size
#define GC9A01A_Width       240
#define GC9A01A_Height      240
// Func
#define swap(a,b) {int16_t t=a;a=b;b=t;}
#define RGB565(r, g, b)         (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

/* Structures */
struct GC9A01_draw_prop {
    uint16_t TextColor;
    uint16_t BackColor;
    sFONT *pFont;
};

struct GC9A01_point {
    uint16_t X, Y;
};

struct GC9A01_frame {
    struct GC9A01_point start, end;
};

/* Hardware and soft func  */
bool lcd_spi_init(void);   /* returns false if SPI init fails (LCD not present) */
void GC9A01_init(void);
void GC9A01A_sleep_mode(uint8_t Mode);
void GC9A01_spi_tx(uint8_t *data, size_t len);
/* Set display rotation: 0=0°, 1=90°, 2=180°, 3=270°. Default after init is 2. */
void GC9A01_set_rotation(uint8_t rotation);

/* Display write func */
void GC9A01_write(uint8_t *data, size_t len);
void GC9A01_write_continue(uint8_t *data, size_t len);
void GC9A01_write_data(uint8_t *data, size_t len);
void GC9A01_write_command(uint8_t cmd);

/* Display picture func */
void GC9A01_set_frame(struct GC9A01_frame frame);
void GC9A01_fonts_init(void);
void GC9A01_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void GC9A01_set_text_color(uint16_t color);
void GC9A01_set_back_color(uint16_t color);
void GC9A01_set_font(sFONT *pFonts);
void GC9A01_draw_pixel(int x, int y, uint16_t color);
void GC9A01_draw_line(uint16_t color, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void GC9A01_draw_circle(uint16_t x0, uint16_t y0, int r, uint16_t color);
void GC9A01_fill_circle(int16_t x, int16_t y, int16_t radius, uint16_t color);
void GC9A01_draw_char(uint16_t x, uint16_t y, uint8_t c);
void GC9A01_draw_string(uint16_t x,uint16_t y, char *str);
void GC9A01_printf(int16_t X, int16_t Y, const char *args, ...);

#endif
