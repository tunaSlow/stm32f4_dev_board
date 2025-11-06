/* lv_port_disp.c */
#include "../../icode/lcd/lcd.h"        /* your driver header with LCD_Write_Cursor, SET_GRAM, etc. */
#include "main.h"
#include "lvgl.h"/* for hsram1, LCD_DAT symbols if needed */


#define LCD_H_RES 800
#define LCD_V_RES 480

static lv_color_t s_buf1[LCD_H_RES * 2];
static lv_color_t s_buf2[LCD_H_RES * 2];

static void lcd_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    uint16_t x1 = (uint16_t)area->x1, x2 = (uint16_t)area->x2;
    uint16_t y1 = (uint16_t)area->y1, y2 = (uint16_t)area->y2;
    uint16_t w  = x2 - x1 + 1;

    const uint16_t *row = (const uint16_t *)px_map; // RGB565

    for(uint16_t y = y1; y <= y2; y++) {
        LCD_Write_Cursor(x1, y);
        LCD_Write_COM(SET_GRAM);
        for(uint16_t i = 0; i < w; i++) {
            uint16_t c = row[i];
            HAL_SRAM_Write_16b(&hsram1, LCD_DAT, &c, 1);
            // Or faster: *(volatile uint16_t*)LCD_DAT = c;  // if LCD_DAT is a mapped address
        }
        row += w;
    }
    lv_display_flush_ready(disp);
}

void lv_port_disp_init(void)
{
    lv_display_t *disp = lv_display_create(LCD_H_RES, LCD_V_RES);

    lv_display_set_buffers(disp,
                           s_buf1, s_buf2,
                           sizeof(s_buf1),      // bytes
                           LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_display_set_flush_cb(disp, lcd_flush_cb);
}

