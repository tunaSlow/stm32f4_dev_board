#include "lvgl.h"
#include <stdio.h>
#include "../../icode/lcd/lcd.h"
#include "../../icode/lcd/touch.h"
#include "lvgl.h"
#include "lv_app.h"

// Add this function to lcd.c or main.c
// It sets the active window for drawing
void LCD_SetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    if(LCD_ID == LCD_ID_OTM8009) {
        // Based on your existing LCD_Write_Cursor logic
        LCD_Write_REG(SET_X, x1 >> 8);
        LCD_Write_REG(SET_X + 1, x1 & 0xFF);
        LCD_Write_REG(SET_X + 2, x2 >> 8);
        LCD_Write_REG(SET_X + 3, x2 & 0xFF);

        LCD_Write_REG(SET_Y, y1 >> 8);
        LCD_Write_REG(SET_Y + 1, y1 & 0xFF);
        LCD_Write_REG(SET_Y + 2, y2 >> 8);
        LCD_Write_REG(SET_Y + 3, y2 & 0xFF);
    } else {
        // For NT35510 / RM68120 (Standard commands usually 0x2A/0x2B)
        // Assuming SET_X and SET_Y map to Column/Page Address Set
        LCD_Write_REG(SET_X, x1 >> 8);
        LCD_Write_REG(SET_X + 1, x1 & 0xFF);
        LCD_Write_REG(SET_X + 2, x2 >> 8);
        LCD_Write_REG(SET_X + 3, x2 & 0xFF);

        LCD_Write_REG(SET_Y, y1 >> 8);
        LCD_Write_REG(SET_Y + 1, y1 & 0xFF);
        LCD_Write_REG(SET_Y + 2, y2 >> 8);
        LCD_Write_REG(SET_Y + 3, y2 & 0xFF);
    }
    // Prepare to write data (command 0x2C is standard for Write Memory)
    LCD_Write_COM(0x2C00);
}

/* -----------------------------------------------------------------------------
   1. FLUSH CALLBACK (Display Driver)
   ----------------------------------------------------------------------------- */
void my_disp_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map)
{
    // 1. Set the drawing window (Same as before)
    LCD_SetWindow(area->x1, area->y1, area->x2, area->y2);

    // 2. Calculate pixel count
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    uint32_t px_count = w * h;

    // 3. Cast pointer to 16-bit
    uint16_t * color_p = (uint16_t *)px_map;

    // --- OPTIMIZATION STARTS HERE ---

    // Instead of the 'for' loop, we send the whole block at once.
    // We access the external memory handle (hsram1) directly.
    // LCD_DAT is the address offset defined in your lcd.h for data writing.

    // NOTE: You might need to extern hsram1 if it's not visible here
    extern SRAM_HandleTypeDef hsram1;

    // Write all pixels in one high-speed burst
    // (Address, Data Pointer, Size)
    HAL_SRAM_Write_16b(&hsram1, (uint32_t *)LCD_DAT, color_p, px_count);

    // --- OPTIMIZATION ENDS HERE ---

    // 4. Inform LVGL we are done
    lv_display_flush_ready(disp);
}

/* -----------------------------------------------------------------------------
   2. INPUT READ CALLBACK (Touch Driver)
   ----------------------------------------------------------------------------- */
void my_touch_read(lv_indev_t * indev, lv_indev_data_t * data)
{
    // 1. Scan the hardware
    // Argument '1' usually stands for Landscape in these drivers.
    // If coordinates are swapped/inverted, try 0, 1, 2, or 3.
    // Ideally use the defined constant from touch.h like 'Landscape'
    TOUCH_Read(1);

    // 2. Check touch status
    // Your driver updates the global TOUCH_STA variable.
    // BIT7=1 means data ready, BIT0-3 = number of touch points.
    bool is_touched = (TOUCH_STA & 0x80) && ((TOUCH_STA & 0x0F) > 0);

    if(is_touched) {
        data->state = LV_INDEV_STATE_PRESSED;

        // 3. Get coordinates from global arrays
        data->point.x = TOUCH_X[0];
        data->point.y = TOUCH_Y[0];
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

/* -----------------------------------------------------------------------------
   3. EVENT CALLBACK (Button Logic)
   ----------------------------------------------------------------------------- */
void btn_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);

    if(code == LV_EVENT_CLICKED) {
        static uint8_t cnt = 0;
        cnt++;

        lv_obj_t * label = lv_obj_get_child(btn, 0);
        lv_label_set_text_fmt(label, "Clicked: %d", cnt);
    }
}
