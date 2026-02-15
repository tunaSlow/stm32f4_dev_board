#pragma once
#ifdef __cplusplus
extern "C" {
#endif

void LCD_SetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void my_disp_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map);
void my_touch_read(lv_indev_t * indev, lv_indev_data_t * data);
void btn_event_cb(lv_event_t * e);

#ifdef __cplusplus
}
#endif
