#include "lvgl.h"
#include <stdio.h>

#if LV_USE_LOG
static void my_log_cb(const char *buf){ printf("%s", buf); }
#endif
static void on_btn_event(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_PRESSED)  LV_LOG_INFO("EV: PRESSED\n");
    if(code == LV_EVENT_PRESSING) LV_LOG_INFO("EV: PRESSING\n");
    if(code == LV_EVENT_RELEASED) LV_LOG_INFO("EV: RELEASED\n");
    if(code == LV_EVENT_CLICKED)  LV_LOG_INFO("EV: CLICKED\n");
}

void lv_app_create_test_ui(void)
{
#if LV_USE_LOG
    lv_log_register_print_cb(my_log_cb);  // ensure logs go to printf
#endif

    lv_obj_t *scr = lv_screen_active();

    // Solid background to verify flush
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x202530), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_100, 0);

    // Big button near top-left (easy target)
    lv_obj_t *btn = lv_button_create(scr);
    lv_obj_set_size(btn, 200, 100);
    lv_obj_set_pos(btn, 10, 10);
    lv_obj_add_event_cb(btn, on_btn_event, LV_EVENT_ALL, NULL);

    // Label on the button
    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, "Touch me");
    lv_obj_center(lbl);

    // Another visual element (to confirm rendering)
    lv_obj_t *box = lv_obj_create(scr);
    lv_obj_set_size(box, 100, 100);
    lv_obj_set_style_bg_color(box, lv_color_hex(0x2ecc71), 0);
    lv_obj_set_pos(box, 250, 10);
}
