/* lv_port_indev.c — LVGL v9 */
#include "lvgl.h"
#include "../../icode/lcd/touch.h"

#include <stdbool.h>
#include <stdint.h>


/* ========= Shared Touch State =========
   This is the minimal state LVGL needs: pressed flag + mapped point in display coordinates.
   Your touch driver (Step 3) must update these via the weak setters below (or you can link directly).
*/
typedef struct {
    bool pressed;
    lv_point_t p;   // Already mapped to display coordinates
} touch_state_t;

static touch_state_t s_touch = { .pressed = false, .p = {0, 0} };

/* ========= Weak setters for hardware layer =========
   Implement these in your touch driver (e.g., Drivers/YourTouch/touch_driver.c):
   - touch_port_set_point(x, y)
   - touch_port_set_pressed(down)
   If you prefer direct linking, you can remove 'weak' and call these functions directly.
*/
__attribute__((weak)) void touch_port_set_point(uint16_t x, uint16_t y)
{
    s_touch.p.x = (lv_coord_t)x;
    s_touch.p.y = (lv_coord_t)y;
}

__attribute__((weak)) void touch_port_set_pressed(bool down)
{
    s_touch.pressed = down;
}

/* ========= LVGL read callback =========
   This reads the latest state and returns it to LVGL.
   IMPORTANT: Do NOT do I2C or any blocking operations here.
*/
static void lv_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    (void)indev;
    data->point.x = s_touch.p.x;
    data->point.y = s_touch.p.y;
    data->state   = s_touch.pressed ? LV_INDEV_STATE_PRESSED
                                    : LV_INDEV_STATE_RELEASED;
}

/* ========= Public init =========
   Call this once after lv_init() and after your display driver is created.
   Example (in your app init):
       lvgl_port_indev_init();
*/
void lvgl_port_indev_init(void)
{
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, lv_touch_read_cb);
}

/* ========= Optional: simple trace (diagnostics) =========
   Create a periodic LVGL timer to log pointer state (useful while bringing up).
   Call lvgl_port_indev_trace_enable(); when you want logs.
*/
#if LV_USE_LOG
static void indev_trace_task(lv_timer_t *t)
{
    (void)t;
    static uint32_t last = 0;
    uint32_t now = lv_tick_get();
    if(now - last >= 200) {
        last = now;
        LV_LOG_INFO("touch: %d,%d %s\n",
                    (int)s_touch.p.x, (int)s_touch.p.y,
                    s_touch.pressed ? "DOWN" : "UP");
    }
}

void lvgl_port_indev_trace_enable(void)
{
    lv_timer_create(indev_trace_task, 50, NULL); // ~20 Hz logs
}
#endif


// --- Set this to match your display port
#define DISP_HOR_RES   LCD_Width
#define DISP_VER_RES   LCD_Height

// Choose orientation once; your driver handles the transform internally.
#define TOUCH_DIR      Landscape   // Portrait / Landscape / Portrait_reversal / Landscape_reversal
typedef struct {
    uint16_t x;
    uint16_t y;
    bool     pressed;
} touch_cache_t;

static touch_cache_t g_touch = {0, 0, false};

// Optional: reduce LVGL read period (default LV_DEF_REFR_PERIOD) if needed
static void indev_tune_period(lv_indev_t *indev, uint32_t period_ms) {
    lv_timer_t * t = lv_indev_get_read_timer(indev);
    if(t) lv_timer_set_period(t, period_ms);
}

// --- Call this from a 100 Hz task/timer (NOT in the LVGL read_cb)
void touch_poll_100hz(void) {
    // Your driver only reads when INT is low; calling TOUCH_Read(dir) is cheap and non-blocking
    TOUCH_Read(TOUCH_DIR);                                     // updates TOUCH_X[], TOUCH_Y[], TOUCH_STA (global)  [1](https://npicedukh-my.sharepoint.com/personal/thura_peou_npic_edu_kh/Documents/Microsoft%20Copilot%20Chat%20Files/touch.c)
    uint8_t count = TOUCH_STA & 0x0F;                          // lower 4 bits: number of points                  [1](https://npicedukh-my.sharepoint.com/personal/thura_peou_npic_edu_kh/Documents/Microsoft%20Copilot%20Chat%20Files/touch.c)
    bool pressed = count > 0;

    // Use the first touch point
    uint16_t x = TOUCH_X[0];
    uint16_t y = TOUCH_Y[0];

    // Clamp to LVGL display resolution to avoid warnings
    if(x >= DISP_HOR_RES) x = DISP_HOR_RES - 1;
    if(y >= DISP_VER_RES) y = DISP_VER_RES - 1;

    g_touch.x = x;
    g_touch.y = y;
    g_touch.pressed = pressed;
}

// --- LVGL v9 read callback: must be very fast, no I2C here
static void touch_read_cb(lv_indev_t * indev, lv_indev_data_t * data) {
    (void)indev;
    data->point.x = g_touch.x;
    data->point.y = g_touch.y;
    data->state   = g_touch.pressed ? LV_INDEV_STATE_PRESSED
                                    : LV_INDEV_STATE_RELEASED;
    data->continue_reading = false;    // no buffered events here
}

// --- Public init: call after lv_init() and display creation
lv_indev_t * lv_port_indev_init(void) {
    // Ensure GT9xxx was initialized elsewhere: TOUCH_Init() returns 1 on success  [1](https://npicedukh-my.sharepoint.com/personal/thura_peou_npic_edu_kh/Documents/Microsoft%20Copilot%20Chat%20Files/touch.c)
    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touch_read_cb);

    // Optional: faster read polling by LVGL (e.g., 10 ms)
    indev_tune_period(indev, 10);      // Change or remove as needed  [4](https://docs.lvgl.io/master/details/main-modules/indev/overview.html)
    return indev;
}
