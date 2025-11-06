#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize I2C2, GPIOs (INT/RESET), and the touch IC. */
void touch_driver_init(uint16_t disp_w, uint16_t disp_h,
                       bool swap_xy, bool invert_x, bool invert_y, uint8_t rotation90);

/** Call from the main loop at ~1–2 kHz or inside a 1 ms timer to service pending IRQ. */
void touch_driver_poll(void);

/** Optional: set display size later if needed. */
void touch_driver_set_display_size(uint16_t w, uint16_t h);

#ifdef __cplusplus
}
#endif
