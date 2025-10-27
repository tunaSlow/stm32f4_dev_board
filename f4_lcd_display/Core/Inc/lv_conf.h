/* lv_conf.h */
#ifndef LV_CONF_H
#define LV_CONF_H

/*====================
   Color & Display
 *====================*/
#define LV_COLOR_DEPTH      16          /* RGB565 */
#define LV_COLOR_16_SWAP    0           /* FSMC writes MSB first -> 0 */
#define LV_HOR_RES_MAX      800
#define LV_VER_RES_MAX      480

/* Small draw buffer: a few lines to save RAM (2-4 lines recommended) */
#define LV_USE_DRAW_SW      1
#define LV_DRAW_SW_COMPLEX  1

/*====================
   OS/Timing
 *====================*/
#define LV_TICK_CUSTOM      1
#define LV_TICK_CUSTOM_INCLUDE "stm32f4xx_hal.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (HAL_GetTick())

#define LV_USE_LOG 1
#define LV_LOG_LEVEL LV_LOG_LEVEL_USER

/*====================
   Features
 *====================*
#define LV_USE_LOG          1
#define LV_LOG_LEVEL        LV_LOG_LEVEL_WARN

/* Widgets you’ll likely use */
#define LV_USE_BTN          1
#define LV_USE_LABEL        1
#define LV_USE_SLIDER       1
#define LV_USE_IMG          1
#define LV_USE_CANVAS       1
#define LV_USE_ARC          1

#endif /* LV_CONF_H */
