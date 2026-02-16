################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/tests/src/test_cases/_test_template.c \
../Display_Driver/lvgl/tests/src/test_cases/test_align_flex.c \
../Display_Driver/lvgl/tests/src/test_cases/test_anim.c \
../Display_Driver/lvgl/tests/src/test_cases/test_anim_timeline.c \
../Display_Driver/lvgl/tests/src/test_cases/test_area.c \
../Display_Driver/lvgl/tests/src/test_cases/test_array.c \
../Display_Driver/lvgl/tests/src/test_cases/test_bindings.c \
../Display_Driver/lvgl/tests/src/test_cases/test_circle_buf.c \
../Display_Driver/lvgl/tests/src/test_cases/test_click.c \
../Display_Driver/lvgl/tests/src/test_cases/test_config.c \
../Display_Driver/lvgl/tests/src/test_cases/test_demo_stress.c \
../Display_Driver/lvgl/tests/src/test_cases/test_demo_widgets.c \
../Display_Driver/lvgl/tests/src/test_cases/test_display.c \
../Display_Driver/lvgl/tests/src/test_cases/test_draw_buf.c \
../Display_Driver/lvgl/tests/src/test_cases/test_event.c \
../Display_Driver/lvgl/tests/src/test_cases/test_event_trickle.c \
../Display_Driver/lvgl/tests/src/test_cases/test_file_explorer.c \
../Display_Driver/lvgl/tests/src/test_cases/test_font_loader.c \
../Display_Driver/lvgl/tests/src/test_cases/test_font_manager.c \
../Display_Driver/lvgl/tests/src/test_cases/test_fs.c \
../Display_Driver/lvgl/tests/src/test_cases/test_gesture_pinch.c \
../Display_Driver/lvgl/tests/src/test_cases/test_grid.c \
../Display_Driver/lvgl/tests/src/test_cases/test_grid_fr.c \
../Display_Driver/lvgl/tests/src/test_cases/test_gridnav.c \
../Display_Driver/lvgl/tests/src/test_cases/test_group.c \
../Display_Driver/lvgl/tests/src/test_cases/test_hover.c \
../Display_Driver/lvgl/tests/src/test_cases/test_indev.c \
../Display_Driver/lvgl/tests/src/test_cases/test_lcd.c \
../Display_Driver/lvgl/tests/src/test_cases/test_margin_align.c \
../Display_Driver/lvgl/tests/src/test_cases/test_margin_flex.c \
../Display_Driver/lvgl/tests/src/test_cases/test_margin_grid.c \
../Display_Driver/lvgl/tests/src/test_cases/test_math.c \
../Display_Driver/lvgl/tests/src/test_cases/test_mem.c \
../Display_Driver/lvgl/tests/src/test_cases/test_observer.c \
../Display_Driver/lvgl/tests/src/test_cases/test_profiler.c \
../Display_Driver/lvgl/tests/src/test_cases/test_recolor.c \
../Display_Driver/lvgl/tests/src/test_cases/test_screen_load.c \
../Display_Driver/lvgl/tests/src/test_cases/test_snapshot.c \
../Display_Driver/lvgl/tests/src/test_cases/test_style.c \
../Display_Driver/lvgl/tests/src/test_cases/test_svg.c \
../Display_Driver/lvgl/tests/src/test_cases/test_svg_anim.c \
../Display_Driver/lvgl/tests/src/test_cases/test_translation.c \
../Display_Driver/lvgl/tests/src/test_cases/test_tree.c \
../Display_Driver/lvgl/tests/src/test_cases/test_txt.c 

OBJS += \
./Display_Driver/lvgl/tests/src/test_cases/_test_template.o \
./Display_Driver/lvgl/tests/src/test_cases/test_align_flex.o \
./Display_Driver/lvgl/tests/src/test_cases/test_anim.o \
./Display_Driver/lvgl/tests/src/test_cases/test_anim_timeline.o \
./Display_Driver/lvgl/tests/src/test_cases/test_area.o \
./Display_Driver/lvgl/tests/src/test_cases/test_array.o \
./Display_Driver/lvgl/tests/src/test_cases/test_bindings.o \
./Display_Driver/lvgl/tests/src/test_cases/test_circle_buf.o \
./Display_Driver/lvgl/tests/src/test_cases/test_click.o \
./Display_Driver/lvgl/tests/src/test_cases/test_config.o \
./Display_Driver/lvgl/tests/src/test_cases/test_demo_stress.o \
./Display_Driver/lvgl/tests/src/test_cases/test_demo_widgets.o \
./Display_Driver/lvgl/tests/src/test_cases/test_display.o \
./Display_Driver/lvgl/tests/src/test_cases/test_draw_buf.o \
./Display_Driver/lvgl/tests/src/test_cases/test_event.o \
./Display_Driver/lvgl/tests/src/test_cases/test_event_trickle.o \
./Display_Driver/lvgl/tests/src/test_cases/test_file_explorer.o \
./Display_Driver/lvgl/tests/src/test_cases/test_font_loader.o \
./Display_Driver/lvgl/tests/src/test_cases/test_font_manager.o \
./Display_Driver/lvgl/tests/src/test_cases/test_fs.o \
./Display_Driver/lvgl/tests/src/test_cases/test_gesture_pinch.o \
./Display_Driver/lvgl/tests/src/test_cases/test_grid.o \
./Display_Driver/lvgl/tests/src/test_cases/test_grid_fr.o \
./Display_Driver/lvgl/tests/src/test_cases/test_gridnav.o \
./Display_Driver/lvgl/tests/src/test_cases/test_group.o \
./Display_Driver/lvgl/tests/src/test_cases/test_hover.o \
./Display_Driver/lvgl/tests/src/test_cases/test_indev.o \
./Display_Driver/lvgl/tests/src/test_cases/test_lcd.o \
./Display_Driver/lvgl/tests/src/test_cases/test_margin_align.o \
./Display_Driver/lvgl/tests/src/test_cases/test_margin_flex.o \
./Display_Driver/lvgl/tests/src/test_cases/test_margin_grid.o \
./Display_Driver/lvgl/tests/src/test_cases/test_math.o \
./Display_Driver/lvgl/tests/src/test_cases/test_mem.o \
./Display_Driver/lvgl/tests/src/test_cases/test_observer.o \
./Display_Driver/lvgl/tests/src/test_cases/test_profiler.o \
./Display_Driver/lvgl/tests/src/test_cases/test_recolor.o \
./Display_Driver/lvgl/tests/src/test_cases/test_screen_load.o \
./Display_Driver/lvgl/tests/src/test_cases/test_snapshot.o \
./Display_Driver/lvgl/tests/src/test_cases/test_style.o \
./Display_Driver/lvgl/tests/src/test_cases/test_svg.o \
./Display_Driver/lvgl/tests/src/test_cases/test_svg_anim.o \
./Display_Driver/lvgl/tests/src/test_cases/test_translation.o \
./Display_Driver/lvgl/tests/src/test_cases/test_tree.o \
./Display_Driver/lvgl/tests/src/test_cases/test_txt.o 

C_DEPS += \
./Display_Driver/lvgl/tests/src/test_cases/_test_template.d \
./Display_Driver/lvgl/tests/src/test_cases/test_align_flex.d \
./Display_Driver/lvgl/tests/src/test_cases/test_anim.d \
./Display_Driver/lvgl/tests/src/test_cases/test_anim_timeline.d \
./Display_Driver/lvgl/tests/src/test_cases/test_area.d \
./Display_Driver/lvgl/tests/src/test_cases/test_array.d \
./Display_Driver/lvgl/tests/src/test_cases/test_bindings.d \
./Display_Driver/lvgl/tests/src/test_cases/test_circle_buf.d \
./Display_Driver/lvgl/tests/src/test_cases/test_click.d \
./Display_Driver/lvgl/tests/src/test_cases/test_config.d \
./Display_Driver/lvgl/tests/src/test_cases/test_demo_stress.d \
./Display_Driver/lvgl/tests/src/test_cases/test_demo_widgets.d \
./Display_Driver/lvgl/tests/src/test_cases/test_display.d \
./Display_Driver/lvgl/tests/src/test_cases/test_draw_buf.d \
./Display_Driver/lvgl/tests/src/test_cases/test_event.d \
./Display_Driver/lvgl/tests/src/test_cases/test_event_trickle.d \
./Display_Driver/lvgl/tests/src/test_cases/test_file_explorer.d \
./Display_Driver/lvgl/tests/src/test_cases/test_font_loader.d \
./Display_Driver/lvgl/tests/src/test_cases/test_font_manager.d \
./Display_Driver/lvgl/tests/src/test_cases/test_fs.d \
./Display_Driver/lvgl/tests/src/test_cases/test_gesture_pinch.d \
./Display_Driver/lvgl/tests/src/test_cases/test_grid.d \
./Display_Driver/lvgl/tests/src/test_cases/test_grid_fr.d \
./Display_Driver/lvgl/tests/src/test_cases/test_gridnav.d \
./Display_Driver/lvgl/tests/src/test_cases/test_group.d \
./Display_Driver/lvgl/tests/src/test_cases/test_hover.d \
./Display_Driver/lvgl/tests/src/test_cases/test_indev.d \
./Display_Driver/lvgl/tests/src/test_cases/test_lcd.d \
./Display_Driver/lvgl/tests/src/test_cases/test_margin_align.d \
./Display_Driver/lvgl/tests/src/test_cases/test_margin_flex.d \
./Display_Driver/lvgl/tests/src/test_cases/test_margin_grid.d \
./Display_Driver/lvgl/tests/src/test_cases/test_math.d \
./Display_Driver/lvgl/tests/src/test_cases/test_mem.d \
./Display_Driver/lvgl/tests/src/test_cases/test_observer.d \
./Display_Driver/lvgl/tests/src/test_cases/test_profiler.d \
./Display_Driver/lvgl/tests/src/test_cases/test_recolor.d \
./Display_Driver/lvgl/tests/src/test_cases/test_screen_load.d \
./Display_Driver/lvgl/tests/src/test_cases/test_snapshot.d \
./Display_Driver/lvgl/tests/src/test_cases/test_style.d \
./Display_Driver/lvgl/tests/src/test_cases/test_svg.d \
./Display_Driver/lvgl/tests/src/test_cases/test_svg_anim.d \
./Display_Driver/lvgl/tests/src/test_cases/test_translation.d \
./Display_Driver/lvgl/tests/src/test_cases/test_tree.d \
./Display_Driver/lvgl/tests/src/test_cases/test_txt.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/tests/src/test_cases/%.o Display_Driver/lvgl/tests/src/test_cases/%.su Display_Driver/lvgl/tests/src/test_cases/%.cyclo: ../Display_Driver/lvgl/tests/src/test_cases/%.c Display_Driver/lvgl/tests/src/test_cases/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-tests-2f-src-2f-test_cases

clean-Display_Driver-2f-lvgl-2f-tests-2f-src-2f-test_cases:
	-$(RM) ./Display_Driver/lvgl/tests/src/test_cases/_test_template.cyclo ./Display_Driver/lvgl/tests/src/test_cases/_test_template.d ./Display_Driver/lvgl/tests/src/test_cases/_test_template.o ./Display_Driver/lvgl/tests/src/test_cases/_test_template.su ./Display_Driver/lvgl/tests/src/test_cases/test_align_flex.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_align_flex.d ./Display_Driver/lvgl/tests/src/test_cases/test_align_flex.o ./Display_Driver/lvgl/tests/src/test_cases/test_align_flex.su ./Display_Driver/lvgl/tests/src/test_cases/test_anim.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_anim.d ./Display_Driver/lvgl/tests/src/test_cases/test_anim.o ./Display_Driver/lvgl/tests/src/test_cases/test_anim.su ./Display_Driver/lvgl/tests/src/test_cases/test_anim_timeline.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_anim_timeline.d ./Display_Driver/lvgl/tests/src/test_cases/test_anim_timeline.o ./Display_Driver/lvgl/tests/src/test_cases/test_anim_timeline.su ./Display_Driver/lvgl/tests/src/test_cases/test_area.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_area.d ./Display_Driver/lvgl/tests/src/test_cases/test_area.o ./Display_Driver/lvgl/tests/src/test_cases/test_area.su ./Display_Driver/lvgl/tests/src/test_cases/test_array.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_array.d ./Display_Driver/lvgl/tests/src/test_cases/test_array.o ./Display_Driver/lvgl/tests/src/test_cases/test_array.su ./Display_Driver/lvgl/tests/src/test_cases/test_bindings.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_bindings.d ./Display_Driver/lvgl/tests/src/test_cases/test_bindings.o ./Display_Driver/lvgl/tests/src/test_cases/test_bindings.su ./Display_Driver/lvgl/tests/src/test_cases/test_circle_buf.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_circle_buf.d ./Display_Driver/lvgl/tests/src/test_cases/test_circle_buf.o ./Display_Driver/lvgl/tests/src/test_cases/test_circle_buf.su ./Display_Driver/lvgl/tests/src/test_cases/test_click.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_click.d ./Display_Driver/lvgl/tests/src/test_cases/test_click.o ./Display_Driver/lvgl/tests/src/test_cases/test_click.su ./Display_Driver/lvgl/tests/src/test_cases/test_config.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_config.d ./Display_Driver/lvgl/tests/src/test_cases/test_config.o ./Display_Driver/lvgl/tests/src/test_cases/test_config.su ./Display_Driver/lvgl/tests/src/test_cases/test_demo_stress.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_demo_stress.d ./Display_Driver/lvgl/tests/src/test_cases/test_demo_stress.o ./Display_Driver/lvgl/tests/src/test_cases/test_demo_stress.su ./Display_Driver/lvgl/tests/src/test_cases/test_demo_widgets.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_demo_widgets.d ./Display_Driver/lvgl/tests/src/test_cases/test_demo_widgets.o ./Display_Driver/lvgl/tests/src/test_cases/test_demo_widgets.su ./Display_Driver/lvgl/tests/src/test_cases/test_display.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_display.d ./Display_Driver/lvgl/tests/src/test_cases/test_display.o ./Display_Driver/lvgl/tests/src/test_cases/test_display.su ./Display_Driver/lvgl/tests/src/test_cases/test_draw_buf.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_draw_buf.d ./Display_Driver/lvgl/tests/src/test_cases/test_draw_buf.o ./Display_Driver/lvgl/tests/src/test_cases/test_draw_buf.su ./Display_Driver/lvgl/tests/src/test_cases/test_event.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_event.d ./Display_Driver/lvgl/tests/src/test_cases/test_event.o ./Display_Driver/lvgl/tests/src/test_cases/test_event.su ./Display_Driver/lvgl/tests/src/test_cases/test_event_trickle.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_event_trickle.d ./Display_Driver/lvgl/tests/src/test_cases/test_event_trickle.o ./Display_Driver/lvgl/tests/src/test_cases/test_event_trickle.su ./Display_Driver/lvgl/tests/src/test_cases/test_file_explorer.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_file_explorer.d ./Display_Driver/lvgl/tests/src/test_cases/test_file_explorer.o ./Display_Driver/lvgl/tests/src/test_cases/test_file_explorer.su ./Display_Driver/lvgl/tests/src/test_cases/test_font_loader.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_font_loader.d ./Display_Driver/lvgl/tests/src/test_cases/test_font_loader.o ./Display_Driver/lvgl/tests/src/test_cases/test_font_loader.su ./Display_Driver/lvgl/tests/src/test_cases/test_font_manager.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_font_manager.d ./Display_Driver/lvgl/tests/src/test_cases/test_font_manager.o ./Display_Driver/lvgl/tests/src/test_cases/test_font_manager.su ./Display_Driver/lvgl/tests/src/test_cases/test_fs.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_fs.d ./Display_Driver/lvgl/tests/src/test_cases/test_fs.o ./Display_Driver/lvgl/tests/src/test_cases/test_fs.su ./Display_Driver/lvgl/tests/src/test_cases/test_gesture_pinch.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_gesture_pinch.d ./Display_Driver/lvgl/tests/src/test_cases/test_gesture_pinch.o ./Display_Driver/lvgl/tests/src/test_cases/test_gesture_pinch.su ./Display_Driver/lvgl/tests/src/test_cases/test_grid.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_grid.d ./Display_Driver/lvgl/tests/src/test_cases/test_grid.o ./Display_Driver/lvgl/tests/src/test_cases/test_grid.su ./Display_Driver/lvgl/tests/src/test_cases/test_grid_fr.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_grid_fr.d ./Display_Driver/lvgl/tests/src/test_cases/test_grid_fr.o ./Display_Driver/lvgl/tests/src/test_cases/test_grid_fr.su ./Display_Driver/lvgl/tests/src/test_cases/test_gridnav.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_gridnav.d ./Display_Driver/lvgl/tests/src/test_cases/test_gridnav.o ./Display_Driver/lvgl/tests/src/test_cases/test_gridnav.su ./Display_Driver/lvgl/tests/src/test_cases/test_group.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_group.d ./Display_Driver/lvgl/tests/src/test_cases/test_group.o
	-$(RM) ./Display_Driver/lvgl/tests/src/test_cases/test_group.su ./Display_Driver/lvgl/tests/src/test_cases/test_hover.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_hover.d ./Display_Driver/lvgl/tests/src/test_cases/test_hover.o ./Display_Driver/lvgl/tests/src/test_cases/test_hover.su ./Display_Driver/lvgl/tests/src/test_cases/test_indev.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_indev.d ./Display_Driver/lvgl/tests/src/test_cases/test_indev.o ./Display_Driver/lvgl/tests/src/test_cases/test_indev.su ./Display_Driver/lvgl/tests/src/test_cases/test_lcd.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_lcd.d ./Display_Driver/lvgl/tests/src/test_cases/test_lcd.o ./Display_Driver/lvgl/tests/src/test_cases/test_lcd.su ./Display_Driver/lvgl/tests/src/test_cases/test_margin_align.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_margin_align.d ./Display_Driver/lvgl/tests/src/test_cases/test_margin_align.o ./Display_Driver/lvgl/tests/src/test_cases/test_margin_align.su ./Display_Driver/lvgl/tests/src/test_cases/test_margin_flex.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_margin_flex.d ./Display_Driver/lvgl/tests/src/test_cases/test_margin_flex.o ./Display_Driver/lvgl/tests/src/test_cases/test_margin_flex.su ./Display_Driver/lvgl/tests/src/test_cases/test_margin_grid.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_margin_grid.d ./Display_Driver/lvgl/tests/src/test_cases/test_margin_grid.o ./Display_Driver/lvgl/tests/src/test_cases/test_margin_grid.su ./Display_Driver/lvgl/tests/src/test_cases/test_math.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_math.d ./Display_Driver/lvgl/tests/src/test_cases/test_math.o ./Display_Driver/lvgl/tests/src/test_cases/test_math.su ./Display_Driver/lvgl/tests/src/test_cases/test_mem.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_mem.d ./Display_Driver/lvgl/tests/src/test_cases/test_mem.o ./Display_Driver/lvgl/tests/src/test_cases/test_mem.su ./Display_Driver/lvgl/tests/src/test_cases/test_observer.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_observer.d ./Display_Driver/lvgl/tests/src/test_cases/test_observer.o ./Display_Driver/lvgl/tests/src/test_cases/test_observer.su ./Display_Driver/lvgl/tests/src/test_cases/test_profiler.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_profiler.d ./Display_Driver/lvgl/tests/src/test_cases/test_profiler.o ./Display_Driver/lvgl/tests/src/test_cases/test_profiler.su ./Display_Driver/lvgl/tests/src/test_cases/test_recolor.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_recolor.d ./Display_Driver/lvgl/tests/src/test_cases/test_recolor.o ./Display_Driver/lvgl/tests/src/test_cases/test_recolor.su ./Display_Driver/lvgl/tests/src/test_cases/test_screen_load.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_screen_load.d ./Display_Driver/lvgl/tests/src/test_cases/test_screen_load.o ./Display_Driver/lvgl/tests/src/test_cases/test_screen_load.su ./Display_Driver/lvgl/tests/src/test_cases/test_snapshot.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_snapshot.d ./Display_Driver/lvgl/tests/src/test_cases/test_snapshot.o ./Display_Driver/lvgl/tests/src/test_cases/test_snapshot.su ./Display_Driver/lvgl/tests/src/test_cases/test_style.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_style.d ./Display_Driver/lvgl/tests/src/test_cases/test_style.o ./Display_Driver/lvgl/tests/src/test_cases/test_style.su ./Display_Driver/lvgl/tests/src/test_cases/test_svg.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_svg.d ./Display_Driver/lvgl/tests/src/test_cases/test_svg.o ./Display_Driver/lvgl/tests/src/test_cases/test_svg.su ./Display_Driver/lvgl/tests/src/test_cases/test_svg_anim.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_svg_anim.d ./Display_Driver/lvgl/tests/src/test_cases/test_svg_anim.o ./Display_Driver/lvgl/tests/src/test_cases/test_svg_anim.su ./Display_Driver/lvgl/tests/src/test_cases/test_translation.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_translation.d ./Display_Driver/lvgl/tests/src/test_cases/test_translation.o ./Display_Driver/lvgl/tests/src/test_cases/test_translation.su ./Display_Driver/lvgl/tests/src/test_cases/test_tree.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_tree.d ./Display_Driver/lvgl/tests/src/test_cases/test_tree.o ./Display_Driver/lvgl/tests/src/test_cases/test_tree.su ./Display_Driver/lvgl/tests/src/test_cases/test_txt.cyclo ./Display_Driver/lvgl/tests/src/test_cases/test_txt.d ./Display_Driver/lvgl/tests/src/test_cases/test_txt.o ./Display_Driver/lvgl/tests/src/test_cases/test_txt.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-tests-2f-src-2f-test_cases

