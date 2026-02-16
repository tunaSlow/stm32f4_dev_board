################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/misc/lv_anim.c \
../Display_Driver/lvgl/src/misc/lv_anim_timeline.c \
../Display_Driver/lvgl/src/misc/lv_area.c \
../Display_Driver/lvgl/src/misc/lv_array.c \
../Display_Driver/lvgl/src/misc/lv_async.c \
../Display_Driver/lvgl/src/misc/lv_bidi.c \
../Display_Driver/lvgl/src/misc/lv_circle_buf.c \
../Display_Driver/lvgl/src/misc/lv_color.c \
../Display_Driver/lvgl/src/misc/lv_color_op.c \
../Display_Driver/lvgl/src/misc/lv_event.c \
../Display_Driver/lvgl/src/misc/lv_fs.c \
../Display_Driver/lvgl/src/misc/lv_grad.c \
../Display_Driver/lvgl/src/misc/lv_iter.c \
../Display_Driver/lvgl/src/misc/lv_ll.c \
../Display_Driver/lvgl/src/misc/lv_log.c \
../Display_Driver/lvgl/src/misc/lv_lru.c \
../Display_Driver/lvgl/src/misc/lv_math.c \
../Display_Driver/lvgl/src/misc/lv_matrix.c \
../Display_Driver/lvgl/src/misc/lv_palette.c \
../Display_Driver/lvgl/src/misc/lv_profiler_builtin.c \
../Display_Driver/lvgl/src/misc/lv_profiler_builtin_posix.c \
../Display_Driver/lvgl/src/misc/lv_rb.c \
../Display_Driver/lvgl/src/misc/lv_style.c \
../Display_Driver/lvgl/src/misc/lv_style_gen.c \
../Display_Driver/lvgl/src/misc/lv_templ.c \
../Display_Driver/lvgl/src/misc/lv_text.c \
../Display_Driver/lvgl/src/misc/lv_text_ap.c \
../Display_Driver/lvgl/src/misc/lv_timer.c \
../Display_Driver/lvgl/src/misc/lv_tree.c \
../Display_Driver/lvgl/src/misc/lv_utils.c 

OBJS += \
./Display_Driver/lvgl/src/misc/lv_anim.o \
./Display_Driver/lvgl/src/misc/lv_anim_timeline.o \
./Display_Driver/lvgl/src/misc/lv_area.o \
./Display_Driver/lvgl/src/misc/lv_array.o \
./Display_Driver/lvgl/src/misc/lv_async.o \
./Display_Driver/lvgl/src/misc/lv_bidi.o \
./Display_Driver/lvgl/src/misc/lv_circle_buf.o \
./Display_Driver/lvgl/src/misc/lv_color.o \
./Display_Driver/lvgl/src/misc/lv_color_op.o \
./Display_Driver/lvgl/src/misc/lv_event.o \
./Display_Driver/lvgl/src/misc/lv_fs.o \
./Display_Driver/lvgl/src/misc/lv_grad.o \
./Display_Driver/lvgl/src/misc/lv_iter.o \
./Display_Driver/lvgl/src/misc/lv_ll.o \
./Display_Driver/lvgl/src/misc/lv_log.o \
./Display_Driver/lvgl/src/misc/lv_lru.o \
./Display_Driver/lvgl/src/misc/lv_math.o \
./Display_Driver/lvgl/src/misc/lv_matrix.o \
./Display_Driver/lvgl/src/misc/lv_palette.o \
./Display_Driver/lvgl/src/misc/lv_profiler_builtin.o \
./Display_Driver/lvgl/src/misc/lv_profiler_builtin_posix.o \
./Display_Driver/lvgl/src/misc/lv_rb.o \
./Display_Driver/lvgl/src/misc/lv_style.o \
./Display_Driver/lvgl/src/misc/lv_style_gen.o \
./Display_Driver/lvgl/src/misc/lv_templ.o \
./Display_Driver/lvgl/src/misc/lv_text.o \
./Display_Driver/lvgl/src/misc/lv_text_ap.o \
./Display_Driver/lvgl/src/misc/lv_timer.o \
./Display_Driver/lvgl/src/misc/lv_tree.o \
./Display_Driver/lvgl/src/misc/lv_utils.o 

C_DEPS += \
./Display_Driver/lvgl/src/misc/lv_anim.d \
./Display_Driver/lvgl/src/misc/lv_anim_timeline.d \
./Display_Driver/lvgl/src/misc/lv_area.d \
./Display_Driver/lvgl/src/misc/lv_array.d \
./Display_Driver/lvgl/src/misc/lv_async.d \
./Display_Driver/lvgl/src/misc/lv_bidi.d \
./Display_Driver/lvgl/src/misc/lv_circle_buf.d \
./Display_Driver/lvgl/src/misc/lv_color.d \
./Display_Driver/lvgl/src/misc/lv_color_op.d \
./Display_Driver/lvgl/src/misc/lv_event.d \
./Display_Driver/lvgl/src/misc/lv_fs.d \
./Display_Driver/lvgl/src/misc/lv_grad.d \
./Display_Driver/lvgl/src/misc/lv_iter.d \
./Display_Driver/lvgl/src/misc/lv_ll.d \
./Display_Driver/lvgl/src/misc/lv_log.d \
./Display_Driver/lvgl/src/misc/lv_lru.d \
./Display_Driver/lvgl/src/misc/lv_math.d \
./Display_Driver/lvgl/src/misc/lv_matrix.d \
./Display_Driver/lvgl/src/misc/lv_palette.d \
./Display_Driver/lvgl/src/misc/lv_profiler_builtin.d \
./Display_Driver/lvgl/src/misc/lv_profiler_builtin_posix.d \
./Display_Driver/lvgl/src/misc/lv_rb.d \
./Display_Driver/lvgl/src/misc/lv_style.d \
./Display_Driver/lvgl/src/misc/lv_style_gen.d \
./Display_Driver/lvgl/src/misc/lv_templ.d \
./Display_Driver/lvgl/src/misc/lv_text.d \
./Display_Driver/lvgl/src/misc/lv_text_ap.d \
./Display_Driver/lvgl/src/misc/lv_timer.d \
./Display_Driver/lvgl/src/misc/lv_tree.d \
./Display_Driver/lvgl/src/misc/lv_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/misc/%.o Display_Driver/lvgl/src/misc/%.su Display_Driver/lvgl/src/misc/%.cyclo: ../Display_Driver/lvgl/src/misc/%.c Display_Driver/lvgl/src/misc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-misc

clean-Display_Driver-2f-lvgl-2f-src-2f-misc:
	-$(RM) ./Display_Driver/lvgl/src/misc/lv_anim.cyclo ./Display_Driver/lvgl/src/misc/lv_anim.d ./Display_Driver/lvgl/src/misc/lv_anim.o ./Display_Driver/lvgl/src/misc/lv_anim.su ./Display_Driver/lvgl/src/misc/lv_anim_timeline.cyclo ./Display_Driver/lvgl/src/misc/lv_anim_timeline.d ./Display_Driver/lvgl/src/misc/lv_anim_timeline.o ./Display_Driver/lvgl/src/misc/lv_anim_timeline.su ./Display_Driver/lvgl/src/misc/lv_area.cyclo ./Display_Driver/lvgl/src/misc/lv_area.d ./Display_Driver/lvgl/src/misc/lv_area.o ./Display_Driver/lvgl/src/misc/lv_area.su ./Display_Driver/lvgl/src/misc/lv_array.cyclo ./Display_Driver/lvgl/src/misc/lv_array.d ./Display_Driver/lvgl/src/misc/lv_array.o ./Display_Driver/lvgl/src/misc/lv_array.su ./Display_Driver/lvgl/src/misc/lv_async.cyclo ./Display_Driver/lvgl/src/misc/lv_async.d ./Display_Driver/lvgl/src/misc/lv_async.o ./Display_Driver/lvgl/src/misc/lv_async.su ./Display_Driver/lvgl/src/misc/lv_bidi.cyclo ./Display_Driver/lvgl/src/misc/lv_bidi.d ./Display_Driver/lvgl/src/misc/lv_bidi.o ./Display_Driver/lvgl/src/misc/lv_bidi.su ./Display_Driver/lvgl/src/misc/lv_circle_buf.cyclo ./Display_Driver/lvgl/src/misc/lv_circle_buf.d ./Display_Driver/lvgl/src/misc/lv_circle_buf.o ./Display_Driver/lvgl/src/misc/lv_circle_buf.su ./Display_Driver/lvgl/src/misc/lv_color.cyclo ./Display_Driver/lvgl/src/misc/lv_color.d ./Display_Driver/lvgl/src/misc/lv_color.o ./Display_Driver/lvgl/src/misc/lv_color.su ./Display_Driver/lvgl/src/misc/lv_color_op.cyclo ./Display_Driver/lvgl/src/misc/lv_color_op.d ./Display_Driver/lvgl/src/misc/lv_color_op.o ./Display_Driver/lvgl/src/misc/lv_color_op.su ./Display_Driver/lvgl/src/misc/lv_event.cyclo ./Display_Driver/lvgl/src/misc/lv_event.d ./Display_Driver/lvgl/src/misc/lv_event.o ./Display_Driver/lvgl/src/misc/lv_event.su ./Display_Driver/lvgl/src/misc/lv_fs.cyclo ./Display_Driver/lvgl/src/misc/lv_fs.d ./Display_Driver/lvgl/src/misc/lv_fs.o ./Display_Driver/lvgl/src/misc/lv_fs.su ./Display_Driver/lvgl/src/misc/lv_grad.cyclo ./Display_Driver/lvgl/src/misc/lv_grad.d ./Display_Driver/lvgl/src/misc/lv_grad.o ./Display_Driver/lvgl/src/misc/lv_grad.su ./Display_Driver/lvgl/src/misc/lv_iter.cyclo ./Display_Driver/lvgl/src/misc/lv_iter.d ./Display_Driver/lvgl/src/misc/lv_iter.o ./Display_Driver/lvgl/src/misc/lv_iter.su ./Display_Driver/lvgl/src/misc/lv_ll.cyclo ./Display_Driver/lvgl/src/misc/lv_ll.d ./Display_Driver/lvgl/src/misc/lv_ll.o ./Display_Driver/lvgl/src/misc/lv_ll.su ./Display_Driver/lvgl/src/misc/lv_log.cyclo ./Display_Driver/lvgl/src/misc/lv_log.d ./Display_Driver/lvgl/src/misc/lv_log.o ./Display_Driver/lvgl/src/misc/lv_log.su ./Display_Driver/lvgl/src/misc/lv_lru.cyclo ./Display_Driver/lvgl/src/misc/lv_lru.d ./Display_Driver/lvgl/src/misc/lv_lru.o ./Display_Driver/lvgl/src/misc/lv_lru.su ./Display_Driver/lvgl/src/misc/lv_math.cyclo ./Display_Driver/lvgl/src/misc/lv_math.d ./Display_Driver/lvgl/src/misc/lv_math.o ./Display_Driver/lvgl/src/misc/lv_math.su ./Display_Driver/lvgl/src/misc/lv_matrix.cyclo ./Display_Driver/lvgl/src/misc/lv_matrix.d ./Display_Driver/lvgl/src/misc/lv_matrix.o ./Display_Driver/lvgl/src/misc/lv_matrix.su ./Display_Driver/lvgl/src/misc/lv_palette.cyclo ./Display_Driver/lvgl/src/misc/lv_palette.d ./Display_Driver/lvgl/src/misc/lv_palette.o ./Display_Driver/lvgl/src/misc/lv_palette.su ./Display_Driver/lvgl/src/misc/lv_profiler_builtin.cyclo ./Display_Driver/lvgl/src/misc/lv_profiler_builtin.d ./Display_Driver/lvgl/src/misc/lv_profiler_builtin.o ./Display_Driver/lvgl/src/misc/lv_profiler_builtin.su ./Display_Driver/lvgl/src/misc/lv_profiler_builtin_posix.cyclo ./Display_Driver/lvgl/src/misc/lv_profiler_builtin_posix.d ./Display_Driver/lvgl/src/misc/lv_profiler_builtin_posix.o ./Display_Driver/lvgl/src/misc/lv_profiler_builtin_posix.su ./Display_Driver/lvgl/src/misc/lv_rb.cyclo ./Display_Driver/lvgl/src/misc/lv_rb.d ./Display_Driver/lvgl/src/misc/lv_rb.o ./Display_Driver/lvgl/src/misc/lv_rb.su ./Display_Driver/lvgl/src/misc/lv_style.cyclo ./Display_Driver/lvgl/src/misc/lv_style.d ./Display_Driver/lvgl/src/misc/lv_style.o ./Display_Driver/lvgl/src/misc/lv_style.su ./Display_Driver/lvgl/src/misc/lv_style_gen.cyclo ./Display_Driver/lvgl/src/misc/lv_style_gen.d ./Display_Driver/lvgl/src/misc/lv_style_gen.o ./Display_Driver/lvgl/src/misc/lv_style_gen.su ./Display_Driver/lvgl/src/misc/lv_templ.cyclo ./Display_Driver/lvgl/src/misc/lv_templ.d ./Display_Driver/lvgl/src/misc/lv_templ.o ./Display_Driver/lvgl/src/misc/lv_templ.su ./Display_Driver/lvgl/src/misc/lv_text.cyclo ./Display_Driver/lvgl/src/misc/lv_text.d ./Display_Driver/lvgl/src/misc/lv_text.o ./Display_Driver/lvgl/src/misc/lv_text.su ./Display_Driver/lvgl/src/misc/lv_text_ap.cyclo ./Display_Driver/lvgl/src/misc/lv_text_ap.d ./Display_Driver/lvgl/src/misc/lv_text_ap.o ./Display_Driver/lvgl/src/misc/lv_text_ap.su ./Display_Driver/lvgl/src/misc/lv_timer.cyclo ./Display_Driver/lvgl/src/misc/lv_timer.d ./Display_Driver/lvgl/src/misc/lv_timer.o ./Display_Driver/lvgl/src/misc/lv_timer.su ./Display_Driver/lvgl/src/misc/lv_tree.cyclo ./Display_Driver/lvgl/src/misc/lv_tree.d ./Display_Driver/lvgl/src/misc/lv_tree.o ./Display_Driver/lvgl/src/misc/lv_tree.su ./Display_Driver/lvgl/src/misc/lv_utils.cyclo ./Display_Driver/lvgl/src/misc/lv_utils.d ./Display_Driver/lvgl/src/misc/lv_utils.o ./Display_Driver/lvgl/src/misc/lv_utils.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-misc

