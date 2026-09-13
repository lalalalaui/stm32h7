cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME touchh7)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

file(GLOB_RECURSE LVGL_CORE_SOURCES CONFIGURE_DEPENDS
    Middlewares/lvgl/src/core/*.c
    Middlewares/lvgl/src/display/*.c
    Middlewares/lvgl/src/indev/*.c
    Middlewares/lvgl/src/layouts/*.c
    Middlewares/lvgl/src/misc/*.c
    Middlewares/lvgl/src/themes/*.c
    Middlewares/lvgl/src/tick/*.c
    Middlewares/lvgl/src/widgets/*.c
)
file(GLOB_RECURSE LVGL_DRAW_SOURCES CONFIGURE_DEPENDS
    Middlewares/lvgl/src/draw/*.c
)
list(FILTER LVGL_DRAW_SOURCES EXCLUDE REGEX ".*/src/draw/convert/helium/.*")
list(FILTER LVGL_DRAW_SOURCES EXCLUDE REGEX ".*/src/draw/convert/neon/.*")
list(FILTER LVGL_DRAW_SOURCES EXCLUDE REGEX ".*/src/draw/convert/riscv_v/.*")
list(FILTER LVGL_DRAW_SOURCES EXCLUDE REGEX ".*/src/draw/dma2d/.*")
list(FILTER LVGL_DRAW_SOURCES EXCLUDE REGEX ".*/src/draw/espressif/.*")
list(FILTER LVGL_DRAW_SOURCES EXCLUDE REGEX ".*/src/draw/eve/.*")
list(FILTER LVGL_DRAW_SOURCES EXCLUDE REGEX ".*/src/draw/nanovg/.*")
list(FILTER LVGL_DRAW_SOURCES EXCLUDE REGEX ".*/src/draw/nema_gfx/.*")
list(FILTER LVGL_DRAW_SOURCES EXCLUDE REGEX ".*/src/draw/nxp/.*")
list(FILTER LVGL_DRAW_SOURCES EXCLUDE REGEX ".*/src/draw/opengles/.*")
list(FILTER LVGL_DRAW_SOURCES EXCLUDE REGEX ".*/src/draw/renesas/.*")
list(FILTER LVGL_DRAW_SOURCES EXCLUDE REGEX ".*/src/draw/sdl/.*")
list(FILTER LVGL_DRAW_SOURCES EXCLUDE REGEX ".*/src/draw/vg_lite/.*")
set(LVGL_SOURCES
    ${LVGL_CORE_SOURCES}
    ${LVGL_DRAW_SOURCES}
    Middlewares/lvgl/src/font/lv_font.c
    Middlewares/lvgl/src/font/fmt_txt/lv_font_fmt_txt.c
    Middlewares/lvgl/src/font/lv_font_montserrat_14.c
    Middlewares/lvgl/src/font/lv_font_montserrat_18.c
    Middlewares/lvgl/src/font/lv_font_montserrat_32.c
    Middlewares/lvgl/src/libs/bin_decoder/lv_bin_decoder.c
    Middlewares/lvgl/src/lv_init.c
    Middlewares/lvgl/src/osal/lv_os.c
    Middlewares/lvgl/src/osal/lv_os_none.c
    Middlewares/lvgl/src/stdlib/builtin/lv_mem_core_builtin.c
    Middlewares/lvgl/src/stdlib/builtin/lv_sprintf_builtin.c
    Middlewares/lvgl/src/stdlib/builtin/lv_string_builtin.c
    Middlewares/lvgl/src/stdlib/builtin/lv_tlsf.c
    Middlewares/lvgl/src/stdlib/lv_mem.c
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${LVGL_SOURCES}
    Core/Src/power_meter_hw.c
    Core/Src/power_meter.c
    Core/Src/responsivity_table.c
    Core/Src/power_meter_port.c
    Core/Src/power_meter_storage.c
    Drivers/SYSTEM/delay/delay.c
    Drivers/SYSTEM/sys/sys.c
    Drivers/SYSTEM/usart/usart.c
    Drivers/BSP/24CXX/24cxx.c
    Drivers/BSP/ADC/adc.c
    Drivers/BSP/FPGA/fpga_display.c
    Drivers/BSP/IIC/myiic.c
    Drivers/BSP/KEY/key.c
    Drivers/BSP/LCD/lcd.c
    Drivers/BSP/LCD/ltdc.c
    Drivers/BSP/LED/led.c
    Drivers/BSP/LVGL/lvgl_port.c
    Drivers/BSP/LVGL/lv_font_chinese_14.c
    Drivers/BSP/LVGL/slave_ui.c
    Drivers/BSP/MPU/mpu.c
    Drivers/BSP/SDRAM/sdram.c
    Drivers/BSP/TOUCH/ctiic.c
    Drivers/BSP/TOUCH/ft5206.c
    Drivers/BSP/TOUCH/gt9xxx.c
    Drivers/BSP/TOUCH/touch.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma2d.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_ltdc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_ltdc_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sdram.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sram.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_fmc.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Drivers
    Drivers/SYSTEM/delay
    Drivers/SYSTEM/sys
    Drivers/SYSTEM/usart
    Drivers/BSP/24CXX
    Drivers/BSP/ADC
    Drivers/BSP/FPGA
    Drivers/BSP/IIC
    Drivers/BSP/KEY
    Drivers/BSP/LCD
    Drivers/BSP/LED
    Drivers/BSP/LVGL
    Drivers/BSP/MPU
    Drivers/BSP/SDRAM
    Drivers/BSP/TOUCH
    Middlewares
    Middlewares/lvgl
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    LV_CONF_INCLUDE_SIMPLE
    LV_CONF_SUPPRESS_DEFINE_CHECK
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m

    # Add user defined libraries
)
