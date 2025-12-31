# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
add_compile_options("-Wno-format")
add_compile_definitions(LV_CONF_INCLUDE_SIMPLE=1)
add_compile_definitions(LV_CONF_PATH="${CMAKE_SOURCE_DIR}/main/lv_conf.h")
set(COMPONENTS main lvgl esp_lvgl_port) 
project(example_qspi_with_ram)
