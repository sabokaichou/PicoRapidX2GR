cmake_minimum_required(VERSION 3.13)

# Pico SDK をインポート
include(pico_sdk_import.cmake)

project(PicoVSyncSeparator C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# PicoRapidX2GR: VSync検出機能統合版
add_executable(PicoRapidX2GR
    PicoRapidX2GR.c
    usb_msc_device.c
    usb_descriptors.c
    vsync_separator.c
)

# PIOファイルを生成（PicoRapidX2用）
pico_generate_pio_header(PicoRapidX2GR ${CMAKE_CURRENT_LIST_DIR}/vsync_separator.pio)

# 追加ヘッダディレクトリが存在する場合のみ使用（無ければコメントアウト可）
if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/header)
    target_include_directories(PicoRapidX2GR PRIVATE ${CMAKE_CURRENT_LIST_DIR}/header)
endif()

# 標準IO設定
pico_enable_stdio_usb(PicoRapidX2GR 0)
pico_enable_stdio_uart(PicoRapidX2GR 0)

# TinyUSB needs access to tusb_config.h from this dir
add_library(tinyusb_config INTERFACE)
target_include_directories(tinyusb_config INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(PicoRapidX2GR
    pico_stdlib
    hardware_flash
    hardware_sync
    hardware_pwm
    hardware_pio
    pico_multicore
    hardware_i2c
    tinyusb_device
    tinyusb_board
    tinyusb_config
)

# 追加生成: map/bin/hex/uf2
pico_add_extra_outputs(PicoRapidX2GR)
