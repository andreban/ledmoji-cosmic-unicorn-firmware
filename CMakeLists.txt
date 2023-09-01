cmake_minimum_required(VERSION 3.12)

set(PICO_BOARD pico_w)
set(PIMORONI_PICO_PATH ${PIMORONI_PICO_PATH})

# Change your executable name to something creative!
set(NAME ledmoji-cosmic-unicorn-firmware)

include(pimoroni_pico_import.cmake)
include(pico_sdk_import.cmake)

# Gooey boilerplate
project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Add your source files
add_executable(${NAME}
    main.cpp # <-- Add source files here!
)

pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)

target_compile_definitions(${NAME} PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        MQTT_SERVER_HOST=\"${MQTT_SERVER_HOST}\"
        NO_SYS=1
)

target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Include required libraries
# This assumes `pimoroni-pico` is stored alongside your project
include(common/pimoroni_i2c)
include(common/pimoroni_bus)
include(libraries/pico_graphics/pico_graphics)
include(libraries/cosmic_unicorn/cosmic_unicorn)

# Don't forget to link the libraries you need!
target_link_libraries(${NAME}
    pico_stdlib
    pico_stdio_usb
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mbedtls
    pico_mbedtls
    pico_lwip_mqtt
    pimoroni_i2c
    pico_graphics
    cosmic_unicorn # <-- List libraries here!
)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# Set up files for the release packages
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.uf2
    ${CMAKE_CURRENT_LIST_DIR}/README.md
    DESTINATION .
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)
