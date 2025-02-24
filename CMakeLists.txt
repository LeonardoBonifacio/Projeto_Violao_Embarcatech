# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Projeto_Violao_EmbarcaTech C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()


# Adiciona o executável principal
add_executable(Projeto_Violao_EmbarcaTech 
    Projeto_Violao_EmbarcaTech.c
    lib/ssd1306.c
)

pico_set_program_name(Projeto_Violao_EmbarcaTech "Projeto_Violao_EmbarcaTech")
pico_set_program_version(Projeto_Violao_EmbarcaTech "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Projeto_Violao_EmbarcaTech 0)
pico_enable_stdio_usb(Projeto_Violao_EmbarcaTech 1)

# Generate PIO header
pico_generate_pio_header(Projeto_Violao_EmbarcaTech ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

# Add the standard include files to the build
target_include_directories(Projeto_Violao_EmbarcaTech PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Adicionar bibliotecas ao projeto
target_link_libraries(Projeto_Violao_EmbarcaTech
    pico_stdlib
    hardware_adc
    hardware_i2c
    hardware_clocks
    hardware_pwm
    hardware_pio
)

pico_add_extra_outputs(Projeto_Violao_EmbarcaTech)
