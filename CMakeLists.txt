# CMakeLists.txt for STM32 standard peripheral library projects.
# This file is modified from a Clion generated CMakeLists.txt.


set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.22)

# You must set these by yourself
set(STD_PERIPH_LIB    $ENV{HOME}/dev/stm32/StandardPeripheralLibrary/STM32F10x_StdPeriph_Lib_V3.6.0)
set(TOOLCHAIN_PATH    $ENV{HOME}/dev/stm32/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi)
set(LINKER_SCRIPT     ${CMAKE_SOURCE_DIR}/STM32F103C8TX_FLASH.ld)
set(OPENOCD_SCRIPT    ${CMAKE_SOURCE_DIR}/stm32.cfg)
set(DEVICE_DEFINE     STM32F10X_MD)
set(STARTUP_CODE      startup_stm32f10x_md.s)

# Specify cross-compilers and tools.
set(TOOLCHAIN_PREFIX      arm-none-eabi-)

set(CMAKE_C_COMPILER      ${TOOLCHAIN_PATH}/bin/${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER    ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER    ${TOOLCHAIN_PATH}/bin/${TOOLCHAIN_PREFIX}g++)
set(CMAKE_AR              ${TOOLCHAIN_PATH}/bin/${TOOLCHAIN_PREFIX}ar)
set(CMAKE_OBJCOPY         ${TOOLCHAIN_PATH}/bin/${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_OBJDUMP         ${TOOLCHAIN_PATH}/bin/${TOOLCHAIN_PREFIX}objdump)
set(CMAKE_SIZE            ${TOOLCHAIN_PATH}/bin/${TOOLCHAIN_PREFIX}size)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Project settings.
project(STM32Template C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# Build type.
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
    add_definitions(-DDEBUG -DUSE_FULL_ASSERT)
endif ()

# Sources and includes.
include_directories(
        src
        ${STD_PERIPH_LIB}/Libraries/CMSIS/CM3/CoreSupport
        ${STD_PERIPH_LIB}/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/inc
        ${STD_PERIPH_LIB}/Project/STM32F10x_StdPeriph_Template)

set(LIBRARY_SOURCES
        ${STD_PERIPH_LIB}/Libraries/CMSIS/CM3/CoreSupport/core_cm3.c
        ${STD_PERIPH_LIB}/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c
        ${STD_PERIPH_LIB}/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/TrueSTUDIO/${STARTUP_CODE}
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/misc.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_adc.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_bkp.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_can.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_cec.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_crc.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dac.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dbgmcu.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_fsmc.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_i2c.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_iwdg.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_pwr.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rtc.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_sdio.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_spi.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c
        ${STD_PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_wwdg.c
        ${STD_PERIPH_LIB}/Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c)

set(USER_SOURCES
        src/adc.c
        src/debug.c
        src/dma.c
        src/gpio.c
        src/irq_handlers.c
        src/main.c
        src/timer.c
        src/syscalls.c
        src/usart.c)

add_definitions(-DUSE_STDPERIPH_DRIVER
        -D${DEVICE_DEFINE}
        -D__PROJECT_NAME__="${PROJECT_NAME}")

# Compile options.
add_compile_options(-mcpu=cortex-m3 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
add_compile_options(-Wall -Wextra -Werror -Wpedantic)

# Enable assembler files preprocessing.
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

# Uncomment for hardware floating point.
#add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
#add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
#add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

# Uncomment for software floating point.
#add_compile_options(-mfloat-abi=soft)

# Uncomment to mitigate c++17 absolute addresses warnings.
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# Link options.
add_link_options(-T ${LINKER_SCRIPT})
add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m3 -mthumb -mthumb-interwork)
add_link_options(-specs=nano.specs)

# Build files.
add_executable(${PROJECT_NAME}.elf ${USER_SOURCES} ${LIBRARY_SOURCES} ${LINKER_SCRIPT})

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

# Creates a custom command that prints out the binary size information.
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
        COMMENT "${PROJECT_NAME}.elf file size information:")

# Creates a custom command that generate hex and binary files.
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}\nBuilding ${BIN_FILE}")

# Add a custom target that flash hex file to STM32 using openocd
add_custom_target(flash-hex
        DEPENDS ${PROJECT_NAME}.elf
        COMMAND openocd -f ${OPENOCD_SCRIPT} -c init -c "reset halt" -c "flash write_image erase ${HEX_FILE}" -c reset -c shutdown
        #COMMAND openocd -f ${ADAPTER_CFG} -f ${BOARDCFG} -c init -c "reset halt" -c "sleep 100" -c wait_halt 2" -c "flash write_image erase ${HEX_FILE}" -c "reset run" -c shutdown
        COMMENT "Flashing ${HEX_FILE} to STM32")

# Add a custom target that flash bin file to STM32 using openocd
add_custom_target(flash-bin
        DEPENDS ${PROJECT_NAME}.elf
        COMMAND openocd -f ${OPENOCD_SCRIPT} -c init -c "reset halt" -c "flash write_image erase ${BIN_FILE} 0x08000000" -c reset -c shutdown
        COMMENT "Flashing ${HEX_FILE} to STM32")

# Add a custom target that erase STM32 using openocd
add_custom_target(erase
        COMMAND openocd -f ${OPENOCD_SCRIPT} -c init -c "reset halt" -c "stm32f1x mass_erase 0" -c halt -c shutdown
        COMMENT "Erasing STM32")

#set(CMAKE_ASM_FLAGS      "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
#set(CMAKE_CXX_FLAGS      "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")
#set(CMAKE_C_LINK_FLAGS   "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lc -lm -Wl,--end-group")
#set(CMAKE_CXX_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lstdc++ -lsupc++ -Wl,--end-group")
