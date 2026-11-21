# MCXScope firmware for the FRDM-MCXA156 (MCUXpresso SDK 26.09, CMake + Kconfig build).
#
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.22.0)

include("${SdkRootDirPath}/cmake/extension/mcux.cmake")

project(
  mcxscope
  LANGUAGES ASM C CXX
  PROJECT_BOARD_PORT_PATH "examples/_boards/${board}/usb_examples/usb_device_cdc_vcom/bm"
)

include("${SdkRootDirPath}/CMakeLists.txt")

mcux_add_macro(CC "-DUSB_STACK_BM")

# Optimise even in the debug preset: the software trigger search has to keep up with
# 2 MS/s, which -O0 can't do. -g stays, so debugging still works.
mcux_add_armgcc_configuration(CC "-O2")

# %g in replies needs float support in newlib-nano's printf
mcux_add_armgcc_configuration(
  LD "-u _printf_float"
)
mcux_add_armgcc_configuration(LD "-Xlinker --defsym=__heap_size__=0x800")
mcux_add_armgcc_configuration(LD "-Xlinker --defsym=__stack_size__=0x1000")

mcux_add_include(INCLUDES src board)

mcux_add_source(
  SOURCES
    src/main.c
    src/adc_capture.c
    src/adc_capture.h
    src/config.h
    src/scope.c
    src/scope.h
    src/scpi.c
    src/scpi.h
    src/timebase.c
    src/timebase.h
    src/usb_cdc.c
    src/usb_cdc.h
    src/usb_device_descriptor.c
    src/usb_device_descriptor.h
    board/pin_mux.c
    board/pin_mux.h
)

mcux_convert_binary(BINARY "${APPLICATION_BINARY_DIR}/${MCUX_SDK_PROJECT_NAME}.bin")
