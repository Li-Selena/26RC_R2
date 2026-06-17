cmake_minimum_required(VERSION 3.27)

# Roots
include("../roots.cmake")

set(CONTEXT 26_RC_02+26_RC_02)
set(TARGET ${CONTEXT})
set(DNAME STM32H723VGTx)
set(DPACK Keil::STM32H7xx_DFP@4.1.3)
set(DPACK_DIR "${CMSIS_PACK_ROOT}/Keil/STM32H7xx_DFP/4.1.3")
set(OUT_DIR "${SOLUTION_ROOT}/out/26_RC_02/26_RC_02")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_COMPILE_COMMANDS ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json)
set(COMPILE_COMMANDS ${OUT_DIR}/compile_commands.json)
set(COMPILE_MACROS ${OUT_DIR}/compile_macros.h)
set(LD_SCRIPT "${SOLUTION_ROOT}/26_RC_02_26_RC_02.sct")
set(LD_SCRIPT_PP ${LD_SCRIPT})

# Processor Options
set(CPU Cortex-M7)
set(FPU DP_FPU)
set(DSP DSP)
set(BYTE_ORDER Little-endian)

# Toolchain config map
set(COMPILER AC6)
include("toolchain.cmake")

# Setup project
project(${CONTEXT} LANGUAGES ASM C)

# Enable color diagnostics
set(CMAKE_COLOR_DIAGNOSTICS ON)

# Compilation database
add_custom_target(database DEPENDS ${COMPILE_COMMANDS} ${COMPILE_MACROS})
add_custom_command(OUTPUT ${COMPILE_COMMANDS}
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_COMPILE_COMMANDS}" "${COMPILE_COMMANDS}"
  DEPENDS "${CMAKE_COMPILE_COMMANDS}"
)
add_custom_command(OUTPUT ${COMPILE_MACROS}
  COMMAND ${CPP} ${CPP_DUMP_MACROS}
)
set(CMAKE_C_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_C_IMPLICIT_INCLUDE_DIRECTORIES})

# Setup context
add_executable(${CONTEXT})
set_target_properties(${CONTEXT} PROPERTIES PREFIX "" SUFFIX ".axf" OUTPUT_NAME "26_RC_02")
set_target_properties(${CONTEXT} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUT_DIR})
add_library(${CONTEXT}_GLOBAL INTERFACE)

# Includes
target_include_directories(${CONTEXT} PUBLIC
  $<$<COMPILE_LANGUAGE:C,CXX>:
    "${SOLUTION_ROOT}/../Core/Inc"
    "${SOLUTION_ROOT}/../USB_DEVICE/App"
    "${SOLUTION_ROOT}/../USB_DEVICE/Target"
    "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Inc"
    "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy"
    "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/include"
    "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS"
    "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F"
    "${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Core/Inc"
    "${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"
    "${SOLUTION_ROOT}/../Drivers/CMSIS/Device/ST/STM32H7xx/Include"
    "${SOLUTION_ROOT}/../Drivers/CMSIS/Include"
    "${SOLUTION_ROOT}/../Applications/Task/Inc"
    "${SOLUTION_ROOT}/../Applications/App_user/Inc"
    "${SOLUTION_ROOT}/../Applications/R2_user/Inc"
    "${SOLUTION_ROOT}/../Components/Algorithm/Inc"
    "${SOLUTION_ROOT}/../Components/Controller/Inc"
    "${SOLUTION_ROOT}/../Components/Device/Inc"
    "${SOLUTION_ROOT}/../BSP/Inc"
  >
  "${SOLUTION_ROOT}/RTE/_26_RC_02"
  "${CMSIS_PACK_ROOT}/ARM/CMSIS-DSP/1.16.2/Include"
  "${CMSIS_PACK_ROOT}/ARM/CMSIS-DSP/1.16.2/PrivateInclude"
  "${CMSIS_PACK_ROOT}/ARM/CMSIS/5.9.0/CMSIS/Core/Include"
)

# Defines
target_compile_definitions(${CONTEXT} PUBLIC
  $<$<COMPILE_LANGUAGE:C,CXX>:
    USE_PWR_LDO_SUPPLY
    USE_HAL_DRIVER
    STM32H723xx
    STM32H723xx
    _RTE_
  >
)

# Compile options
target_compile_options(${CONTEXT} PUBLIC
  $<$<COMPILE_LANGUAGE:ASM>:
    "SHELL:${ASM_CPU}"
    "SHELL:${ASM_FLAGS}"
    "SHELL:${ASM_BYTE_ORDER}"
    "SHELL:-masm=auto"
  >
  $<$<COMPILE_LANGUAGE:C>:
    "SHELL:${CC_CPU}"
    "SHELL:${CC_FLAGS}"
    "SHELL:${CC_BYTE_ORDER}"
    "SHELL:-std=c99"
    "SHELL:-fno-rtti"
    "SHELL:-funsigned-char"
    "SHELL:-fshort-enums"
    "SHELL:-fshort-wchar"
    "SHELL:-ffunction-sections"
    "SHELL:-Wno-packed"
    "SHELL:-Wno-missing-variable-declarations"
    "SHELL:-Wno-missing-prototypes"
    "SHELL:-Wno-missing-noreturn"
    "SHELL:-Wno-sign-conversion"
    "SHELL:-Wno-nonportable-include-path"
    "SHELL:-Wno-reserved-id-macro"
    "SHELL:-Wno-unused-macros"
    "SHELL:-Wno-documentation-unknown-command"
    "SHELL:-Wno-documentation"
    "SHELL:-Wno-license-management"
    "SHELL:-Wno-parentheses-equality"
  >
)

# Compile Options Abstractions
add_library(${CONTEXT}_ABSTRACTIONS INTERFACE)
cbuild_set_options_flags(ASM "debug" "on" "" "" ASM_OPTIONS_FLAGS_${CONTEXT})
cbuild_set_options_flags(CC "debug" "on" "" "" CC_OPTIONS_FLAGS_${CONTEXT})
target_compile_options(${CONTEXT}_ABSTRACTIONS INTERFACE
  $<$<COMPILE_LANGUAGE:ASM>:
    "SHELL:${ASM_OPTIONS_FLAGS_${CONTEXT}}"
  >
  $<$<COMPILE_LANGUAGE:C>:
    "SHELL:${CC_OPTIONS_FLAGS_${CONTEXT}}"
  >
)

# Add groups and components
include("groups.cmake")
include("components.cmake")

target_link_libraries(${CONTEXT} PUBLIC
  Group_Application_MDK-ARM
  Group_Application_User_Core
  Group_Applications_Task
  Group_Applications_R2_user
  Group_Components_Algorithm
  Group_Components_Controller
  Group_Components_Device
  Group_BSP
  Group_Application_User_USB_DEVICE_App
  Group_Application_User_USB_DEVICE_Target
  Group_Drivers_STM32H7xx_HAL_Driver
  Group_Drivers_CMSIS
  Group_Middlewares_FreeRTOS
  Group_Middlewares_USB_Device_Library
  ARM_CMSIS_CORE_5_6_0
  ARM_CMSIS_DSP_Source_1_16_2
)

# Linker options
target_link_options(${CONTEXT} PUBLIC
  "SHELL:${LD_CPU}"
  "SHELL:${_LS}\"${LD_SCRIPT_PP}\""
  "SHELL:--map"
  "SHELL:--load_addr_map_info"
  "SHELL:--xref"
  "SHELL:--callgraph"
  "SHELL:--symbols"
  "SHELL:--info sizes"
  "SHELL:--info totals"
  "SHELL:--info unused"
  "SHELL:--info veneers"
  "SHELL:--strict"
  "SHELL:--summary_stderr"
  "SHELL:--info summarysizes"
)
set(LD_DEPS ${LD_SCRIPT})
set_target_properties(${CONTEXT} PROPERTIES LINK_DEPENDS "${LD_DEPS}")
