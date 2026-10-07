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

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 明确声明当前工程面向 ARM 裸机目标，避免被按 Windows 主机程序处理。
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 根据构建类型设置默认优化选项，优先保证嵌入式调试可用性与发布稳定性。
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE}" CACHE STRING "Build type" FORCE)
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")

if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    message(STATUS "Debug build: enable -Og -g3 for source-level debugging")
    add_compile_options(-Og -g3)
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Release build: enable -O2 -g0 for stable runtime performance")
    add_compile_options(-O2 -g0)
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "RelWithDebInfo build: enable -O2 -g3 for optimized debugging")
    add_compile_options(-O2 -g3)
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "MinSizeRel build: enable -Os -g0 for code size reduction")
    add_compile_options(-Os -g0)
else()
    message(FATAL_ERROR
            "Unsupported CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}. "
            "Allowed values are Debug, Release, RelWithDebInfo, MinSizeRel.")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME H7_Board)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# 输出当前工具链的关键上下文，便于在 IDE 切换 Profile 后快速定位问题。
message(STATUS "CMAKE_TOOLCHAIN_FILE: ${CMAKE_TOOLCHAIN_FILE}")
message(STATUS "CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
message(STATUS "CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")
message(STATUS "CMAKE_CXX_COMPILER: ${CMAKE_CXX_COMPILER}")

# 强制校验当前工程仍然使用 ARM 交叉工具链，避免 CLion 误切回主机编译器后静默生成错误缓存。
get_filename_component(CMAKE_C_COMPILER_NAME "${CMAKE_C_COMPILER}" NAME)
get_filename_component(CMAKE_CXX_COMPILER_NAME "${CMAKE_CXX_COMPILER}" NAME)

if (NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
    message(FATAL_ERROR
            "当前 CMake Profile 未正确指向 ARM 目标。"
            "请优先使用 CMakePresets.json 中的 Debug/Release preset 重新加载工程。")
endif ()

if (NOT CMAKE_C_COMPILER_NAME MATCHES "^arm-none-eabi-gcc(\\.exe)?$"
        OR NOT CMAKE_CXX_COMPILER_NAME MATCHES "^arm-none-eabi-g\\+\\+(\\.exe)?$")
    message(FATAL_ERROR
            "检测到当前编译器不是 arm-none-eabi 工具链："
            "C=${CMAKE_C_COMPILER_NAME}, CXX=${CMAKE_CXX_COMPILER_NAME}。"
            "请检查 CLion 当前 CMake Profile 是否使用了 CMakePresets.json 中声明的 toolchainFile。")
endif ()

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# ========== STM32H7 (Cortex-M7) 硬件浮点配置（核心） ==========
# 1. 浮点/内核相关编译宏定义
add_compile_definitions(
        ARM_MATH_CM7          # M7内核   
        __ARM_FP=14           # 标识FPv5-D16浮点单元
        STM32H723xx 
        __PROGRAM_START       # Suppress CMSIS default startup to fix local type error in cmsis_gcc.h         
)

# 2. 编译选项（M7内核+硬件浮点+优化）
# 核心：硬件浮点（hard）+ FPv5-D16（M7的双精度FPU）+ Cortex-M7内核
add_compile_options(
    -mcpu=cortex-m7
    -mfpu=fpv5-d16
    -mfloat-abi=hard
    -mthumb )         

# 3. 链接选项（和编译选项保持一致，避免浮点ABI不匹配）
add_link_options(
        -mcpu=cortex-m7
        -mthumb
        -mfloat-abi=hard
        -mfpu=fpv5-d16
        -Wl,-Map=H7_Demo.map  # 生成映射文件（方便查看内存分布）
)


#用户源文件以及DSP库文件收集
file(GLOB_RECURSE SOURCES_USERS
        "${CMAKE_CURRENT_SOURCE_DIR}/Development_Layer/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Development_Layer/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Development_Layer/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/Development_Layer/*.hpp"
)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        ${SOURCES_USERS}
        Development_Layer/Robot_Control/Chassis/chassis.cpp
        Development_Layer/Robot_Control/Chassis/chassis.h
        Development_Layer/Robot_Control/Controller/VMC.cpp
        Development_Layer/Robot_Control/Controller/VMC.h
)

# 强制输出 ELF 后缀，避免链接阶段误生成 Windows .exe 目标。
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES SUFFIX ".elf")

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add DSP library search path
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths

        Development_Layer/Function_Class/Class_Robot
        Development_Layer/Function_Class/Class_Sampling
        Development_Layer/Function_Class/Class_FSM
        Development_Layer/Function_Class/Class_Slope
        Development_Layer/Function_Class/Class_Timer
        Development_Layer/Function_Class/Class_CRC
        Development_Layer/Driver/DC_Source
        Development_Layer/Driver/FDCAN
        Development_Layer/Driver/UART
        Development_Layer/Driver/USB
        Development_Layer/Driver/Math
        Development_Layer/Driver/ADC
        Development_Layer/Driver/SPI
        Development_Layer/Driver/TIM
        Development_Layer/Driver/LED_and_BUZZER
        Development_Layer/Driver/Matrix
        Development_Layer/External_Device/DR16
        Development_Layer/External_Device/VTM_RX
        Development_Layer/External_Device/Omega
        Development_Layer/External_Device/Motor/Motor_DJI
        Development_Layer/External_Device/Motor/Motor_DM
        Development_Layer/External_Device/BMI088
        Development_Layer/External_Device/DM_IMU
        Development_Layer/External_Device/Fire_Manage
        Development_Layer/External_Device/Referee
        Development_Layer/External_Device/PC_Driver
        Development_Layer/External_Device/G_C_Communicate
        Development_Layer/External_Device/TOF050F
        Development_Layer/External_Device/FS-IA6B
        Development_Layer/Robot_Control/Booster
        Development_Layer/Robot_Control/Controller
        Development_Layer/Robot_Control/Filter
        Development_Layer/Robot_Control/Gimbal
        Development_Layer/Robot_Control/Foot_Control
        Development_Layer/Task/CallBack_Task
        Development_Layer/Task/RTOS_Task
        Development_Layer/User_Debug/Debug_Config
        Development_Layer/User_Debug/USB_CDC_Debug
        Development_Layer/User_Debug/Serial_Debug
        Development_Layer/Robot_Control/Lift
)
# Force Drivers/CMSIS/DSP/Include to be first to override Middlewares version
target_include_directories(${CMAKE_PROJECT_NAME} BEFORE PRIVATE
        Drivers/CMSIS/DSP/Include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -Wl,-Map=${CMAKE_PROJECT_NAME}.map)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)



# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Use absolute path to ensure we link the GCC version, preventing confusion with Middlewares .lib
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC/libarm_cortexM7lfdp_math.a
    # Add user defined libraries
)
