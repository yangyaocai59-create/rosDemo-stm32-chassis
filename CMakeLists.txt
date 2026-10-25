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
set(CMAKE_PROJECT_NAME rosDemo)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

set(CHASSIS_CONTROL_REVISION "191" CACHE STRING "Chassis control policy: 191, 200, 201, 202, 203, 204 or 205")
if(NOT CHASSIS_CONTROL_REVISION MATCHES "^(191|200|201|202|203|204|205)$")
    message(FATAL_ERROR "Unsupported CHASSIS_CONTROL_REVISION")
endif()
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    CHASSIS_CONTROL_REVISION=${CHASSIS_CONTROL_REVISION})
if(CHASSIS_CONTROL_REVISION STREQUAL "200")
    set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES OUTPUT_NAME "rosDemo-0.20.0")
elseif(CHASSIS_CONTROL_REVISION STREQUAL "201")
    set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES OUTPUT_NAME "rosDemo-0.20.1")
elseif(CHASSIS_CONTROL_REVISION STREQUAL "202")
    set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES OUTPUT_NAME "rosDemo-0.20.2")
elseif(CHASSIS_CONTROL_REVISION STREQUAL "203")
    set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES OUTPUT_NAME "rosDemo-0.20.3")
elseif(CHASSIS_CONTROL_REVISION STREQUAL "204")
    set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES OUTPUT_NAME "rosDemo-0.20.4")
elseif(CHASSIS_CONTROL_REVISION STREQUAL "205")
    set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES OUTPUT_NAME "rosDemo-0.20.5")
endif()

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)
if(CHASSIS_CONTROL_REVISION GREATER_EQUAL 203)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE Application/Chassis/chassis_yaw_feedback.c)
endif()

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Board/board_config.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Chassis/chassis_controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Chassis/chassis_kinematics.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Chassis/chassis_motion_profile.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Chassis/chassis_runtime.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Chassis/encoder_service.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Chassis/motor_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Chassis/wheel_speed_controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Chassis/wheel_speed_estimator.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Communication/chassis_usb_protocol.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Communication/usb_cdc_transport.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Diagnostics/fault_injection_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Diagnostics/motion_trace.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Diagnostics/encoder_diagnostics.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Diagnostics/imu_static_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Diagnostics/imu_calibration_guard.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Diagnostics/imu_spi_identity_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Diagnostics/chassis_ground_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Diagnostics/chassis_motion_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Diagnostics/chassis_runtime_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Diagnostics/four_wheel_speed_control_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Diagnostics/motor_open_loop_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Diagnostics/motor_characterization_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Diagnostics/motor_speed_control_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Power/battery_monitor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Safety/emergency_stop.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Safety/fault_manager.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Safety/watchdog_service.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Sensors/imu_service.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Sensors/mpu6050.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Sensors/mpu6500.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Sensors/mpu_spi_probe.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Services/debug_log.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Services/control_timing.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Storage/imu_bias_storage.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/System/system_status.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Board
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Chassis
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Communication
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Diagnostics
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Power
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Safety
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Sensors
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Services
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Storage
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/System
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    $<$<CONFIG:Debug>:MOTOR_OPEN_LOOP_TEST_ENABLED=1>
    $<$<CONFIG:Debug>:FAULT_INJECTION_TEST_ENABLED=1>
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Always regenerate the flashable Intel HEX image from the just-linked ELF.
# Keeping this in the user-owned top-level file avoids touching CubeMX CMake.
get_filename_component(STM32_TOOLCHAIN_BIN_DIR "${CMAKE_C_COMPILER}" DIRECTORY)
find_program(STM32_OBJCOPY_EXECUTABLE
    NAMES arm-none-eabi-objcopy arm-none-eabi-objcopy.exe
    HINTS "${STM32_TOOLCHAIN_BIN_DIR}"
    REQUIRED
)
# ST Build Analyzer pairs a versioned ELF with a MAP of the same base name.
# The Cube linker still writes rosDemo.map; keep it for packaging and also
# publish a matching versioned copy before notifying the editor.
if(NOT CHASSIS_CONTROL_REVISION STREQUAL "191")
    add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.map
                $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/$<TARGET_FILE_BASE_NAME:${CMAKE_PROJECT_NAME}>.map
        COMMENT "Publishing matching versioned MAP for STM32 Build Analyzer"
        VERBATIM
    )
endif()
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${STM32_OBJCOPY_EXECUTABLE}
            -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/$<TARGET_FILE_BASE_NAME:${CMAKE_PROJECT_NAME}>.hex
    COMMENT "Generating versioned Intel HEX image"
    VERBATIM
)

# Publish the exact successful ELF to the editor without replacing archives.
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
    if(CHASSIS_CONTROL_REVISION STREQUAL "191")
        set(ROSD_EDITOR_PRESET "$<CONFIG>")
    else()
        set(ROSD_EDITOR_PRESET "WheelV0${CHASSIS_CONTROL_REVISION}$<CONFIG>")
    endif()
    add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${Python3_EXECUTABLE} -B
                ${CMAKE_CURRENT_SOURCE_DIR}/Tools/sync_vscode_firmware.py
                --elf $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
                --revision ${CHASSIS_CONTROL_REVISION}
                --configuration $<CONFIG> --preset ${ROSD_EDITOR_PRESET}
        COMMENT "Updating VS Code firmware build status"
        VERBATIM
    )
endif()
