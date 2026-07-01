cmake_minimum_required(VERSION 3.13)

set(CONFIG_FREERTOS 1)

if("${HPM_BUILD_TYPE}" STREQUAL "")
    set(HPM_BUILD_TYPE flash_xip)
endif()

# Match CPU0's self-contained build setup so CPU1 can be opened from SEGGER
# Embedded Studio or regenerated from CMake without relying on machine-global
# SDK/toolchain environment variables.
if("$ENV{HPM_SDK_BASE}" STREQUAL "")
    set(ENV{HPM_SDK_BASE} "${CMAKE_CURRENT_LIST_DIR}/../../sdk_env_v1.11.0/hpm_sdk")
endif()
if("$ENV{GNURISCV_TOOLCHAIN_PATH}" STREQUAL "")
    set(ENV{GNURISCV_TOOLCHAIN_PATH} "${CMAKE_CURRENT_LIST_DIR}/../../sdk_env_v1.11.0/toolchains/rv32imac_zicsr_zifencei_multilib_b_ext-win")
endif()

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(agri_chassis_control_cpu1)

sdk_compile_definitions(-DUSE_NONVECTOR_MODE=1)
sdk_compile_definitions(-DDISABLE_IRQ_PREEMPTIVE=1)

# CPU1 uses the same MR6750 J-Link/JTAG debug path as CPU0.  Setting this in
# CMake prevents regenerated SEGGER projects from falling back to OpenOCD or a
# local simulator/debugger profile.
sdk_ses_opt_debug_connection("J-Link")
sdk_ses_opt_debug_jlink_speed(4000)

sdk_inc(src)
sdk_inc(../../common/include)
sdk_inc(../../config/include)
sdk_inc(../../ipc/include)
sdk_inc(../../os/include)

sdk_app_src(src/main_cpu1.c)
sdk_app_src(../../config/src/ecu_config.c)
sdk_app_src(../../ipc/src/ipc_snapshot.c)
sdk_app_src(../../os/src/ecu_tasks_cpu1.c)

generate_ide_projects()
