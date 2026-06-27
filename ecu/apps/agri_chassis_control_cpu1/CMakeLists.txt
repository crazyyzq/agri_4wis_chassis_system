cmake_minimum_required(VERSION 3.13)

set(CONFIG_FREERTOS 1)

if("${HPM_BUILD_TYPE}" STREQUAL "")
    set(HPM_BUILD_TYPE flash_xip)
endif()

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(agri_chassis_control_cpu1)

sdk_compile_definitions(-DUSE_NONVECTOR_MODE=1)
sdk_compile_definitions(-DDISABLE_IRQ_PREEMPTIVE=1)

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
