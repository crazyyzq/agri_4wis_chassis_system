# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

# ECU_Isolation board based on MR6750-WB32F8AWI-L

sdk_compile_definitions_ifdef(BUILD_FOR_SECONDARY_CORE BOARD_RUNNING_CORE=HPM_CORE1)

sdk_inc(.)
sdk_src(pinmux.c)
sdk_src(board.c)