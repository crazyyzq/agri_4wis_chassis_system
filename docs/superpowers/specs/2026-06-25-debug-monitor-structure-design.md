# Debug monitor structure design

Date: 2026-06-25

## Goal

Add a debugger-controlled ECU monitor for bench bring-up of SBUS, analog inputs, digital inputs, and digital outputs. The operator will use SEGGER Embedded Studio to edit a global C structure at runtime. The firmware will then print the selected live data to the existing UART0 test console.

This feature is for bring-up visibility, not final pass/fail qualification. Existing registered tests such as `SBUS.RX`, `ADC.ALL`, `DI.ALL`, `DO.ALL`, and `DIO.LOOP` remain the formal validation paths.

## Operator workflow

The firmware exposes one global volatile structure:

```c
extern volatile ecu_debug_monitor_t g_ecu_debug_monitor;
```

The operator changes fields in the debugger watch window:

- `enable = 1` enables periodic monitor printing.
- `view` selects what to print: none, SBUS, ADC, DI, DO, or all.
- `channel = 0` prints all channels; nonzero prints only the selected channel where that view supports selection.
- `period_ms` sets the print period; invalid or too-small values are clamped by firmware.
- `do_enable = 1` allows debugger-controlled digital outputs.
- `do_mask` controls EX_OUT1..EX_OUT12 directly, with bit 0 mapped to EX_OUT1 and bit 11 mapped to EX_OUT12.

## Debug control structure

The public API will define stable integer enums and a compact control structure suitable for debugger editing:

```c
typedef enum {
    ECU_DEBUG_VIEW_NONE = 0,
    ECU_DEBUG_VIEW_SBUS = 1,
    ECU_DEBUG_VIEW_ADC  = 2,
    ECU_DEBUG_VIEW_DI   = 3,
    ECU_DEBUG_VIEW_DO   = 4,
    ECU_DEBUG_VIEW_ALL  = 5
} ecu_debug_view_t;

typedef struct {
    volatile uint32_t enable;
    volatile uint32_t view;
    volatile uint32_t channel;
    volatile uint32_t period_ms;
    volatile uint32_t do_enable;
    volatile uint32_t do_mask;
} ecu_debug_monitor_t;
```

Default values at boot:

- `enable = 0`
- `view = ECU_DEBUG_VIEW_NONE`
- `channel = 0`
- `period_ms = 200`
- `do_enable = 0`
- `do_mask = 0`

## Data outputs

SBUS view:

- Configure SBUS UART as 100000 baud, even parity, 2 stop bits.
- Decode standard 25-byte SBUS frames.
- Print all 16 decoded channels when `channel = 0`.
- Print only `chN` when `channel = 1..16`.
- Include `lost` and `failsafe` status flags.
- Do not require the existing `SBUS.RX` stop-transmitter confirmation step.

ADC view:

- Sample the four existing external analog inputs using ADC3.
- Convert each channel to external input voltage in millivolts.
- Print `EX1..EX4` in mV when `channel = 0`.
- Print only the selected `EXn` when `channel = 1..4`.

DI view:

- Read all 12 ECU digital inputs with the existing board API.
- Print logical `0` or `1` directly.
- Print `EX_IN1..EX_IN12` when `channel = 0`.
- Print only the selected `EX_INn` when `channel = 1..12`.

DO view:

- If `do_enable = 0`, force all managed digital outputs off and ignore `do_mask`.
- If `do_enable = 1`, apply `do_mask` directly to EX_OUT1..EX_OUT12.
- Print `EX_OUT1..EX_OUT12` as logical `0` or `1`, plus the raw `do_mask`.
- No automatic pulse or timeout is added. The operator is responsible for clearing `do_mask` or `do_enable` from the debugger.

## Safety and interactions

This debug monitor deliberately allows held digital outputs because the operator requested direct debugger control. To reduce accidental interference:

- On boot the monitor is disabled and all debug-controlled outputs are off.
- When a registered test starts, the monitor must be suspended and all debug-controlled DO states must be turned off before the test takes ownership.
- When the registered test ends, the monitor may resume printing, but DO control remains governed by current `do_enable` and `do_mask`.
- The existing safety manager remains responsible for central DO state changes.
- The feature does not change CAN-FD or Ethernet scope.

## Integration points

Add a small independent module:

- `ecu/apps/ecu_board_test/include/debug_monitor.h`
- `ecu/apps/ecu_board_test/src/debug_monitor.c`

Integrate it by:

- Initializing the monitor after board and safety initialization.
- Polling it in the foreground loop path that already polls the status LED and periodic communication transmitter.
- Suspending it around registered tests in `app_shell.c`.
- Adding algorithm-level selftests for view selection, clamping, DO mask mapping, and formatted sample output where hardware can be abstracted.

## Verification

Required evidence before marking implementation complete:

- A red/green selftest cycle for the hardware-independent monitor logic.
- GNU build success with the default configuration.
- SEGGER project generation or SES build check still succeeds.
- J-Link download succeeds if the board is connected.
- COM9 observation confirms that setting `g_ecu_debug_monitor.enable` and `view` causes UART0 output for at least one non-destructive view.

External electrical confirmation of SBUS, ADC, DI, and DO values remains a bench activity by the operator and must be recorded separately in `docs/ecu-test-progress.md`.
