# ECU Board-Test Function Comment Design

Date: 2026-06-24

## Goal

Make every hand-written ECU board-test function understandable before reading its body, while keeping comments accurate, concise, and useful during hardware bring-up.

## Scope

The comment pass covers all hand-written `.c` and `.h` files under:

- `ecu/apps/ecu_board_test/include`
- `ecu/apps/ecu_board_test/src`
- `ecu/apps/ecu_board_test/tests`
- `ecu/apps/ecu_board_test/selftest`
- `ecu/boards/ecu_isolation/board.c`
- `ecu/boards/ecu_isolation/board.h`

Generated `pinmux.c` and `pinmux.h` are excluded from the general comment rewrite. The existing targeted CAN-termination safety note in generated `pinmux.c` remains because it documents a deliberate post-generation correction.

## Comment Standard

Public declarations use Doxygen-compatible block comments:

```c
/**
 * @brief Convert a 16-bit ADC result to connector voltage.
 *
 * @param raw          ADC code in the inclusive range 0..65535.
 * @param reference_uv ADC reference voltage in microvolts.
 * @return Estimated connector voltage in microvolts.
 */
uint32_t adc_external_uv_from_raw(uint16_t raw, uint32_t reference_uv);
```

Each public function comment states the applicable items from this list:

- purpose and call context;
- parameter meaning and accepted range;
- units such as microvolts, millivolts, hertz, milliseconds, bytes, words, or channel indexes;
- return-value meaning, including error and blocked cases;
- blocking or polling behavior;
- side effects and hardware resources touched;
- ownership and lifetime rules for pointers or returned static storage;
- safety requirements, cleanup guarantees, and destructive address ranges.

Implementation-only functions receive a concise block comment when their behavior is not obvious from the name. These comments explain intent, hardware rationale, state transitions, or failure handling. Trivial wrappers and direct one-line accessors may use a single concise sentence.

## Hardware-Specific Rules

Comments must explicitly identify:

- external ECU channels use one-based indexes, while C arrays use zero-based indexes;
- RGB and CAN termination controls are active-low;
- RS485 direction is owned by UART hardware DE with the current pinmux;
- operator input waits are blocking to the caller but continue polling the RGB status state machine;
- `status_led_poll()` is foreground-driven and reserves no interrupt or timer peripheral;
- the Safety Manager guarantees fail-safe cleanup for DO, CAN termination, and RS485 direction;
- DO activation is limited to one channel and a 500 ms pulse in the production test;
- RAM tests are destructive; QSPI writes are restricted to `0x807F0000..0x80800000`; EEPROM test data is restored;
- Classic CAN is in scope and CAN-FD is not;
- Ethernet returns SKIP because no link fixture is connected.

## Framework Rules

Comments around the framework must make these relationships clear:

- `main()` performs safe board initialization, target self-tests, backend restoration, and shell entry;
- the shell maps commands to immutable registry descriptors;
- `run all` declares the expected required-test count, so partial sessions cannot report board PASS;
- the runner owns prepare/execute/cleanup ordering and always calls `safety_all_off()`;
- abort matching belongs to one `test_context_t` and cannot span independent tests;
- `SELFTEST.ALL` temporarily installs fake backends and restores real backends afterward;
- test results are printed as operator text plus bounded JSON.

## Style Constraints

- Comments are written in clear technical English.
- Comments explain why, contracts, and non-obvious behavior rather than restating syntax.
- Function names, parameter names, constants, and hardware identifiers are quoted exactly.
- No speculative electrical behavior is documented as fact.
- Existing vendor copyright and SPDX headers remain unchanged.
- This pass does not rename APIs, reformat generated code, or change executable behavior.

## Verification

1. Enumerate every function declaration in hand-written public headers and confirm it has a preceding documentation block.
2. Enumerate function definitions in hand-written `.c` files and inspect all nontrivial static helpers for an intent comment.
3. Run `git diff --word-diff` and confirm the source changes are comment-only.
4. Run `git diff --check`.
5. Rebuild the GNU `flash_sdram_xip` image.
6. Regenerate and rebuild the SEGGER Embedded Studio 8.28 Debug project.
7. Confirm binary behavior-sensitive source lines are unchanged except unavoidable comment placement.

## Non-Goals

- No new ECU feature or test case.
- No change to electrical thresholds, timeouts, test order, result logic, or hardware mapping.
- No generated pinmux rewrite.
- No attempt to document HPM SDK internals that are already described by the vendor SDK.
