# ECU 板级全功能测试操作规程

## 设备

限流 12 V 电源、万用表、示波器、J-Link、隔离调试串口、4 路 Classic CAN 回显适配器、3 路 RS485 回显适配器、4 路 RS232 回显适配器、SBUS 接收机/帧发生器、0～5 V 信号源及 DIO 夹具。

Ethernet 因当前无对端设备固定记录 `SKIP_NO_DEVICE`。本版本不包含 CAN-FD 测试。

## 构建和下载

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\build_ecu_test.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\flash_ecu_test.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\serial_console.ps1 -Port COMx -LogPath .\test\reports\<name>.log
```

若本机执行策略允许，也可直接运行脚本。SEGGER Embedded Studio 工程由 `generate_ses_ecu_test.ps1` 生成。

## 上电前

1. 断电检查极性、BOOT 拨码、夹具短路和 12 V 输入阻抗。
2. 电源限流从较低值开始，首次上电监测 12 V、5 V、3.3 V、1.8 V、1.1 V。
3. 不要同时连接 DIO 回环夹具和独立 DO 带载电阻到同一路。
4. CAN 终端电阻测量时必须断开外部总线。

## 执行

串口出现 `ECU_TEST READY` 和自测摘要后执行：

```text
board ECU-001
list
run all
status
report
```

所有危险动作前固件会要求 `y/n` 确认。任何 FAIL、异常电流、器件发热或波形异常应立即断电。回到命令提示符后输入 `abort` 会再次执行统一安全清理；测试进行中如串口无法响应，应直接断开 12 V。

软件复位、看门狗复位和外部复位分别执行后，必须检查启动日志的 `RESET flags`，并重新运行 `SAFE.BOOT`。硬件实测结果只能依据保存的串口日志登记，不能以编译通过替代。

## 报告

- 原始控制台：`YYYYMMDD-HHMMSS_<board-serial>_<firmware-version>.log`
- 结构化结果：`YYYYMMDD-HHMMSS_<board-serial>_<firmware-version>.jsonl`
- `.jsonl` 每行只能包含一个 JSON 对象；从原始日志提取以 `{` 开头、`}` 结尾的结果行。
- `BLOCKED` 表示接线、仪器或前置条件不满足；不得改写为 PASS。
