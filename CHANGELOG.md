# 更新日志

本项目的所有显著变更都将记录在此文件中。

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
并且本项目遵循[语义化版本](https://semver.org/lang/zh-CN/)。

## [未发布]

### 修复
- 标定存储写序列加固，修复"0x70 标定后 TC275 挂死（掉电不恢复）"：擦除前补 `clearStatus`——FSR 错误锁存会让 FMU 静默拒擦、而逐页编程照常执行，对未擦除扇区二次编程产生 ECC 损坏页，`CALIB_init` 启动读取该页即触发 CPU0 同步错误 trap（位于三核同步事件之前，CPU1/CPU2 永远等在 sync），三核全挂；每步擦/编后检查 `FSR.OPER/PROER`；扇区擦除确认（首字节 0xFF）后才允许编程；写失败重试由无限次改为 3 次上限，放弃时串口打 `CALSAVE failed (flash)` / `CALCLEAR failed (flash)`（doc 34 V1.5）

## [1.0.0] - 2026-10-01

首个稳定版，对齐 `mw/app_version.h` 1.0.0。

### 新增
- 按需版本信标：DIAG 0x53/0x24 请求即答 EVT 0x24/0x25（PROTO 消费 CID_DIAG，app_ver 请求标志 CPU0 同轮取走，不再等 5 s 周期；配合 S3 About 页 tap 刷新）

## [0.2.2] - 2026-10-01

### 新增
- 固件版本号落地：`mw/app_version.h` 唯一真源，产物名携带版本，启动横幅打印（APPFW tc275_car vX.Y.Z）
- SBL 版本直读：固定地址 0x80007E00 + magic 校验；版本号上链路（遥测 fwVer + EVT 0x24/0x25 版本信标）
- SBL 接入：槽 A 构建 + OTA 接收接入 + 自检确认（doc/24，整包烧录 merge_hex.py）
- F02 速度闭环 servo + 0x70 台架自动判向
- F04 电池电压采集：VADC G0 CH4 采样经 xcore 出遥测
- 34 号标定/DPT：结果回传、DFlash 记录、0x71~0x74 与 EVT 0x22/0x23
- CPU0/CPU1 硬件看门狗，栈溢出钩子改为上报后断喂复位

### 变更
- SCons 命令行构建入库（SConstruct + site_scons，与 tc275_sbl 同源副本），构建统一 SCons
- 工程名 myCar → tc275_car，统一仓库名/工程名/产物名
- 统一 LF 换行（.gitattributes），保证 contracts 跨平台逐字节校验一致
- SDD 升至 V1.8（同步 F02 闭环与看门狗）

### 修复
- 编码器刻度按实物更正：13 PPR、1061.27 计数/轮转、默认轮径 48
- 测速 8 ms 窗由中值改均值；0x70 判向标定改为按当前符号取反
- 调试 launch 的 SVD 路径对齐本机 ADS 1.10.40

## [0.2.1] - 2026-09-27

### 新增
- 车速显示（F03）：遥测填 vMeasL/R（mm/s）+ odoSession 真值，SPD= 台架行，线格式不变
- CI：GitHub Actions——主机单测与 tag 驱动发布

### 修复
- 链路泵简化：单读快照、LOST 只看 ALIVE、移除重同步活锁

## [0.2.0] - 2026-09-27

板间链路换向 SPI/SF 代码闭合。

### 新增
- CPU2 量产 SPI 链路（QSPI3 主机 + SF 帧）与 USE_SPI_LINK 双构建
- CPU1 霍尔编码器 ×4 测速（GTM TIM0 TIEM 边沿中断 + 软件正交）
- 0x50 DRIVE 摇杆混控为轮速百分比，驾驶命令兼作心跳
- CPU2 链路诊断输出 LINK_diagPrint 与 XCORE_logu

### 变更
- Wi-Fi 驱动由 esp8266 模块换成 CPU2 wifi_at，AT UART 移至 P11.12/P11.10
- 工程目录按 SDD §3.4 重排，doc 体系以 SDD 为唯一设计基准重建

### 修复
- SPI 契约逐行对齐已烧录 C6 固件；相位改 trailing 边沿采样并降慢 CS/数据沿速率
- HTTP keep-alive 回复固定到 +IPD 来源链路

[未发布]: https://github.com/lilicqyu-ship-it/tc275_car/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/lilicqyu-ship-it/tc275_car/compare/v0.2.2...v1.0.0
[0.2.2]: https://github.com/lilicqyu-ship-it/tc275_car/compare/v0.2.1...v0.2.2
[0.2.1]: https://github.com/lilicqyu-ship-it/tc275_car/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/lilicqyu-ship-it/tc275_car/releases/tag/v0.2.0
