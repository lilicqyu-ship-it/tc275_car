# 34 · TC275 编码器判向标定（0x70 / DPT）——功能现状与需求

| 项 | 内容 |
|---|---|
| 文档编号 | 34 |
| 域 | TC275 侧（3x） |
| 状态 | **V1.5（已落地）**（2026-10-01：**§8.3 写序列加固——修复"0x70 标定后 TC275 挂死"**。台架复现：S3 显示 RL/RR INVERTED 结果后整车失联、掉电重启不恢复。根因：FSR 带错误锁存时 FMU **静默拒绝擦除命令**，而逐页编程（每页自带 clearStatus）照常执行——对**未擦除的扇区二次编程**产生 ECC 损坏页；该页被读取（保存时的回读校验、以及每次上电 `CALIB_init` 的加载读）即触发 CPU0 同步数据错误 trap，且 `CALIB_init` 位于三核同步事件**之前**，CPU1/CPU2 永远等在 `IfxCpu_waitEvent`——三核全挂。修复四件套：擦除前补 `clearStatus`（与 `bsp/flash_ota.c` 对齐）；每步擦/编后检查 `FSR.OPER/PROER`；**擦除确认（扇区首字节读 0xFF）后才允许编程**，从结构上杜绝二次编程；写失败重试由无限次（§11.4 C6）改为 `CALIB_FLASH_RETRIES=3` 次上限，放弃时串口打 `CALSAVE failed (flash)` / `CALCLEAR failed (flash)`。V1.4（已落地）（2026-09-30：**编码器硬件刻度修正**——实物 MG310 编码器为 13 PPR 霍尔 AB（×4 = 52 计数/电机转，×20.409 减速 = 1061.27 计数/轮转，48 mm 胎 = 0.1421 mm/计数），§8.1 默认 `wheelDiaMm` 65→**48**（`CALIB_WHEELDIA_DEF`/`ENCODER_WHEEL_DIA_MM` 同批，范围 [30,200] 不变）；换算公式一字未动。V1.3（已落地）（2026-09-27：**§13 新增闭环使能门**——`rt/motor_algo.c` 新增 `g_closedLoopOk`：方向记录 `src=0`（首次烧录 / 校验失败回落 / 0x74 REC_CLEAR）时**强制开环等价**（SERVO `measValid=FALSE` 复用：duty=目标、清积分），把 23 §8.4 一直写着的"校验失败按开环等价行为运行"从口头规定变成固件强制执行，消除"编码器已接线但方向未标定 → 闭环正反馈跑飞"（原点回中电机仍转动的台架现象，§1 证据的延伸）；0x70 DONE / 0x73 REC_SET 自动开门、0x74 关门，门随 RecordLive version 边沿翻转并各打一行日志；§5.3 重写（按 src 分上电行为，删除 M3 完成前"每次上电重跑 0x70"旧口径）、§5.2/§7.2/§10/§12 同步。V1.2 = **§3 M2a + §8 M3' + §9 DPT 命令族全部编码完成**——新增 `mw/calib/`（记录编解码 + CPU0 DFlash 持久化）、xcore 四个新块、`rt/encoder` 参数运行时化、`rt/motor_algo` 结果发布/BUSY 拒绝/jog、0x71~0x74 入口、CPU2 EVT 出站队列；§11 = 实现清单与 5 条偏差（D1 0x22 由 CPU0 组帧、D5 长度 23 B），§12 = 三条待裁决（DF0 真实扇区数、`.cproject` 的 Flash 排除、台架验收）。V1.1 = 追加 §8 DFlash 持久化（取代 §4 bake-in 为正式方案）、§9 DPT 命令族 0x71~0x74 与逐电机 jog、§10 验收追加；V1.0 = 首版） |
| 代码基线 | `main`（含 F02 `73058cd` servo/判向 + F04 `658209b` 电池遥测）之上的本轮工作树 |
| 读者 | 在 tc275_car 作业的 AI / 开发者（实施 M2a/M3 前**必读**，按 [33-ai-codebase-guide.md](33-ai-codebase-guide.md) 进入） |
| 跨库配套 | [esp32c6_car/doc/17-calib-ui.md](../../../../esp32c6_car/doc/17-calib-ui.md)（标定 Web UI 需求；本文 §3 的帧契约与该文档 §4 是同一份协议的两端） |
| 上位文档 | [21 §5.2/§15.3](../20-design/21-software-design.md)（伺服与速度标定）、[22 §5.1/§5.4](../20-design/22-link-spi-design.md)（SF 帧）、[23 §8.4](../20-design/23-wiring.md)（判向规程真源）、[31](31-firmware-architecture.md)（现状架构） |

> **定位**：判向标定功能在 TC275 侧的**单一真源入口**——§2 说清"现在有什么"（F02 已落地，勿重复造轮子），§3/§4 说清"还缺什么、怎么补"（M2a 结果回传帧 / M3 符号固化），§5 沉淀台架硬知识。与 21/22/23 冲突时按 [00-index §3](../../00-index.md) 裁决顺序取权威。

---

## 1. 背景与台架证据

F02 闭环伺服上车后台架复现：UI 驾驶"只能轻微扭动"。串口（COM16，115200）抓取判定**右半桥（E3+E4）编码器聚合符号反**，右侧闭环成正反馈：

```
SRV= 200 76 599 160 -86 656    ← 同一次前向命令：左 duty+599↔实测+76（同号=负反馈正常）
                                  右 duty+656↔实测-86（反号=正反馈，闭环在追假反馈）
SRV= 0 0 37 0 43 -334          ← 摇杆松开（目标 0/0）后右 duty 仍钳在 -334（Kp 项 -34 + 积分钳 -300）
SPD= 0 43 29926 1              ← "空闲"状态右轮累计转出 29.9 m（odoSession 用 |Δ| 累加）
```

根因组合：镜像减速箱（B/C 电机反向装）+ `g_encInvert` 默认全 `+1`（[rt/encoder.c:62](../../../rt/encoder.c)）+ **0x70 自动判向从未跑过**。

判向功能本身 F02 已在 TC275 侧完整落地（见 §2），当前缺口只有三个：

| 缺口 | 影响 | 对应需求 |
|---|---|---|
| 无用户入口（UI 无按钮，BLE 产测通道要 fixture token） | 台架上只能靠笔记本控制台直发 | c6 侧 17 号 M1（不在本仓库） |
| 结果只打串口（`ENCCAL=` 行） | UI 页面拿不到结果，判读依赖人工看串口 | **M2a（本文 §3）** |
| 符号只存 RAM | 每次上电回全 `+1`，标定数据（方向/位置/算法参数）不可保持 | **M3'（本文 §8，V1.1 起改为 DFlash 持久化）** |

---

## 2. 现状盘点（F02 已落地，开发前必读，勿重复实现）

### 2.1 触发链路（CPU2 → CPU0 → CPU1，逐跳）

| 跳 | 位置 | 行为 |
|---|---|---|
| 1 | 上游（esp32c6_car） | v2 命令 0x70 经 `c6_link` 映射为 SF `TYPE_CMD(0x01)` + `CID_DPT(0x04)`，`payload[0]=op=0x70` |
| 2 | [com/link.c:243](../../../com/link.c) | CPU2 `link_dispatch`：CID 白名单 `DRIVE/DIAG/DPT` 放行 → `link_forward(0x70,…)` 入 xcore 命令队列（8 深） |
| 3 | [mw/proto/protocol.c:110](../../../mw/proto/protocol.c) | CPU0 `PROTO_handleCommand` case `PROTO_CMD_DPT_CAL_DIR`(0x70) → `XCORE_dirCalibRequest()`。**设计如此：不发心跳、不受故障锁存门禁**（台架工具，急停由 CPU1 标定分支自行中止） |
| 4 | [rt/motor_algo.c:343](../../../rt/motor_algo.c) | CPU1 1kHz 环 `XCORE_dirCalibConsume()` 消费请求 → `MOTOR_ALGO_calibStart()` |

### 2.2 标定状态机（[rt/motor_algo.c:30-57/183-288](../../../rt/motor_algo.c)）

逐轮（MOTOR_A→B→C→D，轮号兼编码器索引，1:1 对应 E1..E4）执行：

| 阶段 | 时长 | 动作 |
|---|---|---|
| `CALIB_PULSE` | 250 ms | 该轮 `MOTOR_setSpeed(wheel, MOTOR_CALIB_DUTY)`（+12% duty，参数在 [motor_algo.c:38-40](../../../rt/motor_algo.c)） |
| `CALIB_SETTLE` | 80 ms | `MOTOR_stop(wheel)`，读计数 delta：`delta<0` → `ENCODER_setInvert(wheel,-1)`；`delta==0` 记死通道（**保持 +1**，不做方向判定） |

全程 ≈1.4 s；`MOTOR_stopAll()` + `SERVO_reset(0/1)` 开局，结束再 `stopAll`——**标定分支独占电机输出**， servo 目标斜坡暂停但目标继续累积（见 §5.4 判读）。急停（`g_targetEstop || XCORE_estopIsActive()`）走 [MOTOR_ALGO_task](../../../rt/motor_algo.c) 的急停分支 → `calibAbort()` → `ENCCAL aborted (estop)`。

### 2.3 输出与数据归属

- 串口行（CPU1 经 xcore 日志环 → CPU0 UART）：`ENCCAL start`（开局）、`ENCCAL= i0 i1 i2 i3 d0 d1 d2 d3`（结束，前 4 个是各轮符号终值、后 4 个是脉冲计数 delta）、`ENCCAL aborted (estop)`。
- `ENCODER_setInvert/getInvert`（[rt/encoder.c:327-339](../../../rt/encoder.c)）是 `g_encInvert` 的唯一写者/读者接口，CPU1 属主，ISR 立即生效。**运行时 RAM 态，上电回 `{+1,+1,+1,+1}`**。
- 判定语义：`delta<0` = 该轮编码器计数方向与"车头向前"约定相反，自动翻；`delta==0` = **无计数**（接线/传感器故障），不是方向问题，禁止手改 invert 掩盖。

---

## 3. 需求 M2a：标定结果回传帧（本仓库实施部分）

### 3.1 SF 帧契约（与 esp32c6_car 17 号 §4.1 同一份协议）

| 字段 | 值 |
|---|---|
| TYPE | `SF_TYPE_EVT(0x05)`（既有） |
| CID | **新增** `SF_CID_DPT_RESULT = 0x22u`（[mw/sf/sf_frame.h](../../../mw/sf/sf_frame.h)；0x20 ERROR / 0x21 STATE 已占，0x22 空闲） |
| 方向 | 仅 TC275 → C6，每次标定结束发一帧 |
| payload（**22 B** + `saved` = **23 B**，≤248） | `[0] op u8 = 0x70`，`[1] status u8`，`[2..5] invert i8×4`，`[6..21] delta i32×4`，`[22] saved u8` |
| status | 0=完成 1=急停中止 2=忙（标定进行中又收到请求） |
| 多字节序 | **小端**，与既有 `SF_getU16/SF_getU32/SF_putU32`（[sf_frame.h:206-229](../../../mw/sf/sf_frame.h)）一致——DRIVE 帧 v/w 已是同款 |

### 3.2 xcore 新块 `CalibResult`（照 [33 §4](33-ai-codebase-guide.md) 规约）

```c
/* mw/xcore/xcore.h —— CPU1 唯一写者，CPU0 消费（§3.4 偏差 D1） */
typedef struct
{
    uint8  pending;        /* 1 = 有未取走的结果（CPU0 take 原子清零） */
    uint8  status;         /* 0 完成 / 1 急停中止 / 2 忙              */
    sint8  invert[4];      /* 标定后各轮符号终值                       */
    sint32 delta[4];       /* 各轮脉冲计数 delta（后立 invert 语义）    */
} XcoreCalibResult;

void XCORE_calibResultPublish(const XcoreCalibResult *res); /* CPU1：写块+置 pending */
boolean XCORE_calibResultTake(XcoreCalibResult *res);       /* CPU0：锁内读+清 pending，返回有无 */
```

- `XCORE_init` 清零；访问器锁内只拷这几十字节（CPU1 硬实时会为锁自旋，临界区必须小）。
- `delta` 的"未测轮"（急停中止时）填 0，不造数。

### 3.3 CPU1 改动点（[rt/motor_algo.c](../../../rt/motor_algo.c)，不动状态机时序）

| 时机 | 动作 |
|---|---|
| `MOTOR_ALGO_calibStep` 正常走完 4 轮（现有 `wheel >= MOTOR_COUNT` 收尾处） | `Publish{status=0, invert=ENCODER_getInvert(0..3), delta=g_calib.delta}` |
| `MOTOR_ALGO_calibAbort` | `Publish{status=1, 已测轮 delta/invert 照实，未测轮 0}` |
| `XCORE_dirCalibConsume()` 返回真但 `g_calib.active` 已真 | **不再重启标定**：`Publish{status=2}`（现状代码会 restart，需改为拒绝+busy） |

### 3.4 发送点（落地后为 **CPU0 组帧 + CPU2 发送**，见 §11 偏差 D1）

- 原稿（V1.0）：CPU2 在 20 ms 遥测节拍旁 `XCORE_calibResultTake()` → 组 payload → `LINK_send(SF_TYPE_EVT, SF_CID_DPT_RESULT, payload, 23)`。
- **实现改为 CPU0 消费 `CalibResult`、组帧后压入 xcore EVT 出站队列，CPU2 只管发送**：`saved` 字节（§9.4）由写 DFlash 的 CPU0 才知道，CPU2 若自行 take-send 就永远填不出真值。
- 发送失败（TX 队列满）→ 帧**留在队头**，下个 20 ms 节拍重试到发出为止；一次标定恰好一帧（CPU0 侧有 hold 机制保证写 flash 完成前不发 0x22）。
- 无标定发生时零流量。

### 3.5 验证与文档同步（tc275_car 红线：改行为同批改文档）

- **主机单测**：SF 层动了 `sf_frame.h`（新增 CID 常量）+ payload 编解码 → [test/host/test_sf.c](../../../test/host/test_sf.c) 补 EVT 0x22 往返用例，按 [33 §7](33-ai-codebase-guide.md) 命令跑绿。xcore 块与 CPU1/CPU2 钩子无主机单测（依赖 iLLD/FreeRTOS 桩之外的真实核间行为）——**声明：此改动未被单测覆盖，需台架验证**。
- 同批更新：`22 §5.1/§5.4`（CID 载荷契约表登记 0x22）、`21 §5.2`（判向功能补结果回传）、`31`（xcore 通道表加 CalibResult、DPT 段补回传）、`23 §8.4`（规程补"结果经 UI 显示"）、本文件状态。

---

## 4. 需求 M3：判向符号固化（bake-in）

> **V1.1 变更**：应用户需求，标定数据持久化改走 **DFlash（§8）**，本节 bake-in **降级为 DFlash 功能不可用时的应急路径**，流程保留不变。

**选定方案：台架确认后写死进固件默认值。**

```
台架跑 M1 标定页（或控制台直发 0x70）
  → 记录 ENCCAL= 的 invert[0..3]
  → 手写进 rt/encoder.c:62 的 g_encInvert 默认初值
  → AURIX Development Studio (TASKING) 构建烧写（主机/CI 编不了固件）
  → 复跑 0x70 验证：全部 delta ≥ 0（不再翻转）= 固化正确
  → 同批更新：23 §8.4（默认值表）、21 §5.2、本文件状态
```

**明确排除**：TC275 侧 NVM/EEPROM 运行时持久化。理由：TASKING 环境下 flash 编程引入 ENDINIT/OTA 交互风险（[21 §18 C8](../20-design/21-software-design.md) 铁律域）、磨损与掉电一致性复杂度，而判向是**一次性台架事实**，符号不会因运行漂移——bake-in 零运行时风险且可版本化。若未来电机/编码器换型，重跑一次 0x70 + 重 bake 即可。

---

## 5. 标定质量规程与台架硬知识（2026-09-27 台架实测得出）

### 5.1 弱 plant 风险：12% 脉冲可能转不动轮子

实测增益严重低于设计假设：duty 656（≈66%）↔ 实测仅 ~76 mm/s（设计假设满占空比 ≈1000 mm/s）。若车在地面上（有载荷）或电池偏低，`MOTOR_CALIB_DUTY=120` 的 250 ms 脉冲可能克服不了静摩擦 → `delta≈0` → **误报"死通道"**。规程：

1. **必须四轮离地**（23 §8.4 前提，UI 页面强制确认）；
2. `delta==0` 时先验证编码器本身活着：手转该轮，看 `SPD=` 行 odo 是否增长 / `alive=1`；
3. 编码器确活且怀疑弱 plant → 提高 `MOTOR_CALIB_DUTY 120→200` 或 `MOTOR_CALIB_PULSE_MS 250→400`（[motor_algo.c:38-40](../../../rt/motor_algo.c)），**同批改 23 §8.4 的参数记载**；
4. 顺手记录 `SPD=` 速度与 duty 的比值，供 §15.3 速度标定（`ENCODER_FULL_SCALE_MM_S` 整定）用——**那是另一个任务，不在本文范围**。

### 5.2 结果判读语义（页面文案与人工判读共用）

| 现象 | 结论 | 动作 |
|---|---|---|
| `delta < 0` | 编码器计数方向与前进约定相反 | 已自动翻 `-1`，正常 |
| `delta == 0` | 无计数：接线/传感器/供电故障 | **查线，禁止手改 invert 掩盖** |
| 上电即见 `SERVO open-loop (record src=0; calibrate via 0x70)` | 闭环使能门关闭（§13）：记录是默认/回落态，此时 `SRV=` 的 duty 恒等于 target | **先跑 0x70 判向**再谈闭环表现 |
| 推杆时 `SRV=` 的 **duty == target**（且门日志 = open-loop） | 开环签名，非伺服失控：闭环根本没参与 | 判向/记录生效后复测 |
| 标定后推杆，`SRV=` 实测与 duty 同号（两侧） | 闭环负反馈恢复 | 可路试 |
| 标定后空闲 `SRV=` 仍有小残留 duty | 见 §5.4，积分冻结特性 | 另立 servo 调优任务 |

### 5.3 上电行为与标定时机（V1.3 重写，配合 §13 闭环使能门）

方向符号不再"每次上电回全 `+1`"——M3'（§8）起由 DFlash 记录决定，上电行为按记录的 `src` 分两种：

| 上电时记录 src | 含义 | 闭环状态 | 动作 |
|---|---|---|---|
| `0`（默认 / 校验失败回落 / 被 0x74 清过） | 方向未标定，invert 是编译期猜测 | **门关闭 = 开环等价**（§13）：驾驶手感等同 F01 开环，duty 恒等于目标 | 首次闭环驾驶**之前**跑一次 0x70（四轮离地），DONE 后 ~10 ms 内自动开门 |
| `1`（DFlash）/ `2`（在线设置） | 方向已标定且已持久化 | 门开着，闭环直接生效 | 正常驾驶；换电机/编码器或接线变动后重跑 0x70 |

要点：

- **门开着 ≠ 方向一定对**：src=1/2 只表示"有标定事实"，若机械变动后没重标，闭环仍会正反馈——那是操作遗漏，不是门失效（门只挡"从未标定"，挡不了"标定过时"）；
- 0x74 REC_CLEAR 后门重新关闭（回开环），与记录数据回默认同步发生，不会留下"默认符号跑闭环"的窗口；
- 出厂首次烧录（flash 全 0xFF，校验必失败）必然落在 src=0 开环档——这正是 §1 台架现象（编码器已接线 + 默认符号 + 闭环跑飞）被彻底封死的入口。

### 5.4 相关现象判读（别误判成标定失败）

- **空闲残留 duty**：servo 积分在测量死区（`SERVO_E_DEADBAND=3`）内冻结，轮子被小 duty 顶住不动时实测恒 0、误差恒 0，积分不再 unwind——本次台架左侧 `duty=37`（4% 不到，不推动车）即此特性，非缺陷爆发点；右侧钳 `-300` 是符号反的正反馈放大版，**修方向后复测再评估**。
- **odo 虚增**：`SPD=` 的 odoSession 用 `|Δ|` 累加，噪声计数会单调虚增里程，别拿它当位移真值。
- **0x70 不是驾驶命令**：不发心跳、不受故障门禁（§2.1），别在应用层拿它当保活用。

---

## 6. 验收标准

**M2a（本仓库）：**

1. `test/host/test_sf.c` 补 EVT `0x22` 往返用例并全绿（CI 同步过）；
2. 台架：c6 标定页触发 → TC275 串口出 `ENCCAL=`，同时 c6 侧收到 `{"t":"cal","status":0,...}`，`invert/delta` 与串口行逐值一致；
3. 标定中发 0x32 急停 → 串口 `ENCCAL aborted (estop)` + 页面收到 `status=1`；
4. 标定进行中重复触发 → `status=2`，原标定不被打断；
5. 遥测/驾驶/GET_STATUS 回归不受影响（TX 队列无新压力源：一帧/次）。

**M3：**

6. bake-in 后断电重启，推杆 `SRV=` 两侧实测与 duty 同号；空闲残留 duty 消失（回到 0 死区附近）；
7. 复跑 0x70：四轮 `delta ≥ 0`（无翻转发生）；
8. `23 §8.4`/`21 §5.2`/`00-index §2` 同批更新完毕。

---

## 7. 附录

### 7.1 代码锚点速查

| 文件 | 锚点 | 内容 |
|---|---|---|
| rt/motor_algo.c | 30-57 / 183-288 / 343 | 标定参数 / 状态机 / 消费点 |
| rt/encoder.c | 62 / 327-339 | `g_encInvert` 默认值 / setInvert/getInvert |
| rt/servo.c | 45-52 | `!alive` 开环回退（判向期间编码器有效，不回退） |
| mw/proto/protocol.c | 110-116 | 0x70 命令入口（无心跳无门禁） |
| com/link.c | 243 | CID 白名单（DPT 放行） |
| mw/xcore/xcore.c | 49-93 等 | 块/锁/清零规约样板 |
| mw/sf/sf_frame.h | 60-96 / 206-229 | TYPE/CID 表 / LE 多字节助手 |
| Cpu2_Main.c | 195-215 / 251+ | `SPD=` 行 / 主循环与遥测节拍（M2a 发送点挂此） |

### 7.2 串口行格式（115200 8N1，CPU0 ASCLIN0）

```
SRV= tL mL dL tR mR dR              1 Hz，percent*10（运动时才打）
SPD= vL vR odo alive                1 Hz，mm/s / mm / 0|1
ENCCAL start / ENCCAL= 8 值 / ENCCAL aborted (estop)
SERVO open-loop (record src=0; calibrate via 0x70)   门关闭，一次性（§13）
SERVO closed-loop enabled                            门打开，每次翻变各打一次
LINKERR=/LINKDBG= 18 值             变化或异常时（state/irq/clk/ready/…/err 计数）
```

### 7.3 单位域备忘（改本功能必查）

`±100`（协议/robot 层）→ `±1000`（xcore 目标/servo/编码器 pct，CPU0 在 `XCORE_motorSetTarget` 处 ×10）→ `mm/s`（遥测物理域）。判向本身无量纲（±1 符号）；`delta` 是原始四倍频计数。

---

## 8 · V1.1 追加：DFlash 持久化（M3'，正式方案）

> **变更说明**：V1.0 §4 曾以 ENDINIT/OTA 风险为由排除 NVM 持久化；V1.1 按用户需求改为**必须持久化**（标定数据 = 电机位置、运动方向、算法参数）。风险用 §8.3 的写时机与停顿预算控制，§4 bake-in 降级为应急路径。跨库协议见 §9 与 esp32c6_car 17 号 §8.4。

### 8.1 存储介质与记录布局

- 介质：TC275 **DFlash0**（数据保持型，掉电不丢）。**实现时按 iLLD 常量实测的结论**（`_Impl/IfxFlash_cfg.h` / `IfxFlash_cfg.c`，非记忆）：
  - 基址 `IFXFLASH_DFLASH_START = 0xAF000000`；**逻辑扇区 0x2000（8 KB）**、**页 8 B**（`IFXFLASH_DFLASH_PAGE_LENGTH`）——本文件 V1.1 原稿猜的"页典型 64 B、扇区典型 4 KB"**两个都不对**，以本行为准；
  - iLLD 的 DF0 逻辑扇区表列出 **48 个扇区**（至 `0xAF05FFFF`），而 **21 §4.1 记 TC275 DF0 = 128 KB（16 扇区）**——iLLD 表按器件族取上限，**两者矛盾未裁决**（见 §11 待确认 Q1）；
  - HSM 日志在 `0xAF110000..`、UCB 在 `0xAF100000..`，与所用扇区不相交；
  - **落点 = DF0 逻辑扇区 15（`0xAF01E000`）**：在"16 扇区"口径下是末扇区、在"48 扇区"口径下同样是用户扇区，两种读法都合法；把扇区 0..14 留给 21 §4.3 的配置页/黑匣子（**它们不得占用扇区 15**）；
  - 20 B 记录跨 **3 个 8 B 页**（末页零填充），按"每页 enterPageMode→loadPage→writePage"逐页编程（ECC 按页算），回读只比对 20 个记录字节；
- 记录 `CalibRecord`（20 B）：

| 偏移 | 字段 | 类型 / 默认值 |
|---|---|---|
| 0..3 | magic `'S','D','C','1'` | 固定 |
| 4 | ver | u8 = 1 |
| 5 | src | u8（0 默认 / 1 DFlash / 2 在线设置） |
| 6..9 | pos[4] | u8×4：0 前左 / 1 前右 / 2 后左 / 3 后右（默认按 23 §3 电机表：A 前左、B 后左、C 后右、D 前右） |
| 10..13 | invert[4] | i8×4（0x70 结果；默认 +1） |
| 14..15 | fullScaleMmS | i16（LE），默认 1000（= `ENCODER_FULL_SCALE_MM_S`） |
| 16..17 | wheelDiaMm | i16（LE），默认 48（V1.4 起对齐 MG310 实配 48 mm 胎，= `CALIB_WHEELDIA_DEF`/`ENCODER_WHEEL_DIA_MM`；初版 65） |
| 18..19 | crc16 | 覆盖 0..17，用 SF 帧同款 CRC16 算法（sf_frame.c 现行多项式，不新造） |

- **单槽 + magic/CRC/范围三重校验**：任一失败 → 整体回落默认值并按 `src=0` 上报（EVT 0x23 的 `crcOk=0`）。磨损论证：DFlash 扇区擦写寿命典型 ≥10 万次，台架保存频率 ≤10 次/天 → 数十年量级，**无需磨损均衡**（写明依据，防未来过度设计）；
- 范围校验：`fullScaleMmS ∈ [100,5000]`、`wheelDiaMm ∈ [30,200]`、`invert ∈ {+1,-1}`、`pos ≤ 3`，越界按 CRC 失败处理。

### 8.2 算法参数运行时化（rt/encoder.c）

- `ENCODER_FULL_SCALE_MM_S` 宏与轮径宏常量（初版 `65.0f`，V1.4 起默认 48）改为**运行时变量** `g_fullScaleMmS / g_wheelDiaMm`（初值 = 宏默认），`ENCODER_task` 测速与 `ENCODER_publish` 的 pct 换算**共用同一变量**；CPU2 的 vTarget 遥测换算（Cpu2_Main）加载同一数值（经 xcore `RecordLive` 广播或同款加载流程，实现时二选一并写明）；
- **公式一律不动，只换数据源**（改公式 = 重调整个闭环，doc/33 §5.2 红线）；
- 除零/越界保护：换算处对非法值钳回默认（防 DFlash 位翻转把除零带进 1 kHz 环）。

### 8.3 启动加载与写入时机

**加载**：CPU0 启动早期（`core0_main` 初始化段、调度器启动前）读 DFlash → 三重校验 → 发布 xcore `RecordLive{valid, rec}` → CPU1 在 1 kHz 环检测 `valid` 即应用（invert → `ENCODER_setInvert`，fullScale/wheelDia → §8.2 变量，pos → 元数据）；应用前按默认值运行（与现状一致，安全）。DFlash **读**是内存映射，无停顿。

**写入**（唯一执行核 = CPU0，触发三处）：

| 触发 | 行为 |
|---|---|
| `0x70` 成功（M2a 结果 status=0） | 自动写入（invert 更新，src=1） |
| `0x73 REC_SET` | 参数生效 + 写入（src=2） |
| `0x74 REC_CLEAR` | 擦扇区 + 发布默认值（src=0） |

- 写前检查：车辆运动中（目标/实测非 0）则**延迟到静止**再写；写入序列（V1.5 加固，见状态行）= 喂狗 → 关本核中断 → **clearStatus** → 擦扇区 → **查 D0BUSY + FSR.OPER/PROER** → **确认扇区首字节读 0xFF（擦除生效）** → 逐页编程（每页同样查错）→ 回读校验 → 恢复中断 → 喂狗；任一步失败即**放弃本次编程**（擦除未被确认前绝不写页——对未擦除扇区二次编程会造出 ECC 损坏页，该页被读取即 trap 读核，见状态行根因），重试 ≤3 次（每 `CALIB_WRITE_IDLE_MS` 一次），耗尽后打 `CALSAVE failed (flash)` 并保留 RAM 态生效；
- **停顿预算**：页编程 ~几十 µs、扇区擦除 ~几十 ms（以手册为准）——每次保存只发生一次，仅台架场景；CPU0 WDT 窗口 0.3~0.5 s 可容纳（§6.2 的"跑着复位"排查口径不受影响）；CPU1/CPU2 无感；
- **ENDINIT 铁律**（21 §18 C8）：解锁/恢复走 `bsp/wdg.h` 的 clear+set 模式，**禁用 `IfxScuWdt_serviceCpuWatchdog`**；
- **OTA 期间禁止写**（flash 控制器互斥）；CPU0 写入期间 SPI 泵（CPU2）短暂停顿属预期，`LINK_ALIVE_TIMEOUT_MS=500` 远大于擦除耗时，不触发失联。

### 8.4 范围界定

- **生效**：invert（闭环方向）、fullScaleMmS、wheelDiaMm、pos（元数据 + UI 显示）；
- 电机驱动方向**不在本协议范围**：换向属接线级事实（`bsp/motor.c` 的 `g_dirInvert` 编译期表，见 33 §PWM 段），0x70 只修闭环计数方向，`CalibRecord` 与 0x71~0x74 载荷均无该字段，**不改 `bsp/motor.c`**；
- 侧分组（左 = A+B）保持编译期，按位置重映射混控为非目标（避免混控/斜坡/伺服全链联动改动，另立任务评估）。

## 9 · V1.1 追加：DPT 命令族扩展（0x71~0x74）与逐电机 jog

### 9.1 命令表（与 esp32c6_car 17 号 §8.4 同源；v2→SF 映射零改动，入口全在 `PROTO_handleCommand` 新 case）

| op | 名称 | payload（LE） | TC275 行为 | 应答 |
|---|---|---|---|---|
| 0x70 | CAL_DIR（既有） | ∅ | 判向标定 + 成功后自动持久化（§8.3） | EVT 0x22（追加 `saved u8` → 23 B） |
| 0x71 | MOTOR_JOG | `{motor u8, duty i16}` 3 B | 写 xcore `JogCmd`；duty=percent×10 **钳 ±500**；**受故障锁存门禁、不发心跳** | 无（newest-wins，300 ms 固件超时） |
| 0x72 | REC_GET | ∅ | 读 `RecordLive` | EVT 0x23 |
| 0x73 | REC_SET | 12 B（pos/invert/fullScale/wheelDia） | 参数生效 + DFlash 写入 | EVT 0x23（回执） |
| 0x74 | REC_CLEAR | ∅ | 擦扇区 + 恢复默认 | EVT 0x23（回执） |

- 0x71~0x74 与 0x70 一样走 DPT CID 白名单（link.c 无需改），但**语义分层**：0x71 属驱动类（故障门禁），0x70/0x72/0x73/0x74 属台架工具类（无门禁，同 §2.1 口径）；
- 0x70~0x79 之外的 op 仍落 `default` 忽略。

#### 9.1.1 事件载荷逐字节表（TC275 → C6；与 esp32c6_car 17 号 §4.1/§8.4 同一份契约）

**EVT `0x22` 标定结果（23 B，`SF_TYPE_EVT` / `SF_CID_DPT_RESULT`）**

| 偏移 | 字段 | 类型 | 语义 |
|---|---|---|---|
| 0 | op | u8 = 0x70 | 本结果回答的命令 |
| 1 | status | u8 | 0 完成 / 1 急停中止 / 2 忙 |
| 2..5 | invert[4] | i8×4 | 下标 = 轮号 A..D（非 pos） |
| 6..21 | delta[4] | i32×4 LE | 同上；`status!=0` 时未测轮填 0（C6 侧显示"未测"，不判死通道） |
| 22 | saved | u8 | 0 未持久化 / 1 已写 DFlash / 2 写入失败 |

**EVT `0x23` 生效参数（15 B，`SF_CID_DPT_REC`）**

| 偏移 | 字段 | 类型 | 语义 |
|---|---|---|---|
| 0 | ver | u8 = `CALIB_REC_VER` | 记录布局版本 |
| 1 | src | u8 | `CALIB_SRC_DEFAULT/DFLASH/ONLINE` |
| 2..5 | pos[4] | u8×4 | 下标 = 轮号 A..D，值 = 0 前左 / 1 前右 / 2 后左 / 3 后右 |
| 6..9 | invert[4] | i8×4 | 当前生效计数方向符号 |
| 10..11 | fullScaleMmS | i16 LE | |
| 12..13 | wheelDiaMm | i16 LE | |
| 14 | crcOk | u8 | 0 = DFlash 校验/范围失败已回落默认 |

> **长度踩坑记录（已修，回归口径）**：M2a 首版把本节 §3.1 V1.0 的"26 B"当真，`CALIB_EVT_RESULT_LEN` 写成 27u、`saved` 落在 `buf[26]`，于是 `payload[22..25]` **无人赋值**——栈垃圾随帧发出，且 C6 按 `[22]` 读 `saved` 拿的是脏值。现 `calib_record.h` 常量 23u + `CALIB_EVT_RESULT_SAVED 22u`，`test_sf.c` 用 `LEN == SAVED+1` 与 `2+4+16+1 == LEN` 两条**算术不变式**守住连续性（只断言偏移读不到"未赋值"这一类洞）。新增事件帧一律先算字段和再定长，别抄标称值。

### 9.2 xcore 新块（照 33 §4 规约：lock 访问器 + `XCORE_init` 清零）

```c
typedef struct { sint16 duty[4]; uint32 seq; } XcoreJog;      /* CPU0 写 / CPU1 读   */
typedef struct { boolean valid; XcoreCalibRecord rec; } XcoreRecordLive; /* CPU0→CPU1/CPU2 */
```

`XcoreCalibRecord` 即 §8.1 的 `CalibRecord` 内存态（去 magic/crc，保留 ver/src/pos/invert/fullScale/wheelDia）。

### 9.3 CPU1 jog 模式（rt/motor_algo.c）

- 优先级：**急停 > 标定 > jog > 伺服**；
- jog 活跃判定：`JogCmd.seq` 变化后 300 ms 内；活跃期间 `MOTOR_ALGO_task` 跳过 `controlStep`，按 `duty[0..3]` 逐电机 `MOTOR_setSpeed`（**开环、不过伺服**——台架手动场景，编码器不参与），`XCORE_motorStatusSet` 照常发布；
- 300 ms 无刷新 → jog duty 全 0，回伺服路径；急停分支照旧刹车并清 jog 状态；
- 与 M2a 的关系：jog 分支下 `MOTOR_ALGO_diag` 的 `SRV=` 行照常可打（target 视为 0，duty 为 jog 值），便于台架对数。

### 9.4 CPU0 侧（mw/proto/protocol.c + Cpu0_Main 路径）

- 0x71 → 校验 motor<4 → `XCORE_jogSet(motor, duty)`；
- 0x72/0x73/0x74 → 触发 §8.3 流程（读/写/清），完成后组 EVT 0x23 payload 经 CPU2 发送（M2a 同款 take-send 通道，或复用 `CalibResult` 块扩展——实现时保持"一次标定/一次变更恰好一帧"）；
- 0x22 payload 追加 `saved u8`（22→23 B，偏移 `[22]`）：0=未持久化 1=已写 DFlash 2=写入失败。

### 9.5 主机单测与文档同步

- `test/host/test_sf.c`：0x22 扩展字段 + 0x23 往返用例；`test_sf_telemetry` 不动（38 B 遥测布局零改动）；
- xcore 新块 / jog 模式 / DFlash 读写 **无主机单测，需台架验证**（33 §7 口径）；
- 同批更新：`22 §5.1/§5.4`（CID 0x22 扩展 + 0x23 登记）、`21 §5.2/§6.3`（参数持久化与 jog）、`31`（xcore 通道表）、`23 §8.4`（规程：0x70 自动持久化 + REC 参数）、`33`（xcore 块表）、本文件。

## 10 · V1.1 验收追加

10. **断电保持**：`0x70` 成功 → 断电 → 上电 `REC_GET` 返回 `src=1`、invert 与保存值一致；推杆 `SRV=` 两侧实测与 duty 同号；
11. **CRC/首次上电**：无记录或校验失败 → `src=0` 默认值、`crcOk=0`、系统可用（开环等价回退）；
12. **参数生效**：`REC_SET` 改 `fullScaleMmS` → `SRV=` 的 pct 换算按新值；改 `wheelDiaMm` → `SPD=` 的 mm/s 按新值；
13. **写入停顿**：保存瞬间（擦+写 ~几十 ms）后链路无失联（`LINKDBG` state 恒 READY）、无看门狗复位；
14. **jog 三重安全**：故障锁存时拒收；300 ms 超时停；急停立即停；duty 钳 ±500；
15. **主机单测**全绿 + 本节覆盖盲区声明（xcore/jog/DFlash 需台架）。

**V1.3 追加（§13 闭环使能门）：**

16. **门默认关闭**：出厂首次烧录 / 校验失败回落 / 0x74 清记录后上电，串口出现一次性 `SERVO open-loop (record src=0; calibrate via 0x70)`；此时推杆 `SRV=` 的 duty 恒等于 target（开环签名，见 §5.2）；
17. **0x70 自动开门**：四轮离地跑 0x70 至 DONE → ~10 ms 内（CPU0 持久化翻转 RecordLive version 即开门，无需等 DFlash 写完）串口出 `SERVO closed-loop enabled`；此后推杆 `SRV=` 实测与 duty 同号；0x74 清记录后门随记录回默认重新关闭并再次打印 open-loop 行。

---

## 11 · V1.2 实现落地（2026-09-27，M2a + M3' + §9 全部编码完成）

### 11.1 改动清单

| 文件 | 落地内容 |
|---|---|
| `mw/sf/sf_frame.h` | 新增 `SF_CID_DPT_RESULT 0x22` / `SF_CID_DPT_REC 0x23` |
| **`mw/calib/calib_record.c/.h`（新）** | 纯 C99 编解码层（与 `sf_frame.h` 同规矩，主机可编）：`CalibRecord` + 20 B blob（magic/ver/src/pos/invert/fullScale/wheelDia/crc16，CRC 复用 `SF_crc16`）、三重校验 `decode`、`fillDefaults`、`paramsOk`、`jogDecode`（钳 ±500 + `motor<4`）、`recSetDecode`（12 B，长度不符直接拒）、EVT 0x22/0x23 组帧 |
| **`mw/calib/calib_store.c/.h`（新）** | CPU0 持久化：DF0 扇区 15 单槽；`CALIB_init`（同步事件前加载并广播）、`CALIB_tick`（消费 CPU1 结果 + 延迟写队列）、`CALIB_sendRecord/recordSet/recordClear`；写序列 = 喂狗→关中断→擦→逐页写→回读→开中断→喂狗，ENDINIT 走 `bsp/wdg.h`，**未用 `IfxScuWdt_serviceCpuWatchdog`** |
| `mw/xcore/xcore.h/.c` | 新块 `CalibResult`（CPU1 写 / CPU0 取）、`Jog`（4×duty + `jogSeq` 序号）、`RecordLive`（`version` 计数器 + 记录）、**EVT 出站环形队列**（8 槽 × ≤32 B，**只有 CPU0 压**（`mw/calib`）、CPU2 取发） |
| `rt/encoder.c/.h` | §8.2 参数运行时化 `g_fullScaleMmS/g_wheelDiaMm`（**公式一字未动**，只换数据源；setter 越界回落宏默认）；新增 `ENCODER_WHEEL_DIA_MM` 宏（初版 65，V1.4 起默认 48）取代散落的 `65.0f` |
| `rt/motor_algo.c` | `MOTOR_ALGO_calibPublish()`（DONE / ABORT / **BUSY 拒绝且不重启**）；`MOTOR_ALGO_applyRecord()`（`version` 边沿应用 invert/fullScale/wheelDia）；`MOTOR_ALGO_jogStep()`（`jogSeq` 变化续期、**300 ms 超时归零 + `SERVO_reset` + `MOTOR_stopAll`**）；1 kHz 环次序见 §11.3 |
| `mw/proto/protocol.c/.h` | `PROTO_CMD_DPT_MOTOR_JOG/REC_GET/REC_SET/REC_CLEAR`（0x71~0x74）；jog 受故障锁存 + 急停门禁，拒收打日志；0x70~0x74 均不发心跳 |
| `Cpu0_Main.c` | `core0_main`：`XCORE_init → STIME_init → UART_init → CALIB_init`（UART 必须先于 CALIB_init——其加载日志走 ASCLIN0，未初始化访问 SFR 即 bus error；均在放同步事件**之前**，CPU1 起来即有好记录）；`vRobotControlTask`：`ROBOT_task → CALIB_tick → WDG_serviceCpu` |
| `Cpu2_Main.c` | 20 ms 节拍新增 `link_sendPendingEvents()`（peek→send→pop，**发送失败留在队头重试**）；vTarget 换算改读 `RecordLive.fullScaleMmS` |
| `com/link.c` | 只改注释（DPT 通道语义 = 标定/记录族，应答改走 EVT 0x22/0x23）；**分派逻辑零改动**（12 B REC_SET ≤ `PROTO_MAX_PAYLOAD`16，实测确认） |
| `.cproject` | **仅 `TriCore Debug (TASKING)`**：解除 `Libraries/iLLD/TC27D/Tricore/Flash` 与 `Flash/Std` 的 excluding（`IfxFlash.c` 自此参与编译），include 路径加 `Flash`、`Flash/Std` 与 `mw/calib`。**Release (TASKING) 与两个 GCC 配置未动** → 见 §11.4 C7 |
| `test/host/test_sf.c` | 3 个用例组：0x22/0x23 逐字节往返、blob 编解码与三重校验（含单 bit 翻转、CRC 修好后的非法值、范围边界）、0x71/0x73 命令体（钳位与拒收） |

### 11.2 与原稿的偏差（实现后回写，非静默）

| 编号 | 偏差 | 原因 |
|---|---|---|
| **D1** | EVT 0x22 由 **CPU0** 组帧并压 xcore 出站队列，CPU2 只负责发送（§3.4 原稿是 CPU2 直接 take-send） | `saved` 字节只有执行 DFlash 写入的 CPU0 知道；CPU2 自行 take-send 填不出真值。副作用：**新增"一次变更恰好一帧"的 hold 机制**（DONE 结果按住到写完再发，写失败发 `saved=2`） |
| **D2** | `CalibRecord.pos[4]` 落 flash 与线上，但**不参与任何运行时行为**（混控/斜坡/servo 仍按编译期 A+B=左） | §8.4 已把位置重映射列为非目标；`pos` 目前是元数据 + UI 显示，不做暗示它生效的注释 |
| **D3** | `RecordLive` 用 **`version` 递增计数**判变更，不用 §8.3 的 `valid` 布尔 | `XCORE_init` 把共享 RAM 清零，`valid` 与"没发布过"在值上不可区分（清零 `CalibRecord` 恰好等于合法记录）。init 以 `0xFF` 起算，CPU1 见到边沿即应用 |
| **D4** | §8.2 的"CPU2 换算共用"选了**经 xcore `RecordLive` 广播**这一支（原稿允许二选一） | CPU2 不重复读 DFlash，避免两核各自校验出分叉结论 |
| **D5** | 0x22 长度 **23 B**（`[22]=saved`），非 V1.1 的 26/27 B | 见 §11.4 C3：原稿把 `i32×4` 算成 20 B，实现照抄后 `payload[22..25]` 是未初始化栈字节，且 C6 的 `saved` 会读到脏值。已按从机解码器改齐 |

### 11.3 1 kHz 环内的新次序（`MOTOR_ALGO_task`）

`ENCODER_task` → `ADC_task` → **`applyRecord`** → 读目标 → **急停（置位即 `calibAbort` + jog 归零 + `MOTOR_stopAll`）** → **`calib` 步进 → `calibStart`** → **`jogStep`** → `controlStep`。
优先级即 §9.3 要求的 **急停 > 标定 > jog > servo**：急停在最前短路；标定活跃时 `calibStep` 自己控 PWM 且 `jogStep` 让位；无标定时才轮到 jog；都没有才走闭环 `controlStep`。

### 11.4 编码期发现（必须留痕）

- **C1 · DF0 大小两说**：iLLD 逻辑扇区表 48 个（至 `0xAF05FFFF`）vs 21 §4.1 "DFlash0 128 KB"。**未裁决**——落点选在扇区 15 使两种读法都成立。裁决前 21 §4.3 的配置页不得进入扇区 15，OTA（§9）不得使用 DF0 高位扇区。→ §12 Q1
- **C2 · `IfxFlash` 模块此前在 `.cproject` 的 excluding 名单里**（`Libraries/iLLD/TC27D/Tricore/Flash` 与 `Flash/Std`，`IfxFlash.c` 不参与编译）。本实现只用 `IFX_INLINE` 原语（`clearStatus/enterPageMode/loadPage/writePage/eraseSector`），不需要该 `.c` 里的符号；**但已顺手把 Debug 配置的这两项 excluding 解除并补上 include 路径**（更稳，且与 IDE 的"include 路径须存在"检查一致），**IDE 构建仍须实证一次**。→ §12 Q2
- **C3 · 0x22 长度算错**：见 D5。对端 `esp32c6_car` 已在 `doc/17-calib-ui.md` §4.1 与 `bridge.c:bridge_emit_cal` 用 23 B（`saved` 在 `[22]`，`n>=23` 才读），主机单测现按"字段偏移 + 长度下限"两侧对齐。
- **C4 · `0x71~0x74` 在 esp32c6_car 的常量表里另有名字**（`proto_frames.h`：0x71 `DPT_LED` / 0x72 `DPT_MOTOR_RUN` / 0x73 `DPT_ENC_READ` / 0x74 `DPT_CAL`）。同一个字节两仓库两名，与 §3 记录的 0x70 双语义同源；c6 侧 `c6_link/link.c:285` 按 `0x70..0x79` **整段**路由到 `CID_DPT`，故通道无冲突，但**页面与 C6 代码不得真的发送 LED/MOTOR_RUN 语义**——只能按本文 §9.1 的四个新语义发。
- **C5 · 写入期间的中断掩蔽**：`CALIB_tick` 的保存序列整段 `__disable()`（最长 ≈ 擦除 + 3 页编程 + 回读，预算内几十 ms）。CPU0 的 FreeRTOS tick 与 UART RX 在此期间挂起；三核取指都在 PFlash0（`Lcf_*.lsl` 未把任何段放进 DF0），故 CPU1/CPU2 不停顿。喂狗在轮询循环内持续服务，超时上限仍按失败处理。
- **C6 · 保存失败重试上限**（V1.5 起由"无上限"改为 **3 次**，`CALIB_FLASH_RETRIES`）：`g_pending` 只在**写成功**后清零，V1.2~V1.4 实现为不设次数上限地每 `CALIB_WRITE_IDLE_MS`(500 ms) 重试。台架证实该取舍有害：重试每次都是一轮新的擦+编循环，且旧实现擦除前**不** clearStatus、也**不**查 FSR——错误锁存会让 FMU 拒擦而编程照发，对未擦除扇区二次编程产生 ECC 损坏页（状态行根因）。现在每步查错 + 擦除确认后才编程 + 3 次上限，耗尽打 `CALSAVE failed (flash)`；RAM 态记录继续生效（本次上电），EVT 0x22 的 `saved=2` 照旧第一时间发出。
- **C7 · 只有 `TriCore Debug (TASKING)` 一个配置能编这批代码**：读 `.cproject` 得到 —— Release (TASKING) 的 include 路径列表里**没有** `com`、`rt`、`mw/sf`、`mw/calib`（只有 `app/bsp/mw/mw/xcore/mw/proto`），并且**仍排除** `Flash`/`Flash/Std`；两个 GCC 配置同样没有这批路径。也就是说 Release 从 SF 链路落地那次起就与源码脱节（它定义了 `USE_SPI_LINK` 却找不到 `com/link.h`），本轮只是又多欠一项。**本轮不动 Release/GCC 配置**（构建配置改动风险大、且无人验证过 Release 产物）；要用 Release 出镜像前先补 include 路径 + 解除 Flash 排除。→ §12 Q4

### 11.5 验证状态（红线：不许把"编不出来"说成"已验证"）

- **已过**：`test/host/test_sf.c` **2948** 断言 / 0 失败（含新增 3 个用例组；编译单元需 `mw/sf/sf_frame.c` + `mw/calib/calib_record.c`，CI 已同步）；`test/host/test_sf_telemetry.c` **154** 断言 / 0 失败（`-DC6_CROSS_CHECK` 编进从机 `proto_frames.c` 的跨侧模式；不带该宏为 116 项）——38 B 遥测布局未动，此处只作回归。
- **未做（本机不可能做）**：TASKING IDE 构建与烧录——**本轮未过编译器**，C1/C2 两条即以构建为首要验证手段。
- **必须台架验证（无主机单测覆盖）**：xcore 新块与 EVT 队列的跨核真实行为、DFlash 擦/写/回读（含掉电重启后 `REC_GET` 读回）、jog 的 300 ms 超时与故障门禁、保存瞬间链路不失联（§10 的 10~14 全部待测）。

## 12 · 待用户/台架裁决

| 编号 | 事项 | 现状 |
|---|---|---|
| Q1 | DF0 真实扇区数（48 表 vs 21 §4.1 的 128 KB） | 落点扇区 15 对两种读法都成立；查器件手册或台架在 `0xAF01E000` 做一次擦写读即可销项 |
| Q2 | `.cproject` 的 Flash 排除是否该解除 | 已按"更稳"的选择做了：Debug (TASKING) 解除 `Flash`/`Flash/Std` 排除并补 include（`IfxFlash.c` 参与编译）。IDE 构建若因此报重复符号，可退回"只用 `IFX_INLINE`、保持排除"这一支——两条路都能通，**取哪条由构建结果定** |
| Q3 | §10 验收 10~17（断电保持 / CRC 回退 / 参数生效 / 写入停顿 / jog 三重安全 / **闭环使能门开关两向**） | 全部待台架 |
| Q4 | Release (TASKING) 与 GCC 配置的 include 路径 / Flash 排除是否补齐 | 本轮只改 Debug（C7）。要出 Release 或 GCC 镜像前先补，属构建配置改动，等发话 |

---

## 13 · V1.3 闭环使能门（方向未标定，禁止闭环）

### 13.1 动机

V1.2 及以前的闭环安全网只有一张：**编码器不 alive → 回退开环**（SERVO `measValid=FALSE`：duty=目标、清积分）。它假设"编码器没接线 = 唯一危险态"。台架随后证明了这个假设有洞：

> 8 根编码器线**已经接好**（alive=1）、但方向从未标定（`g_encInvert` 全 +1，镜像减速箱让其中若干轮符号必错）→ SERVO 照常闭环 → 错误符号把负反馈变成正反馈。UI 回原点（目标 0/0）时右 duty 仍被钳在 -334（§1 证据行 `SRV= 0 0 37 0 43 -334` 的同款现象）——**目标为零也停不下来**，因为 Kp/积分在追一个符号反了的假反馈。

23 §8.4 早已写下"校验失败则整体回落默认（src=0、crcOk=0）并**按开环等价行为运行**"——本节前这句只是规程，固件并不强制执行，src=0 时闭环照跑。V1.3 把这句话**变成代码**。

### 13.2 设计

- **门变量**：`rt/motor_algo.c` 静态 `g_closedLoopOk`，初值 FALSE；翻转条件 = `RecordLive.rec.src != CALIB_SRC_DEFAULT`（即记录来自 0x70 持久化 src=1 或 0x73 在线设置 src=2）；
- **随 version 边沿翻转**：CPU1 在 1 kHz 环的 `MOTOR_ALGO_applyRecord()` 里比对 `RecordLive.version`（§11.2 D3 的机制原样复用），边沿上同步三件事：应用 invert/fullScale/wheelDia（原有）→ **翻门**（新增）→ 打一次性日志（新增）；
- **门的使用**：`MOTOR_ALGO_controlStep()` 把 `measOk = (enc.alive && g_closedLoopOk)` 作为 `SERVO_update` 的 `measValid`。门关闭时走 SERVO 现成的开环回退——**duty=目标、积分清掉**，与"编码器未接线"时完全相同，SERVO 零改动；
- **为什么不用单独再加"开门延时"**：0x70 DONE 时 CPU0 `CALIB_init` 路径的持久化在**取走结果那一刻**就把 `src=DFLASH` 写进 RecordLive（DFlash 物理擦写是延迟队列，§8.3），version 翻转与符号应用在同一 1 kHz 圈——开门与换符号原子发生，中间不存在"新符号+旧门"或"新门+旧符号"的缝隙窗口；
- **有意不做**：门**不**看 `delta` 数值、不估计方向可信度——src=1/2 只代表"标定事实存在"，不担保"标定仍然正确"（换电机/换线后忘重标 = 操作遗漏，门挡不了也不该挡，§5.3 已写明）。

### 13.3 行为矩阵

| 记录 src | 门 | `enc.alive=1` 时 SERVO 行为 | 串口 |
|---|---|---|---|
| 0（默认/回落/0x74 后） | 关 | **开环等价**：duty=target，积分清 | 一次性 `SERVO open-loop (record src=0; calibrate via 0x70)` |
| 0（默认/回落/0x74 后） | 关 | `enc.alive=0` 照旧开环（原行为） | 同上（只打一次） |
| 1 / 2 | 开 | **闭环**：Kp/Ki/FF 全参与 | `SERVO closed-loop enabled` |
| 1 / 2 → 被 0x74 清回 0 | 关 | 回开环等价 | open-loop 行再打印一次 |

日志每次**翻变**各打一行（开→关、关→开），同状态不重复刷。

### 13.4 代码锚点

| 位置 | 内容 |
|---|---|
| `rt/motor_algo.c` `g_closedLoopOk` 定义处 | 门变量 + 动机注释（为什么 src=0 必须禁闭环） |
| `MOTOR_ALGO_applyRecord()` | version 边沿：应用记录 → 翻门 → 一次性日志 |
| `MOTOR_ALGO_controlStep()` | `measOk = (enc.alive && g_closedLoopOk)` → `SERVO_update` |
| `MOTOR_ALGO_init()` | `g_closedLoopOk = FALSE`（上电默认关门，等 CPU0 广播记录） |

### 13.5 验证

- **主机单测**：本改动**不动任何 SF/遥测字节**，`test_sf.c`/`test_sf_telemetry.c` 无新增用例、照常回归即可；门的翻转逻辑在 CPU1 1 kHz 环内（依赖 xcore 真实跨核行为），**无主机单测覆盖，需台架验证**——台架动作即 §10 的第 16/17 条。
- **同批文档**：23 §8.4（"开环等价"从规程变固件执行）、21 §5.2（§13 门注记）、33 §2/§5.2/§6（代码地图与安全机制补门）、00-index §2（版本同步）。
- **明确的非目标**（本次不做，理由留痕）：`delta==0`（弱 plant 误判死通道）**不做自动重试**——重试意味着发更大的脉冲，而车可能已落地，强脉冲 + 错误方向 = 新风险；§5.1 的手动升 `MOTOR_CALIB_DUTY/PULSE_MS` 流程保留。
