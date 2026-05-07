# UAV FSM 代码清理与优化计划

## Context

用户提供了无人机状态机的最终版状态定义和交互协议，要求：
1. 复述并理解完整的状态机流程
2. 检查现有代码是否与规范一致
3. 删除无用代码和文件
4. 修正误导性日志、统一缩进

---
状态定义：我的状态机（无人机子状态机，位于Status.msg，循环发布，频率1hz）：
uint8 uav_control_state 
uint8 SELF_CHECK = 0
uint8 UAV_START= 1
uint8 TRANSIT_TO_AREA= 2
uint8 UAV_ARRIVED_AERA = 3
uint8 SEARCH_ADJUST_AUTO= 4
uint8 SEARCH_ADJUST_MANUAL= 5
uint8 APPROACH_PLANT= 6
uint8 UAV_PRE_HOLD = 7
uint8 SAMP_ADJUST_AUTO= 8
uint8 SAMP_ADJUST_MANUAL= 9
uint8 UAV_HOLD = 10
uint8 RETREAT = 11
uint8 UAV_BACK_HOME=12
uint8 UAV_TASK_TERM= 13

外部控制主状态机：（订阅的外部topic，如果订阅到，会控制子状态机状态切换；先用按键控制发布需要的topic）
uint8 main_task_state 
uint8 PRE_CHECK = 1 
uint8 WAIT_TASK_ENABLE_AUTH = 2
uint8 NAV_TO_TASK_DOM  = 3
uint8 ARRIVE_TASK_DOM    = 4  
uint8 UAV_SEARCH_TARGS  = 6 
uint8 TARG_GOT= 7 
uint8 TARG_READY = 8 
uint8 UAV_POSE_ADAP = 9 
uint8 ARM_CONFIG_PREP = 10
uint8 SAMPL_OPERA = 11 
uint8 UAV_PRE_BACK_HOME=12
uint8 BACK_HOME = 13

无人机子状态机返回给住状态机标志位（发布topic 位于StatusExecution.msg）
bool uavCheckSucceed = 0/1
bool uavTakeoffSucceed = 0/1
bool uavArrivedTaskAera = 0/1
bool uavSearchSucceed = 0/1
bool uavApproachSucceed = 0/1
bool uavAdjustSucceed = 0/1
bool uavReadyForBack = 0/1

## 状态机流程复述

### 无人机状态（uav_control_state，0-13）

| ID | 状态名 | 进入条件 | 退出方式 |
|----|--------|----------|----------|
| 0 | SELF_CHECK | PRE_CHECK 指令 | isSelfCheckOK() → publishStatusExecution → 等 WAIT_TASK_ENABLE_AUTH |
| 1 | UAV_START | isSelfCheckOK() + WAIT_TASK_ENABLE_AUTH 授权 | 到达起飞航点 → publishStatusExecution → 等 NAV_TO_TASK_DOM |
| 2 | TRANSIT_TO_AREA | 到达起飞航点 + NAV_TO_TASK_DOM 指令 | 飞完所有 transit_waypoints → publishStatusExecution → **自动** 进入 UAV_ARRIVED_AERA |
| 3 | UAV_ARRIVED_AERA | 自动 | 主状态机ARRIVE_TASK_DOM ，如果收到 **UAV_SEARCH_TARGS** → 请求式(SwitchStatus客户端)，选 SEARCH_ADJUST_AUTO/MANUAL；如果收到 **TARG_GOT** → 控制式(AirdropStatus客户端)请求→APPROACH_PLANT|
| 4 | SEARCH_ADJUST_AUTO | SwitchStatus 返回 | 执行搜索航点序列；搜索航点完成后，publishStatusExecution，如果StatusExecution是成功，收到 TARG_GOT → AirdropStatus → APPROACH_PLANT；如果StatusExecution是失败，AirdropStatus → UAV_TASK_TERM |
| 5 | SEARCH_ADJUST_MANUAL | SwitchStatus 返回 | 悬停等待；同上 |
| 6 | APPROACH_PLANT | TARG_GOT + AirdropStatus approved | 按测距/航点靠近；到达距离阈值 → **自动** 进入 UAV_PRE_HOLD，publishStatusExecution |
| 7 | UAV_PRE_HOLD | 自动 | 悬停等待；TARG_READY → 收到 **UAV_POSE_ADAP** → 请求式(SwitchStatus)，选 SAMP_ADJUST_AUTO/MANUAL |
| 8 | SAMP_ADJUST_AUTO | SwitchStatus 返回 | 执行微调航点；微调航点完成后，publishStatusExecution，如果StatusExecution是成功，**自动** 进入 UAV_HOLD；如果StatusExecution是失败，AirdropStatus → UAV_TASK_TERM|
| 9 | SAMP_ADJUST_MANUAL | SwitchStatus 返回 | 悬停等待，同上 |
| 10 | UAV_HOLD | 微调成功后自动或者ARM_CONFIG_PREP | 等 ARM_CONFIG_PREP → SAMPL_OPERA → 等 UAV_PRE_BACK_HOME |
| 11 | RETREAT | UAV_PRE_BACK_HOME 指令 | 后退航点执行完 → publishStatusExecution → 等 BACK_HOME |
| 12 | UAV_BACK_HOME | BACK_HOME 指令 | 飞回 home_waypoint|
| 13 | UAV_TASK_TERM | 搜索失败+AirdropStatus approved、微调失败+AirdropStatus approved | 悬停，等 UAV_PRE_BACK_HOME 或 BACK_HOME |

### 两种地面站交互模式
写一个服务节点，专门用来处理，做仿真用
**请求式（UAV是客户端）→ SwitchStatus.srv**
- 触发时机：UAV_SEARCH_TARGS、UAV_POSE_ADAP
- UAV 发：current_status + switchable_statuses[] + urgency
- 地面站 服务端返回：current_status（用于时效校验）+ target_status（选择的状态）
- 若 target_status == current_status → 拒绝

**控制式（UAV是客户端）→ AirdropStatus.srv（作为客户端发起）**
- 触发时机：TARG_GOT
- UAV 发：current_status + target_status + urgency
- 服务端返回：bool success → true 则切换


### 状态发布
- `status_interfaces_pkg/msg/Status`（status字段=状态ID）：每个控制循环周期发布（默认50ms）
- `status_interfaces_pkg/msg/StatusExecution`：状态阶段完成/失败时发布一次事件

---

## 代码检查结果

### 已正确实现的部分 ✓
- 14个状态完整实现，ID与规范完全一致
- SwitchStatus 作为客户端（UAV_SEARCH_TARGS → requestSwitchChoice）
- AirdropStatus 作为客户端（TARG_GOT → requestAirdropTransition）
- AirdropStatus 作为服务端（handleAirdropStatusService）
- Status.msg 每tick发布 + StatusExecution.msg 按需发布
- main_task_state 订阅并映射为指令（commandFromMainTaskStatus）
- 键盘临时替代主状态机输入（keyboard_control_node.cpp）

### 发现的问题

#### 死代码（Dead Code）
1. **`vehicle_command_pub_`**（line 285）+ 初始化（lines 172-175）+ `publish_vehicle_command()`（lines 472-491）+ `#include <px4_msgs/msg/vehicle_command.hpp>`（line 6）：该发布器和函数从未被调用
2. **`isSearchAdjustState()`** 和 **`isSampleAdjustState()`**：声明（lines 463-464）和定义（lines 2091-2103），从未被调用
3. **`msg/uavFeedbackFlag.msg`**：未注册在CMakeLists中，从未被包含或发布，纯死文件
4. **`srv/ServoControl.srv`**：同上，从未使用

#### 多余依赖
5. **`geometry_msgs`** 和 **`tf2`**：CMakeLists.txt（lines 15,17,38,40）和 package.xml（lines 14,16）中声明，但没有任何源文件 `#include` 这两个包

#### 误导性日志（4处）
6. **line 770**（SEARCH_ADJUST_AUTO/targ_got_confirm_pending_）：`"waiting CONFIRM"` → 实际是等待AirdropStatus服务响应，非用户按键
7. **line 782**（SEARCH_ADJUST_AUTO完成后）：`"waiting TARG_GOT or NO"` → `NO` 不是 main_task_state 指令，应为 `"waiting TARG_GOT or TASK_TERM"`
8. **line 798**（SEARCH_ADJUST_MANUAL/targ_got_confirm_pending_）：同 line 770
9. **line 808**（SEARCH_ADJUST_MANUAL空闲等待）：同 line 782

#### 缩进不一致
10. 文件全局存在4空格和8空格混用，主要集中在 class 内 enum 定义、成员变量声明、switch-case 分支

---

## 实现方案

### Step 1：删除死文件
- 删除 `msg/uavFeedbackFlag.msg`
- 删除 `srv/ServoControl.srv`
- `msg/uavControlState.msg` **保留**（文档参考价值，说明各状态ID），顶部加一行注释说明 C++ 使用内部 enum

### Step 2：更新 CMakeLists.txt
- 移除 `find_package(geometry_msgs REQUIRED)`（line 15）
- 移除 `find_package(tf2 REQUIRED)`（line 17）
- 从 `ament_target_dependencies` 中移除 `geometry_msgs`（line 38）和 `tf2`（line 40）

### Step 3：更新 package.xml
- 移除 `<depend>geometry_msgs</depend>`（line 14）
- 移除 `<depend>tf2</depend>`（line 16）

### Step 4：清理 uav_offboard_fsm.cpp

**4a — 移除死代码（按从后向前顺序，避免行号偏移）：**
1. 移除 `isSearchAdjustState`/`isSampleAdjustState` 定义（lines 2091-2103）
2. 移除 `publish_vehicle_command()` 方法（lines 472-491）
3. 移除 `isSearchAdjustState`/`isSampleAdjustState` 声明（lines 463-464）
4. 移除 `vehicle_command_pub_` 成员声明（line 285）
5. 移除 `vehicle_command_pub_` 初始化块（lines 172-175，含注释）
6. 移除 `#include <px4_msgs/msg/vehicle_command.hpp>`（line 6）

**4b — 修正4处日志消息：**
- line 770: `"TARG_GOT received; waiting CONFIRM"` → `"TARG_GOT received; waiting AirdropStatus service response for APPROACH_PLANT"`
- line 782: `"uavSearchSucceed=0 waiting TARG_GOT or NO"` → `"uavSearchSucceed=0 waiting TARG_GOT or TASK_TERM"`
- line 798: `"TARG_GOT received; waiting CONFIRM"` → `"TARG_GOT received; waiting AirdropStatus service response for APPROACH_PLANT"`
- line 808: `"waiting for TARG_GOT or NO"` → `"waiting for TARG_GOT or TASK_TERM"`

**4c — 统一缩进：**
- class 内所有成员统一4空格缩进
- 两个 enum class（ControlState, CommandType）的值：8空格 → 4空格（class 体内第一级缩进）
- controlLoopOnTimer() 和 onStateEntry() 的 switch-case 分支：统一8空格（function内一级）
- Constructor 内第一个 RCLCPP_INFO 块（lines 211-215）：修正额外多余缩进

---

## 不变更的内容

- `CommandType::NO` + `CommandType::CONFIRM` 键盘终止路径（有效的人工紧急中止备用路径）
- `targ_got_confirm_pending_` 命名（仅修正日志，不做大范围重命名重构）
- `ARRIVE_TASK_DOM` 冗余路径（已有保险带，保留）
- `self_check_requested_` 未在 resetMissionProgress 中清空（设计正确：PRE_CHECK → reset → 立即设为true）

---

## 验证方式

1. **构建测试**：`colcon build --packages-select uav_offboard_fsm` 无新警告
2. **运行测试**：节点启动，`/uav_offboard_fsm/status` 以~20Hz 发布 status=0
3. **状态流测试（键盘）**：S → T → (auto)→ U → A/Z → P(TARG_GOT) → (auto进UAV_PRE_HOLD) → V(UAV_POSE_ADAP) → ...
4. **紧急终止测试**：搜索/采样调整状态中发 X + Y → 确认进入 UAV_TASK_TERM
5. **日志验证**：TARG_GOT后日志不再出现"waiting CONFIRM"，而是"waiting AirdropStatus service response"

---

## 关键文件路径

- `src/uav_offboard_fsm/src/uav_offboard_fsm.cpp` — 主要修改（2257行）
- `src/uav_offboard_fsm/CMakeLists.txt` — 移除2个依赖
- `src/uav_offboard_fsm/package.xml` — 移除2个依赖
- `src/uav_offboard_fsm/msg/uavFeedbackFlag.msg` — 删除
- `src/uav_offboard_fsm/srv/ServoControl.srv` — 删除
- `src/uav_offboard_fsm/msg/uavControlState.msg` — 加注释保留
