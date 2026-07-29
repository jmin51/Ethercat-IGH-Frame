# EtherCAT 控制器架构文档

## 项目结构

```
ethercat_controller/
├── src/
│   ├── main.cpp                          # 主入口
│   ├── ethercat_node.cpp                 # EtherCAT 主节点
│   ├── servo_axis_base.cpp               # 伺服轴基类
│   ├── leisai_servo_axis.cpp             # 雷赛伺服轴实现
│   ├── huichuan_servo_axis.cpp           # 汇川伺服轴实现
│   ├── servo_axis_factory.cpp            # 伺服轴工厂
│   ├── business/
│   │   └── LayerCommandProcessor.cpp     # 层指令处理器
│   ├── io_modules/
│   │   ├── io_interface.cpp              # IO 模块接口
│   │   ├── io_interface.hpp              # IO 模块头文件
│   │   ├── lights_controller.cpp         # 灯光控制器（按钮灯+三色灯）
│   │   ├── lights_controller.hpp         # 灯光控制器头文件
│   │   ├── smema_handler.cpp             # SMEMA协议处理器（下游设备）
│   │   └── smema_handler.hpp             # SMEMA协议头文件
│   └── fault_manager/
│       ├── fault_management_system.cpp   # 故障管理系统（独立实现）
│       ├── fault_management_system.hpp
│       └── fault_codes.hpp               # 故障码表定义
├── include/
│   ├── ethercat_node.hpp
│   ├── servo_axis_base.hpp
│   ├── servo_axis_factory.hpp
│   ├── io_interface.hpp
│   └── LayerCommandProcessor.hpp
├── CMakeLists.txt
└── package.xml
```

## 设备物理结构

### 设计哲学
- **基准层暂存**：缓存架第1层（buffer_out位）作为暂存位，板子在此等位，空出后可提前要板
- **路径不重叠**：新板路径（上游→基准层）与出库板路径（缓存架→接驳台下游）物理隔离，可并行作业
- **axis1 共享**：接驳台输送轴同时服务于产品到位检测（进料）与入库传输，通过 conveyor_occupied 互斥

### 物理布局

```
上游设备
  │
  ▼
feed_detect (DI8) ──────────────── 进料检测
  │
  ▼
buffer_in (DI11) ──────────────── 缓存架入料口
  │
  ▼
buffer_out (DI12) ─────────────── 基准层（暂存位，缓存架第1层）
  │ ════════════════════════════════════════════
  │   以上: 新板进料路径 (产品到位检测 axis1 驱动)
  ══════════════════════════════════════════════
  │   以下: 接驳台区 (axis1 下游段, 升降机 axis5 可移动)
  │ ════════════════════════════════════════════
  │                              ┌──────────────┐
  ▼                              │  出库板路径   │
conveyor_in (M541/DI29) ──────── │  (从缓存架    │
  │   接驳台入料检测(缝隙)       │   经axis2     │
  ▼                              │   进入此段)   │
[接驳台 axis1 下游段 + DO813]    │              │
  │                              │  DO813 控制   │
  ▼                              │  板子往下游   │
conveyor_out (DI14) ──────────── │  释放         │
conveyor_exit_gap (M540/DI28)    └──────────────┘
  │   接驳台出料检测(缝隙)
  ▼
下游设备
  │
  ▲
  │ ════════════════════════════════════════════
  │   缓存架内部: axis2 驱动, axis5 升降选层
  │ ════════════════════════════════════════════
  │
[缓存架各层] ← axis2_1 + axis2_2 内部输送
  │
  ▲
  │
buffer_sensor_2 (DI10) ──────── 缓存架传感器2（板子进入缓存架检测）
```

### DI 信号定义

| 信号 | DI编号 | Modbus地址 | 信号名 | 位置 | 用途 |
|------|--------|-----------|--------|------|------|
| feed_detect | DI8 | M528 | feed_product_detect | 进料口 | 上游来板检测，触发产品到位状态机 |
| buffer_in | DI11 | M531 | buffer_in_position | 缓存架入料口 | 新板进入基准层区域 |
| buffer_out | DI12 | M532 | buffer_out_position | 基准层 | 板子到达暂存位，触发0x0109发布 |
| buffer_sensor_2 | DI10 | M530 | buffer_sensor_2 | 缓存架内部 | 板子进入缓存架目标层检测 |
| conveyor_in | DI13 | M533 | conveyor_in_position | 接驳台入料 | 入库板子进入接驳台 |
| conveyor_out | DI14 | M534 | conveyor_out_position | 接驳台出料 | 入库板子到达接驳台末端 |
| conveyor_entry_gap | DI29 | M541 | conveyor_entry_gap_detect | 接驳台入料缝隙 | 出库板子进入接驳台检测(M541) |
| conveyor_exit_gap | DI28 | M540 | conveyor_exit_gap_detect | 接驳台出料缝隙 | 出库板子离开接驳台检测(M540) |
| smema_uba | DI23 | M535 | smema_uba | SMEMA上游 | 上游有板待发 |
| smema_dbr | DI24 | M536 | smema_dbr | SMEMA下游 | 下游要板 |

### DO 信号定义

| 信号 | DO编号 | Modbus地址 | 用途 |
|------|--------|-----------|------|
| DO811 | DO811 | M811 | 齿轮对接气缸伸出（入库/出库时连接 axis1 与 axis2） |
| DO812 | DO812 | M812 | 内部输送机构控制（入库 POST_LIFT 时激活） |
| DO813 | DO813 | M813 | 接驳台释放机构（入库/出库 CONVEYOR_MOVING 时激活，控制板子往下游运动） |
| MR | DO14 | M814 | SMEMA 本机要板信号（上游握手） |
| BA | DO15 | M815 | SMEMA 本机有板信号（下游握手） |

### 轴定义

| 轴名 | 类型 | 物理位置 | 用途 |
|------|------|---------|------|
| axis1_1 | 输送轴 | 接驳台上游段 | 进料/入库传输（reverse: 板子往下游方向） |
| axis1_2 | 输送轴 | 接驳台下游段 | 进料/入库传输（forward: 板子往下游方向） |
| axis2_1 | 输送轴 | 缓存架内部 | 入库推板（forward）/ 出库取板（reverse） |
| axis2_2 | 输送轴 | 缓存架内部 | 入库推板（reverse）/ 出库取板（forward） |
| axis3 | 调整轴 | 板宽调整 | 板宽调节 |
| axis4 | 调整轴 | 板宽调整 | 板宽调节 |
| axis5 | 升降轴 | 接驳台升降 | 层选择（层号→高度映射） |

### PCB 流动路径

#### 产品到位检测（新板从上游到基准层）
```
上游 → feed_detect → buffer_in → buffer_out（基准层暂存）
      │ axis1_1(reverse) + axis1_2(forward) 驱动 │
      │ 产品到位状态机: IDLE→CONVEYOR_RUNNING→WAITING_BUFFER_OUT→COMPLETED │
      │ 到达 buffer_out → 发布 0x0109 → 停 axis1 │
```

#### 入库（基准层 → 缓存架目标层）
```
buffer_out（基准层）
  → axis1 传输 → conveyor_in → conveyor_out（接驳台末端）
  → axis5 升降到目标层
  → DO812 + axis2 推板 → buffer_sensor_2（进入缓存架）
  → axis5 归位第1层
```

#### 出库（缓存架源层 → 下游）
```
缓存架源层
  → axis5 升降到源层
  → axis2 取板 → conveyor_in_gap(M541)（进入接驳台）
  → axis5 归位第1层
  → DO813 释放 → conveyor_exit_gap(M540) → 下游
  ※ 出库板路径: conveyor_in → conveyor_out, 不经过 buffer_out（基准层）
```

#### 放行（基准层 → 下游，不进缓存架）
```
buffer_out（基准层）
  → axis1 传输 → conveyor_in_gap → conveyor_exit_gap(M540) → 下游
  → 0x0109 已发布, 放行流程直接启动
```

### 要板信号（MR）设计原则

#### 核心设计：以 0x0109 的 IDLE 状态为唯一要板门控

```
MR=ON 条件（全部满足）:
  1. product_arrival_cycle_active = True（作业进行中）
  2. product_arrival_state == "IDLE"（0x0109状态机空闲）
  3. feed_detect = False（无板在进料位）
  4. buffer_in = False（无板在入料口）
  5. buffer_out = False（基准层空）
  6. warehouse_state != CONVEYOR_MOVING（入库传输未占用axis1）
  7. release_state 不在 (CONVEYOR_RUNNING, WAIT_FOR_CONVEYOR_OUT)（放行未占用axis1）

MR=OFF 条件（任一满足）:
  - product_arrival_cycle_active = False（未开始/报错/结束作业）
  - product_arrival_state != "IDLE"（板子在路上或基准层）
  - 任何物理信号有板（feed_detect/buffer_in/buffer_out）
  - 入库/放行占用 axis1
```

#### 出库期间允许要板

出库板路径（`conveyor_in → conveyor_out`）不经过基准层（`buffer_out`），与新板进料路径（`feed_detect → buffer_out`）物理不重叠。出库期间 MR=ON 安全，可并行作业。

#### COMPLETED → IDLE 的安全门控

```
COMPLETED → IDLE 条件:
  not feed_detect and not buffer_out and board_dispatched

board_dispatched 置位时机:
  - 入库 COMPLETED（升降机归位第1层）→ axis1 上板子已送走
  - 放行 CONVEYOR_RUNNING（板子完全进入输送带）→ 板子已在往下游走

→ COMPLETED → IDLE 时, 接驳台必定空闲, MR=ON 安全
```

---

## 故障管理系统架构

### 设计哲学
- **三级故障分类**：系统错误、轴错误、IO错误、通信错误、业务逻辑错误、模式错误、警告
- **自动日志捕获**：通过 ROS2 日志回调自动捕获 RCLCPP_ERROR 和 RCLCPP_WARN
- **实时故障发布**：通过 `/fault_code` 话题实时发布故障状态（无故障时不发布）
- **模式冲突处理**：手动模式下收到自动指令时，上报特定故障码

### 核心组件

#### 1. 故障码表 (fault_codes.hpp)
- **16位故障码结构**：高4位类别，次4位子类别，低8位具体代码
- **七大类故障**：
  - 0x1xxx: 系统错误（初始化、配置、资源、超时）
  - 0x2xxx: 轴错误（回零、运动、限位、过载、通信）
  - 0x3xxx: IO错误（数字输入/输出、模拟输入/输出）
  - 0x4xxx: 通信错误（EtherCAT、Modbus、ROS2、串口）
  - 0x5xxx: 业务逻辑错误（层指令、序列、状态）
  - 0x6xxx: 模式错误（模式冲突、转换失败）
  - 0x7xxx: 警告（性能、温度、电压）
- **遗留兼容**：保留 0x8xxx、0x9xxx、0x91xx 范围用于向后兼容

#### 2. 故障管理系统 (FaultManagementSystem)
- **独立实现**：完整的故障管理功能，不依赖 SimpleFaultManager
- **日志回调**：注册 ROS2 根记录器回调，自动转换 ERROR/WARN 日志为故障码
- **模式冲突处理**：`handle_auto_command_in_manual_mode()` 方法处理手动模式下的自动指令
- **可配置映射**：支持日志消息模式到故障码的映射规则
- **JSON 状态输出**：`get_fault_status_json()` 提供结构化故障状态
- **循环避免**：使用专用日志器名称避免日志回调循环

#### 3. 集成点 (ethercat_node.cpp)
- **初始化**：在节点初始化时创建 FaultManagementSystem 实例
- **自动捕获**：启用日志回调，自动监控所有错误和警告
- **模式检查**：在 `handle_control_command()` 中检查手动模式下的自动指令
- **故障发布**：通过 `/fault_code` 话题发布故障状态字符串

### 关键工作流程

#### 自动故障检测
```
RCLCPP_ERROR/WARN 日志 → ROS2 日志回调 → 故障管理系统 → 映射为故障码 → 发布到 /fault_code
```

#### 手动模式自动指令处理
```
收到 start_auto 命令 → 检查所有轴操作模式 → 发现轴处于手动模式 → 调用 handle_auto_command_in_manual_mode()
→ 添加故障码 FAULT_MODE_MANUAL_RECEIVED_AUTO (0x6201) → 发布故障状态 → 记录警告日志
```

### 故障码示例
- `0x1201`: 系统初始化失败
- `0x2201`: 轴回零失败
- `0x6201`: 手动模式下收到自动指令
- `0x7101`: 通用警告
- `0x8001`: 遗留轴错误

### 使用示例

```cpp
// 手动添加故障
fault_manager_->add_fault("axis1", fault_codes::FAULT_AXIS_HOMING_FAILED, "回零超时");

// 处理模式冲突
uint16_t fault_code = fault_manager_->handle_auto_command_in_manual_mode("axis2");

// 获取故障状态
std::string json_status = fault_manager_->get_fault_status_json();
```

### 配置与扩展

1. **添加新故障码**：在 `fault_codes.hpp` 中定义新常量
2. **扩展映射规则**：调用 `add_fault_mapping()` 添加日志模式到故障码的映射
3. **自定义发布格式**：重写 `publish_fault_status()` 方法
4. **集成其他日志源**：通过 `add_fault()` 方法直接添加故障

### 向后兼容
- 故障管理系统提供与先前版本兼容的 API，现有集成代码无需修改
- 遗留故障码范围 (0x8xxx, 0x9xxx, 0x91xx) 继续支持
- `is_error_code()` 函数同时支持新旧故障码分类

## 灯光控制系统架构

### 设计哲学
- **职责分离**：灯光控制独立于主流程，避免main.cpp臃肿
- **状态机驱动**：三色灯通过状态机管理（OFF → YELLOW_BLINK → GREEN_BLINK）
- **边沿触发**：按钮灯通过边沿检测触发，避免抖动
- **变化检测**：DO写入仅在状态变化时执行，减少Modbus通信

### 核心组件

#### 1. 灯光控制器 (lights_controller)
- **独立模块**：封装按钮灯和三色灯所有逻辑
- **线程安全**：通过互斥锁保护Modbus访问（与io_interface配合）
- **自动闪烁**：500ms周期自动切换黄灯/绿灯闪烁状态

#### 2. 状态定义
```cpp
enum TricolorLightState {
    LIGHT_OFF,          // 全部熄灭
    LIGHT_YELLOW_BLINK, // 黄灯闪烁（复位中）
    LIGHT_GREEN_BLINK,  // 绿灯闪烁（就绪）
    LIGHT_GREEN_ON      // 绿灯常亮（运行中）
};
```

#### 3. 控制流程
| 触发条件 | 按钮灯行为 | 三色灯行为 | 蜂鸣器 |
|----------|-----------|-----------|--------|
| 启动按钮 | 启动灯亮 | 绿灯常亮 | 停 |
| 复位按钮（按住3秒确认） | 复位灯亮 | 黄灯闪烁 | 响 |
| 暂停按钮 | 暂停灯亮 | 全部熄灭 | 停 |
| AL states 0x08 + 全部自动 | 复位灯灭 | 黄灯停闪，绿灯闪烁 | 停 |

### 关键接口

```cpp
// 初始化
void init_lights_controller();

// 主循环调用（每100ms）
void update_button_lights(bool start_btn, bool reset_btn, bool pause_btn);
void update_tricolor_lights();

// 系统就绪通知（由ethercat_node调用）
void notify_system_ready();
```

### 集成点

1. **main.cpp**：初始化后调用 `init_lights_controller()`，主循环中调用更新函数
2. **ethercat_node.cpp**：检测到 AL states 0x08 且全部轴自动时调用 `notify_system_ready()`
3. **io_interface.cpp**：通过互斥锁保护Modbus并发访问

---

## 暂停状态记录与恢复系统架构

### 设计哲学
- **状态冻结**：暂停时记录业务逻辑状态，如同冻结时间切片
- **状态回放**：恢复时重走之前记录的状态，如同重放时间流
- **跨语言协作**：C++端记录硬件状态，Python端记录业务状态
- **最小侵入**：不影响正常业务流程，仅在暂停/恢复时介入

### 核心组件

#### 1. 状态记录结构 (globals.h)
```cpp
struct PauseStateRecord {
    bool warehouse_was_active;      // 入库是否在进行中
    int warehouse_state_value;      // 入库状态机值
    int warehouse_target_layer;     // 入库目标层
    bool outbound_was_active;       // 出库是否在进行中
    int outbound_state_value;       // 出库状态机值
    int outbound_source_layer;      // 出库源层
    bool has_recorded_state;        // 是否已记录状态
};
```

#### 2. C++ 端实现 (main.cpp / ethercat_node.cpp)
- **暂停时记录**：`pause_motors_only()` → 请求Python端报告状态 → 保存到 `g_pause_state_record`
- **恢复时回放**：`resume_from_short_pause()` → 发送恢复命令 → 携带记录的状态信息
- **话题通信**：
  - `/pause_state_command` (C++ → Python)：发送记录/恢复命令
  - `/pause_state_report` (Python → C++)：报告当前业务状态

#### 3. Python 端实现 (pause_resume_manager.py)
- **状态报告**：`PauseResumeManager.report_current_pause_state()` → 将 `warehouse_state`/`outbound_state` 序列化为字符串
- **状态恢复**：`PauseResumeManager.handle_resume_command()` → 解析命令 → `restore_warehouse_state()` / `restore_outbound_state()`
- **状态映射**：根据状态值恢复到对应步骤，确保业务逻辑继续执行

### 状态恢复映射表

#### 入库流程恢复
| 原状态 | 恢复后状态 | 说明 |
|--------|-----------|------|
| IDLE | IDLE | 未启动，无需恢复 |
| WAIT_FOR_ENTRY | WAIT_FOR_ENTRY | 重新等待入库信号 |
| CONVEYOR_MOVING | WAIT_FOR_ENTRY | 重新检测入库条件 |
| LIFT_MOVING | CONVEYOR_MOVING | 重新触发层移动 |
| POST_LIFT_PROCESSING | LIFT_MOVING | 重新执行层移动后操作 |
| DELAY_PROCESSING | POST_LIFT_PROCESSING | 重新执行延迟处理 |
| COMPLETED | COMPLETED | 已完成，回到IDLE |

#### 出库流程恢复
| 原状态 | 恢复后状态 | 说明 |
|--------|-----------|------|
| IDLE | IDLE | 未启动，无需恢复 |
| WAIT_FOR_EXIT | WAIT_FOR_EXIT | 重新等待出库信号 |
| LIFT_MOVING | WAIT_FOR_EXIT | 重新触发层移动 |
| POST_LIFT_PROCESSING | LIFT_MOVING | 重新执行层移动后操作 |
| CONVEYOR_MOVING | POST_LIFT_PROCESSING | 重新执行出库操作 |
| COMPLETED | COMPLETED | 已完成，回到IDLE |

### 关键工作流程

#### 暂停流程
```
用户短按暂停按钮 → lights_controller检测到短按 → 设置 g_short_pause_requested
→ main.cpp 检测到请求 → pause_motors_only() 
→ 发送 RECORD_STATE 到 Python → Python 报告当前状态 → C++ 保存到 g_pause_state_record
→ 停止所有轴和DO信号 → 设置 g_short_pause_active = true
```

#### 恢复流程
```
用户按下启动按钮 → main.cpp 检测到 g_start_button_pressed + g_short_pause_active
→ resume_from_short_pause() 
  → 立即清除 g_short_pause_active（确保层指令能被正常接收）
  → 重置所有轴到 UNINITIALIZED 状态
  → 发送 RESUME 命令（携带记录的状态）到 Python
→ Python 解析命令 → restore_warehouse_state() / restore_outbound_state()
→ Python 重新发送层指令到目标层
→ handle_layer_command() 接收层指令（此时 g_short_pause_active 已清除）
→ 发布位移指令 → 轴开始移动 → 真正到达后触发层移动完成
```

### 使用示例

```cpp
// C++ 端：请求记录状态
global_node->publish_pause_state_record_request();

// C++ 端：发送恢复请求
global_node->publish_pause_state_resume_request();
```

```python
# Python 端：自动处理，无需手动调用
# 通过 /pause_state_command 话题自动触发
```

### 集成点

1. **main.cpp**：`pause_motors_only()` 和 `resume_from_short_pause()` 函数
2. **ethercat_node.cpp**：发布器和订阅器的创建，以及命令处理
3. **business_logic_processor.py**：`pause_state_command_callback()` 和相关恢复函数（委托给 `PauseResumeManager`）

---

## 设计决策记录

1. **为何选择日志回调而非宏替换**：
   - 无需修改现有代码中的 RCLCPP_ERROR/WARN 调用
   - 能够捕获第三方库产生的日志
   - 避免宏替换带来的编译复杂性

2. **为何使用16位故障码**：
   - 提供足够的分类空间（16个主类 × 16个子类 × 256个具体代码）
   - 与现有工业标准兼容
   - 便于十六进制阅读和调试

3. **为何采用状态回退策略而非精确恢复**：
   - 业务逻辑涉及IO信号、轴运动等复杂交互，精确恢复难度大
   - 回退到前一个稳定状态更安全，避免中间状态的不一致性
   - 简化实现，降低维护复杂度

3. **为何保留字符串发布格式**：
   - 保持与现有监控系统兼容
   - 简化调试和日志分析
   - 可通过 JSON 输出提供结构化数据

4. **为何将故障映射设计为可配置**：
   - 适应不同应用场景的日志模式
   - 支持运行时动态更新映射规则
   - 便于测试和模拟特定故障场景

5. **故障清除逻辑的设计演进** (2026-03-26)：
   - **问题**：原逻辑使用固定状态字列表判断是否清除成功，无法覆盖所有驱动器状态
   - **改进**：改为检查状态字 bit3（故障位），只要 bit3=0 即认为清除成功
   - **优势**：
     - 不依赖特定驱动器的状态字实现细节
     - 符合 EtherCAT CiA402 标准的状态机定义
     - 支持雷赛驱动器 0x8402 等新型故障码的自动清除
   - **回退策略**：超时后输出故障码，提示可能需要人工排查或断电重启

6. **雷赛驱动器专用故障清除** (2026-03-26)：
   - **需求**：雷赛驱动器需要通过 SDO 0x2057:00 写入 1 来清除特定故障
   - **实现**：`LeisaiServoAxis::handle_leisai_fault_clear()` 方法
   - **流程**：
     - 步骤0：SDO 写入 0x2057:00 = 1（雷赛专用故障复位，仅手动触发时执行）
     - 步骤1：PDO 写 0x0080（标准故障复位）
     - 步骤2-4：标准 CiA402 状态机恢复流程
   - **自动 vs 手动清除**：
     - `0x821b` 通讯错误：**自动清除**，跳过 SDO 步骤（直接从步骤1开始）
     - `0x8402` 及其他故障：**手动清除**，需调用 `clear_fault()` 触发，执行完整流程（含SDO）
   - **注意**：SDO 操作使用 `ecrt_master_sdo_download`，需要保存 master 指针

7. **STOP命令状态清理机制** (2026-03-26)：
   - **问题**：axis3/axis4（板宽调整）、axis5（层移动）在stop后回到AUTO_MODE时继续执行原指令
   - **根因**：`stop()`仅设置标志使轴进入STOPPED状态，未清理`displacement_updated_`等业务逻辑状态
   - **修复策略**：
     - **ServoAxisBase::stop()**：重置`displacement_updated_`、将`target_pulses_`设为当前位置、清理点动标志
     - **STOPPED状态处理**：在状态机中持续清理运动状态，状态转换前再次确认
     - **业务逻辑层**：`LayerCommandProcessor::reset_motion_state()`清理层移动状态；EthercatNode重置板宽调整标志
   - **设计原则**：
     - 分层清理：基类清理轴状态，业务层清理业务状态
     - 防御式编程：STOPPED状态持续清理，状态转换前二次确认
     - 幂等性：多次stop调用不会产生副作用

8. **暂停恢复流程时序修复** (2026-03-30)：
   - **问题**：短按暂停后恢复，业务逻辑重新发送层指令但被 `handle_layer_command()` 拒绝
   - **根因**：`g_short_pause_active` 在 `resume_from_short_pause()` 末尾才清除，业务逻辑收到恢复请求后立即发送层指令时状态仍为true
   - **修复策略**：将 `g_short_pause_active.store(false)` 从函数末尾移到函数开始处
   - **时序对比**：
     - **修复前**：重置轴 → 发送恢复请求 → Python立即发送层指令 → 被拒绝 → 清除`g_short_pause_active`
     - **修复后**：清除`g_short_pause_active` → 重置轴 → 发送恢复请求 → Python发送层指令 → 正常处理
   - **关键洞察**：状态清除必须在任何可能触发外部响应的操作之前完成

---

## 未来扩展方向

1. **故障历史记录**：添加时间戳和故障持续时间跟踪
2. **严重度分级**：在故障码中嵌入严重度信息（致命、错误、警告、信息）
3. **自动恢复策略**：根据故障类型触发预设的恢复动作
4. **远程监控接口**：通过 ROS2 服务提供故障查询和清除功能
5. **统计分析**：故障频率、MTBF（平均无故障时间）等指标计算

---

## SMEMA 协议通信架构

### 设计哲学
- **标准兼容**：遵循IPC-SMEMA-9851标准，确保上下游设备互联互通
- **中游视角**：本机作为中游设备，双向握手（上游接收+下游发送）
- **双状态机并行**：上游握手状态机 + 下游握手状态机独立运行
- **产品到位驱动**：`product_in_position`信号控制握手方向
  - `false`（无板）→ 输出MR=ON（要板）→ 启动上游握手
  - `true`（有板）→ 输出BA=ON（有板）→ 启动下游握手
- **信号滤波**：50ms防抖，避免误触发
- **超时保护**：30秒超时，防止死锁
- **可屏蔽控制**：`smema_set_enabled()`控制协议启用/禁用

### 启用/禁用配置

#### C++端 (`io_interface.hpp`)
```cpp
#define ENABLE_SMEMA  1  // 1:启用SMEMA协议通讯 0:禁用SMEMA协议
```

#### Python端 (`business_logic_processor.py`)
```python
self.ENABLE_SMEMA = True  # True:启用SMEMA协议通讯 False:禁用SMEMA协议
```

当禁用时：
- C++端：SMEMA处理器函数为空实现，不创建发布器/订阅器
- Python端：不处理SMEMA逻辑，不创建相关发布器/订阅器
- 硬件：DO814(MR)、DO815(BA)信号不会被控制，可作为普通DO使用

**设计说明**：
- SMEMA信号（UBA、DBR）始终存在于DI结构体中，即使 `ENABLE_SMEMA=0`
- **资源浪费**：2字节内存 + 每次读取2次赋值操作（纳秒级）
- **设计理由**：保持代码简洁，避免过度优化；启用时只需改一个宏开关
- **符合原则**：实用主义铁律——不对抗假想敌，避免理论完美陷阱

**禁用时的默认值**：
- `di.smema_uba = false`（默认无板）
- `di.smema_dbr = true`（默认要板，方便调试）
- **设计理由**：禁用SMEMA时，无需下游设备连接，默认要板状态方便业务层调试

### 核心组件

#### 1. SMEMA 处理器 (smema_handler)
- **双向握手**：上游接收板子 + 下游发送板子
- **信号防抖**：50ms防抖（符合SMEMA标准）
- **超时保护**：30秒超时，防止死锁
- **状态查询**：提供上下游状态独立查询接口

#### 2. 信号定义（中游设备）
| 信号 | 方向 | 寄存器 | Modbus索引 | 说明 | 握手方向 |
|------|------|--------|-----------|------|----------|
| UBA | 输入(DI) | M535 | di_values[23] | 上游有板待发 | 上游握手 |
| MR | 输出(DO) | M814 | do_values[14] | 本机要板 | 上游握手 |
| DBR | 输入(DI) | M536 | di_values[24] | 下游要板 | 下游握手 |
| BA | 输出(DO) | M815 | do_values[15] | 本机有板待发 | 下游握手 |

#### 3. 上游握手状态机（接收板子）
```
UPSTREAM_IDLE: 本机无板 → 输出MR=ON
  ↓ product_in_position=false
UPSTREAM_READY: 等待上游UBA=ON
  ↓ UBA=ON
UPSTREAM_RECEIVING: 板子传输中，保持MR=ON
  ↓ UBA=OFF（传输完成）
UPSTREAM_BOARD_ARRIVED: 板子到达，等待业务层确认
  ↓ 业务层确认接收
UPSTREAM_IDLE: 输出MR=OFF，循环
```

#### 4. 下游握手状态机（发送板子）
```
DOWNSTREAM_IDLE: 本机有板 → 输出BA=ON
  ↓ product_in_position=true
DOWNSTREAM_AVAILABLE: 等待下游DBR=ON
  ↓ DBR=ON
DOWNSTREAM_SENDING: 板子传输中，保持BA=ON
  ↓ DBR=OFF（传输完成）
DOWNSTREAM_SENT: 板子已发，等待业务层确认
  ↓ 业务层确认发送
DOWNSTREAM_IDLE: 输出BA=OFF，循环
```

#### 5. 跨层协作
| 层级 | 职责 | 接口 |
|------|------|------|
| C++ SMEMA处理器 | 双向握手、状态机、硬件控制 | `smema_set_product_in_position()` |
| C++ SMEMA处理器 | 上游板子到达通知 | `smema_can_receive_board()` |
| C++ SMEMA处理器 | 下游板子发送通知 | `smema_can_send_board()` |
| Python业务逻辑 | 产品到位信号驱动 | 发布到`/smema/product_in_position`话题 |
| Python业务逻辑 | 状态查询 | 从`/io_status`话题读取UBA/DBR/MR/BA信号 |

**设计原则**：
- **单一信号驱动**：`product_in_position`信号控制所有握手逻辑
- **简化设计**：删除冗余的确认信号，减少ROS2话题
- **业务层控制**：通过设置`product_in_position`控制握手完成时机

### 关键工作流程

#### 上游接收板子流程
```
1. 业务层检测：本机无板（product_in_position=false）
2. C++调用：smema_set_product_in_position(false)
3. SMEMA处理器：输出MR=ON（要板）
4. 上游设备：检测到MR=ON + 上游有板 → 输出UBA=ON
5. SMEMA处理器：检测到UBA=ON → 状态转到RECEIVING
6. 上游设备：传输板子 → 传输完成 → 输出UBA=OFF
7. SMEMA处理器：检测到UBA=OFF → 状态转到BOARD_ARRIVED
8. 业务层：检测到板子到位 → 设置product_in_position=true
9. SMEMA处理器：检测到product_in_position=true → 输出MR=OFF → 状态回到IDLE
```

#### 下游发送板子流程
```
1. 业务层检测：本机有板（product_in_position=true）
2. C++调用：smema_set_product_in_position(true)
3. SMEMA处理器：输出BA=ON（有板）
4. 下游设备：检测到BA=ON + 下游要板 → 输出DBR=ON
5. SMEMA处理器：检测到DBR=ON → 状态转到SENDING
6. 业务层：启动皮带发送板子
7. 下游设备：接收板子 → 接收完成 → 输出DBR=OFF
8. SMEMA处理器：检测到DBR=OFF → 状态转到SENT
9. 业务层：检测到板子已发送 → 设置product_in_position=false
10. SMEMA处理器：检测到product_in_position=false → 输出BA=OFF → 状态回到IDLE
```

#### 产品到位信号驱动逻辑
```python
# Python业务层示例
def update_product_position(self):
    # 检测产品到位信号
    product_in_position = (
        self.di['buffer_in_position'] or 
        self.di['conveyor_in_position']
    )
    
    # 驱动SMEMA握手
    self.set_smema_product_position(product_in_position)
    
    # 检查上游板子是否到达
    if smema_can_receive_board():
        self.handle_board_arrived()
        smema_confirm_board_received()
    
    # 检查下游板子是否发送完成
    if smema_can_send_board():
        self.handle_board_sent()
        smema_confirm_board_sent()
```

### 使用示例

```cpp
// C++端：初始化SMEMA处理器
SMEMA_Config config = {
    .di_base_address = 535,
    .do_base_address = 814,
    .handshake_filter_ms = 50,
    .receive_timeout_ms = 30000,
    .send_timeout_ms = 30000
};
smema_init(&config);

// 主循环每100ms调用
smema_process_cycle();

// 业务层设置产品到位信号（通过ROS2话题）
// Python端发布到 /smema/product_in_position 话题
// C++端自动调用 smema_set_product_in_position()

// 检查上游板子是否到达
if (smema_can_receive_board()) {
    // 处理板子到达逻辑
}

// 检查下游板子是否发送完成
if (smema_can_send_board()) {
    // 处理板子发送完成逻辑
}

// 查询状态
Upstream_State upstream = smema_get_upstream_state();
Downstream_State downstream = smema_get_downstream_state();
SMEMA_Status status = smema_get_status();

// 屏蔽控制
smema_set_enabled(false);  // 禁用SMEMA协议
```

```python
# Python端：业务层驱动示例
def main_loop(self):
    # 检测产品到位信号
    product_in_position = (
        self.di['buffer_in_position'] or 
        self.di['conveyor_in_position']
    )
    
    # 驱动SMEMA握手
    self.c_interface.set_product_in_position(product_in_position)
    
    # 检查上游板子是否到达
    if self.c_interface.can_receive_board():
        self.handle_board_arrived()
        self.c_interface.confirm_board_received()
    
    # 检查下游板子是否发送完成
    if self.c_interface.can_send_board():
        self.handle_board_sent()
        self.c_interface.confirm_board_sent()
```

### 集成点

1. **io_interface.hpp/cpp**：添加SMEMA双向握手信号到DI/DO结构体
   - DI：UBA(M535/di_values[23])、DBR(M536/di_values[24])
   - DO：MR(M814/do_values[14])、BA(M815/do_values[15])
2. **smema_handler.cpp/hpp**：双向握手状态机实现
3. **ethercat_node.cpp**：
   - 初始化SMEMA处理器，IO循环中调用`smema_process_cycle()`
   - `/io_status`话题发布SMEMA信号：DI23/DI24/DO14/DO15
4. **business_logic_processor.py**：业务层通过`/smema/product_in_position`话题驱动握手（IO/SMEMA逻辑委托给 `IoSignalHandler`）
5. **硬件接线**：
   - 上游握手：DI模块M535接上游BA，DO模块M814接上游MR
   - 下游握手：DI模块M536接下游MR，DO模块M815接下游BA

### 设计决策记录（SMEMA）

9. **为何采用双向握手架构** (2026-04-14)：
   - **需求**：设备作为中游设备，既要接收上游板子，又要发送板子给下游
   - **设计**：双状态机并行运行，通过`product_in_position`信号驱动握手方向
   - **优势**：
     - 符合SMEMA标准：上游有板∧本机要板→接收；本机有板∧下游要板→发送
     - 简化业务逻辑：业务层只需设置产品到位信号，握手逻辑自动处理
     - 状态清晰：上下游状态独立，互不干扰
   - **关键洞察**：产品到位信号是握手的唯一驱动源，消除了业务层与握手层的耦合

10. **为何删除确认信号** (2026-04-15)：
    - **问题**：原设计需要业务层发布`board_received`和`board_sent`确认信号，增加了复杂度
    - **改进**：改为通过`product_in_position`信号控制握手完成
    - **优势**：
      - 减少ROS2话题数量（从3个减少到1个）
      - 简化设计：单一信号驱动所有握手逻辑
      - 业务层控制权不变：通过设置`product_in_position`控制握手完成时机
    - **关键洞察**：能消失的分支永远比能写对的分支更优雅

---

## 接驳台速度控制架构

### 设计哲学
- **层号驱动调速**：映射后层号决定接驳台速度，远离中心层越远越快
- **中性区间保持**：-7~+7层保持默认速度，消除无意义的速度波动
- **线性插值**：距离中性区间越远速度线性递增，最高300mm/s
- **意图驱动归位**：回到层1是空跑归位，不按层号调速，强制使用远层速度(300mm/s)

### 速度映射规则

| 映射后层号 | 速度 (mm/s) | 说明 |
|-----------|-------------|------|
| -7 ~ +7 | 150 (默认) | 中性区间，速度不变 |
| < -7 | 150 → 300 | 线性递增，最远-15层≈300mm/s |
| > +7 | 150 → 300 | 线性递增，最远+30层≈300mm/s |
| 归位层1 | 300 (强制) | 空跑归位，`fast_return=True` |

### 层号映射（上游协议 → 内部层号）
- 原始层号1-15 → 映射到 -15 ~ -1
- 原始层号16-43 → 映射到 +3 ~ +30

### 核心组件

#### Python端 (business_logic_processor.py + io_signal_handler.py + process_handlers.py + pause_resume_manager.py + models.py)
- **calculate_conveyor_speed(mapped_layer)**: 层号→速度映射函数（在 `business_logic_processor.py` 中）
- **adjust_conveyor_speed_by_layer(mapped_layer)**: 发布速度指令到axis1_1/axis1_2（在 `business_logic_processor.py` 中）
- **send_layer_command(layer)**: 层指令发送时自动触发调速（在 `business_logic_processor.py` 中）

#### C++端 (ethercat_node.cpp)
- **/jog_speed_command** 话题：接收速度设置命令，格式 `"axis_name:speed"`
- **handle_jog_speed_command()**: 解析并设置轴点动速度

### 话题链路
```
byte_multiarray_parser (层号映射) → /layer_command → LayerCommandProcessor
                                                         ↓
business_logic_processor.send_layer_command()
    → /layer_command (层指令)
    → /jog_speed_command (速度指令: axis1_1:speed; axis1_2:speed)
                                                         ↓
ethercat_node.handle_layer_command() + handle_jog_speed_command()
```

### 配置参数
```python
CONVEYOR_SPEED_NEAR = 150.0      # 近层速度 (mm/s)
CONVEYOR_SPEED_FAR  = 300.0      # 远层速度 (mm/s)
CONVEYOR_SPEED_NEUTRAL_RANGE = 7  # 中性区间半径
CONVEYOR_SPEED_AXIS_NAMES = ["axis5"]  # 升降轴
```

---

## T 型速度斜坡控制架构 (RampProfile)

### 设计哲学
- **软起动 / 软停止**：用加速度斜坡替代阶跃式启停，消除母线电压过压
- **向后兼容**：`accel_per_cycle_=0` 时完全退化为原恒速步进逻辑，不影响未配置的轴
- **职责正交**：`get_max_step()` 回答"允许多快"（硬件限速），`RampState` 回答"如何优雅变速"（软件剖面）
- **消除特殊情况**：通过配置而非类型判断实现差异化——哪个轴需要斜坡就配置它，无需 `if (axis5)` 分支

### 核心组件

#### 1. 斜坡状态机 (servo_axis_base.hpp)
```cpp
struct RampState {
    enum Phase { ACCEL, CONSTANT, DECEL, IDLE };

    Phase   phase{IDLE};
    int32_t current_step_{0};           // 当前周期步长 (pulses)
    int32_t min_step_{5};               // 启 / 停点步长, 防零速抖动
    int32_t accel_per_cycle_{0};        // 加速度 (pulses/cycle^2), 0=禁用

    void reset() { phase = IDLE; current_step_ = 0; }
    bool enabled() const { return accel_per_cycle_ > 0; }
};
```

#### 2. 斜坡控制 (servo_axis_base.cpp - gradual_approach)
```
新运动 → ACCEL:  step 每周期 +accel, 直到 max_step
          ↓
       CONSTANT:  step = max_step (匀速)
          ↓
          DECEL:  step 每周期 -accel, 直到 min_step
          ↓
          IDLE:   到达目标, 精确对齐
```

- **减速点计算**：匀减速运动学 — `decel_needed = (v² - v_min²) / (2a)`
- **短行程三角波**：总距离 < 2×加速距离时自动跳过 CONSTANT 段，加速到峰值即减速
- **运行时调速兼容**：Python 调速（`_check_return_speed_transition`）改变 `get_max_step()` 上限，斜坡自动适配

#### 3. 配置接口 (ServoAxisBase)
```cpp
void configure_ramp(int32_t accel_per_cycle, int32_t min_step);
void reset_motion_ramp();  // 新位移指令到达时调用
```

#### 4. axis5 配置 (ethercat_node.cpp)
```cpp
axis->configure_ramp(2, 5);  // accel=2 pulses/cycle^2, min_step=5
```

### 运动参数推导

| jog_speed | max_step | 加速时间 | 减速距离 |
|-----------|----------|----------|----------|
| 150 mm/s | 450 | 222 ms | ~5,600 pulses |
| 200 mm/s | 600 | 297 ms | ~10,100 pulses |
| 300 mm/s | 900 | 447 ms | ~20,200 pulses |

### 文件改动范围
- **servo_axis_base.hpp**: RampState 结构体 + ramp_ 成员 + configure_ramp/public API
- **servo_axis_base.cpp**: gradual_approach() 重构 + configure_ramp() 实现
- **huichuan_servo_axis.cpp**: manual/auto 位移动处理中加入 ramp_.reset()
- **leisai_servo_axis.cpp**: auto 位移动处理中加入 ramp_.reset()
- **ethercat_node.cpp**: axis5 初始化时 configure_ramp(2, 5)

### 设计决策记录 (Ramp)

11. **为何在 gradual_approach() 内部实现而非独立控制器** (2026-07-08):
    - **问题**：axis5 升降轴在层移动（远层 300mm/s）启停时产生母线电压过压故障
    - **根因**：阶跃式启停（步长瞬时 0→900→0），动能全部转化为再生能量
    - **设计**：在 `gradual_approach()` 内置 T 型速度剖面，通过 `accel_per_cycle_` 控制加速度
    - **优势**：
      - 零定义死代码：`motion_acceleration_` 概念通过 `configure_ramp()` 落地
      - 向后兼容：`accel_per_cycle_=0` 时原逻辑不变
      - 职责分离：`get_max_step()` 不变，ramp 层作为软保护叠加
    - **关键洞察**：母线过压不是速度问题（300mm/s），是速度变化率问题（d²x/dt²=∞）

12. **为何 ramp_.reset() 放在位移指令处理而非 gradual_approach 内部** (2026-07-08):
    - 斜坡状态（IDLE/ACCEL/CONSTANT/DECEL）与一次完整的运动绑定
    - 新位移指令代表新运动开始 → 在指令生产者处重置更符合语义
    - gradual_approach 内部已有 "DECEL 过头回 ACCEL" 的容错逻辑，处理运行时调速场景

---

## Python 业务逻辑模块架构 (business_logic_py)

### 模块结构

```
business_logic_py/business_logic_py/
├── __init__.py                       # 包导出
├── models.py                         # 枚举 + 数据类（无状态，无副作用）
├── io_signal_handler.py              # IO信号解析、SMEMA协议、产品到位检测
├── pause_resume_manager.py           # 暂停状态记录、恢复命令解析、状态还原
├── process_handlers.py               # 入库/出库/放行三大业务状态机
├── business_logic_processor.py       # ROS2 Node壳子：发布/订阅、定时器、调度
└── byte_multiarray_parser.py         # 字节数组解析器
```

### 设计哲学

- **单一职责**：每个文件一个业务领域，状态机完整逻辑不跨文件
- **反向引用**：子模块通过 `self.proc` 引用处理器，子模块之间零依赖
- **Node壳子**：`BusinessLogicProcessor` 仅管理ROS2生命周期和调度，业务逻辑全权委托

### 模块职责

| 文件 | 职责 | 核心类 |
|------|------|--------|
| `models.py` | 枚举(WarehouseState/OutboundState/PassThroughState/CommandType/FaultCode) + 数据类(ControlAction) | 无类，纯定义 |
| `io_signal_handler.py` | IO解析、信号映射、SMEMA握手、产品到位状态机 | `IoSignalHandler` |
| `pause_resume_manager.py` | 暂停记录、恢复命令解析、状态还原、待执行调度 | `PauseResumeManager` |
| `process_handlers.py` | 入库/出库/放行三大业务流程状态机 | `ProcessHandlers` |
| `business_logic_processor.py` | ROS2 Node、发布/订阅器、定时器、命令发送、调度 | `BusinessLogicProcessor` |

### 委托关系

```
BusinessLogicProcessor (Node壳子)
  ├── io_handler = IoSignalHandler(self)
  ├── pause_resume_mgr = PauseResumeManager(self)
  └── process_handlers = ProcessHandlers(self)

process_logic() 每周期调用:
  io_handler.process_product_arrival_logic()
  io_handler.update_product_position()
  io_handler.check_smema_handshake()
  process_handlers.process_warehouse_logic()
  process_handlers.process_outbound_logic()
  process_handlers.process_release_logic()
```

---

## 缝隙信号防抖架构 (GapDetectDebouncer)

### 设计哲学
- **信号语义与物理现实割裂**：缝隙传感器（M540/M541）安装在两个物理区域交界处，信号消失只代表"板子后端通过传感器光束"，不代表"板子完全进入目标区域"。两者之间存在物理差量：$\Delta t = d / v$
- **非对称防抖**：上升沿（板子到达）立即响应，下降沿（板子离开）延迟确认。消抖与确认合一
- **防抖归硬件层**：信号稳定性是硬件关注点，防抖后的信号被所有上层统一消费，消除业务层重复防抖

### 核心组件

#### 1. 防抖状态结构 (ethercat_node.hpp)
```cpp
struct GapDetectDebouncer {
    bool m541_filtered{false};       // M541 防抖后输出
    int  m541_falling_counter{0};    // M541 下降沿持续计数
    bool m540_filtered{false};       // M540 防抖后输出
    int  m540_falling_counter{0};    // M540 下降沿持续计数
};
static constexpr int GAP_DEBOUNCE_CYCLES = 3;  // 3周期 = 300ms @ 100ms周期
```

#### 2. 防抖逻辑 (ethercat_node.cpp - debounce_gap_signals)
```
原始信号 = 1 → 输出立即 = 1，计数器清零
原始信号 = 0 → 计数器递增
             → 计数器 >= 3 → 输出 = 0（确认消失）
             → 计数器 < 3  → 输出保持上一个值（延迟消失）
```

#### 3. 调用位置
- `handle_io_signals()` 入口处调用 `debounce_gap_signals(di)`
- IO监控线程固定100ms周期，防抖时间 = `GAP_DEBOUNCE_CYCLES × 100ms`
- 防抖后信号通过 `/io_status` 话题发布给Python业务层

### 防抖参数推导（日志驱动）
| 指标 | 值 | 来源 |
|------|-----|------|
| M541首次假消失持续 | 111ms | 实测日志：板子后端通过传感器后的抖动 |
| 板子通过M541时间 | 218ms | 实测日志：M541亮起持续时间 |
| 2倍安全余量 | 222ms | 111ms × 2 |
| **防抖时间** | **300ms** | max(222, 218)取整，3周期 |

### IO 防抖红线

| 信号类别 | 防抖策略 | 原因 |
|---------|---------|------|
| **缝隙传感器** M540/M541 | 下降沿300ms防抖 | 物理缝隙特殊性，信号消失≠板子完全通过 |
| **急停** M516/M517 | **绝对禁止** | 低电平有效，下降沿=急停触发，防抖=延迟急停=致命 |
| **安全门** M518/M519 | **绝对禁止** | 安全要求立即响应 |
| **气缸到位** M527-M534 | **绝对禁止** | 瞬态事件信号，防抖破坏互斥时序 |
| **板位检测** M520/M523/M524/M525/M526 | 不需要 | 安装位置安全，无缝隙问题 |
| **按钮** M512-M514 | 不需要 | 已有业务层防抖 |
| **SMEMA** M535/M536 | 已有独立防抖 | smema_handler.cpp 50ms |

**禁止扩展为通用DI防抖框架**：六类信号六种需求，通用框架让致命错误（急停防抖）成为可能。

### 防抖影响的逻辑点

#### M541 下降沿防抖（5个逻辑点）
| # | 流程 | 状态 | 效果 |
|---|------|------|------|
| 1 | 入库 | IDLE | 启动门控延迟300ms（确保接驳台空闲） |
| **2** | **入库** | **CONVEYOR_MOVING** | **axis5移动延迟300ms（防止PCB损坏）** |
| 3 | 入库 | DELAY_PROCESSING | "板子入缓存架"判断延迟300ms（更安全） |
| **4** | **出库** | **POST_LIFT_PROCESSING** | **axis5移动延迟300ms（防止PCB损坏）** |
| 5 | 放行 | CONVEYOR_RUNNING | board_dispatched延迟→要板延迟300ms |

#### M540 下降沿防抖（2个逻辑点）
| # | 流程 | 状态 | 效果 |
|---|------|------|------|
| 6 | 出库 | COMPLETED | 关DO813延迟300ms（确保板子离开） |
| 7 | 放行 | CONVEYOR_RUNNING | 停axis1延迟300ms（确保板子离开） |

### 配合的业务层延迟精简
C++层防抖300ms后，Python业务层的冗余延迟被精简：
| 参数 | 改前 | 改后 | 理由 |
|------|------|------|------|
| 入库/出库条件二延迟 | counter>=1（0ms实延迟） | 移除 | 防抖已确保信号稳定，延迟逻辑冗余 |
| `DELAY_BEFORE_STOP_MS` | 400ms | 200ms | 防抖修复axis5误动，板子姿态正常，停稳余量可减 |
| `OUTBOUND_DELAY_BEFORE_STOP_MS` | 300ms | 100ms | M540防抖已确认板子离开，仅需最小余量 |
| 放行`WAIT_FOR_CONVEYOR_OUT`延迟 | 3周期(300ms) | 1周期(100ms) | M540防抖已确认板子离开 |

### 设计决策记录

13. **为何防抖放在C++ IO层而非Python业务层** (2026-07-28)：
    - **单一真相源**：防抖后信号被所有上层统一消费，消除各处重复防抖
    - **硬件隔离**：信号稳定性是硬件层关注点，不应泄漏到业务层
    - **先例**：SMEMA已在C++层做防抖（smema_handler.cpp）
    - **零侵入**：Python业务层无需修改即可消费防抖后信号

14. **为何用非对称防抖而非对称防抖** (2026-07-28)：
    - **上升沿（板子到达）需立即响应**：检测灵敏度要求高
    - **下降沿（板子离开）需延迟确认**：确保板子完全通过缝隙
    - **对比SMEMA**：SMEMA用对称50ms防抖（握手协议要求双向稳定）；缝隙传感器只需单向（下降沿）防抖

15. **为何防抖时间设为300ms** (2026-07-28)：
    - **日志驱动**：实测M541首次假消失111ms，板子通过时间218ms
    - **2倍余量**：max(111×2, 218) = 222ms，取整300ms（3周期）
    - **因果链切断**：有防抖后axis5不会在板子跨缝隙时移动，后续抖动（548ms）不会发生，300ms足以覆盖首次假消失

---

*文档最后更新：2026-07-28*
*对应架构版本：v3.4（M540/M541缝隙信号防抖 + 业务层延迟精简）*

### 变更日志

#### v3.4 (2026-07-28): M540/M541缝隙信号防抖 + 业务层延迟精简
- **问题**：板子在缓存架与接驳台缝隙中时，M541信号短暂消失（111ms假消失）触发`conveyor_in_then_out`，导致axis5在板子跨缝隙时移动，PCB损坏
- **根因**：M541缝隙传感器的信号消失≠板子完全通过缝隙，存在物理差量。代码用点传感器的下降沿推断空间区域状态，信号语义过载
- **修复策略**：
  - **C++ IO层**：`handle_io_signals()`入口新增`debounce_gap_signals()`，对M540/M541做非对称下降沿防抖（300ms确认消失）
  - **Python业务层**：移除入库/出库条件二延迟逻辑（防抖已冗余）；`DELAY_BEFORE_STOP_MS` 400→200ms；`OUTBOUND_DELAY_BEFORE_STOP_MS` 300→100ms；放行停轴延迟 3周期→1周期
- **设计原则**：防抖归硬件层（单一真相源），业务层只消费稳定信号。非对称防抖——上升沿立即响应，下降沿延迟确认。防抖是缝隙传感器的专属药，不扩展为通用框架

#### v3.3 (2026-07-28): 要板信号配合0x0109周期 + 设备物理结构文档化
- **新增**：设备物理结构章节（物理布局图、DI/DO信号定义表、轴定义表、PCB流动路径、要板信号设计原则）
- **问题**：COMPLETED/PENDING_PUBLISH 状态下仍允许要板，导致基准层有板时上游继续送板，板子卡在 buffer_in 位置
- **根因**：`update_product_position()` 的 `no_board_reasons` 豁免了 COMPLETED 和 PENDING_PUBLISH 状态，认为"基准层有板也可以要板"
- **修复策略（方案A）**：
  - 收紧 `product_arrival_state` 判断为 `!= "IDLE"`（去掉 COMPLETED/PENDING_PUBLISH 豁免）
  - MR=ON 的唯一门控：`product_arrival_state == "IDLE"`
  - COMPLETED→IDLE 的 `board_dispatched` 门控已保证接驳台空闲，不需要在 MR 层重复判断入库状态
- **出库期间允许要板**：出库板路径（conveyor_in→conveyor_out）不经过基准层（buffer_out），与新板进料路径物理不重叠，可并行作业
- **设计原则**：让 MR 跟着 0x0109 状态机周期走。IDLE=基准层空+接驳台空闲→可以要板；非IDLE=板子在路上或基准层→不要板。0x0109状态机自身的 `board_dispatched` 门控是安全性的单一真相源

#### v3.2 (2026-07-21): 0x9113事件型故障不入fault_map
- **问题**：急停恢复后，新作业的 `0x0106` 响应携带异常码 `0x9113` 而非 `0x0000`
- **根因**：`0x9113` 是事件型故障（急停发生一次），却被 `handle_business_logic_fault` 写入了 C++ `fault_map_`（持久化容器），系统重启后未清除，导致 Python parser 的 `current_fault_code` 持续为 `0x9113`，`publish_start_result()` 取到残留值
- **修复策略（方案 F）**：在 `handle_business_logic_fault()` 中，`0x9113` 直接发布到 `/fault_code` 话题通知 parser，但不加入 `fault_map_`。事件型故障不入持久化容器，系统恢复后自然失效
- **设计原则**：事件型故障（急停）与状态型故障（轴通讯错误、超时）的生命周期根本不同。事件发生→通知→结束，不应持久化。`fault_map_` 的语义是"当前系统存在哪些持续故障"，急停不在此列

#### v3.1 (2026-07-09): 产品到位检测生命周期管理
- **问题**：`product_arrival_cycle_active` 只开不关（0x0105 打开，仅下一个 0x0105 通过 reset 间接关闭），报错和 0x0107 不关断，导致：
  - 报错后状态机继续运行，条件满足时继续向后要板
  - 结束作业后状态机继续运行，SMEMA 继续握手要板
- **修复策略**：
  - **publish_fault_code()**：非零故障码 → `product_arrival_cycle_active=False` + `reset_product_arrival_state_machine()` → 状态机回 IDLE + 停输送带
  - **新增 /end_operation_signal**：`process_end_operation(0x0107)` 发布 Empty → `end_operation_signal_callback` → 关断周期
  - **update_product_position()**：取消注释 `product_arrival_cycle_active` guard，恢复 SMEMA 要板保护
  - **IDLE 状态智能恢复**：新增 `buffer_out` 感知——报错 reset 后若出料口已有板，直接跳 PENDING_PUBLISH 补发 0x0109，避免板丢失
- **新增话题**：`/end_operation_signal` (std_msgs/msg/Empty)，由 byte_multiarray_parser 发布，BusinessLogicProcessor 订阅
- **设计原则**：`product_arrival_cycle_active` 是产品到位检测的单一真相源，同时控制状态机运行和 SMEMA 要板
- **问题**：axis5 升降轴层移动（远层 300mm/s, 400mm 行程）启停阶段阶跃变速，产生母线电压过压故障（状态字 0x1638）
- **根因**：`gradual_approach()` 使用恒定步长 `get_max_step()`，启动和到达目标时 step 瞬时跳变，重载动能全部回馈为再生能量
- **修复策略**：
  - **ServoAxisBase**：新增 `RampState` 结构体 + `configure_ramp()` / `reset_motion_ramp()` 接口
  - **gradual_approach()** 重构：`accel_per_cycle_>0` 时启用 T 型速度剖面（ACCEL → CONSTANT → DECEL）
  - **HuichuanServoAxis / LeisaiServoAxis**：位移指令到达时调用 `ramp_.reset()`
  - **ethercat_node.cpp**：axis5 初始化 `configure_ramp(2, 5)`
- **运动学**：减速段 `step` 从 max (900) 线性递减到 min (5)，减速段约 450ms，再生能量均匀分散到 450 个周期
- **设计原则**：向后兼容——未调用 `configure_ramp()` 的轴行为完全不变；通过配置而非类型判断实现差异化

#### v2.9 (2026-05-13): 手动模式层移动支持
- **问题**：手动模式下 axis5 无法接收层移动指令，因双重门控阻断
- **门控1**：`LayerCommandProcessor::process_layer_command()` 中 `g_auto_mode_initialized` 强制门控，手动模式下永远为false
- **门控2**：`HuichuanServoAxis::handle_huichuan_manual_operation()` 只处理点动(jog)，不处理位移指令(displacement_updated_)
- **修复策略**：
  - **LayerCommandProcessor**：移除对 `g_auto_mode_initialized` 的强制依赖，层指令在任何运行模式(手动/自动)下均可立即执行
  - **HuichuanServoAxis 手动模式状态机**：增加对 `displacement_updated_` 的处理，有位移指令时优先执行逐步逼近到位，无位移指令时退回点动控制
- **设计原则**：手动模式的本质是"操作者直接控制"，不应排斥程序化的精确到位；位移指令与点动互斥，位移执行期间暂停点动

#### v2.9.1 (2026-05-13): 手动/自动模式切换与运动安全性修复
- **隐患1（已修复）：位移→点动中断时目标丢失** — 点动请求打断层移动时，`target_pulses_` 被覆盖，位移目标永久丢失且无告警
  - 修复：新增 `saved_displacement_target_` / `has_saved_displacement_` 保护变量，点动中断位移时保存目标，点动结束后自动恢复
- **隐患2（已修复）：`joint_position_` 与实际位置漂移** — `joint_position_` 是纯软件模型，仅在初始化时从 `current_pos` 同步，长时间运行后漂移
  - 修复：轴静止时每周期用 `current_pos` 修正 `joint_position_`，消除累积误差
- **隐患3（已修复）：`stop()` 跨线程写运动变量** — `stop()` 从ROS2订阅者线程直接写 `target_pulses_`/`joint_position_`，与cyclic thread竞争，`volatile` 不保证原子性
  - 修复：`stop()` 只设请求标志（`stop_requested_`），运动变量清理移至cyclic thread的状态机中执行
- **隐患4（已消除）：位移→到位检测双重执行** — `gradual_approach()` 内部已检测到位，外部重复检测冗余
  - 修复：移除 `handle_huichuan_manual_operation` 中的重复到位检测
- **安全保证**：所有 PDO 写入（`EC_WRITE_S32`）均经过步长限制（`gradual_approach` 或 `MAX_STEP`），任何模式切换/指令切换均不会导致电机位置跳变

#### v2.9.2 (2026-05-14): 新增0x011E通知移动/0x011F移动结果协议
- **协议格式**：与0x0101/0x0102一致 — 0x011E payload: [宽度2B + 层号2B + 区域1B]，0x011F payload: [命令码2B + 故障码2B]
- **数据流**：PC→0x011E→byte_multiarray_parser→/layer_move_start→business_logic_processor→send_layer_command()→C++层移动→/layer_motion_completed→business_logic_processor(双重确认)→/layer_move_completed→byte_multiarray_parser→0x011F→PC
- **层号映射**：与0x0101完全一致（1-43→-15~30）
- **完成判断**：`layer_motion_completed` 信号 + `current_layer_float` 与目标层容差0.5层双重确认
- **超时检测**：30秒，超时回包0x011F带故障码0x5002
- **故障回包**：故障时自动回包0x011F（`_send_pending_response_with_fault` 新增分支）
- **设计原则**：层移动是"纯移动"流程，不涉及入库/出库IO操作，移动中不支持点动中断