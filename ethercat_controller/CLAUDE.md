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

#### 3. Python 端实现 (business_logic_processor.py)
- **状态报告**：`report_current_pause_state()` → 将 `warehouse_state`/`outbound_state` 序列化为字符串
- **状态恢复**：`handle_resume_command()` → 解析命令 → `restore_warehouse_state()` / `restore_outbound_state()`
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
3. **business_logic_processor.py**：`pause_state_command_callback()` 和相关恢复函数

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
| DBR | 输入(DI) | M538 | di_values[24] | 下游要板 | 下游握手 |
| BA | 输出(DO) | M815 | do_values[15] | 本机有板待发 | 下游握手 |

**注意**：硬件上M536/M537端子不存在，M538紧接着M535。

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
| Python业务逻辑 | 产品到位信号驱动 | 调用C++接口设置`product_in_position` |
| Python业务逻辑 | 板子接收确认 | 调用`smema_confirm_board_received()` |
| Python业务逻辑 | 板子发送确认 | 调用`smema_confirm_board_sent()` |

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
8. 业务层：检测到板子到达 → 调用smema_confirm_board_received()
9. SMEMA处理器：输出MR=OFF → 状态回到IDLE
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
9. 业务层：确认发送完成 → 调用smema_confirm_board_sent()
10. SMEMA处理器：输出BA=OFF → 状态回到IDLE
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
    .send_timeout_ms = 30000,
    .enable_ugb = false,
    .enable_ubb = false
};
smema_init(&config);

// 主循环每100ms调用
smema_process_cycle();

// 业务层设置产品到位信号
smema_set_product_in_position(true);   // 本机有板
smema_set_product_in_position(false);  // 本机无板

// 检查上游板子是否到达
if (smema_can_receive_board()) {
    // 处理板子到达逻辑
    smema_confirm_board_received();
}

// 检查下游板子是否发送完成
if (smema_can_send_board()) {
    // 处理板子发送完成逻辑
    smema_confirm_board_sent();
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
   - DI：UBA(M535/di_values[23])、DBR(M538/di_values[24])
   - DO：MR(M814/do_values[14])、BA(M815/do_values[15])
   - **注意**：硬件上M536/M537端子不存在，M538紧接着M535
2. **smema_handler.cpp/hpp**：双向握手状态机实现
3. **ethercat_node.cpp**：
   - 初始化SMEMA处理器，IO循环中调用`smema_process_cycle()`
   - `/io_status`话题发布SMEMA信号：DI23/DI24/DO14/DO15
4. **business_logic_processor.py**：业务层通过`smema_set_product_in_position()`驱动握手
5. **硬件接线**：
   - 上游握手：DI模块M535接上游BA，DO模块M814接上游MR
   - 下游握手：DI模块M538接下游MR，DO模块M815接下游BA

### 设计决策记录（SMEMA）

9. **为何采用双向握手架构** (2026-04-14)：
   - **需求**：设备作为中游设备，既要接收上游板子，又要发送板子给下游
   - **设计**：双状态机并行运行，通过`product_in_position`信号驱动握手方向
   - **优势**：
     - 符合SMEMA标准：上游有板∧本机要板→接收；本机有板∧下游要板→发送
     - 简化业务逻辑：业务层只需设置产品到位信号，握手逻辑自动处理
     - 状态清晰：上下游状态独立，互不干扰
   - **关键洞察**：产品到位信号是握手的唯一驱动源，消除了业务层与握手层的耦合

---

*文档最后更新：2026-04-14*
*对应架构版本：v2.6（SMEMA双向握手支持）*