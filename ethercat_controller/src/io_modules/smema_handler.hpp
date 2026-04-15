#ifndef SMEMA_HANDLER_HPP
#define SMEMA_HANDLER_HPP

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "io_interface.hpp"

/* ============================================================
 * SMEMA 协议处理器 - 中游设备双向握手实现
 * 
 * 功能：处理与上游设备的接收握手 + 与下游设备的发送握手
 * 
 * 核心逻辑：
 *   上游握手：上游有板(UBA) ∧ 本机要板(MR) → 接收板子
 *   下游握手：本机有板(BA) ∧ 下游要板(DBR) → 发送板子
 * 
 * 信号定义：
 *   输入：UBA(上游有板)、UGB(上游好板)、UBB(上游坏板)、DBR(下游要板)
 *   输出：MR(本机要板)、BA(本机有板)
 * ============================================================ */

// ==================== 上游握手状态机（接收板子） ====================
typedef enum {
    UPSTREAM_IDLE,           // 空闲：等待本机就绪
    UPSTREAM_READY,           // 就绪：已发送MR，等待上游UBA
    UPSTREAM_RECEIVING,       // 接收中：UBA=ON，板子正在传输
    UPSTREAM_BOARD_ARRIVED,   // 板子到达：UBA=OFF，传输完成
} Upstream_State;

// ==================== 下游握手状态机（发送板子） ====================
typedef enum {
    DOWNSTREAM_IDLE,          // 空闲：本机无板
    DOWNSTREAM_AVAILABLE,     // 有板待发：已发送BA，等待下游DBR
    DOWNSTREAM_SENDING,       // 发送中：DBR=ON，板子正在传输
    DOWNSTREAM_SENT,          // 板子已发：DBR=OFF，传输完成
} Downstream_State;

// ==================== 配置结构 ====================
typedef struct {
    int di_base_address;        // DI起始地址 (默认535)
    int do_base_address;        // DO起始地址 (默认814)
    int handshake_filter_ms;    // 握手滤波时间(毫秒，默认50ms)
    int receive_timeout_ms;     // 接收超时时间(毫秒)
    int send_timeout_ms;        // 发送超时时间(毫秒)
} SMEMA_Config;

// ==================== 状态信息 ====================
typedef struct {
    // 上游状态
    Upstream_State upstream_state;
    bool mr_active;             // 本机要板信号
    bool uba_active;            // 上游有板信号
    time_t upstream_state_time; // 上游状态进入时间
    int receive_count;          // 接收板子计数
    
    // 下游状态
    Downstream_State downstream_state;
    bool ba_active;             // 本机有板信号
    bool dbr_active;            // 下游要板信号
    time_t downstream_state_time; // 下游状态进入时间
    int send_count;             // 发送板子计数
    
    // 错误计数
    int receive_timeout_count;
    int send_timeout_count;
} SMEMA_Status;

// ==================== 初始化与清理 ====================
void smema_init(SMEMA_Config* config);
void smema_deinit(void);

// ==================== 主处理循环 ====================
void smema_process_cycle(void);

// ==================== 状态查询 ====================
Upstream_State smema_get_upstream_state(void);
Downstream_State smema_get_downstream_state(void);
SMEMA_Status smema_get_status(void);
const char* smema_upstream_state_to_string(Upstream_State state);
const char* smema_downstream_state_to_string(Downstream_State state);

// ==================== 业务层接口 ====================
// 设置本机产品到位状态（由业务层调用）
//   product_in_position = true  → 本机有板 → 输出BA给下游
//   product_in_position = false → 本机无板 → 输出MR给上游
void smema_set_product_in_position(bool in_position);

// 检查是否可以接收新板子（上游握手就绪）
bool smema_can_receive_board(void);

// 确认板子已被业务层接收（上游握手完成）
void smema_confirm_board_received(void);

// 检查是否可以发送板子（下游握手就绪）
bool smema_can_send_board(void);

// 确认板子已被业务层发送（下游握手完成）
void smema_confirm_board_sent(void);

// ==================== 手动控制（调试用） ====================
void smema_set_mr_manual(bool state);
void smema_set_ba_manual(bool state);

// ==================== 复位与统计 ====================
void smema_reset(void);
int smema_get_receive_count(void);
int smema_get_send_count(void);

// ==================== 屏蔽控制 ====================
void smema_set_enabled(bool enabled);
bool smema_is_enabled(void);

#endif // SMEMA_HANDLER_HPP
