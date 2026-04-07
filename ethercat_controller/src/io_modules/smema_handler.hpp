#ifndef SMEMA_HANDLER_HPP
#define SMEMA_HANDLER_HPP

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "io_interface.hpp"

/* ============================================================
 * SMEMA 协议处理器 - 下游设备实现
 * 
 * 功能：处理与上游设备的SMEMA标准握手协议
 * 核心：确认上游板子什么时候来，协调业务层接收
 * ============================================================ */

// SMEMA 状态机状态
typedef enum {
    SMEMA_STATE_IDLE,           // 空闲：等待缓存区准备好
    SMEMA_STATE_READY,          // 就绪：已发送MR，等待上游UBA
    SMEMA_STATE_RECEIVING,      // 接收中：UBA=ON，板子正在传输
    SMEMA_STATE_BOARD_ARRIVED,  // 板子到达：UBA=OFF，板子传输完成
} SMEMA_State;

// SMEMA 配置结构
typedef struct {
    int di_base_address;        // DI起始地址 (默认535)
    int do_base_address;        // DO起始地址 (默认814)
    int timeout_ms;             // 握手超时时间(毫秒)
    bool enable_ugb;            // 是否启用好板信号
    bool enable_ubb;            // 是否启用坏板信号
} SMEMA_Config;

// SMEMA 状态信息
typedef struct {
    SMEMA_State state;          // 当前状态
    bool mr_active;             // MR信号当前状态
    bool uba_active;            // UBA信号当前状态
    time_t state_enter_time;    // 进入当前状态的时间
    int board_count;            // 接收板子计数
    int timeout_count;          // 超时计数
} SMEMA_Status;

// 初始化SMEMA处理器
void smema_init(SMEMA_Config* config);

// 反初始化
void smema_deinit(void);

// 主处理循环 - 每周期调用(建议100ms)
void smema_process_cycle(void);

// 获取当前状态
SMEMA_State smema_get_state(void);
const char* smema_state_to_string(SMEMA_State state);

// 获取状态信息
SMEMA_Status smema_get_status(void);

// 强制设置MR信号（用于手动模式或调试）
void smema_set_mr_manual(bool state);

// 复位SMEMA状态机
void smema_reset(void);

// 检查是否可以接收新板子
bool smema_can_receive_board(void);

// 确认板子已被业务层接收
void smema_confirm_board_received(void);

// 获取接收到的板子数量
int smema_get_board_count(void);

// 设置业务层就绪状态（由业务层调用）
void smema_set_business_ready(bool ready);
bool smema_is_business_ready(void);

#endif // SMEMA_HANDLER_HPP
