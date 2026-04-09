// globals.h
#ifndef GLOBALS_H
#define GLOBALS_H

#include <ecrt.h>  // EtherCAT 类型定义
#include <modbus/modbus.h>
#include <atomic>
#include <memory>
#include <string>

// ============================================
// 控制源选择宏 - 全局统一配置
// 1: 使用IO控制手自动模式（通过手自动按钮DI04判断）
// 0: 使用话题控制手自动模式（通过接收ROS话题命令）
// ============================================
#define CONTROL_SOURCE_IO 1

// 前向声明
class EthercatNode;
extern std::shared_ptr<EthercatNode> global_node;

// EtherCAT相关全局变量
extern ec_master_t *master;
extern ec_domain_t *domain1;
extern uint8_t *domain1_pd;
extern std::atomic<bool> g_should_exit;

// 状态变量
extern ec_master_state_t master_state;
extern ec_domain_state_t domain1_state;
extern unsigned int counter;
extern unsigned int sync_ref_counter;
extern const struct timespec cycletime;
extern pthread_t thread;
extern bool running;

// 复位灯控制标志（AL states 0x08且全部轴自动模式时熄灭）
extern std::atomic<bool> g_should_clear_reset_light;

// 短按暂停相关全局变量
extern std::atomic<bool> g_short_pause_active;      // 短按暂停状态
extern std::atomic<bool> g_short_pause_requested;   // 短按暂停请求
extern std::atomic<bool> g_full_shutdown_requested; // 完整关闭请求（急停/长按暂停触发）

// 恢复后模式切换相关标志（解决轴未就绪时无法切换模式的问题）
extern std::atomic<bool> g_resume_mode_switch_pending;  // 有待处理的恢复模式切换
extern std::atomic<bool> g_resume_auto_mode;            // true=自动模式, false=手动模式

// 自动模式初始化完成标志（用于业务逻辑恢复时等待轴就绪）
extern std::atomic<bool> g_auto_mode_initialized;       // 所有轴自动模式位置初始化完成

/* ============================================
 * 暂停状态记录结构 - 用于保存业务逻辑状态
 * ============================================ */
struct PauseStateRecord {
    // 入库流程状态记录
    bool warehouse_was_active = false;           // 入库是否在进行中
    int warehouse_state_value = 0;               // 入库状态机值
    int warehouse_target_layer = 1;              // 入库目标层
    
    // 出库流程状态记录
    bool outbound_was_active = false;            // 出库是否在进行中
    int outbound_state_value = 0;                // 出库状态机值
    int outbound_source_layer = 1;               // 出库源层
    
    // 恢复标记
    bool has_recorded_state = false;             // 是否已记录状态
    bool resume_requested = false;               // 恢复请求标志
    
    // 重置所有状态
    void reset() {
        warehouse_was_active = false;
        warehouse_state_value = 0;
        warehouse_target_layer = 1;
        outbound_was_active = false;
        outbound_state_value = 0;
        outbound_source_layer = 1;
        has_recorded_state = false;
        resume_requested = false;
    }
};

// 全局暂停状态记录
extern PauseStateRecord g_pause_state_record;

#endif