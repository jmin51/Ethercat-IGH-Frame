#include "smema_handler.hpp"
#include <stdio.h>
#include <string.h>

/* ============================================================
 * SMEMA 协议处理器实现 - 中游设备双向握手
 * 
 * 设计原则：
 * 1. 双状态机并行：上游握手 + 下游握手独立运行
 * 2. 产品到位驱动：product_in_position控制握手方向
 * 3. 信号滤波：50ms防抖避免误触发
 * 4. 超时保护：防止死锁
 * ============================================================ */

#if ENABLE_SMEMA  // SMEMA协议开关

// ==================== 内部状态上下文 ====================
static struct {
    SMEMA_Config config;
    SMEMA_Status status;
    bool initialized;
    bool enabled;                        // 协议启用标志（可屏蔽）
    
    // 产品到位信号（业务层驱动）
    bool product_in_position;            // true=本机有板，false=本机无板
    
    // 业务层确认标志（已废弃，改用product_in_position控制）
    // bool board_received_confirmed;
    // bool board_sent_confirmed;
    
    // 手动模式
    bool manual_mode_mr;                 // 手动控制MR
    bool manual_mode_ba;                 // 手动控制BA
    
    // 信号滤波（50ms防抖）
    bool uba_last;
    int uba_stable_count;
    bool dbr_last;
    int dbr_stable_count;
    
    // 超时检测
    time_t upstream_handshake_start;     // 上游握手开始时间
    time_t downstream_handshake_start;   // 下游握手开始时间
} ctx = {0};

// ==================== 常量定义 ====================
#define DEBOUNCE_THRESHOLD_MS    50     // 信号滤波阈值（毫秒）
#define DEFAULT_CYCLE_MS         100    // 默认处理周期（毫秒）
#define RECEIVE_TIMEOUT_S        30     // 接收超时（秒）
#define SEND_TIMEOUT_S           30     // 发送超时（秒）

// 状态名称映射
static const char* upstream_state_names[] = {
    "UPSTREAM_IDLE",           // 空闲
    "UPSTREAM_READY",          // 就绪（MR=ON，等UBA）
    "UPSTREAM_RECEIVING",      // 接收中（UBA=ON）
    "UPSTREAM_BOARD_ARRIVED"   // 板子到达
};

static const char* downstream_state_names[] = {
    "DOWNSTREAM_IDLE",         // 空闲
    "DOWNSTREAM_AVAILABLE",    // 有板待发（BA=ON，等DBR）
    "DOWNSTREAM_SENDING",      // 发送中（DBR=ON）
    "DOWNSTREAM_SENT"          // 板子已发
};

/* ============================================================
 * 内部辅助函数
 * ============================================================ */

// 读取UBA信号（带防抖）
static bool read_uba_debounced(void) {
    DI_Interface di = read_all_di_signals();
    bool uba_raw = di.smema_uba;
    
    if (uba_raw == ctx.uba_last) {
        if (ctx.uba_stable_count < DEBOUNCE_THRESHOLD_MS / DEFAULT_CYCLE_MS) {
            ctx.uba_stable_count++;
        }
    } else {
        ctx.uba_stable_count = 0;
        ctx.uba_last = uba_raw;
    }
    
    return (ctx.uba_stable_count >= DEBOUNCE_THRESHOLD_MS / DEFAULT_CYCLE_MS) ? uba_raw : !uba_raw;
}

// 读取DBR信号（带防抖）
static bool read_dbr_debounced(void) {
    DI_Interface di = read_all_di_signals();
    bool dbr_raw = di.smema_dbr;
    
    if (dbr_raw == ctx.dbr_last) {
        if (ctx.dbr_stable_count < DEBOUNCE_THRESHOLD_MS / DEFAULT_CYCLE_MS) {
            ctx.dbr_stable_count++;
        }
    } else {
        ctx.dbr_stable_count = 0;
        ctx.dbr_last = dbr_raw;
    }
    
    return (ctx.dbr_stable_count >= DEBOUNCE_THRESHOLD_MS / DEFAULT_CYCLE_MS) ? dbr_raw : !dbr_raw;
}

// 写入MR信号（本机要板）
static void write_mr_signal(bool state) {
    if (ctx.status.mr_active != state) {
        write_single_do_signal(ctx.config.do_base_address, state);
        ctx.status.mr_active = state;
        printf("[SMEMA上游] MR信号: %s\n", state ? "ON(要板)" : "OFF");
    }
}

// 写入BA信号（本机有板）
static void write_ba_signal(bool state) {
    if (ctx.status.ba_active != state) {
        write_single_do_signal(ctx.config.do_base_address + 1, state);
        ctx.status.ba_active = state;
        printf("[SMEMA下游] BA信号: %s\n", state ? "ON(有板)" : "OFF");
    }
}

// 上游状态转换
static void transition_upstream_state(Upstream_State new_state) {
    if (ctx.status.upstream_state != new_state) {
        printf("[SMEMA上游] 状态转换: %s → %s\n", 
               upstream_state_names[ctx.status.upstream_state], 
               upstream_state_names[new_state]);
        ctx.status.upstream_state = new_state;
        ctx.status.upstream_state_time = time(NULL);
    }
}

// 下游状态转换
static void transition_downstream_state(Downstream_State new_state) {
    if (ctx.status.downstream_state != new_state) {
        printf("[SMEMA下游] 状态转换: %s → %s\n", 
               downstream_state_names[ctx.status.downstream_state], 
               downstream_state_names[new_state]);
        ctx.status.downstream_state = new_state;
        ctx.status.downstream_state_time = time(NULL);
    }
}

// 检查上游超时
static bool is_upstream_timeout(void) {
    time_t now = time(NULL);
    return (now - ctx.status.upstream_state_time) > RECEIVE_TIMEOUT_S;
}

// 检查下游超时
static bool is_downstream_timeout(void) {
    time_t now = time(NULL);
    return (now - ctx.status.downstream_state_time) > SEND_TIMEOUT_S;
}

/* ============================================================
 * 上游握手状态机（接收板子）
 * ============================================================ */

// UPSTREAM_IDLE：等待本机无板
static void handle_upstream_idle(void) {
    // MR保持OFF
    write_mr_signal(false);
    
    // 检查产品到位信号（无板 = 要板）
    if (!ctx.product_in_position && ctx.enabled && !ctx.manual_mode_mr) {
        printf("[SMEMA上游] 本机无板，发出要板信号\n");
        transition_upstream_state(UPSTREAM_READY);
    }
}

// UPSTREAM_READY：已发MR，等待UBA
static void handle_upstream_ready(void) {
    // 保持MR=ON
    write_mr_signal(true);
    
    // 读取UBA
    bool uba = read_uba_debounced();
    ctx.status.uba_active = uba;
    
    if (uba) {
        // 上游有板要发
        printf("[SMEMA上游] 检测到UBA=ON，上游即将发板\n");
        ctx.upstream_handshake_start = time(NULL);
        transition_upstream_state(UPSTREAM_RECEIVING);
    }
    
    // 检查产品到位信号变化（中途有板了）
    if (ctx.product_in_position) {
        printf("[SMEMA上游] 本机已有板，取消要板\n");
        transition_upstream_state(UPSTREAM_IDLE);
    }
    
    // // 超时检测
    // if (is_upstream_timeout()) {
    //     printf("[SMEMA上游] 警告：等待上游发板超时\n");
    //     ctx.status.receive_timeout_count++;
    //     transition_upstream_state(UPSTREAM_IDLE);
    // }
}

// UPSTREAM_RECEIVING：板子传输中
static void handle_upstream_receiving(void) {
    // 保持MR=ON（传输过程中保持就绪）
    write_mr_signal(true);
    
    // 读取UBA
    bool uba = read_uba_debounced();
    ctx.status.uba_active = uba;
    
    if (!uba) {
        // UBA从ON变为OFF，板子传输完成
        printf("[SMEMA上游] 检测到UBA=OFF，板子传输完成\n");
        ctx.status.receive_count++;
        transition_upstream_state(UPSTREAM_BOARD_ARRIVED);
    }
    
    // 超时检测
    if (is_upstream_timeout()) {
        printf("[SMEMA上游] 警告：接收超时（UBA长时间未释放）\n");
        ctx.status.receive_timeout_count++;
        transition_upstream_state(UPSTREAM_IDLE);
    }
}

// UPSTREAM_BOARD_ARRIVED：等待产品到位
static void handle_upstream_board_arrived(void) {
    // 保持MR=ON直到产品到位
    write_mr_signal(true);
    
    // 检查产品是否到位（业务层通过product_in_position信号控制）
    if (ctx.product_in_position) {
        printf("[SMEMA上游] 产品已到位，返回IDLE\n");
        write_mr_signal(false);
        transition_upstream_state(UPSTREAM_IDLE);
    }
    
    // 超时检测（业务层处理太慢）
    if (is_upstream_timeout()) {
        printf("[SMEMA上游] 警告：等待产品到位超时\n");
        ctx.status.receive_timeout_count++;
        write_mr_signal(false);
        transition_upstream_state(UPSTREAM_IDLE);
    }
}

/* ============================================================
 * 下游握手状态机（发送板子）
 * ============================================================ */

// DOWNSTREAM_IDLE：等待本机有板
static void handle_downstream_idle(void) {
    // BA保持OFF
    write_ba_signal(false);
    
    // 检查产品到位信号（有板 = 发板）
    if (ctx.product_in_position && ctx.enabled && !ctx.manual_mode_ba) {
        printf("[SMEMA下游] 本机有板，发出有板信号\n");
        transition_downstream_state(DOWNSTREAM_AVAILABLE);
    }
}

// DOWNSTREAM_AVAILABLE：已发BA，等待DBR
static void handle_downstream_available(void) {
    // 保持BA=ON
    write_ba_signal(true);
    
    // 读取DBR
    bool dbr = read_dbr_debounced();
    ctx.status.dbr_active = dbr;
    
    if (dbr) {
        // 下游要板
        printf("[SMEMA下游] 检测到DBR=ON，下游要板，开始发板\n");
        ctx.downstream_handshake_start = time(NULL);
        transition_downstream_state(DOWNSTREAM_SENDING);
    }
    
    // 检查产品到位信号变化（中途无板了）
    if (!ctx.product_in_position) {
        printf("[SMEMA下游] 本机已无板，取消有板信号\n");
        transition_downstream_state(DOWNSTREAM_IDLE);
    }
    
    // 注释掉超时检测：下游设备可能暂时不需要板子，BA应保持ON等待
    // 超时会导致振荡循环：AVAILABLE → IDLE → AVAILABLE → ...
    // if (is_downstream_timeout()) {
    //     printf("[SMEMA下游] 警告：等待下游要板超时\n");
    //     ctx.status.send_timeout_count++;
    //     transition_downstream_state(DOWNSTREAM_IDLE);
    // }
}

// DOWNSTREAM_SENDING：板子传输中
static void handle_downstream_sending(void) {
    // 保持BA=ON（传输过程中保持有板）
    write_ba_signal(true);
    
    // 读取DBR
    bool dbr = read_dbr_debounced();
    ctx.status.dbr_active = dbr;
    
    if (!dbr) {
        // DBR从ON变为OFF，板子传输完成
        printf("[SMEMA下游] 检测到DBR=OFF，板子传输完成\n");
        ctx.status.send_count++;
        transition_downstream_state(DOWNSTREAM_SENT);
    }
    
    // 注释掉超时检测：下游设备可能接收慢，BA应保持ON等待
    // 业务层负责启动皮带发送板子，超时应由业务层处理
    // if (is_downstream_timeout()) {
    //     printf("[SMEMA下游] 警告：发送超时（DBR长时间未释放）\n");
    //     ctx.status.send_timeout_count++;
    //     transition_downstream_state(DOWNSTREAM_IDLE);
    // }
}

// DOWNSTREAM_SENT：等待产品离开
static void handle_downstream_sent(void) {
    // 保持BA=ON直到产品离开
    write_ba_signal(true);
    
    // 检查产品是否离开（业务层通过product_in_position信号控制）
    if (!ctx.product_in_position) {
        printf("[SMEMA下游] 产品已离开，返回IDLE\n");
        write_ba_signal(false);
        transition_downstream_state(DOWNSTREAM_IDLE);
    }
    
    // 注释掉超时检测：业务层负责移除板子，超时应由业务层处理
    // if (is_downstream_timeout()) {
    //     printf("[SMEMA下游] 警告：等待产品离开超时\n");
    //     ctx.status.send_timeout_count++;
    //     write_ba_signal(false);
    //     transition_downstream_state(DOWNSTREAM_IDLE);
    // }
}

/* ============================================================
 * 公共接口实现
 * ============================================================ */

void smema_init(SMEMA_Config* config) {
    if (config == NULL) {
        // 使用默认配置
        ctx.config.di_base_address = 535;
        ctx.config.do_base_address = 814;
        ctx.config.handshake_filter_ms = DEBOUNCE_THRESHOLD_MS;
        ctx.config.receive_timeout_ms = RECEIVE_TIMEOUT_S * 1000;
        ctx.config.send_timeout_ms = SEND_TIMEOUT_S * 1000;
    } else {
        memcpy(&ctx.config, config, sizeof(SMEMA_Config));
    }
    
    // 初始化状态
    ctx.status.upstream_state = UPSTREAM_IDLE;
    ctx.status.downstream_state = DOWNSTREAM_IDLE;
    ctx.status.mr_active = false;
    ctx.status.uba_active = false;
    ctx.status.ba_active = false;
    ctx.status.dbr_active = false;
    ctx.status.upstream_state_time = time(NULL);
    ctx.status.downstream_state_time = time(NULL);
    ctx.status.receive_count = 0;
    ctx.status.send_count = 0;
    ctx.status.receive_timeout_count = 0;
    ctx.status.send_timeout_count = 0;
    
    // 初始化内部状态
    ctx.product_in_position = false;
    ctx.manual_mode_mr = false;
    ctx.manual_mode_ba = false;
    ctx.enabled = true;
    ctx.initialized = true;
    
    // 确保初始信号为OFF
    write_single_do_signal(ctx.config.do_base_address, false);      // MR
    write_single_do_signal(ctx.config.do_base_address + 1, false);  // BA
    
    printf("[SMEMA] 双向握手初始化完成\n");
    printf("  - 上游握手: DI=%d(UBA), DO=%d(MR)\n", 
           ctx.config.di_base_address, ctx.config.do_base_address);
    printf("  - 下游握手: DI=%d(DBR), DO=%d(BA)\n", 
           ctx.config.di_base_address + 3, ctx.config.do_base_address + 1);
}

void smema_deinit(void) {
    if (ctx.initialized) {
        // 关闭所有信号
        write_single_do_signal(ctx.config.do_base_address, false);      // MR
        write_single_do_signal(ctx.config.do_base_address + 1, false);  // BA
        ctx.initialized = false;
        printf("[SMEMA] 已反初始化\n");
    }
}

void smema_process_cycle(void) {
    if (!ctx.initialized || !ctx.enabled) {
        return;
    }
    
    // 上游握手状态机
    switch (ctx.status.upstream_state) {
        case UPSTREAM_IDLE:
            handle_upstream_idle();
            break;
        case UPSTREAM_READY:
            handle_upstream_ready();
            break;
        case UPSTREAM_RECEIVING:
            handle_upstream_receiving();
            break;
        case UPSTREAM_BOARD_ARRIVED:
            handle_upstream_board_arrived();
            break;
        default:
            break;
    }
    
    // 下游握手状态机
    switch (ctx.status.downstream_state) {
        case DOWNSTREAM_IDLE:
            handle_downstream_idle();
            break;
        case DOWNSTREAM_AVAILABLE:
            handle_downstream_available();
            break;
        case DOWNSTREAM_SENDING:
            handle_downstream_sending();
            break;
        case DOWNSTREAM_SENT:
            handle_downstream_sent();
            break;
        default:
            break;
    }
}

Upstream_State smema_get_upstream_state(void) {
    return ctx.status.upstream_state;
}

Downstream_State smema_get_downstream_state(void) {
    return ctx.status.downstream_state;
}

SMEMA_Status smema_get_status(void) {
    return ctx.status;
}

const char* smema_upstream_state_to_string(Upstream_State state) {
    if (state >= 0 && state < sizeof(upstream_state_names)/sizeof(upstream_state_names[0])) {
        return upstream_state_names[state];
    }
    return "UNKNOWN";
}

const char* smema_downstream_state_to_string(Downstream_State state) {
    if (state >= 0 && state < sizeof(downstream_state_names)/sizeof(downstream_state_names[0])) {
        return downstream_state_names[state];
    }
    return "UNKNOWN";
}

// ==================== 业务层接口 ====================

void smema_set_product_in_position(bool in_position) {
    if (ctx.product_in_position != in_position) {
        ctx.product_in_position = in_position;
        printf("[SMEMA] 产品到位信号: %s\n", in_position ? "有板" : "无板");
    }
}

bool smema_can_receive_board(void) {
    return (ctx.status.upstream_state == UPSTREAM_BOARD_ARRIVED);
}

bool smema_can_send_board(void) {
    return (ctx.status.downstream_state == DOWNSTREAM_SENT);
}

// ==================== 手动控制 ====================

void smema_set_mr_manual(bool state) {
    ctx.manual_mode_mr = true;
    write_mr_signal(state);
    printf("[SMEMA上游] 手动设置MR=%s\n", state ? "ON" : "OFF");
}

void smema_set_ba_manual(bool state) {
    ctx.manual_mode_ba = true;
    write_ba_signal(state);
    printf("[SMEMA下游] 手动设置BA=%s\n", state ? "ON" : "OFF");
}

// ==================== 复位与统计 ====================

void smema_reset(void) {
    write_mr_signal(false);
    write_ba_signal(false);
    ctx.status.uba_active = false;
    ctx.status.dbr_active = false;
    ctx.manual_mode_mr = false;
    ctx.manual_mode_ba = false;
    transition_upstream_state(UPSTREAM_IDLE);
    transition_downstream_state(DOWNSTREAM_IDLE);
    printf("[SMEMA] 状态机已复位\n");
}

int smema_get_receive_count(void) {
    return ctx.status.receive_count;
}

int smema_get_send_count(void) {
    return ctx.status.send_count;
}

// ==================== 屏蔽控制 ====================

void smema_set_enabled(bool enabled) {
    ctx.enabled = enabled;
    if (!enabled) {
        // 屏蔽时关闭所有信号
        write_mr_signal(false);
        write_ba_signal(false);
        printf("[SMEMA] 协议已屏蔽\n");
    } else {
        printf("[SMEMA] 协议已启用\n");
    }
}

bool smema_is_enabled(void) {
    return ctx.enabled;
}

#else  // SMEMA禁用时的空实现

void smema_init(SMEMA_Config* config) {
    (void)config;
    printf("[SMEMA] SMEMA协议已禁用\n");
}

void smema_deinit(void) {}

void smema_process_cycle(void) {}

Upstream_State smema_get_upstream_state(void) {
    return UPSTREAM_IDLE;
}

Downstream_State smema_get_downstream_state(void) {
    return DOWNSTREAM_IDLE;
}

SMEMA_Status smema_get_status(void) {
    SMEMA_Status status;
    memset(&status, 0, sizeof(SMEMA_Status));
    status.upstream_state = UPSTREAM_IDLE;
    status.downstream_state = DOWNSTREAM_IDLE;
    return status;
}

const char* smema_upstream_state_to_string(Upstream_State state) {
    (void)state;
    return "DISABLED";
}

const char* smema_downstream_state_to_string(Downstream_State state) {
    (void)state;
    return "DISABLED";
}

void smema_set_product_in_position(bool in_position) {
    (void)in_position;
}

bool smema_can_receive_board(void) {
    return false;
}

bool smema_can_send_board(void) {
    return false;
}

void smema_set_mr_manual(bool state) {
    (void)state;
}

void smema_set_ba_manual(bool state) {
    (void)state;
}

void smema_reset(void) {}

int smema_get_receive_count(void) {
    return 0;
}

int smema_get_send_count(void) {
    return 0;
}

void smema_set_enabled(bool enabled) {
    (void)enabled;
}

bool smema_is_enabled(void) {
    return false;
}

#endif  // ENABLE_SMEMA
