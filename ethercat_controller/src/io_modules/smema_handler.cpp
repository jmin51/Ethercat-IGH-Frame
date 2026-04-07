#include "smema_handler.hpp"
#include <stdio.h>
#include <string.h>

/* ============================================================
 * SMEMA 协议处理器实现 - 下游设备
 * 
 * 设计原则：
 * 1. 状态机驱动：IDLE → READY → RECEIVING → (BOARD_ARRIVED) → IDLE
 * 2. 与业务层解耦：通过标志位通知，不直接调用业务逻辑
 * 3. 超时保护：防止死锁
 * 3. 信号防抖：避免误触发
 * ============================================================ */

#if ENABLE_SMEMA  // SMEMA协议开关

// 内部状态
static struct {
    SMEMA_Config config;
    SMEMA_Status status;
    bool initialized;
    bool business_ready;        // 业务层就绪标志
    bool manual_mode;           // 手动模式标志
    bool board_received_by_business;  // 业务层已确认接收
    
    // 信号防抖
    bool uba_last;
    int uba_stable_count;
    bool mr_last;
    int mr_stable_count;
    
    // 超时检测
    time_t last_uba_on_time;
} smema_ctx = {0};

// 防抖阈值（周期数，假设100ms周期，5=500ms）
#define DEBOUNCE_COUNT  3
#define TIMEOUT_SECONDS 30

// 状态名称映射
static const char* state_names[] = {
    "IDLE",           // 空闲
    "READY",          // 就绪（MR=ON，等UBA）
    "RECEIVING",      // 接收中（UBA=ON）
    "BOARD_ARRIVED"   // 板子到达
};

/* ============================================================
 * 内部辅助函数
 * ============================================================ */

// 读取UBA信号（带防抖）
static bool read_uba_debounced(void) {
    DI_Interface di = read_all_di_signals();
    bool uba_raw = di.smema_uba;
    
    if (uba_raw == smema_ctx.uba_last) {
        if (smema_ctx.uba_stable_count < DEBOUNCE_COUNT) {
            smema_ctx.uba_stable_count++;
        }
    } else {
        smema_ctx.uba_stable_count = 0;
        smema_ctx.uba_last = uba_raw;
    }
    
    return (smema_ctx.uba_stable_count >= DEBOUNCE_COUNT) ? uba_raw : !uba_raw;
}

// 写入MR信号
static void write_mr_signal(bool state) {
    if (smema_ctx.status.mr_active != state) {
        write_single_do_signal(smema_ctx.config.do_base_address, state);
        smema_ctx.status.mr_active = state;
        printf("[SMEMA] MR信号: %s\n", state ? "ON" : "OFF");
    }
}

// 状态转换
static void transition_to_state(SMEMA_State new_state) {
    if (smema_ctx.status.state != new_state) {
        printf("[SMEMA] 状态转换: %s → %s\n", 
               state_names[smema_ctx.status.state], 
               state_names[new_state]);
        smema_ctx.status.state = new_state;
        smema_ctx.status.state_enter_time = time(NULL);
    }
}

// 检查超时
static bool is_timeout(void) {
    time_t now = time(NULL);
    return (now - smema_ctx.status.state_enter_time) > TIMEOUT_SECONDS;
}

/* ============================================================
 * 状态机处理
 * ============================================================ */

// IDLE状态处理：等待业务层就绪
static void handle_idle_state(void) {
    // MR保持OFF
    write_mr_signal(false);
    
    // 检查业务层是否就绪（缓存区空闲）
    if (smema_ctx.business_ready && !smema_ctx.manual_mode) {
        // 检查缓存区是否真的有空间
        DI_Interface di = read_all_di_signals();
        bool buffer_clear = !di.buffer_in_position && !di.conveyor_in_position;
        
        if (buffer_clear) {
            transition_to_state(SMEMA_STATE_READY);
        }
    }
}

// READY状态处理：已发MR，等待UBA
static void handle_ready_state(void) {
    // 保持MR=ON
    write_mr_signal(true);
    
    // 读取UBA
    bool uba = read_uba_debounced();
    
    if (uba) {
        // 上游有板要发
        printf("[SMEMA] 检测到UBA=ON，上游即将发板\n");
        smema_ctx.status.uba_active = true;
        smema_ctx.last_uba_on_time = time(NULL);
        transition_to_state(SMEMA_STATE_RECEIVING);
    }
    
    // 检查业务层是否取消就绪
    if (!smema_ctx.business_ready) {
        printf("[SMEMA] 业务层取消就绪，返回IDLE\n");
        transition_to_state(SMEMA_STATE_IDLE);
    }
}

// RECEIVING状态处理：板子传输中
static void handle_receiving_state(void) {
    // 保持MR=ON（传输过程中保持就绪）
    write_mr_signal(true);
    
    // 读取UBA
    bool uba = read_uba_debounced();
    
    if (!uba && smema_ctx.status.uba_active) {
        // UBA从ON变为OFF，板子传输完成
        printf("[SMEMA] 检测到UBA=OFF，板子传输完成\n");
        smema_ctx.status.uba_active = false;
        smema_ctx.status.board_count++;
        smema_ctx.board_received_by_business = false;
        transition_to_state(SMEMA_STATE_BOARD_ARRIVED);
    }
    
    // 更新UBA状态
    smema_ctx.status.uba_active = uba;
    
    // 超时检测
    if (is_timeout()) {
        printf("[SMEMA] 警告：接收超时（UBA长时间未释放）\n");
        smema_ctx.status.timeout_count++;
        // 强制返回IDLE，释放MR
        write_mr_signal(false);
        transition_to_state(SMEMA_STATE_IDLE);
    }
}

// BOARD_ARRIVED状态处理：等待业务层确认
static void handle_board_arrived_state(void) {
    // 保持MR=ON直到业务层确认
    write_mr_signal(true);
    
    // 检查业务层是否已确认接收
    if (smema_ctx.board_received_by_business) {
        printf("[SMEMA] 业务层已确认接收板子，返回IDLE\n");
        write_mr_signal(false);
        transition_to_state(SMEMA_STATE_IDLE);
    }
    
    // 超时检测（业务层处理太慢）
    if (is_timeout()) {
        printf("[SMEMA] 警告：业务层处理超时，强制返回IDLE\n");
        smema_ctx.status.timeout_count++;
        write_mr_signal(false);
        transition_to_state(SMEMA_STATE_IDLE);
    }
}

/* ============================================================
 * 公共接口实现
 * ============================================================ */

void smema_init(SMEMA_Config* config) {
    if (config == NULL) {
        // 使用默认配置
        smema_ctx.config.di_base_address = 535;
        smema_ctx.config.do_base_address = 814;
        smema_ctx.config.timeout_ms = 30000;
        smema_ctx.config.enable_ugb = false;
        smema_ctx.config.enable_ubb = false;
    } else {
        memcpy(&smema_ctx.config, config, sizeof(SMEMA_Config));
    }
    
    smema_ctx.status.state = SMEMA_STATE_IDLE;
    smema_ctx.status.mr_active = false;
    smema_ctx.status.uba_active = false;
    smema_ctx.status.state_enter_time = time(NULL);
    smema_ctx.status.board_count = 0;
    smema_ctx.status.timeout_count = 0;
    
    smema_ctx.business_ready = false;
    smema_ctx.manual_mode = false;
    smema_ctx.board_received_by_business = false;
    
    smema_ctx.uba_last = false;
    smema_ctx.uba_stable_count = 0;
    smema_ctx.mr_last = false;
    smema_ctx.mr_stable_count = 0;
    
    smema_ctx.initialized = true;
    
    // 确保MR初始为OFF
    write_single_do_signal(smema_ctx.config.do_base_address, false);
    
    printf("[SMEMA] 初始化完成，DI基址=%d, DO基址=%d\n", 
           smema_ctx.config.di_base_address, 
           smema_ctx.config.do_base_address);
}

void smema_deinit(void) {
    if (smema_ctx.initialized) {
        // 关闭MR
        write_single_do_signal(smema_ctx.config.do_base_address, false);
        smema_ctx.initialized = false;
        printf("[SMEMA] 已反初始化\n");
    }
}

void smema_process_cycle(void) {
    if (!smema_ctx.initialized) {
        return;
    }
    
    // 状态机处理
    switch (smema_ctx.status.state) {
        case SMEMA_STATE_IDLE:
            handle_idle_state();
            break;
        case SMEMA_STATE_READY:
            handle_ready_state();
            break;
        case SMEMA_STATE_RECEIVING:
            handle_receiving_state();
            break;
        case SMEMA_STATE_BOARD_ARRIVED:
            handle_board_arrived_state();
            break;
        default:
            break;
    }
}

SMEMA_State smema_get_state(void) {
    return smema_ctx.status.state;
}

const char* smema_state_to_string(SMEMA_State state) {
    if (state >= 0 && state < sizeof(state_names)/sizeof(state_names[0])) {
        return state_names[state];
    }
    return "UNKNOWN";
}

SMEMA_Status smema_get_status(void) {
    return smema_ctx.status;
}

void smema_set_mr_manual(bool state) {
    smema_ctx.manual_mode = true;
    write_mr_signal(state);
    printf("[SMEMA] 手动设置MR=%s\n", state ? "ON" : "OFF");
}

void smema_reset(void) {
    write_mr_signal(false);
    smema_ctx.status.uba_active = false;
    smema_ctx.manual_mode = false;
    smema_ctx.board_received_by_business = false;
    transition_to_state(SMEMA_STATE_IDLE);
    printf("[SMEMA] 状态机已复位\n");
}

bool smema_can_receive_board(void) {
    // 只有在IDLE状态且业务层就绪时才能接收新板子
    return (smema_ctx.status.state == SMEMA_STATE_IDLE) && 
           smema_ctx.business_ready &&
           !smema_ctx.manual_mode;
}

void smema_confirm_board_received(void) {
    smema_ctx.board_received_by_business = true;
    printf("[SMEMA] 业务层确认板子已接收\n");
}

int smema_get_board_count(void) {
    return smema_ctx.status.board_count;
}

void smema_set_business_ready(bool ready) {
    smema_ctx.business_ready = ready;
    if (ready) {
        printf("[SMEMA] 业务层就绪，可以接收板子\n");
    } else {
        printf("[SMEMA] 业务层未就绪\n");
    }
}

bool smema_is_business_ready(void) {
    return smema_ctx.business_ready;
}

#else  // SMEMA禁用时的空实现

void smema_init(SMEMA_Config* config) {
    (void)config;
    printf("[SMEMA] SMEMA协议已禁用\n");
}

void smema_deinit(void) {
    // 空实现
}

void smema_process_cycle(void) {
    // 空实现
}

SMEMA_State smema_get_state(void) {
    return SMEMA_STATE_IDLE;
}

const char* smema_state_to_string(SMEMA_State state) {
    (void)state;
    return "DISABLED";
}

SMEMA_Status smema_get_status(void) {
    SMEMA_Status status;
    status.state = SMEMA_STATE_IDLE;
    status.mr_active = false;
    status.uba_active = false;
    status.state_enter_time = 0;
    status.board_count = 0;
    status.timeout_count = 0;
    return status;
}

void smema_set_mr_manual(bool state) {
    (void)state;
    printf("[SMEMA] SMEMA协议已禁用，无法设置MR\n");
}

void smema_reset(void) {
    printf("[SMEMA] SMEMA协议已禁用\n");
}

bool smema_can_receive_board(void) {
    return false;
}

void smema_confirm_board_received(void) {
    printf("[SMEMA] SMEMA协议已禁用\n");
}

int smema_get_board_count(void) {
    return 0;
}

void smema_set_business_ready(bool ready) {
    (void)ready;
    // 空实现
}

bool smema_is_business_ready(void) {
    return false;
}

#endif  // ENABLE_SMEMA
