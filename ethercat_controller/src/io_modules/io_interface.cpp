#include "io_interface.hpp"
#include <stdio.h>
#include <errno.h>
#include <modbus/modbus.h>
#include <cstring>  // 添加这行，用于 memcmp 函数
#include <time.h>
#include <mutex>  // 添加互斥锁支持

static modbus_t *ctx_di = NULL;  // DI设备连接
#if ENABLE_DO_MODULE  // 条件编译
static modbus_t *ctx_do = NULL;  // DO设备连接
#endif
static DO_Interface current_do_state = {0};

// Modbus访问互斥锁（防止多线程同时访问）
static std::mutex modbus_di_mutex;
static std::mutex modbus_do_mutex;

/* ============================================================
 * 断线重连机制 - 连接配置与状态管理
 * ============================================================ */

// 连接配置存储
static struct {
    char ip[32];
    int port;
    int slave_id;
} di_config = {0}, do_config = {0};

// 连接状态与重连控制
static struct {
    time_t last_reconnect_time;  // 上次重连时间
    bool is_connected;           // 连接状态标记
} di_status = {0, false}, do_status = {0, false};

#define RECONNECT_INTERVAL_SECONDS  5   // 重连间隔(秒)
#define RECONNECT_MAX_RETRY         3   // 单次最大重试次数

// 前置声明
static bool try_reconnect_di(void);
static bool try_reconnect_do(void);
static void close_di_connection(void);
static void close_do_connection(void);

// 函数前置声明
#if ENABLE_DO_MODULE
static void refresh_do_state_from_device(void);
#endif

// 从设备刷新DO状态（内部函数）
#if ENABLE_DO_MODULE
static void refresh_do_state_from_device() {
    std::lock_guard<std::mutex> lock(modbus_do_mutex);
    
    // 连接断开时尝试重连
    if (!ctx_do || !do_status.is_connected) {
        // 注意：此处不能调用try_reconnect_do，因为已持有锁会导致死锁
        // 重连逻辑由调用方处理或在写入时触发
        return;
    }
    
    uint8_t do_values[16];
    if (modbus_read_bits(ctx_do, 0, 16, do_values) == 16) {
        current_do_state.start_button_light = do_values[0];
        current_do_state.reset_button_light = do_values[1];
        current_do_state.pause_button_light = do_values[2];
        current_do_state.buzzer = do_values[3];
        current_do_state.red_light = do_values[4];
        current_do_state.yellow_light = do_values[5];
        current_do_state.green_light = do_values[6];
        current_do_state.lift_cylinder_down = do_values[10];
        current_do_state.gear_cylinder_extend = do_values[11];
        current_do_state.belt_forward = do_values[12];
        current_do_state.belt_backward = do_values[13];  // 皮带反转状态读取
        current_do_state.smema_mr = do_values[14];       // SMEMA机器就绪
        // printf("DO状态已从设备刷新\n");  // 调试时启用，正式运行关闭
    } else {
        // 读取失败，关闭连接触发重连
        close_do_connection();
        static int err_cnt = 0;
        if (err_cnt++ % 100 == 0) {
            fprintf(stderr, "刷新DO状态失败: %s (已抑制%d次)\n", 
                    modbus_strerror(errno), err_cnt);
        }
    }
}
#endif

// Modbus 初始化 - 根据宏开关条件连接设备
int init_modbus_interface(const char* di_ip, int di_port, int di_slave_id,
                         const char* do_ip, int do_port, int do_slave_id) {
    
    int result = 0;
    
    // 保存DI配置用于重连
    if (di_ip != NULL) {
        strncpy(di_config.ip, di_ip, sizeof(di_config.ip) - 1);
        di_config.ip[sizeof(di_config.ip) - 1] = '\0';
        di_config.port = di_port;
        di_config.slave_id = di_slave_id;
    }
    
    // 保存DO配置用于重连
    if (do_ip != NULL) {
        strncpy(do_config.ip, do_ip, sizeof(do_config.ip) - 1);
        do_config.ip[sizeof(do_config.ip) - 1] = '\0';
        do_config.port = do_port;
        do_config.slave_id = do_slave_id;
    }
    
    // 初始化DI连接（如果启用）
#if ENABLE_DI_MODULE
    if (di_ip != NULL) {
        ctx_di = modbus_new_tcp(di_ip, di_port);
        if (ctx_di == NULL) {
            fprintf(stderr, "无法创建 DI Modbus 上下文\n");
            di_status.is_connected = false;
            result = -1;
        } else {
            modbus_set_response_timeout(ctx_di, 1, 0);
            modbus_set_slave(ctx_di, di_slave_id);
            
            if (modbus_connect(ctx_di) == -1) {
                fprintf(stderr, "DI连接失败: %s\n", modbus_strerror(errno));
                modbus_free(ctx_di);
                ctx_di = NULL;
                di_status.is_connected = false;
                result = -1;
            } else {
                printf("DI设备连接成功: %s:%d (从站ID: %d)\n", di_ip, di_port, di_slave_id);
                di_status.is_connected = true;
                di_status.last_reconnect_time = time(NULL);
            }
        }
    }
#else
    printf("DI模块已禁用，跳过DI设备连接\n");
#endif
    
    // 初始化DO连接（如果启用）
#if ENABLE_DO_MODULE
    if (do_ip != NULL && result == 0) {
        ctx_do = modbus_new_tcp(do_ip, do_port);
        if (ctx_do == NULL) {
            fprintf(stderr, "无法创建 DO Modbus 上下文\n");
            do_status.is_connected = false;
            result = -1;
        } else {
            modbus_set_response_timeout(ctx_do, 1, 0);
            modbus_set_slave(ctx_do, do_slave_id);
            
            if (modbus_connect(ctx_do) == -1) {
                fprintf(stderr, "DO连接失败: %s\n", modbus_strerror(errno));
                modbus_free(ctx_do);
                ctx_do = NULL;
                do_status.is_connected = false;
                result = -1;
            } else {
                printf("DO设备连接成功: %s:%d (从站ID: %d)\n", do_ip, do_port, do_slave_id);
                do_status.is_connected = true;
                do_status.last_reconnect_time = time(NULL);
                
                // 初始化时读取当前DO状态
                refresh_do_state_from_device();
            }
        }
    }
#else
    printf("DO模块已禁用，跳过DO设备连接\n");
#endif
    
    if (result == 0) {
        printf("Modbus连接初始化完成\n");
    }
    
    return result;
}

void cleanup_modbus_interface() {
#if ENABLE_DI_MODULE
    if (ctx_di) {
        modbus_close(ctx_di);
        modbus_free(ctx_di);
        ctx_di = NULL;
        di_status.is_connected = false;
        printf("DI连接已关闭\n");
    }
#endif
    
#if ENABLE_DO_MODULE
    if (ctx_do) {
        modbus_close(ctx_do);
        modbus_free(ctx_do);
        ctx_do = NULL;
        do_status.is_connected = false;
        printf("DO连接已关闭\n");
    }
#endif
}

/* ============================================================
 * 断线重连实现
 * ============================================================ */

// 关闭DI连接并清理状态
static void close_di_connection(void) {
    if (ctx_di) {
        modbus_close(ctx_di);
        modbus_free(ctx_di);
        ctx_di = NULL;
    }
    di_status.is_connected = false;
}

// 关闭DO连接并清理状态
static void close_do_connection(void) {
    if (ctx_do) {
        modbus_close(ctx_do);
        modbus_free(ctx_do);
        ctx_do = NULL;
    }
    do_status.is_connected = false;
}

// 尝试重连DI设备
static bool try_reconnect_di(void) {
    // 检查重连间隔
    time_t current_time = time(NULL);
    if (current_time - di_status.last_reconnect_time < RECONNECT_INTERVAL_SECONDS) {
        return false;  // 冷却期内，不重连
    }
    
    di_status.last_reconnect_time = current_time;
    
    // 先关闭旧连接
    close_di_connection();
    
    // 创建新连接
    ctx_di = modbus_new_tcp(di_config.ip, di_config.port);
    if (ctx_di == NULL) {
        static int err_cnt = 0;
        if (err_cnt++ % 10 == 0) {
            fprintf(stderr, "DI重连: 无法创建Modbus上下文 (已抑制%d次)\n", err_cnt);
        }
        return false;
    }
    
    modbus_set_response_timeout(ctx_di, 1, 0);
    modbus_set_slave(ctx_di, di_config.slave_id);
    
    if (modbus_connect(ctx_di) == -1) {
        static int err_cnt = 0;
        if (err_cnt++ % 10 == 0) {
            fprintf(stderr, "DI重连失败: %s (已抑制%d次)\n", 
                    modbus_strerror(errno), err_cnt);
        }
        modbus_free(ctx_di);
        ctx_di = NULL;
        return false;
    }
    
    printf("DI设备重连成功: %s:%d\n", di_config.ip, di_config.port);
    di_status.is_connected = true;
    return true;
}

// 尝试重连DO设备
static bool try_reconnect_do(void) {
    // 检查重连间隔
    time_t current_time = time(NULL);
    if (current_time - do_status.last_reconnect_time < RECONNECT_INTERVAL_SECONDS) {
        return false;  // 冷却期内，不重连
    }
    
    do_status.last_reconnect_time = current_time;
    
    // 先关闭旧连接
    close_do_connection();
    
    // 创建新连接
    ctx_do = modbus_new_tcp(do_config.ip, do_config.port);
    if (ctx_do == NULL) {
        static int err_cnt = 0;
        if (err_cnt++ % 10 == 0) {
            fprintf(stderr, "DO重连: 无法创建Modbus上下文 (已抑制%d次)\n", err_cnt);
        }
        return false;
    }
    
    modbus_set_response_timeout(ctx_do, 1, 0);
    modbus_set_slave(ctx_do, do_config.slave_id);
    
    if (modbus_connect(ctx_do) == -1) {
        static int err_cnt = 0;
        if (err_cnt++ % 10 == 0) {
            fprintf(stderr, "DO重连失败: %s (已抑制%d次)\n", 
                    modbus_strerror(errno), err_cnt);
        }
        modbus_free(ctx_do);
        ctx_do = NULL;
        return false;
    }
    
    printf("DO设备重连成功: %s:%d\n", do_config.ip, do_config.port);
    do_status.is_connected = true;
    return true;
}

// DI 信号读取接口
#if ENABLE_DI_MODULE
DI_Interface read_all_di_signals() {
    DI_Interface di = {0};
    
    std::lock_guard<std::mutex> lock(modbus_di_mutex);
    
    // 连接断开时尝试重连
    if (!ctx_di || !di_status.is_connected) {
        try_reconnect_di();
        // 重连后仍无效，返回空数据
        if (!ctx_di) {
            return di;
        }
    }
    
    uint8_t di_values[48];
    int rc = modbus_read_input_bits(ctx_di, 0, 48, di_values);
    
    if (rc == -1) {
        // 读取失败：关闭连接触发下次重连
        close_di_connection();
        
        // 减少日志刷屏：仅在非连续错误时打印
        static int error_count = 0;
        if (error_count++ % 100 == 0) {
            fprintf(stderr, "读取DI失败: %s (已抑制%d次重复错误)\n", 
                    modbus_strerror(errno), error_count);
        }
        return di;
    } else {
        // 成功时重置错误计数（用于失败计数抑制）
        // 注意：此处error_count与失败分支的error_count是不同的静态变量
        // 成功分支的error_count仅用于重置，不需要使用
        static int success_error_count = 0;
        success_error_count = 0;
        (void)success_error_count;  // 避免未使用警告
    }
    
    // 映射到结构体
    di.start_button = di_values[0];          // M512
    di.reset_button = di_values[1];          // M513
    di.pause_button = di_values[2];          // M514
    di.manual_auto_button = di_values[3];    // M515
    di.emergency_stop = di_values[4];         // M516
    di.air_supply = di_values[5];             // M517
    di.safety_door_1 = di_values[6];          // M518
    di.safety_door_2 = di_values[7];          // M519
    di.feed_product_detect = di_values[8];    // M520
    di.buffer_sensor_1 = di_values[9];       // M521
    di.buffer_sensor_2 = di_values[10];      // M522
    di.buffer_in_position = di_values[11];   // M523
    di.buffer_out_position = di_values[12];  // M524
    di.conveyor_in_position = di_values[13]; // M525
    di.conveyor_out_position = di_values[14];// M526
    di.lift_cylinder1_up = di_values[15];    // M527
    di.lift_cylinder1_down = di_values[16]; // M528
    di.lift_cylinder2_up = di_values[17];    // M529
    di.lift_cylinder2_down = di_values[18];  // M530
    di.gear_cylinder1_extend = di_values[19];// M531
    di.gear_cylinder1_retract = di_values[20];// M532
    di.gear_cylinder2_extend = di_values[21];// M533
    di.gear_cylinder2_retract = di_values[22];// M534
    // SMEMA协议信号
#if ENABLE_SMEMA
    di.smema_uba = di_values[23];            // M535 上游有板待发
    di.smema_dbr_test = di_values[25];              // M537 下游要板测试信号
    di.smema_dbr = di_values[24] | di_values[25];  // M536 下游要板 = 主信号 | 测试信号(OR逻辑)
    di.conveyor_exit_gap_detect = di_values[28];    // M540 接驳台出料检测(缝隙)
    di.conveyor_entry_gap_detect = di_values[29];   // M541 接驳台入料检测(缝隙)
#else
    // SMEMA禁用时提供默认值，方便调试
    di.smema_uba = false;                    // 默认无板
    di.smema_dbr = true;                     // 默认要板，方便调试
#endif
    
    return di;
}

bool read_single_di_signal(int di_address) {
    std::lock_guard<std::mutex> lock(modbus_di_mutex);
    
    // 连接断开时尝试重连
    if (!ctx_di || !di_status.is_connected) {
        try_reconnect_di();
        if (!ctx_di) {
            static int err_cnt = 0;
            if (err_cnt++ % 100 == 0) {
                fprintf(stderr, "DI未连接，重连失败，无法读取单信号 (已抑制%d次)\n", err_cnt);
            }
            return false;
        }
    }
    
    // 地址有效性检查
    if (di_address < 512 || di_address > 541) {
        fprintf(stderr, "DI地址超出范围: %d\n", di_address);
        return false;
    }
    
    uint8_t value;
    int rc = modbus_read_input_bits(ctx_di, di_address - 512, 1, &value);
    
    if (rc != 1) {
        // 读取失败，关闭连接触发重连
        close_di_connection();
        static int err_cnt = 0;
        if (err_cnt++ % 100 == 0) {
            fprintf(stderr, "读取单DI信号失败: %s (已抑制%d次)\n", 
                    modbus_strerror(errno), err_cnt);
        }
        return false;
    }
    
    return value;
}
#else
// DI模块禁用时的空实现
DI_Interface read_all_di_signals() {
    DI_Interface di = {0};
    // fprintf(stderr, "警告: DI模块已禁用，返回空数据\n");
    return di;
}

bool read_single_di_signal(int di_address) {
    // fprintf(stderr, "警告: DI模块已禁用，无法读取信号\n");
    return false;
}
#endif

// DO 信号写入接口
#if ENABLE_DO_MODULE
int write_do_signals(DO_Interface do_signals) {
    std::lock_guard<std::mutex> lock(modbus_do_mutex);
    
    // 连接断开时尝试重连
    if (!ctx_do || !do_status.is_connected) {
        try_reconnect_do();
        if (!ctx_do) {
            static int err_cnt = 0;
            if (err_cnt++ % 100 == 0) {
                fprintf(stderr, "DO未连接，重连失败，无法写入 (已抑制%d次)\n", err_cnt);
            }
            return -1;
        }
    }
    
    // 先读取当前所有DO状态，避免清除其他位
    uint8_t do_values[16];
    if (modbus_read_bits(ctx_do, 0, 16, do_values) != 16) {
        close_do_connection();  // 读取失败，关闭连接触发重连
        static int err_cnt = 0;
        if (err_cnt++ % 100 == 0) {
            fprintf(stderr, "读取当前DO状态失败 (已抑制%d次)\n", err_cnt);
        }
        return -1;
    }
    
    // 设置DO值
    do_values[0] = do_signals.start_button_light;    // M800
    do_values[1] = do_signals.reset_button_light;    // M801
    do_values[2] = do_signals.pause_button_light;    // M802
    do_values[3] = do_signals.buzzer;               // M803
    do_values[4] = do_signals.red_light;            // M804
    do_values[5] = do_signals.yellow_light;         // M805
    do_values[6] = do_signals.green_light;          // M806
    do_values[10] = do_signals.lift_cylinder_down;   // M810
    do_values[11] = do_signals.gear_cylinder_extend;// M811
    do_values[12] = do_signals.belt_forward;        // M812
    do_values[13] = do_signals.belt_backward;       // M813 皮带反转控制
    do_values[14] = do_signals.smema_mr;            // M814 SMEMA机器就绪/本机要板
    do_values[15] = do_signals.smema_ba;            // M815 SMEMA本机有板待发
    
    // 写入设备
    int result = modbus_write_bits(ctx_do, 0, 16, do_values);
    
    if (result == 16) {
        // 写入成功，刷新状态
        refresh_do_state_from_device();
    } else {
        // 写入失败，关闭连接触发重连
        close_do_connection();
        static int err_cnt = 0;
        if (err_cnt++ % 100 == 0) {
            fprintf(stderr, "写入DO失败: %s (已抑制%d次)\n", 
                    modbus_strerror(errno), err_cnt);
        }
    }
    
    return result;
}

int write_single_do_signal(int do_address, bool state) {
    std::lock_guard<std::mutex> lock(modbus_do_mutex);
    
    // 连接断开时尝试重连
    if (!ctx_do || !do_status.is_connected) {
        try_reconnect_do();
        if (!ctx_do) {
            static int err_cnt = 0;
            if (err_cnt++ % 100 == 0) {
                fprintf(stderr, "DO未连接，重连失败，无法写入单信号 (已抑制%d次)\n", err_cnt);
            }
            return -1;
        }
    }
    
    // 地址有效性检查
    if (do_address < 800 || do_address > 815) {
        fprintf(stderr, "DO地址超出范围: %d\n", do_address);
        return -1;
    }
    
    // 写入单个信号到设备
    int result = modbus_write_bit(ctx_do, do_address - 800, state);
    
    if (result == 1) {
        // 写入成功后，刷新DO状态缓存（注意：这里不调用refresh_do_state_from_device避免死锁）
        uint8_t do_values[16];
        if (modbus_read_bits(ctx_do, 0, 16, do_values) == 16) {
            current_do_state.start_button_light = do_values[0];
            current_do_state.reset_button_light = do_values[1];
            current_do_state.pause_button_light = do_values[2];
            current_do_state.buzzer = do_values[3];
            current_do_state.red_light = do_values[4];
            current_do_state.yellow_light = do_values[5];
            current_do_state.green_light = do_values[6];
            current_do_state.lift_cylinder_down = do_values[10];
            current_do_state.gear_cylinder_extend = do_values[11];
            current_do_state.belt_forward = do_values[12];
            current_do_state.belt_backward = do_values[13];
            current_do_state.smema_mr = do_values[14];
            current_do_state.smema_ba = do_values[15];
        }
    } else {
        // 写入失败，关闭连接触发重连
        close_do_connection();
        static int err_cnt = 0;
        if (err_cnt++ % 100 == 0) {
            fprintf(stderr, "写入单DO信号失败: %s (已抑制%d次)\n", 
                    modbus_strerror(errno), err_cnt);
        }
    }
    
    return result;
}
#else
// DO模块禁用时的空实现
int write_do_signals(DO_Interface do_signals) {
    fprintf(stderr, "警告: DO模块已禁用，无法写入信号\n");
    return -1;
}

int write_single_do_signal(int do_address, bool state) {
    fprintf(stderr, "警告: DO模块已禁用，无法写入信号\n");
    return -1;
}
#endif

// 获取当前DO状态（始终可用）
DO_Interface get_current_do_state() {
    return current_do_state;
}

// 强制从设备刷新DO状态
void refresh_do_state() {
#if ENABLE_DO_MODULE
    refresh_do_state_from_device();
#else
    printf("DO模块已禁用，无法刷新状态\n");
#endif
}

// 模块状态检查函数
bool is_di_module_enabled() {
#if ENABLE_DI_MODULE
    return true;
#else
    return false;
#endif
}

bool is_do_module_enabled() {
#if ENABLE_DO_MODULE
    return true;
#else
    return false;
#endif
}

// 连接状态检查
bool is_di_connected() {
#if ENABLE_DI_MODULE
    return di_status.is_connected && ctx_di != NULL;
#else
    return false;
#endif
}

bool is_do_connected() {
#if ENABLE_DO_MODULE
    return do_status.is_connected && ctx_do != NULL;
#else
    return false;
#endif
}

// 在文件末尾添加DO控制命令解析实现
bool parse_do_control_command(const std::string& command, DOControlCommand& do_cmd) {
    // 格式: "801:1" 或 "801:0"
    size_t colon_pos = command.find(':');
    if (colon_pos == std::string::npos) {
        fprintf(stderr, "无效的DO控制命令格式，应为 '地址:状态'\n");
        return false;
    }
    
    std::string addr_str = command.substr(0, colon_pos);
    std::string state_str = command.substr(colon_pos + 1);
    
    // 去除空格
    addr_str.erase(0, addr_str.find_first_not_of(" \t"));
    addr_str.erase(addr_str.find_last_not_of(" \t") + 1);
    state_str.erase(0, state_str.find_first_not_of(" \t"));
    state_str.erase(state_str.find_last_not_of(" \t") + 1);
    
    // 验证地址有效性
    try {
        int address = std::stoi(addr_str);
        if (address < 800 || address > 815) {
            fprintf(stderr, "DO地址超出范围 (800-815): %d\n", address);
            return false;
        }
        do_cmd.do_address = addr_str;
    } catch (const std::exception& e) {
        fprintf(stderr, "DO地址解析失败: %s\n", e.what());
        return false;
    }
    
    // 验证状态有效性
    if (state_str == "1" || state_str == "true" || state_str == "True") {
        do_cmd.state = true;
    } else if (state_str == "0" || state_str == "false" || state_str == "False") {
        do_cmd.state = false;
    } else {
        fprintf(stderr, "无效的状态值: %s (应为 0/1 或 true/false)\n", state_str.c_str());
        return false;
    }
    
    return true;
}