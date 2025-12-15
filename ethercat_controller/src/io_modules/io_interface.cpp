#include "io_interface.hpp"
#include <stdio.h>
#include <errno.h>
#include <modbus/modbus.h>
#include <cstring>  // 添加这行，用于 memcmp 函数
#include <time.h>

static modbus_t *ctx_di = NULL;  // DI设备连接
#if ENABLE_DO_MODULE  // 条件编译
static modbus_t *ctx_do = NULL;  // DO设备连接
#endif
static DO_Interface current_do_state = {0};

// 函数前置声明
#if ENABLE_DO_MODULE
static void refresh_do_state_from_device(void);
#endif

// 从设备刷新DO状态（内部函数）
#if ENABLE_DO_MODULE
static void refresh_do_state_from_device() {
    if (!ctx_do) return;
    
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
        printf("DO状态已从设备刷新\n");
    } else {
        fprintf(stderr, "刷新DO状态失败: %s\n", modbus_strerror(errno));
    }
}
#endif

// Modbus 初始化 - 根据宏开关条件连接设备
int init_modbus_interface(const char* di_ip, int di_port, int di_slave_id,
                         const char* do_ip, int do_port, int do_slave_id) {
    
    int result = 0;
    
    // 初始化DI连接（如果启用）
#if ENABLE_DI_MODULE
    if (di_ip != NULL) {
        ctx_di = modbus_new_tcp(di_ip, di_port);
        if (ctx_di == NULL) {
            fprintf(stderr, "无法创建 DI Modbus 上下文\n");
            result = -1;
        } else {
            modbus_set_response_timeout(ctx_di, 1, 0);
            modbus_set_slave(ctx_di, di_slave_id);
            
            if (modbus_connect(ctx_di) == -1) {
                fprintf(stderr, "DI连接失败: %s\n", modbus_strerror(errno));
                modbus_free(ctx_di);
                ctx_di = NULL;
                result = -1;
            } else {
                printf("DI设备连接成功: %s:%d (从站ID: %d)\n", di_ip, di_port, di_slave_id);
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
            result = -1;
        } else {
            modbus_set_response_timeout(ctx_do, 1, 0);
            modbus_set_slave(ctx_do, do_slave_id);
            
            if (modbus_connect(ctx_do) == -1) {
                fprintf(stderr, "DO连接失败: %s\n", modbus_strerror(errno));
                modbus_free(ctx_do);
                ctx_do = NULL;
                result = -1;
            } else {
                printf("DO设备连接成功: %s:%d (从站ID: %d)\n", do_ip, do_port, do_slave_id);
                
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
        printf("DI连接已关闭\n");
    }
#endif
    
#if ENABLE_DO_MODULE
    if (ctx_do) {
        modbus_close(ctx_do);
        modbus_free(ctx_do);
        ctx_do = NULL;
        printf("DO连接已关闭\n");
    }
#endif
}

// DI 信号读取接口
#if ENABLE_DI_MODULE
DI_Interface read_all_di_signals() {
    DI_Interface di = {0};
    
    if (!ctx_di) {
        fprintf(stderr, "DI Modbus 未初始化\n");
        return di;
    }
    
    uint8_t di_values[48];
    int rc = modbus_read_input_bits(ctx_di, 0, 48, di_values);
    
    if (rc == -1) {
        fprintf(stderr, "读取DI失败: %s\n", modbus_strerror(errno));
        return di;
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
    
    return di;
}

bool read_single_di_signal(int di_address) {
    if (!ctx_di) return false;
    
    // 地址有效性检查
    if (di_address < 512 || di_address > 534) {
        fprintf(stderr, "DI地址超出范围: %d\n", di_address);
        return false;
    }
    
    uint8_t value;
    int rc = modbus_read_input_bits(ctx_di, di_address - 512, 1, &value);
    
    return (rc == 1) ? value : false;
}
#else
// DI模块禁用时的空实现
DI_Interface read_all_di_signals() {
    DI_Interface di = {0};
    fprintf(stderr, "警告: DI模块已禁用，返回空数据\n");
    return di;
}

bool read_single_di_signal(int di_address) {
    fprintf(stderr, "警告: DI模块已禁用，无法读取信号\n");
    return false;
}
#endif

// DO 信号写入接口
#if ENABLE_DO_MODULE
int write_do_signals(DO_Interface do_signals) {
    if (!ctx_do) return -1;
    
    // 先读取当前所有DO状态，避免清除其他位
    uint8_t do_values[16];
    if (modbus_read_bits(ctx_do, 0, 16, do_values) != 16) {
        fprintf(stderr, "读取当前DO状态失败\n");
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
    
    // 写入设备
    int result = modbus_write_bits(ctx_do, 0, 16, do_values);
    
    // 无论写入是否成功，都从设备刷新状态以确保一致性
    if (result == 16) {
        refresh_do_state_from_device();
    }
    
    return result;
}

int write_single_do_signal(int do_address, bool state) {
    if (!ctx_do) return -1;
    
    // 地址有效性检查
    if (do_address < 800 || do_address > 812) {
        fprintf(stderr, "DO地址超出范围: %d\n", do_address);
        return -1;
    }
    
    // 写入单个信号到设备
    int result = modbus_write_bit(ctx_do, do_address - 800, state);
    
    // 写入成功后，从设备刷新整个DO状态以确保一致性
    if (result == 1) {
        refresh_do_state_from_device();
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

void print_di_status(DI_Interface di) {
    static DI_Interface last_di = {0};
    static bool first_print = true;
    
    bool has_change = false;
    
    // 检查是否有变化
    if (first_print || memcmp(&di, &last_di, sizeof(DI_Interface)) != 0) {
        has_change = true;
        first_print = false;
    }
    
    if (has_change) {
        printf("\n--- 输入信号变化 ---\n");
        
        // 只打印变化的信号
        if (first_print || di.start_button != last_di.start_button)
            printf("DI01 启动按钮: %s\n", di.start_button ? "按下" : "释放");
        
        if (first_print || di.reset_button != last_di.reset_button)
            printf("DI02 复位按钮: %s\n", di.reset_button ? "按下" : "释放");
        
        if (first_print || di.pause_button != last_di.pause_button)
            printf("DI03 暂停按钮: %s\n", di.pause_button ? "按下" : "释放");
        
        if (first_print || di.manual_auto_button != last_di.manual_auto_button)
            printf("DI04 手自动按钮: %s\n", di.manual_auto_button ? "自动" : "手动");
        
        if (first_print || di.emergency_stop != last_di.emergency_stop)
            printf("DI05 急停状态: %s\n", di.emergency_stop ? "激活" : "正常");
        
        if (first_print || di.air_supply != last_di.air_supply)
            printf("DI06 气源状态: %s\n", di.air_supply ? "正常" : "异常");
        
        if (first_print || di.safety_door_1 != last_di.safety_door_1)
            printf("DI07 安全门1: %s\n", di.safety_door_1 ? "关闭" : "打开");
        
        if (first_print || di.safety_door_2 != last_di.safety_door_2)
            printf("DI08 安全门2: %s\n", di.safety_door_2 ? "关闭" : "打开");
        
        if (first_print || di.feed_product_detect != last_di.feed_product_detect)
            printf("DI09 产品检测: %s\n", di.feed_product_detect ? "有产品" : "无产品");
        
        if (first_print || di.buffer_sensor_1 != last_di.buffer_sensor_1)
            printf("DI10 缓存架传感器1: %s\n", di.buffer_sensor_1 ? "触发" : "正常");
        
        if (first_print || di.buffer_sensor_2 != last_di.buffer_sensor_2)
            printf("DI11 缓存架传感器2: %s\n", di.buffer_sensor_2 ? "触发" : "正常");
        
        if (first_print || di.buffer_in_position != last_di.buffer_in_position)
            printf("DI12 缓存架入料到位: %s\n", di.buffer_in_position ? "到位" : "未到位");
        
        if (first_print || di.buffer_out_position != last_di.buffer_out_position)
            printf("DI13 缓存架出料到位: %s\n", di.buffer_out_position ? "到位" : "未到位");
        
        if (first_print || di.conveyor_in_position != last_di.conveyor_in_position)
            printf("DI14 接驳台入料到位: %s\n", di.conveyor_in_position ? "到位" : "未到位");
        
        if (first_print || di.conveyor_out_position != last_di.conveyor_out_position)
            printf("DI15 接驳台出料到位: %s\n", di.conveyor_out_position ? "到位" : "未到位");
        
        if (first_print || di.lift_cylinder1_up != last_di.lift_cylinder1_up)
            printf("DI16 顶升气缸1上升: %s\n", di.lift_cylinder1_up ? "到位" : "未到位");
        
        if (first_print || di.lift_cylinder1_down != last_di.lift_cylinder1_down)
            printf("DI17 顶升气缸1下降: %s\n", di.lift_cylinder1_down ? "到位" : "未到位");
        
        if (first_print || di.lift_cylinder2_up != last_di.lift_cylinder2_up)
            printf("DI18 顶升气缸2上升: %s\n", di.lift_cylinder2_up ? "到位" : "未到位");
        
        if (first_print || di.lift_cylinder2_down != last_di.lift_cylinder2_down)
            printf("DI19 顶升气缸2下降: %s\n", di.lift_cylinder2_down ? "到位" : "未到位");
        
        if (first_print || di.gear_cylinder1_extend != last_di.gear_cylinder1_extend)
            printf("DI20 齿轮气缸1伸出: %s\n", di.gear_cylinder1_extend ? "到位" : "未到位");
        
        if (first_print || di.gear_cylinder1_retract != last_di.gear_cylinder1_retract)
            printf("DI21 齿轮气缸1缩回: %s\n", di.gear_cylinder1_retract ? "到位" : "未到位");
        
        if (first_print || di.gear_cylinder2_extend != last_di.gear_cylinder2_extend)
            printf("DI22 齿轮气缸2伸出: %s\n", di.gear_cylinder2_extend ? "到位" : "未到位");
        
        if (first_print || di.gear_cylinder2_retract != last_di.gear_cylinder2_retract)
            printf("DI23 齿轮气缸2缩回: %s\n", di.gear_cylinder2_retract ? "到位" : "未到位");
        
        last_di = di;
    }
}
// // DI信号打印函数
// void print_di_status(DI_Interface di) {
//     printf("\n--- 输入信号状态 ---\n");
//     printf("DI01 启动按钮: %s\n", di.start_button ? "按下" : "释放");
//     printf("DI02 复位按钮: %s\n", di.reset_button ? "按下" : "释放");
//     printf("DI03 暂停按钮: %s\n", di.pause_button ? "按下" : "释放");
//     printf("DI04 手自动按钮: %s\n", di.manual_auto_button ? "自动" : "手动");
//     printf("DI05 急停状态: %s\n", di.emergency_stop ? "激活" : "正常");
//     printf("DI06 气源状态: %s\n", di.air_supply ? "正常" : "异常");
//     printf("DI07 安全门1: %s\n", di.safety_door_1 ? "关闭" : "打开");
//     printf("DI08 安全门2: %s\n", di.safety_door_2 ? "关闭" : "打开");
//     printf("DI09 产品检测: %s\n", di.feed_product_detect ? "有产品" : "无产品");
//     printf("DI10 缓存架传感器1: %s\n", di.buffer_sensor_1 ? "触发" : "正常");
//     printf("DI11 缓存架传感器2: %s\n", di.buffer_sensor_2 ? "触发" : "正常");
//     printf("DI12 缓存架入料到位: %s\n", di.buffer_in_position ? "到位" : "未到位");
//     printf("DI13 缓存架出料到位: %s\n", di.buffer_out_position ? "到位" : "未到位");
//     printf("DI14 接驳台入料到位: %s\n", di.conveyor_in_position ? "到位" : "未到位");
//     printf("DI15 接驳台出料到位: %s\n", di.conveyor_out_position ? "到位" : "未到位");
//     printf("DI16 顶升气缸1上升: %s\n", di.lift_cylinder1_up ? "到位" : "未到位");
//     printf("DI17 顶升气缸1下降: %s\n", di.lift_cylinder1_down ? "到位" : "未到位");
//     printf("DI18 顶升气缸2上升: %s\n", di.lift_cylinder2_up ? "到位" : "未到位");
//     printf("DI19 顶升气缸2下降: %s\n", di.lift_cylinder2_down ? "到位" : "未到位");
//     printf("DI20 齿轮气缸1伸出: %s\n", di.gear_cylinder1_extend ? "到位" : "未到位");
//     printf("DI21 齿轮气缸1缩回: %s\n", di.gear_cylinder1_retract ? "到位" : "未到位");
//     printf("DI22 齿轮气缸2伸出: %s\n", di.gear_cylinder2_extend ? "到位" : "未到位");
//     printf("DI23 齿轮气缸2缩回: %s\n", di.gear_cylinder2_retract ? "到位" : "未到位");
// }

// DO信号打印函数
void print_do_status(DO_Interface do_control) {
    printf("\n--- 输出信号状态 ---\n");
    printf("DO01 启动按钮灯: %s\n", do_control.start_button_light ? "亮" : "灭");
    printf("DO02 复位按钮灯: %s\n", do_control.reset_button_light ? "亮" : "灭");
    printf("DO03 暂停按钮灯: %s\n", do_control.pause_button_light ? "亮" : "灭");
    printf("DO04 蜂鸣器: %s\n", do_control.buzzer ? "响" : "静");
    printf("DO05 红灯: %s\n", do_control.red_light ? "亮" : "灭");
    printf("DO06 黄灯: %s\n", do_control.yellow_light ? "亮" : "灭");
    printf("DO07 绿灯: %s\n", do_control.green_light ? "亮" : "灭");
    printf("DO11 顶升气缸下降: %s\n", do_control.lift_cylinder_down ? "动作" : "停止");
    printf("DO12 齿轮气缸伸出: %s\n", do_control.gear_cylinder_extend ? "动作" : "停止");
    printf("DO13 皮带正转: %s\n", do_control.belt_forward ? "运行" : "停止");
}

// 非阻塞延时检查
int should_execute_sequence(time_t *last_time, int interval_seconds) {
    time_t current_time = time(NULL);
    if (current_time - *last_time >= interval_seconds) {
        *last_time = current_time;
        return 1;
    }
    return 0;
}