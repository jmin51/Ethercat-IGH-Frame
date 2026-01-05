#ifndef IO_INTERFACE_HPP
#define IO_INTERFACE_HPP

#include <stdint.h>
#include <stdbool.h>
#include <time.h>  // 添加time.h头文件
#include <string>

// 宏开关配置 - 根据实际需求开启或关闭
#define ENABLE_DI_MODULE    1  // 1:启用DI模块 0:禁用DI模块
#define ENABLE_DO_MODULE    1  // 1:启用DO模块 0:禁用DO模块

// 设备IP配置
#define DI_DEVICE_IP        "192.168.3.12"
#define DO_DEVICE_IP        "192.168.2.12"
#define MODBUS_PORT         502
#define MODBUS_SLAVE_ID     1

// DI 输入信号接口定义
typedef struct {
    bool start_button;           // M512 启动按钮
    bool reset_button;           // M513 复位按钮
    bool pause_button;           // M514 暂停按钮
    bool manual_auto_button;     // M515 手自动按钮
    bool emergency_stop;          // M516 急停按钮
    bool air_supply;             // M517 气源输入
    bool safety_door_1;          // M518 安全门检1
    bool safety_door_2;          // M519 安全门检2
    bool feed_product_detect;    // M520 入料产品检测
    bool buffer_sensor_1;        // M521 缓存架对射1
    bool buffer_sensor_2;        // M522 缓存架对射2
    bool buffer_in_position;     // M523 缓存架入料产品到位检测
    bool buffer_out_position;    // M524 缓存架出料产品到位检测
    bool conveyor_in_position;   // M525 接驳台入料产品到位检测
    bool conveyor_out_position;  // M526 接驳台出料产品到位检测
    bool lift_cylinder1_up;      // M527 顶升气缸1上升到位
    bool lift_cylinder1_down;    // M528 顶升气缸1下降到位
    bool lift_cylinder2_up;      // M529 顶升气缸2上升到位
    bool lift_cylinder2_down;    // M530 顶升气缸2下降到位
    bool gear_cylinder1_extend;  // M531 齿轮对接气缸1伸出到位
    bool gear_cylinder1_retract; // M532 齿轮对接气缸1缩回到位
    bool gear_cylinder2_extend;  // M533 齿轮对接气缸2伸出到位
    bool gear_cylinder2_retract; // M534 齿轮对接气缸2缩回到位
} DI_Interface;

// DO 输出信号接口定义
typedef struct {
    bool start_button_light;     // M800 启动按钮灯
    bool reset_button_light;     // M801 复位按钮灯
    bool pause_button_light;    // M802 暂停按钮灯
    bool buzzer;                // M803 蜂鸣器
    bool red_light;             // M804 三色红灯
    bool yellow_light;          // M805 三色黄灯
    bool green_light;           // M806 三色绿灯
    bool lift_cylinder_down;    // M810 顶升气缸下降
    bool gear_cylinder_extend;  // M811 齿轮对接气缸伸出
    bool belt_forward;          // M812 皮带正转启动
    bool belt_backward;          // M813 皮带反转启动 - 新增
} DO_Interface;

// 初始化函数
int init_modbus_interface(const char* di_ip, int di_port, int di_slave_id, 
                         const char* do_ip, int do_port, int do_slave_id);
void cleanup_modbus_interface();

// DI 读取接口
#if ENABLE_DI_MODULE
DI_Interface read_all_di_signals();
bool read_single_di_signal(int di_address);
#endif

// DO 写入接口
#if ENABLE_DO_MODULE
int write_do_signals(DO_Interface do_signals);
int write_single_do_signal(int do_address, bool state);
#endif

// 获取当前DO状态
DO_Interface get_current_do_state();
void refresh_do_state();  // 强制刷新DO状态

// 信号打印接口
void print_di_status(DI_Interface di);
void print_do_status(DO_Interface do_control);

// 模块状态检查
bool is_di_module_enabled();
bool is_do_module_enabled();

// 非阻塞延时检查
int should_execute_sequence(time_t *last_time, int interval_seconds);
// DO控制命令消息格式
struct DOControlCommand {
    std::string do_address;  // DO地址，如 "801"
    bool state;              // 状态：true(1)/false(0)
};

// DO控制命令解析函数
bool parse_do_control_command(const std::string& command, DOControlCommand& do_cmd);
#endif