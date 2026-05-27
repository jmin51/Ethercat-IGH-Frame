#ifndef ETHERCAT_NODE_HPP
#define ETHERCAT_NODE_HPP

#include "servo_axis_base.hpp"
#include "servo_axis_factory.hpp"
#include "io_interface.hpp"
#include "LayerCommandProcessor.hpp" 
#include "fault_management_system.hpp"
#include "smema_handler.hpp"
#include <std_msgs/msg/u_int8.hpp> 
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/empty.hpp>  // 添加这行，用于Empty消息类型
#include <ecrt.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <vector>
#include <memory>

// 应用参数
#define FREQUENCY 1000
#define PERIOD (1.0 / FREQUENCY)
#define CLOCK_TO_USE CLOCK_MONOTONIC
#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

// 控制命令定义
#define CMD_START_MANUAL "start_manual"
#define CMD_START_AUTO "start_auto" 
#define CMD_STOP "stop"
#define CMD_CLEAR_FAULT "clear_fault"
#define CMD_RESET "reset"

class EthercatNode : public rclcpp::Node {
public:
    EthercatNode(std::string name);
    virtual ~EthercatNode();

    // 轴管理
    void init_axes(ec_master_t* master);
    void register_pdo_entries(ec_domain_t* domain1);
    void handle_axes_state_machines(uint8_t* domain1_pd);
    std::vector<std::shared_ptr<ServoAxisBase>>& get_servo_axes();
    
    // 配置管理
    std::vector<ec_slave_config_t*> get_all_slave_configs();
    
    // 工具函数
    void publish_joint_states();
    void handle_control_command(const std::string& command);
    
    // 添加延迟初始化方法
    void initialize_after_axes();
    
    // IO模块相关
    void start_io_monitoring();
    void stop_io_monitoring();
    void handle_io_signals(DI_Interface di);
    bool is_io_running() const { return io_running_.load(); }

    // 层指令相关方法
    void handle_layer_command(const std_msgs::msg::Int8::SharedPtr msg);
    uint8_t get_current_layer() const { return layer_processor_->get_current_layer(); }
    // 新增：层运动完成检查函数
    void check_layer_motion_completion();
    // 新增：发布当前层号（浮点，支持小数层）
    void publish_current_layer();
    // 新增：自动模式初始化完成后校正层号
    void calibrate_layer_after_auto_init();

    void print_warning(const std::string& message);
    void print_error(const std::string& message);
    
    // +++ SMEMA协议方法 +++
    void init_smema_handler();
    void process_smema_cycle();
    void publish_smema_state();

    // 故障上报接口（供轴状态机调用）
    void report_axis_fault(const std::string& axis_name, uint16_t fault_code, const std::string& description);

    // +++ 新增：处理Python层业务逻辑故障 +++
    void handle_business_logic_fault(const std_msgs::msg::String::SharedPtr msg);

private:
    void initialize_node();
    void handle_displacement_command(const std_msgs::msg::String::SharedPtr msg);

    // 添加新的解析方法
    bool parse_displacement_command(const std::string& command, 
                                    std::vector<std::pair<std::string, double>>& axis_commands);

    void handle_axis_command(size_t axis_index, double newTargetPosition);
    double pulses_to_displacement(int32_t pulses, int32_t initial_pulses);
    void publish_io_status();  // 发布IO状态
    void publish_axis_states();  // 发布所有轴的状态机状态
    
    // Python控制命令处理
    void handle_py_control_command(const std_msgs::msg::String::SharedPtr msg);

    // 添加点动指令处理函数
    void handle_jog_command(const std_msgs::msg::String::SharedPtr msg);

    std::vector<std::shared_ptr<ServoAxisBase>> servo_axes_;
    std::vector<double> last_target_positions_;
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr io_status_pub_;  // IO状态发布器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr displacement_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_command_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr py_control_command_sub_;
    
    // 添加点动指令订阅器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr jog_command_sub_;

    // 添加IO控制话题
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr do_control_sub_;
    // 添加点动速度设置订阅器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr jog_speed_sub_;
    
    // 10ms 定时器，用于发布关节状态和检查层移动完成
    rclcpp::TimerBase::SharedPtr periodic_timer_;
    // 定时器回调函数
    void periodic_timer_callback();

    // 添加点动速度设置处理函数
    void handle_jog_speed_command(const std_msgs::msg::String::SharedPtr msg);
    
    // 添加点动速度解析函数
    bool parse_jog_speed_command(const std::string& command, std::string& axis_name, double& speed);

    std::atomic<bool> node_shutting_down_;
    
    // 故障管理相关：在现有成员变量中添加
    std::unique_ptr<fault_management::FaultManagementSystem> fault_manager_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_code_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr axis_state_pub_;  // 轴状态机状态发布器
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr axis5_layer_pub_;  // axis5当前层号发布器（浮点，支持小数层如5.5）
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr business_logic_fault_sub_;  // Python层故障订阅器

    // IO模块相关
    pthread_t io_thread_;
    std::atomic<bool> io_running_{false};
    DI_Interface current_di_status_;
    DO_Interface current_do_control_;
    pthread_mutex_t io_mutex_;
    
    // 状态变化检测（避免日志洪泛）
    bool last_all_axes_ready_ = false;  // 上次所有轴就绪状态
    bool last_manual_auto_state_ = false;  // 上次手自动状态
    bool auto_mode_init_published_ = false;  // 自动模式初始化完成状态是否已发布
    
    // 命令去重防抖（防止重复命令洪泛）
    std::string last_command_;              // 上次执行的命令
    rclcpp::Time last_command_time_{0, 0, RCL_STEADY_TIME};  // 上次命令时间
    static constexpr double CMD_DEDUP_SEC = 0.5;  // 去重窗口（秒）
    
    // std::vector<AxisCommand> last_executed_commands_;
    // 层指令处理器
    std::unique_ptr<LayerCommandProcessor> layer_processor_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr layer_command_sub_;

    // 删除initialize_business_logic方法
    void initialize_layer_processor();

    void handle_do_control(const std_msgs::msg::String::SharedPtr msg);
    
// 在EthercatNode类定义中添加
private:
    // 板宽控制相关
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr board_width_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr board_width_status_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr axis3_width_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr axis3_width_status_pub_;
  
    // 板宽控制参数 (axis4 - 保持不变)
    double screw_lead_;                   // 丝杠导程10mm
    double gear_ratio_;                   // 减速比9.0
    double axis4_offset_mm_;              // axis4机械零点偏移(mm)，两点标定校准
    int pulses_per_rev_;                  // 每转脉冲数10000
    double current_board_width_;           // 当前板宽（cm）
    double target_board_width_;           // 目标板宽（cm）
    double min_board_width_;              // 最小板宽10cm
    double max_board_width_;              // 最大板宽50cm
    double board_width_resolution_;       // 板宽分辨率
    bool board_width_moving_;             // 板宽调整中标志
    std::atomic<bool> board_width_updated_; // 板宽更新标志
    
    // +++ 新增：axis3 板宽控制专用变量 +++
    double axis3_min_width_;     // axis3 最小板宽
    double axis3_max_width_;     // axis3 最大板宽
    double axis3_current_width_; // axis3 当前板宽
    double axis3_target_width_;  // axis3 目标板宽
    bool axis3_width_moving_;    // axis3 调整中标志
    std::atomic<bool> axis3_width_updated_; // axis3 更新标志
    // 注意：axis3可能需要独立的机械参数（导程、减速比），需根据实际情况设置
    double axis3_screw_lead_;
    double axis3_gear_ratio_;
    
    // 板宽校正标志（启动后读取实际位置校正板宽）
    bool board_width_calibrated_;  // 板宽是否已校正

    // 板宽控制方法
    void initialize_board_width_parameters();
    void handle_board_width_command(const std_msgs::msg::Float64::SharedPtr msg);
    double calculate_displacement_from_width(double board_width_cm);
    bool validate_board_width(double width);
    void execute_board_width_adjustment();
    void publish_board_width_status(double current_width, double target_width, 
                                   bool moving, const std::string& status);
    int find_axis4_index();  // 查找axis4的索引
    // Axis3 板宽控制相关函数声明
    void handle_axis3_width_command(const std_msgs::msg::Float64::SharedPtr msg);
    void execute_axis3_width_adjustment();
    double calculate_axis3_displacement_from_width(double board_width_cm);
    int find_axis3_index();
    void publish_axis3_width_status(double current_width, double target_width,
                                   bool moving, const std::string& status);
    bool are_all_axes_in_auto_mode();  // 检查所有轴是否都在自动模式
    
    // +++ 新增：根据实际位置校正板宽 +++
    void calibrate_board_width_from_position();
    // +++ 实时同步：决策前从轴位置刷新板宽模型 +++
    void sync_axis3_current_width_from_position();
    void sync_axis4_current_width_from_position();

public:
    /* ============================================
     * 暂停状态记录相关接口 (public - 供main.cpp调用)
     * ============================================ */
    // 发布暂停状态记录请求
    void publish_pause_state_record_request();
    // 发布暂停状态恢复请求（携带记录的状态）
    void publish_pause_state_resume_request();
    // 获取暂停状态记录发布器
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr get_pause_state_pub() { return pause_state_pub_; }
    // 处理Python端的状态报告
    void handle_pause_state_report(const std_msgs::msg::String::SharedPtr msg);
    
    // 重置自动模式初始化发布标志（用于暂停后恢复）
    void reset_auto_mode_init_published() { auto_mode_init_published_ = false; }

private:
    // 暂停状态话题发布器
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pause_state_pub_;
    // 订阅Python端的状态报告
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pause_state_report_sub_;
    
    // +++ SMEMA协议相关成员 +++
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr smema_product_position_sub_; // 产品到位信号订阅
    bool smema_initialized_;
};

// 全局变量声明
extern std::shared_ptr<EthercatNode> global_node;
extern ec_master_t *master;
extern ec_domain_t *domain1;
extern uint8_t *domain1_pd;
extern std::atomic<bool> g_should_exit;
// 在全局变量定义区域添加
extern std::atomic<bool> g_system_running;  // 系统运行状态
extern std::atomic<bool> g_start_button_pressed;  // 启动按钮状态
extern std::atomic<bool> g_pause_button_pressed;  // 暂停按钮状态
extern std::atomic<bool> g_reset_button_pressed;  // 新增：复位按钮状态

// 全局函数声明
void signal_handler(int signum);
void safe_shutdown(bool is_pause);
void* rt_task_wrapper(void* arg);
void* io_monitor_thread(void* arg);  // 新增IO监控线程

#endif // ETHERCAT_NODE_HPP