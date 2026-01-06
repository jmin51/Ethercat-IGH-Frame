#ifndef ETHERCAT_NODE_HPP
#define ETHERCAT_NODE_HPP

#include "servo_axis_base.hpp"
#include "servo_axis_factory.hpp"
#include "io_interface.hpp"  // 新增IO模块头文件
#include "BusinessLogicProcessor.hpp"  // 添加这行
#include "LayerCommandProcessor.hpp" 
#include <std_msgs/msg/u_int8.hpp> 
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
    void add_axis(std::shared_ptr<ServoAxisBase> axis);
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
    void handle_layer_command(const std_msgs::msg::UInt8::SharedPtr msg);
    uint8_t get_current_layer() const { return layer_processor_->get_current_layer(); }

private:
    void initialize_node();
    void handle_displacement_command(const std_msgs::msg::String::SharedPtr msg);

    // 添加新的解析方法
    bool parse_displacement_command(const std::string& command, 
                                    std::vector<std::pair<std::string, double>>& axis_commands);

    void handle_control_command_msg(const std_msgs::msg::String::SharedPtr msg);
    void handle_axis_command(size_t axis_index, double newTargetPosition);
    double pulses_to_displacement(int32_t pulses, int32_t initial_pulses);
    void publish_io_status();  // 新增：发布IO状态
    // 添加点动指令处理函数
    void handle_jog_command(const std_msgs::msg::String::SharedPtr msg);

    std::vector<std::shared_ptr<ServoAxisBase>> servo_axes_;
    std::vector<double> last_target_positions_;
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr io_status_pub_;  // 新增IO状态发布器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr displacement_sub_;  // 从Float64MultiArray改为String
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_command_sub_;
    // 添加点动指令订阅器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr jog_command_sub_;
    // 添加入库流程话题订阅器
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr warehouse_start_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr warehouse_stop_sub_;
    // 添加出库流程话题订阅器
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr outbound_start_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr outbound_stop_sub_;
    // 添加IO控制话题
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr do_control_sub_;
    // 添加点动速度设置订阅器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr jog_speed_sub_;
    
    // 添加点动速度设置处理函数
    void handle_jog_speed_command(const std_msgs::msg::String::SharedPtr msg);
    
    // 添加点动速度解析函数
    bool parse_jog_speed_command(const std::string& command, std::string& axis_name, double& speed);

    std::atomic<bool> node_shutting_down_;
    
    // IO模块相关
    pthread_t io_thread_;
    std::atomic<bool> io_running_{false};
    DI_Interface current_di_status_;
    DO_Interface current_do_control_;
    pthread_mutex_t io_mutex_;

    // 业务逻辑处理器 - 负责DI信号到轴控制的映射逻辑
    std::unique_ptr<BusinessLogicProcessor> business_processor_;
    std::vector<AxisCommand> last_executed_commands_;
    // 层指令处理器
    std::unique_ptr<LayerCommandProcessor> layer_processor_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr layer_command_sub_;

    // 初始化业务逻辑处理器——创建处理器实例并配置标准的DI-轴映射关系，初始化层指令处理器
    void initialize_business_logic();
    void initialize_layer_processor();
    void monitor_di_changes(const DI_Interface& current_di);
    void handle_do_control(const std_msgs::msg::String::SharedPtr msg);
    // 添加入库处理函数
    void handle_warehouse_start(const std_msgs::msg::UInt8::SharedPtr msg) {
        if (node_shutting_down_.load() || !rclcpp::ok()) {
            return;
        }
        
        uint8_t target_layer = msg->data;
        RCLCPP_INFO(this->get_logger(), "收到入库启动命令，目标层: %d", target_layer);
        
        if (business_processor_) {
            business_processor_->start_warehouse_process(target_layer);
        }
    }
    
    void handle_warehouse_stop(const std_msgs::msg::Empty::SharedPtr msg) {
        if (node_shutting_down_.load() || !rclcpp::ok()) {
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "收到入库停止命令");
        
        if (business_processor_) {
            business_processor_->stop_warehouse_process();
        }
    }

    // 添加出库处理函数
    void handle_outbound_start(const std_msgs::msg::UInt8::SharedPtr msg) {
        if (node_shutting_down_.load() || !rclcpp::ok()) {
            return;
        }
        
        uint8_t source_layer = msg->data;
        RCLCPP_INFO(this->get_logger(), "收到出库启动命令，源层: %d", source_layer);
        
        if (business_processor_) {
            business_processor_->start_outbound_process(source_layer);
        }
    }

    void handle_outbound_stop(const std_msgs::msg::Empty::SharedPtr msg) {
        if (node_shutting_down_.load() || !rclcpp::ok()) {
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "收到出库停止命令");
        
        if (business_processor_) {
            business_processor_->stop_outbound_process();
        }
    }
};

// 全局变量声明
extern std::shared_ptr<EthercatNode> global_node;
extern ec_master_t *master;
extern ec_domain_t *domain1;
extern uint8_t *domain1_pd;
extern std::atomic<bool> g_should_exit;

// 全局函数声明
void signal_handler(int signum);
void safe_shutdown();
void* rt_task_wrapper(void* arg);
void* io_monitor_thread(void* arg);  // 新增IO监控线程

#endif // ETHERCAT_NODE_HPP