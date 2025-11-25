#ifndef ETHERCAT_NODE_HPP
#define ETHERCAT_NODE_HPP

#include "servo_axis_base.hpp"
#include "servo_axis_factory.hpp"
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

private:
    void initialize_node();
    void handle_displacement_command(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void handle_control_command_msg(const std_msgs::msg::String::SharedPtr msg);
    void handle_axis_command(size_t axis_index, double newTargetPosition);
    double pulses_to_displacement(int32_t pulses, int32_t initial_pulses);

    std::vector<std::shared_ptr<ServoAxisBase>> servo_axes_;
    std::vector<double> last_target_positions_;
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr displacement_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_command_sub_;
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

#endif // ETHERCAT_NODE_HPP