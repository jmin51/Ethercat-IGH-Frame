#ifndef BUSINESS_LOGIC_PROCESSOR_HPP
#define BUSINESS_LOGIC_PROCESSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <map>
#include <vector>
#include <memory>
#include <functional>
#include "io_interface.hpp"

/**
 * @brief DI信号控制动作定义
 * 描述当DI信号触发时要执行的具体轴控制动作
 */
struct ControlAction {
    std::string axis_name;        ///< 控制的轴名称("axis1", "axis2", "all"等)
    double target_position;       ///< 目标位置(mm)
    int timeout_ms;               ///< 动作超时时间(毫秒)
    std::string description;      ///< 动作描述(用于日志和调试)
};

/**
 * @brief DI信号配置定义
 * 配置单个DI信号的触发条件和对应的控制动作
 */
struct DIConfig {
    std::string di_name;          ///< DI信号名称(用于标识和日志)
    std::function<bool(const DI_Interface&)> getter;  ///< DI状态获取函数
    ControlAction on_rising_edge;  ///< 上升沿触发动作
    ControlAction on_falling_edge; ///< 下降沿触发动作
    ControlAction on_high_level;   ///< 高电平触发动作
    ControlAction on_low_level;    ///< 低电平触发动作
    
    // 状态追踪
    bool last_state = false;      ///< 上一次DI状态(用于检测边沿变化)
    int64_t last_trigger_time = 0; ///< 上次触发时间戳(用于去抖)
};

/**
 * @brief 轴控制命令
 * 业务逻辑处理器生成的待执行命令
 */
struct AxisCommand {
    std::string axis_name;        ///< 目标轴名称
    double target_position;       ///< 目标位置
    bool immediate;              ///< 是否立即执行
};

/**
 * @brief 业务逻辑处理器
 * 负责根据DI信号状态自动生成轴控制命令，实现多对多的DI-轴映射关系
 * 设计原则：低耦合、可配置、事件驱动
 */
class BusinessLogicProcessor {
public:
    /**
     * @brief 构造函数
     * @param logger ROS2日志器
     */
    explicit BusinessLogicProcessor(rclcpp::Logger logger);
    ~BusinessLogicProcessor() = default;
    
    /**
     * @brief 初始化处理器
     * @param num_axes 系统支持的轴数量
     */
    void initialize(size_t num_axes);
    
    /**
     * @brief 配置DI信号与控制逻辑的映射关系
     * @param di_name DI信号名称(描述性名称)
     * @param getter DI状态获取函数
     * @param rising_action 上升沿触发动作(可选)
     * @param falling_action 下降沿触发动作(可选)
     * @param high_action 高电平触发动作(可选)
     * @param low_action 低电平触发动作(可选)
     */
    void configure_di_signal(const std::string& di_name, 
                            std::function<bool(const DI_Interface&)> getter,
                            const ControlAction& rising_action = {},
                            const ControlAction& falling_action = {},
                            const ControlAction& high_action = {},
                            const ControlAction& low_action = {});
    
    /**
     * @brief 处理IO信号变化，生成相应的控制命令
     * @param di_signals 当前所有DI信号状态
     */
    void process_io_signals(const DI_Interface& di_signals);
    
    /**
     * @brief 获取待执行的轴控制命令
     * @return 待执行命令列表
     */
    std::vector<AxisCommand> get_pending_commands();
    
    /**
     * @brief 清空待处理命令(通常在命令执行后调用)
     */
    void clear_pending_commands();
    
    // 启用/禁用业务逻辑
    void enable() { enabled_ = true; }
    void disable() { enabled_ = false; }
    bool is_enabled() const { return enabled_; }
    
    // 设置去抖时间(防止信号抖动导致的误触发)
    void set_debounce_time(int ms) { debounce_time_ms_ = ms; }
    
    /**
     * @brief 获取已配置的DI信号列表
     * @return DI信号名称列表
     */
    std::vector<std::string> get_configured_di_signals() const;
    
    /**
     * @brief 预定义的标准业务逻辑配置
     * 包含常用的DI-轴映射关系，方便快速初始化
     */
    void setup_standard_configurations();

private:
    std::map<std::string, DIConfig> di_configs_;  ///< DI信号配置映射表
    std::vector<AxisCommand> pending_commands_;   ///< 待执行的轴控制命令队列
    
    // 状态标志
    bool enabled_ = false;        ///< 处理器使能状态
    bool initialized_ = false;    ///< 初始化完成标志
    int debounce_time_ms_ = 50;   ///< 信号去抖时间(毫秒)
    
    rclcpp::Logger logger_;       ///< ROS2日志器
    
    // 内部处理函数
    void process_di_signal(const std::string& di_name, DIConfig& config, bool current_state);
    void add_command_if_valid(const ControlAction& action);
    bool check_debounce(DIConfig& config, int64_t current_time);
};

#endif // BUSINESS_LOGIC_PROCESSOR_HPP