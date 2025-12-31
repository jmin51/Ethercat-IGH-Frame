#ifndef BUSINESS_LOGIC_PROCESSOR_HPP
#define BUSINESS_LOGIC_PROCESSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <map>
#include <vector>
#include <memory>
#include <functional>
#include "io_interface.hpp"

/**
 * @brief 命令类型定义
 */
enum class CommandType {
    POSITION,  // 位置控制
    JOG,       // 点动控制
    LAYER,     // 层指令
    STOP       // 停止命令
};

/**
 * @brief DI信号控制动作定义
 * 描述当DI信号触发时要执行的具体轴控制动作
 */
struct ControlAction {
    CommandType type;           ///< 命令类型
    std::string axis_name;      ///< 控制的轴名称
    std::string command_value;  ///< 命令值(forward/reverse/stop/层号等)
    double target_position;     ///< 目标位置(mm) - 仅POSITION类型使用
    int timeout_ms;             ///< 动作超时时间(毫秒)
    std::string description;    ///< 动作描述
};

/**
 * @brief 入库流程状态
 */
enum class WarehouseState {
    IDLE,               // 空闲状态
    WAIT_FOR_ENTRY,     // 等待入库
    CONVEYOR_MOVING,    // 输送带运行
    LIFT_MOVING,        // 提升机运行
    COMPLETED           // 完成
};

/**
 * @brief DI信号配置定义
 */
struct DIConfig {
    std::string di_name;
    std::function<bool(const DI_Interface&)> getter;
    std::vector<ControlAction> actions;  // 改为动作列表，支持多步骤
    
    // 状态追踪
    bool last_state = false;
    int64_t last_trigger_time = 0;
    WarehouseState current_state = WarehouseState::IDLE;
};

/**
 * @brief 轴控制命令
 * 业务逻辑处理器生成的待执行命令
 */
struct AxisCommand {
    CommandType type;
    std::string axis_name;
    std::string command_value;
    double target_position;
    bool immediate;
};

/**
 * @brief 业务逻辑处理器 - 支持入库流程
 */
class BusinessLogicProcessor {
public:
    explicit BusinessLogicProcessor(rclcpp::Logger logger);
    ~BusinessLogicProcessor() = default;
    
    void initialize(size_t num_axes);
    
    /**
     * @brief 配置入库流程DI信号
     */
    void configure_warehouse_process();
    
    /**
     * @brief 处理IO信号变化，生成入库控制命令
     */
    void process_io_signals(const DI_Interface& di_signals);
    
    std::vector<AxisCommand> get_pending_commands();
    void clear_pending_commands();
    
    // 启用/禁用业务逻辑
    void enable() { enabled_ = true; }
    void disable() { enabled_ = false; }
    bool is_enabled() const { return enabled_; }
    
    // 设置去抖时间(防止信号抖动导致的误触发)
    void set_debounce_time(int ms) { debounce_time_ms_ = ms; }
    std::vector<std::string> get_configured_di_signals() const;

    /**
     * @brief 设置当前层号(用于入库流程)
     */
    void set_current_layer(uint8_t layer) { current_layer_ = layer; }
    uint8_t get_current_layer() const { return current_layer_; }
    void set_target_layer(uint8_t layer) { target_layer_ = layer; }
    uint8_t get_target_layer() const { return target_layer_; }

private:
    std::map<std::string, DIConfig> di_configs_;  ///< DI信号配置映射表
    std::vector<AxisCommand> pending_commands_;   ///< 待执行的轴控制命令队列
    
    // 入库流程状态
    WarehouseState warehouse_state_ = WarehouseState::IDLE;
    uint8_t current_layer_ = 1;
    uint8_t target_layer_ = 1;
    bool waiting_for_entry_ = false;
    bool waiting_for_exit_ = false;
    
    bool enabled_ = false;
    bool initialized_ = false;
    int debounce_time_ms_ = 50;
    rclcpp::Logger logger_;
    
    // 内部处理函数
    void process_warehouse_logic(const DI_Interface& di_signals);
    void process_di_signal(const std::string& di_name, DIConfig& config, bool current_state);
    void add_command(const ControlAction& action);
    bool check_debounce(DIConfig& config, int64_t current_time);
    
    /**
     * @brief 检查入库条件
     */
    bool check_entry_condition(const DI_Interface& di);
    bool check_exit_condition(const DI_Interface& di);
    bool check_completion_condition(const DI_Interface& di);

    // 延迟停止相关变量
    int delay_counter_ = 0;
    bool delay_started_ = false;
    bool delay_condition_triggered_ = false;  // 新增：标记条件是否已触发
    static constexpr int DELAY_BEFORE_STOP_MS = 5000;  // 5秒延迟
    static constexpr int DELAY_COUNTER_MAX = DELAY_BEFORE_STOP_MS / 100;  // 每100ms计数一次
};

#endif // BUSINESS_LOGIC_PROCESSOR_HPP