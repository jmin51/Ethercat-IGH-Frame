// LayerCommandProcessor.hpp
#ifndef LAYER_COMMAND_PROCESSOR_HPP
#define LAYER_COMMAND_PROCESSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>  // 改为String
#include <std_msgs/msg/bool.hpp>  // 新增Bool类型
#include <map>
#include <atomic>
#include <memory>
#include "servo_axis_base.hpp"  // 添加这行

/**
 * @brief 层指令处理器
 * 负责接收层指令并控制axis5运动到指定层数
 */
class LayerCommandProcessor {
public:
    explicit LayerCommandProcessor(rclcpp::Node* node);
    ~LayerCommandProcessor() = default;
    
    // 初始化层指令处理器 axis5在位移指令数组中的索引
    void initialize(size_t axis5_index);
    
    // 处理层指令 标层数(1-based)
    void process_layer_command(int8_t layer);
    
    // 获取当前层数
    int8_t get_current_layer() const { return current_layer_; }
    
    // 获取目标层数
    int8_t get_target_layer() const { return target_layer_; }
    
    // 检查是否正在运动
    bool is_moving() const { return is_moving_; }
    
    // 设置层高配置 各层高度映射表(层号->高度mm)
    void set_layer_heights(const std::map<int8_t, double>& layer_heights);

    // 设置运动参数 speed_mm_per_s 运动速度(mm/s) acceleration_mm_per_s2 加速度(mm/s²)
    void set_motion_parameters(double speed_mm_per_s, double acceleration_mm_per_s2);
    bool check_motion_completion(const std::shared_ptr<ServoAxisBase>& axis5);  // 检查运动是否完成（基于axis5的标志位）

    // 新增：设置层移动完成发布器
    void set_layer_completion_publisher(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub) {
        layer_completion_pub_ = pub;
    }
    
    // 新增：重置运动状态（用于stop命令后清理状态）
    void reset_motion_state();

    // 新增：检查是否有待执行的命令
    bool has_pending_command() const { return has_pending_command_.load(); }
    
    // 新增：执行待处理的位移命令（在自动模式初始化完成后调用）
    void execute_pending_command();

    // 新增：从axis5实际位置计算当前层号（支持小数层）
    double calculate_layer_from_position(double position_mm);
    
    // 新增：更新当前层号（从axis5实际位置）
    void update_current_layer_from_axis5(const std::shared_ptr<ServoAxisBase>& axis5);
    
    // 新增：获取当前层号（支持小数）
    double get_current_layer_float() const { return current_layer_float_; }
    
    // 新增：校正层号（自动模式初始化完成后调用）
    void calibrate_layer_from_position(const std::shared_ptr<ServoAxisBase>& axis5);

private:
    rclcpp::Node* node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr displacement_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr layer_completion_pub_;  // 新增层移动完成发布器
    
    std::map<int8_t, double> layer_heights_;  // 层高映射表
    size_t axis5_index_;                        // axis5索引
    int8_t current_layer_;                     // 当前层数
    int8_t target_layer_;                      // 目标层数
    std::atomic<bool> is_moving_;               // 运动状态标志
    double current_layer_float_;                // 当前层号（支持小数，如5.5层）
    
    // 运动参数
    double motion_speed_;          // 运动速度(mm/s)
    double motion_acceleration_;   // 加速度(mm/s²)

    // 待处理命令（用于延迟到自动模式初始化完成后执行）
    std::atomic<bool> has_pending_command_{false};  // 是否有待处理的命令
    double pending_target_height_{0.0};             // 待发送的目标高度
    std::atomic<bool> start_msg_published_{false};  // 开始消息是否已发布

    // 初始化默认层高配置
    void initialize_default_layer_heights();

    // 计算层高 layer 层数(1-based) 对应高度(mm)
    double calculate_layer_height(int8_t layer);

    // 验证层数有效性
    bool validate_layer(int8_t layer);

    // 发布位移指令
    void publish_displacement_command(double axis5_target);
    
    // 新增：格式化位移指令为字符串
    std::string format_displacement_command(double axis5_target);
};

#endif // LAYER_COMMAND_PROCESSOR_HPP