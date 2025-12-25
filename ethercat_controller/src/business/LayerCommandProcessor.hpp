// LayerCommandProcessor.hpp
#ifndef LAYER_COMMAND_PROCESSOR_HPP
#define LAYER_COMMAND_PROCESSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <map>
#include <atomic>
#include <memory>

/**
 * @brief 层指令处理器
 * 负责接收层指令并控制axis5运动到指定层数
 */
class LayerCommandProcessor {
public:
    explicit LayerCommandProcessor(rclcpp::Node* node);
    ~LayerCommandProcessor() = default;
    
    /**
     * @brief 初始化层指令处理器
     * @param axis5_index axis5在位移指令数组中的索引
     */
    void initialize(size_t axis5_index);
    
    /**
     * @brief 处理层指令
     * @param layer 目标层数(1-based)
     */
    void process_layer_command(uint8_t layer);
    
    /**
     * @brief 获取当前层数
     */
    uint8_t get_current_layer() const { return current_layer_; }
    
    /**
     * @brief 获取目标层数
     */
    uint8_t get_target_layer() const { return target_layer_; }
    
    /**
     * @brief 检查是否正在运动
     */
    bool is_moving() const { return is_moving_; }
    
    /**
     * @brief 生成位移指令消息
     */
    std_msgs::msg::Float64MultiArray create_displacement_command(
        const std::vector<double>& base_displacements);
    
    /**
     * @brief 设置层高配置
     * @param layer_heights 各层高度映射表(层号->高度mm)
     */
    void set_layer_heights(const std::map<uint8_t, double>& layer_heights);
    
    /**
     * @brief 设置运动参数
     * @param speed_mm_per_s 运动速度(mm/s)
     * @param acceleration_mm_per_s2 加速度(mm/s²)
     */
    void set_motion_parameters(double speed_mm_per_s, double acceleration_mm_per_s2);

private:
    rclcpp::Node* node_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr displacement_pub_;
    
    std::map<uint8_t, double> layer_heights_;  // 层高映射表
    size_t axis5_index_;                        // axis5索引
    uint8_t current_layer_;                     // 当前层数
    uint8_t target_layer_;                      // 目标层数
    std::atomic<bool> is_moving_;               // 运动状态标志
    
    // 运动参数
    double motion_speed_;          // 运动速度(mm/s)
    double motion_acceleration_;   // 加速度(mm/s²)
    
    /**
     * @brief 初始化默认层高配置
     */
    void initialize_default_layer_heights();
    
    /**
     * @brief 计算层高
     * @param layer 层数(1-based)
     * @return 对应高度(mm)
     */
    double calculate_layer_height(uint8_t layer);
    
    /**
     * @brief 验证层数有效性
     */
    bool validate_layer(uint8_t layer);
    
    /**
     * @brief 发布位移指令
     */
    void publish_displacement_command(double axis5_target);
};

#endif // LAYER_COMMAND_PROCESSOR_HPP