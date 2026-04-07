// LayerCommandProcessor.cpp
#include "LayerCommandProcessor.hpp"
#include "globals.h"  // 引入全局变量以检查自动模式状态
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <cmath>  // for std::round

LayerCommandProcessor::LayerCommandProcessor(rclcpp::Node* node) 
    : node_(node), axis5_index_(0), current_layer_(1), target_layer_(1), 
      is_moving_(false), current_layer_float_(1.0), motion_speed_(10.0), motion_acceleration_(50.0) {
    
    // 创建位移指令发布器，使用String类型
    displacement_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/displacement_command", rclcpp::QoS(10).reliable());
    
    // 初始化层高配置
    initialize_default_layer_heights();
}

void LayerCommandProcessor::initialize(size_t axis5_index) {
    axis5_index_ = axis5_index;
    RCLCPP_INFO(node_->get_logger(), 
                "层指令处理器初始化完成，axis5索引: %zu，支持层数范围: -20到+30", axis5_index_);
}

void LayerCommandProcessor::initialize_default_layer_heights() {
    // 默认配置：每层25mm间距，第1层为0mm
    layer_heights_.clear();

        // 默认配置：每层25mm间距，第1层为0mm
    for (int8_t layer = -20; layer <= 28; ++layer) {
        layer_heights_[layer] = (layer - 1) * 25.0;
    }
    RCLCPP_INFO(node_->get_logger(), "初始化默认层高配置，共%ld层", layer_heights_.size());
    RCLCPP_INFO(node_->get_logger(), "初始化默认层高配置，范围: -20到+30，共%d层", static_cast<int>(layer_heights_.size()));
}

void LayerCommandProcessor::process_layer_command(int8_t layer) {
    if (!validate_layer(layer)) {
        RCLCPP_ERROR(node_->get_logger(), "无效层指令: %d", layer);
        return;
    }
    
    if (is_moving_) {
        RCLCPP_WARN(node_->get_logger(), 
                   "轴正在运动中，覆盖层指令: %d -> %d", current_layer_, layer);
        // 不返回，继续执行新指令 return;
    }
    
    target_layer_ = layer;
    double target_height = calculate_layer_height(layer);
    
    is_moving_ = true;
    
    // 保存目标高度和状态
    pending_target_height_ = target_height;
    start_msg_published_.store(false);
    
    // 检查自动模式是否已初始化：如果是则立即执行，否则等待
    if (g_auto_mode_initialized.load()) {
        // 自动模式已就绪，立即执行
        RCLCPP_INFO(node_->get_logger(), 
                    "层指令处理(立即执行): 第%d层 -> 第%d层, 目标高度: %.2fmm", 
                    current_layer_, target_layer_, target_height);
        has_pending_command_.store(true);
        execute_pending_command();
    } else {
        // 自动模式未就绪，延迟到初始化完成后执行
        has_pending_command_.store(true);
        RCLCPP_INFO(node_->get_logger(), 
                    "层指令已记录(延迟执行): 第%d层 -> 第%d层, 目标高度: %.2fmm, 等待自动模式初始化完成", 
                    current_layer_, target_layer_, target_height);
    }
}

void LayerCommandProcessor::execute_pending_command() {
    if (!has_pending_command_.load()) {
        return;  // 没有待处理的命令
    }
    
    // 1. 发布层移动开始消息（如果还未发布）
    if (!start_msg_published_.load() && layer_completion_pub_) {
        auto msg = std_msgs::msg::Bool();
        msg.data = false;  // false表示移动开始/进行中
        layer_completion_pub_->publish(msg);
        RCLCPP_INFO(node_->get_logger(), "发布层移动开始消息: 第%d层 -> 第%d层", 
                   current_layer_, target_layer_);
        start_msg_published_.store(true);
    }
    
    // 2. 发布位移指令
    publish_displacement_command(pending_target_height_);
    RCLCPP_INFO(node_->get_logger(), "执行位移指令: axis5 -> %.2fmm", pending_target_height_);
    
    // 清除待处理标志（位移指令已发送，后续通过check_motion_completion检测完成）
    has_pending_command_.store(false);
}

bool LayerCommandProcessor::validate_layer(int8_t layer) {
    // 支持-20到+30的范围
    if (layer < -20 || layer > 30) {
        RCLCPP_ERROR(node_->get_logger(), "层数超出有效范围(-20到30): %d", layer);
        return false;
    }
    
    if (layer_heights_.find(layer) == layer_heights_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "无效层数: %d, 层高映射表大小: %zu", 
                    static_cast<int>(layer), layer_heights_.size());
        return false;
    }
    
    return true;
}

double LayerCommandProcessor::calculate_layer_height(int8_t layer) {
    auto it = layer_heights_.find(layer);
    if (it != layer_heights_.end()) {
        return it->second;
    }
    
    // 默认计算：每层25mm
    return (layer - 1) * 25.0;
}

void LayerCommandProcessor::publish_displacement_command(double axis5_target) {
    // 创建字符串位移指令
    auto msg = std_msgs::msg::String();
    msg.data = format_displacement_command(axis5_target);
    
    displacement_pub_->publish(msg);
    
    RCLCPP_DEBUG(node_->get_logger(), 
                "发布位移指令: %s", msg.data.c_str());
}

std::string LayerCommandProcessor::format_displacement_command(double axis5_target) {
    std::stringstream ss;
    ss << "axis5:" << std::fixed << std::setprecision(2) << axis5_target;
    return ss.str();
}

void LayerCommandProcessor::set_layer_heights(const std::map<int8_t, double>& layer_heights) {
    layer_heights_ = layer_heights;
    RCLCPP_INFO(node_->get_logger(), "更新层高配置，共%zu层", layer_heights_.size());
}

void LayerCommandProcessor::set_motion_parameters(double speed_mm_per_s, double acceleration_mm_per_s2) {
    motion_speed_ = speed_mm_per_s;
    motion_acceleration_ = acceleration_mm_per_s2;
    RCLCPP_INFO(node_->get_logger(), 
               "更新运动参数: 速度=%.1fmm/s, 加速度=%.1fmm/s²", 
               motion_speed_, motion_acceleration_);
}
// 新增：重置运动状态（用于stop命令后清理状态）
void LayerCommandProcessor::reset_motion_state() {
    if (is_moving_.load() || has_pending_command_.load()) {
        RCLCPP_INFO(node_->get_logger(), 
                   "层移动状态重置: 第%d层 -> 第%d层运动已取消，恢复时将重新触发", 
                   current_layer_, target_layer_);
        is_moving_ = false;
        has_pending_command_.store(false);
        start_msg_published_.store(false);
        
        // 发布层移动取消消息（false表示未完成）
        if (layer_completion_pub_) {
            auto msg = std_msgs::msg::Bool();
            msg.data = false;  // false表示移动未完成/被取消
            layer_completion_pub_->publish(msg);
        }
    }
    // 注意：不重置 target_layer_，保留目标层信息用于恢复时重新触发
}

// 优化后的 check_motion_completion 方法
bool LayerCommandProcessor::check_motion_completion(const std::shared_ptr<ServoAxisBase>& axis5) {
    if (!is_moving_) {
        return false;
    }
    
    // 关键修复：如果还有待处理的命令（等待自动模式初始化），不检查完成
    // 防止自动模式初始化时的 target_reached_flag_ 被误判为层移动完成
    if (has_pending_command_.load()) {
        return false;
    }
    
    if (axis5 && axis5->check_target_reached_flag()) {
        // +++ 优化：用 axis5 实际位置计算当前层号（更准确）+++
        // 而不是简单地设为 target_layer，避免实际位置与目标层不匹配
        double position_before = current_layer_float_;
        
        // 关键修复：重新读取当前脉冲，确保数据新鲜
        // 注意：displacement_to_pulses 返回的是绝对脉冲位置，所以层号计算也用绝对位置
        int32_t current_pulses = axis5->get_actual_position();
        const double SCREW_LEAD = 10.0;
        const int PULSES_PER_REV = 10000;
        // 直接使用 current_pulses 计算绝对位置（与 displacement_to_pulses 一致）
        double position_mm = static_cast<double>(current_pulses * SCREW_LEAD / PULSES_PER_REV);
        double fresh_layer_float = calculate_layer_from_position(position_mm);
        
        // 使用 freshly 计算的层号
        current_layer_float_ = fresh_layer_float;
        
        // 验证实际位置是否接近目标层（容差0.5层）
        double target_layer_float = static_cast<double>(target_layer_);
        double layer_diff = std::abs(current_layer_float_ - target_layer_float);
        
        if (layer_diff > 0.1) {
            // 实际位置与目标层偏差较大，警告但继续完成
            RCLCPP_WARN(node_->get_logger(), 
                        "层移动完成警告: 目标层=%.1f, 实际层=%.2f, 偏差=%.2f层 (current=%d, mm=%.2f)",
                        target_layer_float, current_layer_float_, layer_diff,
                        current_pulses, position_mm);
        }
        
        // 使用实际位置计算的层号作为当前层（四舍五入）
        current_layer_ = static_cast<int8_t>(std::round(current_layer_float_));
        
        // 限制在有效范围
        if (current_layer_ < 1) current_layer_ = 1;
        if (current_layer_ > 28) current_layer_ = 28;
        
        is_moving_ = false;
        
        // 新增：发布层移动完成消息
        if (layer_completion_pub_) {
            auto msg = std_msgs::msg::Bool();
            msg.data = true;
            layer_completion_pub_->publish(msg);
            RCLCPP_INFO(node_->get_logger(), 
                        "发布层移动完成消息: 目标第%d层 -> 实际第%.2f层", 
                        target_layer_, current_layer_float_);
        }
        
        RCLCPP_INFO(node_->get_logger(), 
                    "层指令执行完成: 目标第%d层 -> 实际第%.2f层(整型第%d层)", 
                    target_layer_, current_layer_float_, current_layer_);
        return true;
    }
    
    return false;
}

// 新增：从axis5实际位置计算当前层号（支持小数层）
double LayerCommandProcessor::calculate_layer_from_position(double position_mm) {
    // 层高公式: height = (layer - 1) * 25.0
    // 反推: layer = height / 25.0 + 1
    double layer = position_mm / 25.0 + 1.0;
    return layer;
}

// 新增：更新当前层号（从axis5实际位置）
void LayerCommandProcessor::update_current_layer_from_axis5(const std::shared_ptr<ServoAxisBase>& axis5) {
    if (!axis5) {
        return;
    }
    
    // 获取axis5当前实际位置（脉冲）
    int32_t current_pulses = axis5->get_actual_position();
    
    // 计算绝对位置（mm）- 与 servo_axis_base::displacement_to_pulses 使用相同的参考系
    // displacement_to_pulses 返回的是绝对脉冲位置，所以这里也用绝对脉冲计算
    const double SCREW_LEAD = 10.0;  // 丝杠导程10mm
    const int PULSES_PER_REV = 10000;  // 每转脉冲数
    double position_mm = static_cast<double>(current_pulses * SCREW_LEAD / PULSES_PER_REV);
    
    // 计算层号（支持小数）
    double new_layer_float = calculate_layer_from_position(position_mm);
    
    // 调试日志：如果层号变化较大，打印详细信息
    if (std::abs(new_layer_float - current_layer_float_) > 0.5) {
        RCLCPP_INFO(node_->get_logger(), 
                    "层号更新: %.2f -> %.2f (current=%d, mm=%.2f)",
                    current_layer_float_, new_layer_float,
                    current_pulses, position_mm);
    }
    
    // 更新浮点层号
    current_layer_float_ = new_layer_float;
    
    // 同时更新整数层号（四舍五入）
    current_layer_ = static_cast<int8_t>(std::round(new_layer_float));
    
    // 限制在有效范围内
    if (current_layer_ < 1) current_layer_ = 1;
    if (current_layer_ > 28) current_layer_ = 28;
}

// 新增：校正层号（自动模式初始化完成后调用）
void LayerCommandProcessor::calibrate_layer_from_position(const std::shared_ptr<ServoAxisBase>& axis5) {
    if (!axis5) {
        return;
    }
    
    // 更新层号
    update_current_layer_from_axis5(axis5);
    
    // 使用与位移指令相同的绝对位置计算
    int32_t current_pulses = axis5->get_actual_position();
    double position_mm = static_cast<double>(current_pulses * 10.0 / 10000.0);
    
    RCLCPP_INFO(node_->get_logger(), 
                "层号校正完成: 位置=%.2fmm, 层号=%.2f(浮点), %d(整型)",
                position_mm, current_layer_float_, current_layer_);
}