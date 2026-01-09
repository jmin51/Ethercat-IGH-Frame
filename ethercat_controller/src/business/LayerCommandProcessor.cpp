// LayerCommandProcessor.cpp
#include "LayerCommandProcessor.hpp"
#include <algorithm>
#include <sstream>
#include <iomanip>

LayerCommandProcessor::LayerCommandProcessor(rclcpp::Node* node) 
    : node_(node), axis5_index_(0), current_layer_(1), target_layer_(1), 
      is_moving_(false), motion_speed_(10.0), motion_acceleration_(50.0) {
    
    // 创建位移指令发布器，使用String类型
    displacement_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/displacement_command", rclcpp::QoS(10).reliable());
    
    // 初始化层高配置
    initialize_default_layer_heights();
}

void LayerCommandProcessor::initialize(size_t axis5_index) {
    axis5_index_ = axis5_index;
    RCLCPP_INFO(node_->get_logger(), 
                "层指令处理器初始化完成，axis5索引: %zu", axis5_index_);
}

void LayerCommandProcessor::initialize_default_layer_heights() {
    // 默认配置：每层25mm间距，第1层为0mm
    layer_heights_.clear();
    for (uint8_t layer = 1; layer <= 25; ++layer) {
        layer_heights_[layer] = (layer - 1) * 25.0;
    }
    RCLCPP_INFO(node_->get_logger(), "初始化默认层高配置，共%ld层", layer_heights_.size());
}

void LayerCommandProcessor::process_layer_command(uint8_t layer) {
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
    
    // RCLCPP_INFO(node_->get_logger(), 
    //            "执行层指令: %d -> %d, 目标高度: %.2fmm", 
    //            current_layer_, target_layer_, target_height);
    
    is_moving_ = true;
    
    // 发布位移指令
    publish_displacement_command(target_height);    // 位置要提前
    
    // 注意：实际运动完成检测需要在状态机中处理
    // 这里假设运动立即完成（实际需要等待轴到达目标位置）
    // RCLCPP_INFO(node_->get_logger(), "层指令执行完成: 到达第%d层", current_layer_);
}

bool LayerCommandProcessor::validate_layer(uint8_t layer) {
    if (layer < 1) {
        RCLCPP_ERROR(node_->get_logger(), "层数不能小于1: %d", layer);
        return false;
    }
    
    if (layer_heights_.find(layer) == layer_heights_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "无效层数: %d，可用层数: 1-%zu", 
                    layer, layer_heights_.size());
        return false;
    }
    
    return true;
}

double LayerCommandProcessor::calculate_layer_height(uint8_t layer) {
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

void LayerCommandProcessor::set_layer_heights(const std::map<uint8_t, double>& layer_heights) {
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

bool LayerCommandProcessor::check_motion_completion(const std::shared_ptr<ServoAxisBase>& axis5) {
    if (!is_moving_) {
        return false; // 没有运动在进行
    }
    
    // 检查标志位
    if (axis5 && axis5->check_target_reached_flag()) {
        // 运动完成
        current_layer_ = target_layer_;
        is_moving_ = false;
        
        RCLCPP_INFO(node_->get_logger(), 
                   "层指令执行完成: 到达第%d层", current_layer_);
        return true;
    }
    
    return false;
}