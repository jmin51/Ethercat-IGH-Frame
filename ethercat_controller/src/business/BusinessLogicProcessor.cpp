#include "BusinessLogicProcessor.hpp"
#include <chrono>
#include <algorithm>

#define DELAY_BEFORE_STOP_MS 30000  // 30秒延迟
using namespace std::chrono;

BusinessLogicProcessor::BusinessLogicProcessor(rclcpp::Logger logger) 
    : logger_(logger) {
    RCLCPP_INFO(logger_, "业务逻辑处理器创建完成");
}

void BusinessLogicProcessor::initialize(size_t num_axes) {
    if (initialized_) {
        RCLCPP_WARN(logger_, "业务逻辑处理器已初始化，跳过重复初始化");
        return;
    }
    
    RCLCPP_INFO(logger_, "初始化业务逻辑处理器，系统支持 %zu 个伺服轴", num_axes);
    initialized_ = true;
}

void BusinessLogicProcessor::process_io_signals(const DI_Interface& di_signals) {
    /// 主处理函数：分析DI信号变化并生成相应的轴控制命令
    if (!enabled_ || !initialized_) {
        RCLCPP_DEBUG(logger_, "业务逻辑处理器未启用或未初始化，跳过处理");
        return;
    }
    
    // 清空旧的待处理命令(确保每次处理都是最新的)
    pending_commands_.clear();
    
    // 处理入库业务流程
    process_warehouse_logic(di_signals);
    
    RCLCPP_DEBUG(logger_, "本次IO信号处理生成 %zu 个控制命令", pending_commands_.size());
}

void BusinessLogicProcessor::process_warehouse_logic(const DI_Interface& di_signals) {
    bool buffer_in = di_signals.buffer_in_position;  // DI14
    bool buffer_out = di_signals.buffer_out_position; // DI15
    bool conveyor_in = di_signals.conveyor_in_position;  // DI14
    bool conveyor_out = di_signals.conveyor_out_position; // DI15

    // 固定目标层为2
    target_layer_ = 2;
    
    RCLCPP_DEBUG(logger_, "入库流程状态: %d, DI14: %s, DI15: %s, 当前层: %d, 目标层: %d",
                static_cast<int>(warehouse_state_),
                conveyor_in ? "到位" : "未到位",
                conveyor_out ? "到位" : "未到位",
                current_layer_, target_layer_);
    
    switch (warehouse_state_) {
        case WarehouseState::IDLE:
            if (!buffer_in && !buffer_out && !conveyor_in && !conveyor_out) {
                warehouse_state_ = WarehouseState::WAIT_FOR_ENTRY;
                RCLCPP_INFO(logger_, "进入等待入库状态");
            }
            break;
            
        case WarehouseState::WAIT_FOR_ENTRY:
            // 检测入库条件：DI12到位且DI13未到位
            if (buffer_in && !buffer_out) {
                warehouse_state_ = WarehouseState::CONVEYOR_MOVING;
                RCLCPP_INFO(logger_, "检测到入库条件，开始输送");
                
                // 生成输送带启动命令
                ControlAction conv_action;
                conv_action.type = CommandType::JOG;
                conv_action.axis_name = "axis1_1";
                conv_action.command_value = "forward";
                conv_action.description = "启动轴1_1正转";
                add_command(conv_action);
                
                ControlAction conv_action2;
                conv_action2.type = CommandType::JOG;
                conv_action2.axis_name = "axis1_2";
                conv_action2.command_value = "forward";
                conv_action2.description = "启动轴1_2正转";
                add_command(conv_action2);
                
                // 移动到第1层等待（固定）
                ControlAction layer_action;
                layer_action.type = CommandType::LAYER;
                layer_action.axis_name = "axis5";
                layer_action.command_value = "1";  // 固定为第1层
                layer_action.description = "移动到第1层等待";
                add_command(layer_action);
            }
            break;
            
        case WarehouseState::CONVEYOR_MOVING:
            // 检测出料到位
            if (conveyor_in && buffer_out) {
                write_single_do_signal(812, true); // 激活DO13皮带正转
                RCLCPP_INFO(logger_, "检测到运行至:缓存架出料和接驳台入料位，等待提升机运行");
            }
            // 继续输送直到重新检测到入库条件
            if (conveyor_out) {
                // 停止输送带
                ControlAction stop_action;
                stop_action.type = CommandType::JOG;
                stop_action.axis_name = "axis1_1";
                stop_action.command_value = "stop";
                stop_action.description = "停止轴1_1";
                add_command(stop_action);
                
                ControlAction stop_action2;
                stop_action2.type = CommandType::JOG;
                stop_action2.axis_name = "axis1_2";
                stop_action2.command_value = "stop";
                stop_action2.description = "停止轴1_2";
                add_command(stop_action2);
                
                // 移动到目标层（固定为2层）
                ControlAction target_layer_action;
                target_layer_action.type = CommandType::LAYER;
                target_layer_action.axis_name = "axis5";
                target_layer_action.command_value = "2";  // 固定为第2层
                target_layer_action.description = "移动到目标层2";
                add_command(target_layer_action);
                
                write_single_do_signal(812, false); // 停止DO13皮带正转
                warehouse_state_ = WarehouseState::LIFT_MOVING;
            }

            break;
            
        case WarehouseState::LIFT_MOVING:
            // 检测入库传感器未到位
            if (current_layer_ == target_layer_) {
                RCLCPP_INFO(logger_, "接驳台已到达第%d层", target_layer_);
                
                // 启动轴2反转
                ControlAction axis2_action;
                axis2_action.type = CommandType::JOG;
                axis2_action.axis_name = "axis2_1";
                axis2_action.command_value = "reverse";
                axis2_action.description = "启动轴2_1反转";
                add_command(axis2_action);
                
                ControlAction axis2_action2;
                axis2_action2.type = CommandType::JOG;
                axis2_action2.axis_name = "axis2_2";
                axis2_action2.command_value = "reverse";
                axis2_action2.description = "启动轴2_2反转";
                add_command(axis2_action2);
                
                // 检测停止条件（只需检测一次）
                if (!delay_started_ && !delay_condition_triggered_ && conveyor_in && !conveyor_out) {
                    delay_condition_triggered_ = true;  // 标记条件已触发
                    delay_started_ = true;             // 开始延迟
                    delay_counter_ = 0;                // 重置计数器
                    RCLCPP_INFO(logger_, "检测到停止条件，开始%d秒延迟", DELAY_BEFORE_STOP_MS/1000);
                }
                
                // 处理延迟逻辑
                if (delay_started_) {
                    delay_counter_++;
                    
                    if (delay_counter_ >= DELAY_COUNTER_MAX) {
                        // 延迟结束，执行停止操作
                        warehouse_state_ = WarehouseState::COMPLETED;
                        delay_started_ = false;
                        delay_condition_triggered_ = false;
                        
                        // 停止轴2
                        ControlAction stop_action;
                        stop_action.type = CommandType::JOG;
                        stop_action.axis_name = "axis2_1";
                        stop_action.command_value = "stop";
                        stop_action.description = "轴2_1停止";
                        add_command(stop_action);
                        
                        ControlAction stop_action2;
                        stop_action2.type = CommandType::JOG;
                        stop_action2.axis_name = "axis2_2";
                        stop_action2.command_value = "stop";
                        stop_action2.description = "轴2_2停止";
                        add_command(stop_action2);
                        
                        RCLCPP_INFO(logger_, "延迟结束，停止轴2并进入完成状态");
                    } else {
                        // 延迟中，每5秒记录一次剩余时间
                        if (delay_counter_ % 50 == 0) { // 每5秒记录一次
                            int remaining_seconds = DELAY_BEFORE_STOP_MS/1000 - delay_counter_/10;
                            RCLCPP_INFO(logger_, "延迟剩余时间: %d秒", remaining_seconds);
                        }
                    }
                }
            } else {
                // 如果层条件不满足，重置延迟状态
                if (delay_started_) {
                    delay_started_ = false;
                    delay_condition_triggered_ = false;
                    RCLCPP_DEBUG(logger_, "层条件不满足，重置延迟状态");
                }
            }
            break;
            
        case WarehouseState::COMPLETED:
            // 回到第1层（固定） 看下放这还是放下面
            ControlAction return_action;
            return_action.type = CommandType::LAYER;
            return_action.axis_name = "axis5";
            return_action.command_value = "1";  // 固定为第1层
            return_action.description = "入库流程完成,回到第1层";
            add_command(return_action);
            if (!buffer_in && !buffer_out && !conveyor_in && !conveyor_out && (current_layer_ == 1)) {
                warehouse_state_ = WarehouseState::IDLE;
                RCLCPP_INFO(logger_, "回到初始状态，等待下一次入库");
            }
            break;
    }
}

void BusinessLogicProcessor::add_command(const ControlAction& action) {
    AxisCommand cmd;
    cmd.type = action.type;
    cmd.axis_name = action.axis_name;
    cmd.command_value = action.command_value;
    cmd.target_position = action.target_position;
    cmd.immediate = true;
    
    pending_commands_.push_back(cmd);
    
    RCLCPP_DEBUG(logger_, "生成控制命令: 类型=%d, 轴=%s, 值=%s, 描述=%s", 
                static_cast<int>(action.type), action.axis_name.c_str(),
                action.command_value.c_str(), action.description.c_str());
}

// 其他现有函数保持不变...
void BusinessLogicProcessor::process_di_signal(const std::string& di_name, 
                                               DIConfig& config, 
                                               bool current_state) {
    // 保持现有逻辑，但主要逻辑已移到process_warehouse_logic
    auto now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    if (!check_debounce(config, now_ms)) {
        return;
    }
    
    // 简单的DI触发逻辑（备用）
    if (current_state != config.last_state) {
        for (const auto& action : config.actions) {
            add_command(action);
        }
    }
    
    config.last_state = current_state;
}

std::vector<AxisCommand> BusinessLogicProcessor::get_pending_commands() {
    /// 获取当前所有待执行的轴控制命令
    return pending_commands_;
}

void BusinessLogicProcessor::clear_pending_commands() {
    /// 清空命令队列(通常在命令被执行后调用)
    pending_commands_.clear();
    RCLCPP_DEBUG(logger_, "清空待处理命令队列");
}

std::vector<std::string> BusinessLogicProcessor::get_configured_di_signals() const {
    std::vector<std::string> signals;
    for (const auto& [di_name, _] : di_configs_) {
        signals.push_back(di_name);
    }
    return signals;
}

bool BusinessLogicProcessor::check_debounce(DIConfig& config, int64_t current_time) {
    int64_t time_since_last = current_time - config.last_trigger_time;
    if (time_since_last < debounce_time_ms_) {
        return false;
    }
    config.last_trigger_time = current_time;
    return true;
}