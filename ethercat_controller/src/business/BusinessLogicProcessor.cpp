#include "BusinessLogicProcessor.hpp"
#include <chrono>
#include <algorithm>

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
    if (!enabled_ || !initialized_ || !auto_mode_enabled_) {
        RCLCPP_DEBUG(logger_, "业务逻辑处理器未启用或未初始化或未在自动模式，跳过处理");
        return;
    }
    
    // 清空旧的待处理命令(确保每次处理都是最新的)
    pending_commands_.clear();
    
    // 处理入库业务流程
    process_warehouse_logic(di_signals);
    // 处理出库业务流程
    process_outbound_logic(di_signals);
    
    RCLCPP_DEBUG(logger_, "本次IO信号处理生成 %zu 个控制命令", pending_commands_.size());
}

void BusinessLogicProcessor::start_warehouse_process(uint8_t target_layer) {
    if (warehouse_state_ != WarehouseState::IDLE) {
        RCLCPP_WARN(logger_, "入库流程已在运行中，无法重复启动");
        return;
    }
    
    target_layer_ = target_layer;
    warehouse_process_requested_ = true;
    warehouse_process_stop_requested_ = false;
    
    RCLCPP_INFO(logger_, "收到入库流程启动请求，目标层: %d", target_layer_);
}

void BusinessLogicProcessor::stop_warehouse_process() {
    warehouse_process_stop_requested_ = true;
    RCLCPP_INFO(logger_, "收到入库流程停止请求");
}

void BusinessLogicProcessor::process_warehouse_logic(const DI_Interface& di_signals) {
    bool buffer_in = di_signals.buffer_in_position;  // DI14
    bool buffer_out = di_signals.buffer_out_position; // DI15
    bool conveyor_in = di_signals.conveyor_in_position;  // DI14
    bool conveyor_out = di_signals.conveyor_out_position; // DI15
    
    RCLCPP_DEBUG(logger_, "入库流程状态: %d, DI14: %s, DI15: %s, 当前层: %d, 目标层: %d",
                static_cast<int>(warehouse_state_),
                conveyor_in ? "到位" : "未到位",
                conveyor_out ? "到位" : "未到位",
                current_layer_, target_layer_);
    
    // 处理停止请求
    if (warehouse_process_stop_requested_) {
        warehouse_state_ = WarehouseState::IDLE;
        warehouse_process_stop_requested_ = false;
        warehouse_process_requested_ = false;
        RCLCPP_INFO(logger_, "入库流程已停止");
        return;
    }

    switch (warehouse_state_) {
        case WarehouseState::IDLE:
            // 等待话题启动信号
            if (warehouse_process_requested_ && !buffer_in && !buffer_out && !conveyor_in && !conveyor_out) {
                warehouse_state_ = WarehouseState::WAIT_FOR_ENTRY;
                warehouse_process_requested_ = false; // 重置请求标志
                RCLCPP_INFO(logger_, "入库流程启动，进入等待入库状态，目标层: %d", target_layer_);
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
                conv_action.command_value = "reverse";
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
            if (buffer_out) {
                write_single_do_signal(813, true); // 激活DO14皮带反转
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
                target_layer_action.command_value = std::to_string(target_layer_);  // 使用target_layer_
                target_layer_action.description = "移动到目标层" + std::to_string(target_layer_);
                add_command(target_layer_action);
                
                write_single_do_signal(813, false); // 停止DO14皮带反转
                warehouse_state_ = WarehouseState::LIFT_MOVING;
            }

            break;
            
        case WarehouseState::LIFT_MOVING:
            // 检测入库传感器未到位
            if (current_layer_ == target_layer_) {
                RCLCPP_INFO(logger_, "接驳台已到达第%d层", target_layer_);
                
                write_single_do_signal(812, true); // 启动DO13皮带正转
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
                axis2_action2.command_value = "forward";
                axis2_action2.description = "启动轴2_2反转";
                add_command(axis2_action2);
                write_single_do_signal(811, true); // 激活DO气缸伸出信号

                // 检测信号变化：先检测到conveyor_in为1，再检测到为0
                if (!delay_started_ && !delay_condition_triggered_) {
                    if (conveyor_in) {
                        // 第一次检测到conveyor_in为1，标记条件已满足第一部分
                        if (!conveyor_in_detected_) {
                            conveyor_in_detected_ = true;
                            RCLCPP_INFO(logger_, "检测到货物进入接驳台(conveyor_in=1)");
                        }
                    } else if (conveyor_in_detected_) {
                        // 之前检测到过conveyor_in为1，现在检测到为0，触发延迟
                        delay_condition_triggered_ = true;
                        delay_started_ = true;
                        delay_counter_ = 0;
                        conveyor_in_detected_ = false; // 重置检测标志
                        RCLCPP_INFO(logger_, "检测到货物离开接驳台(conveyor_in=0)，开始%d秒延迟", DELAY_BEFORE_STOP_MS/1000);
                    }
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
                        write_single_do_signal(811, false); 
                        write_single_do_signal(812, false);
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
                // 如果层条件不满足，重置所有状态
                if (delay_started_) {
                    delay_started_ = false;
                    delay_condition_triggered_ = false;
                }
                conveyor_in_detected_ = false; // 重置检测标志
                RCLCPP_DEBUG(logger_, "层条件不满足，重置延迟状态");
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

void BusinessLogicProcessor::start_outbound_process(uint8_t source_layer) {
    if (outbound_state_ != OutboundState::IDLE) {
        RCLCPP_WARN(logger_, "出库流程已在运行中，无法重复启动");
        return;
    }
    
    source_layer_ = source_layer;
    outbound_process_requested_ = true;
    outbound_process_stop_requested_ = false;
    
    RCLCPP_INFO(logger_, "收到出库流程启动请求，源层: %d", source_layer_);
}

void BusinessLogicProcessor::stop_outbound_process() {
    outbound_process_stop_requested_ = true;
    RCLCPP_INFO(logger_, "收到出库流程停止请求");
}

void BusinessLogicProcessor::process_outbound_logic(const DI_Interface& di_signals) {
    bool buffer_in = di_signals.buffer_in_position;  // DI12
    bool buffer_out = di_signals.buffer_out_position; // DI13
    bool conveyor_in = di_signals.conveyor_in_position;  // DI14
    bool conveyor_out = di_signals.conveyor_out_position; // DI15
    
    RCLCPP_DEBUG(logger_, "出库流程状态: %d, DI12: %s, DI13: %s, DI14: %s, DI15: %s, 源层: %d",
                static_cast<int>(outbound_state_),
                buffer_in ? "到位" : "未到位",
                buffer_out ? "到位" : "未到位", 
                conveyor_in ? "到位" : "未到位",
                conveyor_out ? "到位" : "未到位",
                source_layer_);
    
    // 处理停止请求
    if (outbound_process_stop_requested_) {
        outbound_state_ = OutboundState::IDLE;
        outbound_process_stop_requested_ = false;
        outbound_process_requested_ = false;
        RCLCPP_INFO(logger_, "出库流程已停止");
        return;
    }

    switch (outbound_state_) {
        case OutboundState::IDLE:{
            // 等待话题启动信号
            if (outbound_process_requested_ && check_outbound_condition(di_signals)) {
                outbound_state_ = OutboundState::WAIT_FOR_EXIT;
                outbound_process_requested_ = false;
                RCLCPP_INFO(logger_, "出库流程启动，进入等待出库状态，源层: %d", source_layer_);
            }
            break;
        }    
        case OutboundState::WAIT_FOR_EXIT:{
            RCLCPP_INFO(logger_, "检测到出库条件，开始提升机运行");
            
            // 移动到源层
            ControlAction init_layer_action;
            init_layer_action.type = CommandType::LAYER;
            init_layer_action.axis_name = "axis5";
            init_layer_action.command_value = std::to_string(source_layer_);
            init_layer_action.description = "移动到源层" + std::to_string(source_layer_);
            add_command(init_layer_action);

            // 检测提升机到达源层
            if (current_layer_ == source_layer_) {
                RCLCPP_INFO(logger_, "提升机已到达第%d层", source_layer_);
                
                // 启动轴2正转（出库方向）
                ControlAction axis2_action;
                axis2_action.type = CommandType::JOG;
                axis2_action.axis_name = "axis2_1";
                axis2_action.command_value = "forward";
                axis2_action.description = "启动轴2_1正转（出库）";
                add_command(axis2_action);
                
                ControlAction axis2_action2;
                axis2_action2.type = CommandType::JOG;
                axis2_action2.axis_name = "axis2_2";
                axis2_action2.command_value = "reverse";
                axis2_action2.description = "启动轴2_2正转（出库）";
                add_command(axis2_action2);
                write_single_do_signal(811, true); // 激活DO信号
                
                outbound_state_ = OutboundState::LIFT_MOVING;
            }
            break;
        }    
        case OutboundState::LIFT_MOVING:{
            write_single_do_signal(813, true); // 激活DO14皮带反转
            
            // 检测货物到达接驳台 - 修改为检测信号变化：先conveyor_in为1，后conveyor_out为1
            if (!outbound_conveyor_in_detected_) {
                if (conveyor_in) {
                    // 第一次检测到conveyor_in为1
                    outbound_conveyor_in_detected_ = true;
                    RCLCPP_INFO(logger_, "检测到货物进入接驳台(conveyor_in=1)");
                }
            } else {
                // 已经检测到conveyor_in为1，现在等待conveyor_out为1
                if (conveyor_out) {
                    RCLCPP_INFO(logger_, "检测到货物到达缓存架出料位(conveyor_out=1)，停止输送带");
                    
                    // 执行停止操作
                    ControlAction stop_action;
                    stop_action.type = CommandType::JOG;
                    stop_action.axis_name = "axis2_1";
                    stop_action.command_value = "stop";
                    stop_action.description = "停止轴2_1";
                    add_command(stop_action);
                    
                    ControlAction stop_action2;
                    stop_action2.type = CommandType::JOG;
                    stop_action2.axis_name = "axis2_2";
                    stop_action2.command_value = "stop";
                    stop_action2.description = "停止轴2_2";
                    add_command(stop_action2);
                    write_single_do_signal(811, false); // 停止DO信号
                    
                    outbound_state_ = OutboundState::CONVEYOR_MOVING;
                    RCLCPP_INFO(logger_, "进入输送带运行状态");
                }
            }
            break;
        }

        case OutboundState::CONVEYOR_MOVING:{
            // 检测货物到达缓存架 先返回第一层
            ControlAction conveyor_layer_action;
            conveyor_layer_action.type = CommandType::LAYER;
            conveyor_layer_action.axis_name = "axis5";
            conveyor_layer_action.command_value = "1";  // 固定为第1层
            conveyor_layer_action.description = "移动到第1层等待";
            add_command(conveyor_layer_action);
            if (conveyor_out) {
                RCLCPP_INFO(logger_, "检测到货物到达缓存架出料位");
                write_single_do_signal(813, false);
                if (!outbound_delay_started_ && !outbound_delay_condition_triggered_ ) {
                    outbound_delay_condition_triggered_ = true;
                    outbound_delay_started_ = true;
                    outbound_delay_counter_ = 0;
                    RCLCPP_INFO(logger_, "检测到出库停止条件，开始%d秒延迟", DELAY_BEFORE_STOP_MS/1000);
                }
            }
            
            // 处理延迟逻辑
            if (outbound_delay_started_) {
                outbound_delay_counter_++;

                if (outbound_delay_counter_ >= DELAY_COUNTER_MAX) {
                    // 延迟结束，执行停止操作
                    outbound_delay_started_ = false;
                    outbound_delay_condition_triggered_ = false;
                    // write_single_do_signal(813, true);

                    outbound_state_ = OutboundState::COMPLETED;
                    RCLCPP_INFO(logger_, "延迟结束，停止输送带并进入完成状态");
                }
            }
            break;
        }    
        case OutboundState::COMPLETED:{
            // 回到第1层
            ControlAction completed_layer_action;
            completed_layer_action.type = CommandType::LAYER;
            completed_layer_action.axis_name = "axis5";
            completed_layer_action.command_value = "1";
            completed_layer_action.description = "出库流程完成，回到第1层";
            add_command(completed_layer_action);
            
            if (current_layer_ == 1 && check_outbound_completion_condition(di_signals)) {
                outbound_state_ = OutboundState::IDLE;
                // write_single_do_signal(813, false);
                RCLCPP_INFO(logger_, "出库流程完成，回到初始状态");
            }
            break;
        }
    }
}

bool BusinessLogicProcessor::check_outbound_condition(const DI_Interface& di) {
    // 出库启动条件：所有传感器都未到位
    return !di.buffer_in_position && !di.buffer_out_position && 
           !di.conveyor_in_position && !di.conveyor_out_position;
}

bool BusinessLogicProcessor::check_outbound_completion_condition(const DI_Interface& di) {
    // 出库完成条件：回到初始状态
    return !di.buffer_in_position && !di.buffer_out_position && 
           !di.conveyor_in_position && !di.conveyor_out_position;
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

// 自动模式控制相关
// 添加自动模式控制方法
void BusinessLogicProcessor::enable_auto_mode() {
    auto_mode_enabled_ = true;
    RCLCPP_INFO(logger_, "业务逻辑处理器进入自动模式");
}

void BusinessLogicProcessor::disable_auto_mode() {
    auto_mode_enabled_ = false;
    
    // 清空所有状态和命令
    reset_business_logic();
    
    RCLCPP_INFO(logger_, "业务逻辑处理器退出自动模式，已重置所有状态");
}

void BusinessLogicProcessor::reset_business_logic() {
    // 重置入库流程状态
    warehouse_state_ = WarehouseState::IDLE;
    warehouse_process_requested_ = false;
    warehouse_process_stop_requested_ = false;
    
    // 重置出库流程状态
    outbound_state_ = OutboundState::IDLE;
    outbound_process_requested_ = false;
    outbound_process_stop_requested_ = false;
    
    // 重置延迟计数器
    delay_started_ = false;
    delay_condition_triggered_ = false;
    delay_counter_ = 0;
    
    // 重置信号检测状态
    conveyor_in_detected_ = false;
    outbound_conveyor_in_detected_ = false;

    outbound_delay_started_ = false;
    outbound_delay_condition_triggered_ = false;
    outbound_delay_counter_ = 0;
    
    // 清空待处理命令
    pending_commands_.clear();
    
    RCLCPP_INFO(logger_, "业务逻辑处理器状态已重置");
}

bool BusinessLogicProcessor::is_auto_mode_enabled() const {
    return auto_mode_enabled_;
}