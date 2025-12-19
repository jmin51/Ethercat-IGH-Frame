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
    
    // 可以在这里添加默认配置
    setup_standard_configurations();
}

void BusinessLogicProcessor::setup_standard_configurations() {
    RCLCPP_INFO(logger_, "设置标准业务逻辑配置");
    
    // 示例1: DI14控制axis1 - 接驳台入料到位信号逻辑
    ControlAction axis1_to_05 = {"axis1", 0.0, 1000, "DI14未到位，轴1移动到0.5mm位置"};
    ControlAction axis1_to_00 = {"axis1", 0.5, 1000, "DI14到位，轴1回到0.0mm原点"};
    
    configure_di_signal(
        "DI14_接驳台入料到位",
        [](const DI_Interface& di) { return di.conveyor_in_position; },
        {},  // 上升沿不触发
        {},  // 下降沿不触发
        axis1_to_00,  // 高电平(到位)→回原点
        axis1_to_05   // 低电平(未到位)→移动到工作位置
    );
    
    // // 示例2: DI05急停信号 - 安全优先逻辑
    // ControlAction emergency_stop = {"all", 0.0, 0, "急停激活，所有轴立即回零"};
    // configure_di_signal(
    //     "DI05_急停",
    //     [](const DI_Interface& di) { return di.emergency_stop; },
    //     emergency_stop,  // 急停激活(上升沿)→立即停止所有轴
    //     {},              // 急停恢复时不特殊处理
    //     {},              // 高电平不处理
    //     {}               // 低电平不处理
    // );
    
    // // 示例3: DI04手自动模式切换信号
    // ControlAction auto_mode = {"all", 0.0, 0, "切换到自动模式"};
    // ControlAction manual_mode = {"all", 0.0, 0, "切换到手动模式"};
    // configure_di_signal(
    //     "DI04_手自动切换",
    //     [](const DI_Interface& di) { return di.manual_auto_button; },
    //     {},          // 上升沿不处理
    //     {},          // 下降沿不处理
    //     auto_mode,   // 高电平→自动模式
    //     manual_mode  // 低电平→手动模式
    // );
    
    RCLCPP_INFO(logger_, "标准业务逻辑配置完成");
}

void BusinessLogicProcessor::configure_di_signal(
    const std::string& di_name,
    std::function<bool(const DI_Interface&)> getter,
    const ControlAction& rising_action,
    const ControlAction& falling_action,
    const ControlAction& high_action,
    const ControlAction& low_action) {
    
    DIConfig config;
    config.di_name = di_name;
    config.getter = getter;
    config.on_rising_edge = rising_action;
    config.on_falling_edge = falling_action;
    config.on_high_level = high_action;
    config.on_low_level = low_action;
    
    di_configs_[di_name] = config;
    
    // 记录配置信息
    RCLCPP_INFO(logger_, "DI信号配置: %s", di_name.c_str());
    if (!rising_action.axis_name.empty()) {
        RCLCPP_INFO(logger_, "  ↑ 上升沿: %s → %.3fmm (%s)", 
                   rising_action.axis_name.c_str(), rising_action.target_position,
                   rising_action.description.c_str());
    }
    if (!falling_action.axis_name.empty()) {
        RCLCPP_INFO(logger_, "  ↓ 下降沿: %s → %.3fmm (%s)", 
                   falling_action.axis_name.c_str(), falling_action.target_position,
                   falling_action.description.c_str());
    }
    if (!high_action.axis_name.empty()) {
        RCLCPP_INFO(logger_, "  ━ 高电平: %s → %.3fmm (%s)", 
                   high_action.axis_name.c_str(), high_action.target_position,
                   high_action.description.c_str());
    }
    if (!low_action.axis_name.empty()) {
        RCLCPP_INFO(logger_, "  ━ 低电平: %s → %.3fmm (%s)", 
                   low_action.axis_name.c_str(), low_action.target_position,
                   low_action.description.c_str());
    }
}

void BusinessLogicProcessor::process_io_signals(const DI_Interface& di_signals) {
    /// 主处理函数：分析DI信号变化并生成相应的轴控制命令
    if (!enabled_ || !initialized_) {
        RCLCPP_DEBUG(logger_, "业务逻辑处理器未启用或未初始化，跳过处理");
        return;
    }
    
    // 清空旧的待处理命令(确保每次处理都是最新的)
    pending_commands_.clear();
    
    // 获取当前时间戳(用于去抖判断)
    // auto now = system_clock::now();
    // auto now_ms = duration_cast<milliseconds>(now.time_since_epoch()).count();
    
    // 遍历所有配置的DI信号，逐个处理
    for (auto& [di_name, config] : di_configs_) {
        try {
            bool current_state = config.getter(di_signals);
            process_di_signal(di_name, config, current_state);
            config.last_state = current_state;  // 更新状态记录
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "处理DI信号 %s 时发生异常: %s", 
                        di_name.c_str(), e.what());
        }
    }
    
    RCLCPP_DEBUG(logger_, "本次IO信号处理生成 %zu 个控制命令", pending_commands_.size());
}

void BusinessLogicProcessor::process_di_signal(const std::string& di_name, 
                                               DIConfig& config, 
                                               bool current_state) {
    /// 处理单个DI信号的逻辑
    RCLCPP_DEBUG(logger_, "处理DI信号: %s, 当前状态: %s", 
                di_name.c_str(), current_state ? "高电平" : "低电平");
    
    // 步骤1: 检查去抖(防止信号抖动导致的误触发)
    auto now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    if (!check_debounce(config, now_ms)) {
        RCLCPP_DEBUG(logger_, "DI信号 %s 处于去抖期，忽略处理", di_name.c_str());
        return;
    }
    
    // 步骤2: 检测状态变化并触发相应动作
    bool state_changed = (current_state != config.last_state);
    
    if (state_changed) {
        // 边沿触发逻辑
        if (current_state) {
            // 上升沿触发(低电平→高电平)
            if (!config.on_rising_edge.axis_name.empty()) {
                RCLCPP_INFO(logger_, "DI信号 %s 检测到上升沿，触发动作", di_name.c_str());
                add_command_if_valid(config.on_rising_edge);
            }
        } else {
            // 下降沿触发(高电平→低电平)
            if (!config.on_falling_edge.axis_name.empty()) {
                RCLCPP_INFO(logger_, "DI信号 %s 检测到下降沿，触发动作", di_name.c_str());
                add_command_if_valid(config.on_falling_edge);
            }
        }
    } else {
        // 电平触发逻辑(状态未变化，但需要持续检测电平)
        if (current_state && !config.on_high_level.axis_name.empty()) {
            // 高电平触发
            add_command_if_valid(config.on_high_level);
        } else if (!current_state && !config.on_low_level.axis_name.empty()) {
            // 低电平触发
            add_command_if_valid(config.on_low_level);
        }
    }
}

void BusinessLogicProcessor::add_command_if_valid(const ControlAction& action) {
    /// 验证并添加控制命令到待执行队列
    if (!action.axis_name.empty()) {
        AxisCommand cmd;
        cmd.axis_name = action.axis_name;
        cmd.target_position = action.target_position;
        cmd.immediate = true;  // 默认立即执行
        
        pending_commands_.push_back(cmd);
        
        RCLCPP_DEBUG(logger_, "生成控制命令: %s → %.3fmm (%s)", 
                    action.axis_name.c_str(), action.target_position,
                    action.description.c_str());
    }
}

bool BusinessLogicProcessor::check_debounce(DIConfig& config, int64_t current_time) {
    /// 去抖检查: 防止在短时间内重复触发
    int64_t time_since_last = current_time - config.last_trigger_time;
    if (time_since_last < debounce_time_ms_) {
        RCLCPP_DEBUG(logger_, "去抖检查: 距离上次触发仅 %ldms < %dms，忽略", 
                    time_since_last, debounce_time_ms_);
        return false;
    }
    
    config.last_trigger_time = current_time;
    return true;
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
    /// 获取所有已配置的DI信号名称(用于状态监控和调试)
    std::vector<std::string> signals;
    for (const auto& [di_name, _] : di_configs_) {
        signals.push_back(di_name);
    }
    return signals;
}