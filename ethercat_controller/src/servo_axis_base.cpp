#include "servo_axis_base.hpp"
#include <iostream>

ServoAxisBase::ServoAxisBase(const std::string& name, uint16_t position, 
                           AxisType axis_type, DriveBrand brand, uint32_t product_code)
    : axis_name_(name), slave_position_(position), axis_type_(axis_type), 
      brand_(brand), product_code_(product_code) {
    initialize_members();
}

void ServoAxisBase::initialize_members() {
    sc_ = nullptr;
    control_word_ = 0;
    status_word_ = 0;
    off_target_position_ = 0;
    off_actual_position_ = 0;
    off_error_code_ = 0;
    
    current_state_ = AxisState::UNINITIALIZED;
    operation_mode_ = OperationMode::AUTO;
    
    position_initialized_ = false;
    target_pulses_ = 0;
    target_displacement_ = 0.0;
    displacement_updated_ = false;
    joint_position_ = 0;
    initial_position_ = 0;
    target_reached_ = 0;
    
    homing_in_progress_ = false;
    homing_completed_ = false;
    
    start_manual_requested_ = false;
    start_auto_requested_ = false;
    clear_fault_requested_ = false;
    reset_requested_ = false;
    fault_clearing_in_progress_ = false;
    fault_clear_step_ = 0;
    fault_clear_counter_ = 0;

    // 添加速度控制初始化
    jog_speed_ = DEFAULT_JOG_SPEED;
    jog_forward_requested_ = false;
    jog_reverse_requested_ = false;
    jog_stop_requested_ = false;
}

// 基类的默认实现
void ServoAxisBase::set_target_displacement(double displacement) {
    target_displacement_ = displacement;
}

void ServoAxisBase::set_displacement_updated(bool updated) {
    displacement_updated_ = updated;
}

void ServoAxisBase::stop() {
    if (current_state_ == AxisState::MANUAL_MODE || current_state_ == AxisState::AUTO_MODE) {
        current_state_ = AxisState::STOPPED;
        std::cout << "轴 " << axis_name_ << " 进入停止状态" << std::endl;
    }
}

void ServoAxisBase::start_manual_mode() {
    if (current_state_ == AxisState::READY || current_state_ == AxisState::AUTO_MODE) {
        start_manual_requested_ = true;
        std::cout << "轴 " << axis_name_ << " 手动模式启动请求已设置" << std::endl;
    }
}

void ServoAxisBase::start_auto_mode() {
    if (current_state_ == AxisState::READY || current_state_ == AxisState::MANUAL_MODE) {
        start_auto_requested_ = true;
        std::cout << "轴 " << axis_name_ << " 自动模式启动请求已设置" << std::endl;
    }
}

void ServoAxisBase::clear_fault() {
    if (current_state_ == AxisState::FAULT) {
        clear_fault_requested_ = true;
        std::cout << "轴 " << axis_name_ << " 故障清除请求已设置" << std::endl;
    }
}

void ServoAxisBase::reset_axis() {
    reset_requested_ = true;
    std::cout << "轴 " << axis_name_ << " 重置请求已设置" << std::endl;
}

// 获取函数实现
std::string ServoAxisBase::get_name() const { return axis_name_; }
AxisState ServoAxisBase::get_current_state() const { return current_state_; }
OperationMode ServoAxisBase::get_operation_mode() const { return operation_mode_; }
bool ServoAxisBase::is_ready() const { return current_state_ == AxisState::READY; }
bool ServoAxisBase::is_running() const { 
    return current_state_ == AxisState::MANUAL_MODE || current_state_ == AxisState::AUTO_MODE; 
}
bool ServoAxisBase::is_homing_completed() const { return homing_completed_; }
bool ServoAxisBase::is_homing_in_progress() const { return homing_in_progress_; }
ec_slave_config_t* ServoAxisBase::get_slave_config() { return sc_; }
unsigned int ServoAxisBase::get_control_word_offset() const { return control_word_; }
int32_t ServoAxisBase::get_actual_position() const { return joint_position_; }
int32_t ServoAxisBase::get_initial_position() const { return initial_position_; }
DriveBrand ServoAxisBase::get_brand() const { return brand_; }

// 保护方法实现
int32_t ServoAxisBase::displacement_to_pulses(double displacement_mm) {
    const double SCREW_LEAD = 10.0;
    const double GEAR_RATIO = 1.0;
    const int PULSES_PER_REV = 10000;
    return static_cast<int32_t>((displacement_mm / SCREW_LEAD) * GEAR_RATIO * PULSES_PER_REV);
}

double ServoAxisBase::pulses_to_displacement(int32_t pulses) {
    const double SCREW_LEAD = 10.0;
    const double GEAR_RATIO = 1.0;
    const int PULSES_PER_REV = 10000;
    return static_cast<double>(pulses * SCREW_LEAD / (GEAR_RATIO * PULSES_PER_REV));
}

void ServoAxisBase::check_state_changes(uint16_t read_status_word, uint16_t error_code) {
    // 为每个轴单独记录状态字
    static std::map<std::string, uint16_t> last_status_words;
    
    uint16_t& last_status_word = last_status_words[axis_name_];
    
    if (read_status_word != last_status_word) {
        std::cout << "轴 " << axis_name_ << " 状态字变化: 0x" 
                  << std::hex << last_status_word << " -> 0x" << read_status_word << std::dec << std::endl;
        last_status_word = read_status_word;
    }
}

void ServoAxisBase::handle_fault_clear(uint8_t* domain1_pd) {
    fault_clear_counter_++;
    uint16_t current_status = EC_READ_U16(domain1_pd + status_word_);
    
    switch (fault_clear_step_) {
        case 0:
            // 第一步：写0x0080（故障复位）
            EC_WRITE_U16(domain1_pd + control_word_, 0x0080);
            printf("轴 %s 故障清除步骤%d: 发送0x0080\n", axis_name_.c_str(), fault_clear_step_);
            if (fault_clear_counter_ > 10) {
                fault_clear_step_ = 1;
                fault_clear_counter_ = 0;
            }
            break;
            
        case 1:
            // 第二步：写0x0000（准备使能）
            EC_WRITE_U16(domain1_pd + control_word_, 0x0000);
            printf("轴 %s 故障清除步骤%d: 发送0x0000\n", axis_name_.c_str(), fault_clear_step_);
            if (fault_clear_counter_ > 5) {
                fault_clear_step_ = 2;
                fault_clear_counter_ = 0;
            }
            break;
            
        case 2:
            // 第三步：写0x0006（切换到准备开关ON）
            EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
            printf("轴 %s 故障清除步骤%d: 发送0x0006\n", axis_name_.c_str(), fault_clear_step_);
            if (fault_clear_counter_ > 5) {
                fault_clear_step_ = 3;
                fault_clear_counter_ = 0;
            }
            break;
            
        case 3:
            // 第四步：检查状态字，确认故障已清除
            if (current_status == 0x0631 || current_status == 0x0633 || current_status == 0x0670 ||
                current_status == 0x0250 || current_status == 0x0650) {
                fault_clearing_in_progress_ = false;
                
                // 重置位置信息
                int32_t current_pos = EC_READ_S32(domain1_pd + off_actual_position_);
                target_pulses_ = current_pos;
                joint_position_ = current_pos;
                initial_position_ = current_pos;

                printf("轴 %s 故障清除完成，状态字: 0x%04x\n",
                    axis_name_.c_str(), current_status);
            } else if (fault_clear_counter_ > 100) {
                // 超时处理
                fault_clearing_in_progress_ = false;
                printf("轴 %s 故障清除超时，当前状态字: 0x%04x\n",
                    axis_name_.c_str(), current_status);
            }
            break;
    }
}

void ServoAxisBase::check_system_initialization() {
    // 基类实现系统初始化检查
    if (homing_completed_) {
        std::cout << "轴 " << axis_name_ << " 回零完成，系统可初始化" << std::endl;
    }
}