#include "huichuan_servo_axis.hpp"
#include "ethercat_node.hpp"
#include <iostream>
#include <rclcpp/rclcpp.hpp>

// 汇川PDO配置定义
ec_pdo_entry_info_t huichuan_slave_pdo_entries[] = {
    {0x6040, 0x00, 16}, {0x607a, 0x00, 32}, {0x60b8, 0x00, 16},
    {0x60fe, 0x01, 32}, {0x603f, 0x00, 16}, {0x6041, 0x00, 16},
    {0x6064, 0x00, 32}, {0x6077, 0x00, 16}, {0x60f4, 0x00, 32},
    {0x60b9, 0x00, 16}, {0x60ba, 0x00, 32}, {0x60bc, 0x00, 32},
    {0x60fd, 0x00, 32},
};

ec_pdo_info_t huichuan_slave_pdos[] = {
    {0x1701, 4, huichuan_slave_pdo_entries + 0},
    {0x1b01, 9, huichuan_slave_pdo_entries + 4},
};

ec_sync_info_t huichuan_slave_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, huichuan_slave_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, huichuan_slave_pdos + 1, EC_WD_DISABLE},
    {0xff}
};
// 添加缺失的全局变量声明
const int HOMING_TOLERANCE = 100;
const int HOMING_STEP = 50;

HuichuanServoAxis::HuichuanServoAxis(const std::string& name, uint16_t position, AxisType axis_type)
    : ServoAxisBase(name, position, axis_type, DriveBrand::HUICHUAN),
      huichuan_specific_param_(0.0) {
    
    std::cout << "创建汇川伺服轴: " << name << std::endl;
}

void HuichuanServoAxis::configure(ec_master_t* master) {
    sc_ = ecrt_master_slave_config(master, 0, slave_position_,
                                  HUICHUAN_VENDOR_ID, HUICHUAN_PRODUCT_CODE);
    if (!sc_) {
        RCLCPP_FATAL(rclcpp::get_logger("huichuan_servo"), 
                    "创建汇川 %s 轴从站配置失败", axis_name_.c_str());
        return;
    }
    
    if (ecrt_slave_config_pdos(sc_, EC_END, huichuan_slave_syncs)) {
        RCLCPP_FATAL(rclcpp::get_logger("huichuan_servo"), 
                    "%s 轴PDO配置失败", axis_name_.c_str());
    }
    
    ecrt_slave_config_dc(sc_, 0x0300, PERIOD_NS, 0, 0, 0);
    RCLCPP_INFO(rclcpp::get_logger("huichuan_servo"), 
                "汇川轴 %s 配置完成", axis_name_.c_str());
}

void HuichuanServoAxis::register_pdo_entries(ec_pdo_entry_reg_t* reg_list, int& index) {
    // 汇川特有的PDO注册
    control_word_ = ecrt_slave_config_reg_pdo_entry(
        sc_, 0x6040, 0, domain1, NULL);
    status_word_ = ecrt_slave_config_reg_pdo_entry(
        sc_, 0x6041, 0, domain1, NULL);
    off_target_position_ = ecrt_slave_config_reg_pdo_entry(
        sc_, 0x607A, 0, domain1, NULL);
    off_actual_position_ = ecrt_slave_config_reg_pdo_entry(
        sc_, 0x6064, 0, domain1, NULL);
    off_error_code_ = ecrt_slave_config_reg_pdo_entry(
        sc_, 0x603F, 0, domain1, NULL);
    
    // if (control_word_ == nullptr) {
    //     RCLCPP_FATAL(rclcpp::get_logger("huichuan_servo"),
    //                 "%s 轴PDO注册失败", axis_name_.c_str());
    // }
    
    // // 注册到列表
    // reg_list[index++] = {slave_position_, 0x6040, 0, domain1, &control_word_};
    // reg_list[index++] = {slave_position_, 0x6041, 0, domain1, &status_word_};
    // reg_list[index++] = {slave_position_, 0x607A, 0, domain1, &off_target_position_};
    // reg_list[index++] = {slave_position_, 0x6064, 0, domain1, &off_actual_position_};
    // reg_list[index++] = {slave_position_, 0x603F, 0, domain1, &off_error_code_};
}

void HuichuanServoAxis::handle_state_machine(uint8_t* domain1_pd) {
    int32_t current_pos = EC_READ_S32(domain1_pd + off_actual_position_);
    uint16_t read_status_word = EC_READ_U16(domain1_pd + status_word_);
    uint16_t error_code = EC_READ_U16(domain1_pd + off_error_code_);
    
    switch (current_state_) {
        case AxisState::UNINITIALIZED:
            if (read_status_word == 0x1650) {
                current_state_ = AxisState::INITIALIZING;
                RCLCPP_INFO(rclcpp::get_logger("huichuan_servo"), 
                           "汇川轴 %s 进入初始化状态", axis_name_.c_str());
            }
            break;
            
        case AxisState::INITIALIZING:
            // handle_huichuan_initialization(domain1_pd, read_status_word);
            static unsigned int init_delay = 0;
            init_delay++;
            
            // 前100个周期只观察状态，不发送控制命令
            if (init_delay < 100) {
                break;
            }
            // 执行初始化序列至1633状态
            if (read_status_word == 0x1650) {
                EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
            } else if (read_status_word == 0x1631) {
                EC_WRITE_U16(domain1_pd + control_word_, 0x0007);
            } else if (read_status_word == 0x1633) {
                EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                EC_WRITE_U16(domain1_pd + control_word_, 0x000F);
                current_state_ = AxisState::READY;
                printf("轴 %s 进入就绪状态\n", axis_name_.c_str());
            } else if (read_status_word == 0x1637) {
                EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                current_state_ = AxisState::READY;
                printf("轴 %s 进入就绪状态\n", axis_name_.c_str());
            }
            break;
            
        case AxisState::READY:
            if (start_manual_requested_ && read_status_word == 0x1637) {
                start_manual_requested_ = false;
                operation_mode_ = OperationMode::MANUAL;
                current_state_ = AxisState::MANUAL_MODE;
                RCLCPP_INFO(rclcpp::get_logger("huichuan_servo"), 
                           "汇川轴 %s 进入手动模式", axis_name_.c_str());
            } 
            else if (start_auto_requested_ && read_status_word == 0x1637) {
                start_auto_requested_ = false;
                operation_mode_ = OperationMode::AUTO;
                current_state_ = AxisState::AUTO_MODE;
                RCLCPP_INFO(rclcpp::get_logger("huichuan_servo"), 
                           "汇川轴 %s 进入自动模式", axis_name_.c_str());
            }
            break;
            
        case AxisState::MANUAL_MODE:
            handle_huichuan_manual_operation(domain1_pd, current_pos);
            break;
            
        case AxisState::AUTO_MODE:
            handle_huichuan_auto_operation(domain1_pd, current_pos);
            break;
            
        case AxisState::FAULT:
            // 汇川特有的故障处理
            if (error_code == 0x821b) {
                RCLCPP_INFO(rclcpp::get_logger("huichuan_servo"), 
                           "汇川轴 %s 检测到通讯错误0x821b，自动清除", axis_name_.c_str());
                fault_clearing_in_progress_ = true;
                fault_clear_step_ = 0;
            }
            if (fault_clearing_in_progress_) {
                handle_fault_clear(domain1_pd);
            }
            break;
    }
    
    check_state_changes(read_status_word, error_code);
}

void HuichuanServoAxis::handle_huichuan_initialization(uint8_t* domain1_pd, uint16_t status_word) {
    // // 汇川特有的初始化序列
    // static unsigned int init_counter = 0;
    // init_counter++;
    
    // if (status_word == 0x0250) {
    //     EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
    // } else if (status_word == 0x0631) {
    //     EC_WRITE_U16(domain1_pd + control_word_, 0x0007);
    // } else if (status_word == 0x0633) {
    //     int32_t current_pos = EC_READ_S32(domain1_pd + off_actual_position_);
    //     EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
    //     EC_WRITE_U16(domain1_pd + control_word_, 0x000F);
        
    //     if (init_counter > 50) { // 汇川需要更长的初始化时间
    //         current_state_ = AxisState::READY;
    //         RCLCPP_INFO(rclcpp::get_logger("huichuan_servo"), 
    //                    "汇川轴 %s 进入就绪状态", axis_name_.c_str());
    //     }
    // }
}

void HuichuanServoAxis::handle_huichuan_homing(uint8_t* domain1_pd, int32_t current_pos) {
    // 汇川特有的回零逻辑
    if (!homing_in_progress_) {
        homing_in_progress_ = true;
        home_target_position_ = current_pos;
        RCLCPP_INFO(rclcpp::get_logger("huichuan_servo"), 
                   "汇川轴 %s 开始回零，当前位置: %d", axis_name_.c_str(), current_pos);
    }
    
    // 汇川采用不同的回零策略
    int32_t target = 0;
    int32_t error = current_pos - target;
    
    if (abs(error) < HOMING_TOLERANCE) {
        homing_in_progress_ = false;
        homing_completed_ = true;
        EC_WRITE_S32(domain1_pd + off_target_position_, target);
        RCLCPP_INFO(rclcpp::get_logger("huichuan_servo"), 
                   "汇川轴 %s 回零完成", axis_name_.c_str());
    } else {
        // 汇川采用比例控制
        int32_t step = error / 10;
        if (abs(step) < 1) step = (error > 0) ? 1 : -1;
        
        home_target_position_ -= step;
        EC_WRITE_S32(domain1_pd + off_target_position_, home_target_position_);
    }
}

void HuichuanServoAxis::handle_huichuan_manual_operation(uint8_t* domain1_pd, int32_t current_pos) {
    // 汇川手动模式特有逻辑
    if (!position_initialized_) {
        joint_position_ = current_pos;
        initial_position_ = current_pos;
        target_pulses_ = current_pos;
        position_initialized_ = true;
    }
    
    if (displacement_updated_) {
        displacement_updated_ = false;
        int32_t displacement_pulses = displacement_to_pulses(target_displacement_);
        target_pulses_ = joint_position_ + displacement_pulses; // 相对位移
    }
    
    // 汇川特有的速度限制
    static int position_counter = 0;
    if (position_counter++ >= 5) { // 更快的控制周期
        position_counter = 0;
        
        const int32_t MAX_STEP = 50; // 汇川速度限制
        int32_t error = target_pulses_ - joint_position_;
        int32_t step = (abs(error) > MAX_STEP) ? 
                      ((error > 0) ? MAX_STEP : -MAX_STEP) : error;
        
        joint_position_ += step;
        EC_WRITE_S32(domain1_pd + off_target_position_, joint_position_);
    }
}

void HuichuanServoAxis::handle_huichuan_auto_operation(uint8_t* domain1_pd, int32_t current_pos) {
    // 汇川自动模式特有逻辑
    if (!homing_completed_) {
        handle_huichuan_homing(domain1_pd, current_pos);
        if (displacement_updated_) {
            displacement_updated_ = false;
            RCLCPP_WARN(rclcpp::get_logger("huichuan_servo"), 
                       "汇川轴 %s 回零期间忽略位移指令", axis_name_.c_str());
        }
        return;
    }
    
    if (!position_initialized_) {
        joint_position_ = current_pos;
        initial_position_ = current_pos;
        target_pulses_ = current_pos;
        position_initialized_ = true;
    }
    
    if (displacement_updated_) {
        displacement_updated_ = false;
        target_pulses_ = initial_position_ + displacement_to_pulses(target_displacement_);
        
        // 汇川特有的安全检测
        const int32_t MAX_SAFE_DELTA = 2500;
        int32_t position_delta = abs(target_pulses_ - joint_position_);
        if (position_delta > MAX_SAFE_DELTA) {
            current_state_ = AxisState::FAULT;
            RCLCPP_ERROR(rclcpp::get_logger("huichuan_servo"), 
                        "汇川轴 %s 运动控制错误：差值过大(%d脉冲)", 
                        axis_name_.c_str(), position_delta);
            return;
        }
    }
    
    // 汇川直接位置控制
    EC_WRITE_S32(domain1_pd + off_target_position_, target_pulses_);
    joint_position_ = target_pulses_;
}

void HuichuanServoAxis::set_huichuan_specific_parameter(double param) {
    huichuan_specific_param_ = param;
}

double HuichuanServoAxis::get_huichuan_specific_parameter() const {
    return huichuan_specific_param_;
}