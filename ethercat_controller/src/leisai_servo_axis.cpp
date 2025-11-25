#include "leisai_servo_axis.hpp"
#include "ethercat_node.hpp"
#include <iostream>
#include <rclcpp/rclcpp.hpp>

// 雷赛PDO配置定义
ec_pdo_entry_info_t leisai_slave_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Control Word */
    {0x607a, 0x00, 32}, /* Profile Target Position */
    {0x60b8, 0x00, 16}, /* Touch Probe Function */
    {0x6840, 0x00, 16}, /* Control Word */
    {0x687a, 0x00, 32}, /* Profile Target Position */
    {0x68b8, 0x00, 16}, /* Touch Probe Function */
    {0x603f, 0x00, 16}, /* Last Error Code */
    {0x6041, 0x00, 16}, /* Status Word */
    {0x6061, 0x00, 8}, /* Modes of Operation Display */
    {0x6064, 0x00, 32}, /* Actual Motor Position */
    {0x60b9, 0x00, 16}, /* Touch Probe Status */
    {0x60ba, 0x00, 32}, /* Touch Probe 1 Positive Value */
    {0x60fd, 0x00, 32}, /* Digital Inputs */
    {0x683f, 0x00, 16}, /* Last Error Code */
    {0x6841, 0x00, 16}, /* Status Word */
    {0x6861, 0x00, 8}, /* Modes of Operation Display */
    {0x6864, 0x00, 32}, /* Actual Motor Position */
    {0x68b9, 0x00, 16}, /* Touch Probe Status */
    {0x68ba, 0x00, 32}, /* Touch Probe 1 Positive Value */
    {0x68fd, 0x00, 32}, /* Digital Inputs */
};

ec_pdo_info_t leisai_slave_pdos[] = {
    {0x1600, 3, leisai_slave_pdo_entries + 0}, /* Axis 1 Receive PDO 1 Mapping Parameter */
    {0x1610, 3, leisai_slave_pdo_entries + 3}, /* Axis 2 Receive PDO 1 Mapping Parameter */
    {0x1a00, 7, leisai_slave_pdo_entries + 6}, /* Axis 1 Transmit PDO 1 Mapping Parameter */
    {0x1a10, 7, leisai_slave_pdo_entries + 13}, /* Axis 2 Transmit PDO 1 Mapping Parameter */
};

ec_sync_info_t leisai_slave_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, leisai_slave_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, leisai_slave_pdos + 2, EC_WD_DISABLE},
    {0xff}
};
// 添加缺失的常量定义
const int HOMING_TOLERANCE = 100;
const int HOMING_STEP = 50;
// extern ec_domain_t *domain1;
// extern uint8_t *domain1_pd;  // 统一使用domain1_pd

LeisaiServoAxis::LeisaiServoAxis(const std::string& name, uint16_t position, AxisType axis_type)
    : ServoAxisBase(name, position, axis_type, DriveBrand::LEISAI), leisai_specific_param_(0) {
}

void LeisaiServoAxis::configure(ec_master_t* master) {
    sc_ = ecrt_master_slave_config(master, 0, slave_position_,
                                 LEISAI_VENDOR_ID, LEISAI_PRODUCT_CODE);
    if (!sc_) {
        RCLCPP_FATAL(rclcpp::get_logger("ethercat_controller"), 
                    "创建 %s 轴从站配置失败", axis_name_.c_str());
        return;
    }
    
    if (ecrt_slave_config_pdos(sc_, EC_END, leisai_slave_syncs)) {
        RCLCPP_FATAL(rclcpp::get_logger("ethercat_controller"), 
                    "%s 轴PDO配置失败", axis_name_.c_str());
    }
    
    ecrt_slave_config_dc(sc_, 0x0300, PERIOD_NS, 0, 0, 0);
}

void LeisaiServoAxis::register_pdo_entries(ec_pdo_entry_reg_t* reg_list, int& index) {
    
    if (axis_type_ == AxisType::AXIS1) {
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
    } else {
        control_word_ = ecrt_slave_config_reg_pdo_entry(
            sc_, 0x6840, 0, domain1, NULL);
        status_word_ = ecrt_slave_config_reg_pdo_entry(
            sc_, 0x6841, 0, domain1, NULL);
        off_target_position_ = ecrt_slave_config_reg_pdo_entry(
            sc_, 0x687A, 0, domain1, NULL);
        off_actual_position_ = ecrt_slave_config_reg_pdo_entry(
            sc_, 0x6864, 0, domain1, NULL);
        off_error_code_ = ecrt_slave_config_reg_pdo_entry(
            sc_, 0x683F, 0, domain1, NULL);
    }
    
    // if (control_word_ < 0 || status_word_ < 0 || 
    //     off_target_position_ < 0 || off_actual_position_ < 0) {
    // if (control_word_ == nullptr) {
    //     RCLCPP_FATAL(rclcpp::get_logger("ethercat_controller"),
    //                 "%s 轴PDO注册失败", axis_name_.c_str());
    // }
    
    // // 注册到列表
    // reg_list[index++] = {slave_position_, 0x6040, 0, domain1, &control_word_};
    // reg_list[index++] = {slave_position_, 0x6041, 0, domain1, &status_word_};
    // reg_list[index++] = {slave_position_, 0x607A, 0, domain1, &off_target_position_};
    // reg_list[index++] = {slave_position_, 0x6064, 0, domain1, &off_actual_position_};
    // reg_list[index++] = {slave_position_, 0x603F, 0, domain1, &off_error_code_};
}

void LeisaiServoAxis::handle_state_machine(uint8_t* domain1_pd) {
    int32_t current_pos = EC_READ_S32(domain1_pd + off_actual_position_); // 修复：使用domain_pd
    uint16_t read_status_word = EC_READ_U16(domain1_pd + status_word_);
    uint16_t error_code = EC_READ_U16(domain1_pd + off_error_code_);

    // 检查故障清除请求
    if (clear_fault_requested_ && current_state_ == AxisState::FAULT) {
        clear_fault_requested_ = false;
        fault_clearing_in_progress_ = true;
        fault_clear_step_ = 0;
        fault_clear_counter_ = 0;
        printf("轴 %s 开始清除故障流程\n", axis_name_.c_str());
    }

    // 检查重置请求
    if (reset_requested_) {
        reset_requested_ = false;
        current_state_ = AxisState::UNINITIALIZED;
        printf("轴 %s 执行重置，回到未初始化状态\n", axis_name_.c_str());
    }

    // 状态转换逻辑
    switch (current_state_) {
        case AxisState::UNINITIALIZED:
            if (read_status_word == 0x1650) {
                current_state_ = AxisState::INITIALIZING;
                printf("轴 %s 进入初始化状态\n", axis_name_.c_str());
            }
            break;
            
        case AxisState::INITIALIZING:
            handle_leisai_initialization(domain1_pd, read_status_word);
            break;
            
        case AxisState::READY:
            handle_leisai_ready_state(domain1_pd, read_status_word);
            break;
            
        case AxisState::MANUAL_MODE:
            if (!(read_status_word == 0x1637 || read_status_word == 0x1237)) {
                current_state_ = AxisState::INITIALIZING;
                printf("轴 %s 退出手动模式\n", axis_name_.c_str());
            } else {
                handle_leisai_manual_operation(domain1_pd, current_pos);
            }
            break;
            
        case AxisState::AUTO_MODE:
            if (read_status_word != 0x1637) {
                current_state_ = AxisState::INITIALIZING;
                printf("轴 %s 退出自动模式\n", axis_name_.c_str());
            } else {
                handle_leisai_auto_operation(domain1_pd, current_pos);
            }
            break;
            
        case AxisState::FAULT:
            handle_leisai_fault_state(domain1_pd, error_code);
            break;
    }
    
    check_state_changes(read_status_word, error_code);
}

void LeisaiServoAxis::handle_leisai_initialization(uint8_t* domain1_pd, uint16_t read_status_word) {
    static unsigned int init_delay = 0;
    init_delay++;
    
    if (init_delay < 100) {
        return; // 修复：使用return而不是break
    }
    
    // 雷赛特有初始化序列
    if (read_status_word == 0x0250 || read_status_word == 0x0650 || 
        read_status_word == 0x0670 || read_status_word == 0x0770) {
        EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
    } else if (read_status_word == 0x0631 || read_status_word == 0x0731) {
        EC_WRITE_U16(domain1_pd + control_word_, 0x0007);
    } else if (read_status_word == 0x0633 || read_status_word == 0x0733) {
        int32_t current_pos = EC_READ_S32(domain1_pd + off_actual_position_); // 修复：使用domain_pd
        EC_WRITE_S32(domain1_pd + off_target_position_, current_pos); // 修复：使用domain_pd
        EC_WRITE_U16(domain1_pd + control_word_, 0x001F);
        current_state_ = AxisState::READY;
        printf("轴 %s 进入就绪状态\n", axis_name_.c_str());
    } else if (read_status_word == 0x1637 || read_status_word == 0x1237 || 
               read_status_word == 0x16b7) {
        int32_t current_pos = EC_READ_S32(domain1_pd + off_actual_position_); // 修复：使用domain_pd
        EC_WRITE_S32(domain1_pd + off_target_position_, current_pos); // 修复：使用domain_pd
        current_state_ = AxisState::READY;
        printf("轴 %s 进入就绪状态\n", axis_name_.c_str());
    }
}
// 实现缺失的函数
void LeisaiServoAxis::handle_leisai_ready_state(uint8_t* domain1_pd, uint16_t status_word) {
    // 实现代码
    if (start_manual_requested_ && status_word == 0x1237) {
        start_manual_requested_ = false;
        operation_mode_ = OperationMode::MANUAL;
        current_state_ = AxisState::MANUAL_MODE;
    }
    // ... 其他逻辑
}

void LeisaiServoAxis::handle_leisai_manual_operation(uint8_t* domain1_pd, int32_t current_pos) {
    // 实现手动模式操作逻辑
    if (!position_initialized_) {
        joint_position_ = current_pos;
        initial_position_ = current_pos;
        position_initialized_ = true;
    }
    // ... 其他逻辑
}

void LeisaiServoAxis::handle_leisai_auto_operation(uint8_t* domain1_pd, int32_t current_pos) {
    // 实现自动模式操作逻辑
    if (!homing_completed_) {
        // 回零逻辑
        return;
    }
    // ... 其他逻辑
}

void LeisaiServoAxis::handle_leisai_fault_state(uint8_t* domain1_pd, uint16_t error_code) {
    // 实现故障状态处理逻辑
    if (fault_clearing_in_progress_) {
        handle_fault_clear(domain1_pd);
    }
}