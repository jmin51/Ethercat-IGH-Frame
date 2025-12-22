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

LeisaiServoAxis::LeisaiServoAxis(const std::string& name, uint16_t position, AxisType axis_type, uint32_t product_code)
    : ServoAxisBase(name, position, axis_type, DriveBrand::LEISAI, product_code), 
      leisai_specific_param_(0),
      product_code_(product_code) {  // 存储产品号
}

void LeisaiServoAxis::configure(ec_master_t* master) {
    sc_ = ecrt_master_slave_config(master, 0, slave_position_,
                                 LEISAI_VENDOR_ID, product_code_);  // 使用存储的产品号
    if (!sc_) {
        RCLCPP_FATAL(rclcpp::get_logger("ethercat_controller"), 
                    "创建 %s 轴从站配置失败，产品号: 0x%08x", axis_name_.c_str(), product_code_);
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

    // // 状态转换逻辑
    // switch (current_state_) {
    //     case AxisState::UNINITIALIZED:
    //             current_state_ = AxisState::INITIALIZING;
    //             printf("轴 %s 进入初始化状态\n", axis_name_.c_str());
    //         break;
            
    //     case AxisState::INITIALIZING:
    //         handle_leisai_initialization(domain1_pd, read_status_word);
    //         break;
            
    //     case AxisState::READY:
    //         handle_leisai_ready_state(domain1_pd, read_status_word);
    //         break;
            
    //     case AxisState::MANUAL_MODE:
    //         if (!(read_status_word == 0x1637 || read_status_word == 0x1237)) {
    //             current_state_ = AxisState::INITIALIZING;
    //             printf("轴 %s 退出手动模式\n", axis_name_.c_str());
    //         } else {
    //             handle_leisai_manual_operation(domain1_pd, current_pos);
    //         }
    //         break;
            
    //     case AxisState::AUTO_MODE:
    //         if (read_status_word != 0x1637) {
    //             current_state_ = AxisState::INITIALIZING;
    //             printf("轴 %s 退出自动模式\n", axis_name_.c_str());
    //         } else {
    //             handle_leisai_auto_operation(domain1_pd, current_pos);
    //         }
    //         break;
         
    //     case AxisState::STOPPED:
    //         // 停止状态：保持当前位置，等待回到INITIALIZING
    //         EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
    //         EC_WRITE_U16(domain1_pd + control_word_, 0x000F);

    //         // 检查是否可以回到READY状态
    //         // if (read_status_word == 0x1637 && !global_data_blocked.load()) {
    //         if (read_status_word == 0x1637 ) {
    //             current_state_ = AxisState::INITIALIZING;
    //             printf("轴 %s 从停止状态回到就绪状态\n", axis_name_.c_str());
    //         }
    //         break;
            
    //     case AxisState::FAULT:
    //         // 检查是否是821b通讯错误的自动清除流程
    //         if (error_code == 0x821b && fault_clearing_in_progress_) {
    //             handle_fault_clear(domain1_pd);

    //             // 故障清除完成后，回到初始化状态
    //             if (!fault_clearing_in_progress_) {
    //                 current_state_ = AxisState::INITIALIZING;
    //                 printf("轴 %s 821b故障清除完成，回到初始化状态\n", axis_name_.c_str());
    //             }
    //         }
    //         // 原有的故障处理逻辑保持不变
    //         else if (clear_fault_requested_ && current_state_ == AxisState::FAULT) {
    //             clear_fault_requested_ = false;
    //             fault_clearing_in_progress_ = true;
    //             fault_clear_step_ = 0;
    //             fault_clear_counter_ = 0;
    //             printf("轴 %s 开始清除故障流程\n", axis_name_.c_str());
    //         }

    //         if (fault_clearing_in_progress_) {
    //             handle_fault_clear(domain1_pd);
    //         } else {
    //             EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
    //         }
    //         break;
    // }
    // 状态转换逻辑
    switch (current_state_) {
        case AxisState::UNINITIALIZED:
                current_state_ = AxisState::INITIALIZING;
                printf("轴 %s 进入初始化状态\n", axis_name_.c_str());
            break;
            
        case AxisState::INITIALIZING:
            static unsigned int init_delay = 0;
            init_delay++;
            
            // 前100个周期只观察状态，不发送控制命令
            if (init_delay < 100) {
                break;
            }
            // 新增：检查821b通讯错误并自动清除
            if (error_code == 0x821b) {
                printf("轴 %s 检测到通讯错误0x821b, 开始自动清除故障流程\n", axis_name_.c_str());
                fault_clearing_in_progress_ = true;
                fault_clear_step_ = 0;
                fault_clear_counter_ = 0;
                current_state_ = AxisState::FAULT;
                break;
            }

            // 执行初始化序列至1633状态
            if (read_status_word == 0x0250 || read_status_word == 0x0650 || read_status_word == 0x0670 || read_status_word == 0x0770) {
                EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
                printf("发送状态转换命令: 0x0006\n");
            } else if (read_status_word == 0x0631 || read_status_word == 0x0731) {
                EC_WRITE_U16(domain1_pd + control_word_, 0x0007);
                printf("发送状态转换命令: 0x0007\n");
            } else if (read_status_word == 0x0633 || read_status_word == 0x0733) {
                EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                EC_WRITE_U16(domain1_pd + control_word_, 0x001F);
                printf("发送状态转换命令: 0x001F\n");
                current_state_ = AxisState::READY;
                printf("轴 %s 进入就绪状态\n", axis_name_.c_str());
            } else if (read_status_word == 0x1637 || read_status_word == 0x1237 || read_status_word == 0x16b7) {
                EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                current_state_ = AxisState::READY;
                printf("轴 %s 进入就绪状态\n", axis_name_.c_str());
            } 
            // else if (read_status_word == 0x0638) {
            //     // 读取错误代码并打印详细故障信息
            //     uint16_t leisai_error_code = EC_READ_U16(domain1_pd + off_error_code_);
                
            //     // 特殊处理：0x821b错误代码完全忽略，继续初始化
            //     if (leisai_error_code == 0x821b || leisai_error_code == 0x0000) {
            //         printf("轴 %s -----", axis_name_.c_str());
            //         // 不改变current_state_，继续执行初始化序列
            //     } else {
            //         // 其他错误代码正常进入故障模式
            //         current_state_ = AxisState::FAULT;
            //         printf("轴 %s 检测到故障状态字0x0638,错误代码: 0x%04X\n",
            //             axis_name_.c_str(), leisai_error_code);
            //     }
            // }
            break;
            
        case AxisState::READY:
            // 处理启动请求
            if (start_manual_requested_ && (read_status_word == 0x1637 || read_status_word == 0x1237)) {
                start_manual_requested_ = false;
                operation_mode_ = OperationMode::MANUAL;
                current_state_ = AxisState::MANUAL_MODE;
                printf("轴 %s 进入手动模式\n", axis_name_.c_str());

                // 初始化手动模式位置
                // 退出自动模式时重置回零状态
                homing_completed_ = false;
                homing_in_progress_ = false;
                position_initialized_ = false;
            } 
            else if (start_auto_requested_ && (read_status_word == 0x1637 || read_status_word == 0x1237)) {
                start_auto_requested_ = false;
                operation_mode_ = OperationMode::AUTO;
                current_state_ = AxisState::AUTO_MODE;

                printf("轴 %s 进入自动模式\n", axis_name_.c_str());
            }
            else {
                // 保持在READY状态，保持当前位置
                EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                EC_WRITE_U16(domain1_pd + control_word_, 0x000F);
                
                // 新增：检测故障状态字
                if (read_status_word == 0x0638) {
                    printf("轴 %s 检测到故障状态字0x1638，进入故障模式\n", 
                        axis_name_.c_str());
                    current_state_ = AxisState::FAULT;
                    // 记录故障信息
                    break; // 立即跳出，不再执行后续逻辑
                }

                // 如果有启动请求但状态字不满足，检查是否状态异常
                if (start_manual_requested_ || start_auto_requested_) {
                    static int warning_counter = 0;
                    if (warning_counter++ % 100 == 0) {
                        printf("轴 %s 等待状态字0x1637才能进入模式切换，当前状态字: 0x%04x\n", 
                            axis_name_.c_str(), read_status_word);
                    }
                    
                    // 新增：如果状态字异常（非0x1637），自动跳转回初始化
                    if (!(read_status_word == 0x1637 || read_status_word == 0x1237)) {
                        printf("轴 %s 检测到异常状态字0x%04x，自动跳转回初始化状态\n", 
                            axis_name_.c_str(), read_status_word);
                        current_state_ = AxisState::INITIALIZING;
                        
                        // 可选：清除启动请求标志，避免重复触发
                        // start_manual_requested_ = false;
                        // start_auto_requested_ = false;
                    }
                }
            }
            break;
            
        case AxisState::MANUAL_MODE:
            // 手动模式处理
            if (!(read_status_word == 0x1637 || read_status_word == 0x1237)) {
                current_state_ = AxisState::INITIALIZING;
                printf("轴 %s 退出手动模式\n", axis_name_.c_str());
            } else {
                handle_leisai_manual_operation(domain1_pd, current_pos);
            }
            break;
            
        case AxisState::AUTO_MODE:
            // 自动模式处理
            if (read_status_word != 0x1637) {
                current_state_ = AxisState::INITIALIZING;
                printf("轴 %s 退出自动模式\n", axis_name_.c_str());
            } else {
                handle_leisai_auto_operation(domain1_pd, current_pos);
            }
            break;
            
        case AxisState::STOPPED:
            // 停止状态：保持当前位置，等待回到INITIALIZING
            EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
            EC_WRITE_U16(domain1_pd + control_word_, 0x000F);

            // 检查是否可以回到READY状态
            // if (read_status_word == 0x1637 && !global_data_blocked.load()) {
            if (read_status_word == 0x1637 ) {
                current_state_ = AxisState::INITIALIZING;
                printf("轴 %s 从停止状态回到就绪状态\n", axis_name_.c_str());
            }
            break;
            
        case AxisState::FAULT:
            // if (error_code == 0x821b && fault_clearing_in_progress_) {
            //     handle_fault_clear(domain1_pd);
            //     // 故障清除完成后，回到初始化状态
            //     if (!fault_clearing_in_progress_) {
            //         current_state_ = AxisState::INITIALIZING;
            //         printf("轴 %s 821b故障清除完成, 回到初始化状态\n", axis_name_.c_str());
            //     }
            // }
            // // 原有的故障处理逻辑保持不变
            // else if (clear_fault_requested_ && current_state_ == AxisState::FAULT) {
            //     clear_fault_requested_ = false;
            //     fault_clearing_in_progress_ = true;
            //     fault_clear_step_ = 0;
            //     fault_clear_counter_ = 0;
            //     printf("轴 %s 开始清除故障流程\n", axis_name_.c_str());
            // }

            // if (fault_clearing_in_progress_) {
            //     handle_fault_clear(domain1_pd);
            // } else {
            //     EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
            // }
            if (fault_clearing_in_progress_) {
                handle_fault_clear(domain1_pd);
                
                // 检查清除是否完成
                if (!fault_clearing_in_progress_) {
                    current_state_ = AxisState::INITIALIZING;
                    fault_clear_step_ = 0;  // 重置步骤
                    fault_clear_counter_ = 0;
                    printf("轴 %s 故障清除完成，回到初始化状态\n", axis_name_.c_str());
                }
            } 
            // 检查是否需要启动清除流程
            else if (error_code == 0x821b || clear_fault_requested_) {
                fault_clearing_in_progress_ = true;
                fault_clear_step_ = 0;
                fault_clear_counter_ = 0;
                clear_fault_requested_ = false;
                printf("轴 %s 开始故障清除流程\n", axis_name_.c_str());
            }
            // 默认故障处理
            else {
                EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
            }
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
        target_pulses_ = current_pos;
        position_initialized_ = true;
        printf("轴 %s 手动模式位置初始化完成\n", axis_name_.c_str());
    }
    
    // 处理点动控制
    if (jog_forward_requested_) {
        // 正转
        int32_t speed_pulses = displacement_to_pulses(jog_speed_ / FREQUENCY);
        target_pulses_ += speed_pulses;
    } else if (jog_reverse_requested_) {
        // 反转
        int32_t speed_pulses = displacement_to_pulses(jog_speed_ / FREQUENCY);
        target_pulses_ -= speed_pulses;
    } else if (jog_stop_requested_) {
        // 停止
        target_pulses_ = current_pos;
        jog_stop_requested_ = false;
    }
    
    // 平滑移动到目标位置
    static int position_counter = 0;
    if (position_counter++ >= 10) {
        position_counter = 0;
        
        const int32_t TOLERANCE = 100; //PULSE_Tolerance;
        int32_t error = target_pulses_ - joint_position_;
        
        // // 渐进式移动，避免突变
        // if (abs(error) > TOLERANCE) {
        //     int32_t step = error / 100; // 分10步移动
        //     if (abs(step) < 1) step = (error > 0) ? 1 : -1;

        //     joint_position_ += step;
        // } else {
        //     joint_position_ = target_pulses_;
        // }
        // 基于速度的匀速移动（单位：脉冲/周期）
        if (abs(error) > TOLERANCE) {
            const int32_t VELOCITY = 150; // 速度值，可根据需要调整
            int32_t step = (error > 0) ? VELOCITY : -VELOCITY;
            
            // 如果剩余距离小于步长，直接到达目标
            if (abs(error) <= abs(step)) {
                joint_position_ = target_pulses_;
            } else {
                joint_position_ += step;
            }
        } else {
            joint_position_ = target_pulses_;
        }

        EC_WRITE_S32(domain1_pd + off_target_position_, joint_position_);
    }
}

void LeisaiServoAxis::handle_leisai_auto_operation(uint8_t* domain1_pd, int32_t current_pos) {
    // 实现自动模式操作逻辑
    if (!homing_completed_) {
        // handle_homing_sequence(domain1_pd, current_pos); //待完善12.04

        // 新增：回零期间忽略位移更新
        if (displacement_updated_) {
            displacement_updated_ = false;
            printf("轴 %s 回零期间忽略位移指令，等待回零完成\n", axis_name_.c_str());
        }
        return;
    }
    
    if (!position_initialized_) {
        joint_position_ = current_pos;
        initial_position_ = current_pos;
        target_pulses_ = current_pos;
        position_initialized_ = true;
        printf("轴 %s 自动模式位置初始化完成\n", axis_name_.c_str());
    }
    
    // 自动模式：使用绝对位置控制
    if (displacement_updated_) {
        // printf("轴 %s 检测到位移更新标志，目标位移: %.6fmm\n", 
        //        axis_name_.c_str(), target_displacement_);
        displacement_updated_ = false;

        // 自动模式使用绝对位移
        target_pulses_ = initial_position_ + displacement_to_pulses(target_displacement_);

        // 新增：安全检测 - 检查目标位置与当前位置的差值
        const int32_t MAX_SAFE_DELTA = 3000; // 最大安全脉冲差值，可根据需要调整
        int32_t position_delta = abs(target_pulses_ - joint_position_);

        if (position_delta > MAX_SAFE_DELTA) {
            // 触发故障模式
            current_state_ = AxisState::FAULT;
            printf("轴 %s 运动控制错误：目标位置与当前位置差值过大(%d脉冲)，可能发生飞车！\n", 
                   axis_name_.c_str(), position_delta);
            printf("轴 %s 进入故障模式，需要手动清除故障\n", axis_name_.c_str());

            // 发布错误状态
            if (global_node) {
                auto status_msg = std_msgs::msg::String();
                status_msg.data = "轴 " + axis_name_ + " 运动控制错误：目标位置与当前位置差值过大";
                // global_node->system_status_pub_->publish(status_msg); // 需要添加system_status_pub_的访问权限
            }
            
            return; // 直接返回，不执行后续运动控制
        }
    }
    
    // 直接移动到目标位置
    static int position_counter = 0;
    if (position_counter++ >= 10) {
        position_counter = 0;
        
        const int32_t TOLERANCE = 100; // PULSE_Tolerance;

        if (abs(joint_position_ - target_pulses_) > TOLERANCE) {
            // 自动模式使用更直接的移动
            joint_position_ = target_pulses_;
        }

        EC_WRITE_S32(domain1_pd + off_target_position_, joint_position_);

        // 检查是否到达目标
        if (abs(joint_position_ - target_pulses_) <= TOLERANCE && !target_reached_) {
            target_reached_ = true;
            printf("轴 %s 已到达目标位置 %d!\n", axis_name_.c_str(), target_pulses_);
        }
    }
}

void LeisaiServoAxis::handle_leisai_fault_state(uint8_t* domain1_pd, uint16_t error_code) {
    // 实现故障状态处理逻辑
    if (fault_clearing_in_progress_) {
        handle_fault_clear(domain1_pd);
    }
}

void LeisaiServoAxis::handle_sync_motor_control(uint8_t* domain1_pd, int32_t current_pos) {
    // 双轴同步控制逻辑
    if (sync_position_updated_) {
        sync_position_updated_ = false;
        
        // 设置当前轴的目标位置
        EC_WRITE_S32(domain1_pd + off_target_position_, sync_target_position_);
        
        // 同步设置另一个轴的目标位置
        if (sync_axis_) {
            auto sync_domain_pd = domain1_pd; // 同一个从站，使用相同的domain1_pd
            EC_WRITE_S32(sync_domain_pd + sync_axis_->off_target_position_, sync_target_position_);
        }
        
        RCLCPP_DEBUG(rclcpp::get_logger("ethercat_controller"),
                    "双轴同步控制: %s 和 %s 目标位置: %d",
                    axis_name_.c_str(), sync_axis_->get_name().c_str(), sync_target_position_);
    }
    
    // 保持控制字更新
    EC_WRITE_U16(domain1_pd + control_word_, 0x000F);
}

void LeisaiServoAxis::update_sync_target_position(int32_t target_pos) {
    sync_target_position_ = target_pos;
    sync_position_updated_ = true;
}

// 双轴管理器实现
void LeisaiDualAxisManager::register_dual_axis(uint16_t slave_position,
                                             std::shared_ptr<LeisaiServoAxis> axis1,
                                             std::shared_ptr<LeisaiServoAxis> axis2) {
    dual_axes_map_[slave_position] = {axis1, axis2};
    
    // 设置同步关系
    axis1->set_sync_axis(axis2);
    axis2->set_sync_axis(axis1);
}

std::pair<std::shared_ptr<LeisaiServoAxis>, std::shared_ptr<LeisaiServoAxis>>
LeisaiDualAxisManager::get_dual_axis(uint16_t slave_position) {
    auto it = dual_axes_map_.find(slave_position);
    if (it != dual_axes_map_.end()) {
        return it->second;
    }
    return {nullptr, nullptr};
}

bool LeisaiDualAxisManager::is_dual_axis_configured(uint16_t slave_position) const {
    return dual_axes_map_.find(slave_position) != dual_axes_map_.end();
}