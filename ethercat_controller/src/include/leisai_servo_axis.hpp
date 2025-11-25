#ifndef LEISAI_SERVO_AXIS_HPP
#define LEISAI_SERVO_AXIS_HPP

#include "servo_axis_base.hpp"
#include <ecrt.h>

// 雷赛伺服驱动器配置
#define LEISAI_VENDOR_ID 0x00100000
#define LEISAI_PRODUCT_CODE 0x000c010d

// 雷赛PDO配置
extern ec_pdo_entry_info_t leisai_slave_pdo_entries[];
extern ec_pdo_info_t leisai_slave_pdos[];
extern ec_sync_info_t leisai_slave_syncs[];

class LeisaiServoAxis : public ServoAxisBase {
public:
    LeisaiServoAxis(const std::string& name, uint16_t position, AxisType axis_type);
    virtual ~LeisaiServoAxis() = default;

    // 实现基类纯虚函数
    void configure(ec_master_t* master) override;
    void register_pdo_entries(ec_pdo_entry_reg_t* reg_list, int& index) override;
    void handle_state_machine(uint8_t* domain1_pd) override;

    // 雷赛特有功能
    void set_leisai_specific_parameter(int param);
    int get_leisai_specific_parameter() const;

private:
    // 雷赛特有实现
    void handle_leisai_initialization(uint8_t* domain1_pd, uint16_t status_word);
    void handle_leisai_homing(uint8_t* domain1_pd, int32_t current_pos);
    void handle_leisai_manual_operation(uint8_t* domain1_pd, int32_t current_pos);
    void handle_leisai_auto_operation(uint8_t* domain1_pd, int32_t current_pos);
    
    void handle_leisai_ready_state(uint8_t* domain1_pd, uint16_t status_word);
    void handle_leisai_fault_state(uint8_t* domain1_pd, uint16_t error_code);

    int leisai_specific_param_;
};

#endif // LEISAI_SERVO_AXIS_HPP