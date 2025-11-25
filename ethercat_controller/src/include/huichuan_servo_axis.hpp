#ifndef HUICHUAN_SERVO_AXIS_HPP
#define HUICHUAN_SERVO_AXIS_HPP

#include "servo_axis_base.hpp"
#include <ecrt.h>


// 汇川伺服驱动器配置
#define HUICHUAN_VENDOR_ID 0x00100000
#define HUICHUAN_PRODUCT_CODE 0x000c010d

// 汇川PDO配置
extern ec_pdo_entry_info_t huichuan_slave_pdo_entries[];
extern ec_pdo_info_t huichuan_slave_pdos[];
extern ec_sync_info_t huichuan_slave_syncs[];

class HuichuanServoAxis : public ServoAxisBase {
public:
    HuichuanServoAxis(const std::string& name, uint16_t position, AxisType axis_type);
    virtual ~HuichuanServoAxis() = default;

    // 实现基类纯虚函数
    void configure(ec_master_t* master) override;
    void register_pdo_entries(ec_pdo_entry_reg_t* reg_list, int& index) override;
    void handle_state_machine(uint8_t* domain1_pd) override;

    // 汇川特有功能
    void set_huichuan_specific_parameter(double param);
    double get_huichuan_specific_parameter() const;

private:
    // 汇川特有实现
    void handle_huichuan_initialization(uint8_t* domain1_pd, uint16_t status_word);
    void handle_huichuan_homing(uint8_t* domain1_pd, int32_t current_pos);
    void handle_huichuan_manual_operation(uint8_t* domain1_pd, int32_t current_pos);
    void handle_huichuan_auto_operation(uint8_t* domain1_pd, int32_t current_pos);
    
    double huichuan_specific_param_;
    // 添加缺失的成员变量
    int32_t home_target_position_;
};

#endif // HUICHUAN_SERVO_AXIS_HPP