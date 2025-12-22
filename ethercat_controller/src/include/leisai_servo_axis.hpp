#ifndef LEISAI_SERVO_AXIS_HPP
#define LEISAI_SERVO_AXIS_HPP

#include "servo_axis_base.hpp"
#include <ecrt.h>
#include <map>
#include <memory>

// 雷赛伺服驱动器配置
#define LEISAI_VENDOR_ID 0x00004321
#define LEISAI_PRODUCT_CODE_1 0x0000a400
#define LEISAI_PRODUCT_CODE_2 0x00002700
// 默认产品号（可根据需要选择其中一个作为默认）
#define LEISAI_PRODUCT_CODE_DEFAULT LEISAI_PRODUCT_CODE_1

// 雷赛PDO配置
extern ec_pdo_entry_info_t leisai_slave_pdo_entries[];
extern ec_pdo_info_t leisai_slave_pdos[];
extern ec_sync_info_t leisai_slave_syncs[];

class LeisaiServoAxis : public ServoAxisBase {
public:
    LeisaiServoAxis(const std::string& name, uint16_t position, AxisType axis_type, uint32_t product_code = LEISAI_PRODUCT_CODE_DEFAULT);
    virtual ~LeisaiServoAxis() = default;

    // 实现基类纯虚函数
    void configure(ec_master_t* master) override;
    void register_pdo_entries(ec_pdo_entry_reg_t* reg_list, int& index) override;
    void handle_state_machine(uint8_t* domain1_pd) override;

    // 雷赛特有功能
    void set_leisai_specific_parameter(int param);
    int get_leisai_specific_parameter() const;
    // 添加产品号获取函数
    uint32_t get_product_code() const override { return product_code_; }

    // 双轴控制相关方法
    void set_sync_motor_enabled(bool enabled) { sync_motor_enabled_ = enabled; }
    bool is_sync_motor_enabled() const { return sync_motor_enabled_; }
    void set_sync_axis(std::shared_ptr<LeisaiServoAxis> sync_axis) { sync_axis_ = sync_axis; }
    std::shared_ptr<LeisaiServoAxis> get_sync_axis() const { return sync_axis_; }
    void update_sync_target_position(int32_t target_pos);

private:
    // 雷赛特有实现
    void handle_leisai_initialization(uint8_t* domain1_pd, uint16_t status_word);
    void handle_leisai_manual_operation(uint8_t* domain1_pd, int32_t current_pos);
    void handle_leisai_auto_operation(uint8_t* domain1_pd, int32_t current_pos);
    
    void handle_leisai_ready_state(uint8_t* domain1_pd, uint16_t status_word);
    void handle_leisai_fault_state(uint8_t* domain1_pd, uint16_t error_code);
    
    // 双轴同步控制
    void handle_sync_motor_control(uint8_t* domain1_pd, int32_t current_pos);

    int leisai_specific_param_;
    uint32_t product_code_;  // 存储实际使用的产品号
    
    // 双轴控制相关成员
    bool sync_motor_enabled_ = false;
    std::shared_ptr<LeisaiServoAxis> sync_axis_;
    int32_t sync_target_position_ = 0;
    bool sync_position_updated_ = false;
};

// 双轴管理器类
class LeisaiDualAxisManager {
public:
    static LeisaiDualAxisManager& getInstance() {
        static LeisaiDualAxisManager instance;
        return instance;
    }
    
    void register_dual_axis(uint16_t slave_position, 
                           std::shared_ptr<LeisaiServoAxis> axis1,
                           std::shared_ptr<LeisaiServoAxis> axis2);
    std::pair<std::shared_ptr<LeisaiServoAxis>, std::shared_ptr<LeisaiServoAxis>> 
    get_dual_axis(uint16_t slave_position);
    
    bool is_dual_axis_configured(uint16_t slave_position) const;

private:
    LeisaiDualAxisManager() = default;
    std::map<uint16_t, std::pair<std::shared_ptr<LeisaiServoAxis>, 
                                std::shared_ptr<LeisaiServoAxis>>> dual_axes_map_;
};

#endif // LEISAI_SERVO_AXIS_HPP