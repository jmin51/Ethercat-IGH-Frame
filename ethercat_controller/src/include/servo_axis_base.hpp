#ifndef SERVO_AXIS_BASE_HPP
#define SERVO_AXIS_BASE_HPP

#include <ecrt.h>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>

// 状态机状态定义
enum class AxisState {
    UNINITIALIZED = 0,
    INITIALIZING = 1,
    READY = 2,
    MANUAL_MODE = 3,
    AUTO_MODE = 4,
    STOPPED = 5,
    FAULT = 6
};

enum class OperationMode {
    MANUAL = 0,
    AUTO = 1
};

enum class AxisType {
    AXIS1 = 0,
    AXIS2 = 1
};

enum class DriveBrand {
    LEISAI = 0,
    HUICHUAN = 1
};

class ServoAxisBase {
public:
    ServoAxisBase(const std::string& name, uint16_t position, AxisType axis_type, DriveBrand brand, uint32_t product_code);
    virtual ~ServoAxisBase() = default;
    // 添加获取产品号的虚函数
    virtual uint32_t get_product_code() const = 0;

    // 纯虚函数 - 必须由子类实现
    virtual void configure(ec_master_t* master) = 0;
    virtual void register_pdo_entries(ec_pdo_entry_reg_t* reg_list, int& index) = 0;
    virtual void handle_state_machine(uint8_t* domain1_pd) = 0;
    
    // 虚函数 - 可以有默认实现
    virtual void set_target_displacement(double displacement);
    virtual void set_displacement_updated(bool updated);
    virtual void stop();
    virtual void start_manual_mode();
    virtual void start_auto_mode();
    virtual void clear_fault();
    virtual void reset_axis();

    // 获取函数
    virtual std::string get_name() const;
    virtual AxisState get_current_state() const;
    virtual OperationMode get_operation_mode() const;
    virtual bool is_ready() const;
    virtual bool is_running() const;
    virtual bool is_homing_completed() const;
    virtual bool is_homing_in_progress() const;
    virtual ec_slave_config_t* get_slave_config();
    virtual unsigned int get_control_word_offset() const;
    virtual int32_t get_actual_position() const;
    virtual int32_t get_initial_position() const;
    virtual DriveBrand get_brand() const;
    // 添加点动控制方法
    virtual void jog_forward() { 
        jog_forward_requested_ = true;
        jog_reverse_requested_ = false;
        jog_stop_requested_ = false;
    }
    
    virtual void jog_reverse() {
        jog_reverse_requested_ = true; 
        jog_forward_requested_ = false;
        jog_stop_requested_ = false;
    }
    
    virtual void jog_stop() {
        jog_stop_requested_ = true;
        jog_forward_requested_ = false;
        jog_reverse_requested_ = false;
    }
    
    virtual bool is_jogging() const {
        return jog_forward_requested_ || jog_reverse_requested_;
    }
    // 最新移动从protected
    virtual int32_t displacement_to_pulses(double displacement_mm);

protected:
    // 保护成员变量 - 子类可以访问
    std::string axis_name_;
    uint16_t slave_position_;
    AxisType axis_type_;
    DriveBrand brand_;
    uint32_t product_code_;  // 在基类中存储产品号
    
    ec_slave_config_t* sc_;
    unsigned int control_word_;
    unsigned int status_word_;
    unsigned int off_target_position_;
    unsigned int off_actual_position_;
    unsigned int off_error_code_;
    
    AxisState current_state_;
    OperationMode operation_mode_;
    
    bool position_initialized_;
    volatile int32_t target_pulses_;  // 告诉编译器"这个变量可能被外部访问"
    double target_displacement_;
    bool displacement_updated_;
    int32_t joint_position_;
    int32_t initial_position_;
    int32_t target_reached_;
    
    bool homing_in_progress_;
    bool homing_completed_;
    
    // 控制标志
    bool start_manual_requested_;
    bool start_auto_requested_;
    bool clear_fault_requested_;
    bool reset_requested_;
    bool fault_clearing_in_progress_;
    int fault_clear_step_;
    int fault_clear_counter_;

    // 保护方法 - 子类可以重写或使用
    virtual void initialize_members();
    // virtual int32_t displacement_to_pulses(double displacement_mm);
    virtual double pulses_to_displacement(int32_t pulses);
    virtual void check_state_changes(uint16_t read_status_word, uint16_t error_code);
    virtual void handle_fault_clear(uint8_t* domain1_pd);
    virtual void check_system_initialization();

    // 速度控制相关
    double jog_speed_;           // 点动速度 (mm/s)
    bool jog_forward_requested_; // 正转请求
    bool jog_reverse_requested_; // 反转请求
    bool jog_stop_requested_;    // 停止请求
    
    // 速度控制参数
    const double DEFAULT_JOG_SPEED = 0.262; // 默认点动速度 0.262mm/s 或者是 50rpm/min

    // 新增逐步逼近相关变量
    int32_t target_offset_;           // 目标偏移量
    int direction_flag_;              // 0: 无方向 1: 正转 -1: 反转
    int32_t new_target_;              // 新目标位置
    
    void gradual_approach(int32_t target_pulses, uint8_t* domain1_pd) {
        const int32_t MAX_STEP = 50; // 最大步进脉冲数
        int32_t error = target_pulses - joint_position_;
        int32_t step = (abs(error) > MAX_STEP) ? 
                      ((error > 0) ? MAX_STEP : -MAX_STEP) : error;
        
        joint_position_ += step;
        EC_WRITE_S32(domain1_pd + off_target_position_, joint_position_);
    }
};

#endif // SERVO_AXIS_BASE_HPP