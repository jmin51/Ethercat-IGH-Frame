#include "ethercat_node.hpp"
#include <thread>
#include <chrono>
#include <signal.h>
#include <sys/mman.h>
#include <modbus/modbus.h>

// 全局变量定义
std::shared_ptr<EthercatNode> global_node = nullptr;
ec_master_t *master = nullptr;
ec_domain_t *domain1 = nullptr;
uint8_t *domain1_pd = nullptr;
std::atomic<bool> g_should_exit(false);

// 添加缺失的常量定义
const int HOMING_TOLERANCE = 100;
const int HOMING_STEP = 50;
bool running = true;  // 添加缺失的running变量

// 其他全局变量
ec_master_state_t master_state = {};
ec_domain_state_t domain1_state = {};
unsigned int counter = 0;
unsigned int blink = 0;
unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};
pthread_t thread = 0;

// Modbus相关变量
modbus_t *mb_ctx = nullptr;
pthread_t modbus_thread;
volatile int modbus_running = 1;
std::atomic<int> di13_state{0};
std::atomic<bool> homing_completed{false};
std::atomic<bool> system_initialized{false};

EthercatNode::EthercatNode(std::string name) : Node(name) {
    initialize_node();
}

EthercatNode::~EthercatNode() {
    safe_shutdown();
}

void EthercatNode::initialize_node() {
    RCLCPP_INFO(this->get_logger(), "初始化EtherCAT节点: %s", this->get_name());
    
    // 创建发布器和订阅器
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 50);
    system_status_pub_ = this->create_publisher<std_msgs::msg::String>("/system_status", 10);

    displacement_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/displacement_command", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            handle_displacement_command(msg);
        });
        
    control_command_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/control_command", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            handle_control_command_msg(msg);
        });
        
    RCLCPP_INFO(this->get_logger(), "EtherCAT节点初始化完成");
}

void EthercatNode::init_axes(ec_master_t* master) {
    RCLCPP_INFO(this->get_logger(), "开始初始化伺服轴");
    
    // 通过工厂创建不同品牌的伺服轴
    // 示例：创建雷赛X轴和汇川Y轴
    // servo_axes_.push_back(ServoAxisFactory::create_servo_axis(
    //     DriveBrand::LEISAI, "joint1", 0, AxisType::AXIS1));
        
    servo_axes_.push_back(ServoAxisFactory::create_servo_axis(
        DriveBrand::HUICHUAN, "joint2", 0, AxisType::AXIS2));
    
    // 配置每个轴
    for (auto& axis : servo_axes_) {
        axis->configure(master);
    }
    
    last_target_positions_.resize(servo_axes_.size(), 0.0);
    RCLCPP_INFO(this->get_logger(), "伺服轴初始化完成，共 %zu 个轴", servo_axes_.size());
}

void EthercatNode::register_pdo_entries(ec_domain_t* domain1) {
    RCLCPP_INFO(this->get_logger(), "开始注册PDO条目");
    
    const int entries_per_axis = 5;
    int total_entries = servo_axes_.size() * entries_per_axis + 1;
    
    ec_pdo_entry_reg_t* reg_list = new ec_pdo_entry_reg_t[total_entries];
    int index = 0;
    
    // 每个轴注册自己的PDO条目
    for (auto& axis : servo_axes_) {
        axis->register_pdo_entries(reg_list, index);
    }
    
    reg_list[index] = {};
    
    if (ecrt_domain_reg_pdo_entry_list(domain1, reg_list)) {
        RCLCPP_FATAL(this->get_logger(), "PDO注册失败");
    }
    
    delete[] reg_list;
    RCLCPP_INFO(this->get_logger(), "PDO条目注册完成");
}

void EthercatNode::handle_axes_state_machines(uint8_t* domain1_pd) {
    for (auto& axis : servo_axes_) {
        axis->handle_state_machine(domain1_pd);
    }
}

void EthercatNode::add_axis(std::shared_ptr<ServoAxisBase> axis) {
    servo_axes_.push_back(std::move(axis));
    last_target_positions_.resize(servo_axes_.size(), 0.0);
    RCLCPP_INFO(this->get_logger(), "添加新轴: %s", servo_axes_.back()->get_name().c_str());
}

std::vector<std::shared_ptr<ServoAxisBase>>& EthercatNode::get_servo_axes() {
    return servo_axes_;
}

std::vector<ec_slave_config_t*> EthercatNode::get_all_slave_configs() {
    std::vector<ec_slave_config_t*> configs;
    for (auto& axis : servo_axes_) {
        configs.push_back(axis->get_slave_config());
    }
    return configs;
}

void EthercatNode::publish_joint_states() {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    
    // 收集所有轴的名称和位置
    for (auto& axis : servo_axes_) {
        msg.name.push_back(axis->get_name());
    }
    
    for (auto& axis : servo_axes_) {
        int32_t current_pulses = axis->get_actual_position();
        int32_t initial_pulses = axis->get_initial_position();
        double displacement_mm = pulses_to_displacement(current_pulses, initial_pulses);
        msg.position.push_back(displacement_mm);
    }
    
    joint_state_pub_->publish(msg);
    
    static int log_counter = 0;
    if (log_counter++ >= 100) {
        log_counter = 0;
        RCLCPP_DEBUG(this->get_logger(), "发布关节状态: %zu个关节", msg.name.size());
    }
}

void EthercatNode::handle_control_command(const std::string& command) {
    RCLCPP_INFO(this->get_logger(), "收到控制命令: %s", command.c_str());
    
    // 发布系统状态
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "执行命令: " + command;
    system_status_pub_->publish(status_msg);

    if (command == CMD_START_MANUAL) {
        for (auto& axis : servo_axes_) {
            axis->start_manual_mode();
        }
        RCLCPP_INFO(this->get_logger(), "所有轴接收到手动模式启动命令");
        
    } else if (command == CMD_START_AUTO) {
        for (auto& axis : servo_axes_) {
            axis->start_auto_mode();
        }
        RCLCPP_INFO(this->get_logger(), "所有轴接收到自动模式启动命令");
        
    } else if (command == CMD_STOP) {
        for (auto& axis : servo_axes_) {
            axis->stop();
        }
        RCLCPP_INFO(this->get_logger(), "所有轴接收到停止命令");
        
    } else if (command == CMD_CLEAR_FAULT) {
        for (auto& axis : servo_axes_) {
            axis->clear_fault();
        }
        RCLCPP_INFO(this->get_logger(), "所有轴接收到清除故障命令");
        
    } else if (command == CMD_RESET) {
        for (auto& axis : servo_axes_) {
            axis->reset_axis();
        }
        RCLCPP_INFO(this->get_logger(), "所有轴接收到重置命令");
        
    } else {
        RCLCPP_WARN(this->get_logger(), "未知控制命令: %s", command.c_str());
    }
}

void EthercatNode::handle_control_command_msg(const std_msgs::msg::String::SharedPtr msg) {
    handle_control_command(msg->data);
}

void EthercatNode::handle_displacement_command(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {  
    if (servo_axes_.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "等待伺服轴初始化...");
        return;
    }

    if (msg->data.size() < servo_axes_.size()) {
        RCLCPP_WARN(this->get_logger(), "指令轴数(%zu)少于系统轴数(%zu)", 
                    msg->data.size(), servo_axes_.size());
        return;
    }
    
    // 添加位移变化率限制
    static std::vector<double> last_displacements(servo_axes_.size(), 0.0);
    static auto last_time = this->now();
    
    auto current_time = this->now();
    double dt = (current_time - last_time).seconds();
    last_time = current_time;
    
    // 防止dt为0或负值
    if (dt <= 0) dt = 0.01;
    
    // 最大允许变化率：0.01mm/ms = 10mm/s
    const double MAX_RATE_MM_PER_MS = 0.01;
    const double MAX_DELTA_PER_CYCLE = MAX_RATE_MM_PER_MS * dt * 1000;
    
    // 处理每个轴的位移命令
    for (size_t i = 0; i < servo_axes_.size(); ++i) {
        if (i >= msg->data.size()) break;
        
        if (servo_axes_[i]->is_running()) {
            double requested_displacement = msg->data[i];
            double last_displacement = last_displacements[i];
            double delta = requested_displacement - last_displacement;
            
            // 限制变化率
            if (std::abs(delta) > MAX_DELTA_PER_CYCLE) {
                RCLCPP_WARN(this->get_logger(), 
                            "轴%zu位移变化率过大(%.6fmm/ms)，限制为%.6fmm/ms", 
                            i, std::abs(delta)/dt/1000, MAX_RATE_MM_PER_MS);
                
                double limited_delta = (delta > 0) ? MAX_DELTA_PER_CYCLE : -MAX_DELTA_PER_CYCLE;
                requested_displacement = last_displacement + limited_delta;
            }
            
            handle_axis_command(i, requested_displacement);
            last_displacements[i] = requested_displacement;
        }
    }
}

void EthercatNode::handle_axis_command(size_t axis_index, double newTargetPosition) {
    if (axis_index < servo_axes_.size()) {
        servo_axes_[axis_index]->set_target_displacement(newTargetPosition);
        servo_axes_[axis_index]->set_displacement_updated(true);
        
        RCLCPP_DEBUG(this->get_logger(), "轴[%zu] 位移指令: %.3fmm", 
                     axis_index, newTargetPosition);
    }
}

double EthercatNode::pulses_to_displacement(int32_t pulses, int32_t initial_pulses) {
    const double SCREW_LEAD = 0.314;
    const double GEAR_RATIO = 1.0;
    const int PULSES_PER_REV = 10000;
    return static_cast<double>((pulses - initial_pulses) * SCREW_LEAD / (GEAR_RATIO * PULSES_PER_REV));
}