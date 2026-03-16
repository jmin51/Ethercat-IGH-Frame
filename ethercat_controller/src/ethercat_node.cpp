#include "ethercat_node.hpp"
#include <thread>
#include <chrono>
#include <signal.h>
#include <sys/mman.h>
#include <modbus/modbus.h>
#include <string>
#include <sstream>

#define CONTROL_SOURCE_IO 0  // 1:使用IO控制手自动模式, 0:使用话题控制
// 全局变量定义
std::shared_ptr<EthercatNode> global_node = nullptr;
ec_master_t *master = nullptr;
ec_domain_t *domain1 = nullptr;
uint8_t *domain1_pd = nullptr;
std::atomic<bool> g_should_exit(false);
std::atomic<bool> node_shutting_down_{false};
// 全局变量定义
std::atomic<bool> g_system_running(false);
std::atomic<bool> g_start_button_pressed(false);
std::atomic<bool> g_pause_button_pressed(false);
std::atomic<bool> g_reset_button_pressed(false);  // 复位按钮状态

// 添加缺失的常量定义
// const int HOMING_TOLERANCE = 100;
// const int HOMING_STEP = 50;
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

EthercatNode::EthercatNode(std::string name) : Node(name) {
    initialize_node();
}

EthercatNode::~EthercatNode() {
    node_shutting_down_.store(true);
    stop_io_monitoring();  // 停止IO监控
}

void EthercatNode::initialize_node() {
    RCLCPP_INFO(this->get_logger(), "初始化EtherCAT节点: %s", this->get_name());
    
    // 创建发布器和订阅器
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 50);
    system_status_pub_ = this->create_publisher<std_msgs::msg::String>("/system_status", 10);
    io_status_pub_ = this->create_publisher<std_msgs::msg::String>("/io_status", 10);  // 新增IO状态发布器

    // 添加与Python节点通信的发布器和订阅器
    py_io_status_pub_ = this->create_publisher<std_msgs::msg::String>("/py_io_status", 10);
    py_control_command_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/py_control_command", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            this->handle_py_control_command(msg);
        });

    displacement_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/displacement_command", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            handle_displacement_command(msg);
        });
        
    control_command_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/control_command", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            handle_control_command_msg(msg);
        });
    // 添加点动指令订阅
    jog_command_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/jog_command", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            handle_jog_command(msg);
        });
    // 新增：点动速度设置订阅
    jog_speed_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/jog_speed_command", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            handle_jog_speed_command(msg);
        });

    // 添加入库流程话题订阅器
    warehouse_start_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/warehouse_start", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::UInt8::SharedPtr msg) {
            handle_warehouse_start(msg);
        });
        
    warehouse_stop_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/warehouse_stop", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::Empty::SharedPtr msg) {
            handle_warehouse_stop(msg);
        });
    // 添加出库流程话题订阅器
    outbound_start_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/outbound_start", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::UInt8::SharedPtr msg) {
            handle_outbound_start(msg);
        });
        
    outbound_stop_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/outbound_stop", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::Empty::SharedPtr msg) {
            handle_outbound_stop(msg);
        });
    // 添加DO控制话题订阅
    do_control_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/do_control", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            handle_do_control(msg);
        });
    // 添加故障码发布器初始化
    fault_code_pub_ = this->create_publisher<std_msgs::msg::String>("/fault_code", 10);
    // 初始化故障管理器（升级为故障管理系统）
    fault_manager_ = std::make_unique<fault_management::FaultManagementSystem>(this);
    fault_manager_->set_fault_publisher(fault_code_pub_);
    fault_manager_->initialize();  // 启用日志回调自动捕获

    // +++ 新增：创建并启动 10ms 周期定时器 +++
    periodic_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // 100ms 周期
        std::bind(&EthercatNode::periodic_timer_callback, this)
    );
    RCLCPP_INFO(this->get_logger(), "10ms 周期定时器已创建并启动");

    // 初始化IO互斥锁
    pthread_mutex_init(&io_mutex_, nullptr);
    
    RCLCPP_INFO(this->get_logger(), "EtherCAT节点初始化完成");

    // 初始化板宽控制
    initialize_board_width_parameters();
    
    // 创建板宽设定订阅器
    board_width_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/board_width_command", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
            this->handle_board_width_command(msg);
        });
    
    // 创建板宽状态发布器
    board_width_status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/board_width_status", rclcpp::QoS(10).reliable());
    
    // +++ 新增：创建axis3板宽设定订阅器 +++
    axis3_width_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/axis3_width_command", rclcpp::QoS(10).reliable(), // 新话题名
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
            this->handle_axis3_width_command(msg); // 绑定新的处理函数
        });
    
    // +++ 新增：创建axis3板宽状态发布器 +++
    axis3_width_status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/axis3_width_status", rclcpp::QoS(10).reliable()); // 新话题名

    RCLCPP_INFO(this->get_logger(), "板宽控制模块初始化完成");
}

// +++ 新增：定时器回调函数实现 +++
void EthercatNode::periodic_timer_callback() {
    // 添加关闭检查
    if (node_shutting_down_.load() || !rclcpp::ok()) {
        return;
    }
    
    // 1. 发布关节状态
    publish_joint_states();
    
    // 2. 检查层移动完成状态
    check_layer_motion_completion();

    // 3. 发布故障状态
    if (fault_manager_) {
        fault_manager_->publish_fault_status();
    }
    
    // 4. 启动后校正板宽（所有轴进入自动模式后只执行一次）
    if (!board_width_calibrated_ && are_all_axes_in_auto_mode()) {
        calibrate_board_width_from_position();
    }
}

void EthercatNode::handle_py_control_command(const std_msgs::msg::String::SharedPtr msg) {
    std::string command = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到Python控制命令: %s", command.c_str());
    handle_control_command(command);
}

void EthercatNode::publish_py_io_status(const DI_Interface& di) {
    auto msg = std_msgs::msg::String();
    std::stringstream ss;
    
    ss << "DI12:" << di.buffer_in_position
       << ",DI13:" << di.buffer_out_position
       << ",DI14:" << di.conveyor_in_position
       << ",DI15:" << di.conveyor_out_position;
    
    msg.data = ss.str();
    py_io_status_pub_->publish(msg);
}

// 新增：在轴初始化后调用的方法
void EthercatNode::initialize_after_axes() {
    RCLCPP_INFO(this->get_logger(), "开始初始化业务逻辑模块（延迟初始化）");
    
    // 检查轴是否已初始化
    if (servo_axes_.empty()) {
        print_error("伺服轴未初始化，无法初始化业务逻辑模块");
        return;
    }
    
    // 初始化业务逻辑处理器
    initialize_layer_processor();
    RCLCPP_INFO(this->get_logger(), "业务逻辑模块初始化完成");
}

void EthercatNode::init_axes(ec_master_t* master) {
    RCLCPP_INFO(this->get_logger(), "开始初始化伺服轴");
    
    // 清空现有轴对象，确保重新启动时初值正确
    stop_io_monitoring();  // 等待IO线程完全退出
    servo_axes_.clear();
    last_target_positions_.clear();  // 同时清空位置记录
    // +++ 新增：重置板宽控制状态 +++
    current_board_width_ = 15.0;   // 重置为默认板宽
    target_board_width_ = 15.0;
    board_width_moving_ = false;
    board_width_updated_.store(false);
    // +++ 新增：重置axis3板宽控制状态 +++
    axis3_current_width_ = 15.0; // 与initialize中默认值一致
    axis3_target_width_ = 15.0;
    axis3_width_moving_ = false;
    axis3_width_updated_.store(false);
    
    // 重置板宽校正标志（下次启动时重新校正）
    board_width_calibrated_ = false;

    // 添加从站1：第二个雷赛双轴驱动器
    auto axis1_1 = ServoAxisFactory::create_servo_axis(
        DriveBrand::LEISAI, "axis1_1", 0, AxisType::AXIS1, LEISAI_PRODUCT_CODE_3);  // 从站位置=1
    auto axis1_2 = ServoAxisFactory::create_servo_axis(
        DriveBrand::LEISAI, "axis1_2", 0, AxisType::AXIS2, LEISAI_PRODUCT_CODE_3);  // 从站位置=1（同一个从站）

    servo_axes_.push_back(std::move(axis1_1));
    servo_axes_.push_back(std::move(axis1_2));
    // 创建雷赛双轴驱动器 - 从站2
    auto axis2_1 = ServoAxisFactory::create_servo_axis(
        DriveBrand::LEISAI, "axis2_1", 1, AxisType::AXIS1, LEISAI_PRODUCT_CODE_1);
    auto axis2_2 = ServoAxisFactory::create_servo_axis(
        DriveBrand::LEISAI, "axis2_2", 1, AxisType::AXIS2, LEISAI_PRODUCT_CODE_1);

    servo_axes_.push_back(std::move(axis2_1));
    servo_axes_.push_back(std::move(axis2_2));
    
    servo_axes_.push_back(ServoAxisFactory::create_servo_axis(
        DriveBrand::LEISAI, "axis3", 2, AxisType::AXIS1, LEISAI_PRODUCT_CODE_2, 1.8)); // 轴，减速比2.0*20/22 =1.818,调式结果是1.8
    servo_axes_.push_back(ServoAxisFactory::create_servo_axis(
        DriveBrand::HUICHUAN, "axis4", 3, AxisType::AXIS1, 0, 7.34)); // 汇川轴，减速比9.0*28/34 =7.411,调式结果是7.5
    servo_axes_.push_back(ServoAxisFactory::create_servo_axis(
        DriveBrand::HUICHUAN, "axis5", 4, AxisType::AXIS1));
    // 配置每个轴
    for (auto& axis : servo_axes_) {
        axis->configure(master);
    }
    
    // 新增：为每个轴设置独立的初始点动速度
    for (auto& axis : servo_axes_) {
        std::string name = axis->get_name();
        if (name == "axis4") {
            axis->set_jog_speed(8.0); // 将 axis4 的点动速度初始化为 8 mm/s
            RCLCPP_INFO(this->get_logger(), "轴 %s 初始点动速度已设为: 8.0 mm/s", name.c_str());
        } else if (name == "axis1_1" || name == "axis1_2" || name == "axis2_1" || name == "axis2_2") {
            // 示例：为 axis1_1 和 axis1_2 设置其他速度
            axis->set_jog_speed(120.0);
            RCLCPP_INFO(this->get_logger(), "轴 %s 初始点动速度已设为: 120.0 mm/s", name.c_str());
        } else {
            // 其他轴保持默认速度（DEFAULT_JOG_SPEED，当前为20.0 mm/s）
            RCLCPP_DEBUG(this->get_logger(), "轴 %s 使用默认点动速度: %.1f mm/s", 
                         name.c_str(), axis->get_jog_speed());
        }
    }
    last_target_positions_.resize(servo_axes_.size(), 0.0);
    start_io_monitoring();  // 在轴初始化后启动IO监控，确保轴配置完成后才开始监控IO状态 todo3.11不能删，删了后暂停会出问题
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
    // 添加关闭检查
    if (node_shutting_down_.load() || !rclcpp::ok()) {
        return;
    }

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

    // 检查板宽调整是否完成
    if (board_width_moving_) {
        int axis4_index = find_axis4_index();
        if (axis4_index != -1) {
            auto& axis4 = servo_axes_[axis4_index];
            
            // 检查是否到达目标
            if (axis4->is_target_reached()) {
                // 更新当前板宽
                current_board_width_ = target_board_width_;
                board_width_moving_ = false;
                
                RCLCPP_INFO(this->get_logger(), 
                           "板宽调整完成: 当前板宽%.1fcm", current_board_width_);
                
                publish_board_width_status(current_board_width_, target_board_width_, 
                                         false, "调整完成");
            }
        }
    }
    // +++ 新增：检查axis3板宽调整是否完成 +++
    if (axis3_width_moving_) {
        int axis3_index = find_axis3_index();
        if (axis3_index != -1) {
            auto& axis3 = servo_axes_[axis3_index];
            if (axis3->is_target_reached()) {
                axis3_current_width_ = axis3_target_width_;
                axis3_width_moving_ = false;
                RCLCPP_INFO(this->get_logger(), 
                           "[Axis3] 板宽调整完成: 当前板宽%.1fcm", axis3_current_width_);
                publish_axis3_width_status(axis3_current_width_, axis3_target_width_, 
                                         false, "调整完成");
            }
        }
    }
}

void EthercatNode::handle_control_command(const std::string& command) {
    if (node_shutting_down_.load() || !rclcpp::ok()) {
        return;
    }
    RCLCPP_INFO(this->get_logger(), "收到控制命令: %s", command.c_str());
    
    // 发布系统状态
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "执行命令: " + command;
    system_status_pub_->publish(status_msg);

    if (command == CMD_START_MANUAL) {
        // 手动模式
        for (auto& axis : servo_axes_) {
            axis->start_manual_mode();
        }
        RCLCPP_INFO(this->get_logger(), "所有轴接收到手动模式启动命令");
        
    } else if (command == CMD_START_AUTO) {
        // 自动模式
        bool any_axis_in_manual = false;
        for (auto& axis : servo_axes_) {
            if (axis->get_operation_mode() == OperationMode::MANUAL) {
                any_axis_in_manual = true;
                break;
            }
        }
        
        if (any_axis_in_manual && fault_manager_) {
            // 手动模式下收到自动指令，上报故障
            uint16_t fault_code = fault_manager_->handle_auto_command_in_manual_mode();
            std::stringstream warn_ss;
            warn_ss << "手动模式下收到自动指令，已上报故障码: 0x" << std::hex << std::setw(4) << std::setfill('0') << fault_code;
            print_warning(warn_ss.str());
        }
        
        // 仍然尝试启动自动模式（轴内部会处理请求）
        for (auto& axis : servo_axes_) {
            axis->start_auto_mode();
        }
        RCLCPP_INFO(this->get_logger(), "所有轴接收到自动模式启动命令");
        
    } else if (command == CMD_STOP) {
        // 停止模式
        for (auto& axis : servo_axes_) {
            axis->stop();
        }
        RCLCPP_INFO(this->get_logger(), "所有轴接收到停止命令");
        // 清除所有系统故障和告警
        if (fault_manager_) {
            fault_manager_->clear_all_faults();
            RCLCPP_INFO(this->get_logger(), "已清除所有系统故障和告警");
        }
        
    } else if (command == CMD_CLEAR_FAULT) {
        // 清除故障
        for (auto& axis : servo_axes_) {
            axis->clear_fault();
        }
        RCLCPP_INFO(this->get_logger(), "所有轴接收到清除故障命令");
        
    } else if (command == CMD_RESET) {
        // 重置
        for (auto& axis : servo_axes_) {
            axis->reset_axis();
        }
        RCLCPP_INFO(this->get_logger(), "所有轴接收到重置命令");
        
    } else if (command == "clear_all_faults") {
        // 清除所有系统故障和告警
        if (fault_manager_) {
            fault_manager_->clear_all_faults();
            RCLCPP_INFO(this->get_logger(), "已清除所有系统故障和告警");
        }
        
    } else {
        print_warning("未知控制命令: " + command);
    }
}

void EthercatNode::handle_control_command_msg(const std_msgs::msg::String::SharedPtr msg) {
    handle_control_command(msg->data);
}

// 实现新的位移指令处理函数
void EthercatNode::handle_displacement_command(const std_msgs::msg::String::SharedPtr msg) {  
    if (servo_axes_.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "等待伺服轴初始化...");
        return;
    }

    std::string command = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到位移指令: %s", command.c_str());
    
    // 解析命令
    std::vector<std::pair<std::string, double>> axis_commands;
    if (!parse_displacement_command(command, axis_commands)) {
        print_error("位移指令解析失败: " + command);
        return;
    }
    
    // 处理每个轴的位移命令
    for (const auto& axis_cmd : axis_commands) {
        const std::string& axis_name = axis_cmd.first;
        double displacement = axis_cmd.second;
        
        printf("解析位移指令: %s = %.6fmm\n", axis_name.c_str(), displacement);
        
        // 查找对应的轴
        bool axis_found = false;
        for (size_t i = 0; i < servo_axes_.size(); ++i) {
            if (servo_axes_[i]->get_name() == axis_name) {
                    // 单轴控制
                    if (servo_axes_[i]->is_running()) {
                        handle_axis_command(i, displacement);
                    }
                    axis_found = true;
                break;
            }
        }
        
        if (!axis_found) {
            print_error("未找到轴: " + axis_name);
        }
    }
}

// 实现位移指令解析函数
bool EthercatNode::parse_displacement_command(const std::string& command, 
                                             std::vector<std::pair<std::string, double>>& axis_commands) {
    axis_commands.clear();
    
    // 支持多种格式：
    // 1. 单个轴: "axis4:1.0"
    // 2. 多个轴: "axis4:1.0;axis5:2.0"
    
    // 解析新格式
    std::vector<std::string> parts;
    size_t start = 0;
    size_t end = command.find(';');
    
    while (end != std::string::npos) {
        parts.push_back(command.substr(start, end - start));
        start = end + 1;
        end = command.find(';', start);
    }
    parts.push_back(command.substr(start));
    
    for (const auto& part : parts) {
        size_t colon_pos = part.find(':');
        if (colon_pos == std::string::npos) {
            print_error("无效指令格式，缺少冒号: " + part);
            return false;
        }
        
        std::string axis_name = part.substr(0, colon_pos);
        std::string value_str = part.substr(colon_pos + 1);
        
        // 去除空格
        axis_name.erase(0, axis_name.find_first_not_of(" \t"));
        axis_name.erase(axis_name.find_last_not_of(" \t") + 1);
        value_str.erase(0, value_str.find_first_not_of(" \t"));
        value_str.erase(value_str.find_last_not_of(" \t") + 1);
        
        if (axis_name.empty() || value_str.empty()) {
            print_error("轴名或值为空: " + part);
            return false;
        }
        
        try {
            double value = std::stod(value_str);
            axis_commands.push_back({axis_name, value});
        } catch (const std::exception& e) {
            print_error("数值转换失败: " + value_str + ", 错误: " + e.what());
            return false;
        }
    }
    
    return true;
}

void EthercatNode::handle_axis_command(size_t axis_index, double newTargetPosition) {
    if (axis_index < servo_axes_.size()) {
        servo_axes_[axis_index]->set_target_displacement(newTargetPosition);
        servo_axes_[axis_index]->set_displacement_updated(true);
        
        RCLCPP_DEBUG(this->get_logger(), "轴[%zu] 位移指令: %.3fmm", 
                     axis_index, newTargetPosition);
        // // 详细状态检查
        // RCLCPP_INFO(this->get_logger(), "运行状态=%s, 当前状态=%d", 
        //         axis->is_running() ? "是" : "否",
        //         static_cast<int>(axis->get_current_state()));
    }
}

double EthercatNode::pulses_to_displacement(int32_t pulses, int32_t initial_pulses) {
    const double SCREW_LEAD = 0.314;
    const double GEAR_RATIO = 1.0;
    const int PULSES_PER_REV = 10000;
    return static_cast<double>((pulses - initial_pulses) * SCREW_LEAD / (GEAR_RATIO * PULSES_PER_REV));
}

void EthercatNode::handle_jog_command(const std_msgs::msg::String::SharedPtr msg) {
    std::string command = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到点动命令: %s", command.c_str());
    
//     for (auto& axis : servo_axes_) {
//         if (command == "forward") {
//             axis->jog_forward();
//             RCLCPP_INFO(this->get_logger(), "轴 %s 开始正转", axis->get_name().c_str());
//         } else if (command == "reverse") {
//             axis->jog_reverse();
//             RCLCPP_INFO(this->get_logger(), "轴 %s 开始反转", axis->get_name().c_str());
//         } else if (command == "stop") {
//             axis->jog_stop();
//             RCLCPP_INFO(this->get_logger(), "轴 %s 停止", axis->get_name().c_str());
//         }
//     }
    // 解析格式："axis1_1:forward" 或 "axis1_1:reverse" 或 "axis1_1:stop"
    size_t colon_pos = command.find(':');
    if (colon_pos == std::string::npos) {
        print_error("无效命令格式，应为 '轴名:命令'");
        return;
    }
    
    std::string axis_name = command.substr(0, colon_pos);
    std::string jog_cmd = command.substr(colon_pos + 1);
    
    // if (axis_name == "all") {
    //     // 控制所有轴
    //     for (auto& axis : servo_axes_) {
    //         execute_jog_command(axis, jog_cmd);
    //     }
    // } else {
        // 控制指定轴
        for (auto& axis : servo_axes_) {
            if (axis->get_name() == axis_name) {
                if (jog_cmd == "forward") {
                    axis->jog_forward();
                } else if (jog_cmd == "reverse") {
                    axis->jog_reverse();
                } else if (jog_cmd == "stop") {
                    axis->jog_stop();
                }
                break;
            }
        }
    // }
}

// ========== IO模块相关函数 ==========

void EthercatNode::start_io_monitoring() {
    if (io_running_.load()) {
        RCLCPP_WARN(this->get_logger(), "IO监控线程已在运行");
        return;
    }
    
    // 初始化Modbus接口
    const char* di_ip = is_di_module_enabled() ? DI_DEVICE_IP : NULL;
    const char* do_ip = is_do_module_enabled() ? DO_DEVICE_IP : NULL;
    
    if (init_modbus_interface(di_ip, MODBUS_PORT, MODBUS_SLAVE_ID,
                             do_ip, MODBUS_PORT, MODBUS_SLAVE_ID) != 0) {
        print_error("Modbus初始化失败");
        return;
    }
    
    io_running_.store(true);
    
    // 创建IO监控线程
    if (pthread_create(&io_thread_, nullptr, io_monitor_thread, this) != 0) {
        print_error("创建IO监控线程失败");
        io_running_.store(false);
        cleanup_modbus_interface();
        return;
    }
    
    pthread_setname_np(io_thread_, "io-monitor");
    RCLCPP_INFO(this->get_logger(), "IO监控线程启动成功");
}

void EthercatNode::stop_io_monitoring() {
    if (!io_running_.load()) {
        return;
    }
    
    io_running_.store(false);
    
    if (io_thread_) {
        pthread_join(io_thread_, nullptr);
        io_thread_ = 0;
    }
    
    cleanup_modbus_interface();
    pthread_mutex_destroy(&io_mutex_);
    RCLCPP_INFO(this->get_logger(), "IO监控线程已停止");
}

void EthercatNode::handle_io_signals(DI_Interface di) {
    // 发布IO状态到Python节点
    publish_py_io_status(di);
    
    monitor_di_changes(di);
    pthread_mutex_lock(&io_mutex_);
    current_di_status_ = di;
    pthread_mutex_unlock(&io_mutex_);

    // 删除原有的BusinessLogicProcessor处理逻辑
    // 业务逻辑现在由Python节点处理
    // 启动按钮处理（上升沿触发）
    static bool last_start_button = false;
    if (di.start_button && !last_start_button) {
        g_start_button_pressed.store(true);
        RCLCPP_INFO(this->get_logger(), "启动按钮按下，开始启动系统");
    }
    last_start_button = di.start_button;

    // 暂停按钮处理（上升沿触发）
    static bool last_pause_button = false;
    if (di.pause_button && !last_pause_button) {
        g_pause_button_pressed.store(true);
        RCLCPP_INFO(this->get_logger(), "暂停按钮按下，开始安全关闭");
    }
    last_pause_button = di.pause_button;

    // +++ 新增：复位按钮处理（上升沿触发） +++
    static bool last_reset_button = false;
    if (di.reset_button && !last_reset_button) {
        g_reset_button_pressed.store(true);
        RCLCPP_INFO(this->get_logger(), "复位按钮按下，准备回原流程");
    }
    last_reset_button = di.reset_button;

    // 新增：IO控制模式切换（仅在宏开关启用时生效）
#if CONTROL_SOURCE_IO
    bool current_manual_auto_state = di.manual_auto_button; // 当前按钮状态（DI04）

    // 检查所有轴是否都处于READY状态
    bool all_axes_ready = true;
    for (auto& axis : servo_axes_) {
        AxisState state = axis->get_current_state();
        if (state != AxisState::READY) {
            all_axes_ready = false;
            break;
        }
    }
    
    // 如果所有轴都处于READY状态，尝试模式切换
    if (all_axes_ready) {
        RCLCPP_INFO(this->get_logger(), "所有轴已就绪，准备模式切换。手自动按钮状态: %s", 
                   current_manual_auto_state ? "自动" : "手动");
        
        // 触发模式切换命令
        std::string command = current_manual_auto_state ? CMD_START_AUTO : CMD_START_MANUAL;
        handle_control_command(command); // 通过统一命令处理
        
        RCLCPP_INFO(this->get_logger(), "已发送%s模式切换命令", 
                   current_manual_auto_state ? "自动" : "手动");
    } else {
        // 记录哪些轴未就绪（用于调试）
        static int log_counter = 0;
        if (log_counter++ % 500 == 0) { // 每500次记录一次，避免刷屏
            std::stringstream ss;
            ss << "等待所有轴就绪: ";
            for (size_t i = 0; i < servo_axes_.size(); ++i) {
                AxisState state = servo_axes_[i]->get_current_state();
                if (state != AxisState::READY) {
                    ss << servo_axes_[i]->get_name() << "=" << static_cast<int>(state) << " ";
                }
            }
            RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
            log_counter = 0;
        }
    }
#endif
    // 发布IO状态
    publish_io_status();
}

void EthercatNode::publish_io_status() {
    if (node_shutting_down_.load() || !rclcpp::ok()) {
        return;
    }
    
    auto msg = std_msgs::msg::String();
    std::stringstream ss;
    
    pthread_mutex_lock(&io_mutex_);
    
    // 读取当前DI和DO状态
    DI_Interface di = current_di_status_;
    DO_Interface do_status = get_current_do_state();
    
    // 按照表格顺序发布DI状态 (0-22)
    ss << "DI状态: ";
    ss << "DI00:" << (di.start_button ? "1" : "0") << ",";        // 启动按钮
    ss << "DI01:" << (di.reset_button ? "1" : "0") << ",";        // 复位按钮
    ss << "DI02:" << (di.pause_button ? "1" : "0") << ",";        // 暂停按钮
    ss << "DI03:" << (di.manual_auto_button ? "1" : "0") << ",";  // 手自动按钮
    ss << "DI04:" << (di.emergency_stop ? "1" : "0") << ",";      // 急停按钮
    ss << "DI05:" << (di.air_supply ? "1" : "0") << ",";          // 气源输入
    ss << "DI06:" << (di.safety_door_1 ? "1" : "0") << ",";       // 安全门检1
    ss << "DI07:" << (di.safety_door_2 ? "1" : "0") << ",";       // 安全门检2
    ss << "DI08:" << (di.feed_product_detect ? "1" : "0") << ","; // 入料产品检测
    ss << "DI09:" << (di.buffer_sensor_1 ? "1" : "0") << ",";     // 缓存架对射1
    ss << "DI10:" << (di.buffer_sensor_2 ? "1" : "0") << ",";     // 缓存架对射2
    ss << "DI11:" << (di.buffer_in_position ? "1" : "0") << ",";  // 缓存架入料产品到位检测
    ss << "DI12:" << (di.buffer_out_position ? "1" : "0") << ","; // 缓存架出料产品到位检测
    ss << "DI13:" << (di.conveyor_in_position ? "1" : "0") << ","; // 接驳台入料产品到位检测
    ss << "DI14:" << (di.conveyor_out_position ? "1" : "0") << ","; // 接驳台出料产品到位检测
    ss << "DI15:" << (di.lift_cylinder1_up ? "1" : "0") << ",";  // 顶升气缸1上升到位
    ss << "DI16:" << (di.lift_cylinder1_down ? "1" : "0") << ","; // 顶升气缸1下降到位
    ss << "DI17:" << (di.lift_cylinder2_up ? "1" : "0") << ",";  // 顶升气缸2上升到位
    ss << "DI18:" << (di.lift_cylinder2_down ? "1" : "0") << ","; // 顶升气缸2下降到位
    ss << "DI19:" << (di.gear_cylinder1_retract ? "1" : "0") << ","; // 齿轮对接气缸1伸出到位
    ss << "DI20:" << (di.gear_cylinder1_retract ? "1" : "0") << ","; // 齿轮对接气缸1缩回到位
    ss << "DI21:" << (di.gear_cylinder2_extend ? "1" : "0") << ","; // 齿轮对接气缸2伸出到位
    ss << "DI22:" << (di.gear_cylinder2_retract ? "1" : "0");       // 齿轮对接气缸2缩回到位

    ss << " | DO状态: ";
    
    // 按照表格顺序发布DO状态 (0-13)
    ss << "DO00:" << (do_status.start_button_light ? "1" : "0") << ",";  // 启动按钮灯
    ss << "DO01:" << (do_status.reset_button_light ? "1" : "0") << ",";  // 复位按钮灯
    ss << "DO02:" << (do_status.pause_button_light ? "1" : "0") << ",";  // 暂停按钮灯
    ss << "DO03:" << (do_status.buzzer ? "1" : "0") << ",";              // 蜂鸣器
    ss << "DO04:" << (do_status.red_light ? "1" : "0") << ",";           // 三色红灯
    ss << "DO05:" << (do_status.yellow_light ? "1" : "0") << ",";        // 三色黄灯
    ss << "DO06:" << (do_status.green_light ? "1" : "0") << ",";         // 三色绿灯
    ss << "DO07:0,";                                                    // 预留
    ss << "DO08:0,";                                                    // 预留
    ss << "DO09:0,";                                                    // 预留
    ss << "DO10:" << (do_status.lift_cylinder_down ? "1" : "0") << ","; // 顶升气缸下降
    ss << "DO11:" << (do_status.gear_cylinder_extend ? "1" : "0") << ","; // 齿轮对接气缸伸出
    ss << "DO12:" << (do_status.belt_forward ? "1" : "0") << ",";       // 皮带正转启动
    ss << "DO13:" << (do_status.belt_backward ? "1" : "0");              // 皮带反转启动
    
    pthread_mutex_unlock(&io_mutex_);
    
    msg.data = ss.str();
    io_status_pub_->publish(msg);
}

// IO监控线程函数
void* io_monitor_thread(void* arg) {
    EthercatNode* node = static_cast<EthercatNode*>(arg);
    time_t last_display = time(NULL);
    // 静态标志位，确保禁用警告只报告一次
    static bool di_disabled_warned = false;
    static bool do_disabled_warned = false;

    RCLCPP_INFO(node->get_logger(), "IO监控线程开始运行");
    
    while (node->is_io_running() && !g_should_exit.load()) {
        // 读取DI信号（如果启用）
        DI_Interface di;
        di = read_all_di_signals();
        
        // 获取当前DO状态
        // DO_Interface do_control = get_current_do_state();
        
        // 每1秒显示一次状态，避免刷屏
        if (should_execute_sequence(&last_display, 1)) {
#if ENABLE_DI_MODULE
            // print_di_status(di);
#else
            if (!di_disabled_warned) {
                node->print_warning("DI模块已禁用");
                di_disabled_warned = true;
            }
#endif
            
#if ENABLE_DO_MODULE
            // print_do_status(do_control);
#else
            if (!do_disabled_warned) {
                node->print_warning("DO模块已禁用");
                do_disabled_warned = true;
            }
#endif
            // printf("\n----------------------------------------\n");
        }
        
        // 处理IO信号
        node->handle_io_signals(di);
        
        usleep(100000); // 100ms刷新周期
    }
    
    RCLCPP_INFO(node->get_logger(), "IO监控线程退出");
    return nullptr;
}

void EthercatNode::initialize_layer_processor() {
    // 创建层指令处理器
    layer_processor_ = std::make_unique<LayerCommandProcessor>(this);
    // 创建层移动完成发布器
    auto layer_completion_pub = this->create_publisher<std_msgs::msg::Bool>(
        "/layer_motion_completed", rclcpp::QoS(10).reliable());
    
    // 设置发布器给层指令处理器
    layer_processor_->set_layer_completion_publisher(layer_completion_pub);    

    // 查找axis5的索引 - 需要正确找到axis5的位置
    size_t axis5_index = 0;
    bool axis5_found = false;
    
    for (size_t i = 0; i < servo_axes_.size(); ++i) {
        if (servo_axes_[i]->get_name() == "axis5") {
            axis5_index = i;
            axis5_found = true;
            RCLCPP_INFO(this->get_logger(), "找到axis5，索引位置: %zu", axis5_index);
            break;
        }
    }
    
    if (!axis5_found) {
        print_error("未找到axis5，使用默认索引0");
        axis5_index = 0; // 如果找不到，可能需要调整这个默认值
    }
    // // 查找axis5的索引
    // size_t axis5_index = 0;
    // for (size_t i = 0; i < servo_axes_.size(); ++i) {
    //     if (servo_axes_[i]->get_name() == "axis5") {
    //         axis5_index = i;
    //         RCLCPP_INFO(this->get_logger(), "找到axis5，索引位置: %zu", axis5_index);
    //         break;
    //     }
    // }
    
    // 初始化处理器
    layer_processor_->initialize(axis5_index);
    
    // 创建层指令订阅器
    layer_command_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/layer_command", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::Int8::SharedPtr msg) {
            handle_layer_command(msg);
        });
    
    RCLCPP_INFO(this->get_logger(), "层指令处理器初始化完成");
}

void EthercatNode::handle_layer_command(const std_msgs::msg::Int8::SharedPtr msg) {
    if (node_shutting_down_.load() || !rclcpp::ok()) {
        return;
    }
    
    int8_t layer = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到层指令: 第%d层", layer);
    
    if (layer_processor_) {
        layer_processor_->process_layer_command(layer);
    } else {
        print_error("层指令处理器未初始化");
    }

    // 调试信息（todo删）：打印当前所有轴的状态
    RCLCPP_INFO(this->get_logger(), "当前轴数量: %zu", servo_axes_.size());
    for (size_t i = 0; i < servo_axes_.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "轴[%zu]: %s", i, servo_axes_[i]->get_name().c_str());
    }
}

void EthercatNode::check_layer_motion_completion() {
    if (!layer_processor_) {
        return;
    }
    
    // 查找axis5轴
    std::shared_ptr<ServoAxisBase> axis5;
    for (auto& axis : servo_axes_) {
        if (axis->get_name() == "axis5") {
            axis5 = axis;
            break;
        }
    }
    
    if (axis5) {
        layer_processor_->check_motion_completion(axis5);
    }
}

void EthercatNode::monitor_di_changes(const DI_Interface& current_di) {
    static DI_Interface previous_di = {0};
    
    // 检查每个DI信号的变化
    std::vector<std::pair<std::string, bool>> changes;
    
    if (current_di.buffer_in_position != previous_di.buffer_in_position) {
        changes.push_back({"DI12", current_di.buffer_in_position});
    }
    if (current_di.buffer_out_position != previous_di.buffer_out_position) {
        changes.push_back({"DI13", current_di.buffer_out_position});
    }
    if (current_di.conveyor_in_position != previous_di.conveyor_in_position) {
        changes.push_back({"DI14", current_di.conveyor_in_position});
    }
    if (current_di.conveyor_out_position != previous_di.conveyor_out_position) {
        changes.push_back({"DI15", current_di.conveyor_out_position});
    }
    
    // 打印变化
    for (const auto& change : changes) {
        RCLCPP_INFO(this->get_logger(), "%s 状态变化: %s", 
                   change.first.c_str(), 
                   change.second ? "HIGH" : "LOW");
    }
    
    // 更新前一次状态
    previous_di = current_di;
}

void EthercatNode::handle_do_control(const std_msgs::msg::String::SharedPtr msg) {
    if (node_shutting_down_.load() || !rclcpp::ok()) {
        return;
    }
    
    std::string command = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到DO控制命令: %s", command.c_str());
    
    // 解析命令
    DOControlCommand do_cmd;
    if (!parse_do_control_command(command, do_cmd)) {
        print_error("DO控制命令解析失败: " + command);
        return;
    }
    
    // 执行DO控制
    int do_address = std::stoi(do_cmd.do_address);
    int result = write_single_do_signal(do_address, do_cmd.state);
    
    if (result == 1) {
        RCLCPP_INFO(this->get_logger(), "DO控制成功: %s -> %s", 
                   do_cmd.do_address.c_str(), 
                   do_cmd.state ? "true" : "false");
        
        // 发布状态更新
        publish_io_status();
    } else {
        print_error("DO控制失败: " + command + ", 错误码: " + std::to_string(result));
    }
}

// 添加点动速度设置处理函数
void EthercatNode::handle_jog_speed_command(const std_msgs::msg::String::SharedPtr msg) {
    if (node_shutting_down_.load() || !rclcpp::ok()) {
        return;
    }
    
    std::string command = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到点动速度设置命令: %s", command.c_str());
    
    std::string axis_name;
    double speed;
    
    if (!parse_jog_speed_command(command, axis_name, speed)) {
        print_error("点动速度命令解析失败: " + command);
        return;
    }
    
    // 查找对应的轴并设置速度
    bool axis_found = false;
    for (auto& axis : servo_axes_) {
        if (axis->get_name() == axis_name) {
            if (axis->set_jog_speed(speed)) {
                RCLCPP_INFO(this->get_logger(), "成功设置轴 %s 的点动速度为: %.1f mm/s", 
                           axis_name.c_str(), speed);
                
                // 发布系统状态
                auto status_msg = std_msgs::msg::String();
                status_msg.data = "轴 " + axis_name + " 点动速度设置为: " + std::to_string(speed) + " mm/s";
                system_status_pub_->publish(status_msg);
            } else {
                print_error("设置轴 " + axis_name + " 的点动速度失败");
            }
            axis_found = true;
            break;
        }
    }
    
    if (!axis_found) {
        print_error("未找到轴: " + axis_name);
    }
}

// 添加点动速度解析函数
bool EthercatNode::parse_jog_speed_command(const std::string& command, std::string& axis_name, double& speed) {
    // 格式: "axis_name:speed" 例如: "axis1_1:30.5"
    size_t colon_pos = command.find(':');
    if (colon_pos == std::string::npos) {
        print_error("无效命令格式，应为 '轴名:速度'");
        return false;
    }
    
    axis_name = command.substr(0, colon_pos);
    std::string speed_str = command.substr(colon_pos + 1);
    
    // 去除空格
    axis_name.erase(0, axis_name.find_first_not_of(" \t"));
    axis_name.erase(axis_name.find_last_not_of(" \t") + 1);
    speed_str.erase(0, speed_str.find_first_not_of(" \t"));
    speed_str.erase(speed_str.find_last_not_of(" \t") + 1);
    
    if (axis_name.empty() || speed_str.empty()) {
        print_error("轴名或速度值为空");
        return false;
    }
    
    try {
        speed = std::stod(speed_str);
        return true;
    } catch (const std::exception& e) {
        print_error("速度值转换失败: " + speed_str + ", 错误: " + e.what());
        return false;
    }
}

void EthercatNode::handle_warehouse_start(const std_msgs::msg::UInt8::SharedPtr msg) {
    if (node_shutting_down_.load() || !rclcpp::ok()) {
        return;
    }
    
    uint8_t target_layer = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到入库启动命令，目标层: %d", target_layer);
    
    // 这里可以添加直接处理逻辑，或者通过Python节点处理
}

void EthercatNode::handle_warehouse_stop(const std_msgs::msg::Empty::SharedPtr msg) {
    if (node_shutting_down_.load() || !rclcpp::ok()) {
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "收到入库停止命令");
}

void EthercatNode::handle_outbound_start(const std_msgs::msg::UInt8::SharedPtr msg) {
    if (node_shutting_down_.load() || !rclcpp::ok()) {
        return;
    }
    
    uint8_t source_layer = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到出库启动命令，源层: %d", source_layer);
}

void EthercatNode::handle_outbound_stop(const std_msgs::msg::Empty::SharedPtr msg) {
    if (node_shutting_down_.load() || !rclcpp::ok()) {
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "收到出库停止命令");
}

/* ---------------------------------------板宽度调整------------------------------------------------ */
void EthercatNode::initialize_board_width_parameters() {
    // axis4 参数初始化 (保持不变)
    screw_lead_ = 10.0;           // 丝杠导程10mm
    // gear_ratio_ = 9.0;            // 减速比9.0
    pulses_per_rev_ = 10000;      // 每转脉冲数10000
    min_board_width_ = 8.0;      // 最小板宽10cm
    max_board_width_ = 48.0;      // 最大板宽50cm
    board_width_resolution_ = 0.01; // 板宽分辨率0.01cm
    current_board_width_ = 15.0;   // 默认板宽10cm
    target_board_width_ = 15.0;
    board_width_moving_ = false;
    board_width_updated_ = false;
    
    // +++ 新增：axis3 板宽参数初始化 +++
    axis3_screw_lead_ = 5.0;         // 示例：axis3丝杠导程可能不同
    axis3_gear_ratio_ = 1.0;         // 示例：减速比
    axis3_min_width_ = 8.0;          // axis3的最小板宽范围
    axis3_max_width_ = 48.0;
    axis3_current_width_ = 15.0;     // 默认板宽
    axis3_target_width_ = 15.0;
    axis3_width_moving_ = false;
    axis3_width_updated_ = false;
    
    // 板宽校正标志初始化（启动后需要读取实际位置校正）
    board_width_calibrated_ = false;
    
    RCLCPP_INFO(this->get_logger(),
               "axis3板宽控制参数: 范围%.2f-%.2fcm, 导程%.2fmm, 减速比%.2f",
               axis3_min_width_, axis3_max_width_, axis3_screw_lead_, axis3_gear_ratio_);
    RCLCPP_INFO(this->get_logger(), 
               "axis4板宽控制参数: 范围%.2f-%.2fcm, 导程%.2fmm, 减速比%.2f, 分辨率%.2fcm",
               min_board_width_, max_board_width_, screw_lead_, gear_ratio_, board_width_resolution_);
}

// 新增：axis3 板宽命令处理函数
void EthercatNode::handle_axis3_width_command(const std_msgs::msg::Float64::SharedPtr msg) {
    if (node_shutting_down_.load() || !rclcpp::ok()) {
        return;
    }
    
    double target_width = msg->data;
    RCLCPP_INFO(this->get_logger(), "[Axis3] 收到板宽设定命令: %.2fcm", target_width);
    
    // 验证板宽范围 (使用axis3的专用范围)
    if (target_width < axis3_min_width_ || target_width > axis3_max_width_) {
        std::stringstream err_ss;
        err_ss << "[Axis3] 无效板宽: " << std::fixed << std::setprecision(2) << target_width
               << "cm, 有效范围: " << axis3_min_width_ << "-" << axis3_max_width_ << "cm";
        print_error(err_ss.str());
        return;
    }
    
    // 检查是否与当前板宽相同
    if (fabs(target_width - axis3_current_width_) < board_width_resolution_) {
        RCLCPP_INFO(this->get_logger(), "[Axis3] 板宽已为目标值: %.2fcm", target_width);
        publish_axis3_width_status(axis3_current_width_, target_width, false, "已到达目标板宽");
        return;
    }
    
    // 设置目标并执行调整
    axis3_target_width_ = target_width;
    axis3_width_moving_ = true;
    
    RCLCPP_INFO(this->get_logger(), 
               "[Axis3] 开始调整板宽: %.2fcm -> %.2fcm", axis3_current_width_, axis3_target_width_);

    // 立即执行axis3的板宽调整
    execute_axis3_width_adjustment();
    
    publish_axis3_width_status(axis3_current_width_, axis3_target_width_, true, "开始调整板宽");
}
// 新增：计算axis3从板宽变化到需要的位移（毫米）
double EthercatNode::calculate_axis3_displacement_from_width(double board_width_cm) {
    // 计算原理同axis4，但使用axis3的机械参数
    double width_change_cm = board_width_cm - axis3_current_width_;
    double width_change_mm = width_change_cm * 10.0;  // cm转mm
    // 位移 = 板宽变化量 × 机械传动比
    // 注意：此处系数1.0为示例，需根据axis3实际机械结构调整
    double displacement_mm = width_change_mm * 1.0;
    
    RCLCPP_DEBUG(this->get_logger(),
                "[Axis3] 板宽计算: %.2fcm->%.2fcm, 变化%.2fmm, 需要位移%.3fmm",
                axis3_current_width_, board_width_cm, width_change_mm, displacement_mm);
    
    return displacement_mm;
}

// 新增：执行axis3的板宽调整
void EthercatNode::execute_axis3_width_adjustment() {
    if (!axis3_width_moving_) {
        return;
    }
    
    // 查找axis3的索引
    int axis3_index = find_axis3_index();
    if (axis3_index == -1) {
        print_error("未找到axis3，无法执行板宽调整");
        axis3_width_moving_ = false;
        return;
    }
    
    // 计算需要的位移（毫米）- 使用axis3的专用计算函数
    double displacement_mm = calculate_axis3_displacement_from_width(axis3_target_width_);
    
    RCLCPP_INFO(this->get_logger(),
               "[Axis3] 板宽调整: 目标%.2fcm, 需要位移%.3fmm",
               axis3_target_width_, displacement_mm);
    
    // 使用现有的位移命令接口控制电机
    // 假设零点相同或不同，这里使用绝对位移。零点偏移需根据实际情况确定。
    double absolute_displacement_mm = (15.0 - axis3_target_width_) * 10.0; // 基于15.0板宽计算绝对位移，运动方向相反
    handle_axis_command(axis3_index, absolute_displacement_mm);
    
    // 发布状态
    publish_axis3_width_status(axis3_current_width_, axis3_target_width_, true, "板宽调整中");
}

// 新增：查找axis3索引的辅助函数
int EthercatNode::find_axis3_index() {
    for (size_t i = 0; i < servo_axes_.size(); ++i) {
        if (servo_axes_[i]->get_name() == "axis3") {
            return static_cast<int>(i);
        }
    }
    return -1;  // 未找到
}
// 新增：发布axis3板宽状态
void EthercatNode::publish_axis3_width_status(double current_width, double target_width, 
                                            bool moving, const std::string& status) {
    if (node_shutting_down_.load() || !rclcpp::ok()) {
        return;
    }
    
    auto msg = std_msgs::msg::String();
    std::stringstream ss;
    
    ss << "current:" << std::fixed << std::setprecision(2) << current_width
       << ",target:" << std::fixed << std::setprecision(2) << target_width
       << ",moving:" << (moving ? "true" : "false")
       << ",status:" << status;
    
    msg.data = ss.str();
    axis3_width_status_pub_->publish(msg); // 需要创建对应的发布器，见下一步
    
    // 减少日志频率
    static int log_counter = 0;
    if (log_counter++ % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), "[Axis3] 板宽状态: %s", msg.data.c_str());
        log_counter = 0;
    }
}

void EthercatNode::handle_board_width_command(const std_msgs::msg::Float64::SharedPtr msg) {
    if (node_shutting_down_.load() || !rclcpp::ok()) {
        return;
    }
    
    double target_width = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到板宽设定命令: %.2fcm", target_width);
    
    // 验证板宽范围
    if (!validate_board_width(target_width)) {
        std::stringstream err_ss;
        err_ss << "无效板宽: " << std::fixed << std::setprecision(2) << target_width
               << "cm, 有效范围: " << min_board_width_ << "-" << max_board_width_ << "cm";
        print_error(err_ss.str());
        return;
    }
    
    // 检查是否与当前板宽相同
    if (fabs(target_width - current_board_width_) < board_width_resolution_) {
        RCLCPP_INFO(this->get_logger(), "板宽已为目标值: %.2fcm", target_width);
        publish_board_width_status(current_board_width_, target_width, false, "已到达目标板宽");
        return;
    }
    
    // 设置目标板宽并立即执行调整
    target_board_width_ = target_width;
    board_width_moving_ = true;
    
    RCLCPP_INFO(this->get_logger(), 
               "开始调整板宽: %.2fcm -> %.2fcm", current_board_width_, target_board_width_);

    // 立即执行板宽调整
    execute_board_width_adjustment();
    
    publish_board_width_status(current_board_width_, target_board_width_, true, "开始调整板宽");
}

bool EthercatNode::validate_board_width(double width) {
    if (width < min_board_width_ || width > max_board_width_) {
        return false;
    }
    
    // 检查分辨率
    double remainder = fmod(width * 100, board_width_resolution_ * 100);
    if (fabs(remainder) > 0.001) {  // 浮点数精度容差
        std::stringstream warn_ss;
        warn_ss << "板宽" << std::fixed << std::setprecision(2) << width 
                << "cm超出分辨率" << board_width_resolution_ << "cm，将四舍五入";
        print_warning(warn_ss.str());
    }
    
    return true;
}

double EthercatNode::calculate_displacement_from_width(double board_width_cm) {
    // 计算原理：
    // 1. 板宽变化量 (cm) = 目标板宽 - 当前板宽
    // 2. 转换为毫米: Δwidth_mm = Δwidth_cm × 10
    // 3. 考虑机械传动: 位移 = Δwidth_mm / (丝杠导程 × 减速比)
    // 4. 转换为脉冲数: 脉冲 = 位移 × (脉冲数/转) / 丝杠导程
    // 计算板宽变化量（厘米转毫米）
    double width_change_cm = board_width_cm - current_board_width_;
    double width_change_mm = width_change_cm * 10.0;  // cm转mm
    
    // 计算需要的电机位移（毫米）
    // 位移 = 板宽变化量 × 机械传动比
    double displacement_mm = width_change_mm;  // 根据实际机械结构调整系数
    
    RCLCPP_DEBUG(this->get_logger(),
                "板宽计算: %.2fcm->%.2fcm, 变化%.2fmm, 需要位移%.3fmm",
                current_board_width_, board_width_cm, width_change_mm, displacement_mm);
    
    return displacement_mm;
}

// double EthercatNode::calculate_width_from_displacement(double displacement_mm) {
//     // 反向计算：从位移计算板宽
//     // 电机转数 = 位移 / 丝杠导程
//     // 输出转数 = 电机转数 / 减速比
//     // 板宽变化 = 输出转数 × 丝杠导程
    
//     double motor_revolutions = displacement_mm / screw_lead_;
//     double output_revolutions = motor_revolutions / gear_ratio_;
//     double width_change_mm = output_revolutions * screw_lead_;
//     double width_change_cm = width_change_mm / 10.0;
    
//     return current_board_width_ + width_change_cm;
// }

// +++ 新增：根据实际位置校正板宽（启动后执行一次）+++
void EthercatNode::calibrate_board_width_from_position() {
    // 检查轴是否已初始化
    if (servo_axes_.empty()) {
        return;
    }
    
    int axis3_index = find_axis3_index();
    int axis4_index = find_axis4_index();
    
    if (axis3_index == -1 || axis4_index == -1) {
        RCLCPP_WARN(this->get_logger(), "板宽校正：未找到axis3或axis4，跳过校正");
        board_width_calibrated_ = true;  // 标记为已校正，避免重复尝试
        return;
    }
    
    // 获取axis3和axis4的当前实际位置（脉冲）
    int32_t axis3_actual_pos = servo_axes_[axis3_index]->get_actual_position();
    int32_t axis4_actual_pos = servo_axes_[axis4_index]->get_actual_position();
    
    // 获取轴的减速比（从轴对象或成员变量）
    double axis3_gear_ratio = 1.8;  // axis3减速比
    double axis4_gear_ratio = 7.34; // axis4减速比
    const double PULSES_PER_REV = 10000.0; // 10000脉冲/转
    const double SCREW_LEAD = 10.0; // 丝杠导程10mm（假设两轴相同）
    
    // 脉冲转换为毫米位移：displacement(mm) = (pulses / 10000) / gear_ratio * screw_lead
    // 即：电机转数 = 脉冲数/10000，输出转数 = 电机转数/减速比，位移 = 输出转数 * 导程
    double axis3_displacement_mm = (axis3_actual_pos / PULSES_PER_REV) / axis3_gear_ratio * SCREW_LEAD;
    double axis4_displacement_mm = (axis4_actual_pos / PULSES_PER_REV) / axis4_gear_ratio * SCREW_LEAD;
    
    // 根据位移计算板宽（15cm为基准，对应脉冲/位移为0）
    // 机械关系：板宽变化1cm = 丝杠位移10mm（根据实际机械结构确认）
    // axis3: 位移(mm) = (15.0 - target_width) * 10.0  →  反向: width = 15.0 - displacement/10.0
    double calculated_axis3_width = 15.0 - axis3_displacement_mm / 10.0;
    
    // axis4: 位移(mm) = (target_width - 15.0) * 10.0  →  反向: width = 15.0 + displacement/10.0
    double calculated_axis4_width = 15.0 + axis4_displacement_mm / 10.0;
    
    // 限制在有效范围内
    calculated_axis3_width = std::max(axis3_min_width_, std::min(axis3_max_width_, calculated_axis3_width));
    calculated_axis4_width = std::max(min_board_width_, std::min(max_board_width_, calculated_axis4_width));
    
    // 更新板宽值（取两个轴的平均值或分别更新）
    axis3_current_width_ = calculated_axis3_width;
    current_board_width_ = calculated_axis4_width;
    
    // 同时更新目标值为当前值（避免启动后突然运动）
    axis3_target_width_ = calculated_axis3_width;
    target_board_width_ = calculated_axis4_width;
    
    board_width_calibrated_ = true;
    
    RCLCPP_INFO(this->get_logger(), 
                "板宽校正完成: axis3=%.2fcm (位置:%d脉冲, 位移:%.2fmm), axis4=%.2fcm (位置:%d脉冲, 位移:%.2fmm)",
                axis3_current_width_, axis3_actual_pos, axis3_displacement_mm,
                current_board_width_, axis4_actual_pos, axis4_displacement_mm);
}

int EthercatNode::find_axis4_index() {
    for (size_t i = 0; i < servo_axes_.size(); ++i) {
        if (servo_axes_[i]->get_name() == "axis4") {
            return static_cast<int>(i);
        }
    }
    return -1;  // 未找到
}

// 检查所有轴是否都在自动模式
bool EthercatNode::are_all_axes_in_auto_mode() {
    if (servo_axes_.empty()) {
        return false;  // 轴未初始化
    }
    
    for (const auto& axis : servo_axes_) {
        if (axis->get_current_state() != AxisState::AUTO_MODE) {
            return false;  // 有轴不在自动模式
        }
    }
    return true;  // 所有轴都在自动模式
}

void EthercatNode::execute_board_width_adjustment() {
    if (!board_width_moving_) {
        return;
    }
    
    // 查找axis4（板宽调整轴）
    int axis4_index = find_axis4_index();
    if (axis4_index == -1) {
        print_error("未找到axis4，无法执行板宽调整");
        board_width_moving_ = false;
        return;
    }
    
    // 计算需要的位移（毫米）
    double displacement_mm = calculate_displacement_from_width(target_board_width_);
    
    RCLCPP_INFO(this->get_logger(),
               "板宽调整: 目标%.2fcm, 需要位移%.3fmm",
               target_board_width_, displacement_mm);
    
    // 使用现有的位移命令接口控制电机
    // handle_axis_command(axis4_index, displacement_mm);
    double absolute_displacement_mm = (target_board_width_ - 15.0) * 10.0; // 15.0为板宽零点，无偏移
    handle_axis_command(axis4_index, absolute_displacement_mm);
    
    // 发布状态
    publish_board_width_status(current_board_width_, target_board_width_, true, "板宽调整中");
}

void EthercatNode::publish_board_width_status(double current_width, double target_width, 
                                            bool moving, const std::string& status) {
    if (node_shutting_down_.load() || !rclcpp::ok()) {
        return;
    }
    
    auto msg = std_msgs::msg::String();
    std::stringstream ss;
    
    ss << "current:" << std::fixed << std::setprecision(2) << current_width
       << ",target:" << std::fixed << std::setprecision(2) << target_width
       << ",moving:" << (moving ? "true" : "false")
       << ",status:" << status;
    
    msg.data = ss.str();
    board_width_status_pub_->publish(msg);
    
    // 减少日志频率，避免刷屏
    static int log_counter = 0;
    if (log_counter++ % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), "板宽状态: %s", msg.data.c_str());
        log_counter = 0;
    }
}

// void EthercatNode::print_warning(const std::string& message) {
    // bool has_fault = false;
    
    // // 检查所有轴故障
    // for (auto& axis : servo_axes_) {
    //     if (axis->has_fault()) {
    //         uint16_t fault_code = axis->get_fault_code();
    //         std::string axis_name = axis->get_name();
            
    //         if (has_fault) ss << ",";
    //         has_fault = true;
            
    //         ss << axis_name << ":0x" << std::hex << std::setw(4) << std::setfill('0') << fault_code;
            
    //         // 同时通过故障管理器记录
    //         fault_manager_->add_axis_fault(axis_name, fault_code, "驱动器故障");
    //     }
    // }
    
    // // 如果还有其他故障源，也在这里添加
    
    // std_msgs::msg::String msg;
    // if (has_fault) {
    //     std::string fault_str = ss.str();
    //     msg.data = fault_str;
    // } else {
    //     msg.data = "0";
    // }
    
    // fault_code_pub_->publish(msg); 110ms发布周期，减少日志频率避免刷屏

    // if (node_shutting_down_.load() || !rclcpp::ok()) {
    //     return;
    // }
    
    // auto msg = std_msgs::msg::String();
    // std::stringstream ss;
    
    // bool has_fault = false;
    
    // // 检查所有轴的故障状态
    // for (auto& axis : servo_axes_) {
    //     // 获取轴的错误代码
    //     uint16_t error_code = axis->get_error_code();
        
    //     // 检查是否处于故障状态
    //     if (axis->get_current_state() == AxisState::FAULT) {
    //         // 只有故障状态且错误码非0才发布
    //         if (error_code != 0) {
    //             ss << axis->get_name() << ":0x" << std::hex << error_code << ",";
    //             has_fault = true;
                
    //             // 记录故障日志（减少频率避免刷屏）
    //             static std::unordered_map<std::string, uint16_t> last_error_codes;
    //             uint16_t last_code = last_error_codes[axis->get_name()];
                
    //             if (error_code != last_code) {
    //                 RCLCPP_WARN(this->get_logger(), 
    //                            "检测到轴故障: %s, 错误码: 0x%04X", 
    //                            axis->get_name().c_str(), error_code);
    //                 last_error_codes[axis->get_name()] = error_code;
    //             }
    //         }
    //     }
    // }
    
    // if (has_fault) {
    //     std::string fault_str = ss.str();
    //     // 移除最后一个逗号
    //     if (!fault_str.empty() && fault_str.back() == ',') {
    //         fault_str.pop_back();
    //     }
    //     msg.data = fault_str;
    // } else {
    //     // 无故障时发布"0"
    //     msg.data = "0";
    // }
    
    // fault_code_pub_->publish(msg);
    
    // // 减少日志频率，避免刷屏
    // static int log_counter = 0;
    // if (log_counter++ % 50 == 0) {  // 每5秒记录一次（假设100ms发布周期）
    //     if (has_fault) {
    //         RCLCPP_DEBUG(this->get_logger(), "发布故障状态: %s", msg.data.c_str());
    //     } else {
    //         RCLCPP_DEBUG(this->get_logger(), "系统正常，无故障");
    //     }
    //     log_counter = 0;
    // }
// }

void EthercatNode::print_warning(const std::string& message) {
    RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
    
    if (fault_manager_) {
        fault_manager_->add_system_warning(message);
    }
}

void EthercatNode::print_error(const std::string& message) {
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    
    if (fault_manager_) {
        fault_manager_->add_system_error(message);
    }
}