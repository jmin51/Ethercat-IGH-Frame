#include "ethercat_node.hpp"
#include <thread>
#include <chrono>
#include <signal.h>
#include <sys/mman.h>
#include <modbus/modbus.h>
#include <string>
#include <sstream>
#include <std_msgs/msg/empty.hpp>  // 添加这行

// 全局变量定义
std::shared_ptr<EthercatNode> global_node = nullptr;
ec_master_t *master = nullptr;
ec_domain_t *domain1 = nullptr;
uint8_t *domain1_pd = nullptr;
std::atomic<bool> g_should_exit(false);
std::atomic<bool> node_shutting_down_{false};

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
std::atomic<bool> system_initialized{false};

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
    // 初始化IO互斥锁
    pthread_mutex_init(&io_mutex_, nullptr);
    
    RCLCPP_INFO(this->get_logger(), "EtherCAT节点初始化完成");
}

// 新增：在轴初始化后调用的方法
void EthercatNode::initialize_after_axes() {
    RCLCPP_INFO(this->get_logger(), "开始初始化业务逻辑模块（延迟初始化）");
    
    // 检查轴是否已初始化
    if (servo_axes_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "伺服轴未初始化，无法初始化业务逻辑模块");
        return;
    }
    
    // 初始化业务逻辑处理器
    initialize_business_logic();
    initialize_layer_processor();
    RCLCPP_INFO(this->get_logger(), "业务逻辑模块初始化完成");
}

void EthercatNode::init_axes(ec_master_t* master) {
    RCLCPP_INFO(this->get_logger(), "开始初始化伺服轴");
    
    auto& dual_axis_manager = LeisaiDualAxisManager::getInstance();
    
    // 添加从站1：第二个雷赛双轴驱动器
    auto axis1_1 = ServoAxisFactory::create_servo_axis(
        DriveBrand::LEISAI, "axis1_1", 0, AxisType::AXIS1, LEISAI_PRODUCT_CODE_1);  // 从站位置=1
    auto axis1_2 = ServoAxisFactory::create_servo_axis(
        DriveBrand::LEISAI, "axis1_2", 0, AxisType::AXIS2, LEISAI_PRODUCT_CODE_1);  // 从站位置=1（同一个从站）

    // 注册从站1的双轴关系
    dual_axis_manager.register_dual_axis(0,  // 从站位置1
        std::dynamic_pointer_cast<LeisaiServoAxis>(axis1_1),
        std::dynamic_pointer_cast<LeisaiServoAxis>(axis1_2));

    // 启用从站3的双轴同步
    auto leisai_axis1_1 = std::dynamic_pointer_cast<LeisaiServoAxis>(axis1_1);
    auto leisai_axis1_2 = std::dynamic_pointer_cast<LeisaiServoAxis>(axis1_2);
    leisai_axis1_1->set_sync_motor_enabled(true);
    leisai_axis1_2->set_sync_motor_enabled(true);

    servo_axes_.push_back(std::move(axis1_1));
    servo_axes_.push_back(std::move(axis1_2));
    // 创建雷赛双轴驱动器 - 从站2
    auto axis2_1 = ServoAxisFactory::create_servo_axis(
        DriveBrand::LEISAI, "axis2_1", 1, AxisType::AXIS1, LEISAI_PRODUCT_CODE_1);
    auto axis2_2 = ServoAxisFactory::create_servo_axis(
        DriveBrand::LEISAI, "axis2_2", 1, AxisType::AXIS2, LEISAI_PRODUCT_CODE_1);
    
    // 注册双轴关系
    dual_axis_manager.register_dual_axis(1, 
        std::dynamic_pointer_cast<LeisaiServoAxis>(axis2_1),
        std::dynamic_pointer_cast<LeisaiServoAxis>(axis2_2));
    
    // 启用双轴同步
    auto leisai_axis2_1 = std::dynamic_pointer_cast<LeisaiServoAxis>(axis2_1);
    auto leisai_axis2_2 = std::dynamic_pointer_cast<LeisaiServoAxis>(axis2_2);
    leisai_axis2_1->set_sync_motor_enabled(true);
    leisai_axis2_2->set_sync_motor_enabled(true);

    servo_axes_.push_back(std::move(axis2_1));
    servo_axes_.push_back(std::move(axis2_2));
    
    // 可以继续添加其他轴
    servo_axes_.push_back(ServoAxisFactory::create_servo_axis(
        DriveBrand::LEISAI, "axis3", 2, AxisType::AXIS1, LEISAI_PRODUCT_CODE_2));
    servo_axes_.push_back(ServoAxisFactory::create_servo_axis(
        DriveBrand::HUICHUAN, "axis4", 3, AxisType::AXIS1, 0, 9.0));
    servo_axes_.push_back(ServoAxisFactory::create_servo_axis(
        DriveBrand::HUICHUAN, "axis5", 4, AxisType::AXIS1));
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
        // 手动模式：禁用业务逻辑自动模式
        if (business_processor_) {
            business_processor_->disable_auto_mode();
        }
        
        for (auto& axis : servo_axes_) {
            axis->start_manual_mode();
        }
        RCLCPP_INFO(this->get_logger(), "所有轴接收到手动模式启动命令");
        
    } else if (command == CMD_START_AUTO) {
        // 自动模式：启用业务逻辑自动模式
        if (business_processor_) {
            business_processor_->enable_auto_mode();
        }
        
        for (auto& axis : servo_axes_) {
            axis->start_auto_mode();
        }
        RCLCPP_INFO(this->get_logger(), "所有轴接收到自动模式启动命令");
        
    } else if (command == CMD_STOP) {
        // 停止模式：禁用业务逻辑自动模式并重置
        if (business_processor_) {
            business_processor_->disable_auto_mode();
        }
        
        for (auto& axis : servo_axes_) {
            axis->stop();
        }
        RCLCPP_INFO(this->get_logger(), "所有轴接收到停止命令");
        
    } else if (command == CMD_CLEAR_FAULT) {
        // 清除故障时也禁用自动模式
        if (business_processor_) {
            business_processor_->disable_auto_mode();
        }
        
        for (auto& axis : servo_axes_) {
            axis->clear_fault();
        }
        RCLCPP_INFO(this->get_logger(), "所有轴接收到清除故障命令");
        
    } else if (command == CMD_RESET) {
        // 重置时也禁用自动模式
        if (business_processor_) {
            business_processor_->disable_auto_mode();
        }
        
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
        RCLCPP_ERROR(this->get_logger(), "位移指令解析失败: %s", command.c_str());
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
                // 检查是否为雷赛双轴
                auto leisai_axis = std::dynamic_pointer_cast<LeisaiServoAxis>(servo_axes_[i]);
                if (leisai_axis && leisai_axis->is_sync_motor_enabled()) {
                    // 双轴同步控制：只需要处理其中一个轴，另一个会自动同步
                    if (leisai_axis->get_sync_axis()) {
                        int32_t target_pulses = leisai_axis->get_initial_position() + 
                                               leisai_axis->displacement_to_pulses(displacement);
                        
                        // 更新同步目标位置
                        leisai_axis->update_sync_target_position(target_pulses);
                        
                        RCLCPP_DEBUG(this->get_logger(), 
                                   "双轴同步控制: %s 和 %s 目标位移: %.3fmm, 脉冲: %d",
                                   leisai_axis->get_name().c_str(),
                                   leisai_axis->get_sync_axis()->get_name().c_str(),
                                   displacement, target_pulses);
                    }
                } else {
                    // 单轴控制
                    if (servo_axes_[i]->is_running()) {
                        handle_axis_command(i, displacement);
                    }
                }
                axis_found = true;
                break;
            }
        }
        
        if (!axis_found) {
            RCLCPP_ERROR(this->get_logger(), "未找到轴: %s", axis_name.c_str());
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
            RCLCPP_ERROR(this->get_logger(), "无效指令格式，缺少冒号: %s", part.c_str());
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
            RCLCPP_ERROR(this->get_logger(), "轴名或值为空: %s", part.c_str());
            return false;
        }
        
        try {
            double value = std::stod(value_str);
            axis_commands.push_back({axis_name, value});
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "数值转换失败: %s, 错误: %s", value_str.c_str(), e.what());
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
        RCLCPP_ERROR(this->get_logger(), "无效命令格式，应为 '轴名:命令'");
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
        RCLCPP_ERROR(this->get_logger(), "Modbus初始化失败");
        return;
    }
    
    io_running_.store(true);
    
    // 创建IO监控线程
    if (pthread_create(&io_thread_, nullptr, io_monitor_thread, this) != 0) {
        RCLCPP_ERROR(this->get_logger(), "创建IO监控线程失败");
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

void EthercatNode::initialize_business_logic() {
    /// 创建并配置业务逻辑处理器
    business_processor_ = std::make_unique<BusinessLogicProcessor>(this->get_logger());
    
    // 初始化处理器(传入轴数量用于验证)
    size_t num_axes = servo_axes_.size();
    business_processor_->initialize(num_axes);

    // 设置固定目标层为1
    business_processor_->set_target_layer(1);

    // 启用业务逻辑处理
    business_processor_->enable(); //disable
    
    RCLCPP_INFO(this->get_logger(), "业务逻辑处理器准备就绪，已配置 %zu 个轴的映射关系", num_axes);
}

void EthercatNode::handle_io_signals(DI_Interface di) {
    // 这里可以添加IO信号处理逻辑
    // 例如：根据DI信号状态控制伺服轴
    // RCLCPP_INFO(this->get_logger(), 
    //             "DI信号状态: DI12=%d, DI13=%d, DI14=%d, DI15=%d", 
    //             static_cast<int>(di.buffer_in_position), 
    //             static_cast<int>(di.buffer_out_position),
    //             static_cast<int>(di.conveyor_in_position), 
    //             static_cast<int>(di.conveyor_out_position));
    monitor_di_changes(di);
    pthread_mutex_lock(&io_mutex_);
    current_di_status_ = di;
    pthread_mutex_unlock(&io_mutex_);

    // 步骤1: 如果有业务逻辑处理器且已启用，并且处于自动模式，处理自动控制逻辑
    if (business_processor_ && business_processor_->is_enabled() && business_processor_->is_auto_mode_enabled()) {
        // 将DI信号传递给业务逻辑处理器进行分析
        business_processor_->process_io_signals(di);
        
        // 获取业务逻辑生成的轴控制命令
        auto commands = business_processor_->get_pending_commands();
        
        // RCLCPP_INFO(this->get_logger(), "生成 %zu 个业务逻辑命令", commands.size());
        
        for (const auto& cmd : commands) {
            RCLCPP_INFO(this->get_logger(), "执行命令: 类型=%d, 轴=%s, 值=%s", 
                       static_cast<int>(cmd.type), cmd.axis_name.c_str(), cmd.command_value.c_str());
            
            switch (cmd.type) {
                case CommandType::JOG:
                    // 直接处理点动命令
                    for (auto& axis : servo_axes_) {
                        if (axis->get_name() == cmd.axis_name) {
                            if (cmd.command_value == "forward") {
                                axis->jog_forward();
                            } else if (cmd.command_value == "reverse") {
                                axis->jog_reverse();
                            } else if (cmd.command_value == "stop") {
                                axis->jog_stop();
                            }
                            break;
                        }
                    }
                    break;
                    
                case CommandType::LAYER:
                    // 直接处理层指令
                    try {
                        uint8_t layer = std::stoi(cmd.command_value);
                        layer_processor_->process_layer_command(layer);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "层指令解析失败: %s", e.what());
                    }
                    break;
                    
                case CommandType::POSITION:
                    // 原有的位置控制逻辑
                    for (size_t i = 0; i < servo_axes_.size(); ++i) {
                        if (servo_axes_[i]->get_name() == cmd.axis_name) {
                            handle_axis_command(i, cmd.target_position);
                            break;
                        }
                    }
                    break;
                    
                case CommandType::STOP:
                    // 停止命令
                    for (size_t i = 0; i < servo_axes_.size(); ++i) {
                        if (servo_axes_[i]->get_name() == cmd.axis_name) {
                            servo_axes_[i]->jog_stop();
                            break;
                        }
                    }
                    break;
            }
        }
        
        // 清空已处理的命令
        business_processor_->clear_pending_commands();
    }
    // 关键：每次处理IO信号时，都检查层运动是否完成
    if (layer_processor_) {
        // 查找axis5轴
        std::shared_ptr<ServoAxisBase> axis5 = nullptr;
        for (auto& axis : servo_axes_) {
            if (axis->get_name() == "axis5") {
                axis5 = axis;
                break;
            }
        }
        
        if (axis5) {
            bool motion_completed = layer_processor_->check_motion_completion(axis5);
            if (motion_completed) {
                RCLCPP_DEBUG(this->get_logger(), "检测到axis5运动完成");
                
                // 更新业务逻辑的当前层状态
                if (business_processor_) {
                    business_processor_->set_current_layer(layer_processor_->get_current_layer());
                    
                    RCLCPP_INFO(this->get_logger(), 
                              "业务逻辑层状态更新: 当前层=%d, 目标层=%d",
                              business_processor_->get_current_layer(),
                              business_processor_->get_target_layer());
                }
            }
        }
    }
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
    ss << "DI状态: 启动按钮=" << (current_di_status_.start_button ? "按下" : "释放")
       << ", 急停=" << (current_di_status_.emergency_stop ? "激活" : "正常");
    
    DO_Interface do_status = get_current_do_state();
    ss << " | DO状态: 启动灯=" << (do_status.start_button_light ? "亮" : "灭")
       << ", 绿灯=" << (do_status.green_light ? "亮" : "灭");
    pthread_mutex_unlock(&io_mutex_);
    
    msg.data = ss.str();
    io_status_pub_->publish(msg);
}

// IO监控线程函数
void* io_monitor_thread(void* arg) {
    EthercatNode* node = static_cast<EthercatNode*>(arg);
    time_t last_display = time(NULL);
    
    RCLCPP_INFO(node->get_logger(), "IO监控线程开始运行");
    
    while (node->is_io_running() && !g_should_exit.load()) {
        // 读取DI信号（如果启用）
        DI_Interface di;
#if ENABLE_DI_MODULE
        di = read_all_di_signals();
#else
        // DI禁用时使用空结构体
        di = (DI_Interface){0};
#endif
        
        // 获取当前DO状态
        // DO_Interface do_control = get_current_do_state();
        
        // 每1秒显示一次状态，避免刷屏
        if (should_execute_sequence(&last_display, 1)) {
#if ENABLE_DI_MODULE
            // print_di_status(di);
#else
            // printf("\n--- DI模块已禁用 ---\n");
#endif
            
#if ENABLE_DO_MODULE
            // print_do_status(do_control);
#else
            // printf("\n--- DO模块已禁用 ---\n");
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
        RCLCPP_ERROR(this->get_logger(), "未找到axis5，使用默认索引0");
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
    
    // 配置自定义层高（可选）
    std::map<uint8_t, double> layer_heights;
    for (uint8_t layer = 1; layer <= 10; ++layer) {
        layer_heights[layer] = (layer - 1) * 25.0;  // 每层25mm
    }
    layer_processor_->set_layer_heights(layer_heights);
    
    // 创建层指令订阅器
    layer_command_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/layer_command", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::UInt8::SharedPtr msg) {
            handle_layer_command(msg);
        });
    
    RCLCPP_INFO(this->get_logger(), "层指令处理器初始化完成");
}

void EthercatNode::handle_layer_command(const std_msgs::msg::UInt8::SharedPtr msg) {
    if (node_shutting_down_.load() || !rclcpp::ok()) {
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "收到层指令: 第%d层", msg->data);
    layer_processor_->process_layer_command(msg->data);
    // 调试信息：打印当前所有轴的状态
    RCLCPP_INFO(this->get_logger(), "当前轴数量: %zu", servo_axes_.size());
    for (size_t i = 0; i < servo_axes_.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "轴[%zu]: %s", i, servo_axes_[i]->get_name().c_str());
    }
    
    layer_processor_->process_layer_command(msg->data);
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
        RCLCPP_ERROR(this->get_logger(), "DO控制命令解析失败: %s", command.c_str());
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
        RCLCPP_ERROR(this->get_logger(), "DO控制失败: %s, 错误码: %d", 
                     command.c_str(), result);
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
        RCLCPP_ERROR(this->get_logger(), "点动速度命令解析失败: %s", command.c_str());
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
                RCLCPP_ERROR(this->get_logger(), "设置轴 %s 的点动速度失败", axis_name.c_str());
            }
            axis_found = true;
            break;
        }
    }
    
    if (!axis_found) {
        RCLCPP_ERROR(this->get_logger(), "未找到轴: %s", axis_name.c_str());
    }
}

// 添加点动速度解析函数
bool EthercatNode::parse_jog_speed_command(const std::string& command, std::string& axis_name, double& speed) {
    // 格式: "axis_name:speed" 例如: "axis1_1:30.5"
    size_t colon_pos = command.find(':');
    if (colon_pos == std::string::npos) {
        RCLCPP_ERROR(this->get_logger(), "无效命令格式，应为 '轴名:速度'");
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
        RCLCPP_ERROR(this->get_logger(), "轴名或速度值为空");
        return false;
    }
    
    try {
        speed = std::stod(speed_str);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "速度值转换失败: %s, 错误: %s", speed_str.c_str(), e.what());
        return false;
    }
}