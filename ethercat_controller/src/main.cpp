#include "ethercat_node.hpp"
#include <pthread.h>
#include <sched.h>
#include <modbus/modbus.h>
#include "globals.h"
#include <sys/mman.h> 
#include <string.h>  

// 添加缺失的cycletime定义
const struct timespec cycletime = {0, PERIOD_NS};

// 时间计算函数
struct timespec timespec_add(struct timespec time1, struct timespec time2) {
    struct timespec result;
    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    } else {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }
    return result;
}

#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

// Modbus线程函数
void* modbus_read_thread(void *arg) {
    mb_ctx = modbus_new_tcp("192.168.1.12", 502);
    if (mb_ctx == nullptr) {
        fprintf(stderr, "无法创建Modbus上下文\n");
        return nullptr;
    }

    modbus_set_slave(mb_ctx, 1);

    if (modbus_connect(mb_ctx) == -1) {
        fprintf(stderr, "Modbus连接失败: %s\n", modbus_strerror(errno));
        modbus_free(mb_ctx);
        mb_ctx = nullptr;
        return nullptr;
    }

    printf("Modbus连接成功，开始监测DI13状态...\n");

    while (modbus_running) {      
        uint8_t di_value;
        int rc = modbus_read_input_bits(mb_ctx, 12, 1, &di_value);
        
        if (rc == -1) {
            fprintf(stderr, "读取DI13失败: %s\n", modbus_strerror(errno));
            usleep(100000);
            continue;
        }
        
        di13_state.store(di_value ? 1 : 0);
        
        static int last_state = -1;
        int current_state = di13_state.load();
        if (current_state != last_state) {
            printf("DI13状态: %s\n", current_state ? "ON" : "OFF");
            last_state = current_state;
        }
        
        usleep(50000);
    }
    
    modbus_close(mb_ctx);
    modbus_free(mb_ctx);
    mb_ctx = nullptr;
    return nullptr;
}

int init_modbus_monitor() {
    return pthread_create(&modbus_thread, nullptr, modbus_read_thread, nullptr);
}

// 主站状态检查
void check_master_state(void) {
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);
    
    if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
        
    master_state = ms;
}

// 信号处理
void signal_handler(int signum) {
    static std::atomic<bool> shutdown_initiated{false};
    
    // 防止重复调用
    if (shutdown_initiated.exchange(true)) {
        return;
    }
    printf("\n收到信号 %d, 开始安全关闭...\n", signum);
    running = 0;
    g_should_exit = true;
}

// 安全关闭
void safe_shutdown() {
    static std::atomic<bool> shutdown_in_progress{false};
    
    if (shutdown_in_progress.exchange(true)) {
        printf("关闭流程已在执行中...\n");
        return;
    }
    
    printf("\n开始安全关闭流程...\n");
    // 1. 先关闭ROS2执行器（最重要的一步）
    if (rclcpp::ok()) {
        // 强制中断ROS2执行器
        rclcpp::shutdown();
        
        // 等待ROS2线程完全退出
        usleep(200000); // 200ms等待
    } else {
    }
    
    // 1. 先设置退出标志
    running = 0;
    g_should_exit.store(true);
    
    printf("[DEBUG] 步骤2: 等待100ms让实时线程检测退出标志\n");
    usleep(100000); // 100ms
    
    // 2. 处理实时线程
    if (thread) {
        printf("[DEBUG] 步骤3: 开始处理实时线程\n");
        
        // 先尝试温和的等待
        int wait_count = 0;
        const int max_wait = 30; // 3秒超时
        
        while (wait_count < max_wait) {
            int result = pthread_tryjoin_np(thread, NULL);
            if (result == 0) {
                printf("[DEBUG] 实时线程正常退出\n");
                break;
            } else if (result == EBUSY) {
                printf("[DEBUG] 实时线程仍在运行，等待计数: %d/%d\n", wait_count + 1, max_wait);
                usleep(100000); // 100ms
                wait_count++;
            } else {
                printf("[DEBUG] pthread_tryjoin_np错误: %d\n", result);
                break;
            }
        }
        
        if (wait_count >= max_wait) {
            printf("[DEBUG] 超时，强制取消实时线程\n");
            pthread_cancel(thread);
            pthread_join(thread, NULL);
        }
        
        thread = 0;
        printf("[DEBUG] 实时线程处理完成\n");
    } else {
        printf("[DEBUG] 实时线程句柄为空\n");
    }
    
    printf("[DEBUG] 步骤4: 处理Modbus线程\n");
    // 3. 停止Modbus线程
    modbus_running = 0;
    if (modbus_thread) {
        printf("等待Modbus线程退出...\n");
        pthread_join(modbus_thread, nullptr);
        modbus_thread = 0;
        printf("Modbus线程已退出\n");
    }

    // 4. 停止Modbus线程
    modbus_running = 0;
    if (modbus_thread) {
        pthread_join(modbus_thread, nullptr);
        modbus_thread = 0;
    }

    // 5. 禁用驱动器
    if (master && global_node && domain1_pd) {
        printf("禁用所有驱动器...\n");
        auto& axes = global_node->get_servo_axes();
        for (auto& axis : axes) {
            unsigned int offset = axis->get_control_word_offset();
            EC_WRITE_U16(domain1_pd + offset, 0x0006); // 禁用命令
        }
        
        // 发送最后一次命令 - 修复这里
        ecrt_domain_queue(domain1);  // 直接调用，不检查返回值
        // ecrt_master_send(master);
        usleep(10000);
    }
    
    printf("安全关闭完成...\n释放EtherCAT资源...\n");

    if (master) {
        ecrt_release_master(master);
        master = nullptr;
    }
    printf("解除内存锁定...\n");
    munlockall();
    
    // 最后清理全局节点
    if (global_node) {
        global_node.reset();
    }
    
    printf("安全关闭完成\n");
}

// 域状态检查
void check_domain1_state(void) {
    ec_domain_state_t ds;
    ecrt_domain_state(domain1, &ds);
    
    if (ds.working_counter != domain1_state.working_counter)
        printf("Domain1 WC: %u\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printf("Domain1 State: %u\n", ds.wc_state);
        
    domain1_state = ds;
}

// 实时任务线程
void* rt_task_wrapper(void* arg) {
    printf("实时线程启动 (优先级: %d)\n", sched_get_priority_max(SCHED_FIFO));
    
    // 设置CPU亲和性
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset)) {
        perror("设置CPU亲和性失败");
    }
    
    struct timespec wakeup_time, current_time;
    clock_gettime(CLOCK_TO_USE, &wakeup_time);
    
    while (running) {
        wakeup_time = timespec_add(wakeup_time, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeup_time, NULL);
        
        // 设置应用时间
        ecrt_master_application_time(master, TIMESPEC2NS(wakeup_time));
        
        // EtherCAT通信处理
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);
        
        // 状态检查
        // check_domain1_state();
        
        if (counter == 0) {
            counter = FREQUENCY;
            check_master_state();
        } else {
            counter--;
        }
        
        // 处理轴状态机
        if (global_node && domain1_pd) {
            global_node->handle_axes_state_machines(domain1_pd);
        }
        
        // 发布关节状态（每10ms）
        static int joint_state_counter = 0;
        if (joint_state_counter++ >= 10) {
            joint_state_counter = 0;
            if (global_node) {
                global_node->publish_joint_states();
            }
        }
        
        // 同步参考时钟
        if (sync_ref_counter == 0) {
            sync_ref_counter = 1;
            clock_gettime(CLOCK_TO_USE, &current_time);
            ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(current_time));
        } else {
            sync_ref_counter--;
        }
        
        // 同步从站时钟并发送数据
        ecrt_master_sync_slave_clocks(master);
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
    }
    
    printf("实时线程退出\n");
    return nullptr;
}

// 主函数
int main(int argc, char **argv) {
    // 初始化ROS2
    rclcpp::init(argc, argv);
    global_node = std::make_shared<EthercatNode>("ethercat_controller");
    
    // 设置信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // 锁定内存
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall失败");
        return 1;
    }
    
    // 请求EtherCAT主站
    master = ecrt_request_master(0);
    if (!master) {
        fprintf(stderr, "请求EtherCAT主站失败\n");
        return 1;
    }
    
    // 创建域
    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
        fprintf(stderr, "创建域失败\n");
        safe_shutdown();
        return 1;
    }
    
    // 初始化Modbus监控
    if (init_modbus_monitor() != 0) {
        fprintf(stderr, "创建Modbus监测线程失败\n");
    }
    
    // 初始化轴和PDO
    global_node->init_axes(master);
    global_node->register_pdo_entries(domain1);
    
    // 配置分布式时钟
    auto slave_configs = global_node->get_all_slave_configs();
    for (auto sc : slave_configs) {
        if (sc) {
            ecrt_slave_config_dc(sc, 0x0300, PERIOD_NS, 0, 0, 0);
        }
    }
    
    // 激活主站
    printf("激活EtherCAT主站...\n");
    if (ecrt_master_activate(master)) {
        fprintf(stderr, "主站激活失败\n");
        safe_shutdown();
        return 1;
    }
    
    // 获取域数据指针
    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        fprintf(stderr, "获取域数据失败\n");
        safe_shutdown();
        return 1;
    }
    
    // 启动IO监控
    printf("启动IO监控模块...\n");
    global_node->start_io_monitoring();
    // 在轴初始化后初始化业务逻辑和层处理器
    global_node->initialize_after_axes();
    
    // 设置实时线程属性
    pthread_attr_t attr;
    struct sched_param param;
    
    if (pthread_attr_init(&attr)) {
        perror("线程属性初始化失败");
        safe_shutdown();
        return 1;
    }
    
    if (pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN + 32768)) {
        perror("设置栈大小失败");
        safe_shutdown();
        return 1;
    }
    
    if (pthread_attr_setschedpolicy(&attr, SCHED_FIFO)) {
        perror("设置调度策略失败");
        safe_shutdown();
        return 1;
    }
    
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (pthread_attr_setschedparam(&attr, &param)) {
        perror("设置优先级失败");
        safe_shutdown();
        return 1;
    }
    
    if (pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED)) {
        perror("设置继承调度失败");
        safe_shutdown();
        return 1;
    }
    
    // 创建实时线程
    if (pthread_create(&thread, &attr, rt_task_wrapper, NULL)) {
        perror("创建实时线程失败");
        safe_shutdown();
        return 1;
    }
    
    pthread_setname_np(thread, "ethercat-rt");
    printf("实时线程创建成功 (优先级: %d)\n", param.sched_priority);
    
    // 运行ROS2执行器
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(global_node);
    
    printf("=== EtherCAT控制系统启动完成 ===\n");
    printf("系统状态:\n");
    printf("  - EtherCAT主站: 已激活\n");
    printf("  - 实时线程: 运行中\n");
    printf("  - IO监控: %s\n", global_node->is_io_running() ? "已启动" : "未启动");
    printf("  - 伺服轴数量: %zu\n", global_node->get_servo_axes().size());
    printf("  - DI模块: %s\n", is_di_module_enabled() ? "启用" : "禁用");
    printf("  - DO模块: %s\n", is_do_module_enabled() ? "启用" : "禁用");
    printf("按Ctrl+C退出程序\n\n");
    
    // 使用非阻塞的spin方式
    while (rclcpp::ok() && !g_should_exit) {
        executor.spin_some(std::chrono::milliseconds(100));
        
        // 定期检查退出标志
        if (g_should_exit) {
            break;
        }
    }
    printf("ROS2执行器已退出，开始清理...\n");
    safe_shutdown();
    
    return 0;
}