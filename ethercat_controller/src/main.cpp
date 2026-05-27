#include "ethercat_node.hpp"
#include "io_interface.hpp"
#include "lights_controller.hpp"  // 灯光控制器
#include <pthread.h>
#include <sched.h>
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
    safe_shutdown(false);  // false表示完全关闭模式
}

// 仅停止电机和皮带（短按暂停）
void pause_motors_only() {
    printf("\n[短按暂停] ========================================\n");
    printf("[短按暂停] 停止电机和皮带，保持EtherCAT运行...\n");
    
    // 0. 首先记录当前业务状态（在停止前获取）
    if (global_node) {
        printf("[短按暂停] 请求记录当前业务逻辑状态...\n");
        global_node->publish_pause_state_record_request();
        // 等待一小段时间让Python端处理并返回状态报告
        usleep(200000);  // 200ms
    }
    
    // 1. 复位关键DO信号（M810-M813）- 停止皮带和气缸
    printf("[短按暂停] 复位关键DO信号 M810-M813...\n");
    write_single_do_signal(810, false);  // 顶升气缸下降
    write_single_do_signal(811, false);  // 齿轮对接气缸伸出
    write_single_do_signal(812, false);  // 皮带正转启动
    write_single_do_signal(813, false);  // 皮带反转启动
    printf("[短按暂停] DO信号复位完成\n");
    
    // 2. 停止所有轴的运动
    if (global_node) {
        printf("[短按暂停] 停止所有伺服轴...\n");
        auto& axes = global_node->get_servo_axes();
        for (auto& axis : axes) {
            // 2.1 调用stop()完全停止轴（包括清除目标位置、点动请求等）
            axis->stop();
            // 2.2 清除目标到达标志（读取并清除一次性标志）
            axis->check_target_reached_flag();
        }
        printf("[短按暂停] 所有伺服轴已停止\n");
    }
    
    // 3. 设置短按暂停状态
    g_short_pause_active.store(true);
    g_short_pause_requested.store(false);
    g_system_running.store(false);  // 设置系统停止状态
    
    // 4. 重置自动模式初始化标志（确保下次恢复时重新等待轴就绪）
    g_auto_mode_initialized.store(false);
    if (global_node) {
        global_node->reset_auto_mode_init_published();
    }
    
    // 4. 打印记录的状态信息
    if (g_pause_state_record.has_recorded_state) {
        printf("[短按暂停] 业务状态已记录:\n");
        printf("[短按暂停]   - 入库流程: %s (状态=%d, 目标层=%d)\n",
               g_pause_state_record.warehouse_was_active ? "进行中" : "未激活",
               g_pause_state_record.warehouse_state_value,
               g_pause_state_record.warehouse_target_layer);
        printf("[短按暂停]   - 出库流程: %s (状态=%d, 源层=%d)\n",
               g_pause_state_record.outbound_was_active ? "进行中" : "未激活",
               g_pause_state_record.outbound_state_value,
               g_pause_state_record.outbound_source_layer);
    }
    
    printf("[短按暂停] 电机和皮带已停止，系统处于暂停状态，按启动按钮恢复\n");
    printf("[短按暂停] ========================================\n");
}

// 恢复系统运行（短按暂停后的启动）
void resume_from_short_pause() {
    printf("\n[恢复运行] ========================================\n");
    printf("[恢复运行] 从短按暂停状态恢复...\n");
    
    // 立即清除短按暂停状态，确保后续层指令能被正常接收
    g_short_pause_active.store(false);
    
    if (global_node) {
        auto& axes = global_node->get_servo_axes();
        printf("[恢复运行] 重置 %zu 个轴到未初始化状态，准备重新初始化...\n", axes.size());
        for (auto& axis : axes) {
            axis->reset_axis();  // 强制回到 UNINITIALIZED
        }
        
        // 不等待所有轴就绪，直接让IO控制逻辑处理
        // IO控制逻辑会在主循环中检测READY状态的轴并自动切换模式
        printf("[恢复运行] 轴已重置，由IO控制逻辑自动处理模式切换\n");
    }
#if CONTROL_SOURCE_IO
    // ============================================================
    // IO控制模式：通过手自动按钮状态决定进入手动还是自动模式
    // ============================================================
    DI_Interface di = read_all_di_signals();
    bool is_auto_mode = di.manual_auto_button;  // 手自动按钮状态
    
    printf("[恢复运行] IO控制模式 - 手自动按钮状态: %s\n", 
           is_auto_mode ? "自动" : "手动");
    
    // 设置恢复后模式切换标志，等待轴就绪后再执行模式切换
    // 避免轴从UNINITIALIZED复位后未就绪时无法切换模式的问题
    g_resume_auto_mode.store(is_auto_mode);
    g_resume_mode_switch_pending.store(true);
    printf("[恢复运行] 已设置模式切换标志(%s)，等待轴就绪后自动切换...\n", 
           is_auto_mode ? "自动" : "手动");
#else
    // ============================================================
    // 话题控制模式：不直接启动轴，等待外部话题命令
    // 只发送恢复请求，由外部控制逻辑决定是否启动
    // ============================================================
    printf("[恢复运行] 话题控制模式 - 等待外部话题命令启动...\n");
    // 不调用 start_auto_mode() 或 start_manual_mode()
    // 等待 handle_py_control_command() 接收话题命令后再启动
#endif
    
    // 2. 发送恢复请求，携带记录的状态信息
    if (global_node && g_pause_state_record.has_recorded_state) {
        printf("[恢复运行] 发送业务状态恢复请求...\n");
        global_node->publish_pause_state_resume_request();
        // 等待一小段时间让Python端处理
        usleep(100000);  // 100ms
    }
    
    // 注意：不在这里清除启动按钮状态，让主循环在恢复完成后清除
    // 避免在恢复流程执行期间按钮按下被清除
    
    printf("[恢复运行] 系统已恢复运行，业务逻辑将重走之前记录的状态\n");
    printf("[恢复运行] ========================================\n");
}

// 安全关闭
void safe_shutdown(bool is_pause = false) {
    static std::atomic<bool> shutdown_in_progress{false};
    
    if (shutdown_in_progress.exchange(true)) {
        printf("关闭流程已在执行中...\n");
        return;
    }
    
    // 清除短按暂停状态（如果是长按暂停，需要清除短按状态）
    g_short_pause_active.store(false);
    
    if (is_pause) {
        printf("\n开始安全暂停流程...\n");
        // 暂停时复位关键DO信号（M810-M813）
        printf("[暂停] 复位关键DO信号 M810-M813...\n");
        write_single_do_signal(810, false);  // 顶升气缸下降
        write_single_do_signal(811, false);  // 齿轮对接气缸伸出
        write_single_do_signal(812, false);  // 皮带正转启动
        write_single_do_signal(813, false);  // 皮带反转启动
        printf("[暂停] DO信号复位完成\n");
    } else {
        printf("\n开始安全关闭流程...\n");
        g_should_exit.store(true, std::memory_order_release);
        // 退出时复位所有 DO 信号（M800-M813）
        printf("[关闭] 复位所有 DO 信号 M800-M813...\n");
        for (int addr = 800; addr <= 813; addr++) {
            write_single_do_signal(addr, false);
        }
        //     g_should_exit.store(true);
    }
    
    // 1. 设置退出标志
    running = 0;
    
    if (!is_pause) {
        // 完全关闭时才停止ROS2执行器
        printf("[DEBUG] 步骤1: 停止ROS2执行器\n");
        if (rclcpp::ok()) {
            rclcpp::shutdown();
            usleep(200000); // 200ms等待ROS2线程退出
        }
    }
    
    printf("[DEBUG] 步骤2: 等待100ms让实时线程检测退出标志\n");
    usleep(100000); // 100ms
    
    // 2. 处理实时线程
    if (thread) {
        printf("[DEBUG] 步骤3: 处理实时线程\n");
        
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
    
    // 3. 暂停模式下不停止Modbus线程和IO监控（删除）
    
    // 4. 禁用驱动器
    if (master && global_node && domain1_pd) {
        printf("禁用所有驱动器...\n");
        auto& axes = global_node->get_servo_axes();
        for (auto& axis : axes) {
            unsigned int offset = axis->get_control_word_offset();
            EC_WRITE_U16(domain1_pd + offset, 0x0006); // 禁用命令
        }
        
        // 发送最后一次命令
        ecrt_domain_queue(domain1);
        usleep(10000);
    }
    
    // 5. 释放EtherCAT资源
    if (master) {
        if (is_pause) {
            printf("暂停模式：释放EtherCAT主站资源\n");
        } else {
            printf("释放EtherCAT资源...\n");
        }
        ecrt_master_deactivate(master);
        ecrt_release_master(master);
        master = nullptr;
    }
    
    // 6. 完全关闭时的额外清理
    if (!is_pause) {
        printf("解除内存锁定...\n");
        munlockall();
    } else {
        printf("暂停模式：保持内存锁定和全局节点\n");
    }
    
    if (is_pause) {
        printf("安全暂停完成\n");
    } else {
        // 清理全局节点
        if (global_node) {
            global_node.reset();
        }
        printf("安全关闭完成\n");
    }
    
    shutdown_in_progress.store(false);
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
    if (!master || !domain1 || !domain1_pd) {
        printf("EtherCAT资源未就绪，等待初始化...\n");
        return nullptr;
    }
    // 设置CPU亲和性
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset)) {
        perror("设置CPU亲和性失败");
    }
    
    struct timespec wakeup_time, current_time;
    clock_gettime(CLOCK_TO_USE, &wakeup_time);
    
    while (running && !g_should_exit.load()) {
        // 检查暂停按钮
        if (g_pause_button_pressed.load()) {
            printf("检测到暂停按钮，准备安全关闭...\n");
            break;
        }
        
        // 检查系统是否完全停止（长按暂停/关闭）
        // 短按暂停时保持EtherCAT通信运行
        if (!g_system_running.load() && !g_short_pause_active.load()) {
            usleep(50000); // 50ms等待
            continue;
        }
        
        wakeup_time = timespec_add(wakeup_time, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeup_time, NULL);
        
        // 设置应用时间
        ecrt_master_application_time(master, TIMESPEC2NS(wakeup_time));
        
        // EtherCAT通信处理（短按暂停时保持运行）
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
        
        // 处理轴状态机（始终保持运行以维持使能）
        if (global_node && domain1_pd) {
            global_node->handle_axes_state_machines(domain1_pd);
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
      
    // 启动IO监控
    printf("启动IO监控模块...\n");
    global_node->start_io_monitoring();
    // 在轴初始化后初始化业务逻辑和层处理器
    // global_node->initialize_after_axes();
    
    // 设置实时线程属性
    pthread_attr_t attr;
    struct sched_param param;
    
    if (pthread_attr_init(&attr)) {
        perror("线程属性初始化失败");
        safe_shutdown(false);
        return 1;
    }
    
    if (pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN + 32768)) {
        perror("设置栈大小失败");
        safe_shutdown(false);
        return 1;
    }
    
    if (pthread_attr_setschedpolicy(&attr, SCHED_FIFO)) {
        perror("设置调度策略失败");
        safe_shutdown(false);
        return 1;
    }
    
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (pthread_attr_setschedparam(&attr, &param)) {
        perror("设置优先级失败");
        safe_shutdown(false);
        return 1;
    }
    
    if (pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED)) {
        perror("设置继承调度失败");
        safe_shutdown(false);
        return 1;
    }
    
    // 创建实时线程
    if (pthread_create(&thread, &attr, rt_task_wrapper, NULL)) {
        perror("创建实时线程失败");
        safe_shutdown(false);
        return 1;
    }
    
    pthread_setname_np(thread, "ethercat-rt");
    printf("实时线程创建成功 (优先级: %d)\n", param.sched_priority);
    
    // 运行ROS2执行器
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(global_node);
    
    // 初始化灯光控制器
    init_lights_controller();
    
    printf("=== EtherCAT控制系统启动完成 ===\n");
    printf("系统状态:\n");
    printf("  - EtherCAT主站: 已激活\n");
    printf("  - IO监控: %s\n", global_node->is_io_running() ? "已启动" : "未启动");
    printf("  - 伺服轴数量: %zu\n", global_node->get_servo_axes().size());
    printf("  - DI模块: %s\n", is_di_module_enabled() ? "启用" : "禁用");
    printf("  - DO模块: %s\n", is_do_module_enabled() ? "启用" : "禁用");
    printf("按Ctrl+C退出程序\n\n");
    printf("  - 实时线程: 等待启动按钮\n");
    
    // 使用非阻塞的spin方式，添加启动/暂停检测
    while (rclcpp::ok() && !g_should_exit) {
        executor.spin_some(std::chrono::milliseconds(100));
        
        // 灯光控制（按钮灯 + 三色灯）
        DI_Interface di = read_all_di_signals();
        // 急停按钮：M516（emergency_stop）和 M517（air_supply/气源输入）短按启动3秒计时，满3秒触发暂停
        update_button_lights(di.start_button, di.reset_button, di.pause_button, di.emergency_stop, di.air_supply);
        update_tricolor_lights();
        
        // 检查短按暂停请求
        if (g_short_pause_requested.load() && g_system_running.load()) {
            printf("检测到短按暂停请求...\n");
            pause_motors_only();
            printf("系统已短按暂停，等待启动按钮恢复...\n");
        }
        
        // 检查完整关闭请求（急停或长按暂停按钮触发）
        if (g_full_shutdown_requested.load()) {
            printf("检测到完整关闭请求（急停）...\n");
            g_system_running.store(false);
            g_start_button_pressed.store(false);
            g_reset_button_pressed.store(false);
            g_full_shutdown_requested.store(false);
            safe_shutdown(true);  // true表示暂停模式
            printf("系统已完全暂停，等待启动按钮...\n");
        }
        
        // 检查恢复后的模式切换请求（轴就绪后再执行模式切换）
        if (g_resume_mode_switch_pending.load() && g_system_running.load() && global_node) {
            auto& axes = global_node->get_servo_axes();
            bool all_axes_ready = true;
            for (auto& axis : axes) {
                if (!axis->is_ready()) {
                    all_axes_ready = false;
                    break;
                }
            }
            
            if (all_axes_ready) {
                bool is_auto = g_resume_auto_mode.load();
                printf("[恢复运行] 所有轴已就绪，执行%s模式切换...\n", 
                       is_auto ? "自动" : "手动");
                // 所有轴就绪，绿灯常亮
                notify_all_axes_ready();
                for (auto& axis : axes) {
                    if (is_auto) {
                        axis->start_auto_mode();
                    } else {
                        axis->start_manual_mode();
                    }
                }
                g_resume_mode_switch_pending.store(false);
                printf("[恢复运行] 模式切换完成\n");
            }
        }
        
        // 检查自动模式位置初始化完成状态（用于恢复时通知业务逻辑）
        if (g_system_running.load() && global_node && !g_auto_mode_initialized.load()) {
            auto& axes = global_node->get_servo_axes();
            bool all_axes_auto_initialized = true;
            for (auto& axis : axes) {
                if (!axis->is_auto_mode_initialized()) {
                    all_axes_auto_initialized = false;
                    break;
                }
            }
            
            if (all_axes_auto_initialized) {
                g_auto_mode_initialized.store(true);
                
                // 复位回原完成后 → 安全关闭（释放EtherCAT，不改变灯光，不发布初始化完成信号）
                if (g_reset_homing_shutdown_pending.load()) {
                    printf("[复位回原] 所有轴自动模式位置初始化完成，安全关闭释放EtherCAT资源...\n");
                    g_reset_homing_shutdown_pending.store(false);
                    // 复位完成，黄灯闪烁 → 绿灯闪烁，熄灭复位灯与蜂鸣器
                    notify_system_ready();
                    // 重置标志，确保重启后重新发布初始化完成信号
                    g_auto_mode_initialized.store(false);
                    if (global_node) {
                        global_node->reset_auto_mode_init_published();
                    }
                    // 安全关闭：释放EtherCAT资源，保持灯光和ROS2
                    g_system_running.store(false);
                    g_reset_button_pressed.store(false);
                    safe_shutdown(true);
                    printf("[复位回原] 安全关闭完成，按启动按钮重新启动\n");
                } else {
                    // 正常流程：发布自动模式初始化完成信号
                    printf("[恢复运行] 所有轴自动模式位置初始化完成\n");
                    // 关键修复：自动模式初始化完成后校正层号
                    global_node->calibrate_layer_after_auto_init();
                    // 所有轴自动模式初始化完成，绿灯常亮
                    notify_all_axes_ready();
                }
            }
        }
        
        // 新增：检测系统正常运行时所有轴是否就绪（启动按钮按下后的状态）
        if (g_system_running.load() && global_node && get_tricolor_state() == LIGHT_GREEN_BLINK) {
            auto& axes = global_node->get_servo_axes();
            bool all_axes_ready = true;
            for (auto& axis : axes) {
                if (!axis->is_ready()) {
                    all_axes_ready = false;
                    break;
                }
            }
            if (all_axes_ready) {
                notify_all_axes_ready();
            }
        }
        
        // 检查启动按钮 - 区分是恢复还是重新启动
        if (g_start_button_pressed.load() && !g_system_running.load()) {
            // 检查是否是短按暂停后的恢复
            if (g_short_pause_active.load()) {
                printf("从短按暂停状态恢复...\n");
                g_system_running.store(true);
                resume_from_short_pause();
                printf("系统已恢复运行\n");
                // 恢复完成后清除启动按钮状态，避免重复触发
                g_start_button_pressed.store(false);
                continue;
            }

            printf("开始重新启动系统...\n");
            g_system_running.store(true);
            g_start_button_pressed.store(false);
            // 清除暂停标志，防止实时线程立即退出
            g_pause_button_pressed.store(false);
            // 重置自动模式初始化标志（确保重新等待轴就绪）
            g_auto_mode_initialized.store(false);
            if (global_node) {
                global_node->reset_auto_mode_init_published();
            }
            
            // 重新初始化EtherCAT资源
            if (!master) {
                printf("重新初始化EtherCAT资源...\n");
                
                // 重新请求主站
                master = ecrt_request_master(0);
                if (!master) {
                    fprintf(stderr, "重新请求EtherCAT主站失败\n");
                    g_system_running.store(false);
                    continue;
                }
                
                // 重新创建域
                domain1 = ecrt_master_create_domain(master);
                if (!domain1) {
                    fprintf(stderr, "重新创建域失败\n");
                    ecrt_release_master(master);
                    master = nullptr;
                    g_system_running.store(false);
                    continue;
                }
                
                // 重新配置从站和PDO
                global_node->init_axes(master);
                global_node->register_pdo_entries(domain1);
                
                // 重新激活主站
                if (ecrt_master_activate(master)) {
                    fprintf(stderr, "主站重新激活失败\n");
                    safe_shutdown(false);
                    continue;
                }
                
                // 重新获取域数据指针
                domain1_pd = ecrt_domain_data(domain1);
                if (!domain1_pd) {
                    fprintf(stderr, "重新获取域数据失败\n");
                    safe_shutdown(false);
                    continue;
                }
                // 在轴初始化后初始化业务逻辑和层处理器
                global_node->initialize_after_axes();
            }
            
            // 创建新的实时线程
            running = 1;
            if (pthread_create(&thread, &attr, rt_task_wrapper, NULL)) {
                perror("创建实时线程失败");
                safe_shutdown(false);
                continue;
            }
            pthread_setname_np(thread, "ethercat-rt");
            printf("实时线程重新创建成功\n");
            
            printf("系统重新启动完成\n");
        }
        else if (g_reset_button_pressed.load() && !g_system_running.load())
        {
            printf("回原启动系统...\n");
            g_system_running.store(true);
            // 清除暂停标志，防止实时线程立即退出（急停后残留）
            g_pause_button_pressed.store(false);
            // 清除完整关闭请求标志
            g_full_shutdown_requested.store(false);
            // 重置自动模式初始化标志（确保重新等待轴就绪）
            g_auto_mode_initialized.store(false);
            if (global_node) {
                global_node->reset_auto_mode_init_published();
            }
            // 标记复位回原完成后需要安全关闭（释放EtherCAT资源，保持灯光不变）
            g_reset_homing_shutdown_pending.store(true);
            
            // 重新初始化EtherCAT资源
            if (!master) {
                printf("重新初始化EtherCAT资源...\n");
                
                // 重新请求主站
                master = ecrt_request_master(0);
                if (!master) {
                    fprintf(stderr, "重新请求EtherCAT主站失败\n");
                    g_system_running.store(false);
                    continue;
                }
                
                // 重新创建域
                domain1 = ecrt_master_create_domain(master);
                if (!domain1) {
                    fprintf(stderr, "重新创建域失败\n");
                    ecrt_release_master(master);
                    master = nullptr;
                    g_system_running.store(false);
                    continue;
                }
                
                // 重新配置从站和PDO
                global_node->init_axes(master);
                global_node->register_pdo_entries(domain1);
                
                // 重新激活主站
                if (ecrt_master_activate(master)) {
                    fprintf(stderr, "主站重新激活失败\n");
                    safe_shutdown(false);
                    continue;
                }
                
                // 重新获取域数据指针
                domain1_pd = ecrt_domain_data(domain1);
                if (!domain1_pd) {
                    fprintf(stderr, "重新获取域数据失败\n");
                    safe_shutdown(false);
                    continue;
                }
                // 在轴初始化后初始化业务逻辑和层处理器
                global_node->initialize_after_axes();
            }
            
            // 创建新的实时线程
            running = 1;
            if (pthread_create(&thread, &attr, rt_task_wrapper, NULL)) {
                perror("创建实时线程失败");
                safe_shutdown(false);
                continue;
            }
            pthread_setname_np(thread, "ethercat-rt");
            printf("实时线程重新创建成功\n");
            
            printf("系统重新启动完成\n");    
        }

        // 定期检查退出标志
        if (g_should_exit) {
            break;
        }
    }
    return 0; // 添加适当的返回值
}