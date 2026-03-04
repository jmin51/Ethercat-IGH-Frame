#!/bin/bash
# start_ngbuffer_simple.sh - 简化版启动脚本

# 函数：检查EtherCAT主站状态
check_ethercat_status() {
    print_step "检查EtherCAT主站状态"
    
    # 检查ethercat命令是否可用
    if ! command -v ethercat &> /dev/null; then
        print_error "ethercat命令未找到，请检查EtherCAT安装"
        return 1
    fi
    
    # 尝试获取从站列表，判断主站是否已运行
    if sudo ethercat slaves > /dev/null 2>&1; then
        print_info "EtherCAT主站已在运行状态"
        # 记录当前从站状态
        sudo ethercat slaves > $LOG_DIR/ethercat_slaves_current.log 2>&1
        return 0
    else
        print_warning "EtherCAT主站未运行或需要重启"
        return 1
    fi
}

# 函数：启动EtherCAT主站（条件判断）
start_ethercat_master() {
    # 先检查主站状态
    if check_ethercat_status; then
        print_info "跳过EtherCAT主站重启"
        return 0
    fi
    
    print_step "启动EtherCAT主站"
    
    # 检查服务脚本是否存在
    if [ ! -f /etc/init.d/ethercat ]; then
        print_error "EtherCAT服务脚本不存在: /etc/init.d/ethercat"
        return 1
    fi
    
    # 执行重启
    if sudo /etc/init.d/ethercat restart; then
        sleep 3
        # 验证启动结果
        if sudo ethercat slaves > $LOG_DIR/ethercat_slaves.log 2>&1; then
            print_info "EtherCAT主站启动成功"
            return 0
        else
            print_error "EtherCAT主站启动后从站检测失败"
            return 1
        fi
    else
        print_error "EtherCAT主站重启失败"
        return 1
    fi
}

# 配置
LOG_DIR="$HOME/ros2_launch_logs"
mkdir -p "$LOG_DIR"

# 生成带日期的日志文件名
LOG_FILE="$LOG_DIR/ngbuffer_$(date +%Y%m%d_%H%M%S).log"

# 启动并记录
echo "启动: $(date)" > "$LOG_FILE"
ros2 launch business_logic_py NGbuffer_system.launch.py "$@" 2>&1 | tee -a "$LOG_FILE"
echo "结束: $(date)" >> "$LOG_FILE"

echo "日志已保存到: $LOG_FILE"