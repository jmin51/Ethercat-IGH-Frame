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

# 用户密码（用于sudo）
USER_PASSWORD="000"

# 函数：带密码的sudo执行
sudo_with_password() {
    echo "$USER_PASSWORD" | sudo -S "$@"
}

# 函数：启动EtherCAT主站（带密码）
start_ethercat_with_password() {
    print_step() { echo "[步骤] $1"; }
    print_info() { echo "[信息] $1"; }
    print_warning() { echo "[警告] $1"; }
    print_error() { echo "[错误] $1"; }

    print_step "启动EtherCAT主站"

    # 检查服务脚本是否存在
    if [ ! -f /etc/init.d/ethercat ]; then
        print_error "EtherCAT服务脚本不存在: /etc/init.d/ethercat"
        return 1
    fi

    # 停止现有服务（如果正在运行）
    echo "$USER_PASSWORD" | sudo -S /etc/init.d/ethercat stop > /dev/null 2>&1
    sleep 1

    # 启动服务
    if echo "$USER_PASSWORD" | sudo -S /etc/init.d/ethercat start; then
        sleep 3
        # 验证启动结果
        if echo "$USER_PASSWORD" | sudo -S ethercat slaves > "$LOG_DIR/ethercat_slaves.log" 2>&1; then
            print_info "EtherCAT主站启动成功"
            cat "$LOG_DIR/ethercat_slaves.log"
            return 0
        else
            print_error "EtherCAT主站启动后从站检测失败"
            return 1
        fi
    else
        print_error "EtherCAT主站启动失败"
        return 1
    fi
}

# 启动EtherCAT主站
start_ethercat_with_password

# ROS2日志格式配置 - 显示时间戳
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{time}] [{severity}] [{name}]: {message}"
export RCUTILS_TIME_OUTPUT_FORMAT="%Y-%m-%d %H:%M:%S"

# 生成带日期的日志文件名
LOG_FILE="$LOG_DIR/ngbuffer_$(date +%Y%m%d_%H%M%S).log"

# 启动并记录
echo "启动: $(date)" > "$LOG_FILE"
ros2 launch business_logic_py NGbuffer_system.launch.py "$@" 2>&1 | tee -a "$LOG_FILE"
echo "结束: $(date)" >> "$LOG_FILE"

echo "日志已保存到: $LOG_FILE"