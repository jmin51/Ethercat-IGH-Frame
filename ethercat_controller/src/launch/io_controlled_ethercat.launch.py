from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
import os

def generate_launch_description():
    # 获取包共享目录
    pkg_ethercat_controller = get_package_share_directory('ethercat_controller')
    pkg_business_logic_py = get_package_share_directory('business_logic_py')
    
    # 声明启动参数
    enable_dynamic_control_arg = DeclareLaunchArgument(
        'enable_dynamic_control',
        default_value='true',
        description='启用动态控制（IO按钮控制EtherCAT启停）'
    )
    
    use_realtime_arg = DeclareLaunchArgument(
        'use_realtime',
        default_value='true',
        description='启用实时调度（需要root权限）'
    )
    
    auto_mode_enabled_arg = DeclareLaunchArgument(
        'auto_mode_enabled', 
        default_value='false',
        description='初始是否启用自动模式'
    )
    
    # 构建参数文件路径
    config_file = PathJoinSubstitution([
        pkg_ethercat_controller,
        'config',
        'io_control_params.yaml'
    ])
    
    # 检查参数文件是否存在
    config_file_path = os.path.join(pkg_ethercat_controller, 'config', 'io_control_params.yaml')
    if not os.path.exists(config_file_path):
        # 如果文件不存在，使用默认参数
        default_params = {
            'enable_dynamic_control': LaunchConfiguration('enable_dynamic_control'),
            'startup_delay_ms': 2000,
            'shutdown_delay_ms': 1000,
            'start_button_di': 0,
            'pause_button_di': 2, 
            'emergency_stop_di': 4
        }
        param_source = default_params
    else:
        param_source = [config_file]
    
    # EtherCAT控制器节点
    ethercat_node = Node(
        package='ethercat_controller',
        executable='ethercat_controller',
        name='ethercat_controller',
        output='screen',
        parameters=[param_source],
        # # 修复：使用PythonExpression替代map
        # prefix=PythonExpression([
        #     "'sudo -E env \"PATH=$PATH\" chrt -f 99' if '", 
        #     LaunchConfiguration('use_realtime'), 
        #     "' == 'true' else ''"
        # ]),
        emulate_tty=True,
        ros_arguments=['--log-level', 'info']
    )
    
    # Python业务逻辑节点
    business_logic_node = Node(
        package='business_logic_py',
        executable='business_logic_node',
        name='business_logic_processor',
        output='screen',
        parameters=[{
            'use_unified_protocol': False,
            'auto_mode_enabled': LaunchConfiguration('auto_mode_enabled')
        }],
        # 业务逻辑节点不需要实时优先级
        emulate_tty=True
    )
    
    # ByteMultiArray解析器节点
    byte_parser_node = Node(
        package='business_logic_py',
        executable='byte_multiarray_parser',
        name='byte_multiarray_parser',
        output='screen',
        # 解析器节点通常不需要特殊参数
        emulate_tty=True
    )
    
    # 启动信息
    startup_info = LogInfo(
        msg=["=== EtherCAT控制系统启动 ===",
             "节点配置:",
             "  - EtherCAT控制器: 已启用动态控制",
             "  - 业务逻辑处理器: 已启动", 
             "  - ByteMultiArray解析器: 已启动",
             "动态控制: ", LaunchConfiguration('enable_dynamic_control'),
             "实时调度: ", LaunchConfiguration('use_realtime'),
             "自动模式: ", LaunchConfiguration('auto_mode_enabled'),
             "等待IO启动信号..."]
    )
    
    return LaunchDescription([
        enable_dynamic_control_arg,
        use_realtime_arg, 
        auto_mode_enabled_arg,
        startup_info,
        ethercat_node,
        business_logic_node,
        byte_parser_node,
    ])