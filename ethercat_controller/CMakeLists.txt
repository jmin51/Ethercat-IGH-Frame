cmake_minimum_required(VERSION 3.8)
project(ethercat_controller)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # 禁用特定警告
  add_compile_options(-Wno-unused-parameter -Wno-missing-field-initializers -Wno-switch)
endif()

# 设置构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

# EtherCAT库路径
include_directories(/usr/local/etherlab/include)
link_directories(/usr/local/etherlab/lib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_library(MODBUS_LIBRARY modbus)

# 包含目录
include_directories(
  src/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/io_modules  # 新增IO模块路径
  ${CMAKE_CURRENT_SOURCE_DIR}/src/business  # 新增业务逻辑模块路径
)

# 源文件
set(SOURCES
  src/main.cpp
  src/ethercat_node.cpp
  src/servo_axis_base.cpp
  src/leisai_servo_axis.cpp
  src/huichuan_servo_axis.cpp
  src/servo_axis_factory.cpp
  src/io_modules/io_interface.cpp  # 新增IO模块
  src/business/LayerCommandProcessor.cpp  # 新增层指令处理模块
)

# 创建可执行文件
add_executable(ethercat_controller ${SOURCES})

# 链接依赖
ament_target_dependencies(ethercat_controller
    rclcpp
    std_msgs
    sensor_msgs
)

# 链接库
target_link_libraries(ethercat_controller
    /usr/local/etherlab/lib/libethercat.so
    ${MODBUS_LIBRARY}
    pthread
    rt
)

# 安装可执行文件
install(TARGETS ethercat_controller
    DESTINATION lib/${PROJECT_NAME}
)

# 修复：安装包含文件（确保目录存在）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/include")
  install(DIRECTORY src/include/
    DESTINATION include/${PROJECT_NAME}
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/io_modules")
  install(DIRECTORY src/io_modules/
    DESTINATION include/${PROJECT_NAME}/io_modules
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/business")
  install(DIRECTORY src/business/
    DESTINATION include/${PROJECT_NAME}/business
  )
endif()

# 安装脚本文件（如果存在）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/scripts/set_permissions.sh")
  install(PROGRAMS src/scripts/set_permissions.sh
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# 安装 launch 文件（如果存在）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/launch")
  install(DIRECTORY
    src/launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# 导出依赖
ament_export_dependencies(rclcpp std_msgs sensor_msgs)
ament_export_include_directories(src/include)
ament_export_include_directories(src/io_modules)
ament_export_include_directories(src/business)

ament_package()

# cmake_minimum_required(VERSION 3.8)
# project(ethercat_controller)

# # 设置C++标准
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # EtherCAT库路径
# set(CMAKE_BUILD_TYPE "Debug")
# include_directories(/usr/local/etherlab/include)
# link_directories(/usr/local/etherlab/lib)

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)  # 必须添加
# find_library(MODBUS_LIBRARY modbus)

# # 包含目录
# include_directories(
#   src/include
#   ${CMAKE_CURRENT_SOURCE_DIR}src/include
# )

# # 源文件
# set(SOURCES
#   src/main.cpp
#   src/ethercat_node.cpp
#   src/servo_axis_base.cpp
#   src/leisai_servo_axis.cpp
#   src/huichuan_servo_axis.cpp
#   src/servo_axis_factory.cpp
# )

# # 创建可执行文件
# add_executable(ethercat_controller ${SOURCES})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights 条件判断是否启用测试功能
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo) 检查源代码文件是否包含版权声明
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   # 运行cpplint工具检查C++代码风格
#   ament_lint_auto_find_test_dependencies()
# endif()


# # 链接依赖
# ament_target_dependencies(ethercat_controller
#     rclcpp
#     std_msgs
#     sensor_msgs  # 关键
# )

# # ROS2特定链接
# target_link_libraries(ethercat_controller
#     ${rclcpp_LIBRARIES}  # 显式链接rclcpp库 9.15添加
#     /usr/local/etherlab/lib/libethercat.so
#     ${MODBUS_LIBRARY}  # 将MODBUS_LIBRARY移到这里
#     pthread
#     rt  # 添加实时库
# )

# # 安装和导出
# install(TARGETS ethercat_controller
#     DESTINATION lib/${PROJECT_NAME}
# )

# # 安装包含文件
# install(DIRECTORY src/include/
#   DESTINATION include/${PROJECT_NAME}
# )

# # 导出依赖
# ament_export_dependencies(rclcpp sensor_msgs std_msgs)
# ament_export_include_directories(src/include)

# install(PROGRAMS src/scripts/set_permissions.sh
#         DESTINATION lib/${PROJECT_NAME})
# # 安装 launch 文件
# install(DIRECTORY
#     src/launch
#     DESTINATION share/${PROJECT_NAME}
# )
# ament_package()
