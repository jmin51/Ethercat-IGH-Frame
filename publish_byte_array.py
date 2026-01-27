# """
# ROS2 Byte 发布脚本 - 修复版
# 修复了字节格式化问题
# """

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Byte

# def main():
#     rclpy.init()
    
#     # 创建节点
#     node = Node('minimal_byte_publisher')
#     publisher = node.create_publisher(Byte, '/integrated_control_test', 10)
    
#     # 等待连接建立
#     rclpy.spin_once(node, timeout_sec=1.0)
    
#     # 创建并发送消息
#     msg = Byte()
#     msg.data = bytes([0x01])  # 正确的字节赋值
    
#     try:
#         count = 0
#         while rclpy.ok():
#             publisher.publish(msg)
            
#             # 修复：将bytes转换为整数后再格式化
#             byte_value = msg.data[0]  # 提取字节值转换为整数
#             node.get_logger().info(f'发布消息 #{count}: 0x{byte_value:02X}')
            
#             count += 1
#             rclpy.spin_once(node, timeout_sec=1.0)  # 1秒间隔
#     except KeyboardInterrupt:
#         print('\n停止发布')
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()#!/usr/bin/env python3
#!/usr/bin/env python3
# """
# ROS2 ByteMultiArray 发布脚本 - 修复版
# 修复了data字段类型问题
# """

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import ByteMultiArray, MultiArrayLayout, MultiArrayDimension

# def main():
#     rclpy.init()
    
#     # 创建节点
#     node = Node('minimal_byte_multiarray_publisher')
#     publisher = node.create_publisher(ByteMultiArray, '/integrated_control_test', 10)
    
#     # 等待连接建立
#     rclpy.spin_once(node, timeout_sec=1.0)
    
#     # 创建MultiArrayLayout
#     layout = MultiArrayLayout()
    
#     # 创建维度描述
#     dimension = MultiArrayDimension()
#     dimension.label = ''
#     dimension.size = 2    # 数组大小
#     dimension.stride = 1
    
#     # 设置layout
#     layout.dim = [dimension]
#     layout.data_offset = 0
    
#     # 创建ByteMultiArray消息
#     msg = ByteMultiArray()
#     msg.layout = layout
    
#     # 修复：将整数列表转换为bytes对象列表
#     # 每个元素必须是bytes类型，而不是整数
#     msg.data = [bytes([0x01]), bytes([0x01])]  # 每个元素都是bytes对象
    
#     try:
#         count = 0
#         while rclpy.ok():
#             publisher.publish(msg)
            
#             # 日志输出也需要相应调整
#             data_hex = ' '.join([f'0x{b.hex().upper()}' for b in msg.data])
#             node.get_logger().info(f'发布ByteMultiArray消息 #{count}: [{data_hex}]')
            
#             count += 1
#             rclpy.spin_once(node, timeout_sec=1.0)
            
#             # 动态修改数据内容也需要使用bytes对象
#             if count % 5 == 0:
#                 # 修复：使用bytes对象而不是整数
#                 msg.data = [bytes([count % 256]), bytes([(count + 1) % 256])]
                
#     except KeyboardInterrupt:
#         print('\n停止发布')
#     except Exception as e:
#         print(f'发生错误: {e}')
#         import traceback
#         traceback.print_exc()
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
"""
ROS2 ByteMultiArray 发布脚本 - 单次发送版
只发送一次消息后退出
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, MultiArrayLayout, MultiArrayDimension

def main():
    rclpy.init()
    
    # 创建节点
    node = Node('single_byte_multiarray_publisher')
    publisher = node.create_publisher(ByteMultiArray, '/integrated_control', 10)
    
    # 等待连接建立
    rclpy.spin_once(node, timeout_sec=1.0)
    
    # 创建MultiArrayLayout
    layout = MultiArrayLayout()
    
    # 创建维度描述
    dimension = MultiArrayDimension()
    dimension.label = ''
    dimension.size = 4    # 数组大小
    dimension.stride = 1
    
    # 设置layout
    layout.dim = [dimension]
    layout.data_offset = 0
    
    # 创建ByteMultiArray消息
    msg = ByteMultiArray()
    msg.layout = layout
    
    # 设置数据
    msg.data = [bytes([0x15]), bytes([0x01]), bytes([0x00]), bytes([0x0])]  # 每个元素都是bytes对象
    
    try:
        # 只发送一次消息
        publisher.publish(msg)
        
        # 日志输出
        data_hex = ' '.join([f'0x{b.hex().upper()}' for b in msg.data])
        node.get_logger().info(f'发布ByteMultiArray消息: [{data_hex}]')
        node.get_logger().info('消息已发送，程序退出')
        
    except Exception as e:
        print(f'发生错误: {e}')
        import traceback
        traceback.print_exc()
    finally:
        # 短暂等待确保消息发送
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()