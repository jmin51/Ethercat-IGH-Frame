#!/usr/bin/env python3
"""
ROS2 ByteMultiArray 发布脚本 - 支持多种指令
支持入库指令（0x0101）、出库指令（0x0103）和IO控制指令（0x0115）
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, MultiArrayLayout, MultiArrayDimension
import argparse
import sys

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='发布ByteMultiArray消息')
    parser.add_argument('--command', type=str, required=True, 
                       choices=['warehouse', 'outbound', 'io'], 
                       help='指令类型: warehouse(入库) 或 outbound(出库) 或 io(IO控制)')
    parser.add_argument('--layer', type=int, default=1,
                       help='层高（仅warehouse和outbound指令有效，默认1）')
    parser.add_argument('--io_low', type=lambda x: int(x, 0), default=0x00,
                       help='IO状态低位字节（仅io指令有效，十六进制，默认0x00）')
    parser.add_argument('--io_high', type=lambda x: int(x, 0), default=0x00,
                       help='IO状态高位字节（仅io指令有效，十六进制，默认0x00）')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    # 创建节点
    node = Node('byte_multiarray_publisher')
    publisher = node.create_publisher(ByteMultiArray, '/integrated_control', 10)
    
    # 等待连接建立
    rclpy.spin_once(node, timeout_sec=1.0)
    
    # 创建MultiArrayLayout
    layout = MultiArrayLayout()
    layout.data_offset = 0
    
    # 根据指令类型构造不同的消息
    if args.command == 'warehouse':
        # 入库指令 (0x0101)
        # 格式: [指令码低位0x01, 指令码高位0x01, 组号低位0x00, 组号高位0x00, 
        #        IO状态低位0x00, IO状态高位0x00, 层高低位, 层高高位]
        
        # 将层高转换为小端序的两个字节
        layer_low = args.layer & 0xFF  # 低8位
        layer_high = (args.layer >> 8) & 0xFF  # 高8位
        
        msg_data = [
            bytes([0x01]),  # 指令码低位
            bytes([0x01]),  # 指令码高位 (0x0101 = 入库指令)
            bytes([0x00]),  # 组号低位
            bytes([0x00]),  # 组号高位
            bytes([0x00]),  # IO状态低位
            bytes([0x00]),  # IO状态高位
            bytes([layer_low]),   # 层高低位
            bytes([layer_high])   # 层高高位
        ]
        
        layout.dim = [MultiArrayDimension()]
        layout.dim[0].label = 'warehouse_command'
        layout.dim[0].size = 8
        layout.dim[0].stride = 1
        
        node.get_logger().info(f'构造入库指令: 层高={args.layer} (0x{layer_low:02X} 0x{layer_high:02X})')
        
    elif args.command == 'outbound':
        # 出库指令 (0x0103)
        # 格式: [指令码低位0x03, 指令码高位0x01, 组号低位0x00, 组号高位0x00, 
        #        IO状态低位0x00, IO状态高位0x00, 层高低位, 层高高位]
        
        # 将层高转换为小端序的两个字节
        layer_low = args.layer & 0xFF  # 低8位
        layer_high = (args.layer >> 8) & 0xFF  # 高8位
        
        msg_data = [
            bytes([0x03]),  # 指令码低位
            bytes([0x01]),  # 指令码高位 (0x0103 = 出库指令)
            bytes([0x00]),  # 组号低位
            bytes([0x00]),  # 组号高位
            bytes([0x00]),  # IO状态低位
            bytes([0x00]),  # IO状态高位
            bytes([layer_low]),   # 层高低位
            bytes([layer_high])   # 层高高位
        ]
        
        layout.dim = [MultiArrayDimension()]
        layout.dim[0].label = 'outbound_command'
        layout.dim[0].size = 8
        layout.dim[0].stride = 1
        
        node.get_logger().info(f'构造出库指令: 层高={args.layer} (0x{layer_low:02X} 0x{layer_high:02X})')
        
    elif args.command == 'io':
        # IO控制指令 (0x0115)
        # 格式: [指令码低位0x15, 指令码高位0x01, 组号低位0x00, 组号高位0x00, 
        #        IO状态低位, IO状态高位]
        
        msg_data = [
            bytes([0x15]),  # 指令码低位
            bytes([0x01]),  # 指令码高位 (0x0115 = IO控制指令)
            bytes([0x00]),  # 组号低位
            bytes([0x00]),  # 组号高位
            bytes([args.io_low]),   # IO状态低位
            bytes([args.io_high])   # IO状态高位
        ]
        
        layout.dim = [MultiArrayDimension()]
        layout.dim[0].label = 'io_command'
        layout.dim[0].size = 6
        layout.dim[0].stride = 1
        
        node.get_logger().info(f'构造IO控制指令: IO状态=0x{args.io_high:02X}{args.io_low:02X}')
    
    # 创建ByteMultiArray消息
    msg = ByteMultiArray()
    msg.layout = layout
    msg.data = msg_data
    
    try:
        # 发送消息
        publisher.publish(msg)
        
        # 日志输出
        data_hex = ' '.join([f'0x{b.hex().upper()}' for b in msg.data])
        node.get_logger().info(f'发布ByteMultiArray消息: [{data_hex}]')
        node.get_logger().info('消息已发送，程序退出')
        
    except Exception as e:
        node.get_logger().error(f'发生错误: {e}')
        import traceback
        traceback.print_exc()
    finally:
        # 短暂等待确保消息发送
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()