import serial
import struct
import time
import numpy as np
import matplotlib.pyplot as plt

# 初始化串口
ser = serial.Serial('/dev/ttyAMA0', 9600)  # 请根据实际情况更改端口
# 定义连杆的长度
a = 10.4
b = 9.9
c = 16.5

def arm_point(x, y):
    t1, t2, t3 = inverse_kinematics(x, y)
    th_end1=90-t1
    th_end2= t2
    th_end3=180-t3
    print(f"目标点({x}, {y}) -> 角度: theta1 = {th_end1:.2f}, theta2 = {th_end2:.2f}, theta3 = {th_end3:.2f}")
    arm_control(3, 1000, th_end1, th_end2, th_end3)

def calculate_checksum(x_logic, x, angle1_int, angle2_int, angle3_int):
    # 按位与计算校验位
    return x_logic & x & angle1_int & angle2_int & angle3_int

def arm_control(x_logic, x, angle1, angle2, angle3):
    # 将角度乘以100转换为整数
    angle1_int = int(angle1 * 100)
    angle2_int = int(angle2 * 100)
    angle3_int = int(angle3 * 100)
   
    # 计算校验位
    checksum = calculate_checksum(x_logic, x, angle1_int, angle2_int, angle3_int)
    # 将数据打包成二进制格式
    frame = struct.pack('>BBHHHHBB',
                        0x7B,       # 帧头
                        x_logic,    # x逻辑
                        x,          # x
                        angle1_int, # servo1 角度
                        angle2_int, # servo2 角度
                        angle3_int, # servo3 角度
                        checksum,   # 校验位
                        0x7D)       # 帧尾

    # 发送数据帧
    ser.write(frame)
    print(f"Sent: {frame}")

    # 等待确认消息
    while True:
        if ser.in_waiting >= 3:
            ack_frame = ser.read(3)
            if ack_frame[0] == 0x7B and ack_frame[1] == 1 and ack_frame[2] == 0x7D:
                print("Acknowledgment received")
                break
            else:
                print("Invalid acknowledgment")

def inverse_kinematics(x, y):
    # 计算c杆的终点
    x_c = x
    y_c = y - c

    # 计算b杆终点的长度
    L = np.sqrt(x_c ** 2 + y_c ** 2)

    if L > a + b:
        raise ValueError("目标点超出工作空间")

    # 计算角度
    theta2 = np.arccos((x_c ** 2 + y_c ** 2 - a ** 2 - b ** 2) / (2 * a * b))
    theta1 = np.arctan2(y_c, x_c) - np.arctan2(b * np.sin(theta2), a + b * np.cos(theta2))

    # 转换为度数
    theta1 = np.degrees(theta1)
    theta2 = np.degrees(theta2)
    theta3 = 90 - (theta1 + theta2)  # 由于c杆垂直于x轴

    return theta1, theta2, theta3


# 示例调用
arm_point(10,14)