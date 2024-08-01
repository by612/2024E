import serial
import struct
import time
import numpy as np

angle1_bias = -600
angle2_bias = 0
angle3_bias = 450

# 初始化串口
ser = serial.Serial('/dev/ttyAMA0', 9600)  # 请根据实际情况更改端口
# 定义连杆的长度
a = 10.4
b = 9.9
c = 16.5
# z14.3,y9-19


def arm_point(x, y, z):
    t1, t2, t3 = inverse_kinematics(y, z)
    th_end1 = 90 - t1
    th_end2 = t2
    th_end3 = 180 - t3
    #print(f"目标点({x}, {y}, {z}) -> 角度: theta1 = {th_end1:.2f}, theta2 = {th_end2:.2f}, theta3 = {th_end3:.2f}")
    arm_i = 10
    while arm_i != 0:
        arm_control(1, x, th_end1 + 2 * arm_i, th_end2 - arm_i, th_end3)
        time.sleep(0.05)
        arm_i = arm_i - 1


def arm_point_derect(x, y, z):
    t1, t2, t3 = inverse_kinematics(y, z)
    th_end1 = 90 - t1
    th_end2 = t2
    th_end3 = 180 - t3
    #print(f"目标点({x}, {y}, {z}) -> 角度: theta1 = {th_end1:.2f}, theta2 = {th_end2:.2f}, theta3 = {th_end3:.2f}")
    arm_control(1, x, th_end1, th_end2, th_end3)


def calculate_checksum(x_logic, x, angle1_int, angle2_int, angle3_int):
    # 按位与计算校验位
    return x_logic & x & angle1_int & angle2_int & angle3_int


def arm_control(x_logic, x, angle1, angle2, angle3):
    # 将角度乘以100转换为整数
    angle1_int = int(angle1 * 100) + angle1_bias
    angle2_int = int(angle2 * 100) + angle2_bias
    angle3_int = int(angle3 * 100) + angle3_bias

    # 计算校验位
    checksum = calculate_checksum(x_logic, x, angle1_int, angle2_int, angle3_int)
    # 将数据打包成二进制格式
    frame = struct.pack('>BBHHHHBB',
                        0x7B,  # 帧头
                        x_logic,  # x逻辑
                        x,  # x
                        angle1_int,  # servo1 角度
                        angle2_int,  # servo2 角度
                        angle3_int,  # servo3 角度
                        checksum,  # 校验位
                        0x7D)  # 帧尾

    # 发送数据帧
    ser.write(frame)
    # print(f"Sent: {frame}")

    # 等待确认消息
    while True:
        if ser.in_waiting >= 3:
            ack_frame = ser.read(3)
            if ack_frame[0] == 0x7B and ack_frame[1] == 1 and ack_frame[2] == 0x7D:
                # print("Acknowledgment received")
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
# y_wait 15
# y_get 17

# i = 0
# while i < 5:
#     arm_point_derect(4050, 15, 12 + i)
#     time.sleep(0.0005)
#     i = i + 0.02
# time.sleep(3)
# arm_point(4050, 15, 16)
# time.sleep(10)
# arm_point(0, 18, 14)
# time.sleep(100)
# arm_control(6, 1000,180,10,45)
l1 = 4270
x1 = 17
l2 = l1-650
x2 = x1-3
l3 = l1-1300
x3 = x1-5.8
down = 17.3 # down to 17cm
up = 16 #put at 16cm
l4 = l1 + 1100
l5 = l3 - 800
l6 = l4 - 100
x4 = 19.8
x5 = 8.5
l7 = l5-50
x12 = x2-0.1
l8 = l6-50
l9=l7-90
arm_control(1, 0, 180, 10, 45)
time.sleep(2)


def slow(x, y, z):
    i = 14
    while i < z:
        arm_point_derect(x, y, i)
        time.sleep(0.0005)
        i = i + 0.02
    time.sleep(1)


def sii(x):
    if (x == 1):
        arm_control(7, 1000, 180, 10, 45)
        time.sleep(1)
    else:
        arm_control(6, 1000, 180, 10, 45)


def slow_up(x, y, i):
    while i > 15:
        arm_point_derect(x, y, i)
        time.sleep(0.0005)
        i = i - 0.02
    time.sleep(0.5)


def xi(l, x): 
    slow(l, x, down)
    sii(1)
    slow_up(l, x, down)


def fang(l, x):
    slow(l, x, up)
    sii(0)
    slow_up(l, x, up)
    arm_control(1, 1000, 180, 10, 45)
    time.sleep(0.5)


def question3(l, x, n=0):
    n+=1
    print(f'n: {n}')
    if(n==1):
        xi(l7, x1)
    elif(n==2):
        xi(l6, x1)
    elif(n==3):
        xi(l9, x12)    
    elif(n==4):
        xi(l6, x4)
    else:
        print("out of 4 times")
        return 0
    
    slow(l, x, up)
    sii(0)
    slow_up(l, x, up)
    arm_control(1, 1000, 180, 10, 45)
    time.sleep(0.5)


def question2(x,n):
    print(f'n: {n}')
    if(n==1):
        xi(l7, x1)
    elif(n==2):
        xi(l6, x1)
    elif(n==3):
        xi(l9, x12)    
    elif(n==4):
        xi(l6, x4)
    else:
        print("out of 4 times")
        return 0
        
    if (x == 1):
        fang(l3, x1)
        print("放的位置是1")
    elif (x == 2):
        fang(l2, x1)
        print("放的位置是2")
    elif (x == 3):
        fang(l1, x1)
        print("放的位置是3")
    elif (x == 4):
        fang(l3, x2)
        print("放的位置是4")
    elif (x == 5):
        fang(l2, x2)
        print("放的位置是5")
    elif (x == 6):
        fang(l1, x2)
        print("放的位置是6")
    elif (x == 7):
        fang(l3, x3)
        print("放的位置是7")
    elif (x == 8):
        fang(l2, x3)
        print("放的位置是9")
    elif (x == 9):
        fang(l1, x3)
        print("放的位置是9")


# xi(l1,x1)
# fang(l6,x2)
# xi(l1,x2)
# fang(l2,x1)
# xi(l2,x1)
# fang(l2,x2)
# xi(l2,x2)
# fang(l3,x1)
# xi(l3,x1)
# fang(l1,x3)
# xi(l1,x3)
# fang(l2,x3)
# xi(l2,x3)
# fang(l3,x3)
# arm_point(l2, x1, y1-0.3)
# time.sleep(2)
# arm_control(6, 1000,180,10,45)
# time.sleep(2)
# arm_point(l3, x1, y1)
# time.sleep(1)
# arm_point(l1, x2, y1)
# time.sleep(1)
# arm_point(l2, x2, y1)
# time.sleep(1)
# arm_point(l3, x2, y1)
# time.sleep(1)
# arm_point(l1, x3, y3)
# time.sleep(1)
# arm_point(l2, x3, y3)
# time.sleep(1)
# arm_point(l3, x3, y3)
# time.sleep(1)
# bias1 = 0.7
# l4 = l1+1200
# x4 = x3-bias1
# x5 = x2-bias1
# x6 = x1-bias1
# x7 = x1+2.5
# x8 = x7+2
# y4 = y3+0.2
# y5 = y4+0.2
# l5 = l3 - 1200
# arm_point(l5, x4, y4)
# time.sleep(1)
# arm_point(l5, x5, y4)
# time.sleep(1)
# arm_point(l5, x6, y5)
# time.sleep(1)
# arm_point(l5, x7, y5)
# time.sleep(1)
# arm_point(l5, x8, y4)
# time.sleep(1)

# i=10
# while i!=0:
#     i=i-1
#     arm_point(4050,19-i,14)
#     time.sleep(1)
# time.sleep(10)
# 831.81-pix*37.93 = x_value
# 6886.8-5.91*pix = l_value


