import serial
import struct
import time

# 初始化串口
ser = serial.Serial('/dev/ttyUSB0', 9600)  # 请根据实际情况更改端口


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
    frame = struct.pack('<BBHHHHBB',
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


# 示例调用
arm_control(1, 1500, 90.00, 45.00, 135.00)
