import cv2
import numpy as np
import threading
from collections import deque

import camera
from control import fang, question2
from camera import lhr, rectangle_detection, qizi_detect, global_rectangles
import time
import random
import RPi.GPIO as GPIO
import key_module

# 定义按键引脚
KEY1 = 20
KEY2 = 21
KEY3 = 16
KEY4 = 12
KEY5 = 26
KEY6 = 19

global vision_running

PLAYER = 'X'
OPPONENT = 'O'

# 设置GPIO模式
GPIO.setmode(GPIO.BCM)
GPIO.setup(KEY1, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(KEY2, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(KEY3, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(KEY4, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(KEY5, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(KEY6, GPIO.IN, GPIO.PUD_UP)

# 全局变量，用于在两个线程之间共享数据
rect_centers = deque(maxlen=10)
lock = threading.Lock()
condition = threading.Condition(lock)

# 标志位，确保只执行一次机械臂控制操作
control_executed = threading.Event()


def initialize_board(start_player='X'):
    """初始化棋盘"""
    if start_player not in ['X', 'O']:
        raise ValueError("start_player 必须是 'X' 或 'O'")
    board = [['_' for _ in range(3)] for _ in range(3)]
    return board


def resize_image(img, scale):
    """调整图像大小"""
    width = int(img.shape[1] * scale)
    height = int(img.shape[0] * scale)
    dimensions = (width, height)
    return cv2.resize(img, dimensions, interpolation=cv2.INTER_AREA)


def draw_rectangle(img_contour, cx, cy, x, y, w, h, approx, i):
    """在图像上绘制矩形"""
    cv2.rectangle(img_contour, (x, y), (x + w, y + h), (0, 0, 255), 2)
    cv2.drawMarker(img_contour, (cx, cy), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
    coord_text = f"({cx},{cy})"
    cv2.putText(img_contour, coord_text, (cx + 10, cy - 10), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 0, 0), 1)
    cv2.putText(img_contour, f"{i + 1}", (cx - 10, cy - 20), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 0, 255), 2)


def draw_circle(img_contour, cx, cy, radius):
    """在图像上绘制圆形"""
    cv2.circle(img_contour, (cx, cy), radius, (255, 0, 0), 2)


def is_moves_left(board):
    """检查棋盘是否还有空位"""
    for row in board:
        if '_' in row:
            return True
    return False


def evaluate(board):
    """评估当前棋盘状态"""
    for row in board:
        if row[0] == row[1] == row[2]:
            if row[0] == PLAYER:
                return +10
            elif row[0] == OPPONENT:
                return -10

    for col in range(3):
        if board[0][col] == board[1][col] == board[2][col]:
            if board[0][col] == PLAYER:
                return +10
            elif board[0][col] == OPPONENT:
                return -10

    if board[0][0] == board[1][1] == board[2][2]:
        if board[0][0] == PLAYER:
            return +10
        elif board[0][0] == OPPONENT:
            return -10

    if board[0][2] == board[1][1] == board[2][0]:
        if board[0][2] == PLAYER:
            return +10
        elif board[0][2] == OPPONENT:
            return -10

    return 0


def minimax(board, depth, is_max, alpha, beta):
    """Minimax算法带alpha-beta剪枝"""
    score = evaluate(board)

    if score == 10:
        return score - depth

    if score == -10:
        return score + depth

    if not is_moves_left(board):
        return 0

    if is_max:
        best = -1000
        for i in range(3):
            for j in range(3):
                if board[i][j] == '_':
                    board[i][j] = PLAYER
                    best = max(best, minimax(board, depth + 1, False, alpha, beta))
                    alpha = max(alpha, best)
                    board[i][j] = '_'
                    if beta <= alpha:
                        break
        return best
    else:
        best = 1000
        for i in range(3):
            for j in range(3):
                if board[i][j] == '_':
                    board[i][j] = OPPONENT
                    best = min(best, minimax(board, depth + 1, True, alpha, beta))
                    beta = min(beta, best)
                    board[i][j] = '_'
                    if beta <= alpha:
                        break
        return best


def find_best_move(board):
    """寻找最佳移动位置"""
    best_val = -1000
    best_move = (-1, -1)

    for i in range(3):
        for j in range(3):
            if board[i][j] == '_':
                board[i][j] = PLAYER
                move_val = minimax(board, 0, False, -1000, 1000)
                board[i][j] = '_'
                if move_val > best_val:
                    best_move = (i, j)
                    best_val = move_val

    return best_move


def tuple_to_number(position):
    """将位置元组转换为编号"""
    row, col = position
    if 1 <= row <= 3 and 1 <= col <= 3:
        return (row - 1) * 3 + col
    else:
        raise ValueError("行和列的值必须在1到3之间")


def vision_thread(stop_event):
    """视觉检测线程"""
    cap = cv2.VideoCapture(0)
    scale = 1.5
    max_area_limit = 90000
    min_area_limit = 30
    historical_data = {}
    frame_count = 0

    while not stop_event.is_set():
        success, img = cap.read()
        if not success:
            break

        imgContour = img.copy()
        imgGray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        imgBlur = cv2.GaussianBlur(imgGray, (5, 5), 1)
        imgCanny = cv2.Canny(imgBlur, 50, 150)

        success, imgContour = camera.rectangle_detection(imgCanny, imgContour, max_area_limit, min_area_limit, historical_data, frame_count)

        if success:
            # 将排序后的矩形中心加入共享队列
            with lock:
                for i, rect in enumerate(camera.global_rectangles):
                    rect_num, cx, cy = rect
                    rel_cx, rel_cy = cx, cy
                    rect_centers.append((i, rel_cx, rel_cy))
                    print(rect_centers)
                condition.notify_all()

        imgContour = resize_image(imgContour, scale)
        cv2.imshow("矩形检测", imgContour)

        # 检查KEY4是否被按下
        # if cv2.waitKey(1) & GPIO.input(KEY4) == 0:
        #     time.sleep(0.01)
        #     print("按键4被按下，退出程序")
        #     stop_event.set()  # 设置停止事件
        #     break
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def control_thread():
    """机械臂控制线程"""
    selected_number = key_module.select_grid()  # 获取选择的棋格编号
    print('***********')
    print(f'选择的棋格编号： {selected_number}')
    print('***********')

    with condition:
        condition.wait()  # 等待视觉检测线程的通知
        if rect_centers:
            # 根据选择的棋格编号找到对应的 rect_center
            for rect_num, rel_cx, rel_cy in rect_centers:
                if (rect_num + 1) == selected_number:
                    # 将浮点数转换为整数
                    int_cx = int(7074.3 - rel_cx * 5.96)
                    int_cy = int((831.81 - rel_cy) / 37.93)
                    fang(int_cx, int_cy)
                    print('----------')
                    print(f'矩形编号： {rect_num}')
                    print("pwd公式解算出的坐标")
                    print(int_cx, int_cy)
                    print('----------')
                    break  # 找到后退出循环
            else:
                print("未找到对应的矩形编号！")
        control_executed.set()  # 设置标志，表示已执行机械臂控制


def main():
    """主程序入口"""
    i = 1
    while True:
        print("请选择要执行的题目")
        time.sleep(0.1)
        if GPIO.input(KEY1) == 0:
            time.sleep(0.01)
            print("按键1被按下，执行题(1)")
            question2(5, 1)
            while GPIO.input(KEY1) == 0:
                time.sleep(0.01)
        elif GPIO.input(KEY2) == 0:
            time.sleep(0.01)
            print("按键2被按下，执行题(2)")
            selected_number = key_module.select_grid()
            question2(selected_number, i)
            i += 1
            print(f"确认最后输出的数: {selected_number}")
            while GPIO.input(KEY2) == 0:
                time.sleep(0.01)
        elif GPIO.input(KEY3) == 0:
            time.sleep(0.01)
            print("按键3被按下，执行题(3)")
            board = initialize_board('X')

            for _ in range(4):  # 连续执行四次
                stop_event = threading.Event()  # 创建停止事件
                vision = threading.Thread(target=vision_thread, args=(stop_event,))
                control = threading.Thread(target=control_thread)

                vision.start()
                control.start()

                vision.join()  # 等待视觉线程结束
                control.join()  # 等待视觉线程结束
                print("机械臂控制操作已完成")
        elif GPIO.input(KEY4) == 0:
            time.sleep(0.01)
            while True:
                print("按键4被按下，执行题(4)，请选择题号")
                if GPIO.input(KEY5) == 0:
                    time.sleep(0.01)
                    # lhr1()
                    break
                elif GPIO.input(KEY6) == 0:
                    time.sleep(0.01)
                    # lhr2()
                    break
            while GPIO.input(KEY4) == 0:
                time.sleep(0.01)
            break


if __name__ == "__main__":
    main()