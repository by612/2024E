import cv2
import numpy as np
import threading
from collections import deque
from control import arm_point
from control import question2
import time
import random
import RPi.GPIO as GPIO
import key_module

KEY1 = 20
KEY2 = 21
KEY3 = 16
KEY4 = 12

PLAYER = 'X'
OPPONENT = 'O'

GPIO.setmode(GPIO.BCM)
GPIO.setup(KEY1, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(KEY2, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(KEY3, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(KEY4, GPIO.IN, GPIO.PUD_UP)


def key():
    FLAG = 1
    while FLAG == 1:
        time.sleep(0.05)
        if GPIO.input(KEY1) == 0:
            print("KEY1 PRESSED")
            while GPIO.input(KEY1) == 0:
                FLAG = 0
                time.sleep(0.01)
        elif GPIO.input(KEY2) == 0:
            print("KEY2 PRESSED")
            while GPIO.input(KEY2) == 0:
                FLAG = 0
                time.sleep(0.01)
        elif GPIO.input(KEY3) == 0:
            print("KEY3 PRESSED")
            while GPIO.input(KEY3) == 0:
                FLAG = 0
                time.sleep(0.01)
        elif GPIO.input(KEY4) == 0:
            print("KEY4 PRESSED")
            while GPIO.input(KEY4) == 0:
                FLAG = 0
                time.sleep(0.01)


# 全局变量，用于在两个线程之间共享数据
rect_centers = deque(maxlen=10)
lock = threading.Lock()
condition = threading.Condition(lock)

# 标志位，确保只执行一次机械臂控制操作
control_executed = threading.Event()


def initialize_board(start_player='X'):
    if start_player not in ['X', 'O']:
        raise ValueError("start_player 必须是 'X' 或 'O'")

    board = [['_' for _ in range(3)] for _ in range(3)]
    return board


def resize_image(img, scale):
    width = int(img.shape[1] * scale)
    height = int(img.shape[0] * scale)
    dimensions = (width, height)
    return cv2.resize(img, dimensions, interpolation=cv2.INTER_AREA)


def draw_rectangle(img_contour, cx, cy, x, y, w, h, approx, i):
    cv2.rectangle(img_contour, (x, y), (x + w, y + h), (0, 0, 255), 2)
    cv2.drawMarker(img_contour, (cx, cy), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
    coord_text = f"({cx},{cy})"
    cv2.putText(img_contour, coord_text, (cx + 10, cy - 10), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 0, 0), 1)
    cv2.putText(img_contour, f"{i + 1}", (cx - 10, cy - 20), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 0, 255), 2)


def draw_circle(img_contour, cx, cy, radius):
    cv2.circle(img_contour, (cx, cy), radius, (255, 0, 0), 2)


def is_moves_left(board):
    for row in board:
        if '_' in row:
            return True
    return False


def evaluate(board):
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
    row, col = position
    if 1 <= row <= 3 and 1 <= col <= 3:
        return (row - 1) * 3 + col
    else:
        raise ValueError("行和列的值必须在1到3之间")


def rectangle_detection(img, img_contour, prev_centers, max_area_limit, min_area_limit):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cross_centers = []

    for obj in contours:
        area = cv2.contourArea(obj)
        if area > max_area_limit or area < min_area_limit:
            continue
        perimeter = cv2.arcLength(obj, True)
        approx = cv2.approxPolyDP(obj, 0.02 * perimeter, True)
        CornerNum = len(approx)
        x, y, w, h = cv2.boundingRect(approx)
        cx, cy = x + w // 2, y + h // 2

        if CornerNum == 4:
            cross_centers.append((cx, cy, x, y, w, h, approx))

    if len(cross_centers) == 9:
        centers = np.array([c[:2] for c in cross_centers])
        rect = np.zeros((9, 2), dtype="float32")

        s = centers.sum(axis=1)
        idx_0 = np.argmin(s)
        idx_8 = np.argmax(s)

        diff = np.diff(centers, axis=1).flatten()
        idx_2 = np.argmin(diff)
        idx_6 = np.argmax(diff)

        rect[0] = centers[idx_0]
        rect[2] = centers[idx_2]
        rect[6] = centers[idx_6]
        rect[8] = centers[idx_8]

        mask = np.ones(centers.shape[0], dtype=bool)
        mask[[idx_0, idx_2, idx_6, idx_8]] = False
        others = centers[mask]

        idx_l = others[:, 0].argmin()
        idx_r = others[:, 0].argmax()
        idx_t = others[:, 1].argmin()
        idx_b = others[:, 1].argmax()

        found = np.array([idx_l, idx_r, idx_t, idx_b])
        mask = np.isin(range(len(others)), found, invert=False)
        idx_c = np.where(mask == False)[0]

        if len(idx_c) == 1:
            rect[1] = others[idx_t]
            rect[3] = others[idx_l]
            rect[4] = others[idx_c]
            rect[5] = others[idx_r]
            rect[7] = others[idx_b]

            # 获取编号为5的矩形的中心坐标
            cx_5, cy_5 = rect[4]  # rect[4] 对应编号为5的矩形

            for i in range(9):
                cx, cy = rect[i]
                rel_cx, rel_cy = cx - cx_5, cy - cy_5  # 相对坐标
                for c in cross_centers:
                    if c[0] == cx and c[1] == cy:
                        x, y, w, h, approx = c[2:]
                        draw_rectangle(img_contour, cx, cy, x, y, w, h, approx, i)
                        # 将矩形编号和中心坐标加入共享队列
                        with lock:
                            rect_centers.append((i + 1, rel_cx, rel_cy))
                            condition.notify()
                        break
        else:
            print("> 45 degree")
    else:
        cross_centers.sort(key=lambda c: (c[1], c[0]))

        rects = [(x, y, x + w, y + h) for _, _, x, y, w, h, _ in cross_centers]
        rects, _ = cv2.groupRectangles(rects, groupThreshold=1, eps=0.1)

        for i, (x1, y1, x2, y2) in enumerate(rects):
            w, h = x2 - x1, y2 - y1
            cx, cy = x1 + w // 2, y1 + h // 2
            approx = np.array([[[x1, y1]], [[x2, y1]], [[x2, y2]], [[x1, y2]]])
            draw_rectangle(img_contour, cx, cy, x1, y1, w, h, approx, i)
            # 将矩形编号和中心坐标加入共享队列
            with lock:
                rect_centers.append((i + 1, cx, cy))
                condition.notify()


def circle_detection(img, img_contour, max_area_limit, min_area_limit):
    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, dp=1.2, minDist=20,
                               param1=50, param2=50, minRadius=5, maxRadius=50)

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")

        filtered_circles = []
        for (x, y, r) in circles:
            area = np.pi * (r ** 2)
            if min_area_limit <= area <= max_area_limit:
                filtered_circles.append((x, y, r))

        # 合并重叠的圆形
        merged_circles = []
        for (x, y, r) in filtered_circles:
            merged = False
            for i, (mx, my, mr) in enumerate(merged_circles):
                if np.sqrt((x - mx) ** 2 + (y - my) ** 2) < (r + mr) / 2:
                    merged_circles[i] = (int((x + mx) / 2), int((y + my) / 2), int((r + mr) / 2))
                    merged = True
                    break
            if not merged:
                merged_circles.append((x, y, r))

        for (x, y, r) in merged_circles:
            draw_circle(img_contour, x, y, r)


def vision_thread():
    cap = cv2.VideoCapture(0)
    scale = 1.3
    max_area_limit = 90000
    min_area_limit = 30
    prev_centers = deque(maxlen=10)

    while True:
        success, img = cap.read()
        if not success:
            break

        imgContour = img.copy()
        imgGray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        imgBlur = cv2.GaussianBlur(imgGray, (5, 5), 1)
        imgCanny = cv2.Canny(imgBlur, 50, 150)

        rectangle_detection(imgCanny, imgContour, prev_centers, max_area_limit, min_area_limit)
        circle_detection(imgCanny, imgContour, max_area_limit, min_area_limit)

        imgContour = resize_image(imgContour, scale)
        cv2.imshow("Shape Detection", imgContour)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def control_thread():
    with condition:
        condition.wait()  # 等待视觉检测线程的通知
        if rect_centers:
            # 随机选择一个矩形的中心坐标和编号
            rect_num, rel_cx, rel_cy = random.choice(rect_centers)
            # 将浮点数转换为整数
            int_cx = int(rel_cx + 1600)
            int_cy = int(rel_cy * 0.01 + 10)
            int_z = 16
            arm_point(int_cx, int_cy, int_z)
            print('----------')
            print(f'矩形编号 {rect_num}')
            print(int_cx, int_cy, int_z)
            print('----------')
            control_executed.set()  # 设置标志，表示已执行机械臂控制


def main():
    # key()
    selected_number = key_module.select_grid()
    print(f"确认最后输出的函数: {selected_number}")


# board = initialize_board('X')
# print('初始棋盘状态：')
# for row in board:
#     print(row)
#
# vision = threading.Thread(target=vision_thread)
# control = threading.Thread(target=control_thread)
#
# vision.start()
# # control.start()
#
# control_executed.wait()  # 等待控制线程执行完毕
#
# vision.join()
# print("机械臂控制操作已完成")


if __name__ == "__main__":
    main()
