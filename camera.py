import time

import cv2
import numpy as np
from sort import bh_sort
import RPi.GPIO as GPIO
import control

# 全局变量用于存储九宫格的编号和坐标结果
global_rectangles = []
global_circles = []  # 用于存储检测到的圆的信息
# Initialize prev_state globally
global_prev_state = [0] * 9
KEY_START = 13

GPIO.setmode(GPIO.BCM)
GPIO.setup(KEY_START, GPIO.IN, GPIO.PUD_UP)


def check_circles_status():
    # 初始化字典，用于存储每个编号对应的圆状态
    circles_status = {rect_id: 0 for rect_id, _, _ in global_rectangles}

    # 遍历所有检测到的圆
    for rect_id, color, (x, y) in global_circles:
        # 如果圆的编号在 rectangles 中，更新状态
        if rect_id in circles_status:
            if color == "black":
                circles_status[rect_id] = 1
            elif color == "white":
                circles_status[rect_id] = 2

    return circles_status


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


def rectangle_detection(img, img_contour, max_area_limit, min_area_limit, historical_data, frame_count):
    contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
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

    # 合并相近的矩形
    def merge_close_rectangles(cross_centers, threshold=10):
        merged_centers = []
        while cross_centers:
            base = cross_centers.pop(0)
            close_rects = [base]
            for other in cross_centers[:]:
                if np.sqrt((base[0] - other[0]) ** 2 + (base[1] - other[1]) ** 2) < threshold:
                    close_rects.append(other)
                    cross_centers.remove(other)
            avg_cx = int(np.mean([rect[0] for rect in close_rects]))
            avg_cy = int(np.mean([rect[1] for rect in close_rects]))
            avg_x = int(np.mean([rect[2] for rect in close_rects]))
            avg_y = int(np.mean([rect[3] for rect in close_rects]))
            avg_w = int(np.mean([rect[4] for rect in close_rects]))
            avg_h = int(np.mean([rect[5] for rect in close_rects]))
            merged_centers.append((avg_cx, avg_cy, avg_x, avg_y, avg_w, avg_h, close_rects[0][6]))
        return merged_centers

    cross_centers = merge_close_rectangles(cross_centers)
    print(f"Detected {len(cross_centers)} merged rectangles")

    if len(cross_centers) == 9:
        # 仅传递坐标点到 bh_sort 函数
        coordinates = [(cx, cy) for cx, cy, x, y, w, h, approx in cross_centers]
        sorted_coordinates = bh_sort(coordinates)

        # Step 1: 创建映射字典
        sorted_coordinates_index = {(x, y): i for i, (x, y) in enumerate(sorted_coordinates)}
        # Step 2: 排序 cross_centers
        sorted_centers = sorted(cross_centers, key=lambda item: sorted_coordinates_index[(item[0], item[1])])

        cx_5, cy_5 = sorted_centers[4][:2]  # 中心点为排序后第五个点
        global_rectangles.clear()

        for i, (cx, cy, x, y, w, h, approx) in enumerate(sorted_centers):
            rel_cx, rel_cy = cx - cx_5, cy - cy_5  # 相对坐标
            draw_rectangle(img_contour, cx, cy, x, y, w, h, approx, i)
            historical_data[(cx, cy)] = frame_count
            global_rectangles.append((i + 1, cx, cy))  # 存储编号和坐标
        return True, img_contour  # 返回成功和处理后的图像
    else:
        print("> 45 degree")

    return False, img_contour  # 返回失败和处理后的图像


def qizi_detect(img, min_radius, max_radius, distance_threshold=10):
    imgGray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    imgBlur = cv2.GaussianBlur(imgGray, (5, 5), 1)
    imgCanny = cv2.Canny(imgBlur, 50, 150)
    imgContour = img.copy()

    circles = cv2.HoughCircles(
        imgCanny,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=20,
        param1=50,
        param2=30,
        minRadius=min_radius,
        maxRadius=max_radius
    )

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        merged_circles = merge_close_circles(circles, threshold=10)  # 合并相近的圆
        for (x, y, r) in merged_circles:
            for rect in global_rectangles:
                rect_id, rect_x, rect_y = rect
                distance = np.sqrt((x - rect_x) ** 2 + (y - rect_y) ** 2)
                if distance <= distance_threshold:
                    color = "black" if imgGray[y, x] < 128 else "white"
                    global_circles.append((rect_id, color, (x, y)))
                    cv2.circle(imgContour, (x, y), r, (0, 255, 0), 2)
                    cv2.drawMarker(imgContour, (x, y), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
                    cv2.putText(imgContour, f"{rect_id} ({color})", (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
    else:
        print("No circles detected")

    return imgContour


def merge_close_circles(circles, threshold=10):
    merged_circles = []
    while len(circles) > 0:
        base_circle = circles[0]
        close_circles = [base_circle]
        for other_circle in circles[1:]:
            distance = np.sqrt((base_circle[0] - other_circle[0]) ** 2 + (base_circle[1] - other_circle[1]) ** 2)
            if distance < threshold:
                close_circles.append(other_circle)
        for close_circle in close_circles:
            circles = np.delete(circles, np.where(np.all(circles == close_circle, axis=1)), axis=0)
        avg_x = int(np.mean([circle[0] for circle in close_circles]))
        avg_y = int(np.mean([circle[1] for circle in close_circles]))
        avg_r = int(np.mean([circle[2] for circle in close_circles]))
        merged_circles.append((avg_x, avg_y, avg_r))
    return merged_circles


def qipan_detect(img, max_area_limit, min_area_limit, historical_data, frame_count):
    imgContour = img.copy()
    imgGray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    imgBlur = cv2.GaussianBlur(imgGray, (5, 5), 1)
    imgCanny = cv2.Canny(imgBlur, 50, 150)

    # 识别矩形
    success, imgContour = rectangle_detection(imgCanny, imgContour, max_area_limit, min_area_limit, historical_data, frame_count)
    if not success:
        print("未能找到九个矩形")


def get_changed_positions(prev_state, current_state):
    """
    判断棋局是否发生变化，返回变化的棋子上一步所在格子编号。

    :param prev_state: 上一步棋局状态（长度为9的列表）
    :param current_state: 当前棋局状态（长度为9的列表）
    :return: 变化的棋子上一步所在格子编号的列表
    """
    from_qz = 0
    go_qz = 0
    chan = False
    for i in range(9):
        if prev_state[i] != current_state[i] and prev_state[i] != 0:
            chan=True
    if chan == True:
        for i in range(9):
            #qii移动到哪
            if prev_state[i] != current_state[i] and prev_state[i] == 0:
                from_qz=i+1
            #qii从哪移动
            if prev_state[i] != current_state[i] and current_state[i] == 0:
                go_qz=i+1

    return chan,from_qz,go_qz


def evaluate_board(state, player):
    """
    评估棋盘状态，返回一个最佳的落子位置。

    :param state: 当前棋局状态（长度为9的列表）
    :param player: 当前玩家（1为黑棋，2为白棋）
    :return: 最佳的落子位置（1-9）或 -1 如果游戏已经结束
    """
    opponent = 3 - player
    best_move = None
    best_score = -float('inf')

    def check_win(board, player):
        win_conditions = [
            [0, 1, 2], [3, 4, 5], [6, 7, 8],  # Rows
            [0, 3, 6], [1, 4, 7], [2, 5, 8],  # Columns
            [0, 4, 8], [2, 4, 6]  # Diagonals
        ]
        for condition in win_conditions:
            if all(board[pos] == player for pos in condition):
                return True
        return False

    def get_winning_move(board, player):
        for move in range(9):
            if board[move] == 0:  # 空位
                new_state = board.copy()
                new_state[move] = player
                if check_win(new_state, player):
                    return move + 1
        return None

    # 检查游戏是否已经结束
    if check_win(state, player) or check_win(state, opponent) or all(cell != 0 for cell in state):
        return -1

    # 优先选择中间位置
    if player == 1 and state[4] == 0:
        return 5

    # 先手优先尝试直接获胜
    winning_move = get_winning_move(state, player)
    if winning_move is not None:
        return winning_move

    # 阻止对手直接获胜
    blocking_move = get_winning_move(state, opponent)
    if blocking_move is not None:
        return blocking_move

    for move in range(9):
        if state[move] == 0:  # 空位
            new_state = state.copy()
            new_state[move] = player

            # 简单评分：对角线上的分数
            score = 0
            for condition in [[0, 1, 2], [3, 4, 5], [6, 7, 8], [0, 3, 6], [1, 4, 7], [2, 5, 8], [0, 4, 8], [2, 4, 6]]:
                player_count = sum(1 for pos in condition if new_state[pos] == player)
                opponent_count = sum(1 for pos in condition if new_state[pos] == opponent)
                if player_count == 2 and opponent_count == 0:
                    score += 10  # 我方即将形成三子连线的威胁
                if opponent_count == 2 and player_count == 0:
                    score += 5  # 对手即将形成三子连线的威胁

            if score > best_score:
                best_score = score
                best_move = move + 1

    return best_move if best_move is not None else -1


# 在 lhr 函数中调用 check_circles_status 来打印结果
def lhr(player):
    cap = cv2.VideoCapture(1)
    scale = 1.3
    max_area_limit = 20000
    min_area_limit = 7000
    min_radius = 30
    max_radius = 39  # 直径 50 以下
    distance_threshold = 50
    historical_data = {}
    frame_count = 0
##################################################################################################
    while True:
        success, img = cap.read()
        if not success:
            break

        img_resized = resize_image(img, scale)
        global_rectangles.clear()
        qipan_detect(img_resized, max_area_limit, min_area_limit, historical_data, frame_count)
        frame_count += 1

        # 如果已经检测到了九宫格，则停止识别
        if len(global_rectangles) == 9:
            print("九宫格检测完成，编号和坐标如下：")
            for rect in global_rectangles:
                print(f"编号: {rect[0]}, 坐标: ({rect[1]}, {rect[2]})")
            break

    if player == 1:
        step_count = 5
    else:
        step_count = 4

    iii = 0
    naqz_i = 0
    while iii < step_count:
        iii += 1
        FLAG = True
        while FLAG:
            if GPIO.input(KEY_START) == 0:
                FLAG = False

        global_circles.clear()
        for _ in range(20):
            success, img = cap.read()
            if not success:
                break
            img_resized = resize_image(img, scale)
            imgContour = qizi_detect(img_resized, min_radius, max_radius, distance_threshold)
            # cv2.imshow("Detected Circles", imgContour)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

        # 检查每个矩形编号是否有对应的圆
        status = check_circles_status()
        print("编号对应的圆状态如下：")
        for rect_id, status_value in status.items():
            status_str = {0: "没有圆", 1: "黑色", 2: "白色"}[status_value]
            print(f"编号: {rect_id}, 状态: {status_str}")
        print(status)

        could, xia_x, xia_y = xiaqi(player)
        if could is True:
            if xia_x == 0:
                print("win")
                break
            print("下棋")
            control.egde_xi(player,naqz_i)#两侧拿起子
            naqz_i = naqz_i + 1
            control.fang(xia_x,xia_y)
        else:
            step_count = step_count+1

    cap.release()
    cv2.destroyAllWindows()


def xiaqi(player):
    global global_prev_state

    prev_state = global_prev_state  # 初始状态
    current_state = [0] * 9

    status = check_circles_status()
    for rect_id, status_value in status.items():
        current_state[rect_id-1]=status_value
    print(current_state)

    print("prev_state：", prev_state)
    # 判断棋局变化
    changes,from_qz,go_qz = get_changed_positions(prev_state, current_state)
    # 更新前一步状态
    global_prev_state = current_state
    if changes:
        print("棋局变化：上一步变化的棋子格子编号：", from_qz,go_qz)
        for rect in global_rectangles:
            if from_qz == rect[0]:
                fromqz_x = rect[1]
                fromqz_y = rect[2]
            if go_qz == rect[0]:
                goqz_x = rect[1]
                goqz_y = rect[2]

        ################################################################################################################

        control.xi(fromqz_x,fromqz_y)
        control.fang(goqz_x,goqz_y)
        return False,0,0
    else:
        # 计算最佳下一步
        best_move = evaluate_board(current_state, player)
        if best_move != -1:
            print(f"最佳下一步落子位置：{best_move}")
            for rect in global_rectangles:
                if best_move == rect[0]:
                    bm_x = rect[1]
                    bm_y = rect[2]
            return True, bm_x, bm_y
        else:
            return True, 0, 0
