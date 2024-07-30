import cv2
import numpy as np
from collections import deque
# from control import arm_control

PLAYER = 'X'
OPPONENT = 'O'


def initialize_board(start_player='X'):
    """
    初始化棋盘，设置先行玩家
    :param start_player: 'X' 或 'O' 表示先行的玩家
    :return: 初始化后的棋盘
    """
    if start_player not in ['X', 'O']:
        raise ValueError("start_player 必须是 'X' 或 'O'")

    # 初始化一个空棋盘
    board = [['_' for _ in range(3)] for _ in range(3)]

    # 设置先行玩家在中间位置
    # board[1][1] = start_player

    return board

    # 测试初始化棋盘


board = initialize_board('X')
print('棋盘状态')
for row in board:
    print(row)

# 输出:
# ['_', '_', '_']
# ['_', 'X', '_']
# ['_', '_', '_']


def resize_image(img, scale):
    """
    缩放图像到指定比例
    :param img: 输入图像
    :param scale: 缩放比例，>1 表示放大，<1 表示缩小
    :return: 缩放后的图像
    """
    width = int(img.shape[1] * scale)
    height = int(img.shape[0] * scale)
    dimensions = (width, height)
    return cv2.resize(img, dimensions, interpolation=cv2.INTER_AREA)


def draw_rectangle(img_contour, cx, cy, x, y, w, h, approx, i):
    """
    在图像上绘制矩形和角点
    """
    cv2.rectangle(img_contour, (x, y), (x + w, y + h), (0, 0, 255), 2)  # 绘制边界框
    cv2.drawMarker(img_contour, (cx, cy), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
    coord_text = f"({cx},{cy})"
    cv2.putText(img_contour, coord_text, (cx + 10, cy - 10), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 0, 0), 1)
    cv2.putText(img_contour, f"{i + 1}", (cx - 10, cy - 20), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 0, 255), 2)
    # for point in approx:
    #     cv2.circle(img_contour, tuple(point[0]), 5, (0, 255, 255), -1)  # 标记每个角点


# 检查棋盘上是否还有空格可供移动
# 存在空格返回True
# 不存在空格返回False
def is_moves_left(board):
    for row in board:
        if '_' in row:
            return True
    return False


# 评估当前棋盘的状态，判断是否有玩家获胜
# 如果玩家获胜，加10分
# 如果对手获胜，减10分
# 否则返回0
def evaluate(board):
    # 检查行
    for row in board:
        if row[0] == row[1] == row[2]:
            if row[0] == PLAYER:
                return +10
            elif row[0] == OPPONENT:
                return -10

    # 检查列
    for col in range(3):
        if board[0][col] == board[1][col] == board[2][col]:
            if board[0][col] == PLAYER:
                return +10
            elif board[0][col] == OPPONENT:
                return -10

    # 检查对角线
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


# 实现了Minimax算法，并使用α-β剪枝进行优化
# 它递归地评估所有可能的棋盘状态，返回最佳得分
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


# 找到最佳移动，返回最佳移动的位置
def find_best_move(board):
    # 输入board: 当前棋盘状态，一个3×3的列表
    # 输出best_move：最佳移动的位置元组(row, col)
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
    """
    将棋盘上的行列元组转换为对应的编号。
    :param position: 一个表示位置的元组
    :return: 对应的编号
    """
    row, col = position
    if 1 <= row <= 3 and 1 <= col <= 3:
        return (row - 1) * 3 + col
    else:
        raise ValueError("行和列的值必须在1到3之间")


def rectangle_detection(img, img_contour, prev_centers, max_area_limit, min_area_limit):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)  # 寻找轮廓点
    cross_centers = []

    for obj in contours:
        area = cv2.contourArea(obj)  # 计算轮廓内区域的面积
        if area > max_area_limit or area < min_area_limit:  # 跳过超过最大面积限制或小于最小面积限制的矩形
            continue
        perimeter = cv2.arcLength(obj, True)  # 计算轮廓周长
        approx = cv2.approxPolyDP(obj, 0.02 * perimeter, True)  # 获取轮廓角点坐标
        CornerNum = len(approx)  # 轮廓角点的数量
        x, y, w, h = cv2.boundingRect(approx)  # 获取坐标值和宽度、高度

        # 仅检测矩形
        if CornerNum == 4:
            cx, cy = x + w // 2, y + h // 2
            cross_centers.append((cx, cy, x, y, w, h, approx))

    # 如果找到的中心点数量为 9 个
    if len(cross_centers) == 9:
        # 提取中心点坐标
        centers = np.array([c[:2] for c in cross_centers])
        rect = np.zeros((9, 2), dtype="float32")

        # 计算每个中心点的 x 和 y 坐标之和
        s = centers.sum(axis=1)
        # 找到左上角和右下角的点
        idx_0 = np.argmin(s)
        idx_8 = np.argmax(s)

        # 计算每个中心点的 y 和 x 坐标之差
        diff = np.diff(centers, axis=1).flatten()
        # 找到右上角和左下角的点
        idx_2 = np.argmin(diff)
        idx_6 = np.argmax(diff)

        rect[0] = centers[idx_0]
        rect[2] = centers[idx_2]
        rect[6] = centers[idx_6]
        rect[8] = centers[idx_8]

        # 找到其余的五个点
        mask = np.ones(centers.shape[0], dtype=bool)
        mask[[idx_0, idx_2, idx_6, idx_8]] = False
        others = centers[mask]

        # 找到最左边、最右边、最上边、最下边的点
        idx_l = others[:, 0].argmin()
        idx_r = others[:, 0].argmax()
        idx_t = others[:, 1].argmin()
        idx_b = others[:, 1].argmax()

        found = np.array([idx_l, idx_r, idx_t, idx_b])
        mask = np.isin(range(len(others)), found, invert=False)
        idx_c = np.where(mask == False)[0]

        # 如果成功找到中心点，则排序并编号
        if len(idx_c) == 1:
            rect[1] = others[idx_t]
            rect[3] = others[idx_l]
            rect[4] = others[idx_c]
            rect[5] = others[idx_r]
            rect[7] = others[idx_b]

            # 写编号
            for i in range(9):
                cx, cy = rect[i]
                # 找出对应的cross_centers元素
                for c in cross_centers:
                    if c[0] == cx and c[1] == cy:
                        x, y, w, h, approx = c[2:]
                        draw_rectangle(img_contour, cx, cy, x, y, w, h, approx, i)
                        break
        else:
            # 大于 45 度的情况
            print("> 45 degree")
    else:
        # 按从上到下、从左到右的顺序对中心点进行排序
        cross_centers.sort(key=lambda c: (c[1], c[0]))  # 按 y 坐标和 x 坐标排序

        # 合并重叠的矩形
        rects = [(x, y, x + w, y + h) for _, _, x, y, w, h, _ in cross_centers]
        rects, _ = cv2.groupRectangles(rects, groupThreshold=1, eps=0.1)

        for i, (x1, y1, x2, y2) in enumerate(rects):
            w, h = x2 - x1, y2 - y1
            cx, cy = x1 + w // 2, y1 + h // 2
            approx = np.array([[[x1, y1]], [[x2, y1]], [[x2, y2]], [[x1, y2]]])
            draw_rectangle(img_contour, cx, cy, x1, y1, w, h, approx, i)


def main():
    # 初始化棋盘并设置初始状态
    board = initialize_board('X')

    print('初始棋盘状态：')
    for row in board:
        print(row)

    # 捕获视频流
    cap = cv2.VideoCapture(1)
    scale = 1.3  # 设置缩放比例
    max_area_limit = 90000  # 设置矩形的最大面积限制
    min_area_limit = 30  # 设置矩形的最小面积限制

    # 初始化前一帧
    prev_centers = deque(maxlen=10)  # 保存前几帧的检测结果

    while True:
        success, img = cap.read()
        if not success:
            break

        imgContour = img.copy()

        imgGray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)  # 转灰度图
        imgBlur = cv2.GaussianBlur(imgGray, (5, 5), 1)  # 高斯模糊
        imgCanny = cv2.Canny(imgBlur, 50, 150)  # 调高阈值进行Canny边缘检测
        rectangle_detection(imgCanny, imgContour, prev_centers, max_area_limit, min_area_limit)  # 矩形检测

        imgContour = resize_image(imgContour, scale)  # 调整图像大小

        cv2.imshow("Rectangle Detection", imgContour)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 游戏进行中
    game_over = False
    current_player = OPPONENT

    while not game_over:
        if current_player == PLAYER:
            print("Player's turn (X):")
            best_move = find_best_move(board)
            board[best_move[0]][best_move[1]] = PLAYER
            move_number = tuple_to_number((best_move[0] + 1, best_move[1] + 1))  # 转换为1-9编号
            print(f"Player (X) moves to position {move_number}: {best_move}")
        else:
            print("Opponent's turn (O):")
            # 假设对手的走棋策略是随机选择（可以根据实际需要调整）
            empty_cells = [(i, j) for i in range(3) for j in range(3) if board[i][j] == '_']
            if empty_cells:
                move = empty_cells[np.random.randint(len(empty_cells))]
                board[move[0]][move[1]] = OPPONENT
                move_number = tuple_to_number((move[0] + 1, move[1] + 1))  # 转换为1-9编号
                print(f"Opponent (O) moves to position {move_number}: {move}")

        # 打印当前棋盘状态
        print('当前棋盘状态：')
        for row in board:
            print(row)

        # 检查游戏是否结束
        score = evaluate(board)
        if score == 10:
            print("Player (X) wins!")
            game_over = True
        elif score == -10:
            print("Opponent (O) wins!")
            game_over = True
        elif not is_moves_left(board):
            print("It's a draw!")
            game_over = True
        else:
            # 切换玩家
            current_player = PLAYER if current_player == OPPONENT else OPPONENT

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
