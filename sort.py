import math
import numpy as np


def plot_points(points, title='Points on Coordinate System', x_label='X coordinate', y_label='Y coordinate'):
    """
    在坐标系上绘制点，并标注坐标。

    参数:
    - points: 包含点坐标的列表，每个点是一个二元组 (x, y)。
    - title: 图形标题
    - x_label: X 轴标签
    - y_label: Y 轴标签
    """
    # 提取 x 和 y 坐标
    x_coords = [p[0] for p in points]
    y_coords = [p[1] for p in points]


def find_l_point(points):
    min_x = min(p[0] for p in points)
    candidates = [p for p in points if p[0] == min_x]
    if len(candidates) > 1:
        return max(candidates, key=lambda p: p[1])
    else:
        return candidates[0]


def find_h_point(points):
    max_y = max(p[1] for p in points)
    candidates = [p for p in points if p[1] == max_y]
    if len(candidates) > 1:
        center_x = np.mean([p[0] for p in candidates])
        center_y = max_y
        return center_x, center_y
    else:
        return candidates[0]


def find_r_point(points):
    max_x = max(p[0] for p in points)
    candidates = [p for p in points if p[0] == max_x]
    if len(candidates) > 1:
        return max(candidates, key=lambda p: p[1])
    else:
        return candidates[0]


def find_extreme_points(pts):
    leftmost = find_l_point(pts)
    rightmost = find_r_point(pts)
    topmost = find_h_point(pts)
    return leftmost, rightmost, topmost


def compute_angle(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    angle = math.degrees(math.atan2(dy, dx))
    return angle


def normalize_angle(angle):
    if angle > 90:
        angle -= 180
    elif angle < -90:
        angle += 180
    return angle


def rotate_point(p, center, angle):
    angle_rad = math.radians(angle)
    i, x, y = p
    cx, cy = center
    x_new = cx + (x - cx) * math.cos(angle_rad) - (y - cy) * math.sin(angle_rad)
    y_new = cy + (x - cx) * math.sin(angle_rad) + (y - cy) * math.cos(angle_rad)
    return i, x_new, y_new


def sort_by_position(pts):
    sorted_pts = sorted(pts, key=lambda p: p[2])
    groups = [sorted_pts[i:i + 3] for i in range(0, len(sorted_pts), 3)]
    grouped_sorted_pts = []
    for group in groups:
        sorted_group = sorted(group, key=lambda p: p[1], reverse=True)
        grouped_sorted_pts.extend(sorted_group)
    return grouped_sorted_pts


def update_points(sorted_rotated_points, new_points):
    # 创建一个字典，以便快速查找 new_points 中的元素
    new_points_dict = {p[0]: (p[1], p[2]) for p in new_points}
    # 遍历 sorted_rotated_points，并根据 new_points 中的匹配项更新第二和第三个元素
    updated_points = []
    for point in sorted_rotated_points:
        if point[0] in new_points_dict:
            new_x, new_y = new_points_dict[point[0]]
            updated_points.append((point[0], new_x, new_y))
        else:
            updated_points.append(point)

    return updated_points


def compute_center(pts):
    x_sum = sum(p[0] for p in pts)
    y_sum = sum(p[1] for p in pts)
    n = len(pts)
    return x_sum / n, y_sum / n


def bh_sort(points):
    # 1. 找到最左端、最右端和最上端的点
    leftmost, rightmost, topmost = find_extreme_points(points)

    # 2. 选出最左端和最右端中更高的点
    high_point = max(leftmost, rightmost, key=lambda p: p[1])

    # 3. 计算偏转角
    angle = compute_angle(topmost, high_point)
    angle = normalize_angle(angle)
    print(angle)

    new_points = tuple((i,) + points[i] for i in range(len(points)))
    # 4. 计算中心点，并旋转所有点
    center = compute_center(points)
    rotated_points = [rotate_point(p, center, -angle) for p in new_points]
    # print(rotated_points)

    # 5. 按照旋转后的点进行排序
    sorted_rotated_points = sort_by_position(rotated_points)

    # print(sorted_rotated_points)
    sorted_original_points = update_points(sorted_rotated_points, new_points)
    # 返回排序后的原始点
    return [(p[1], p[2]) for p in sorted_original_points]
