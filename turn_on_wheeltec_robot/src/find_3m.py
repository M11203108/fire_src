# Description: 找出地圖上距離特定點位3m的地方
import math
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid


class MapHandler:
    def __init__(self):
        self.map = None
        self.map_info = None

    def map_callback(self, msg):
        self.map = msg.data
        self.map_info = msg.info
        print("Map received")

    def map_ready(self):
        return self.map is not None and self.map_info is not None
    
def world_to_map(x, y, map_info):
    # 將世界座標轉換為地圖座標
    # x, y: 世界座標
    # map_info: 地圖資訊

    map_x = int((x - map_info.origin.position.x) / map_info.resolution)
    map_y = int((y - map_info.origin.position.y) / map_info.resolution)
    return map_x, map_y

def generate_circle_points(center_x, center_y, radius, map_info):
    # center_x, center_y: 圓心座標
    # radius: 圓半徑
    # resolution: 地圖網格解析度

    circle_3m_points = []
    # 將圓心座標轉換為地圖座標
    center_x, center_y = map(int, world_to_map(center_x, center_y, map_info))

    # 半徑轉換為網格單位
    radius_in_cells = int(radius / map_info.resolution)

    # 以圓心為中心，生成圓周上的點位
    for i in range(0, 360):
        angle = math.radians(i)
        x = int(center_x + radius_in_cells * math.cos(angle))
        y = int(center_y + radius_in_cells * math.sin(angle))
        circle_3m_points.append((x, y))
    return list(set(circle_3m_points))

def check_circle_point_valid(circle_3m_points, map_handler, robot_radius):
    # 檢查圓周上的點位是否有效
    valid_points = []
    if not map_handler.map_ready():
        print("Map not ready")
        return valid_points

    map_data = map_handler.map
    map_width = map_handler.map_info.width
    map_height = map_handler.map_info.height
    resolution = map_handler.map_info.resolution

    # 計算機器人半徑的網格單位
    robot_radius_in_cells = int(robot_radius / resolution)

    for grid_x, grid_y in circle_3m_points:
        if point_valid(map_data, map_width, map_height, grid_x, grid_y, robot_radius_in_cells):
            valid_points.append((grid_x, grid_y))

    return valid_points

def point_valid(map_data, map_width, map_height, grid_x, grid_y, robot_radius_in_cells):
    # 檢查網格點是否有效
    #迴圈看機器人範圍網格內是否有碰到障礙物或在地圖外面
    for i in range(-robot_radius_in_cells, robot_radius_in_cells + 1):
        for j in range(-robot_radius_in_cells, robot_radius_in_cells + 1):
            #nx robot左邊和右邊
            #ny robot上面和下面
            nx, ny = grid_x + i, grid_y + j
            if nx < 0 or nx >= map_width or ny < 0 or ny >= map_height:
                return False
            index = ny * map_width + nx
            if map_data[index] > 0:  # 非空閒區域
                return False
    return True

def find_recent_point(valid_points, robot_x, robot_y):
    # 找出距離機器人最近的點位
    # valid_points: 有效的點位
    # robot_x, robot_y: 機器人座標
    # map_info: 地圖資訊

    min_distance = float('inf')
    recent_point = None

    for x, y in valid_points:
        distance = math.sqrt((x - robot_x) ** 2 + (y - robot_y) ** 2)
        if distance < min_distance:
            min_distance = distance
            recent_point = (x, y)

    return recent_point, min_distance




if __name__ == '__main__':
    # 圓心座標
    center_x = 50
    center_y = 50
    # 圓半徑
    radius = 3
    # 地圖網格解析度
    resolution = 0.05

    circle_3m_points = generate_circle_points(center_x, center_y, radius, resolution)
    # print(circle_3m_points)
