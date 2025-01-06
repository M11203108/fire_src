# Description: 找出地圖上距離特定點位3m的地方
import yaml
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from nav_msgs.msg import OccupancyGrid, MapMetaData


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
    
def load_map(map_file):
    """
    從 YAML 和 PGM 載入 SLAM 地圖並生成 OccupancyGrid 格式數據
    """
    with open(map_file, 'r') as f:
        map_data = yaml.safe_load(f)

    # 獲取圖片的絕對路徑
    image_path = map_data['image']
    if not os.path.isabs(image_path):
        # 如果是相對路徑，基於 YAML 文件的路徑解析
        image_path = os.path.join(os.path.dirname(map_file), image_path)

    # 確認圖片文件是否存在
    if not os.path.exists(image_path):
        raise FileNotFoundError(f"地圖圖片文件未找到: {image_path}")

    # 載入地圖圖片 (.pgm)
    image = Image.open(image_path)
    image_data = np.array(image)

    # 轉換為 ROS 格式的地圖數據
    grid_data = []
    for row in image_data:
        for pixel in row:
            # 根據 SLAM 工具的規則映射像素值
            if pixel == 0:  # 障礙物
                grid_data.append(100)
            elif pixel == 254:  # 可通行區域
                grid_data.append(0)
            else:  # 未知區域
                grid_data.append(-1)

    # 建立 OccupancyGrid 消息
    occupancy_grid = OccupancyGrid()
    occupancy_grid.data = grid_data

    # 填入地圖資訊
    map_meta = MapMetaData()
    map_meta.resolution = map_data['resolution']
    map_meta.origin.position.x = map_data['origin'][0]
    map_meta.origin.position.y = map_data['origin'][1]
    map_meta.width = image_data.shape[1]
    map_meta.height = image_data.shape[0]

    occupancy_grid.info = map_meta

    return occupancy_grid
    
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

    # 圓心轉為地圖座標
    center_x, center_y = map(int, world_to_map(center_x, center_y, map_info))
    radius_in_cells = int(radius / map_info.resolution)  # 半徑轉為地圖單位

    # 使用集合避免重複
    circle_3m_points = set()

    # 生成圓周點（基於對稱性）
    for angle in range(0, 360):
        radian = math.radians(angle)
        x = int(center_x + radius_in_cells * math.cos(radian))
        y = int(center_y + radius_in_cells * math.sin(radian))
        circle_3m_points.add((x, y))  # 加入集合避免重複

    return list(circle_3m_points)

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

def visualize_map(map_handler, circle_points, valid_points):
    if not map_handler.map_ready():
        print("地圖尚未準備好")
        return

    # 將地圖數據轉換為二維陣列
    map_data = np.array(map_handler.map).reshape(
        (map_handler.map_info.height, map_handler.map_info.width)
    )

    # 使用 origin='upper' 來正確顯示 .pgm 地圖方向
    plt.imshow(map_data, cmap='gray', origin='upper')

    # 繪製圓周點
    for x, y in circle_points:
        plt.plot(x, y, 'ro', markersize=2)  # 紅點表示圓周點

    # 繪製有效點
    for x, y in valid_points:
        plt.plot(x, y, 'go', markersize=2)  # 綠點表示有效點

    plt.title("地圖與圓周點")
    plt.xlabel("地圖寬度（格）")
    plt.ylabel("地圖高度（格）")
    plt.show()



if __name__ == '__main__':
    # 替換為您的地圖檔案路徑
    map_yaml_file = "/home/robot/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC.yaml"

    # 載入地圖
    map_handler = MapHandler()
    occupancy_grid = load_map(map_yaml_file)

    # 模擬地圖回調
    map_handler.map_callback(occupancy_grid)

    # 測試參數
    center_x = 0  # 測試點
    center_y = 0
    radius = 3
    robot_radius = 0.6  # 機器人半徑

    # 生成圓周點並檢查有效性
    circle_3m_points = generate_circle_points(center_x, center_y, radius, map_handler.map_info)
    valid_points = check_circle_point_valid(circle_3m_points, map_handler, robot_radius)

    print(f"地圖寬度: {map_handler.map_info.width} 格")
    print(f"地圖高度: {map_handler.map_info.height} 格")
    print(f"解析度: {map_handler.map_info.resolution} 米/格")
    print(f"地圖原點: ({map_handler.map_info.origin.position.x}, {map_handler.map_info.origin.position.y})")


    print(f"有效點數量: {len(valid_points)}")
    print(f"最近的有效點: {find_recent_point(valid_points, center_x, center_y)}")
    visualize_map(map_handler, circle_3m_points, valid_points)