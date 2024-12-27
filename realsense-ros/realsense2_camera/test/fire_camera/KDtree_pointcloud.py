import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import time

# 初始化 RealSense 管線
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# 開始捕獲深度影像
pipeline.start(config)

# 等待幾秒鐘，以確保相機已完全開啟
time.sleep(2)

align = rs.align(rs.stream.color)
frames = pipeline.wait_for_frames()
aligned_frames = align.process(frames)

# 取得對齊的深度影像
depth_frame = aligned_frames.get_depth_frame()
if not depth_frame:
    raise RuntimeError("無法捕獲深度影像")

# 將深度影像轉換為 NumPy 陣列
pc = rs.pointcloud()
points = pc.calculate(depth_frame)
vertices = np.asanyarray(points.get_vertices()) #xyz point cloud
pipeline.stop()

# 將點雲資料轉換為 Open3D 格式
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(vertices.view(np.float32).reshape(-1, 3))

# 構建 KDTree
kdtree = o3d.geometry.KDTreeFlann(point_cloud)

# 指定目標點
target_point = np.array([x, y, z])  

# 查找近鄰點 (比如 10 個最近點)
[k, idx, _] = kdtree.search_knn_vector_3d(target_point, 10)

# 提取近鄰點
neighbor_points = np.asarray(point_cloud.points)[idx, :]

# 計算深度值 (Z 值)
depths = neighbor_points[:, 2]
print("周圍點的深度值：", depths)

# 添加目標點
point_cloud.paint_uniform_color([0.5, 0.5, 0.5])  # 點雲灰色
target_cloud = o3d.geometry.PointCloud()
target_cloud.points = o3d.utility.Vector3dVector([target_point])
target_cloud.paint_uniform_color([1.0, 0, 0])  # 目標點紅色

# 合併點雲
combined = point_cloud + target_cloud

# 可視化
o3d.visualization.draw_geometries([combined])