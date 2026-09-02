#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Open3D 点云处理全流程模块 - 面向智元G2深度相机自主抓取

安装依赖：
    pip install open3d numpy scipy matplotlib

使用方法：
    from open3d_grasp_tutorial import full_grasp_pipeline, g2_grasp_pipeline
    results = full_grasp_pipeline("g2_point_cloud.ply")
"""

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# 第1部分：基础工具函数
# ============================================================

def load_point_cloud(path):
    """加载点云文件（支持 ply, pcd, xyz, xyzrgb, pts 等格式）"""
    pcd = o3d.io.read_point_cloud(path)
    print(f"✅ 加载点云: {len(pcd.points)} 个点")
    return pcd

def save_point_cloud(pcd, path):
    """保存点云"""
    o3d.io.write_point_cloud(path, pcd)
    print(f"💾 已保存: {path}")

def visualize(pcd_list, window_name="Open3D Viewer"):
    """可视化点云列表"""
    o3d.visualization.draw_geometries(
        pcd_list if isinstance(pcd_list, list) else [pcd_list],
        window_name=window_name,
        point_show_normal=False
    )

# ============================================================
# 第2部分：点云预处理（滤波 + 降采样）
# ============================================================

def preprocess_point_cloud(pcd, voxel_size=0.005):
    """
    点云预处理流水线
    1. 统计滤波去噪
    2. 体素降采样
    3. 法线估计
    """
    print(f"\n🔧 开始预处理 (voxel_size={voxel_size}m)...")

    # 1. 统计滤波 - 去除离群点
    print("   步骤1: 统计滤波去噪...")
    pcd_clean, _ = pcd.remove_statistical_outlier(
        nb_neighbors=20,
        std_ratio=2.0
    )
    print(f"      去噪后: {len(pcd_clean.points)} 个点")

    # 2. 体素降采样
    print("   步骤2: 体素降采样...")
    pcd_down = pcd_clean.voxel_down_sample(voxel_size=voxel_size)
    print(f"      降采样后: {len(pcd_down.points)} 个点")

    # 3. 法线估计
    print("   步骤3: 法线估计...")
    pcd_down.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size * 2,
            max_nn=30
        )
    )
    pcd_down.orient_normals_towards_camera_location(
        camera_location=np.array([0, 0, 0])
    )
    print("   ✅ 预处理完成")

    return pcd_down

# ============================================================
# 第3部分：平面分割（去除桌面/背景）
# ============================================================

def segment_plane(pcd, distance_threshold=0.01, ransac_n=3, num_iterations=1000):
    """RANSAC 平面分割 - 用于去除桌面等支撑平面"""
    print(f"\n📐 RANSAC 平面分割...")

    plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=ransac_n,
        num_iterations=num_iterations
    )

    a, b, c, d = plane_model
    print(f"   平面方程: {a:.4f}x + {b:.4f}y + {c:.4f}z + {d:.4f} = 0")

    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([0.5, 0.5, 0.5])

    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    outlier_cloud.paint_uniform_color([1.0, 0.0, 0.0])

    print(f"   平面内点: {len(inlier_cloud.points)}")
    print(f"   平面外点: {len(outlier_cloud.points)}")

    return inlier_cloud, outlier_cloud, plane_model

# ============================================================
# 第4部分：欧氏聚类分割（分离多个物体）
# ============================================================

def cluster_objects(pcd, eps=0.02, min_points=50):
    """DBSCAN 欧氏聚类 - 将点云分割为独立物体"""
    print(f"\n🎯 DBSCAN 欧氏聚类 (eps={eps}m, min_points={min_points})...")

    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug):
        labels = np.array(pcd.cluster_dbscan(
            eps=eps,
            min_points=min_points,
            print_progress=True
        ))

    max_label = labels.max()
    print(f"   检测到 {max_label + 1} 个物体")

    if max_label < 0:
        print("   ⚠️ 未检测到任何聚类，所有点被标记为噪声")
        print("   💡 建议：减小 eps 或 min_points 参数")
        return [], labels

    # 为每个聚类分配不同颜色
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = [0, 0, 0, 1]
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

    clusters = []
    for i in range(max_label + 1):
        cluster = pcd.select_by_index(np.where(labels == i)[0])
        clusters.append(cluster)

    return clusters, labels

# ============================================================
# 第5部分：物体特征提取（包围盒 + 质心）
# ============================================================

def get_object_features(cluster_pcd):
    """提取物体的几何特征"""
    points = np.asarray(cluster_pcd.points)

    centroid = np.mean(points, axis=0)
    aabb = cluster_pcd.get_axis_aligned_bounding_box()
    aabb_extent = aabb.get_extent()

    obb = cluster_pcd.get_oriented_bounding_box()
    obb_extent = obb.extent

    centered = points - centroid
    cov = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    idx = eigenvalues.argsort()[::-1]
    eigenvalues = eigenvalues[idx]
    eigenvectors = eigenvectors[:, idx]

    features = {
        'centroid': centroid,
        'aabb_extent': aabb_extent,
        'obb_extent': obb_extent,
        'principal_axes': eigenvectors,
        'eigenvalues': eigenvalues,
        'volume': np.prod(aabb_extent),
    }

    return features

def draw_bounding_box(pcd, features, color=[0, 1, 0]):
    """绘制有向包围盒"""
    obb = pcd.get_oriented_bounding_box()
    obb.color = color
    return obb

# ============================================================
# 第6部分：模型导入与配准
# ============================================================

def load_mesh_model(path, num_points=10000):
    """加载3D模型文件（obj, stl, ply 等）并采样为点云"""
    print(f"\n📦 加载模型: {path}")

    mesh = o3d.io.read_triangle_mesh(path)
    if len(mesh.triangles) == 0:
        pcd = o3d.io.read_point_cloud(path)
        print(f"   按点云读取: {len(pcd.points)} 个点")
        return pcd, None

    mesh.compute_vertex_normals()
    pcd = mesh.sample_points_uniformly(number_of_points=num_points)
    pcd.estimate_normals()

    print(f"   三角面数: {len(mesh.triangles)}")
    print(f"   采样点云: {len(pcd.points)} 个点")

    return pcd, mesh

def register_model_to_scene(model_pcd, scene_pcd, voxel_size=0.005):
    """ICP 点云配准 - 将模型对齐到场景中的物体"""
    print(f"\n🔄 ICP 点云配准...")

    source_down = model_pcd.voxel_down_sample(voxel_size)
    target_down = scene_pcd.voxel_down_sample(voxel_size)

    source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        source_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100)
    )
    target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        target_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100)
    )

    distance_threshold = voxel_size * 1.5
    result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        4,
        [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
         o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
        o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500)
    )

    print(f"   RANSAC 粗配准 fitness: {result_ransac.fitness:.4f}")

    result_icp = o3d.pipelines.registration.registration_icp(
        source_down, target_down,
        distance_threshold,
        result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )

    print(f"   ICP 精配准 fitness: {result_icp.fitness:.4f}")
    print(f"   均方误差: {result_icp.inlier_rmse:.6f}")

    return result_icp.transformation, result_icp.fitness

# ============================================================
# 第7部分：抓取位姿生成
# ============================================================

def generate_grasp_poses(cluster_pcd, features, gripper_width=0.08, approach_dist=0.15):
    """基于物体几何特征生成候选抓取位姿"""
    print(f"\n🤖 生成抓取位姿...")

    centroid = features['centroid']
    axes = features['principal_axes']
    extent = features['obb_extent']

    grasp_poses = []

    # 策略1: 沿最短轴抓取
    min_axis_idx = np.argmin(extent)
    grasp_axis = axes[:, min_axis_idx]

    other_axes = [axes[:, i] for i in range(3) if i != min_axis_idx]

    for approach_axis in other_axes:
        if approach_axis[2] < 0:
            approach_axis = -approach_axis

        z_axis = approach_axis / np.linalg.norm(approach_axis)
        x_axis = grasp_axis / np.linalg.norm(grasp_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)

        rotation = np.column_stack([x_axis, y_axis, z_axis])

        pre_grasp_pos = centroid - z_axis * approach_dist
        pre_grasp_pose = (pre_grasp_pos, rotation)

        grasp_pos = centroid - z_axis * (extent[min_axis_idx] / 2 + 0.02)
        grasp_pose = (grasp_pos, rotation)

        grasp_poses.append({
            'pre_grasp': pre_grasp_pose,
            'grasp': grasp_pose,
            'approach_axis': approach_axis,
            'grasp_axis': grasp_axis
        })

    # 策略2: 顶部抓取（低矮物体）
    if extent[2] < 0.05:
        top_approach = np.array([0, 0, 1])
        horizontal_extent = extent[:2]
        if horizontal_extent[0] > horizontal_extent[1]:
            x_axis = np.array([1, 0, 0])
        else:
            x_axis = np.array([0, 1, 0])

        y_axis = np.cross(top_approach, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, top_approach)

        rotation = np.column_stack([x_axis, y_axis, top_approach])

        pre_grasp_pos = centroid + np.array([0, 0, approach_dist])
        grasp_pos = centroid + np.array([0, 0, extent[2]/2 + 0.02])

        grasp_poses.append({
            'pre_grasp': (pre_grasp_pos, rotation),
            'grasp': (grasp_pos, rotation),
            'approach_axis': top_approach,
            'grasp_axis': x_axis,
            'type': 'top_grasp'
        })

    print(f"   生成 {len(grasp_poses)} 个候选位姿")
    return grasp_poses

def visualize_grasp_pose(pcd, grasp_pose, gripper_width=0.08, gripper_depth=0.12):
    """可视化抓取位姿（绘制夹爪框架）"""
    pos, rot = grasp_pose

    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.05, origin=pos
    )
    frame.rotate(rot, center=pos)

    left_finger = o3d.geometry.TriangleMesh.create_box(
        width=gripper_width/2, height=0.02, depth=gripper_depth
    )
    left_finger.translate(pos + rot[:, 0] * gripper_width/4 - rot[:, 2] * gripper_depth/2)
    left_finger.rotate(rot, center=pos)
    left_finger.paint_uniform_color([0.8, 0.2, 0.2])

    right_finger = o3d.geometry.TriangleMesh.create_box(
        width=gripper_width/2, height=0.02, depth=gripper_depth
    )
    right_finger.translate(pos - rot[:, 0] * gripper_width/4 - rot[:, 2] * gripper_depth/2)
    right_finger.rotate(rot, center=pos)
    right_finger.paint_uniform_color([0.8, 0.2, 0.2])

    return [frame, left_finger, right_finger]

# ============================================================
# 第8部分：完整抓取流水线
# ============================================================

def full_grasp_pipeline(pcd_path, voxel_size=0.005, visualize_steps=False):
    """
    完整的点云处理 → 物体分割 → 抓取位姿生成流水线

    Args:
        pcd_path: 点云文件路径
        voxel_size: 体素大小（米）
        visualize_steps: 是否显示中间步骤的可视化窗口

    Returns:
        results: 包含所有物体信息和抓取位姿的字典列表
    """
    print("=" * 60)
    print("🚀 启动完整抓取流水线")
    print("=" * 60)

    # 1. 加载点云
    pcd = load_point_cloud(pcd_path)
    if visualize_steps:
        visualize(pcd, "原始点云")

    # 2. 预处理
    pcd_processed = preprocess_point_cloud(pcd, voxel_size)
    if visualize_steps:
        visualize(pcd_processed, "预处理后点云")

    # 3. 平面分割（去除桌面）
    # 注意：如果点云坐标是mm级别，需要调整阈值
    plane_cloud, object_cloud, plane_model = segment_plane(pcd_processed, distance_threshold=0.01)
    if visualize_steps:
        visualize([plane_cloud, object_cloud], "平面分割结果")

    # 4. 物体聚类
    # 根据点云密度自动调整 eps
    # 如果点云坐标是mm级别（如 x: -4805~5088），eps 应该用米为单位
    # 但如果坐标值很大（如4805mm），说明单位可能是mm，需要确认
    # 这里先尝试用较小的 eps
    clusters, labels = cluster_objects(object_cloud, eps=voxel_size*4, min_points=50)

    # 如果没检测到物体，尝试更大的 eps
    if len(clusters) == 0:
        print("\n⚠️ 第一次聚类失败，尝试增大 eps...")
        clusters, labels = cluster_objects(object_cloud, eps=voxel_size*8, min_points=30)

    if visualize_steps and len(clusters) > 0:
        visualize(object_cloud, "聚类结果")

    # 5. 对每个物体提取特征并生成抓取位姿
    results = []
    all_geometries = [plane_cloud]

    for i, cluster in enumerate(clusters):
        print(f"\n📦 处理物体 {i+1}/{len(clusters)}...")

        features = get_object_features(cluster)
        print(f"   质心: ({features['centroid'][0]:.3f}, {features['centroid'][1]:.3f}, {features['centroid'][2]:.3f})")
        print(f"   尺寸: {features['aabb_extent']}")

        grasp_poses = generate_grasp_poses(cluster, features)

        cluster.paint_uniform_color([0, 0.8, 0])
        obb = draw_bounding_box(cluster, features)
        all_geometries.extend([cluster, obb])

        if grasp_poses:
            grasp_viz = visualize_grasp_pose(cluster, grasp_poses[0]['grasp'])
            all_geometries.extend(grasp_viz)

        results.append({
            'cluster': cluster,
            'features': features,
            'grasp_poses': grasp_poses
        })

    if visualize_steps and len(clusters) > 0:
        visualize(all_geometries, "完整结果")

    print("\n" + "=" * 60)
    print(f"✅ 流水线完成！检测到 {len(results)} 个物体")
    print("=" * 60)

    return results

# ============================================================
# 第9部分：与G2相机数据对接
# ============================================================

def g2_depth_to_open3d_pcd(depth_img, intrinsic, max_depth=5000):
    """将G2深度图转换为Open3D点云"""
    fx, fy, cx, cy = intrinsic
    height, width = depth_img.shape

    u = np.arange(width)
    v = np.arange(height)
    u, v = np.meshgrid(u, v)

    z = depth_img.astype(np.float32) / 1000.0

    valid_mask = (z > 0.1) & (z < max_depth / 1000.0)

    u = u[valid_mask]
    v = v[valid_mask]
    z = z[valid_mask]

    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    points = np.stack([x, y, z], axis=-1)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    return pcd

def g2_grasp_pipeline(depth_img, intrinsic, voxel_size=0.005):
    """从G2深度图直接到抓取位姿的完整流水线"""
    pcd = g2_depth_to_open3d_pcd(depth_img, intrinsic)

    temp_path = "/tmp/g2_scene.ply"
    o3d.io.write_point_cloud(temp_path, pcd)

    results = full_grasp_pipeline(temp_path, voxel_size, visualize_steps=False)

    return results
