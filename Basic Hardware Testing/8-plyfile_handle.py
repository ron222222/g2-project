# 新建一个文件，比如 test_grasp.py，写入：
from open3d_grasp_tutorial import full_grasp_pipeline

results = full_grasp_pipeline(
    "g2_point_cloud.ply",      # 你的点云文件路径
    voxel_size=0.005,
    visualize_steps=True        # 会弹出可视化窗口
)

for i, obj in enumerate(results):
    print(f"物体{i}: 质心={obj['features']['centroid']}")
    print(f"  抓取位姿: {len(obj['grasp_poses'])} 个")