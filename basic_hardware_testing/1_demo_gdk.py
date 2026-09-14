import agibot_gdk

print("✅ agibot_gdk 导入成功！")

# 列出前20个可用接口
members = [name for name in dir(agibot_gdk) if not name.startswith('_')]
print(f"可用接口数量: {len(members)}")
print(f"前20个: {members[:20]}")

# 测试创建 Robot 对象
robot = agibot_gdk.Robot()
print("✅ Robot 对象创建成功！")