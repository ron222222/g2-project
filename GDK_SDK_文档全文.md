# 智元 G2 GDK SDK 文档（离线全量）

## 目录

- [GDK 开发手册](404.md)
- [关节错误码](contents_appendices_joint_error_code.md)
- [Media_Manager](contents_appendices_media_manager.md)
- [混合部署](contents_concepts_hybrid_deploy.md)
- [传感器术语](contents_concepts_sensors.md)
- [TF坐标变换](contents_concepts_transform.md)
- [C++](contents_getting-started_quick-start_cpp.md)
- [Quick start](contents_getting-started_quick-start.md)
- [Python](contents_getting-started_quick-start_python.md)
- [Ros2](contents_getting-started_quick-start_ros2.md)
- [C++](contents_howto_C++.md)
- [部署](contents_howto_deploy.md)
- [如何使用](contents_howto.md)
- [mode](contents_howto_mode.md)
- [时间同步](contents_howto_ptp.md)
- [python](contents_howto_python.md)
- [ros2](contents_howto_ros2.md)
- [Camera](contents_reference_C++_camera.md)
- [Common](contents_reference_C++_common.md)
- [Env](contents_reference_C++_env.md)
- [IMU](contents_reference_C++_imu.md)
- [Interaction](contents_reference_C++_interaction.md)
- [Lidar](contents_reference_C++_lidar.md)
- [Map](contents_reference_C++_map.md)
- [PNC](contents_reference_C++_pnc.md)
- [Robot](contents_reference_C++_robot.md)
- [SLAM](contents_reference_C++_slam.md)
- [TF](contents_reference_C++_tf.md)
- [UltrasonicRadar](contents_reference_C++_uss.md)
- [Camera](contents_reference_Python_camera.md)
- [Common](contents_reference_Python_common.md)
- [Env](contents_reference_Python_env.md)
- [IMU](contents_reference_Python_imu.md)
- [Interaction](contents_reference_Python_interaction.md)
- [Lidar](contents_reference_Python_lidar.md)
- [Map](contents_reference_Python_map.md)
- [PNC](contents_reference_Python_pnc.md)
- [Robot](contents_reference_Python_robot.md)
- [SLAM](contents_reference_Python_slam.md)
- [TF](contents_reference_Python_tf.md)
- [Types](contents_reference_Python_types.md)
- [UltrasonicRadar](contents_reference_Python_uss.md)
- [Camera](contents_reference_ROS2_camera_node.md)
- [Control](contents_reference_ROS2_control_node.md)
- [IMU](contents_reference_ROS2_imu_node.md)
- [Lidar](contents_reference_ROS2_lidar_node.md)
- [Odom](contents_reference_ROS2_odom_node.md)
- [TF](contents_reference_ROS2_tf_node.md)
- [错误处理](contents_reference_error_handling.md)
- [开发规范](contents_reference_guidelines.md)
- [接口文档](contents_reference.md)
- [Web相机查看器 (camera_web_viewer.py)](contents_tools_camera_web_viewer.md)
- [关节控制工具 (mc_example.py)](contents_tools_mc_example.md)
- [What is GDK for?](root.md)



---

<!-- source: 404 -->

# GDK 开发手册

# 404 - Not found



---

<!-- source: contents_appendices_joint_error_code -->

# 关节错误码

# 关节错误码

## 头部和腰部关节错误码

### 适用关节

以下错误码适用于所有头部和腰部关节：

- `idx01_body_joint1`
- `idx02_body_joint2`
- `idx03_body_joint3`
- `idx04_body_joint4`
- `idx05_body_joint5`
- `idx11_head_joint1`
- `idx12_head_joint2`
- `idx13_head_joint3`

### 错误码列表

| 故障码（十六进制） | 故障码（十进制） | 含义 | 原因 |
| --- | --- | --- | --- |
| `0x00000001` | 1 | FOC错误 | - |
| `0x00000002` | 2 | 过压 | 母线电压超过设置阈值 |
| `0x00000004` | 4 | 欠压 | 母线电压超过设置阈值 |
| `0x00000008` | 8 | 过温 | 温度传感器读取线圈温度>110° |
| `0x00000010` | 16 | 启动失败 | - |
| `0x00000020` | 32 | 编码器错误 | 绝对值编码器数据异常 |
| `0x00000040` | 64 | 过流/过载 | - |
| `0x00000080` | 128 | 软件错误 | - |
| `0x00000100` | 256 | 温度传感器错误 | - |
| `0x00000200` | 512 | 位置超限错误 | 电机当前角度超过设置的正负限位 |
| `0x00000400` | 1024 | 关节ID非法 | - |
| `0x00000800` | 2048 | 位置跟踪错误 | - |
| `0x00001000` | 4096 | 电流检测错误 | - |
| `0x00002000` | 8192 | 抱闸打开失败 | - |
| `0x00004000` | 16384 | 位置指令阶跃警告 | 上一次目标指令和当前目标指令超过100cnt |
| `0x00008000` | 32768 | 多圈关节丢圈数 | - |
| `0x0000F000` | 61440 | 通信丢帧 | 1、从站执行器CANFD信号发送超时5次 2、硬件错误中断超过5次 |
| `0x00010000` | 65536 | 堵转 | 1、实际速度小于20RPM 2、实际电流维持在设置峰值电流2S |
| `0x00020000` | 131072 | 超速 | - |
| `0x00040000` | 262144 | 主机丢失(心跳掉线) | - |
| `0x00080000` | 524288 | 速度误差过大 | - |

## 手臂关节错误码

### 适用关节

以下错误码适用于所有手臂关节：

- `idx21_arm_l_joint1`
- `idx22_arm_l_joint2`
- `idx23_arm_l_joint3`
- `idx24_arm_l_joint4`
- `idx25_arm_l_joint5`
- `idx26_arm_l_joint6`
- `idx27_arm_l_joint7`
- `idx61_arm_r_joint1`
- `idx62_arm_r_joint2`
- `idx63_arm_r_joint3`
- `idx64_arm_r_joint4`
- `idx65_arm_r_joint5`
- `idx66_arm_r_joint6`
- `idx67_arm_r_joint7`

### 错误码列表

| 故障码（十六进制） | 故障码（十进制） | 含义 | 原因 |
| --- | --- | --- | --- |
| `0x2250` | 8784 | 驱动器短路 | 1.驱动器UVW输出线缆发生短路或对地短路 2.电机UVW发生短路或对地短路 3.驱动器内部发生短路或对地短路 4.驱动器受干扰导致误报此故障 |
| `0x2310` | 8976 | U相输出电流过大 | 1.电流环调节器参数设置不合理,导致电流控制振荡 2.电机参数设置错误 3.驱动器内部电流采样电路异常 |
| `0x2311` | 8977 | V相输出电流过大 | 1.电流环调节器参数设置不合理,导致电流控制振荡 2.电机参数设置错误 3.驱动器内部电流采样电路异常 |
| `0x2312` | 8978 | W相输出电流过大 | 1.电流环调节器参数设置不合理,导致电流控制振荡 2.电机参数设置错误 3.驱动器内部电流采样电路异常 |
| `0x2320` | 8992 | 驱动器硬件过流 | 1.电机负载过大或电机加速度、减速度设置过大,加减速时间设置过小 2.磁极相位角设置值有偏差，同时不满足磁极定位错误检出条件 3.编码器反馈有异常跳变 4.电流环调节器参数设置不合理，导致电流控制振荡 5.电机参数设置错误(线电阻、线电感、反电动势和转子惯量等) 6.驱动器内部电流检测电路异常，或驱动器抱闸电路损坏,无24V输出 7.电机抱闸损坏 8.转矩偏移值或静态平衡补偿值设置不合理 |
| `0x2330` | 9008 | 驱动器输出对地短路 | 1.驱动器UVW输出线缆发生对地短路 2.电机UVW发生对地短路 3.驱动器内部发生短路或对地短路 |
| `0x3130` | 12592 | 主电源输入异常 | 1.驱动器动力输入电源接线不良 2.驱动器功率回路设定选择为三相输入,但实际动力电源输入为单相 3.前端使用电子变压器,电子变压器谐波异常 |
| `0x3210` | 12816 | 直流母线过压 | 1.驱动器动力输入电源电压过大 2.电机快速停止时的能耗制动能量过大 3.能耗制动电阻未接或接线错误 4.能耗制动电阻阻值过大 5.驱动器内部异常 |
| `0x3220` | 12832 | 直流母线欠压 | 1.驱动器动力输入电源电压过低 2.驱动器内部电压采样电路异常 3.驱动器功率回路设置错误,220V供电设置为380V供电 4.驱动器动力输入电源线断开 |
| `0x4210` | 16912 | 功率模块过热 | 1.电机负载过大 2.驱动器内部温度采样电路异常 3.驱动器运行环境温度超出了允许工作范围 |
| `0x6010` | 24592 | CPU1看门狗溢出 | 驱动器内部异常 |
| `0x6011` | 24593 | CPU2看门狗溢出 | 驱动器内部异常 |
| `0x7112` | 28946 | 能耗制动电阻过载 | 1.电机频繁进行快速停止操作导致能耗制动能量过大 2.能耗制动电阻功率设置与实际电阻不一致 |
| `0x8311` | 33553 | 电机持续过载 | 1.电机负载过大 2.电机加减速时间设置过小 3.电机参数设置错误 4.抱闸释放动作异常 5.电机选型错误,功率过小(比如大功率驱动器,带载小功率电机长时间满载高速运行) 6.驱动器内部电流采样电路异常 |
| `0x8611` | 34321 | 位置跟随误差过大 | 1.电机负载过大 2.控制参数不合适 3.抱闸释放动作异常 4.位置跟随误差过大判定阈值或判定时间设施过小 |
| `0x8612` | 34322 | 正向软限位 | 位置反馈值超过（正向软限位值+定位完成阈值） |
| `0x8613` | 34323 | 负向软限位 | 位置反馈值超过（负向软限位值-定位完成阈值） |
| `0x8800` | 34816 | 第一编码器数据溢出 | 位置模式下，未使能无限位置控制时，编码器多圈值超出了实际编码器的多圈位数。 |
| `0x8801` | 34817 | 第二编码器数据溢出 | 位置模式下，未使能无限位置控制时，编码器多圈值超出了实际编码器的多圈位数。 |
| `0xFF00` | 65280 | CPU1工作异常 | 1.驱动器固件运行异常 2.驱动器内部异常 |
| `0xFF01` | 65281 | CPU2工作异常 | 1.驱动器固件运行异常 2.驱动器内部异常 |
| `0xFF02` | 65282 | CPU1内存异常 | 1.驱动器固件运行异常 2.驱动器内部异常 |
| `0xFF03` | 65283 | CPU2内存异常 | 1.驱动器固件运行异常 2.驱动器内部异常 |
| `0xFF04` | 65284 | CPU内存冲突 | 1.驱动器固件运行异常 2.驱动器内部异常 |
| `0xFF05` | 65285 | 磁极定位错误 | 1.电机磁极相位角设定值与检测值不一致 2.磁极定位故障检测灵敏度设置过小 3.静态平衡力矩补偿值设置与实际负载不一致。 4.电机接线错误导致磁极相位角发生改变。 5.电机异常导致磁极相位角发生改变。 6.重力负载导致伺服使能瞬间电机发生转动，速度超过磁极定位故障检测灵敏度设置阈值 |
| `0xFF06` | 65286 | 第一编码器数据异常 | 1.编码器本身数据发生异常 2.编码器线缆线序错误或接触不良 3.由于噪音干扰导致编码器数据异常 |
| `0xFF07` | 65287 | 第一编码器通信异常 | 1.编码器本身发生异常 2.编码器线缆线序错误或接触不良 3.由于噪音干扰导致编码器数据异常 |
| `0xFF08` | 65288 | 第一编码器通信超时 | 1.编码器本身发生异常 2.编码器线缆线序错误或接触不良 3.由于噪音干扰导致编码器数据异常 |
| `0xFF09` | 65289 | 第一编码器内部异常1 | 编码器内部状态异常 |
| `0xFF10` | 65296 | 第二编码器数据异常 | 1.编码器本身数据发生异常 2.编码器线缆线序错误或接触不良 3.由于噪音干扰导致编码器数据异常 |
| `0xFF11` | 65297 | 第二编码器通信异常 | 1.编码器本身发生异常 2.编码器线缆线序错误或接触不良 3.由于噪音干扰导致编码器数据异常 |
| `0xFF12` | 65298 | 第二编码器通信超时 | 1.编码器本身发生异常 2.编码器线缆线序错误或接触不良 3.由于噪音干扰导致编码器数据异常 |
| `0xFF13` | 65299 | 第二编码器内部异常1 | 编码器内部状态异常 |
| `0xFF14` | 65300 | 控制编码器超速 | 1 编码器位置反馈值在一个位置采样周期内的变化量过大，超过电机最高转速的1.3倍。 2 编码器异常 3.由于噪音干扰导致编码器数据异常 |
| `0xFF15` | 65301 | 驱动器持续过载 | 1.电机负载过大或加减速时间设置过小 2.实际机械负载过大或存在卡死现象 3.电机抱闸未释放 4.电机或电机抱闸异常 5.驱动器内部异常 |
| `0xFF16` | 65302 | 驱动器瞬时过载 | 1.电机瞬时负载过大 2.驱动器内部温度采样电路异常 3.驱动器运行环境温度超出了允许工作范围 4.由于噪音干扰导致功率模块温度采样异常 |
| `0xFF17` | 65303 | 驱动器输出缺相 | 1.驱动器U,V,W输出存在断线或接线不良等现象 2.电机阻抗过大 3.驱动器内部电流采样电路异常 |
| `0xFF18` | 65304 | 电机失速 | 1.驱动器调节器参数设置不合理导致速度跟踪超调过大 2.由于电磁噪音干扰导致编码器数据异常变化 3.由于编码器损坏导致编码器数据异常变化 4.驱动器内部电路异常 |
| `0xFF19` | 65305 | 协处理器通讯异常 | 1.编码器线缆线序错误或接触不良 2.由于噪音干扰导致编码器数据异常 |
| `0xFF20` | 65312 | 编码器AB信号变化异常 | 1.编码器线缆线序错误或接触不良 2.由于噪音干扰导致编码器数据异常 |
| `0xFF21` | 65313 | 电流跟随误差过大 | 1.驱动器调节器参数设置不合理导致速度跟踪超调过大 2.由于电磁噪音干扰导致编码器数据异常变化 3.由于电机损坏而导致电流反馈异常变化 4.驱动器内部电路异常 |
| `0xFF22` | 65314 | 位置目标值异常 | 1.CSP模式下，伺服使能瞬间，位置目标值和位置实际值之差超过位置跟随误差过大设定阈值 2.CSP模式下，电机运行过程中，目标轨迹加速度超过参数 最大加速度设定阈值，且位置目标值和位置实际值之差超过位置跟随误差过大设定阈值 |
| `0xFF23` | 65315 | 第一编码器上电数据溢出 | 驱动器上电时反馈位置值超出了编码器允许的最大范围 |
| `0xFF24` | 65316 | 位置目标值溢出 | 位置模式下，当禁止无限位置控制时，位置目标值超出了允许的最大范围 |
| `0xFF25` | 65317 | 电机抱闸异常 | 1.电机抱闸本身发生异常，不能正常制动。 2.电机在高速运行时突然伺服OFF，制动时间过长。 3.电机抱闸制动时间设置值小于抱闸实际制动动作时间。 4.转子定位故障检测灵敏度设置过小 |
| `0xFF26` | 65318 | 控制电源欠压 | 1.24V控制电源异常 2.24V控制电源接线错误,或接触不良 3.24V控制电源负载过大 4.驱动器内部电路异常 |
| `0xFF27` | 65319 | STO1触发 | STO1触发或接线不良 |
| `0xFF28` | 65320 | STO2触发 | STO2触发或接线不良 |
| `0xFF29` | 65321 | 正向硬限位开关触发 | 单方向运行至机械限位，导致硬件限位触发 |
| `0xFF30` | 65328 | 负向硬限位开关触发 | 单方向运行至机械限位，导致硬件限位触发 |
| `0xFF31` | 65329 | 电机超速 | 1 电机实际速度反馈值超出电机最高转速的1.1倍。 2 编码器异常 |
| `0xFF32` | 65330 | 急停输入开关触发 | 急停输入开关触发或接线不良 |
| `0xFF33` | 65331 | 转矩监测饱和故障 | 1.电机负载过大，超过了转矩监测告警阈值 2.转矩饱和监测阈值设置过小 |
| `0xFF34` | 65332 | 速度跟随误差过大 | 1.电机负载过大 2.控制参数不合适 3.抱闸释放动作异常 4.速度跟随误差过大判定阈值或判定时间设置过小 |
| `0xFF35` | 65333 | 全闭环位置偏差过大 | 1.电机负载过大 2.控制参数不合适 3.抱闸释放动作异常 4.全闭环位置跟随误差过大判定阈值设置过小 |
| `0xFF36` | 65334 | 寻原点失败 | 1.寻原点参数设置不合理 2.寻原点启动时,电机已处于限位开关触发状态 3.寻原点过程中切换至非HM 模式 |
| `0xFF37` | 65335 | EtherCAT过程数据错误 | PDO设定值超出了对象允许范围 |
| `0xFF38` | 65336 | EtherCAT总线指令非法 | EtherCAT 通讯状态机与控制字时序配合错误 |
| `0xFF39` | 65337 | EtherCAT通讯周期错误 | 1.EtherCAT 通讯周期小于伺服控制周期 2.EtherCAT通讯周期设置不是250us的2的整数次幂 |
| `0xFF40` | 65344 | 位置规划运行错误 | 1.EtherCAT控制权下，运行PP模式时，位置目标值缓存超限 2.驱动器内部异常 |
| `0xFF41` | 65345 | EtherCAT非法同步模式 | 1.EtherCAT通信DC模式配置错误 2.EtherCAT通信未激活DC模式 |
| `0xFF42` | 65346 | 位置目标值超出设定范围 | 模数无限位置控制模式或者禁止无限位置控制模式时，位置目标值超出了允许设定范围 |
| `0xFF43` | 65347 | 整流模块过热 | 1.驱动器内部温度采样电路异常 2.驱动器运行环境温度超出了允许工作范围 |
| `0xFF44` | 65348 | 散热器过热 | 1.驱动器内部温度采样电路异常 2.驱动器运行环境温度超出了允许工作范围 |
| `0xFF45` | 65349 | 电机U相瞬时过载 | 1.电机负载过大 2.电机加减速时间设置过小 3.电机参数设置错误 4.抱闸释放动作异常 5.电机选型错误,功率过小(比如大功率驱动器,带载小功率电机长时间满载高速运行) 6.驱动器内部电流采样电路异常 7.电机快速过载保护阈值和保护时间设置偏小 |
| `0xFF46` | 65350 | 电机V相瞬时过载 | 1.电机负载过大 2.电机加减速时间设置过小 3.电机参数设置错误 4.抱闸释放动作异常 5.电机选型错误,功率过小(比如大功率驱动器,带载小功率电机长时间满载高速运行) 6.驱动器内部电流采样电路异常 7.电机快速过载保护阈值和保护时间设置偏小 |
| `0xFF47` | 65351 | 电机W相瞬时过载 | 1.电机负载过大 2.电机加减速时间设置过小 3.电机参数设置错误 4.抱闸释放动作异常 5.电机选型错误,功率过小(比如大功率驱动器,带载小功率电机长时间满载高速运行) 6.驱动器内部电流采样电路异常 7.电机快速过载保护阈值和保护时间设置偏小 |
| `0xFF48` | 65352 | 动态制动过载 | 电机运行时，相邻2次动态制动停机间隔时间过短 |
| `0xFF49` | 65353 | 驱动器内部异常 | 驱动器内部异常 |
| `0xFF50` | 65360 | 限位开关异常 | 限位开关触发或接线不良 |
| `0xFF51` | 65361 | EtherCAT总线通讯异常 | 1.EtherCAT 通讯受到干扰 2.EtherCAT 网线断开或接触不良 3.上位机实时性不够 4.上位机 EtherCAT 主站底层 DC 同步机制与驱动器需求不匹配 5.驱动器内部异常 |
| `0xFF52` | 65362 | 接口编码器分辨率变更 | 更改了接口编码器分辨率 |
| `0xFF53` | 65363 | 第一编码器过热 | 1.编码器实际温度过高 2.编码器异常 |
| `0xFF54` | 65364 | 第一编码器电池欠电压故障 | 1.编码器电池电压过低 2.编码器电池接线接触不良 |
| `0xFF55` | 65365 | 第二编码器过热 | 1.编码器实际温度过高 2.编码器异常 |
| `0xFF56` | 65366 | 第二编码器电池欠电压故障 | 1.编码器电池电压过低 2.编码器电池接线接触不良 |
| `0xFF57` | 65367 | 控制模式设定错误 | 伺服ON时，控制模式设定为驱动器不支持的模式，如NM、VL或IP，或者EtherCAT控制权时控制模式设置为PV或PT |
| `0xFF58` | 65368 | 上电位置偏差过大 | 驱动器上电时，与上一次掉电保存位置不一致，超过设定阈值 |
| `0xFF59` | 65369 | 第一编码器加速度异常故障 | 1.编码器本身数据发生异常 2.编码器线缆线序错误或接触不良 3.由于噪音干扰导致编码器数据异常 |
| `0xFF60` | 65376 | 电机堵转 | 1.机械负载存在卡死现象或堵转现象 2.电机抱闸未释放 |
| `0xFF61` | 65377 | 电机过热 | 1.电机负载过大 2.电机运行环境温度过高，超过允许范围 3.电机过热保护热偶电阻值设置错误 4.电机温度传感器异常 5.驱动器内部异常 |
| `0xFF62` | 65378 | 增量式编码器Z信号异常 | 1.编码器本身数据发生异常 2.编码器线缆线序错误或接触不良 3.由于噪音干扰导致编码器数据异常 |
| `0xFF63` | 65379 | 写EEPROM数据异常 | 驱动器内部异常 |
| `0xFF64` | 65380 | 读EEPROM数据异常 | 驱动器内部异常 |
| `0xFF65` | 65381 | 能耗制动电路异常 | 伺服参数设置的能耗制动选择，与能耗制动电阻实际接线不一致 |
| `0xFF66` | 65382 | 抱闸控制电路异常 | 1.电机抱闸接线短路或接触不良 2.电机抱闸内部发生短路或接触不良 3.驱动器内部异常 |
| `0xFF67` | 65383 | CPU过热 | 1.驱动器内部温度采样电路异常 2.驱动器运行环境温度超出了允许工作范围 |
| `0xFF68` | 65384 | CPU1过载 | 1.驱动器运行受到噪音干扰 2.调试软件数据采集过大 3.驱动器内部电路异常 |
| `0xFF69` | 65385 | CPU2过载 | 1.驱动器运行受到噪音干扰 2.调试软件数据采集过大 3.驱动器内部电路异常 |
| `0xFF70` | 65392 | CPU1握手失败 | 1.驱动器固件运行异常 2.驱动器内部异常 |
| `0xFF71` | 65393 | DriveMaster通讯超时 | 1.驱动器调试线缆断开或接触不良 2.驱动器调试串口通讯受到干扰 |
| `0xFF72` | 65394 | 蓝牙通讯异常 | 驱动器内部异常 |
| `0xFF73` | 65395 | 高速同步通讯异常 | 1.高速龙门同步通信线接线错误或这接触不良 2.由于噪音干扰导致高速龙门通信异常 3.驱动器内部异常 |
| `0xFF74` | 65396 | TF卡异常 | 驱动器内部异常 |
| `0xFF75` | 65397 | TF卡通讯异常 | 驱动器内部异常 |
| `0xFF76` | 65398 | ESC内部访问错误 | 驱动器内部异常 |
| `0xFF77` | 65399 | 伺服使能未准备好 | 1.伺服ON时，驱动器处于实际电机且虚拟编码器模式 2.伺服ON时，编码器通信处于断开状态 3.伺服ON时，电机转速高于30rpm 4.伺服ON时，STO状态未解除 5.伺服ON时，直流母线电压过低，充电继电器未吸合 6.伺服ON时，动态制动状态未解除 7.驱动器内部异常 |
| `0xFF78` | 65400 | CPU2握手失败 | 1.驱动器固件运行异常 2.驱动器内部异常 |
| `0xFF79` | 65401 | CPU1主任务超时 | 1.驱动器运行受到噪音干扰 2.调试软件数据采集过大 3.驱动器内部电路异常 |
| `0xFF80` | 65408 | 主电源掉电 | 1.驱动器伺服ON状态下，发生了动力电源掉电 2.由于噪音干扰导致主电源掉电检测异常 3.伺服参数主电源掉电检测时间设置过小 |
| `0xFF81` | 65409 | 直流母线充电继电器异常 | 驱动器内部充电继电器发生异常 |
| `0xFF82` | 65410 | CPU内部错误 | 1.驱动器固件运行异常 2.驱动器内部异常 |
| `0xFF83` | 65411 | 位置实际值溢出 | 位置模式下，当禁止无限位置控制时，位置实际值超出了允许的最大范围 |
| `0xFF84` | 65412 | 第二编码器上电数据溢出 | 驱动器上电时反馈位置值超出了编码器允许的最大范围 |
| `0xFF85` | 65413 | 第一编码器内部异常2 | 编码器内部状态异常 |
| `0xFF86` | 65414 | 第二编码器内部异常2 | 编码器内部状态异常 |
| `0xFF87` | 65415 | 第一编码器内部异常3 | 编码器内部状态异常 |
| `0xFF88` | 65416 | 第二编码器内部异常3 | 编码器内部状态异常 |
| `0xFF89` | 65417 | 第二编码器加速度异常故障 | 1.编码器本身数据发生异常 2.编码器线缆线序错误或接触不良 3.由于噪音干扰导致编码器数据异常 |
| `0xFF8A` | 65418 | STO1电路诊断异常 | 1.STO1触发或接线不良 2.驱动器内部异常 |
| `0xFF8B` | 65419 | STO2电路诊断异常 | 1.STO2触发或接线不良 2.驱动器内部异常 |
| `0xFF8C` | 65420 | 霍尔信号异常 | 1.霍尔传感器本身信号发生异常 2.霍尔传感器接线线序错误或接触不良 3.由于噪音干扰导致霍尔信号异常 |
| `0xFF8D` | 65421 | 编码器霍尔-AB信号欠相异常 | 1.霍尔传感器或AB编码器本身信号发生异常 2.霍尔传感器或AB编码器接线线序错误或接触不良 3.由于噪音干扰导致霍尔或AB编码器信号异常 |
| `0xFF8E` | 65422 | 第2位置跟随误差过大 | 1.电机负载过大 2.控制参数不合适 3.抱闸释放动作异常 4.第2位置跟随误差过大判定阈值或判定时间设施过小 |
| `0xFF8F` | 65423 | STO接线异常 | STO1或STO2触发或接线不良 |
| `0xFF90` | 65424 | 龙门位置跟随误差过大 | 1.龙门主轴和从轴之间负载差异过大，导致两者之间位置误差超过了设定阈值 2.龙门主轴和从轴伺服驱动器或伺服电机规格不一致 |
| `0xFF91` | 65425 | 龙门通讯异常 | 1.高速龙门同步通信线接线错误或这接触不良 2.由于噪音干扰导致高速龙门通信异常 3.驱动器内部异常 |
| `0xFF92` | 65426 | 龙门控制异常 | 1.高速龙门同步通信线接线错误或这接触不良 2.由于噪音干扰导致高速龙门通信异常 3.驱动器内部异常 |
| `0xFF93` | 65427 | 第2速度跟随误差过大 | 1.电机负载过大 2.控制参数不合适 3.抱闸释放动作异常 4.第2速度跟随误差过大判定阈值或判定时间设施过小 |
| `0xE000` | 57344 | 控制电源欠压告警 | 1.24V控制电源异常 2.24V控制电源接线错误,或接触不良 3.24V控制电源负载过大 4.驱动器内部电路异常 |
| `0xE001` | 57345 | 环境温度过高 | 1.驱动器运行环境温度超出了允许工作范围 2.驱动器内部温度采样电路异常 |
| `0xE002` | 57346 | STO触发 | STO1或STO2触发或接线不良 |
| `0xE003` | 57347 | 转矩监测饱和告警 | 1.电机负载过大，超过了转矩监测告警阈值 2.转矩饱和监测阈值设置过小 |
| `0xE004` | 57348 | CPU1过载告警 | 1.驱动器运行受到噪音干扰 2.调试软件数据采集过大 3.驱动器内部电路异常 |
| `0xE005` | 57349 | CPU2过载告警 | 1.驱动器运行受到噪音干扰 2.调试软件数据采集过大 3.驱动器内部电路异常 |
| `0xE006` | 57350 | 更改了重上电有效参数 | 更改了重新上电有效的参数 |
| `0xE007` | 57351 | 急停开关触发 | 急停输入开关触发或接线不良 |
| `0xE008` | 57352 | 机械原点未标定 | 未标定机械原点 |
| `0xE009` | 57353 | 电机过载告警 | 1.电机负载过大 2.电机加减速时间设置过小 3.电机参数设置错误 4.抱闸释放动作异常 5.电机选型错误,功率过小(比如大功率驱动器,带载小功率电机长时间满载高速运行) 6.驱动器内部电流采样电路异常 |
| `0xE010` | 57360 | 转速限制告警 | 由于驱动器输入电源电压过低而导致电机转速被限制 |
| `0xE011` | 57361 | 直流母线欠压告警 | 1.驱动器动力输入电源电压过低 2.驱动器内部电压采样电路异常 3.驱动器功率回路设置错误,220V供电设置为380V供电 4.驱动器动力输入电源线断开 |
| `0xE012` | 57362 | 故障历史记录异常 | 1 历史故障记录异常 2 驱动器内部异常 |
| `0xE013` | 57363 | AD校正系数无效 | 1.驱动器内部异常 2.AD校正系数错误 |
| `0xE014` | 57364 | CoE通讯参数异常 | 驱动器内部异常 |
| `0xE015` | 57365 | 伺服参数恢复默认值 | 伺服参数恢复了默认值 |
| `0xE016` | 57366 | 不支持设定控制模式 | 伺服OFF时，控制模式设定为NM、VL或IP等不支持的模式 |
| `0xE017` | 57367 | 第一编码器电池欠电压告警 | 1.编码器电池电压过低 2.编码器电池接线接触不良 |
| `0xE018` | 57368 | 第二编码器电池欠电压告警 | 1.编码器电池电压过低 2.编码器电池接线接触不良 |
| `0xE019` | 57369 | 驱动器内部告警 | 驱动器内部异常 |
| `0xE020` | 57376 | 第一编码器通信异常告警 | 1.编码器本身发生异常 2.编码器线缆线序错误或接触不良 3.由于噪音干扰导致编码器数据异常 |
| `0xE021` | 57377 | 第二编码器通信异常告警 | 1.编码器本身发生异常 2.编码器线缆线序错误或接触不良 3.由于噪音干扰导致编码器数据异常 |
| `0xE022` | 57378 | 第一编码器通信超时告警 | 1.编码器本身发生异常 2.编码器线缆线序错误或接触不良 3.由于噪音干扰导致编码器数据异常 |
| `0xE023` | 57379 | 第二编码器通信超时告警 | 1.编码器本身发生异常 2.编码器线缆线序错误或接触不良 3.由于噪音干扰导致编码器数据异常 |
| `0xE024` | 57380 | 第一编码器数据异常告警 | 1.编码器本身发生异常 2.编码器线缆线序错误或接触不良 3.由于噪音干扰导致编码器数据异常 |
| `0xE025` | 57381 | 第二编码器数据异常告警 | 1.编码器本身发生异常 2.编码器线缆线序错误或接触不良 3.由于噪音干扰导致编码器数据异常 |
| `0xE026` | 57382 | 位置限位告警 | 单方向运行至机械限位，导致硬件限位触发 |
| `0xE027` | 57383 | 位置规划参数异常告警 | 1.规划速度，或者规划加速度，或者规划减速度设置为0 2.规划减速度过小，使得电机最高速度下的减速时间超出了受控停机允许时间规定的数值 |
| `0xE028` | 57384 | SDO写入失败告警 | 由于设定数值超出对象允许范围，导致SDO对象写入失败 |
| `0xE029` | 57385 | 寻原点配置错误 | 1.寻原点参数设置与实际编码器类型不匹配 2.未配置与设置的寻原点方法相匹配的DI信号 3.寻原点方法设置非CiA402标准规定的数值 |
| `0xE030` | 57392 | 编码器内部告警 | 1.编码器本身发生异常 2.编码器线缆线序错误或接触不良 3.由于噪音干扰导致编码器数据异常 |
| `0xE031` | 57393 | 能耗制动电阻过载告警 | 1.电机频繁进行快速停止操作导致能耗制动能量过大 2.能耗制动电阻功率设置与实际电阻不一致 |
| `0xE032` | 57394 | 动态制动停机超时告警 | 动态制动停机时，电机速度减速到受控停机完成速度阈值时间过长，超出了受控停机允许延迟时间。 |
| `0xE033` | 57395 | SD卡载入异常 | 驱动器内部异常 |
| `0xFFFF` | 65535 | EtherCat通信丢失 | soc无法识别到机械臂从站 |



---

<!-- source: contents_appendices_media_manager -->

# Media_Manager

# media\_manager 使用说明

## 这个工具是做什么的？

把您\*\*电脑上的文件或目录\*\*上传到交互板，或对交互板上的媒体资源进行查看、删除。

远端根目录固定为交互板的 `/home/agi/gdk_resources`，所有操作都限制在该目录内。

---

## 使用前您需要有什么？

1. **可执行文件**
   在开发机安装 GDK 后，可执行文件在该路径下：`~/.cache/agibot/app/gdk/tools/media_manager`
2. **网络**
   开发机与 G2 本体网线直连并可相互 ping 通。

---

## 可用命令总览

```
./media_manager put <本地路径>
./media_manager ls [远端路径]
./media_manager rm <远端路径>
```

远端路径（`ls` / `rm` 参数）可以是：
- **相对路径**：`media_pack/prompt.wav`（相对于 `/home/agi/gdk_resources`）
- **绝对路径**：`/home/agi/gdk_resources/media_pack/prompt.wav`

---

## put — 上传文件或目录

```
# 上传单个文件
./media_manager put /path/to/prompt.wav

# 上传整个目录
./media_manager put ~/media_pack

# 静默模式，不显示进度
./media_manager -q put ~/media_pack
```

上传后，本地 `~/media_pack` 会出现在交互板的 `/home/agi/gdk_resources/media_pack`。
同名文件默认覆盖，每个文件传输时实时刷新进度百分比。

---

## ls — 查看远端内容

```
# 列出 /home/agi/gdk_resources 下的内容
./media_manager ls

# 列出子目录内容（相对路径）
./media_manager ls audio

# 列出子目录内容（绝对路径）
./media_manager ls /home/agi/gdk_resources/audio

# 查看单个文件
./media_manager ls audio/prompt.wav
```

输出为绝对路径，每行一条，例如：

```
/home/agi/gdk_resources/audio/prompt.wav
/home/agi/gdk_resources/audio/demo.wav
```

---

## rm — 删除远端文件或目录

```
# 删除单个文件（相对路径）
./media_manager rm media_pack/old.wav

# 删除整个目录（递归删除）
./media_manager rm media_pack

# 删除单个文件（绝对路径）
./media_manager rm /home/agi/gdk_resources/media_pack/old.wav
```

- 目录会\*\*递归删除\*\*，无需加额外参数。
- \*\*不会\*\*弹出确认提示，执行即生效。
- 禁止删除根目录 `/home/agi/gdk_resources` 本身。

---

## 完整示例（上传 + 播放音视频）

### 场景

开发机上准备好目录 `~/media_pack`，内含：

- `prompt.wav`（提示音）
- `demo.mp4`（演示视频）

### 1）上传整个目录

```
./media_manager put ~/media_pack
```

传完后，交互板上的路径为：

- `/home/agi/gdk_resources/media_pack/prompt.wav`
- `/home/agi/gdk_resources/media_pack/demo.mp4`

### 2）用 ls 确认路径

```
./media_manager ls media_pack
```

输出示例：

```
/home/agi/gdk_resources/media_pack/demo.mp4
/home/agi/gdk_resources/media_pack/prompt.wav
```

### 3）调用 GDK 接口播放

将 `ls` 输出的绝对路径直接传给 GDK：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

interaction = agibot_gdk.Interaction()
time.sleep(1)

# 播放音频文件
try:
    interaction.play_audio("/home/agi/gdk_resources/media_pack/prompt.wav")
    print("音频播放成功")
    time.sleep(5)  # 等待播放完成
except Exception as e:
    print(f"播放音频失败: {e}")

agibot_gdk.gdk_release()
```

---

## 常见情况

| 现象 | 可能原因 | 建议 |
| --- | --- | --- |
| 提示找不到命令 | 当前目录不对，或没写 `./` | 先 `cd` 到程序所在目录，使用 `./media_manager`；或用\*\*绝对路径\*\*运行。 |
| 没有执行权限 | macOS / Linux 未赋予可执行权限 | 执行：`chmod +x media_manager` |
| 认证失败、无法连接 | 密码/密钥变更、账号权限变化 | 联系研发重新配置并\*\*重新编译\*\*发您新程序。 |
| 传一半卡住 | 网络不稳定或文件很大 | 稍等或检查网络；大文件多等一段时间。 |
| rm 报"禁止操作根目录" | 路径解析后等于 `/home/agi/gdk_resources` | 检查路径参数是否填写了具体文件或子目录。 |
| 路径被拒绝（逃逸） | 路径含 `..` 或非 `/home/agi/gdk_resources` 开头的绝对路径 | 只能操作 `/home/agi/gdk_resources` 内的路径。 |



---

<!-- source: contents_concepts_hybrid_deploy -->

# 混合部署

# 混合部署

GDK 采用混合部署模式：用户代码运行在开发机（用户侧），通过有线网络调用机器人本体上的 GDK 服务（机器人端）。

混合部署需要将机器人和开发机用网线直连，并给开发机配置静态 IP，使两端可以互相通信。G02 的机器人 IP 为 `10.42.1.101`，开发机 IP 通常配置为 `10.42.1.102`。

## 用户侧

英文 Client Side。开发者的个人电脑或工作站。用户编写的应用代码与 GDK SDK 运行在这里。用户侧无需安装硬件驱动，所有对机器人的操作通过网络发送到机器人端执行。文档中「用户侧」与「客户端」含义相同。

## 机器人端

英文 Robot Side。机器人本体上的板载计算机。GDK 服务、硬件驱动及底层控制程序运行在这里。机器人端负责接收来自用户侧的指令并直接操控硬件。文档中「机器人端」与「端侧」含义相同。



---

<!-- source: contents_concepts_sensors -->

# 传感器术语

# 传感器术语

GDK 接口文档和示例代码中频繁出现以下传感器类型名称。

## RGBD 相机

英文 RGB-D Camera。同时输出彩色图（RGB）和深度图（Depth）的相机。深度值表示每个像素到相机的距离，通常通过结构光或 ToF（飞行时间）技术实现。GDK 中 RGB-D 相机安装于机器人头部和手部，用于近距离感知与抓取任务。

## 立体相机

英文 Stereo Camera。由两个固定基线距离的镜头组成，通过左右图像的视差计算深度。GDK 中立体相机用于头部的远距离三维感知。

## 鱼眼相机

英文 Fisheye Camera。超广角镜头，视角可达 180° 以上，以较大的图像畸变换取宽广的视野范围。GDK 中鱼眼相机安装于头部，用于环境全景感知。

## 激光雷达

英文 LiDAR。通过激光测距获取环境三维点云，精度高且不受光照影响。GDK 中激光雷达安装于底盘，用于 SLAM 建图与自主导航。

## IMU

英文 Inertial Measurement Unit，惯性测量单元。测量线性加速度和角速度的传感器，用于姿态估计和运动补偿。GDK 中 IMU 的线性加速度单位为 g（重力加速度倍数），静止时 z 轴约为 1.0 g；角速度单位为 rad/s。

## USS

英文 Ultrasonic Sensor，超声波传感器。通过声波测距检测近距离障碍物，用于底盘的近距离避障补充感知。



---

<!-- source: contents_concepts_transform -->

# TF坐标变换

# TF (Transform) 坐标变换

## 什么是TF

TF即transform，坐标转换。GDK中，TF(transform)模块提供各个坐标系之间的转换关系。包含机器人上各个坐标系之间的转换关系以及各传感器之间的转换关系。

在机器人系统中，有三个常见的坐标系：
- 基坐标系：固定在机器人底座或工作环境中的坐标系，通常作为机器人运动和定位的全局参考。
- 相机坐标系：固定在相机上的坐标系，用来描述目标在相机视角下的位置和方向。
- 末端坐标系：固定在机器人末端执行器上的坐标系，用来描述夹爪、工具或执行器的位姿。



---

<!-- source: contents_getting-started_quick-start_cpp -->

# C++

# C++

## 安装GDK

将开发机与G02机器人通过网线直连，配置开发机的静态IP为10.42.1.102。确保ping 10.42.1.101通信正常。

在开发机环境安装GDK依赖：

```
curl -sSL http://10.42.1.101:8849/install.sh | bash
```

## 通过CMakeLists编译

安装cmake

```
sudo apt install cmake
```

编译

```
cd ~/.cache/agibot/app/gdk/examples/cpp
cmake -B build
make -C build
```

## 运行机器人运控示例

```
source ~/.cache/agibot/app/env.sh
./build/mc_example joint_name1 pos1 joint_name2 pos2 ...
```

joint\_name为机器人关节名称，pos为关节目标位置，单位为弧度。

joint\_name列表如下：

```
"idx01_body_joint1", #腰部五自由度（从底部往上）
"idx02_body_joint2",
"idx03_body_joint3",
"idx04_body_joint4",
"idx05_body_joint5",
"idx11_head_joint1", #头部三自由度（r,p,y）
"idx13_head_joint3",
"idx12_head_joint2",
"idx21_arm_l_joint1", #左手臂七自由度（从肩部往下）
"idx22_arm_l_joint2",
"idx23_arm_l_joint3",
"idx24_arm_l_joint4",
"idx25_arm_l_joint5",
"idx26_arm_l_joint6",
"idx27_arm_l_joint7",
"idx61_arm_r_joint1", #右手臂七自由度（从肩部往下）
"idx62_arm_r_joint2",
"idx63_arm_r_joint3",
"idx64_arm_r_joint4",
"idx65_arm_r_joint5",
"idx66_arm_r_joint6",
"idx67_arm_r_joint7",
```



---

<!-- source: contents_getting-started_quick-start -->

# Quick start

# 概览

GDK（Genie Development Kit）是为Genie机器人提供的开发工具包，旨在帮助开发者和用户高效实现机器人功能的二次开发与集成。

GDK支持多种主流开发语言（C++、Python、ROS2），并提供丰富的接口，方便用户快速调用机器人的运动控制、传感器数据获取、建图、移动等底层能力。同时，GDK还支持与大模型的高效对接，助力机器人在教育、科研、工业、服务等多种场景下的应用落地。

要开始使用GDK，请根据你的开发语言选择相应的入门指南：

- [C++快速上手](cpp/)
- [Python快速上手](python/)
- [ROS2快速上手](ros2/)



---

<!-- source: contents_getting-started_quick-start_python -->

# Python

# Python

## 安装

将开发机与G02机器人通过网线直连，配置开发机的静态IP为10.42.1.102。确保ping 10.42.1.101通信正常。

在开发机环境安装GDK依赖：

```
curl -sSL http://10.42.1.101:8849/install.sh | bash
```

## 运行示例

### 声明python路径

```
source ~/.cache/agibot/app/env.sh
```

### 运行运控示例代码

```
cd ~/.cache/agibot/app/gdk/examples/python
python3 mc_example.py
```

#### 操作说明

| 按键 | 功能 |
| --- | --- |
| a/d | 切换关节 |
| w/s | 调整关节角度 |
| m | 切换动作序列 |
| p | 播放动作序列 |
| q | 退出程序 |



---

<!-- source: contents_getting-started_quick-start_ros2 -->

# Ros2

# ROS

## 安装

将开发机与G02机器人通过网线直连，配置开发机的静态IP为10.42.1.102。确保ping 10.42.1.101通信正常。

在开发机环境安装GDK依赖：

```
curl -sSL http://10.42.1.101:8849/install.sh | bash
```

## 声明环境变量与模式切换

在终端执行以下命令：

```
source ~/.cache/agibot/app/env.sh
~/.cache/agibot/app/bin/mode_switch --mode base-fastdds

source ~/.cache/agibot/app/gdk/scripts/ros_env.sh
source ~/.cache/agibot/app/share/gdk_msgs/local_setup.bash
```

切换到 FastDDS 模式后，可直接订阅 `/gdk/joint_state` 查看机器人关节状态：

```
ros2 topic list
ros2 topic hz /gdk/joint_state    # 查看发布频率
ros2 topic echo /gdk/joint_state  # 查看关节状态
```

## 编译代码（C++）

如果环境里缺少 JSON 解析库，请先安装 nlohmann-json3-dev：

```
sudo apt update
sudo apt install -y nlohmann-json3-dev
```

然后执行 `deploy_gdk_msgs.sh` 脚本获取 `gdk_msgs` 消息包后，即可编译示例代码：

```
sudo chmod +x ~/.cache/agibot/app/gdk/scripts/deploy_gdk_msgs.sh
~/.cache/agibot/app/gdk/scripts/deploy_gdk_msgs.sh
cd ~/.cache/agibot/app/gdk/examples/ros
colcon build
```

## 运行机器人运控示例

新开一个终端执行：

```
source ~/.cache/agibot/app/gdk/scripts/ros_env.sh
source ~/.cache/agibot/app/share/gdk_msgs/local_setup.bash
source ~/.cache/agibot/app/gdk/examples/ros/install/setup.bash
ros2 run control_example control_example
```

再开一个终端执行：

```
source ~/.cache/agibot/app/gdk/scripts/ros_env.sh
source ~/.cache/agibot/app/share/gdk_msgs/local_setup.bash
source ~/.cache/agibot/app/gdk/examples/ros/install/setup.bash
ros2 run control_example key_listen
```

**操作说明**

| 按键 | 功能 |
| --- | --- |
| a/d | 切换关节 |
| w/s | 调整关节角度 |
| m | 切换动作序列 |
| p | 播放动作序列 |
| q | 退出程序 |

## 运行相机示例（彩色图像）

新开一个终端执行：

```
source ~/.cache/agibot/app/gdk/scripts/ros_env.sh
source ~/.cache/agibot/app/share/gdk_msgs/local_setup.bash
source ~/.cache/agibot/app/gdk/examples/ros/install/setup.bash

ros2 run camera_example camera_example  # 默认为 /gdk/camera/head_color
```

可通过参数指定不同的相机话题进行查看（除 `/gdk/camera/head_depth` 外）：

```
ros2 run camera_example camera_example \
  --ros-args \
  -p image_topic:=/gdk/camera/hand_left_color
```



---

<!-- source: contents_howto_C++ -->

# C++

# GDK C++ 使用说明

[混合部署](../../concepts/hybrid_deploy/)时，可以在x86平台上基于GDK C++接口二次开发。本文介绍如何在c++项目中使用GDK。

## 基本开发（以运控为例）

### 引入头文件

```
#include "gdk/gdk.h"
```

### 初始化GDK

```
if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
    std::cout << "GDK初始化失败" << std::endl;
    return -1;
}
```

### 初始化机器人对象

```
agibot::gdk::Robot robot;
std::cout << "Robot init" << std::endl;
std::this_thread::sleep_for(std::chrono::seconds(1));
```

### 设置控制的关节与对应的关节角位置，执行速度

```
agibot::gdk::JointControlReq joint_control;
joint_control.life_time = 5.0;
joint_control.joint_names = {"arm_l_joint1", "arm_l_joint2", "arm_l_joint3"};
joint_control.joint_positions = {0.5, -0.3, 0.8};
joint_control.joint_velocities = {0.3, 0.3, 0.3};
```

### 发送关节控制命令

```
if (robot.JointControl(joint_control) != agibot::gdk::GDKRes::kSuccess) {
    std::cout << "Failed to control joints" << std::endl;
} else {
    std::cout << "关节控制指令发送成功" << std::endl;
}
```

## 编译

### CMake编译配置

GDK的头文件和动态库部署在GDK\_HOME目录，混合部署下GDK\_HOME默认在~/.cache/agibot。
将以下内容添加到你的CMakeLists.txt当中，来将你的程序与链接gdk链接库以及头文件，并使用该方法创建编译项目

```
if(NOT DEFINED ENV{GDK_HOME})
    set(GDK_HOME_PATH ~/.cache/agibot)
else()
    set(GDK_HOME_PATH $ENV{GDK_HOME})
endif()

function(add_adapter name)
    add_executable(${name} src/${name}.cpp)

    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
        target_include_directories(${name} PRIVATE ${GDK_HOME_PATH}/app/gdk/build_dep/cpp/x86_64/include)
        target_link_directories(${name} PRIVATE ${GDK_HOME_PATH}/app/gdk/build_dep/cpp/x86_64/lib)
        target_link_libraries(${name} PRIVATE ${GDK_HOME_PATH}/app/gdk/build_dep/cpp/x86_64/lib/libgdk_adapter.so)
    elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
        target_include_directories(${name} PRIVATE ${GDK_HOME_PATH}/app/gdk/build_dep/cpp/aarch64/include)
        target_link_directories(${name} PRIVATE ${GDK_HOME_PATH}/app/gdk/build_dep/cpp/aarch64/lib)
        target_link_libraries(${name} PRIVATE ${GDK_HOME_PATH}/app/gdk/build_dep/cpp/aarch64/lib/libgdk_adapter.so)
    endif()
endfunction()

add_adapter(your_program)
```

### 声明编译链接库路径

```
source ~/.cache/agibot/app/env.sh
```

### 编译

动态库版本一致性

确保编译与运行时链接的动态库版本一致。版本不匹配时 GDK 会输出错误：

```
Version mismatch, runtime version: 2.2.0, build version: 2.2.1
```

```
cmake -B build
make -C build
```

编译产物可在build文件夹中查询

### 运行程序

运行前需先执行 `source ~/.cache/agibot/app/env.sh`，该脚本会自动将所有 GDK 动态库路径加入 `LD_LIBRARY_PATH`。

```
./build/your_program
```



---

<!-- source: contents_howto_deploy -->

# 部署

# 部署

## 环境要求

### 硬件需求

- 符合ISO/IEC 11801：2002和EIA/TIA 568B或ISO/IEC 11801:2002和EIA/TIA 568-B.2-1标准的网线
- 开发机（Ubuntu22.04，x86\_64，Intel i9及以上性能CPU）

### 网络配置

使用网线连接G2接口面板的Debug网口和开发机网口，然后给开发机的网卡配置静态IP：

```
IP：10.42.1.xxx（除 10.42.1.101 外均可使用，101 为 G2 默认 IP）
MASK：255.255.255.0
```

在开发机上验证与G2的通信是否正常：

```
ping 10.42.1.101
```

## 安装GDK

### 混合部署">混合部署

在开发机执行以下命令安装GDK：

```
curl -sSL http://10.42.1.101:8849/install.sh | bash
```

## 获取示例代码

### 从G2上获取示例代码压缩包

```
curl -sSL http://10.42.1.101:8849/install_example.sh | bash
```

该示例文件夹主要包含运控控制，相机图像，传感器数据读取等示例demo，涉及C++、Python、ROS三种编程语言。

### 文件夹结构

```
├── cpp
│   ├── CMakeLists.txt
│   └── src
├── python
│   ├── camera_demo.py
│   ├── camera_web_viewer.py
│   ├── imu_demo.py
│   ├── lidar_demo.py
│   ├── mc_example.py
│   ├── move_chassis.py
│   ├── pnc_example.py
│   ├── robot_demo.py
│   ├── slam_demo.py
│   ├── uss_demo.py
│   ├── proto
│   └── saved_commands
└── ros
    ├── camera_example
    └── control_example
```



---

<!-- source: contents_howto -->

# 如何使用

# 如何使用

## 推荐阅读顺序

```
flowchart LR
    A[部署] --> B[时间同步]
    B --> C{选择开发语言}
    C --> D[C++]
    C --> E[Python]
    C --> F[ROS2]
    D & E & F -.->|按需| G[模式切换]
```

## 各页说明

### 必读

| 页面 | 说明 |
| --- | --- |
| [部署](deploy/) | 将 GDK 安装到开发机，完成网络连接与环境配置 |
| [时间同步](ptp/) | 配置开发机与机器人之间的 PTP 时间同步，传感器数据融合必须 |
| [C++ 使用](C%2B%2B/) | 在 C++ 项目中引入 GDK 头文件、配置 CMake、编译运行 |
| [Python 使用](python/) | 安装 GDK Python 包，编写并运行 Python 控制脚本 |
| [ROS2 使用](ros2/) | 切换到 FastDDS 模式，接入 GDK 的 ROS2 话题 |

### 按需查阅

| 页面 | 何时需要 |
| --- | --- |
| [模式切换](mode/) | 需要在仿真模式、真机模式等不同运行模式之间切换时 |



---

<!-- source: contents_howto_mode -->

# mode

# GDK模式介绍

GDK支持多种模式，包括：

- base：基础模式，支持机器人运动控制、传感器数据获取、建图、移动等底层能力
- develop：开发者模式，支持自定义打开/关闭相机以及将来可能的扩展功能
- base-fastdds：fastdds模式，使用FastDDS作为中间件，支持ROS2通信

# GDK模式切换

## 1. 切换到base模式

```
source ~/.cache/agibot/app/env.sh
~/.cache/agibot/app/bin/mode_switch --mode base
```

## 2. 切换到develop模式

```
source ~/.cache/agibot/app/env.sh
~/.cache/agibot/app/bin/mode_switch --mode develop
```

## 3. 切换到base-fastdds模式

```
source ~/.cache/agibot/app/env.sh
~/.cache/agibot/app/bin/mode_switch --mode base-fastdds
```



---

<!-- source: contents_howto_ptp -->

# 时间同步

# PTP 时间同步配置指南

## 概述

PTP（Precision Time Protocol，精确时间协议）是一种用于在计算机网络中同步时钟的协议，可以实现纳秒级的时间同步精度。GDK 提供脚本用于建立 PTP 时间同步：\*\*Thor 版本\*\*在[用户侧](../../concepts/hybrid_deploy/#用户侧)只需运行 `ptp_hard.sh`；\*\*其他版本\*\*沿用[机器人端](../../concepts/hybrid_deploy/#机器人端) `ptp_server.sh` + 用户侧 `ptp_client.sh` 的分步流程。

## 使用场景

- **多传感器数据融合**：需要精确时间戳对齐多个传感器数据
- **分布式系统同步**：多个设备需要高精度时间同步
- **实时控制系统**：需要纳秒级时间精度的控制应用
- **数据采集与分析**：需要精确时间戳的数据采集场景

## 前置要求

### 用户侧（Client Side）要求

- 网卡 IP 地址为 `10.42.1.*` 网段
- 具有 PTP 硬件时钟设备（`/dev/ptp*`）
- 已安装 `linuxptp` 工具包
- 需要 root 权限执行脚本

## 使用步骤

### Thor 版本

在\*\*用户侧机器\*\*上执行 `ptp_hard.sh`。该脚本使用硬件时间戳（`ptp4l -H`），`ptp4l` 与 `phc2sys` 会常驻运行，按 `Ctrl+C` 结束：

```
cd ~/.cache/agibot/app/gdk/scripts
sudo chmod +x ptp_hard.sh
sudo ./ptp_hard.sh
```

无需再运行 `ptp_server.sh` / `ptp_client.sh`。

### 非 Thor 版本

#### 1. 在机器人端（Robot Side）执行服务器脚本

在机器人端执行 `ptp_server.sh`：

```
cd /home/agi/app/gdk/scripts
sudo chmod +x ptp_server.sh
sudo ./ptp_server.sh
```

#### 2. 在用户侧（Client Side）执行客户端脚本

保持机器人端 `ptp_server.sh` 运行，在用户侧机器上（不要在容器内运行）执行 `ptp_client.sh`：

```
cd ~/.cache/agibot/app/gdk/scripts
sudo chmod +x ptp_client.sh
sudo ./ptp_client.sh
```

**重要**：上述脚本需要 root 权限，必须使用 `sudo` 执行。

## 注意事项

- **Thor / `ptp_hard.sh`**：硬件时间戳、`ptp4l`/`phc2sys` 常驻，需 `Ctrl+C` 停止；**非 Thor / `ptp_client.sh`**：见下条
- **同步精度**：`ptp_client.sh` 软件时间戳版本在同步完后可以达到 10ms 以内的延迟，同步完成后 `ptp_client.sh` 自动退出，`ptp_server.sh` 需要手动停止
- **同步失败处理**：若延时过大，请重新进行同步
- **时间漂移**：同步完后误差会随着时间逐渐变大，一小时内会变到50ms
- **Lidar延迟**：lidar延迟机内机外都在100ms左右
- **传感器延迟**：调用传感器延迟接口前，需要先进行时间同步，否则延迟统计结果不准确



---

<!-- source: contents_howto_python -->

# python

# GDK Python 使用说明

## 安装

### 方式一：默认安装（Python 3.10，推荐）

适用于使用 Python 3.10、且已通过[快速开始](../../getting-started/quick-start/python.md)完成 GDK 部署的用户。

GDK 部署时已自动安装好 Python 包，无需额外操作，只需声明运行路径：

```
source ~/.cache/agibot/app/env.sh
```

### 方式二：从源码安装（其他 Python 版本）

适用于需要使用 Python 3.10 以外版本的用户。

#### 系统要求

- Linux (x86\_64 或 aarch64)
- Python 3.8+

#### 依赖库

- pybind11 >= 2.6.0
- numpy >= 1.19.0
- protobuf >= 5.28.3

```
sudo apt update && sudo apt install -y libprotobuf-dev protobuf-compiler
```

#### 从源码安装

```
cd ~/.cache/agibot/app/gdk/build_dep/python/pybind/
pip install . --no-build-isolation
```

#### 编译 wheel 打包

```
cd ~/.cache/agibot/app/gdk/build_dep/python/pybind/
python setup.py bdist_wheel --dist-dir dist
```

#### 验证安装

```
pip list
```

## 基本使用

### 完整案例

```
import agibot_gdk
import time

agibot_gdk.gdk_init()
robot = agibot_gdk.Robot()
time.sleep(1)

joint_states = robot.get_joint_states()
print(joint_states)
```

### 代码说明

#### 导入agibot\_gdk模块

```
import agibot_gdk
```

#### 初始化gdk模块与机器人对象

```
agibot_gdk.gdk_init()
robot = agibot_gdk.Robot()
time.sleep(1)
```

#### 获取机器人关节信息

```
joint_states = robot.get_joint_states()
print(joint_states)
```

#### 声明python路径

```
source ~/.cache/agibot/app/env.sh
```

#### 执行程序

```
python3 <your_py>
```



---

<!-- source: contents_howto_ros2 -->

# ros2

# GDK ROS2 使用说明

Genie 02使用高性能自研中间件，默认不发送ROS2消息。如需使用ROS2，需要切换到FastDDS模式。
GDK在部署时会同时安装自定义ROS消息，用户声明好相关环境变量后，即可使用ROS2接收/发送GDK消息。

## ROS2 版本要求

需要使用ROS2版本humble以上。

## 使用方式

### 1. 环境要求

使用 ROS2 接口需要：

1. 切换到 GDK 的 FastDDS 模式；
2. 设置环境变量

#### 1.1 切换到 FastDDS 模式

```
source ~/.cache/agibot/app/env.sh
~/.cache/agibot/app/bin/mode_switch --mode base-fastdds
```

#### 1.2 设置环境变量

```
source ~/.cache/agibot/app/gdk/scripts/ros_env.sh
source ~/.cache/agibot/app/share/gdk_msgs/local_setup.bash
```

> 需要特别说明的是：
>
> - 上述环境变量设置仅对当前终端有效
> - 每打开一个新的终端窗口，都需要重新执行环境初始化步骤
> - 若环境未初始化完成，后续 `ros2 topic list`、`ros2 topic echo`、Python 导入或 `colcon build` 均可能失败

### 2. QoS 要求说明

本章节用于说明 ROS2 话题通信中的 QoS 约束，以及在使用程序开发时应如何设置订阅端和发布端的 QoS 配置

#### 2.1 适用范围

1. 使用 Python 编写 ROS2 节点
2. 使用 C++ 编写 ROS2 节点
3. 订阅或发布 `/gdk/...` 话题或其他标准 ROS2 话题，例如 `sensor_msgs`、`tf2_msgs` 等

#### 2.2 原理简述

ROS2 话题通信基于 DDS，每个发布端和订阅端都带有各自的 QoS（Quality of Service）配置，只有当发布端与订阅端的 QoS 策略兼容时，订阅端才能正常收到数据

QoS 不兼容时，常见现象如下：

1. `ros2 topic list` 可以看到话题
2. `ros2 topic type` 可以看到消息类型
3. 但 `ros2 topic echo` 无数据
4. 或程序日志中出现 `incompatible QoS` 告警

#### 2.3 建议配置

对于 `/gdk/...` 话题或其他标准 ROS2 话题，订阅端建议显式使用以下 QoS：

- history = KEEP\_LAST
- depth = 5
- reliability = BEST\_EFFORT
- durability = VOLATILE

**Python 示例**

```
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
```

**C++ 示例**

```
auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
qos.best_effort();
qos.durability_volatile();
```

### 3. rclpy 程序开发说明

**步骤 1：完成环境初始化**

应先执行「第 1 章 环境要求」中的操作

若未完成该步骤，后续 Python 程序可能出现以下问题：

- ros2 命令不可用
- `import gdk_msgs.msg` 时缺少 ROS2 Python 运行时依赖

**步骤 2：编写 Python 节点**

在当前终端中，可直接导入以下模块：

- `rclpy`
- `gdk_msgs.msg`

最小示例如下：

```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from gdk_msgs.msg import JointState

class DemoNode(Node):
    def __init__(self):
        super().__init__("demo_joint_state_sub")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            JointState,
            "/gdk/joint_state",
            self.on_msg,
            qos,
        )

    def on_msg(self, msg):
        self.get_logger().info(f"joint count = {len(msg.name)}")

def main():
    rclpy.init()
    node = DemoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("收到 Ctrl+C，程序退出")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == "__main__":
    main()
```

**步骤 3：运行程序**

假设上述文件名为 demo\_joint\_state.py，则运行方式如下：

```
python3 demo_joint_state.py
```

**步骤 4：验证运行结果**

程序正常运行时，应满足以下条件：

1. 进程能够正常启动
2. 无 `import gdk_msgs`、`import rclpy` 等导入错误
3. 在 `/gdk/joint_state` 有数据的情况下，终端可持续输出关节数量日志

### 4. rclcpp 程序开发说明

**步骤 1：完成环境初始化**

在开始 C++ 编译前，应先执行「1. 环境要求」中的操作

若未完成该步骤，后续 C++ 程序开发过程可能出现以下问题：

- ros2 命令不可用
- `colcon build` 无法正确继承 ROS2 环境
- 工作空间下的 C++ 代码无法识别 `gdk_msgs` 运行环境

**步骤 2：创建并进入 ROS 2 工作空间**

创建开发工作空间，假设在 `~/` 路径下创建名为 `ros2_ws` 的工作空间：

```
mkdir -p ~/ros2_ws/src  # 路径无严格要求、工作空间命名无严格要求
```

进入 `~/ros2_ws/src`：

```
cd ~/ros2_ws/src  # ROS 官方推荐将工作空间中的任何包放置在 src 目录中
```

**步骤 3：编写最小 C++ ROS2 节点**

本步骤用于说明在工作空间中如何编写一个基于 `gdk_msgs` 向 `/gdk/joint_pos_request` 发布一条关节位置控制命令的 ROS2 C++ 节点

一、创建功能包目录

在 `~/ros2_ws/src` 下创建一个示例功能包，例如命名为 `joint_position_demo`：

```
mkdir -p ~/ros2_ws/src/joint_position_demo/src
cd ~/ros2_ws/src/joint_position_demo/src
```

二、编写节点源码

在 `~/ros2_ws/src/joint_position_demo/src` 下创建 `joint_position_pub.cpp` 文件。

编写 C++ 源码时，应先引入当前程序实际需要使用的 ROS2 头文件和 `gdk_msgs` 消息头文件：

```
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "gdk_msgs/msg/joint_position_requst.hpp"
```

QoS 策略建议显式配置如下：

```
auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
qos.best_effort();
qos.durability_volatile();
qos.deadline(std::chrono::nanoseconds(10000000));
```

控制命令通常需要携带唯一标识，用于区分不同请求，可通过 `boost::uuid` 生成 UUID：

```
inline std::string generate_uuid_boost() {
  boost::uuids::random_generator gen;
  return boost::uuids::to_string(gen());
}
```

创建 `/gdk/joint_pos_request` 发布器，可向机器人发送关节位置控制命令：

```
pub_joint_position_ =
    this->create_publisher<gdk_msgs::msg::JointPositionRequst>(
        "/gdk/joint_pos_request",
        qos
    );
```

构造控制命令并发布：

```
gdk_msgs::msg::JointPositionRequst request;
request.lifetime = 1.0;
request.joint_names = joint_names;
request.joint_positions = target_positions;
request.joint_velocities = target_velocity;
request.uuid = generate_uuid_boost();

pub_joint_position_->publish(request);
```

完整的最小控制节点示例如下：

> 该节点启动后会等待 1 秒，然后向 `/gdk/joint_pos_request` 发布一条控制命令

```
inline std::string generate_uuid_boost() {
  boost::uuids::random_generator gen;
  return boost::uuids::to_string(gen());
}

class JointPositionPublisherNode : public rclcpp::Node {
 public:
  JointPositionPublisherNode() : Node("joint_position_publisher_demo") {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
    qos.best_effort();
    qos.durability_volatile();
    qos.deadline(std::chrono::nanoseconds(10000000));

    publisher_ = this->create_publisher<gdk_msgs::msg::JointPositionRequst>(
        "/gdk/joint_pos_request", qos);

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&JointPositionPublisherNode::publish_once, this));
  }

 private:
  void publish_once() {
    if (published_) {
      return;
    }

    gdk_msgs::msg::JointPositionRequst request;
    request.lifetime = 1.0;
    request.joint_names = {"idx21_arm_l_joint1"};
    request.joint_positions = {0.5};
    request.joint_velocities = {0.3};
    request.uuid = generate_uuid_boost();
    request.detail = "joint position demo";

    publisher_->publish(request);
    published_ = true;

    RCLCPP_INFO(this->get_logger(),
                "published joint position request to /gdk/joint_pos_request");
  }

  bool published_ = false;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<gdk_msgs::msg::JointPositionRequst>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointPositionPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

说明：

- `gdk_msgs::msg::JointPositionRequst` 是控制命令消息类型；
- `/gdk/joint_pos_request` 是当前对外暴露的关节位置控制请求话题；
- `joint_names`、`joint_positions`、`joint_velocities` 三个列表长度必须一致；
- 示例中的关节名 `idx21_arm_l_joint1` 仅用于演示，实际使用时应替换为目标机器人上真实存在的关节名称；
- 示例中使用 `timer_` 延迟 1 秒发送一次命令，便于节点启动完成后再执行发布。

三、编写 CMakeLists.txt

在 `~/ros2_ws/src/joint_position_demo` 下创建 `CMakeLists.txt` 文件

若程序中使用了 `gdk_msgs` 消息类型，则 `CMakeLists.txt` 中至少应声明以下依赖：

```
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gdk_msgs REQUIRED)

ament_target_dependencies(control_example
  rclcpp
  gdk_msgs
)
```

说明：

- `find_package(gdk_msgs REQUIRED)` 用于引入 `gdk_msgs` 消息包；
- `ament_target_dependencies(...)` 中必须包含 `gdk_msgs`，否则编译时无法正确解析消息类型相关头文件与依赖。

四、编写 package.xml

在 `~/ros2_ws/src/joint_position_demo` 下创建 `package.xml` 文件

对应的 `package.xml` 中至少应声明：

```
<buildtool_depend>ament_cmake</buildtool_depend>

<depend>rclcpp</depend>
<depend>gdk_msgs</depend>
```

若程序还依赖其他 ROS2 包，应继续补充对应 `<depend>`

**步骤 4：生成最小 gdk\_msgs 源码包脚本**

当用户需要在工作空间中编译依赖 `gdk_msgs` 的 C++ 程序时，应先在工作空间中准备一份可编译的 `gdk_msgs` 源码包

详细操作如下：

若工作空间路径为 `~/ros2_ws/src`，则执行：

```
sudo chmod +x ~/.cache/agibot/app/gdk/scripts/deploy_gdk_msgs.sh
~/.cache/agibot/app/gdk/scripts/deploy_gdk_msgs.sh ~/ros2_ws/src
```

执行完成后，应生成：

```
~/ros2_ws/src/gdk_msgs
```

**步骤 5：执行工作空间编译**

如果环境里缺少 JSON 解析库，请先安装 nlohmann-json3-dev：

```
sudo apt update
sudo apt install -y nlohmann-json3-dev
```

执行下述命令返回工作空间根目录：

```
cd ~/ros2_ws
```

执行编译命令：

```
colcon build
```

**步骤 6：加载本地 install 并运行程序**

编译完成后执行：

```
source install/setup.bash
ros2 run <your_pkg> <your_executable>
```

**步骤 7：验证编译结果**

编译成功时，应满足以下条件：

1. `colcon build` 正常结束
2. 工作空间中生成 `build/`、`install/`、`log/`目录
3. `source install/setup.bash` 后可以正常执行 `ros2 run`
4. 业务程序中可正常包含并使用 `gdk_msgs` 消息类型

### 5. 启动Rviz进行URDF可视化

#### 环境说明

需要一份机器人描述文件,文件路径示例为：

```
your_path/robot_description/urdf/xxx.urdf
```

mesh 资源目录示例：

```
your_path/robot_description/meshes
```

#### 可视化工具安装

```
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install -y \
  ros-humble-rviz2 \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  liburdfdom-tools
```

#### 新建工作空间

##### 创建工作空间

```
mkdir -p ~/rviz_ws/src
cd ~/rviz_ws
source /opt/ros/humble/setup.bash
colcon build
```

##### 配置环境变量

每次打开新终端时运行：

```
source /opt/ros/humble/setup.bash
source ~/rviz_ws/install/setup.bash
```

#### 整理robot description package

##### 将现有 robot description 文件复制到工作空间

```
cd ~/rviz_ws/src
cp -r your_path/robot_description ./genie_robot_description
```

复制后目录应为：

```
~/rviz_ws/src/genie_robot_description
```

##### 补充 ROS2 package文件

新建 package.xml：

```
cat > ~/rviz_ws/src/genie_robot_description/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>genie_robot_description</name>
  <version>0.0.0</version>
  <description>Genie robot description package</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF
```

新建 CMakeLists.txt：

```
cat > ~/rviz_ws/src/genie_robot_description/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(genie_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
EOF
```

##### 编译工作空间

```
cd ~/rviz_ws
source /opt/ros/humble/setup.bash
colcon build
```

编译后需要source:

```
source /opt/ros/humble/setup.bash
source ~/rviz_ws/install/setup.bash
```

##### 验证 package 注册是否成功

```
ros2 pkg list | grep genie_robot_description
ros2 pkg prefix genie_robot_description
```

如输出安装路径，则package安装成功

#### 处理 URDF 中的 mesh 路径

打开URDF文件，搜索 mesh filename:

如果不是

```
<mesh filename="package://genie_robot_description/meshes/xxx.STL" />
```

则需要统一将 mesh 文件路径修改为上述形式

#### 新建模型显示 package

##### 创建 package

```
cd ~/rviz_ws/src
ros2 pkg create --build-type ament_python genie_robot_display
```

##### 创建目录

```
cd ~/rviz_ws/src/genie_robot_display
mkdir launch rviz
```

#### 编写launch文件

```
cd ~/rviz_ws/src/genie_robot_display/launch
```

新建 display.launch.py 文件：

```
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_path = os.path.expanduser(
        "~/rviz_ws/src/genie_robot_description/urdf/xxx.urdf"
    )
    pkg_share = get_package_share_directory("genie_robot_display")
    rviz_config = os.path.join(pkg_share, "rviz", "display.rviz")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
```

注意修改URDF路径和URDF名

#### 编写 RViz 配置文件

```
cd ~/rviz_ws/src/genie_robot_display/rviz
```

新建 display.rviz 文件:

```
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Enabled: true
      Name: Grid
    - Class: rviz_default_plugins/RobotModel
      Enabled: true
      Name: RobotModel
      Description Source: Topic
      Description Topic:
        Value: /robot_description
      Visual Enabled: true
      Collision Enabled: false
      Alpha: 1
    - Class: rviz_default_plugins/TF
      Enabled: true
      Name: TF
  Global Options:
    Fixed Frame: base_link
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 3
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Pitch: 0.5
      Yaw: 0.5
Window Geometry:
  Height: 900
  Width: 1400
```

#### 修改 setup.py

```
cd ~/rviz_ws/src/genie_robot_display
```

修改 setup.py 为:

```
from setuptools import setup
import os
from glob import glob

package_name = 'genie_robot_display'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Display launcher for genie robot in RViz2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
```

#### 编译并启动

##### 编译

```
cd ~/rviz_ws
source /opt/ros/humble/setup.bash
colcon build
```

##### 启动

```
source /opt/ros/humble/setup.bash
source ~/rviz_ws/install/setup.bash
ros2 launch genie_robot_display display.launch.py
```

#### 查看机器人状态

```
source /opt/ros/humble/setup.bash
source ~/rviz_ws/install/setup.bash
ros2 topic echo /joint_states
```

能够查看机器人各关节信息:

```
name:
- arm_l_joint1
- arm_l_joint2
position:
- 0.0
- 0.3
```

#### 最终目录结构

```
~/rviz_ws/
├── src/
│   ├── genie_robot_description/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── urdf/
│   │   └── meshes/
│   └── genie_robot_display/
│       ├── launch/
│       │   └── display.launch.py
│       ├── rviz/
│       │   └── display.rviz
│       ├── package.xml
│       ├── setup.py
│       └── resource/
├── build/
├── install/
└── log/
```

### 6. 注意事项

1. 在新终端使用前务必先执行：

   ```
   source ~/.cache/agibot/app/gdk/scripts/ros_env.sh
   source /home/agi/app/share/gdk_msgs/local_setup.bash
   ```
2. 在建图中，或者重定位成功后，会发布/gdk/odom\_info数据
3. 当话题无数据时，建议按以下顺序排查：
   1. 确认环境初始化是否完成
   2. 确认 `ros2 topic list` 能看到目标话题
   3. 确认 `ros2 topic type` 能获取正确消息类型
   4. 检查程序或命令行是否出现 `incompatible QoS` 告警
   5. 将订阅端 QoS 调整为显式配置
4. 若执行 `colcon build` 时提示 `bash: colcon: command not found`，说明当前环境仅包含 ROS2 运行时，不包含 colcon 开发工具，若要继续开发，应先安装 colcon 开发工具



---

<!-- source: contents_reference_C++_camera -->

# Camera

# GDK Camera 接口文档（C++）

## 概述

Camera（相机）模块为G02机器人提供了获取实时图像数据的功能。通过C++接口，开发者可以方便地获取机器人的视觉感知数据，适用于目标检测、图像识别、视觉导航、SLAM建图、环境监控等多种场景。主要的相机类别如下所示。

- RGB-D 相机：
  同时输出彩色图（RGB）和深度图（Depth）的相机。深度值表示每个像素到相机的距离，通常通过结构光或 ToF（飞行时间）技术实现。GDK 中 RGB-D 相机安装于机器人头部和手部，用于近距离感知与抓取任务。
- 立体相机
  由两个固定基线距离的镜头组成，通过左右图像的视差计算深度。GDK 中立体相机用于头部的远距离三维感知。
- 鱼眼相机
  超广角镜头，视角可达 180° 以上，以较大的图像畸变换取宽广的视野范围。GDK 中鱼眼相机安装于头部，用于环境全景感知。

## 接口说明

### Camera 类

该类封装了相机传感器的主要数据获取接口。

#### 构造函数 / 初始化

- **无参构造**：`Camera()`
  启动所有相机
- **指定相机构造**：`Camera(camera_types)`
  仅启动传入的相机类型，可减少资源占用。

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `camera_types` | `const std::vector<CameraType>&` | 要启动的相机类型列表；默认空表示启动全部相机 |

**示例**：

```
#include "gdk/gdk.h"

// 启动所有相机（默认）
agibot::gdk::Camera camera;

// 仅启动头部彩色与左右手彩色
// std::vector<agibot::gdk::CameraType> types = {
//     agibot::gdk::CameraType::kHeadColor,
//     agibot::gdk::CameraType::kHandLeftColor,
//     agibot::gdk::CameraType::kHandRightColor,
// };
// agibot::gdk::Camera camera(types);
```

#### 1. `GetLatestImage()`

- **功能**：获取最新的图像数据
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `camera_type` | `const CameraType&` | 相机类型枚举值 |
| `timeout_ms` | `const float` | 超时时间（毫秒） |
| `image` | `std::shared_ptr<Image>&` | 输出参数，图像数据指针 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`image`参数包含图像数据

#### Image对象详细说明

**Image结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `timestamp_ns` | `uint64_t` | 图像采集的时间戳 | 纳秒 |
| `width` | `uint32_t` | 图像的宽度（像素） | 像素 |
| `height` | `uint32_t` | 图像的高度（像素） | 像素 |
| `encoding` | `Encoding` | 图像的编码格式枚举 | 枚举值 |
| `color_format` | `ColorFormat` | 图像的颜色格式枚举 | 枚举值 |
| `bit_depth` | `uint8_t` | 每个像素的位数 | 位 |
| `data_view` | `DataView` | 图像的原始像素数据视图 | 数据视图 |

```
struct Image {
  uint32_t width{0};   ///< image width
  uint32_t height{0};  ///< image height

  enum class Encoding : uint8_t {
    UNCOMPRESSED,  ///< uncompressed
    JPEG,          ///< JPEG
    PNG            ///< PNG
  } encoding{Encoding::UNCOMPRESSED};

  enum class ColorFormat : uint8_t {
    RGB,
    BGR,
    RGBA,
    BGRA,

    YUV420,
    YUV422,
    YUV444,
    NV12,
    NV21,

    GRAY8,
    GRAY16,

    BAYER_RGGB,
    BAYER_BGGR,
    BAYER_GBRG,
    BAYER_GRBG,

    RS2_FORMAT_Z16
  } color_format{ColorFormat::RGB};

  uint8_t bit_depth{8};  ///< bit depth

  DataView data_view{};      ///< image data view
  uint64_t timestamp_ns{0};  ///< image timestamp(ns)
};
```

**encoding (编码格式)**：

- **类型**：`Encoding`枚举
- **常见值**：
- `Encoding::UNCOMPRESSED`: 未压缩
- `Encoding::JPEG`: JPEG压缩
- `Encoding::PNG`: PNG压缩

```
 enum class Encoding : uint8_t {
   UNCOMPRESSED,  ///< uncompressed
   JPEG,          ///< JPEG
   PNG            ///< PNG
 } encoding{Encoding::UNCOMPRESSED};
```

**color\_format (颜色格式)**：

- **类型**：`ColorFormat`枚举
- **常见值**：
- `ColorFormat::RGB`: 红绿蓝
- `ColorFormat::BGR`: 蓝绿红
- `ColorFormat::RGBA`: 红绿蓝透明度
- `ColorFormat::BGRA`: 蓝绿红透明度
- `ColorFormat::GRAY8`: 8位灰度
- `ColorFormat::GRAY16`: 16位灰度
- `ColorFormat::YUV420`: YUV420格式
- `ColorFormat::YUV422`: YUV422格式
- `ColorFormat::YUV444`: YUV444格式
- `ColorFormat::NV12`: NV12格式
- `ColorFormat::NV21`: NV21格式
- `ColorFormat::BAYER_RGGB`: RGGB拜耳模式
- `ColorFormat::BAYER_BGGR`: BGGR拜耳模式
- `ColorFormat::BAYER_GBRG`: GBRG拜耳模式
- `ColorFormat::BAYER_GRBG`: GRBG拜耳模式
- `ColorFormat::RS2_FORMAT_Z16`: RealSense Z16深度格式

```
 enum class ColorFormat : uint8_t {
   RGB,
   BGR,
   RGBA,
   BGRA,

   YUV420,
   YUV422,
   YUV444,
   NV12,
   NV21,

   GRAY8,
   GRAY16,

   BAYER_RGGB,
   BAYER_BGGR,
   BAYER_GBRG,
   BAYER_GRBG,

   RS2_FORMAT_Z16
 } color_format{ColorFormat::RGB};
```

**bit\_depth (位深度)**：

- **类型**：`uint8_t`
- **常见值**：
- `8`: 8位（0-255）
- `16`: 16位（0-65535）
- `32`: 32位（浮点）

**data\_view (图像数据)**：

- **类型**：`DataView`
- **描述**：图像的原始像素数据视图
- **用途**：图像处理、显示、保存
- **注意**：需要根据encoding和尺寸进行解析

**相机类型**：
- `CameraType::kHeadBackFisheye`: 头部背部鱼眼相机
- `CameraType::kHeadLeftFisheye`: 头部左侧鱼眼相机
- `CameraType::kHeadRightFisheye`: 头部右侧鱼眼相机
- `CameraType::kHeadStereoLeft`: 头部立体左相机
- `CameraType::kHeadStereoRight`: 头部立体右相机
- `CameraType::kHandLeftColor`: 左手彩色相机
- `CameraType::kHandRightColor`: 右手彩色相机
- `CameraType::kHeadColor`: 头部彩色相机
- `CameraType::kHeadDepth`: 头部深度相机（输出为深度图）
- `CameraType::kHandLeftDepth`: 左手深度相机（输出为深度图）
- `CameraType::kHandRightDepth`: 右手深度相机（输出为深度图）

- `CameraType::kHandLeftUpperColor`: 左手上部彩色相机 (预留)
- `CameraType::kHandRightUpperColor`: 右手上部彩色相机 (预留)
- `CameraType::kHandLeftLowerColor`: 左手下部彩色相机 (预留)
- `CameraType::kHandRightLowerColor`: 右手下部彩色相机 (预留)
- `CameraType::kHandLeftUpperDepth`: 左手上部深度相机（输出为深度图） (预留)
- `CameraType::kHandRightUpperDepth`: 右手上部深度相机（输出为深度图） (预留)
- `CameraType::kHandLeftLowerDepth`: 左手下部深度相机（输出为深度图） (预留)
- `CameraType::kHandRightLowerDepth`: 右手下部深度相机（输出为深度图） (预留)
- 在常规模式下，默认打开头部立体左相机，头部立体右相机，左右彩色相机，右手彩色相机，头部彩色相机，头部深度相机，其余相机默认关闭，且不建议在常规模式下打开其余相机
- 可以在develop模式下开启或关闭其余相机
- **示例**：

```
#include <iostream>
#include <thread>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Camera gdk_camera;
    std::cout << "Camera init" << std::endl;

    std::shared_ptr<agibot::gdk::Image> image = std::make_shared<agibot::gdk::Image>();

    agibot::gdk::CameraType camera_type = agibot::gdk::CameraType::kHandLeftColor;

    if (gdk_camera.GetLatestImage(camera_type, 500, image) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to get latest image" << std::endl;
    } else {
        std::cout << "Image shape: " << image->width << "x" << image->height << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 2. `GetNearestImage()`

- **功能**：获取指定时间戳附近最近的图像数据
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `camera_type` | `const CameraType&` | 相机类型枚举值 |
| `timestamp_ns` | `const uint64_t` | 目标时间戳（纳秒） |
| `timeout_ms` | `const float` | 超时时间（毫秒） |
| `image` | `std::shared_ptr<Image>&` | 输出参数，图像数据指针 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`image`参数包含图像数据，结构与`GetLatestImage()`相同
- **示例**：

```
#include <iostream>
#include <thread>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Camera gdk_camera;
    std::cout << "Camera init" << std::endl;

    std::shared_ptr<agibot::gdk::Image> image = std::make_shared<agibot::gdk::Image>();

    agibot::gdk::CameraType camera_type = agibot::gdk::CameraType::kHandLeftColor;

    if(gdk_camera.GetNearestImage(camera_type, 0, 2000.0, image) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GetNearestImage failed" << std::endl;
        return -1;
    } else {
        std::cout << "Image shape: " << image->width << " x " << image->height << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 3. `GetImageShape()`

- **功能**：获取图像数据大小
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `camera_type` | `const CameraType&` | 相机类型枚举值 |
| `shape` | `std::tuple<int, int>&` | 输出参数，图像宽度和高度的元组 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`shape`参数包含图像尺寸
- **示例**：

```
#include <iostream>
#include <thread>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Camera gdk_camera;
    std::cout << "Camera init" << std::endl;

    std::shared_ptr<agibot::gdk::Image> image = std::make_shared<agibot::gdk::Image>();

    agibot::gdk::CameraType camera_type = agibot::gdk::CameraType::kHandRightColor;

    std::tuple<int, int> shape;
    if (gdk_camera.GetImageShape(camera_type, shape) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to get image shape" << std::endl;
    } else {
        std::cout << "Image shape: " << std::get<0>(shape) << "x" << std::get<1>(shape) << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 4. `GetImageFps()`

- **功能**：获取图像捕获帧率
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `camera_type` | `const CameraType&` | 相机类型枚举值 |
| `fps` | `float&` | 输出参数，图像帧率（FPS） |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`fps`参数包含帧率值
- **示例**：

```
#include <iostream>
#include <thread>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Camera gdk_camera;
    std::cout << "Camera init" << std::endl;

    std::shared_ptr<agibot::gdk::Image> image = std::make_shared<agibot::gdk::Image>();

    agibot::gdk::CameraType camera_type = agibot::gdk::CameraType::kHandLeftColor;

    float fps;
    if (gdk_camera.GetImageFps(camera_type, fps) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to get image fps" << std::endl;
    } else {
        std::cout << "Image fps: " << fps << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 5. `GetImageLatency()`

- **注意事项**：获取图像延迟统计信息前，需要先进行时间同步，否则延迟统计结果不准确
- **功能**：获取图像延迟统计信息
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `camera_type` | `const CameraType&` | 相机类型枚举值 |
| `window_seconds` | `const float` | 统计窗口时间（秒） |
| `latency` | `LatencyStats&` | 输出参数，延迟统计信息 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`latency`参数包含延迟统计信息
- **LatencyStats结构体说明**：

```
struct LatencyStats {
  double max_latency_ms{0.0};    ///< max latency(ms)
  double avg_latency_ms{0.0};    ///< average latency(ms)
  double p99_latency_ms{0.0};    ///< 99th percentile latency(ms)
  double p999_latency_ms{0.0};   ///< 99.9th percentile latency(ms)
  double p9999_latency_ms{0.0};  ///< 99.99th percentile latency(ms)
};
```

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `max_latency_ms` | `double` | 最大延迟 | 毫秒 |
| `avg_latency_ms` | `double` | 平均延迟 | 毫秒 |
| `p99_latency_ms` | `double` | 99分位延迟 | 毫秒 |
| `p999_latency_ms` | `double` | 99.9分位延迟 | 毫秒 |
| `p9999_latency_ms` | `double` | 99.99分位延迟 | 毫秒 |

- **示例**：

```
#include <iostream>
#include <thread>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Camera gdk_camera;
    std::cout << "Camera init" << std::endl;

    std::shared_ptr<agibot::gdk::Image> image = std::make_shared<agibot::gdk::Image>();

    agibot::gdk::CameraType camera_type = agibot::gdk::CameraType::kHandLeftColor;

    agibot::gdk::LatencyStats latency;
    if (gdk_camera.GetImageLatency(agibot::gdk::CameraType::kHeadStereoLeft, 1.0, latency) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to get image latency" << std::endl;
    } else {
        std::cout << "Image latency: " << latency.max_latency_ms << "ms" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 6. `GetCameraIntrinsic()`

- **功能**：获取相机内参信息
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `camera_type` | `const CameraType&` | 相机类型枚举值 |
| `intrinsic` | `CameraIntrinsic&` | 输出参数，相机内参信息 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`intrinsic`参数包含相机内参
- **注意**：并非所有相机类型都支持内参获取。支持内参的相机类型包括：`kHeadBackFisheye`、`kHeadLeftFisheye`、`kHeadRightFisheye`、`kHeadStereoLeft`、`kHeadStereoRight`、`kHandLeftColor`、`kHandRightColor`、`kHeadColor`、`kHeadDepth`、`kHandLeftDepth`、`kHandRightDepth`。对于不支持的相机类型，调用此接口将返回错误。
- **CameraIntrinsic结构体说明**：

```
struct CameraIntrinsic {
  std::vector<double> intrinsic{};  ///< camera intrinsic, fx, fy, cx, cy
  std::vector<double> distortion{}; ///< camera distortion, k1, k2, p1, p2, k3, k4, k5, k6
};
```

| 成员名 | 类型 | 描述 | 索引 | 单位 |
| --- | --- | --- | --- | --- |
| `intrinsic[0]` | `double` | 焦距x方向 (fx) | 0 | 像素 |
| `intrinsic[1]` | `double` | 焦距y方向 (fy) | 1 | 像素 |
| `intrinsic[2]` | `double` | 主点x坐标 (cx) | 2 | 像素 |
| `intrinsic[3]` | `double` | 主点y坐标 (cy) | 3 | 像素 |
| `distortion[0]` | `double` | 径向畸变系数1 (k1) | 0 | 无量纲 |
| `distortion[1]` | `double` | 径向畸变系数2 (k2) | 1 | 无量纲 |
| `distortion[2]` | `double` | 切向畸变系数1 (p1) | 2 | 无量纲 |
| `distortion[3]` | `double` | 切向畸变系数2 (p2) | 3 | 无量纲 |
| `distortion[4]` | `double` | 径向畸变系数3 (k3) | 4 | 无量纲 |
| `distortion[5]` | `double` | 径向畸变系数4 (k4) | 5 | 无量纲 |
| `distortion[6]` | `double` | 径向畸变系数5 (k5) | 6 | 无量纲 |
| `distortion[7]` | `double` | 径向畸变系数6 (k6) | 7 | 无量纲 |

- **不同相机类型的内参支持**：

| 相机类型 | intrinsic向量大小 | distortion向量大小 | 说明 |
| --- | --- | --- | --- |
| **双目相机** | 4 (fx, fy, cx, cy) | 8 (k1, k2, p1, p2, k3, k4, k5, k6) | 完整的12参数畸变模型 |
| **RGB-D相机** | 4 (fx, fy, cx, cy) | 5 (k1, k2, p1, p2, k3) | 9参数畸变模型 |
| **鱼眼相机** | 4 (fx, fy, cx, cy) | 6 (k1, k2, p1, p2, k3, k4) | 10参数畸变模型 |

- **畸变模型说明**：
- **径向畸变**：`k1, k2, k3, k4, k5, k6` - 用于校正镜头径向畸变
- **切向畸变**：`p1, p2` - 用于校正镜头切向畸变
- **不同相机类型**：根据镜头特性使用不同数量的畸变参数
- **示例**：

```
#include <iostream>
#include <thread>
#include "gdk/gdk.h"

void printCameraIntrinsic(const agibot::gdk::CameraType& camera_type,
                         const agibot::gdk::CameraIntrinsic& intrinsic) {
    std::cout << "Camera intrinsic for " << static_cast<int>(camera_type) << ":" << std::endl;

    // 显示内参矩阵 (fx, fy, cx, cy)
    if (intrinsic.intrinsic.size() >= 4) {
        std::cout << "  fx: " << intrinsic.intrinsic[0] << ", fy: " << intrinsic.intrinsic[1] << std::endl;
        std::cout << "  cx: " << intrinsic.intrinsic[2] << ", cy: " << intrinsic.intrinsic[3] << std::endl;
    }

    // 显示畸变参数
    if (intrinsic.distortion.size() > 0) {
        std::cout << "  k1: " << intrinsic.distortion[0];
        if (intrinsic.distortion.size() > 1) std::cout << ", k2: " << intrinsic.distortion[1];
        if (intrinsic.distortion.size() > 2) std::cout << ", p1: " << intrinsic.distortion[2];
        if (intrinsic.distortion.size() > 3) std::cout << ", p2: " << intrinsic.distortion[3];
        if (intrinsic.distortion.size() > 4) std::cout << ", k3: " << intrinsic.distortion[4];
        std::cout << std::endl;

        // 根据相机类型显示额外的畸变参数
        if (camera_type == agibot::gdk::CameraType::kHeadStereoLeft ||
            camera_type == agibot::gdk::CameraType::kHeadStereoRight) {
            // 双目相机：显示 k4, k5, k6
            if (intrinsic.distortion.size() > 5) std::cout << "  k4: " << intrinsic.distortion[5];
            if (intrinsic.distortion.size() > 6) std::cout << ", k5: " << intrinsic.distortion[6];
            if (intrinsic.distortion.size() > 7) std::cout << ", k6: " << intrinsic.distortion[7];
            std::cout << " (双目相机，12参数畸变模型)" << std::endl;
        } else if (camera_type == agibot::gdk::CameraType::kHeadDepth) {
            // RGBD相机：只显示 k1, k2, k3
            std::cout << "  (RGBD相机，9参数畸变模型)" << std::endl;
        } else if (camera_type == agibot::gdk::CameraType::kHeadBackFisheye ||
                   camera_type == agibot::gdk::CameraType::kHandLeftColor ||
                   camera_type == agibot::gdk::CameraType::kHandRightColor) {
            // 鱼眼相机：显示 k4
            if (intrinsic.distortion.size() > 5) {
                std::cout << "  k4: " << intrinsic.distortion[5] << " (鱼眼相机，10参数畸变模型)" << std::endl;
            }
        }
    }
}

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Camera gdk_camera;
    std::cout << "Camera init" << std::endl;

    // 获取不同相机类型的内参
    std::vector<agibot::gdk::CameraType> camera_types = {
        agibot::gdk::CameraType::kHeadStereoLeft,    // 双目相机
        agibot::gdk::CameraType::kHeadDepth,         // RGBD相机
        agibot::gdk::CameraType::kHeadBackFisheye  // 鱼眼相机
    };

    for (auto camera_type : camera_types) {
        agibot::gdk::CameraIntrinsic intrinsic;
        if (gdk_camera.GetCameraIntrinsic(camera_type, intrinsic) != agibot::gdk::GDKRes::kSuccess) {
            std::cout << "Failed to get camera intrinsic for type " << static_cast<int>(camera_type) << std::endl;
        } else {
            printCameraIntrinsic(camera_type, intrinsic);
        }
        std::cout << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 7. `SetDevCameraConfig()`

- **功能**：相机客制化功能(开关相机、设置帧率)
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `cam_conf_path` | `const std::string&` | 相机配置文件路径 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **注意**：
- 配置文件路径必须存在，否则返回`GDKRes::kInvalidInput`
- 每次修改客制化相机配置文件且调用接口后，需要重新切换模式至develop模式

##### 配置选项

GDK支持配置相机的开关与设置的帧率，相机客制化的配置文件在部署包下，其绝对路径通常为 `~/.cache/agibot/app/gdk/config/r1_camera_conf.json`或`~/.cache/agibot/app/gdk/config/thor_camera_conf.json`（其中 `~` 表示用户主目录，如 `/home/your_name`）
r1机器的配置文件为r1\_camera\_conf.json，thor机器的配置文件为thor\_camera\_conf.json，使用时注意文件名称
设置publish为true开启相机，false关闭相机，设置fps来控制相机帧率

相机具体配置形式如下

```
{
    "cam0": {
        "fps": "30",
        "name": "head_stereo_right",
        "publish": true
    },
    "cam3": {
        "fps": "30",
        "name": "head_stereo_left",
        "publish": true
    },
    "cam4": {
        "fps": "30",
        "name": "hand_left_depth",
        "publish": false
    },
    "cam5": {
        "fps": "30",
        "name": "hand_left_color",
        "publish": true
    },
    "cam6": {
        "fps": "30",
        "name": "hand_right_depth",
        "publish": false
    },
    "cam7": {
        "fps": "30",
        "name": "hand_right_color",
        "publish": true
    },
    "cam10": {
        "fps": "30",
        "name": "head_right_fisheye",
        "publish": false
    },
    "cam11": {
        "fps": "30",
        "name": "head_left_fisheye",
        "publish": false
    },
    "cam12": {
        "fps": "30",
        "name": "head_back_fisheye",
        "publish": false
    },
    "cam14": {
        "fps": "30",
        "name": "head_depth",
        "publish": true
    },
    "cam15": {
        "fps": "30",
        "name": "head_color",
        "publish": true
    }
}
```

##### 模式切换

```
./mode_switch --mode develop # 切换到develop模式
```

如需切换回之前base模式，则执行

```
./mode_switch --mode base # 切换到base模式
```

- **示例**：

```
#include <iostream>
#include <thread>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Camera gdk_camera;
    std::cout << "Camera init" << std::endl;

    // 设置相机配置
    std::string config_path = "/home/<your_name>/.cache/agibot/app/gdk/config/r1_camera_conf.json";
    if (gdk_camera.SetDevCameraConfig(config_path) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to set camera config" << std::endl;
    } else {
        std::cout << "Camera config set successfully" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 8. `CloseCamera()`

- **功能**：关闭相机DDS连接
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include <iostream>
#include <thread>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Camera gdk_camera;
    std::cout << "Camera init" << std::endl;

    // 使用相机...

    // 关闭相机
    if (gdk_camera.CloseCamera() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to close camera" << std::endl;
    } else {
        std::cout << "Camera closed successfully" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

## 使用注意事项

1. **GDK初始化**：使用Camera功能前必须先调用`agibot::gdk::GDKInit()`初始化GDK系统
2. **GDK释放**：程序结束前必须调用`agibot::gdk::GDKRelease()`释放GDK系统资源
3. **超时设置**：根据实际需求设置合适的超时时间，避免长时间阻塞
4. **数据有效性**：使用前请检查返回的图像数据是否为nullptr
5. **时间戳精度**：时间戳单位为纳秒，可用于精确的时间同步
6. **图像处理**：图像数据量较大，处理时注意内存使用
7. **相机选择**：根据应用场景选择合适的相机类型（鱼眼/立体/深度等）
8. **帧率控制**：注意相机的帧率限制，避免过度请求
9. **相机配置**：使用`SetDevCameraConfig()`设置相机配置时，确保配置文件路径有效，相机类型与配置文件匹配
10. **错误处理**：始终检查GDKRes返回值，确保操作成功
11. **相机开关**：在常规模式/develop模式下，若打开更多相机，存在性能风险

## 应用场景

- **目标检测**：利用图像数据进行目标识别和检测
- **视觉导航**：为机器人导航提供视觉信息
- **SLAM建图**：结合图像数据进行同时定位与地图构建
- **环境监控**：实时监控周围环境变化
- **深度感知**：使用深度相机获取3D环境信息
- **立体视觉**：利用双目相机进行距离测量
- **数据融合**：与其他传感器数据进行融合，提高感知精度
- **相机标定**：使用相机内参进行图像校正和畸变补偿
- **3D重建**：结合相机内参进行三维点云重建
- **视觉测量**：利用相机内参进行精确的尺寸测量



---

<!-- source: contents_reference_C++_common -->

# Common

# GDK Common 接口文档（C++）

## 概述

Common（通用）模块为G02机器人提供了GDK系统的初始化和释放功能。通过C++接口，开发者可以方便地管理GDK系统的生命周期，确保系统正确启动和资源正确释放。

## 接口说明

### 全局函数

#### 1. `GDKInit()`

- **功能**：初始化GDK系统
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码
- `GDKRes::kSuccess`：初始化成功
- 其他值：初始化失败
- **说明**：
- 必须在调用其他GDK功能之前调用此函数
- 初始化GDK内部系统、DDS连接、配置管理等
- 建议在程序开始时调用一次
- **示例**：

```
#include <iostream>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    agibot::gdk::GDKRes init_result = agibot::gdk::GDKInit();

    if (init_result != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败: " << static_cast<int>(init_result) << std::endl;
        return -1;
    }

    std::cout << "GDK初始化成功" << std::endl;

    // 使用GDK功能...

    return 0;
}
```

#### 2. `GetAppVersion()`

- **功能**：获取应用版本号
- **前置条件**：需先调用 `GDKInit()` 初始化 GDK 系统
- **参数**：
- `version`：`std::string&`，输出参数，用于接收版本字符串
- **返回值**：`GDKRes`，操作结果状态码
- `GDKRes::kSuccess`：成功，版本已写入 `version`
- 其他值：失败（如配置文件未找到、解析错误）
- **示例**：

```
#include <iostream>
#include "gdk/gdk.h"
#include <thread>
int main() {
    agibot::gdk::GDKRes init_result = agibot::gdk::GDKInit();
    if (init_result != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败，错误码: " << static_cast<int>(init_result) << std::endl;
        return -1;
    }
    std::string version;
    if (agibot::gdk::GetAppVersion(version) == agibot::gdk::GDKRes::kSuccess) {
        std::cout << "应用版本: " << version << std::endl;
    } else {
        std::cout << "获取版本失败" << std::endl;
    }

    agibot::gdk::GDKRelease();
    return 0;
}
```

#### 3. `GetRobotAid()`

- **功能**：获取机器人 AID
- **前置条件**：需先调用 `GDKInit()` 初始化 GDK 系统
- **参数**：
- `aid`：`std::string&`，输出参数，用于接收 AID 字符串
- **返回值**：`GDKRes`，操作结果状态码
- `GDKRes::kSuccess`：成功，AID 已写入 `aid`
- 其他值：失败（如配置文件未找到、AID 字段缺失）
- **示例**：

```
#include <iostream>
#include "gdk/gdk.h"
#include <thread>
int main() {
    agibot::gdk::GDKRes init_result = agibot::gdk::GDKInit();
    if (init_result != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败，错误码: " << static_cast<int>(init_result) << std::endl;
        return -1;
    }
    std::string aid;
    if (agibot::gdk::GetRobotAid(aid) == agibot::gdk::GDKRes::kSuccess) {
        std::cout << "机器人 AID: " << aid << std::endl;
    } else {
        std::cout << "获取 AID 失败" << std::endl;
    }

    agibot::gdk::GDKRelease();
    return 0;
}
```

#### 4. `GDKRelease()`

- **功能**：释放GDK系统资源
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码
- `GDKRes::kSuccess`：释放成功
- 其他值：释放失败
- **说明**：
- 在程序结束前调用此函数释放GDK系统资源
- 清理DDS连接、关闭文件句柄、释放内存等
- 建议在程序退出前调用一次
- **示例**：

```
#include <iostream>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    agibot::gdk::GDKRes init_result = agibot::gdk::GDKInit();

    if (init_result != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    std::cout << "GDK初始化成功" << std::endl;

    // 使用GDK功能...

    // 释放GDK系统资源
    agibot::gdk::GDKRes release_result = agibot::gdk::GDKRelease();

    if (release_result != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败: " << static_cast<int>(release_result) << std::endl;
        return -1;
    }

    std::cout << "GDK释放成功" << std::endl;
    return 0;
}
```

## 使用注意事项

1. **初始化顺序**：必须在调用任何其他GDK功能（包括 `GetAppVersion`、`GetRobotAid`）之前调用`GDKInit()`
2. **释放顺序**：在程序结束前调用`GDKRelease()`释放资源
3. **错误处理**：始终检查返回值，确保操作成功
4. **单次调用**：通常只需要在程序开始时调用一次`GDKInit()`，结束时调用一次`GDKRelease()`
5. **异常安全**：在异常情况下也要确保调用`GDKRelease()`释放资源
6. **多线程**：GDK初始化是全局的，多线程环境下只需要一个线程调用即可

## 错误码说明

| 错误码 | 描述 | 可能原因 |
| --- | --- | --- |
| `GDKRes::kSuccess` | 操作成功 | - |
| `GDKRes::kInvalidInput` | 无效输入 | 参数错误 |
| `GDKRes::kInvalidOutput` | 无效输出 | 输出参数错误 |
| `GDKRes::kTimeout` | 超时 | 操作超时 |
| `GDKRes::kNotInitialized` | 未初始化 | 系统未正确初始化 |
| `GDKRes::kAlreadyInitialized` | 已初始化 | 重复初始化 |
| `GDKRes::kInternalError` | 内部错误 | 系统内部错误 |

## 应用场景

- **系统启动**：在机器人系统启动时初始化GDK
- **资源管理**：确保GDK系统资源正确释放



---

<!-- source: contents_reference_C++_env -->

# Env

# GDK Env 接口文档（C++）

## 概述

Env（环境）模块为 G02 机器人提供统一的观测与控制接口，适用于强化学习、VLA、遥操作等需要对机器人进行 step(action) → observation 式交互的场景。Env 将相机图像、机器人状态、TF 等数据聚合为 `agibot::gdk::protocol::Observation` 结构体，并支持通过 `protocol::Action` 执行轨迹控制。

## 头文件

```
#include "gdk/gdk.h"
```

## 接口说明

### Env 类

该类封装了观测获取、轨迹执行与参考系设置等功能。

#### 1. 构造函数

- **函数签名**：`Env(const std::vector<CameraType>& camera_types = {});`
- **功能**：创建 Env 实例并初始化
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `camera_types` | `const std::vector<CameraType>&` | 相机类型列表；空或默认表示使用默认相机组 |

- **示例**：

```
#include "gdk/gdk.h"
#include <chrono>
#include <thread>

int main() {
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        return -1;
    }

    // 使用默认相机组
    agibot::gdk::Env env;

    // 或指定相机类型
    // agibot::gdk::Env env({
    //     agibot::gdk::CameraType::kHeadColor,
    //     agibot::gdk::CameraType::kHandLeftColor,
    //     agibot::gdk::CameraType::kHandRightColor
    // });

    agibot::gdk::GDKRelease();
    return 0;
}
```

#### 2. `GetObservation()`

- **函数签名**：

```
GDKRes GetObservation(agibot::gdk::protocol::Observation& obs,
                      bool include_images = true,
                      bool include_states = true);
```

- **功能**：获取最新观测数据
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `obs` | `protocol::Observation&` | 输出参数，观测结构体 |
| `include_images` | `bool`，默认 `true` | 是否包含相机图像 |
| `include_states` | `bool`，默认 `true` | 是否包含机器人关节与状态数据 |

- **返回值**：`GDKRes`，成功返回 `GDKRes::kSuccess`

**Observation 结构体**（定义于 `common/protocol.h`）：

```
struct Observation {
    std::string protocol_version{"1.1.2"};
    std::map<std::string, int64_t> timestamps;  // 各传感器时间戳（纳秒）
    Images images;                               // 图像数据
    States states;                               // 机器人状态
};
```

- `protocol_version`：协议版本号，当前为 `"1.1.2"`，可用于版本兼容性检查。
- `timestamps`：各传感器数据的采集时间戳（纳秒），键为传感器名称，如 `"arm"`、`"head_camera"` 等。

---

#### `Images`

所有相机字段均为 `std::optional<ImagePacket>`，未开启的相机对应字段为空。

| 字段名 | 描述 |
| --- | --- |
| `head` | 头部彩色相机 |
| `head_depth` | 头部深度相机 |
| `hand_left` | 左手相机 |
| `hand_right` | 右手相机 |
| `head_left_fisheye` | 头部左鱼眼 |
| `head_right_fisheye` | 头部右鱼眼 |
| … | 其他相机字段 |

**`ImagePacket` 字段说明：**

```
struct ImagePacket {
    std::string encoding;            // 编码格式，如 "JPEG"、"PNG"、"UNCOMPRESSED"
    std::string color_format;        // 颜色格式，如 "RGB"、"BGR"、"RS2_FORMAT_Z16"
    int width{0};                    // 图像宽度（像素）
    int height{0};                   // 图像高度（像素）
    std::vector<uint8_t> image_data; // 图像原始字节数据
};
```

---

#### `States`

所有字段均为 `std::optional`，未获取到数据时为空。按功能分组如下。

**关节位置**

| 字段名 | 类型 | 描述 |
| --- | --- | --- |
| `head_joint_states` | `vector<double>` | 头部关节角（弧度） |
| `arm_joint_states` | `vector<double>` | 双臂关节角（弧度），左右臂拼接 |
| `waist_joint_states` | `vector<double>` | 腰部关节角（弧度） |
| `gripper_states` | `vector<double>` | 夹爪状态 |
| `hand_joint_states` | `vector<double>` | 灵巧手关节角（弧度） |
| `effector_states` | `vector<double>` | 末端执行器状态 |

**关节速度 / 模式 / 力矩**

| 字段名 | 类型 | 描述 |
| --- | --- | --- |
| `arm_velocity` | `vector<double>` | 双臂关节速度（弧度/秒） |
| `arm_mode` | `vector<int>` | 双臂关节控制模式 |
| `arm_effort` | `vector<double>` | 双臂关节力矩（牛顿·米） |
| `head_velocity` | `vector<double>` | 头部关节速度 |
| `head_mode` | `vector<int>` | 头部关节模式 |
| `head_effort` | `vector<double>` | 头部关节力矩 |
| `waist_velocity` | `vector<double>` | 腰部关节速度 |
| `waist_mode` | `vector<int>` | 腰部关节模式 |
| `waist_effort` | `vector<double>` | 腰部关节力矩 |
| `motor_velocity` | `vector<double>` | 全身电机速度（旧版，已逐步废弃） |
| `motor_mode` | `vector<int>` | 全身电机模式 |
| `motor_effort` | `vector<double>` | 全身电机力矩 |

**末端位姿（结构化格式）**

末端执行器位姿、六维力、六维速度均以 `EndEffectorPoses` / `EndEffectorWrenches` / `EndEffectorTwists` 结构体封装，包含 `arm_base_link`（相对于机械臂基座）和 `base_link`（相对于机器人基座）两个参考系版本。

| 字段名 | 类型 | 描述 |
| --- | --- | --- |
| `end_pose` | `EndEffectorPoses` | 末端执行器位姿 |
| `end_wrench` | `EndEffectorWrenches` | 末端执行器六维力 |
| `end_twist` | `EndEffectorTwists` | 末端执行器六维速度 |
| `tool_pose` | `EndEffectorPoses` | 工具坐标系位姿 |
| `tool_wrench` | `EndEffectorWrenches` | 工具坐标系六维力 |
| `tool_twist` | `EndEffectorTwists` | 工具坐标系六维速度 |
| `waist_pose` | `Pose` | 腰部位姿 |

相关基础结构体定义：

```
struct Pose {
    std::vector<double> position;    // [x, y, z]（米）
    std::vector<double> orientation; // [qx, qy, qz, qw] 四元数
};

struct Wrench {
    std::vector<double> force;   // [fx, fy, fz]（牛顿）
    std::vector<double> torque;  // [tx, ty, tz]（牛顿·米）
};

struct Twist {
    std::vector<double> linear;  // [vx, vy, vz]（米/秒）
    std::vector<double> angular; // [wx, wy, wz]（弧度/秒）
};

// EndEffectorPoses 举例（Wrenches / Twists 结构相同）
struct EndEffectorPoses {
    std::optional<DualArmPose> arm_base_link; // 相对于机械臂基座的位姿
    std::optional<DualArmPose> base_link;     // 相对于机器人基座的位姿
};

struct DualArmPose {
    std::optional<Pose> left_arm;
    std::optional<Pose> right_arm;
};
```

**示例：读取左臂末端位置（相对 base\_link）：**

```
if (obs.states.end_pose && obs.states.end_pose->base_link) {
    const auto& left = obs.states.end_pose->base_link->left_arm;
    if (left) {
        double x = left->position[0];
        double y = left->position[1];
        double z = left->position[2];
    }
}
```

**TF**

`tf`（`std::optional<std::map<std::string, TF>>`）：坐标变换树数据，键为目标坐标系名称（`child_frame_id`）。

```
struct TF {
    int64_t     timestamp_ns{0};      // 时间戳（纳秒）
    std::string frame_id;             // 父坐标系名称
    std::string child_frame_id;       // 子坐标系名称
    std::vector<double> position;     // [x, y, z]（米）
    std::vector<double> orientation;  // [qx, qy, qz, qw] 四元数
};
```

**触觉传感器（末端力传感器阵列）**

| 字段名 | 类型 | 描述 |
| --- | --- | --- |
| `left_ee_force` | `EndForceState` | 左末端触觉传感器阵列 |
| `right_ee_force` | `EndForceState` | 右末端触觉传感器阵列 |

```
struct EndForceState {
    bool controlled{false};                        // 是否处于力控模式
    std::map<std::string, ForceSensor> sensors;    // 各传感器数据，key 为传感器名称
};

struct ForceSensor {
    uint32_t rows{0};                  // 阵列行数
    uint32_t cols{0};                  // 阵列列数
    float resolution_x{0.0f};         // X 方向分辨率（米）
    float resolution_y{0.0f};         // Y 方向分辨率（米）
    std::vector<float> normal_force;  // 法向力，长度 rows×cols（牛顿）
    std::vector<float> shear_force_x; // 切向力 X，长度 rows×cols
    std::vector<float> shear_force_y; // 切向力 Y，长度 rows×cols
    std::vector<uint8_t> contact;     // 接触状态（1=接触），长度 rows×cols
    std::vector<uint8_t> valid;       // 数据有效性标记，长度 rows×cols
    uint32_t err_code{0};             // 错误码，0 表示正常
};
```

**机器人位姿与状态**

| 字段名 | 类型 | 描述 |
| --- | --- | --- |
| `robot_position` | `vector<double>` | 机器人底盘全局位置 [x, y, z]（米） |
| `robot_orientation` | `vector<double>` | 机器人底盘全局姿态 [qx, qy, qz, qw] |
| `copilot_state` | `string` | copilot 状态描述字符串 |
| `copilot_reward` | `string` | copilot reward 信息 |
| `joint_current_values` | `vector<double>` | 关节电流值 |

**电机错误状态**

| 字段名 | 类型 | 描述 |
| --- | --- | --- |
| `end_mode` | `vector<int>` | 末端控制模式 |
| `err_code` | `vector<int>` | 各关节错误码，0 表示正常 |
| `err_msg` | `vector<string>` | 各关节错误描述 |

**已弃用字段**

| 字段名 | 描述 |
| --- | --- |
| `end_wrench_states` | 原始 12 维六维力，已被 `end_wrench` 替代 |
| `end_velocity` | 原始 12 维末端速度，已被 `end_twist` 替代 |

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        return -1;
    }

    agibot::gdk::Env env;

    agibot::gdk::protocol::Observation obs;
    if (env.GetObservation(obs) != agibot::gdk::GDKRes::kSuccess) {
        std::cerr << "GetObservation failed" << std::endl;
        agibot::gdk::GDKRelease();
        return -1;
    }

    std::cout << "协议版本: " << obs.protocol_version << std::endl;

    // 输出各相机图像数据大小
    auto print_img = [](const char* name, const auto& opt) {
        if (opt && !opt->image_data.empty()) {
            std::cout << "  " << name << ": " << opt->width << "x"
                      << opt->height << ", " << opt->image_data.size()
                      << " bytes" << std::endl;
        }
    };
    print_img("head", obs.images.head);
    print_img("head_depth", obs.images.head_depth);
    print_img("hand_left", obs.images.hand_left);
    print_img("hand_right", obs.images.hand_right);

    // 仅获取状态，不包含图像
    obs = {};
    env.GetObservation(obs, false, true);

    agibot::gdk::GDKRelease();
    return 0;
}
```

#### 3. `ExecuteTrajectory()`

- **函数签名**：

```
GDKRes ExecuteTrajectory(const agibot::gdk::protocol::Action& action);
```

- **功能**：执行轨迹动作
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `action` | `const protocol::Action&` | 轨迹动作结构体 |

**Action 结构体**：

```
struct Action {
    double trajectory_reference_time{1.0};

    std::optional<ArmControl> left_arm;
    std::optional<ArmControl> right_arm;
    std::optional<std::vector<std::vector<double>>> head;
    std::optional<WaistControl> waist;
    std::optional<std::vector<std::vector<double>>> left_effector;
    std::optional<std::vector<std::vector<double>>> right_effector;
    std::optional<std::string> frame_id;
};

struct ArmControl {
    std::string kind;  // "EEF_ABS", "EEF_DELTA", "JOINT_ABS", "JOINT_DELTA", "EEF_FORCE"
    std::vector<std::vector<double>> values;
};

struct WaistControl {
    std::string kind;  // "ABS_POSE", "ABS_JOINT"
    std::vector<std::vector<double>> values;
};
```

- `trajectory_reference_time` 表示从机器人当前位置执行该轨迹所使用的参考时间（秒）。当轨迹较长时，应适当增大该值；如果设置过小，机器人可能以过快速度执行轨迹，存在碰撞风险。
- `JOINT_ABS` 表示绝对关节角控制，`ArmControl::values` 中的每一项为一组关节角。
- `EEF_ABS` 表示绝对末端位姿控制，`ArmControl::values` 中的每一项按 `{x, y, z, qx, qy, qz, qw}` 排列；前 3 项为末端位置，后 4 项为 `xyzw` 顺序的四元数。该模式仅手部控制支持（对应 `left_arm` / `right_arm`）。
- `frame_id` 用于指定目标坐标系；不填写时默认为 `base_link`。
- **返回值**：`GDKRes`，成功返回 `GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <chrono>
#include <thread>

int main() {
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        return -1;
    }

    agibot::gdk::Env env;

    agibot::gdk::protocol::Action action;
    action.trajectory_reference_time = 1.0;
    action.left_arm = agibot::gdk::protocol::ArmControl{
        "JOINT_ABS",
        {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}
    };
    action.right_arm = agibot::gdk::protocol::ArmControl{
        "JOINT_ABS",
        {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}
    };

    if (env.ExecuteTrajectory(action) != agibot::gdk::GDKRes::kSuccess) {
        // 处理失败
    }

    agibot::gdk::GDKRelease();
    return 0;
}
```

#### 4. `SetReferenceFramePoses()`

- **函数签名**：

```
GDKRes SetReferenceFramePoses(
    const std::string& l_reference_frame_name,
    const std::string& r_reference_frame_name,
    double l_reference_frame_orientation_x,
    double l_reference_frame_orientation_y,
    double l_reference_frame_orientation_z,
    double l_reference_frame_orientation_w,
    double r_reference_frame_orientation_x,
    double r_reference_frame_orientation_y,
    double r_reference_frame_orientation_z,
    double r_reference_frame_orientation_w);
```

- **功能**：设置左右末端执行器参考坐标系名称及姿态（四元数 x,y,z,w），用于后续解算
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `l_reference_frame_name` | `const std::string&` | 左臂参考坐标系名称 |
| `r_reference_frame_name` | `const std::string&` | 右臂参考坐标系名称 |
| `l_reference_frame_orientation_x` | `double` | 左末端参考系四元数 x |
| `l_reference_frame_orientation_y` | `double` | 左末端参考系四元数 y |
| `l_reference_frame_orientation_z` | `double` | 左末端参考系四元数 z |
| `l_reference_frame_orientation_w` | `double` | 左末端参考系四元数 w |
| `r_reference_frame_orientation_x` | `double` | 右末端参考系四元数 x |
| `r_reference_frame_orientation_y` | `double` | 右末端参考系四元数 y |
| `r_reference_frame_orientation_z` | `double` | 右末端参考系四元数 z |
| `r_reference_frame_orientation_w` | `double` | 右末端参考系四元数 w |

- **返回值**：`GDKRes`，成功返回 `GDKRes::kSuccess`
- **示例**：

```
// 设置左右末端参考系名称及姿态（单位四元数）
env.SetReferenceFramePoses("arm_l_link3", "arm_r_link3",
                          0.0, 0.0, 0.0, 1.0,
                          0.0, 0.0, 0.0, 1.0);
```

## 使用注意事项

1. **GDK 初始化**：使用 Env 前必须先调用 `GDKInit()` 初始化 GDK 系统
2. **GDK 释放**：程序结束前必须调用 `GDKRelease()` 释放资源
3. **协议类型**：C++ 使用 `agibot::gdk::protocol` 命名空间下的 `Observation`、`Action` 等结构体
4. **可选字段**：Observation 与 Action 中的 optional 成员需按需填充

## 应用场景

- **强化学习**：作为 Gym-like 环境的 step 接口
- **模仿学习**：采集 (observation, action) 数据对
- **遥操作**：实时获取观测并下发动作
- **算法验证**：快速搭建 obs–action 闭环测试



---

<!-- source: contents_reference_C++_imu -->

# IMU

# GDK [IMU](../../../concepts/sensors/#imu) 接口文档（C++）

## 概述

IMU（惯性测量单元）模块为G02机器人提供了获取实时惯性数据的功能。通过C++接口，开发者可以方便地获取机器人的方向、角速度和线性加速度信息，适用于姿态检测、运动分析、导航等多种场景。

## 接口说明

### Imu 类

该类封装了IMU传感器的主要数据获取接口。

#### 构造函数 / 初始化

- **无参构造**：`Imu()`
  启动所有 IMU
- **指定 IMU 构造**：`Imu(imu_types)`
  仅启动传入的 IMU 类型，可减少资源占用。

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `imu_types` | `const std::vector<ImuType>&` | 要启动的 IMU 类型列表；默认空表示启动全部 IMU |

**示例**：

```
#include "gdk/gdk.h"

// 启动所有 IMU（默认）
agibot::gdk::Imu imu;

// 仅启动前部与底盘 IMU
// std::vector<agibot::gdk::ImuType> types = {
//     agibot::gdk::ImuType::kImuFront,
//     agibot::gdk::ImuType::kImuChassis,
// };
// agibot::gdk::Imu imu(types);
```

#### 1. `GetLatestImu()`

- **功能**：获取最新的IMU数据
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `imu_type` | `const ImuType&` | IMU类型枚举值 |
| `timeout_ms` | `const float` | 超时时间（毫秒） |
| `imu` | `std::shared_ptr<ImuData>&` | 输出参数，IMU数据指针 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`imu`参数包含IMU数据

#### ImuData对象详细说明

**ImuData结构体包含以下成员**：

```
struct ImuData {
  Vector3 angular_velocity{};     ///< imu current angular velocity
  Vector3 linear_acceleration{}; ///< imu current linear acceleration
  uint64_t timestamp_ns{0};      ///< imu timestamp(ns)
};
```

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `angular_velocity` | `Vector3` | 角速度，机器人在三个轴上的角速度 | 弧度/秒 |
| `linear_acceleration` | `Vector3` | 线性加速度，机器人在三个轴上的线性加速度 | 米/秒² |
| `timestamp_ns` | `uint64_t` | 数据采集的时间戳，精度为纳秒 | 纳秒 |
| **Vector3 结构体说明**： |  |  |  |

| 成员名 | 类型 | 描述 |
| --- | --- | --- |
| `x` | `double` | X轴分量 |
| `y` | `double` | Y轴分量 |
| `z` | `double` | Z轴分量 |

```
struct Vector3 {
  double x{};
  double y{};
  double z{};
};
```

**IMU类型**：
- `ImuType::kImuFront`: 前部IMU
- `ImuType::kImuBack`: 后部IMU
- `ImuType::kImuChassis`: 底盘IMU
- `ImuType::kImuHeadRgbd`: 头部[RGBD](../../../concepts/sensors/#rgbd) IMU

- **示例**：

```
#include <iostream>
#include <chrono>
#include <thread>
#include "gdk/gdk.h"

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    std::cout<< "IMU示例程序" << std::endl;
    agibot::gdk::Imu imu;
    std::shared_ptr<agibot::gdk::ImuData> imu_data;
    imu.GetLatestImu(agibot::gdk::ImuType::kImuChassis, 500.0, imu_data);

    if (imu_data != nullptr) {
        std::cout << "\n--- IMU数据 ---" << std::endl;
        std::cout << "时间戳: " << imu_data->timestamp_ns << std::endl;

        // 角速度
        std::cout << "角速度: x=" << imu_data->angular_velocity.x << ", "
              << "y=" << imu_data->angular_velocity.y << ", "
              << "z=" << imu_data->angular_velocity.z << std::endl;

        // 线性加速度
        std::cout << "线性加速度: x=" << imu_data->linear_acceleration.x << ", "
              << "y=" << imu_data->linear_acceleration.y << ", "
              << "z=" << imu_data->linear_acceleration.z << std::endl;
    } else {
        std::cout << "未收到IMU数据" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 2. `GetNearestImu()`

- **功能**：获取指定时间戳附近最近的IMU数据
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `imu_type` | `const ImuType&` | IMU类型枚举值 |
| `timestamp_ns` | `const uint64_t` | 目标时间戳（纳秒） |
| `timeout_ms` | `const float` | 超时时间（毫秒） |
| `imu` | `std::shared_ptr<ImuData>&` | 输出参数，IMU数据指针 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`imu`参数包含IMU数据
- **示例**：

```
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include "gdk/gdk.h"

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    std::cout << "IMU示例程序" << std::endl;
    agibot::gdk::Imu imu;

    agibot::gdk::ImuType imu_type = agibot::gdk::ImuType::kImuChassis;
    std::shared_ptr<agibot::gdk::ImuData> imu_data;
    imu.GetLatestImu(imu_type, 500.0, imu_data);

    if (imu_data != nullptr) {
        std::cout << "\n--- IMU数据 ---" << std::endl;
        std::cout << "时间戳: " << imu_data->timestamp_ns << std::endl;

        // 角速度
        std::cout << "角速度: x=" << imu_data->angular_velocity.x << ", "
                  << "y=" << imu_data->angular_velocity.y << ", "
                  << "z=" << imu_data->angular_velocity.z << std::endl;

        // 线性加速度
        std::cout << "线性加速度: x=" << imu_data->linear_acceleration.x << ", "
                  << "y=" << imu_data->linear_acceleration.y << ", "
                  << "z=" << imu_data->linear_acceleration.z << std::endl;

        // 查找最近的IMU数据
        for (int i = 0; i < 10; ++i) {
            std::shared_ptr<agibot::gdk::ImuData> imu_data_nearest;
            agibot::gdk::GDKRes res = imu.GetNearestImu(
                imu_type,
                imu_data->timestamp_ns - 1000000000LL, // 往前1秒
                1000.0,
                imu_data_nearest
            );
            if (res == agibot::gdk::GDKRes::kSuccess && imu_data_nearest != nullptr) {
                std::cout << "✅ 最近IMU数据: " << imu_data_nearest->timestamp_ns << std::endl;
                std::cout << std::fixed << std::setprecision(4);
                std::cout << "角速度: x=" << imu_data_nearest->angular_velocity.x
                          << ", y=" << imu_data_nearest->angular_velocity.y
                          << ", z=" << imu_data_nearest->angular_velocity.z << std::endl;
                std::cout << "线性加速度: x=" << imu_data_nearest->linear_acceleration.x
                          << ", y=" << imu_data_nearest->linear_acceleration.y
                          << ", z=" << imu_data_nearest->linear_acceleration.z << std::endl;
            } else {
                std::cout << "❌ 未找到最近的 IMU 数据" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    } else {
        std::cout << "未收到IMU数据" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 3. `GetImuFps()`

- **功能**：获取IMU数据采集帧率
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `imu_type` | `const ImuType&` | IMU类型枚举值 |
| `fps` | `int&` | 输出参数，IMU帧率（FPS） |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`fps`参数包含帧率值
- **示例**：

```
#include <iostream>
#include <chrono>
#include <thread>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Imu imu;

    agibot::gdk::ImuType imu_type = agibot::gdk::ImuType::kImuChassis;

    float fps;
    if (imu.GetImuFps(imu_type, fps) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to get imu fps" << std::endl;
    } else {
        std::cout << "IMU fps: " << fps << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 4. `GetImuLatency()`

- **注意事项**：获取IMU数据延迟统计信息前，需要先进行时间同步，否则延迟统计结果不准确
- **功能**：获取IMU数据延迟统计信息
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `imu_type` | `const ImuType&` | IMU类型枚举值 |
| `window_seconds` | `const float` | 统计窗口时间（秒） |
| `latency` | `LatencyStats&` | 输出参数，延迟统计信息 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`latency`参数包含延迟统计信息
- **LatencyStats结构体说明**：

```
struct LatencyStats {
  double max_latency_ms{0.0};    ///< max latency(ms)
  double avg_latency_ms{0.0};    ///< average latency(ms)
  double p99_latency_ms{0.0};    ///< 99th percentile latency(ms)
  double p999_latency_ms{0.0};   ///< 99.9th percentile latency(ms)
  double p9999_latency_ms{0.0};  ///< 99.99th percentile latency(ms)
};
```

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `max_latency_ms` | `double` | 最大延迟 | 毫秒 |
| `avg_latency_ms` | `double` | 平均延迟 | 毫秒 |
| `p99_latency_ms` | `double` | 99分位延迟 | 毫秒 |
| `p999_latency_ms` | `double` | 99.9分位延迟 | 毫秒 |
| `p9999_latency_ms` | `double` | 99.99分位延迟 | 毫秒 |

- **示例**：

```
#include <iostream>
#include <chrono>
#include <thread>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Imu imu;

    agibot::gdk::ImuType imu_type = agibot::gdk::ImuType::kImuChassis;

    agibot::gdk::LatencyStats latency;
    if (imu.GetImuLatency(imu_type, 1.0, latency) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to get imu latency" << std::endl;
    } else {
        std::cout << "IMU latency stats:" << std::endl;
        std::cout << "  Max latency: " << latency.max_latency_ms << "ms" << std::endl;
        std::cout << "  Average latency: " << latency.avg_latency_ms << "ms" << std::endl;
        std::cout << "  P99 latency: " << latency.p99_latency_ms << "ms" << std::endl;
        std::cout << "  P99.9 latency: " << latency.p999_latency_ms << "ms" << std::endl;
        std::cout << "  P99.99 latency: " << latency.p9999_latency_ms << "ms" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 5. `CloseImu()`

- **功能**：关闭IMU DDS连接
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include <iostream>
#include <chrono>
#include <thread>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Imu imu;
    std::cout << "IMU init" << std::endl;

    // 使用IMU...

    // 关闭IMU
    if (imu.CloseImu() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to close imu" << std::endl;
    } else {
        std::cout << "IMU closed successfully" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

## 使用注意事项

1. **GDK初始化**：使用IMU功能前必须先调用`agibot::gdk::GDKInit()`初始化GDK系统
2. **GDK释放**：程序结束前必须调用`agibot::gdk::GDKRelease()`释放GDK系统资源
3. **超时设置**：根据实际需求设置合适的超时时间，避免长时间阻塞
4. **返回值检查**：使用前请检查GDKRes返回值是否为kSuccess
5. **智能指针管理**：ImuData对象使用shared\_ptr管理，注意生命周期
6. **时间戳精度**：时间戳单位为纳秒，可用于精确的时间同步
7. **数据融合**：IMU数据通常需要与其他传感器数据进行融合以提高精度
8. **资源释放**：使用完毕后调用`CloseImu()`释放资源
9. **错误处理**：始终检查GDKRes返回值，确保操作成功
10. **未实现方法**：`GetImuFps()`和`GetImuLatency()`当前未实现，使用时需注意

## 应用场景

- **运动分析**：利用角速度和线性加速度分析机器人运动状态
- **导航定位**：结合其他传感器进行机器人定位和导航
- **平衡控制**：用于机器人的平衡和稳定性控制
- **数据融合**：与其他传感器数据进行卡尔曼滤波等融合算法
- **运动预测**：基于历史数据预测机器人运动轨迹
- **异常检测**：检测机器人的异常运动状态
- **校准补偿**：进行传感器校准和误差补偿
- **振动监测**：监测机器人的振动和冲击情况



---

<!-- source: contents_reference_C++_interaction -->

# Interaction

# GDK 交互接口文档（C++）

## 概述

交互模块为G02机器人提供了语音交互、显示控制、音频播放等功能。通过C++接口，开发者可以方便地实现对机器人的语音控制、TTS播放、音频视频播放、显示控制等功能，适用于语音交互、多媒体展示、人机交互等多种场景。

## 注意事项

交互接口依赖跨网段通信模块，因此如果在隔离环境中使用该类接口（如docker容器），需要确保容器可以修改宿主机的网络配置。（启动容器时添加参数‘--privileged’）

## 接口说明

### Interaction 类

该类封装了机器人交互的主要功能接口。

#### 1. `SetLanguage()`

- **功能**：设置语音语言
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `language` | `const Language&` | 语言类型，`Language::kLanguageChinese` 表示中文，`Language::kLanguageEnglish` 表示英文 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`

**Language枚举值**：

| 枚举值 | 描述 |
| --- | --- |
| `Language::kLanguageChinese` | 中文 |
| `Language::kLanguageEnglish` | 英文 |

```
enum class Language {
  kLanguageChinese = 0,
  kLanguageEnglish = 1,
};
```

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Interaction interaction;
    std::cout << "Interaction init" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 设置语言为中文
    if (interaction.SetLanguage(agibot::gdk::Language::kLanguageChinese) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "设置语言失败" << std::endl;
    } else {
        std::cout << "设置语言成功" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 2. `SetCallMode()`

- **功能**：设置通话模式
- **参数**：
  | 参数名 | 类型 | 描述 |
  | :--- | :--- | :--- |
  | `is_on` | `const bool&` | `true` 表示开启通话模式，`false` 表示关闭通话模式 |
- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **注意事项**：
- 开启通话模式后，打开降噪模式，自动关闭唤醒状态，无法获取asr文本
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Interaction interaction;

    // 开启通话模式
    bool is_on = true;
    if (interaction.SetCallMode(is_on) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "开启通话模式失败" << std::endl;
    } else {
        std::cout << "开启通话模式成功" << std::endl;
    }

    is_on = false;
    if (interaction.SetCallMode(is_on) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "关闭通话模式失败" << std::endl;
    } else {
        std::cout << "关闭通话模式成功" << std::endl;
    }

    agibot::gdk::GDKRelease();
    return 0;
}
```

#### 3. `SetVolume()`

- **功能**：设置音量
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `volume` | `const int32_t&` | 音量值，范围通常为0-500 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Interaction interaction;

    // 设置音量为50
    int32_t volume = 50;
    if (interaction.SetVolume(volume) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "设置音量失败" << std::endl;
    } else {
        std::cout << "设置音量成功" << std::endl;
    }

    agibot::gdk::GDKRelease();
    return 0;
}
```

#### 4. `SetWakeupSwitch()`

- **功能**：设置唤醒开关
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `is_on` | `const bool&` | `true` 表示开启唤醒，`false` 表示关闭唤醒 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Interaction interaction;

    // 开启唤醒功能
    bool is_on = true;
    if (interaction.SetWakeupSwitch(is_on) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "设置唤醒开关失败" << std::endl;
    } else {
        std::cout << "开启唤醒功能成功" << std::endl;
    }

    agibot::gdk::GDKRelease();
    return 0;
}
```

#### 5. `SetAudioSwitch()`

- **功能**：设置音频开关
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `is_on` | `const bool&` | `true` 表示开启音频，`false` 表示关闭音频 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Interaction interaction;

    // 开启音频功能
    bool is_on = true;
    if (interaction.SetAudioSwitch(is_on) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "设置音频开关失败" << std::endl;
    } else {
        std::cout << "开启音频功能成功" << std::endl;
    }

    agibot::gdk::GDKRelease();
    return 0;
}
```

#### 6. `SetDisplaySwitch()`

- **功能**：设置显示开关
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `is_on` | `const bool&` | `true` 表示开启显示，`false` 表示关闭显示 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Interaction interaction;

    // 开启显示功能
    bool is_on = true;
    if (interaction.SetDisplaySwitch(is_on) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "设置显示开关失败" << std::endl;
    } else {
        std::cout << "开启显示功能成功" << std::endl;
    }

    agibot::gdk::GDKRelease();
    return 0;
}
```

#### 7. `PlayTts()`

- **功能**：播放TTS（文本转语音）
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `text` | `const std::string&` | 要播放的文本内容 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Interaction interaction;

    // 播放TTS
    std::string text = "你好，我是精灵G2";
    if (interaction.PlayTts(text) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "播放TTS失败" << std::endl;
    } else {
        std::cout << "TTS播放成功" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));  // 等待播放完成
    }

    agibot::gdk::GDKRelease();
    return 0;
}
```

#### 8. `PlayAudio()`

- **功能**：播放音频文件
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `audio_path` | `const std::string&` | 音频文件路径 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Interaction interaction;

    // 播放音频文件
    // 音频文件需要使用官方工具配置路径，可联系运维人员获取工具，使用方法可见附录中data_push使用说明
    std::string audio_path = "/path/to/audio.wav";
    if (interaction.PlayAudio(audio_path) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "播放音频失败" << std::endl;
    } else {
        std::cout << "音频播放成功" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(5));  // 等待播放完成
    }

    agibot::gdk::GDKRelease();
    return 0;
}
```

#### 9. `PlayVideo()`

- **功能**：播放视频文件
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `video_path` | `const std::string&` | 视频文件路径 |
| `loop_count` | `const int32_t&` | 循环播放次数，-1表示无限循环 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Interaction interaction;

    // 播放视频文件，循环1次
    // 音频文件需要使用官方工具配置路径，可联系运维人员获取工具，使用方法可见附录中data_push使用说明
    std::string video_path = "/path/to/video.mp4";
    int32_t loop_count = 1;
    if (interaction.PlayVideo(video_path, loop_count) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "播放视频失败" << std::endl;
    } else {
        std::cout << "视频播放成功" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(10));  // 等待播放完成
    }

    // 无限循环播放
    loop_count = -1;
    if (interaction.PlayVideo(video_path, loop_count) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "播放视频失败" << std::endl;
    } else {
        std::cout << "视频开始循环播放" << std::endl;
    }

    agibot::gdk::GDKRelease();
    return 0;
}
```

#### 10. `GetFuncStatus()`

- **功能**：获取语音功能状态
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `func_status` | `VoiceFuncStatus&` | 输出参数，语音功能状态对象 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`func_status`参数包含功能状态信息

#### VoiceFuncStatus对象详细说明

**VoiceFuncStatus结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `func_status` | `uint32_t` | 功能状态 | 无单位 |
| `wakeup_status` | `uint32_t` | 唤醒状态 | 无单位 |
| `requester` | `std::string` | 请求者 | 字符串 |
| `wakeup_enabled` | `bool` | 唤醒功能是否启用 | 布尔值 |
| `display_enabled` | `bool` | 显示功能是否启用 | 布尔值 |
| `audio_enabled` | `bool` | 音频功能是否启用 | 布尔值 |
| `en_settings` | `VoiceSettings` | 英文语音设置 | 语音设置 |
| `cn_settings` | `VoiceSettings` | 中文语音设置 | 语音设置 |
| `timestamp` | `uint64_t` | 时间戳 | 纳秒 |

**func\_status**：
| 值 | 描述 |
| :--- | :--- |
| 0 | 空闲 |
| 1 | 电话模式 |
| 2 | 语音唤醒 + 自由问答 |
| 3 | 语音唤醒 + ASR |
| 4 | 语音播报 |
| 5 | 多轮语音交互 |
| 6 | 音频播放 |
| 9 | 异常 |

**wakeup\_status**：
| 值 | 描述 |
| :--- | :--- |
| 0 | 未唤醒 |
| 1 | 唤醒后倾听 |
| 2 | 唤醒后思考 |
| 4 | 唤醒后播报 |

```
struct VoiceFuncStatus {
  uint32_t func_status{0};
  uint32_t wakeup_status{0};
  std::string requester{};
  bool wakeup_enabled{false};
  bool display_enabled{false};
  bool audio_enabled{false};
  VoiceSettings en_settings{};
  VoiceSettings cn_settings{};
  uint64_t timestamp{0};
};
```

**VoiceSettings结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `volume` | `uint32_t` | 音量 | 无单位 |
| `speech_rate` | `float` | 语速 | 无单位 |
| `voice_tone` | `std::string` | 音色 | 字符串 |
| `is_curr_setting` | `bool` | 是否为当前设置 | 布尔值 |

```
struct VoiceSettings {
  uint32_t volume{0};
  float speech_rate{0.0};
  std::string voice_tone{};
  bool is_curr_setting{false};
};
```

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Interaction interaction;

    agibot::gdk::VoiceFuncStatus func_status;
    if (interaction.GetFuncStatus(func_status) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "获取功能状态失败" << std::endl;
    } else {
        std::cout << "功能状态信息:" << std::endl;
        std::cout << "  功能状态: " << func_status.func_status << std::endl;
        std::cout << "  唤醒状态: " << func_status.wakeup_status << std::endl;
        std::cout << "  请求者: " << func_status.requester << std::endl;
        std::cout << "  唤醒功能启用: " << (func_status.wakeup_enabled ? "是" : "否") << std::endl;
        std::cout << "  显示功能启用: " << (func_status.display_enabled ? "是" : "否") << std::endl;
        std::cout << "  音频功能启用: " << (func_status.audio_enabled ? "是" : "否") << std::endl;
        std::cout << "  中文设置 - 音量: " << func_status.cn_settings.volume
                  << ", 语速: " << func_status.cn_settings.speech_rate
                  << ", 音色: " << func_status.cn_settings.voice_tone << std::endl;
        std::cout << "  英文设置 - 音量: " << func_status.en_settings.volume
                  << ", 语速: " << func_status.en_settings.speech_rate
                  << ", 音色: " << func_status.en_settings.voice_tone << std::endl;
        std::cout << "  时间戳: " << func_status.timestamp << std::endl;
    }

    agibot::gdk::GDKRelease();
    return 0;
}
```

#### 11. `GetAsrText()`

- **功能**：获取ASR（自动语音识别）文本
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `asr_text` | `std::string&` | 输出参数，识别到的文本内容 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`asr_text`参数包含识别到的文本
- **注意事项**：
- 在唤醒状态下，且语音唤醒之后，可以获取asr文本
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Interaction interaction;

    std::string asr_text;
    if (interaction.GetAsrText(asr_text) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "获取ASR文本失败" << std::endl;
    } else {
        std::cout << "识别到的文本: " << asr_text << std::endl;
    }

    agibot::gdk::GDKRelease();
    return 0;
}
```

#### 12. `RegisterCallback()`

- **功能**：注册回调函数
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `type` | `const std::string&` | 回调类型 |
| `callback` | `std::function<void(const std::any&)>` | 回调函数 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **注意事项**：
- 在唤醒状态下，且语音唤醒之后，可以获取asr文本
- **当前支持的回调类型**：
- `get_asr_text`：回调函数参数类型为`const std::string&`，唤醒状态下，识别到新的语音输入时，会调用回调函数，并传递识别到的文本内容
- 其余功能回调暂不支持

#### 13. `UnregisterCallback()`

- **功能**：注销回调函数
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `type` | `const std::string&` | 回调类型 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **当前支持的回调类型**：
- `get_asr_text`：注销回调函数，注销后，唤醒状态下，识别到新的语音输入时，不会调用回调函数
- 其余功能回调暂不支持
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>

std::atomic<bool> g_running{true};

void signal_handler(int signum) {
  std::cout << "收到中断信号" << std::endl;
  g_running = false;
}

int main()
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Interaction interaction;

    std::string asr_text;
    if (interaction.RegisterCallback("get_asr_text", [&asr_text](const std::any& data) {
        asr_text = std::any_cast<const std::string&>(data);
        std::cout << "ASR文本: " << asr_text << std::endl;
    }) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "注册回调失败" << std::endl;
    } else {
        std::cout << "注册回调成功" << std::endl;
    }

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    interaction.UnregisterCallback("get_asr_text");
    agibot::gdk::GDKRelease();
    return 0;
}
```

#### 13. `InitLedStripControl()`

- **功能**：申请灯带控制权，申请成功后调用 `SetLedStrips()` 下发的命令才会生效
- **参数**：无
- **返回值**：`GDKRes`，成功返回 `GDKRes::kSuccess`
- **注意事项**：
- 程序结束或不再需要控制灯带时，应调用 `ReleaseLedStripControl()` 释放控制权
- 获取灯带控制权后，此期间整机故障、急停、低电量等状态不会通过灯带或屏幕给出默认告警提示

#### 14. `ReleaseLedStripControl()`

- **功能**：释放灯带控制权。
- **参数**：无
- **返回值**：`GDKRes`，成功返回 `GDKRes::kSuccess`

#### 15. `SetLedStrips()`

- **功能**：批量下发灯带控制命令。共 7 路灯带可控，一次调用可独立配置 1–7 路灯带的模式、颜色和动效速度参数。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `led_strip_controls` | `const std::vector<LedStripControl>&` | 灯带控制项列表，长度 1–7 |

- **返回值**：`GDKRes`，成功返回 `GDKRes::kSuccess`
- **注意事项**：
- 列表长度必须在 `[1, 7]` 之间；空列表或超过 7 项均返回 `GDKRes::kInvalidInput`
- 每个控制项只对自己的 `strip_id` 生效；如需让多路灯带同时显示同一效果，逐路下发即可
- `mode` 取值非法会返回 `GDKRes::kInvalidInput`
- 在调用前需要先获取控制权

**LedStripControl结构体详细说明**

单路灯带控制项。一次 `SetLedStrips()` 调用最多包含 7 项（每路一条）。

| 成员名 | 类型 | 描述 | 取值范围 |
| --- | --- | --- | --- |
| `strip_id` | `LedStripId` | 灯带 ID | 见下表 |
| `mode` | `LedStripMode` | 显示模式 | 见下表 |
| `red` | `uint8_t` | 红色分量 | 0–255 |
| `green` | `uint8_t` | 绿色分量 | 0–255 |
| `blue` | `uint8_t` | 蓝色分量 | 0–255 |
| `speed` | `uint8_t` | 动画周期参数，单位 25 ms | 0–255（对应 0–6.375 s） |

```
struct LedStripControl {
  LedStripId strip_id;
  LedStripMode mode;
  uint8_t red{0};
  uint8_t green{0};
  uint8_t blue{0};
  uint8_t speed{0};  // 0~255 对应 0~6.375s，单位 25ms
};
```

**说明**：

- `speed` 仅对动效类模式（流水 / 反流水 / 三角波呼吸 / 正弦呼吸 / 闪烁）起作用；取值 0–255，对应 0–6.375 s，单位 25 ms；常亮模式可设为 0。流水 / 反流水模式下更容易观察到动画周期参数的变化；呼吸模式下变化可能不明显。

**LedStripId枚举详细说明**

灯带 ID，对应机器人本体上的 7 路灯带物理位置。

| 枚举值 | 数值 | 物理位置 |
| --- | --- | --- |
| `kChassisFront` | 1 | 底盘前 |
| `kChassisLeft` | 2 | 底盘左 |
| `kChassisRight` | 3 | 底盘右 |
| `kChestLeft` | 4 | 胸左 |
| `kChestRight` | 5 | 胸右 |
| `kBack` | 6 | 背部 |
| `kHead` | 7 | 头部 |

```
enum class LedStripId {
  kChassisFront = 1,
  kChassisLeft = 2,
  kChassisRight = 3,
  kChestLeft = 4,
  kChestRight = 5,
  kBack = 6,
  kHead = 7,
};
```

**LedStripMode枚举详细说明**

灯带显示模式。模式只对当前控制项中指定的 `strip_id` 生效；如需让所有灯带显示同一效果，逐路下发即可。

| 枚举值 | 数值 | 描述 |
| --- | --- | --- |
| `kHold` | 0 | 暂停/冻结当前显示帧 |
| `kSync` | 1 | 同步点亮（常亮） |
| `kSequence` | 2 | 流水 |
| `kReverseSequence` | 3 | 反流水 |
| `kTriangleBreath` | 4 | 三角波呼吸 |
| `kFlash` | 5 | 闪烁 |
| `kSineBreath` | 6 | 正弦呼吸 |

```
enum class LedStripMode {
  kHold = 0,
  kSync = 1,
  kSequence = 2,
  kReverseSequence = 3,
  kTriangleBreath = 4,
  kFlash = 5,
  kSineBreath = 6,
};
```

**说明**：

- `kHold` 模式下 RGB 数值不起作用；后续发送其他模式可正常恢复控制。
- 需要灭灯时，请使用 `kSync + RGB=(0,0,0)`。
- 三角波呼吸和正弦呼吸对红、绿、蓝、白表现正常；对黄、品红、青等双通道复合色会退化为单通道主色（如黄 R+G → 红），不建议在呼吸模式下使用双通道复合色。
- **示例：控制 7 路灯带**

```
#include "gdk/gdk.h"
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

int main()
{
    using namespace agibot::gdk;

    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    Interaction interaction;

    if (interaction.InitLedStripControl() != GDKRes::kSuccess) {
        std::cout << "获取灯带控制权失败" << std::endl;
        GDKRelease();
        return -1;
    }

    const std::vector<LedStripId> all_ids = {
        LedStripId::kChassisFront,
        LedStripId::kChassisLeft,
        LedStripId::kChassisRight,
        LedStripId::kChestLeft,
        LedStripId::kChestRight,
        LedStripId::kBack,
        LedStripId::kHead,
    };

    auto make_ctrl = [](LedStripId id, LedStripMode mode,
                        uint8_t r, uint8_t g, uint8_t b, uint8_t speed = 0) {
        LedStripControl c;
        c.strip_id = id;
        c.mode = mode;
        c.red = r; c.green = g; c.blue = b;
        c.speed = speed;
        return c;
    };

    auto make_uniform = [&](LedStripMode mode,
                            uint8_t r, uint8_t g, uint8_t b, uint8_t speed = 0) {
        std::vector<LedStripControl> ctrls;
        for (auto id : all_ids)
            ctrls.push_back(make_ctrl(id, mode, r, g, b, speed));
        return ctrls;
    };

    // 1. 7 路灯带独立配置：不同位置显示不同常亮颜色
    std::vector<LedStripControl> controls = {
        make_ctrl(LedStripId::kChassisFront, LedStripMode::kSync, 255,   0,   0),
        make_ctrl(LedStripId::kChassisLeft,  LedStripMode::kSync,   0, 255,   0),
        make_ctrl(LedStripId::kChassisRight, LedStripMode::kSync,   0,   0, 255),
        make_ctrl(LedStripId::kChestLeft,    LedStripMode::kSync, 255,   0,   0),
        make_ctrl(LedStripId::kChestRight,   LedStripMode::kSync,   0, 255,   0),
        make_ctrl(LedStripId::kBack,         LedStripMode::kSync,   0,   0, 255),
        make_ctrl(LedStripId::kHead,         LedStripMode::kSync, 255, 255, 255),
    };
    interaction.SetLedStrips(controls);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // 2. 全部 7 路显示同一效果：蓝色流水
    // speed 单位为 25 ms，流水 / 反流水模式下 speed 变化最易观察
    interaction.SetLedStrips(make_uniform(LedStripMode::kSequence, 0, 0, 255, 50));
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // 3. 全部 7 路熄灭：使用 kSync + RGB=(0,0,0)
    interaction.SetLedStrips(make_uniform(LedStripMode::kSync, 0, 0, 0));

    interaction.ReleaseLedStripControl();
    GDKRelease();
    return 0;
}
```

## 使用注意事项

1. **GDK初始化**：使用Interaction功能前必须先调用`agibot::gdk::GDKInit()`初始化GDK系统
2. **GDK释放**：程序结束前必须调用`agibot::gdk::GDKRelease()`释放GDK系统资源
3. **返回值检查**：所有接口都会返回`GDKRes`状态码，请及时检查返回值，确保操作成功
4. **文件路径**：播放音频和视频时，确保文件路径正确且文件存在
5. **语言设置**：设置语言后会影响TTS的语言，请根据实际需求设置
6. **音量范围**：设置音量时注意合理范围，避免过大或过小
7. **循环播放**：视频循环播放时，使用-1表示无限循环，注意及时停止
8. **状态查询**：定期查询功能状态，确保各功能正常工作
9. **ASR文本**：获取ASR文本时，需要确保语音识别功能已启用
10. **字符串参数**：传递字符串参数时，确保字符串有效且不为空
11. **错误处理**：始终检查GDKRes返回值，确保操作成功

## 应用场景

- **语音交互**：实现机器人的语音识别和语音合成功能
- **多媒体展示**：播放音频、视频内容，进行信息展示
- **人机交互**：通过语音和显示进行人机交互
- **状态监控**：监控语音功能状态，确保系统正常运行
- **多语言支持**：支持中英文切换，适应不同场景需求
- **音量控制**：动态调整音量，适应不同环境
- **功能开关**：灵活控制唤醒、音频、显示等功能开关



---

<!-- source: contents_reference_C++_lidar -->

# Lidar

# GDK Lidar 接口文档（C++）

## 概述

Lidar（激光雷达）模块为G02机器人提供了获取实时点云数据的功能。通过C++接口，开发者可以方便地获取机器人的环境感知数据，适用于SLAM建图、障碍物检测、导航避障、环境建模等多种场景。

## 接口说明

### Lidar 类

该类封装了激光雷达传感器的主要数据获取接口。

#### 构造函数 / 初始化

- **无参构造**：`Lidar()`
  启动所有激光雷达
- **指定雷达构造**：`Lidar(lidar_types)`
  仅启动传入的雷达类型，可减少资源占用。

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `lidar_types` | `const std::vector<LidarType>&` | 要启动的雷达类型列表；默认空表示启动全部雷达 |

**示例**：

```
#include "gdk/gdk.h"

// 启动所有雷达（默认）
agibot::gdk::Lidar lidar;

// 仅启动前部雷达
// std::vector<agibot::gdk::LidarType> types = {
//     agibot::gdk::LidarType::kLidarFront,
// };
// agibot::gdk::Lidar lidar(types);
```

#### 1. `GetLatestPointCloud()`

- **功能**：获取最新的点云数据
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `lidar_type` | `const LidarType&` | 激光雷达类型枚举值 |
| `timeout_ms` | `const float` | 超时时间（毫秒） |
| `pointcloud` | `std::shared_ptr<PointCloud>&` | 输出参数，点云数据指针 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`pointcloud`参数包含点云数据

#### PointCloud对象详细说明

**PointCloud结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `timestamp_ns` | `uint64_t` | 点云数据采集的时间戳 | 纳秒 |
| `width` | `int` | 点云的宽度（点数） | 点数 |
| `height` | `int` | 点云的高度（点数） | 点数 |
| `point_step` | `int` | 每个点占用的字节数 | 字节 |
| `row_step` | `int` | 每行占用的字节数 | 字节 |
| `is_bigendian` | `bool` | 数据是否为大端序 | 布尔值 |
| `is_dense` | `bool` | 是否为密集点云（无无效点） | 布尔值 |
| `fields` | `std::vector<PointField>` | 字段信息列表，定义点云中每个点的属性结构 | 无单位 |
| `data_view` | `DataView` | 点云的原始二进制数据视图 | 数据视图 |

```
struct PointCloud {
 int width{0};   ///< point cloud width
 int height{0};  ///< point cloud height

 std::vector<PointField> fields{};  ///< point cloud fields
 int point_step{0};                 ///< point step
 int row_step{0};                   ///< row step
 bool is_bigendian{false};          ///< is bigendian
 bool is_dense{true};               ///< is dense

 DataView data_view{};      ///< point cloud data view
 uint64_t timestamp_ns{0};  ///< point cloud timestamp(ns)
};
```

**fields (字段信息列表)**：

每个字段包含以下属性：

| 成员名 | 类型 | 描述 |
| --- | --- | --- |
| `name` | `std::string` | 字段名称（如"x", "y", "z", "intensity"） |
| `offset` | `uint32_t` | 在点数据中的偏移量 |
| `datatype` | `uint8_t` | 数据类型 |
| `count` | `uint32_t` | 该字段的元素数量 |

```
struct PointField {
 std::string name{};   // point field name (x, y, z, intensity, etc.)
 uint32_t offset{0};   // point field offset
 uint8_t datatype{0};  // point field datatype
 uint32_t count{1};    // point field count
};
```

**data\_view (原始数据)**：

- **类型**：`DataView`
- **描述**：点云的原始二进制数据视图
- **注意**：需要根据fields信息进行解析

```
class DataView {
public:
 /// @enum OwnershipType
 /// @brief ownership type
 /// @details ownership type related information
 enum class OwnershipType { OWNED, BORROWED };

 DataView() = default;

 DataView(const void* data, size_t size);

 explicit DataView(const std::vector<uint8_t>& vec);

 DataView(const DataView& other);

 DataView& operator=(const DataView& other);
 DataView(DataView&& other) noexcept;
 DataView& operator=(DataView&& other) noexcept;

 /// @brief data
 /// @details use to get the data of the data view
 /// @return the data of the data view
 const uint8_t* data() const { return data_; }

 /// @brief mutable_data
 /// @details use to get the mutable data of the data view
 /// @return the mutable data of the data view
 uint8_t* mutable_data() { return const_cast<uint8_t*>(data_); }

 /// @brief size
 /// @details use to get the size of the data view
 /// @return the size of the data view
 size_t size() const { return size_; }

 /// @brief IsOwned
 /// @details use to check if the data view is owned
 /// @return true if the data view is owned, false otherwise
 bool IsOwned() const { return ownership_ == OwnershipType::OWNED; }

 /// @brief Clone
 /// @details use to clone the data view
 /// @return the cloned data view
 DataView Clone() const;

 /// @brief CreateOwnedFrom
 /// @details use to create a data view from owned data
 /// @param data the data of the data view, input parameter
 /// @param size the size of the data view, input parameter
 /// @return the created data view
 static DataView CreateOwnedFrom(const void* data, size_t size);

 /// @brief AssignOwnedData
 /// @details use to assign owned data to the data view
 /// @param data the data of the data view, input parameter
 /// @param size the size of the data view, input parameter
 void AssignOwnedData(const void* data, size_t size);

private:
 const uint8_t* data_ = nullptr;
 size_t size_ = 0;
 OwnershipType ownership_ = OwnershipType::BORROWED;
 std::vector<uint8_t> owned_buffer_;
};
```

**激光雷达类型**：
- `LidarType::kLidarFront`: 前部激光雷达
- `LidarType::kLidarBack`: 后部激光雷达

- **示例**：

```
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Lidar lidar;

    // 可选类型: kLidarFront（前部雷达）、kLidarBack（后部雷达）
    agibot::gdk::LidarType lidar_type = agibot::gdk::LidarType::kLidarFront;
    std::shared_ptr<agibot::gdk::PointCloud> pointcloud = std::make_shared<agibot::gdk::PointCloud>();
    lidar.GetLatestPointCloud(lidar_type, 500.0, pointcloud);

    if (pointcloud != nullptr) {
        std::cout << "✅ 时间戳: " << pointcloud->timestamp_ns << std::endl;
        std::cout << "点云尺寸: " << pointcloud->width << " x " << pointcloud->height << std::endl;
        std::cout << "点步长: " << pointcloud->point_step << std::endl;
        std::cout << "行步长: " << pointcloud->row_step << std::endl;
        std::cout << "是否大端序: " << (pointcloud->is_bigendian ? "是" : "否") << std::endl;
        std::cout << "是否密集: " << (pointcloud->is_dense ? "是" : "否") << std::endl;

        // 打印字段信息
        std::cout << "字段数量: " << pointcloud->fields.size() << std::endl;
        for (size_t j = 0; j < pointcloud->fields.size(); ++j) {
            const auto& field = pointcloud->fields[j];
            std::cout << "  字段 " << (j + 1) << ": " << field.name
                    << " (偏移: " << field.offset
                    << ", 类型: " << field.datatype
                    << ", 数量: " << field.count << ")" << std::endl;
        }
    } else {
        std::cout << "未获取到点云数据" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 2. `GetNearestPointCloud()`

- **功能**：获取指定时间戳附近最近的点云数据
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `lidar_type` | `const LidarType&` | 激光雷达类型枚举值 |
| `timestamp_ns` | `const uint64_t` | 目标时间戳（纳秒） |
| `timeout_ms` | `const float` | 超时时间（毫秒） |
| `pointcloud` | `std::shared_ptr<PointCloud>&` | 输出参数，点云数据指针 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`pointcloud`参数包含点云数据
- **示例**：

```
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Lidar lidar;

    // 可选类型: kLidarFront（前部雷达）、kLidarBack（后部雷达）
    agibot::gdk::LidarType lidar_type = agibot::gdk::LidarType::kLidarFront;
    std::shared_ptr<agibot::gdk::PointCloud> pointcloud = std::make_shared<agibot::gdk::PointCloud>();
    lidar.GetLatestPointCloud(lidar_type, 500.0, pointcloud);

    if (pointcloud != nullptr) {
        std::cout << "✅ 时间戳: " << pointcloud->timestamp_ns << std::endl;
        std::cout << "点云尺寸: " << pointcloud->width << " x " << pointcloud->height << std::endl;
        std::cout << "点步长: " << pointcloud->point_step << std::endl;
        std::cout << "行步长: " << pointcloud->row_step << std::endl;
        std::cout << "是否大端序: " << (pointcloud->is_bigendian ? "是" : "否") << std::endl;
        std::cout << "是否密集: " << (pointcloud->is_dense ? "是" : "否") << std::endl;

        // 打印字段信息
        std::cout << "字段数量: " << pointcloud->fields.size() << std::endl;
        for (size_t j = 0; j < pointcloud->fields.size(); ++j) {
            const auto& field = pointcloud->fields[j];
            std::cout << "  字段 " << (j + 1) << ": " << field.name
                    << " (偏移: " << field.offset
                    << ", 类型: " << field.datatype
                    << ", 数量: " << field.count << ")" << std::endl;
        }

        // 查找最近的点云数据
        std::shared_ptr<agibot::gdk::PointCloud> pointcloud_nearest = std::make_shared<agibot::gdk::PointCloud>();
        agibot::gdk::GDKRes res = lidar.GetNearestPointCloud(
            lidar_type,
            pointcloud->timestamp_ns - 1000000000LL, // 往前1秒
            1000.0,
            pointcloud_nearest
        );
        if (res == agibot::gdk::GDKRes::kSuccess && pointcloud_nearest != nullptr) {
            std::cout << "✅ 最近点云数据: " << pointcloud_nearest->timestamp_ns << std::endl;
            std::cout << "点云尺寸: " << pointcloud_nearest->width << " x " << pointcloud_nearest->height << std::endl;
        } else {
            std::cout << "❌ 未找到最近的 Front 雷达数据" << std::endl;
        }
    } else {
        std::cout << "未获取到点云数据" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 3. `GetLidarFps()`

- **功能**：获取激光雷达数据采集帧率
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `lidar_type` | `const LidarType&` | 激光雷达类型枚举值 |
| `fps` | `float&` | 输出参数，激光雷达帧率（FPS） |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`fps`参数包含帧率值
- **示例**：

```
#include <iostream>
#include <chrono>
#include <thread>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Lidar lidar;

    agibot::gdk::LidarType lidar_type = agibot::gdk::LidarType::kLidarFront;

    float fps;
    if (lidar.GetLidarFps(lidar_type, fps) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to get lidar fps" << std::endl;
    } else {
        std::cout << "Lidar fps: " << fps << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 4. `GetLidarLatency()`

- **注意事项**：获取激光雷达数据延迟统计信息前，需要先进行时间同步，否则延迟统计结果不准确
- **功能**：获取激光雷达数据延迟统计信息
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `lidar_type` | `const LidarType&` | 激光雷达类型枚举值 |
| `window_seconds` | `const float` | 统计窗口时间（秒） |
| `latency` | `LatencyStats&` | 输出参数，延迟统计信息 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`latency`参数包含延迟统计信息
- **LatencyStats结构体说明**：

```
struct LatencyStats {
  double max_latency_ms{0.0};    ///< max latency(ms)
  double avg_latency_ms{0.0};    ///< average latency(ms)
  double p99_latency_ms{0.0};    ///< 99th percentile latency(ms)
  double p999_latency_ms{0.0};   ///< 99.9th percentile latency(ms)
  double p9999_latency_ms{0.0};  ///< 99.99th percentile latency(ms)
};
```

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `max_latency_ms` | `double` | 最大延迟 | 毫秒 |
| `avg_latency_ms` | `double` | 平均延迟 | 毫秒 |
| `p99_latency_ms` | `double` | 99分位延迟 | 毫秒 |
| `p999_latency_ms` | `double` | 99.9分位延迟 | 毫秒 |
| `p9999_latency_ms` | `double` | 99.99分位延迟 | 毫秒 |

- **示例**：

```
#include <iostream>
#include <chrono>
#include <thread>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Lidar lidar;

    agibot::gdk::LidarType lidar_type = agibot::gdk::LidarType::kLidarFront;

    agibot::gdk::LatencyStats latency;
    if (lidar.GetLidarLatency(lidar_type, 1.0, latency) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to get lidar latency" << std::endl;
    } else {
        std::cout << "Lidar latency stats:" << std::endl;
        std::cout << "  Max latency: " << latency.max_latency_ms << "ms" << std::endl;
        std::cout << "  Average latency: " << latency.avg_latency_ms << "ms" << std::endl;
        std::cout << "  P99 latency: " << latency.p99_latency_ms << "ms" << std::endl;
        std::cout << "  P99.9 latency: " << latency.p999_latency_ms << "ms" << std::endl;
        std::cout << "  P99.99 latency: " << latency.p9999_latency_ms << "ms" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 5. `CloseLidar()`

- **功能**：关闭激光雷达DDS连接
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include <iostream>
#include <chrono>
#include <thread>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Lidar lidar;
    std::cout << "Lidar init" << std::endl;

    // 使用激光雷达...

    // 关闭激光雷达
    if (lidar.CloseLidar() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to close lidar" << std::endl;
    } else {
        std::cout << "Lidar closed successfully" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

## 使用注意事项

1. **GDK初始化**：使用Lidar功能前必须先调用`agibot::gdk::GDKInit()`初始化GDK系统
2. **GDK释放**：程序结束前必须调用`agibot::gdk::GDKRelease()`释放GDK系统资源
3. **超时设置**：根据实际需求设置合适的超时时间，避免长时间阻塞
4. **返回值检查**：使用前请检查GDKRes返回值是否为kSuccess
5. **智能指针管理**：PointCloud对象使用shared\_ptr管理，注意生命周期
6. **时间戳精度**：时间戳单位为纳秒，可用于精确的时间同步
7. **点云处理**：点云数据量较大，处理时注意内存使用
8. **雷达选择**：根据应用场景选择合适的雷达类型（前部/后部）
9. **数据解析**：需要根据fields信息正确解析点云数据
10. **坐标系**：注意点云数据的坐标系定义
11. **资源释放**：使用完毕后调用`CloseLidar()`释放资源
12. **错误处理**：始终检查GDKRes返回值，确保操作成功
13. **未实现方法**：`GetLidarFps()`和`GetLidarLatency()`当前未实现，使用时需注意

## 应用场景

- **SLAM建图**：利用点云数据进行同时定位与地图构建
- **障碍物检测**：实时检测环境中的障碍物
- **导航避障**：为机器人导航提供环境感知信息
- **环境建模**：构建3D环境模型
- **目标识别**：结合点云数据进行目标检测和识别
- **路径规划**：基于点云数据规划安全路径
- **数据融合**：与其他传感器数据进行融合，提高感知精度
- **3D重建**：利用点云数据进行3D场景重建
- **距离测量**：精确测量到障碍物的距离
- **安全监控**：监控机器人周围的安全区域



---

<!-- source: contents_reference_C++_map -->

# Map

# GDK Map 接口文档（C++）

## 概述

Map（地图）模块为G02机器人提供了地图管理功能。通过C++接口，开发者可以方便地获取、切换、管理机器人构建的地图，适用于地图存储、地图切换、地图管理等多种场景。

## 接口说明

### Map 类

该类封装了地图管理的主要功能接口。

#### 1. `GetCurrMap()`

- **功能**：获取当前使用的地图信息
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `map_name` | `MapName&` | 输出参数，当前地图名称信息 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`map_name`参数包含当前地图信息

#### MapName对象详细说明

**MapName结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `id` | `uint32_t` | 地图ID | 无单位 |
| `name` | `std::string` | 地图名称 | 字符串 |
| `is_curr_map` | `bool` | 是否为当前地图 | 布尔值 |

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
using namespace agibot::gdk;

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    Map map_manager;
    MapName current_map;
    GDKRes result = map_manager.GetCurrMap(current_map);

    if (result == GDKRes::kSuccess) {
        std::cout << "当前地图: " << std::endl;
        std::cout << "当前地图ID: " << current_map.id << std::endl;
        std::cout << "当前地图名称: " << current_map.name << std::endl;
        std::cout << "是否为当前地图: " << (current_map.is_curr_map ? "是" : "否") << std::endl;
    } else {
        std::cout << "获取当前地图失败" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 2. `GetMap()`

- **功能**：根据地图ID获取地图详细信息（含栅格地图、点云、墙壁、引导点等）
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `map_id` | `const uint8_t` | 地图ID（0-255） |
| `map_info` | `MapInfo&` | 输出参数，地图详细信息 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回 `GDKRes::kSuccess`，`map_info` 参数包含地图详情
- **MapInfo 主要成员**：`gravity`、`grid_map`（OccupancyGrid）、`walls`、`infeasible_areas`、`guide_pts` 等。`grid_map` 含 `width`、`height`、`resolution`、`origin`（Pose）、`data` 等。
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
using namespace agibot::gdk;

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    Map map_manager;

    // 获取当前地图ID后，获取完整地图信息
    MapName current_map;
    if (map_manager.GetCurrMap(current_map) != GDKRes::kSuccess) {
        std::cout << "获取当前地图失败" << std::endl;
        return -1;
    }
    MapInfo map_info;
    if (map_manager.GetMap(current_map.id, map_info) != GDKRes::kSuccess) {
        std::cout << "获取地图失败" << std::endl;
        return -1;
    }

    std::cout << "地图名称: " << map_info.name << ", ID: " << map_info.id << std::endl;
    std::cout << "栅格尺寸: " << map_info.grid_map.width << " x " << map_info.grid_map.height << std::endl;
    std::cout << "分辨率: " << map_info.grid_map.resolution << std::endl;
    std::cout << "原点: (" << map_info.grid_map.origin.position.x << ", "
              << map_info.grid_map.origin.position.y << ")" << std::endl;

    if (GDKRelease() != GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    return 0;
}
```

#### 3. `GetAllMap()`

- **功能**：获取所有可用地图列表
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `map_names` | `std::vector<MapName>&` | 输出参数，地图名称列表 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`map_names`参数包含所有地图信息
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
using namespace agibot::gdk;

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    Map map_manager;
    std::vector<MapName> map_names;
    GDKRes result = map_manager.GetAllMap(map_names);

    if (result == GDKRes::kSuccess) {
        std::cout << "地图列表: " << std::endl;
        std::cout << "地图数量: " << map_names.size() << std::endl;

        for (const auto& map_name : map_names) {
            std::cout << "地图ID: " << map_name.id
                      << ", 名称: " << map_name.name
                      << ", 当前地图: " << (map_name.is_curr_map ? "是" : "否") << std::endl;
        }
    } else {
        std::cout << "获取地图列表失败" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 4. `SwitchMap()`

- **功能**：切换到指定ID的地图
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `map_id` | `const uint8_t` | 地图ID |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
using namespace agibot::gdk;

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    Map map_manager;
    // 切换到地图ID为1的地图
    GDKRes result = map_manager.SwitchMap(1);

    if (result == GDKRes::kSuccess) {
        std::cout << "地图切换成功" << std::endl;

        // 验证切换结果
        MapName current_map;
        result = map_manager.GetCurrMap(current_map);
        if (result == GDKRes::kSuccess) {
            std::cout << "当前地图ID: " << current_map.id << std::endl;
        }
    } else {
        std::cout << "地图切换失败" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 5. `RemoveMap()`

- **功能**：删除指定ID的地图
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `map_id` | `const uint8_t` | 地图ID |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
using namespace agibot::gdk;

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    Map map_manager;
    // 删除地图ID为2的地图
    GDKRes result = map_manager.RemoveMap(2);

    if (result == GDKRes::kSuccess) {
        std::cout << "地图删除成功" << std::endl;
    } else {
        std::cout << "地图删除失败" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

## 使用注意事项

1. **GDK初始化**：使用Map功能前必须先调用`agibot::gdk::GDKInit()`初始化GDK系统
2. **GDK释放**：程序结束前必须调用`agibot::gdk::GDKRelease()`释放GDK系统资源
3. **地图ID有效性**：使用前请确保地图ID存在且有效
4. **返回值检查**：使用前请检查GDKRes返回值是否为kSuccess
5. **地图切换**：切换地图时确保当前没有正在进行的导航任务
6. **地图更新**：更新地图时注意数据格式的正确性
7. **资源管理**：注意地图数据的内存使用，特别是大型地图
8. **错误处理**：始终检查GDKRes返回值，确保操作成功

## 应用场景

- **地图存储**：管理和存储机器人构建的地图数据
- **地图切换**：在不同环境间切换使用不同的地图
- **地图管理**：添加、删除、更新地图信息
- **导航支持**：为导航系统提供地图数据支持
- **环境建模**：构建和维护环境的空间模型
- **路径规划**：为路径规划提供地图基础数据
- **引导点管理**：管理地图中的关键位置点
- **区域标记**：标记不可通行区域和特殊区域



---

<!-- source: contents_reference_C++_pnc -->

# PNC

# GDK PNC 接口文档（C++）

## 概述

PNC（Planning and Control，规划与控制）模块为G02机器人提供了路径规划和导航控制功能。通过C++接口，开发者可以方便地实现机器人的自主导航、路径规划、任务状态管理等功能，适用于自主导航、路径规划、任务调度等多种场景。

## 接口说明

### Pnc 类

该类封装了机器人路径规划和导航控制的主要接口。

#### 1. `GetTaskState()`

- **功能**：获取当前任务状态
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `task_state` | `PNCTaskState&` | 输出参数，任务状态信息对象 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`task_state`参数包含任务状态信息

#### PNCTaskState对象详细说明

**PNCTaskState结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `id` | `uint32_t` | 任务ID | 无单位 |
| `state` | `uint32_t` | 任务状态码 | 无单位 |
| `type` | `uint32_t` | 任务类型 | 无单位 |
| `message` | `std::string` | 状态描述信息 | 字符串 |

**任务状态码说明**：
- `0`: 空闲
- `1`: 启动中
- `2`: 运行中
- `3`: 暂停中
- `4`: 已暂停
- `5`: 恢复中
- `6`: 取消中
- `7`: 已取消
- `8`: 失败
- `9`: 成功

**任务类型说明**：
- `0`: 空闲
- `1`: 点对点导航
- `2`: 高精度导航
- `3`: 远程控制
- `4`: 相对移动导航
- `5`: 路径规划导航

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
using namespace agibot::gdk;

int main() {
    Pnc pnc;
    PNCTaskState task_state;
    GDKRes result = pnc.GetTaskState(task_state);

    if (result == GDKRes::kSuccess) {
      std::cout << "PNC任务状态: " << task_state.state << std::endl;
      std::cout << "PNC任务ID: " << task_state.id << std::endl;
      std::cout << "PNC任务内容: " << task_state.message << std::endl;
      std::cout << "PNC任务种类: " << task_state.type << std::endl;
    } else {
      std::cout << "获取任务状态失败" << std::endl;
    }

    return 0;
}
```

#### 2. `NormalNavi()`

- **功能**：执行正常导航到指定目标点, 执行之前需要在G02 Pad上进行重定位
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `navi_req` | `const NaviReq&` | 导航请求对象 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`

#### NaviReq对象详细说明

**NaviReq结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `target` | `Pose` | 目标位姿 | 位姿 |
| `timestamp_ns` | `uint64_t` | 导航时间戳 | 纳秒 |

**Pose结构体**：

| 成员名 | 类型 | 描述 |
| --- | --- | --- |
| `position` | `Position` | 位置信息 |
| `orientation` | `Orientation` | 方向信息 |

**Position结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `x` | `double` | 目标位置X坐标 | 米 |
| `y` | `double` | 目标位置Y坐标 | 米 |
| `z` | `double` | 目标位置Z坐标 | 米 |

**Orientation结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `x` | `double` | 目标方向四元数X分量 | 无单位 |
| `y` | `double` | 目标方向四元数Y分量 | 无单位 |
| `z` | `double` | 目标方向四元数Z分量 | 无单位 |
| `w` | `double` | 目标方向四元数W分量 | 无单位 |

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
using namespace agibot::gdk;

int main() {
    Pnc pnc;
    NaviReq navi_req;
    navi_req.target.position.x = 2.0;  // 目标X坐标
    navi_req.target.position.y = 3.0;  // 目标Y坐标
    navi_req.target.position.z = 0.0;  // 目标Z坐标
    navi_req.target.orientation.x = 0.0;  // 目标方向四元数X
    navi_req.target.orientation.y = 0.0;  // 目标方向四元数Y
    navi_req.target.orientation.z = 0.0;  // 目标方向四元数Z
    navi_req.target.orientation.w = 1.0;  // 目标方向四元数W
    navi_req.timestamp_ns = 0;  // 时间戳

    GDKRes result = pnc.NormalNavi(navi_req);

    if (result == GDKRes::kSuccess) {
        std::cout << "正常导航启动成功" << std::endl;
    } else {
        std::cout << "正常导航启动失败" << std::endl;
    }

    return 0;
}
```

#### 3. `HighPrecisionNavi()`

- **功能**：执行高精度导航到指定目标点, 执行之前需要在G02 Pad上进行重定位
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `navi_req` | `const NaviReq&` | 导航请求对象 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>

using namespace agibot::gdk;

int main() {
    Pnc pnc;

    NaviReq navi_req;
    navi_req.target.position.x = 1.5;  // 目标X坐标
    navi_req.target.position.y = 2.5;  // 目标Y坐标
    navi_req.target.position.z = 0.0;  // 目标Z坐标
    navi_req.target.orientation.x = 0.0;  // 目标方向四元数X
    navi_req.target.orientation.y = 0.0;  // 目标方向四元数Y
    navi_req.target.orientation.z = 0.0;  // 目标方向四元数Z
    navi_req.target.orientation.w = 1.0;  // 目标方向四元数W
    navi_req.timestamp_ns = 0;  // 时间戳

    GDKRes result = pnc.HighPrecisionNavi(navi_req);

    if (result == GDKRes::kSuccess) {
        std::cout << "高精度导航启动成功" << std::endl;
    } else {
        std::cout << "高精度导航启动失败" << std::endl;
    }

    return 0;
}
```

#### 4. `RelativeMove()`

- **功能**：执行小范围平移, 简单停障，无避障
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `navi_req` | `const NaviReq&` | 导航请求对象 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>

using namespace agibot::gdk;

int main() {
    Pnc pnc;

    NaviReq navi_req;
    navi_req.target.position.x = 0.5;  // 相对X移动距离
    navi_req.target.position.y = 0.0;  // 相对Y移动距离
    navi_req.target.position.z = 0.0;  // 相对Z移动距离
    navi_req.target.orientation.x = 0.0;  // 相对旋转X
    navi_req.target.orientation.y = 0.0;  // 相对旋转Y
    navi_req.target.orientation.z = 0.0;  // 相对旋转Z
    navi_req.target.orientation.w = 1.0;  // 相对旋转W
    navi_req.timestamp_ns = 0;  // 时间戳

    GDKRes result = pnc.RelativeMove(navi_req);

    if (result == GDKRes::kSuccess) {
        std::cout << "相对移动启动成功" << std::endl;
    } else {
        std::cout << "相对移动启动失败" << std::endl;
    }

    return 0;
}
```

#### 5. `CancelTask()`

- **功能**：取消指定ID的导航任务
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `task_id` | `uint32_t` | 要取消的任务ID |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace agibot::gdk;

int main() {
    Pnc pnc;

    // 先获取当前任务状态以获取任务ID
    PNCTaskState task_state;
    if (pnc.GetTaskState(task_state) == GDKRes::kSuccess) {
        uint32_t task_id = task_state.id;
        GDKRes result = pnc.CancelTask(task_id);

        if (result == GDKRes::kSuccess) {
            std::cout << "任务取消成功" << std::endl;
        } else {
            std::cout << "任务取消失败" << std::endl;
        }
    }

    return 0;
}
```

#### 6. `PauseTask()`

- **功能**：暂停指定ID的导航任务
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `task_id` | `uint32_t` | 要暂停的任务ID |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace agibot::gdk;

int main() {
    Pnc pnc;

    // 先获取当前任务状态以获取任务ID
    PNCTaskState task_state;
    if (pnc.GetTaskState(task_state) == GDKRes::kSuccess) {
        uint32_t task_id = task_state.id;
        GDKRes result = pnc.PauseTask(task_id);

        if (result == GDKRes::kSuccess) {
            std::cout << "任务暂停成功" << std::endl;
        } else {
            std::cout << "任务暂停失败" << std::endl;
        }
    }

    return 0;
}
```

#### 7. `ResumeTask()`

- **功能**：恢复指定ID的导航任务
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `task_id` | `uint32_t` | 要恢复的任务ID |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace agibot::gdk;

int main() {
    Pnc pnc;

    // 先获取当前任务状态以获取任务ID
    PNCTaskState task_state;
    if (pnc.GetTaskState(task_state) == GDKRes::kSuccess) {
        uint32_t task_id = task_state.id;
        GDKRes result = pnc.ResumeTask(task_id);

        if (result == GDKRes::kSuccess) {
            std::cout << "任务恢复成功" << std::endl;
        } else {
            std::cout << "任务恢复失败" << std::endl;
        }
    }

    return 0;
}
```

#### 8. `RequestChassisControl()`

- **功能**：请求底盘控制权限，用于远程控制模式
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `control_mode` | `int32_t` | 控制请求：0 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`

**控制模式说明**：
- **阿克曼模式**：适用于前轮转向的车辆运动学模型，通过线速度(`linear.x`)和角速度(`angular.z`)控制
- **蟹行模式**：支持全向移动，通过`linear.x`和`linear.y`分别控制前后和左右移动

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace agibot::gdk;

int main() {
    Pnc pnc;

    // 请求远程控制
    GDKRes res = pnc.RequestChassisControl(0);
    if (res == GDKRes::kSuccess) {
        std::cout << "远程控制请求成功" << std::endl;
    } else {
        std::cout << "控制请求失败" << std::endl;
        return 1;
    }

    // 等待控制权限生效
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 现在可以使用 MoveChassis() 控制底盘
    Twist twist;
    twist.linear.x = 0.3;
    twist.angular.z = 0.0;
    pnc.MoveChassis(twist);

    return 0;
}
```

#### 9. `MoveChassis()`

- **功能**：移动底盘，用于远程控制模式下的底盘运动控制
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `twist` | `const Twist&` | 速度指令对象，包含线速度和角速度 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`

#### Twist对象详细说明

**Twist结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `linear` | `Vector3` | 线速度向量 | 米/秒 (m/s) |
| `angular` | `Vector3` | 角速度向量 | 弧度/秒 (rad/s) |

**Vector3结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `x` | `double` | X轴分量 | 根据上下文 |
| `y` | `double` | Y轴分量 | 根据上下文 |
| `z` | `double` | Z轴分量 | 根据上下文 |

**使用说明**：
- 在调用`MoveChassis()`之前，需要先调用`RequestChassisControl()`请求底盘控制权限
- `linear.x`：前进/后退速度（正值为前进，负值为后退）
- `linear.y`：左右平移速度（蟹行模式使用，正值为左移，负值为右移）
- `linear.z`：通常为0
- `angular.z`：绕Z轴旋转角速度（正值为逆时针，负值为顺时针）

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace agibot::gdk;

int main() {
    Pnc pnc;

    // 请求底盘控制权限
    GDKRes res = pnc.RequestChassisControl(0);
    if (res != GDKRes::kSuccess) {
        std::cout << "请求底盘控制失败" << std::endl;
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 阿克曼模式：前进并右转
    Twist twist;
    twist.linear.x = 0.5;   // 前进速度 0.5 m/s
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = -0.3; // 右转角速度 0.3 rad/s

    // 蟹行模式：向左平移
    /*
    Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.5; // 向左平移速度 0.5 m/s
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    */

    res = pnc.MoveChassis(twist);
    if (res == GDKRes::kSuccess) {

    }

    res = pnc.MoveChassis(twist);
    if (res == GDKRes::kSuccess) {
        std::cout << "底盘移动指令发送成功" << std::endl;
    }

    return 0;
}
```

#### 10. 完整使用示例

- **功能**：演示PNC模块的完整使用流程
- **示例**：

```
  #include <iostream>
  #include <chrono>
  #include <thread>
  #include <csignal>
  #include "gdk/gdk.h"

  void signal_handler(int signum) {
      std::cout << "Interrupt signal (" << signum << ") received." << std::endl;
      exit(signum);
  }

  int main(int argc, char** argv) {
      if (argc != 8) {
          std::cerr << "Usage: " << argv[0]
                  << " position_x position_y position_z orientation_x "
                      "orientation_y orientation_z orientation_w"
                  << std::endl;
          return 1;
      }
      double position_x = std::stod(argv[1]);
      double position_y = std::stod(argv[2]);
      double position_z = std::stod(argv[3]);
      double orientation_x = std::stod(argv[4]);
      double orientation_y = std::stod(argv[5]);
      double orientation_z = std::stod(argv[6]);
      double orientation_w = std::stod(argv[7]);
      agibot::gdk::Pnc pnc;
      std::cout << "Pnc init" << std::endl;
      agibot::gdk::NaviReq navi_req;
      navi_req.target.position.x = position_x;
      navi_req.target.position.y = position_y;
      navi_req.target.position.z = position_z;
      navi_req.target.orientation.x = orientation_x;
      navi_req.target.orientation.y = orientation_y;
      navi_req.target.orientation.z = orientation_z;
      navi_req.target.orientation.w = orientation_w;

      auto res = pnc.NormalNavi(navi_req);
      if (res != agibot::gdk::GDKRes::kSuccess) {
          std::cerr << "NormalNavi failed" << std::endl;
          return 1;
      }
      // std::this_thread::sleep_for(std::chrono::seconds(1));

      // 获取任务ID
      agibot::gdk::PNCTaskState task_state;
      res = pnc.GetTaskState(task_state);
      if (res != agibot::gdk::GDKRes::kSuccess) {
          std::cerr << "GetTaskState failed" << std::endl;
          return 1;
      }
      uint32_t task_id = task_state.id;

      std::cout << "PauseTask" << std::endl;
      res = pnc.PauseTask(task_id);

      if (res != agibot::gdk::GDKRes::kSuccess) {
          std::cerr << "PauseTask failed" << std::endl;
          return 1;
      }
      std::cout << "ResumeTask" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      res = pnc.ResumeTask(task_id);
      if (res != agibot::gdk::GDKRes::kSuccess) {
          std::cerr << "ResumeTask failed" << std::endl;
          return 1;
      }
      std::cout << "CancelTask" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      res = pnc.CancelTask(task_id);
      if (res != agibot::gdk::GDKRes::kSuccess) {
          std::cerr << "CancelTask failed" << std::endl;
          return 1;
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
      if (res != agibot::gdk::GDKRes::kSuccess) {
          // until ctrl c to exit
          std::signal(SIGINT, signal_handler);
          while (true) {
          std::this_thread::sleep_for(std::chrono::seconds(1));
          }

          return 0;
      }
  }
```

#### 11. `OmniNavi()`

- **功能**：执行全向导航到指定目标点，支持任意方向的移动（包括横向移动），执行之前需要在G02 Pad上进行重定位
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `navi_req` | `const NaviReq&` | 导航请求对象(map坐标系下) |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`

**NaviReq对象说明**：与`NormalNavi()`相同，参见上文NaviReq对象详细说明

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
using namespace agibot::gdk;

int main() {
    Pnc pnc;
    std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待1秒以确保DDS连接建立
    NaviReq navi_req;
    navi_req.target.position.x = 2.0;  // 目标X坐标
    navi_req.target.position.y = 3.0;  // 目标Y坐标
    navi_req.target.position.z = 0.0;  // 目标Z坐标
    navi_req.target.orientation.x = 0.0;  // 目标方向四元数X
    navi_req.target.orientation.y = 0.0;  // 目标方向四元数Y
    navi_req.target.orientation.z = 0.0;  // 目标方向四元数Z
    navi_req.target.orientation.w = 1.0;  // 目标方向四元数W
    navi_req.timestamp_ns = 0;  // 时间戳

    GDKRes result = pnc.OmniNavi(navi_req);

    if (result == GDKRes::kSuccess) {
        std::cout << "全向导航启动成功" << std::endl;
    } else {
        std::cout << "全向导航启动失败" << std::endl;
    }

    return 0;
}
```

## 使用注意事项

1. **地图准备**：执行导航前确保已有可用的地图
2. **目标点设置**：设置目标点时注意坐标系的正确性
3. **任务状态监控**：建议定期检查任务状态以了解导航进度
4. **任务管理**：合理使用暂停、恢复、取消功能
5. **精度选择**：根据需求选择合适的导航精度（正常/高精度）
6. **相对移动**：使用相对移动时注意移动距离的合理性

## 应用场景

- **自主导航**：实现机器人的自主路径规划和导航
- **精确导航**：执行高精度的位置控制
- **相对移动**：执行基于当前位置的相对移动
- **任务调度**：管理多个导航任务的执行
- **路径规划**：为机器人规划最优路径
- **避障导航**：在复杂环境中进行安全导航
- **多目标导航**：实现多个目标点的连续导航
- **远程控制**：支持远程控制模式下的导航



---

<!-- source: contents_reference_C++_robot -->

# Robot

# GDK Robot 接口文档（C++）

## 概述

Robot（机器人）模块为G02机器人提供了统一的机器人控制接口，集成了机器人本体状态获取、运动控制、关节控制、末端执行器控制等功能。通过C++接口，开发者可以方便地实现对机器人的全面控制，适用于机器人控制、状态监控、动作执行、路径规划等多种场景。

## 关于运动控制模式的说明

机器人双臂的运动控制接口支持不同的\*\*运动控制模式\*\*：`1`（关节位控/位置控制）、`2`（笛卡尔阻抗控制）、`3`（关节阻抗控制）。机器人启动后默认处于力控模式；使用其他控制模式时，需通过`SetControlMode()`切换。

机器人的运动控制模式是持久的，即一旦切换（无论是通过GDK还是通过其他方式）后会一直保持，直到再次切换或机器人重启。调用相关控制接口前，GDK 会检查机器人\*\*当前实际\*\*的运动控制模式是否在该接口支持的模式列表中，若不在则直接返回 `kRuntimeError`，并在日志中打印原因。各接口的支持情况请参考下方"支持的控制模式"说明。

### 力控使用工作流

笛卡尔/关节力控（阻抗控制）依赖于准确的动力学模型，控制器会基于"机器人本体 + 末端负载"的整体动力学进行重力补偿与柔顺跟踪。若末端实际负载与配置不一致，会导致力反馈失真、柔顺跟踪偏移、甚至触发误报警。建议按以下顺序进入力控：

1. **设置负载**（仅当末端确实存在负载时）：通过 `SetPayload()` 配置末端负载的质量、质心、惯性张量。空载场景应保持/重置为零负载，详见 [`SetPayload()`](#30-setpayload) 中的"使用建议"。
2. **传感器零点标定**（持久负载场景）：负载长期挂载（如固定工具、夹爪上常驻物）时，设置完 `SetPayload()` 后调用一次 `TorqueSensorCalibration()`，补偿传感器零点偏置。短期搬运类任务无需每次标定。
3. **切换控制模式**：`SetControlMode(2)` 进入笛卡尔力控，或 `SetControlMode(3)` 进入关节力控。
4. **配置阻抗参数**：进入对应控制模式后，再调用 `SetCartesianImpedance()` 或 `SetJointImpedance()` 设置参数；仅 `kp` 字段当前生效。
5. **下发目标位姿/关节量**：使用 `EndEffectorPoseControl()`、`JointControl()` 等接口下发目标，底层按已配置的阻抗参数柔顺跟踪。

### 笛卡尔阻抗 vs 关节阻抗

两类阻抗都用于实现"柔顺跟踪 + 力反馈"，区别在于刚度/柔顺性是定义在哪个空间，进而影响控制目标的下发形式与适合的任务类型：

| 维度 | 笛卡尔阻抗（`control_mode=2`） | 关节阻抗（`control_mode=3`） |
| --- | --- | --- |
| **刚度定义空间** | 末端笛卡尔空间（`x/y/z/roll/pitch/yaw` 6 维） | 关节空间（每个关节单独配置，单臂 7 维） |
| **执行轨迹** | 目标位姿直接在笛卡尔空间执行，不经过全身逆解 | 目标关节量在关节空间执行；如下发的是位姿，会先经全身逆解再到关节空间柔顺跟踪 |
| **适合任务** | 接触类任务、装配、贴合、沿笛卡尔某轴方向柔顺（如 z 向贴合） | 拖拽示教、关节级柔顺、对单关节刚度有差异化要求的任务 |
| **可观察柔顺性方向** | 在笛卡尔轴上更直观（如降低 z 向 `kp` 让末端在垂直方向更"软"） | 在关节维度上更直观（如降低肘关节 `kp` 让肘部更"软"） |
| **配置接口** | [`SetCartesianImpedance()`](#24-setcartesianimpedance) | [`SetJointImpedance()`](#28-setjointimpedance) |
| **`kp` 维度** | 单臂 6 维：`x/y/z/roll/pitch/yaw` | 单臂 7 维：关节 1~7 |

**共同要点**：

- 设置阻抗参数前，必须先切换到对应控制模式：笛卡尔阻抗参数需先 `SetControlMode(2)`，关节阻抗参数需先 `SetControlMode(3)`。
- 当前实现仅 `kp` 字段生效（刚度系数），`ki` / `kv` 暂未启用。
- 重启后参数会丢失，每次需重新下发；详见 [`TorqueSensorCalibration()`](#31-torquesensorcalibration) 的"持久化说明"。
- 两种阻抗模式互斥：同一时刻只能处于其中一种，切换需重新调用 `SetControlMode()`。
- 进入阻抗模式前请先按"力控使用工作流"配置好负载，避免因动力学不匹配导致柔顺跟踪偏移。

**注意事项**：

- 控制关节角度的相关接口，支持位控模式和关节阻抗模式；
- 控制末端位姿势的相关接口，支持位控模式，笛卡尔阻抗模式和关节阻抗模式；
- 在不同的控制模式下控制末端位姿时，需要使用不同的参考坐标系，详细可参考EndEffectorPoseControl()的使用说明。

## 接口说明

### Robot 类

该类封装了机器人的主要控制接口，集成了HAL（硬件抽象层）和运动控制功能。

#### 1. `GetJointStates()`

- **注意事项**：使用motor\_position和motor\_velocity获取电机位置和速度，position和velocity为低速电机预留字段，当前版本无需关注
- **功能**：获取机器人关节状态信息
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `joint_states` | `JointStates&` | 输出参数，关节状态信息对象 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`joint_states`参数包含关节状态信息

#### JointStates对象详细说明

**JointStates结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `nums` | `size_t` | 关节数量 | 无单位 |
| `states` | `std::vector<JointState>` | 关节状态列表 | 关节状态列表 |
| `timestamp` | `uint64_t` | 时间戳 | 纳秒 |

```
struct JointStates {
  size_t nums{};                     ///< number of joint states
  std::vector<JointState> states{};  ///< joint states
  uint64_t timestamp{};              ///< joint states timestamp(ns)
};
```

**JointState结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `name` | `std::string` | 关节名称 | 字符串 |
| `mode` | `uint32_t` | 关节模式 | 无单位 |
| `position` | `double` | 关节位置 | 弧度 |
| `velocity` | `double` | 关节速度 | 弧度/秒 |
| `effort` | `double` | 关节力矩 | N·m |
| `motor_position` | `double` | 电机位置 | 弧度 |
| `motor_velocity` | `double` | 电机速度 | 弧度/秒 |
| `motor_current` | `double` | 电机电流 | A |
| `error_code` | `uint32_t` | 错误码，0表示正常 | 无单位 |

```
struct JointState {
  std::string name{};
  uint32_t mode{};
  double position{};
  double velocity{};
  double effort{};
  double motor_position{};
  double motor_velocity{};
  double motor_current{};
  uint32_t error_code{};
};
```

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Robot robot;
    std::cout << "Robot init" << std::endl;

    agibot::gdk::JointStates joint_states;
    if (robot.GetJointStates(joint_states) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to get joint states" << std::endl;
    } else {
        std::cout << "关节数量: " << joint_states.nums << std::endl;
        std::cout << "时间戳: " << joint_states.timestamp << std::endl;
        for (const auto& joint_state : joint_states.states) {
            std::cout << "关节: " << joint_state.name
                      << ", 位置: " << joint_state.position
                      << ", 速度: " << joint_state.velocity
                      << ", 力矩: " << joint_state.effort
                      << ", 错误码: " << joint_state.error_code << std::endl;
        }
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 2. `GetJointLimits()`

- **功能**：获取机器人关节限位信息
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `joint_limits` | `std::unordered_map<std::string, JointLimit>&` | 输出参数，键为关节名，值为关节限位信息 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`joint_limits`参数包含关节限位信息

#### JointLimit对象详细说明

**JointLimit结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `min` | `double` | 关节最小位置限位 | 弧度 |
| `max` | `double` | 关节最大位置限位 | 弧度 |

```
struct JointLimit {
    double min{};
    double max{};
};
```

- **说明**：当前返回结果包含机器人本体关节与末端执行器关节（left\_tool/right\_tool）的限位信息。

G02 各关节限位参考值（单位：弧度）：

| 关节名 | 分组 | 最小值 | 最大值 |
| --- | --- | --- | --- |
| `idx01_body_joint1` | 躯干 | -1.0821 | 0.0002 |
| `idx02_body_joint2` | 躯干 | -0.0002 | 2.6529 |
| `idx03_body_joint3` | 躯干 | -1.9199 | 1.5710 |
| `idx04_body_joint4` | 躯干 | -0.4363 | 0.4363 |
| `idx05_body_joint5` | 躯干 | -3.0456 | 3.0456 |
| `idx11_head_joint1` | 头部 | -1.5710 | 1.5710 |
| `idx12_head_joint2` | 头部 | -0.3492 | 0.3492 |
| `idx13_head_joint3` | 头部 | -0.5348 | 0.5348 |
| `idx21_arm_l_joint1` | 左臂 | -3.0718 | 3.0718 |
| `idx22_arm_l_joint2` | 左臂 | -2.0595 | 2.0595 |
| `idx23_arm_l_joint3` | 左臂 | -3.0718 | 3.0718 |
| `idx24_arm_l_joint4` | 左臂 | -2.4958 | 1.0123 |
| `idx25_arm_l_joint5` | 左臂 | -3.0718 | 3.0718 |
| `idx26_arm_l_joint6` | 左臂 | -1.0123 | 1.0123 |
| `idx27_arm_l_joint7` | 左臂 | -1.5359 | 1.5359 |
| `idx61_arm_r_joint1` | 右臂 | -3.0718 | 3.0718 |
| `idx62_arm_r_joint2` | 右臂 | -2.0595 | 2.0595 |
| `idx63_arm_r_joint3` | 右臂 | -3.0718 | 3.0718 |
| `idx64_arm_r_joint4` | 右臂 | -2.4958 | 1.0123 |
| `idx65_arm_r_joint5` | 右臂 | -3.0718 | 3.0718 |
| `idx66_arm_r_joint6` | 右臂 | -1.0123 | 1.0123 |
| `idx67_arm_r_joint7` | 右臂 | -1.5359 | 1.5359 |

- **示例**：

  ```
  #include "gdk/gdk.h"
  #include <iostream>
  #include <unordered_map>

  int main() {
      if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
          std::cout << "GDK初始化失败" << std::endl;
          return -1;
      }

      agibot::gdk::Robot robot;
      std::unordered_map<std::string, agibot::gdk::JointLimit> joint_limits;
      if (robot.GetJointLimits(joint_limits) != agibot::gdk::GDKRes::kSuccess) {
          std::cout << "Failed to get joint limits" << std::endl;
      } else {
          for (const auto& [name, limit] : joint_limits) {
              std::cout << name << ": [" << limit.min << ", " << limit.max << "]" << std::endl;
          }
      }

      agibot::gdk::GDKRelease();
      return 0;
  }
  ```

#### 3. `GetEndState()`

- **功能**：获取末端执行器状态信息
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `end_state` | `DualEndState&` | 输出参数，双末端执行器状态信息对象 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`end_state`参数包含末端执行器状态信息

#### DualEndState对象详细说明

**DualEndState结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `left_end_state` | `EndState` | 左末端执行器状态 | 末端状态 |
| `right_end_state` | `EndState` | 右末端执行器状态 | 末端状态 |

```
struct DualEndState {
  EndState left_end_state{};
  EndState right_end_state{};
};
```

**EndState结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `controlled` | `bool` | 是否被控制 | 布尔值 |
| `type` | `uint32_t` | 末端执行器类型 | 无单位 |
| `names` | `std::vector<std::string>` | 关节名称列表 | 字符串列表 |
| `end_states` | `std::vector<MotorState>` | 电机状态列表 | 电机状态列表 |

```
struct EndState {
  bool controlled{};
  uint32_t type{};
  std::vector<std::string> names{};
  std::vector<MotorState> end_states{};
};
```

**MotorState结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `id` | `uint32_t` | 电机ID | 无单位 |
| `enable` | `bool` | 是否使能 | 布尔值 |
| `position` | `double` | 电机位置 | 弧度 |
| `velocity` | `double` | 电机速度 | 弧度/秒 |
| `effort` | `double` | 电机力矩 | N·m |
| `current` | `float` | 电机电流 | A |
| `voltage` | `float` | 电机电压 | V |
| `temperature` | `float` | 电机温度 | °C |
| `status` | `uint32_t` | 电机状态 | 无单位 |
| `err_code` | `uint32_t` | 错误码 | 无单位 |

```
struct MotorState {
  uint32_t id = 0;
  bool enable = false;
  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0;
  float current = 0.0f;
  float voltage = 0.0f;
  float temperature = 0.0f;
  uint32_t status = 0;
  uint32_t err_code = 0;
};
```

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Robot robot;
    std::cout << "Robot init" << std::endl;

    agibot::gdk::DualEndState end_state;
    if (robot.GetEndState(end_state) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to get end state" << std::endl;
    } else {
        std::cout << "左末端执行器状态:" << std::endl;
        std::cout << "  控制状态: " << (end_state.left_end_state.controlled ? "是" : "否") << std::endl;
        std::cout << "  类型: " << end_state.left_end_state.type << std::endl;
        for (const auto& motor_state : end_state.left_end_state.end_states) {
            std::cout << "  电机ID: " << motor_state.id
                      << ", 位置: " << motor_state.position
                      << ", 电流: " << motor_state.current
                      << ", 温度: " << motor_state.temperature << std::endl;
        }

        std::cout << "右末端执行器状态:" << std::endl;
        std::cout << "  控制状态: " << (end_state.right_end_state.controlled ? "是" : "否") << std::endl;
        std::cout << "  类型: " << end_state.right_end_state.type << std::endl;
        for (const auto& motor_state : end_state.right_end_state.end_states) {
            std::cout << "  电机ID: " << motor_state.id
                      << ", 位置: " << motor_state.position
                      << ", 电流: " << motor_state.current
                      << ", 温度: " << motor_state.temperature << std::endl;
        }
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 4. `GetEndForceState()`

- **功能**：获取灵巧手触觉数据
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `end_force_state` | `DualEndForceState&` | 输出参数，双末端灵巧手触觉数据对象 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`end_force_state`参数包含触觉传感器数据

#### DualEndForceState对象详细说明

**DualEndForceState结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `left_end_force_state` | `EndForceState` | 左末端触觉数据 | 触觉状态 |
| `right_end_force_state` | `EndForceState` | 右末端触觉数据 | 触觉状态 |

```
struct DualEndForceState {
  EndForceState left_end_force_state{};
  EndForceState right_end_force_state{};
};
```

**EndForceState结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `timestamp` | `uint64_t` | 时间戳 | 纳秒 |
| `controlled` | `bool` | 是否受控 | 布尔值 |
| `type` | `uint32_t` | 末端执行器类型 | 无单位 |
| `force` | `std::vector<ForceSensor>` | 触觉传感器数据列表 | 传感器数据列表 |
| `status` | `uint32_t` | 状态码 | 无单位 |
| `err_code` | `uint32_t` | 错误码，0表示正常 | 无单位 |

```
struct EndForceState {
  uint64_t timestamp{};
  bool controlled{};
  uint32_t type{};
  std::vector<ForceSensor> force{};
  uint32_t status{};
  uint32_t err_code{};
};
```

**ForceSensor结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `sensor_type` | `uint32_t` | 传感器类型：0 Unknown，1 force，2 tactile，3 torque，4 fts | 无单位 |
| `sensor_name` | `std::string` | 传感器名称，格式通常为 `{frame}_{surface}_{sensor_type}`，`surface` 可为 inner、outer、palmar、dorsal | 字符串 |
| `frame` | `std::string` | 传感器安装所在的 link 名 | 字符串 |
| `rows` | `uint32_t` | 触觉阵列行数 M | 无单位 |
| `cols` | `uint32_t` | 触觉阵列列数 N | 无单位 |
| `resolution_x` | `float` | 相邻两个 cell 在 x 方向的间距；没有时为 0 | mm |
| `resolution_y` | `float` | 相邻两个 cell 在 y 方向的间距；没有时为 0 | mm |
| `normal_force` | `std::vector<float>` | 法向力，必选；按 row-major 展开，长度为 `rows * cols` | mN |
| `shear_force_x` | `std::vector<float>` | 切向力 x；没有时为空数组或同长度 0，按右手定则 | mN |
| `shear_force_y` | `std::vector<float>` | 切向力 y；没有时为空数组或同长度 0，按右手定则 | mN |
| `contact` | `std::vector<uint8_t>` | 是否接触 | 无单位 |
| `valid` | `std::vector<uint8_t>` | 是否有效；若对应 index 处布局无传感器，则为 false | 无单位 |
| `err_code` | `uint32_t` | 传感器错误码 | 无单位 |

```
struct ForceSensor {
  uint32_t sensor_type{};
  std::string sensor_name{};
  std::string frame{};
  uint32_t rows{};
  uint32_t cols{};
  float resolution_x{};
  float resolution_y{};
  std::vector<float> normal_force{};
  std::vector<float> shear_force_x{};
  std::vector<float> shear_force_y{};
  std::vector<uint8_t> contact{};
  std::vector<uint8_t> valid{};
  uint32_t err_code{};
};
```

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Robot robot;
    std::cout << "Robot init" << std::endl;

    agibot::gdk::DualEndForceState end_force_state;
    if (robot.GetEndForceState(end_force_state) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to get end force state" << std::endl;
    } else {
        auto print_float_vector = [](const std::string& name, const std::vector<float>& values) {
            std::cout << "    " << name << ": [";
            for (size_t i = 0; i < values.size(); ++i) {
                if (i > 0) {
                    std::cout << ", ";
                }
                std::cout << values[i];
            }
            std::cout << "]" << std::endl;
        };

        auto print_end_force_state = [&](const std::string& side,
                                         const agibot::gdk::EndForceState& state) {
            std::cout << side << "末端触觉数据:" << std::endl;
            std::cout << "  时间戳: " << state.timestamp << std::endl;
            std::cout << "  受控状态: " << (state.controlled ? "是" : "否") << std::endl;
            std::cout << "  类型: " << state.type << std::endl;
            std::cout << "  状态码: " << state.status << std::endl;
            std::cout << "  错误码: " << state.err_code << std::endl;
            std::cout << "  触觉传感器数量: " << state.force.size() << std::endl;

            for (size_t i = 0; i < state.force.size(); ++i) {
                const auto& sensor = state.force[i];
                std::cout << "  传感器 " << i << ":" << std::endl;
                std::cout << "    名称: " << sensor.sensor_name << std::endl;
                std::cout << "    类型: " << sensor.sensor_type << std::endl;
                std::cout << "    坐标系: " << sensor.frame << std::endl;
                std::cout << "    阵列: " << sensor.rows << "x" << sensor.cols << std::endl;
                std::cout << "    分辨率: " << sensor.resolution_x << " x "
                          << sensor.resolution_y << std::endl;
                std::cout << "    错误码: " << sensor.err_code << std::endl;

                if (sensor.rows > 0 && sensor.cols > 0) {
                    print_float_vector("法向力 normal_force", sensor.normal_force);
                    print_float_vector("切向力 shear_force_x", sensor.shear_force_x);
                    print_float_vector("切向力 shear_force_y", sensor.shear_force_y);
                }
            }
        };

        print_end_force_state("左", end_force_state.left_end_force_state);
        print_end_force_state("右", end_force_state.right_end_force_state);
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 5. `GetWholeBodyStatus()`

- **功能**：获取机器人全身状态信息
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `whole_body_status` | `WholeBodyStatus&` | 输出参数，全身状态信息对象 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`whole_body_status`参数包含全身状态信息

#### WholeBodyStatus对象详细说明

**WholeBodyStatus结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `right_arm_error` | `uint32_t` | 右臂错误码 | 无单位 |
| `left_arm_error` | `uint32_t` | 左臂错误码 | 无单位 |
| `right_arm_control` | `bool` | 右臂控制状态 | 布尔值 |
| `left_arm_control` | `bool` | 左臂控制状态 | 布尔值 |
| `right_arm_estop` | `bool` | 右臂急停状态 | 布尔值 |
| `left_arm_estop` | `bool` | 左臂急停状态 | 布尔值 |
| `right_end_error` | `uint32_t` | 右末端执行器错误码 | 无单位 |
| `left_end_error` | `uint32_t` | 左末端执行器错误码 | 无单位 |
| `right_end_model` | `std::string` | 右末端执行器型号 | 字符串 |
| `left_end_model` | `std::string` | 左末端执行器型号 | 字符串 |
| `waist_error` | `uint32_t` | 腰部错误码 | 无单位 |
| `lift_error` | `uint32_t` | 升降错误码 | 无单位 |
| `neck_error` | `uint32_t` | 颈部错误码 | 无单位 |
| `chassis_error` | `uint32_t` | 底盘错误码 | 无单位 |
| `timestamp` | `uint64_t` | 时间戳 | 纳秒 |

```
struct WholeBodyStatus {
  uint32_t right_arm_error{0};
  uint32_t left_arm_error{0};
  bool right_arm_control{false};
  bool left_arm_control{false};
  bool right_arm_estop{false};
  bool left_arm_estop{false};
  uint32_t right_end_error{0};
  uint32_t left_end_error{0};
  std::string right_end_model{};
  std::string left_end_model{};
  uint32_t waist_error{0};
  uint32_t lift_error{0};
  uint32_t neck_error{0};
  uint32_t chassis_error{0};
  uint64_t timestamp{};
};
```

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Robot robot;
    std::cout << "Robot init" << std::endl;

    agibot::gdk::WholeBodyStatus whole_body_status;
    if (robot.GetWholeBodyStatus(whole_body_status) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to get whole body status" << std::endl;
    } else {
        std::cout << "全身状态信息:" << std::endl;
        std::cout << "右臂错误码: " << whole_body_status.right_arm_error << std::endl;
        std::cout << "左臂错误码: " << whole_body_status.left_arm_error << std::endl;
        std::cout << "右臂控制状态: " << (whole_body_status.right_arm_control ? "是" : "否") << std::endl;
        std::cout << "左臂控制状态: " << (whole_body_status.left_arm_control ? "是" : "否") << std::endl;
        std::cout << "右臂急停状态: " << (whole_body_status.right_arm_estop ? "是" : "否") << std::endl;
        std::cout << "左臂急停状态: " << (whole_body_status.left_arm_estop ? "是" : "否") << std::endl;
        std::cout << "右末端执行器型号: " << whole_body_status.right_end_model << std::endl;
        std::cout << "左末端执行器型号: " << whole_body_status.left_end_model << std::endl;
        std::cout << "腰部错误码: " << whole_body_status.waist_error << std::endl;
        std::cout << "升降错误码: " << whole_body_status.lift_error << std::endl;
        std::cout << "颈部错误码: " << whole_body_status.neck_error << std::endl;
        std::cout << "底盘错误码: " << whole_body_status.chassis_error << std::endl;
        std::cout << "时间戳: " << whole_body_status.timestamp << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 6. `GetMotionControlStatus()`

- **功能**：获取末端控制状态信息
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `status` | `MotionControlStatus&` | 输出参数，末端控制状态对象 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`status`参数包含运动控制状态信息

#### MotionControlStatus对象详细说明

**MotionControlStatus结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `mode` | `uint8_t` | 运动模式，0为停止，1为伺服，2为规划 | 无单位 |
| `control_mode` | `uint8_t` | 控制模式，1为关节位控（PD/轨迹），2为笛卡尔阻抗，3为关节阻抗 | 无单位 |
| `error_code` | `uint8_t` | 错误码，0表示正常 | 无单位 |
| `error_msg` | `std::string` | 错误信息 | 字符串 |
| `frame_names` | `std::vector<std::string>` | 末端关节名称列表 | 无单位 |
| `frame_poses` | `std::vector<Pose>` | 末端关节位姿列表 | 米 |
| `twists` | `std::vector<Twist>` | 末端关节速度列表 | 米/秒 |
| `wrenches` | `std::vector<Wrench>` | 末端关节力矩列表 | 牛米（torque）、牛（force） |
| `arm_disturbance_force` | `std::vector<double>` | 各关节所受外力 | 牛米 |
| `collision_pairs_1` | `std::vector<std::string>` | 碰撞对列表1 | 无单位 |
| `collision_pairs_2` | `std::vector<std::string>` | 碰撞对列表2 | 无单位 |

**碰撞对说明**：
碰撞对是指当前状态下发生碰撞的关节对，如有碰撞会在列表中注明碰撞信息（碰撞的link（urdf的link）），否则为空。

```
struct MotionControlStatus {
  uint8_t mode{};          ///< motion mode, 0: stop, 1: servo, 2: planning
  uint8_t control_mode{};  ///< control mode, 1: joint position, 2: cartesian impedance, 3: joint impedance
  uint8_t error_code{};
  std::string error_msg{};

  std::vector<std::string> frame_names{};
  std::vector<Pose> frame_poses{};
  std::vector<Twist> twists{};
  std::vector<Wrench> wrenches{};

  std::vector<double> arm_disturbance_force{};

  std::vector<std::string> collision_pairs_1{};
  std::vector<std::string> collision_pairs_2{};
};
```

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Robot robot;
    std::cout << "Robot init" << std::endl;

    agibot::gdk::MotionControlStatus status;
    if (robot.GetMotionControlStatus(status) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to get motion control status" << std::endl;
    } else {
        std::cout << "=== 运动控制状态 ===" << std::endl;
        std::cout << "运动模式(mode): " << static_cast<int>(status.mode) << std::endl;
        std::cout << "控制模式(control_mode): " << static_cast<int>(status.control_mode) << std::endl;
        std::cout << "错误码(error_code): " << static_cast<int>(status.error_code) << std::endl;
        std::cout << "错误信息(error_msg): " << status.error_msg << std::endl;

        std::cout << "\n=== 末端关节名称(frame_names) ===" << std::endl;
        std::cout << "数量: " << status.frame_names.size() << std::endl;
        for (size_t i = 0; i < status.frame_names.size(); ++i) {
            std::cout << "  [" << i << "] " << status.frame_names[i] << std::endl;
        }

        std::cout << "\n=== 末端关节位姿(frame_poses) ===" << std::endl;
        std::cout << "数量: " << status.frame_poses.size() << std::endl;
        for (size_t i = 0; i < status.frame_poses.size(); ++i) {
            const auto& pose = status.frame_poses[i];
            std::cout << "  [" << i << "] ";
            if (i < status.frame_names.size()) {
                std::cout << status.frame_names[i];
            } else {
                std::cout << "frame_" << i;
            }
            std::cout << ":" << std::endl;
            std::cout << "    position: x=" << pose.position.x
                      << ", y=" << pose.position.y
                      << ", z=" << pose.position.z << std::endl;
            std::cout << "    orientation: x=" << pose.orientation.x
                      << ", y=" << pose.orientation.y
                      << ", z=" << pose.orientation.z
                      << ", w=" << pose.orientation.w << std::endl;
        }

        std::cout << "\n=== 末端关节速度(twists) ===" << std::endl;
        std::cout << "数量: " << status.twists.size() << std::endl;
        for (size_t i = 0; i < status.twists.size(); ++i) {
            const auto& twist = status.twists[i];
            std::cout << "  [" << i << "] ";
            if (i < status.frame_names.size()) {
                std::cout << status.frame_names[i];
            } else {
                std::cout << "frame_" << i;
            }
            std::cout << ":" << std::endl;
            std::cout << "    linear: x=" << twist.linear.x
                      << ", y=" << twist.linear.y
                      << ", z=" << twist.linear.z << std::endl;
            std::cout << "    angular: x=" << twist.angular.x
                      << ", y=" << twist.angular.y
                      << ", z=" << twist.angular.z << std::endl;
        }

        std::cout << "\n=== 末端关节力/力矩(wrenches) ===" << std::endl;
        std::cout << "数量: " << status.wrenches.size() << std::endl;
        for (size_t i = 0; i < status.wrenches.size(); ++i) {
            const auto& wrench = status.wrenches[i];
            std::cout << "  [" << i << "] ";
            if (i < status.frame_names.size()) {
                std::cout << status.frame_names[i];
            } else {
                std::cout << "frame_" << i;
            }
            std::cout << ":" << std::endl;
            std::cout << "    force: x=" << wrench.force.x
                      << ", y=" << wrench.force.y
                      << ", z=" << wrench.force.z << std::endl;
            std::cout << "    torque: x=" << wrench.torque.x
                      << ", y=" << wrench.torque.y
                      << ", z=" << wrench.torque.z << std::endl;
        }

        std::cout << "\n=== 各关节所受外力(arm_disturbance_force) ===" << std::endl;
        std::cout << "数量: " << status.arm_disturbance_force.size() << std::endl;
        for (size_t i = 0; i < status.arm_disturbance_force.size(); ++i) {
            std::cout << "  [" << i << "] " << status.arm_disturbance_force[i] << std::endl;
        }

        std::cout << "\n=== 碰撞对(collision_pairs_1 / collision_pairs_2) ===" << std::endl;
        std::cout << "碰撞对数量: " << status.collision_pairs_1.size() << std::endl;
        for (size_t i = 0; i < status.collision_pairs_1.size() && i < status.collision_pairs_2.size(); ++i) {
            std::cout << "  [" << i << "] " << status.collision_pairs_1[i]
                      << " <-> " << status.collision_pairs_2[i] << std::endl;
        }
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 7. `GetChassisPowerState()`

- **功能**：获取底盘电源状态
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `chassis_power_state` | `ChassisPowerState&` | 底盘电源状态对象 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`

#### ChassisPowerState对象详细说明

**ChassisPowerState结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `battery_main_power_switch_state` | `uint8_t` | 电池主电源开关状态 | 无单位 |
| `emergency_stop_pedal_state` | `uint8_t` | 急停踏板状态 | 无单位 |
| `battery_main_power_switch_fault_state` | `uint8_t` | 电池主电源开关故障状态 | 无单位 |
| `emergency_stop_pedal_fault_state` | `uint8_t` | 急停踏板故障状态 | 无单位 |
| `chassis_power_board_state` | `uint8_t` | 底盘电源板状态 | 无单位 |
| `chassis_left_traction_motor_power_state` | `uint8_t` | 底盘左牵引电机电源状态 | 无单位 |
| `chassis_right_traction_motor_power_state` | `uint8_t` | 底盘右牵引电机电源状态 | 无单位 |
| `chassis_left_steering_motor_power_state` | `uint8_t` | 底盘左转向电机电源状态 | 无单位 |
| `chassis_right_steering_motor_power_state` | `uint8_t` | 底盘右转向电机电源状态 | 无单位 |
| `chassis_lidar1_power_state` | `uint8_t` | 底盘激光雷达1电源状态 | 无单位 |
| `chassis_lidar2_power_state` | `uint8_t` | 底盘激光雷达2电源状态 | 无单位 |
| `chassis_ultrasonic_radar_power_state` | `uint8_t` | 底盘超声波雷达电源状态 | 无单位 |
| `chassis_tof_camera_power_state` | `uint8_t` | 底盘ToF相机电源状态 | 无单位 |
| `chassis_ethernet_switch_power_state` | `uint8_t` | 底盘以太网交换机电源状态 | 无单位 |
| `chassis_external_power_output_state` | `uint8_t` | 底盘外部电源输出状态 | 无单位 |
| `battery_main_power_output_switch_state` | `uint8_t` | 电池主电源输出开关状态 | 无单位 |
| `battery_states` | `std::vector<BatteryState>` | 电池状态列表 | 电池状态列表 |
| `charge_plug_insert_state` | `uint8_t` | 充电插头插入状态 | 无单位 |
| `charge_plug_input_voltage` | `float` | 充电插头输入电压 | V |
| `charge_plug_input_current` | `float` | 充电插头输入电流 | A |
| `charge_plug_input_short_circuit_fault_state` | `uint8_t` | 充电插头输入短路故障状态 | 无单位 |
| `charge_plug_input_open_circuit_fault_state` | `uint8_t` | 充电插头输入开路故障状态 | 无单位 |
| `chassis_led_strip_power_state` | `uint8_t` | 底盘LED灯带电源状态 | 无单位 |
| `chassis_power_board_temperature` | `float` | 底盘电源板温度 | °C |
| `power_48v_bus_power_on_fault_state` | `uint8_t` | 48V总线电源故障状态 | 无单位 |
| `power_poe_bus_power_on_fault_state` | `uint8_t` | PoE总线电源故障状态 | 无单位 |
| `chassis_board_12v_output_fault_state` | `uint8_t` | 底盘板12V输出故障状态 | 无单位 |
| `chassis_board_5v_output_fault_state` | `uint8_t` | 底盘板5V输出故障状态 | 无单位 |
| `chassis_power_board_fault_state` | `uint32_t` | 底盘电源板故障状态 | 无单位 |
| `timestamp` | `uint64_t` | 时间戳 | 纳秒 |

```
struct ChassisPowerState {
  uint8_t battery_main_power_switch_state{0};
  uint8_t emergency_stop_pedal_state{0};
  uint8_t battery_main_power_switch_fault_state{0};
  uint8_t emergency_stop_pedal_fault_state{0};
  uint8_t chassis_power_board_state{0};
  uint8_t chassis_left_traction_motor_power_state{0};
  uint8_t chassis_right_traction_motor_power_state{0};
  uint8_t chassis_left_steering_motor_power_state{0};
  uint8_t chassis_right_steering_motor_power_state{0};
  uint8_t chassis_lidar1_power_state{0};
  uint8_t chassis_lidar2_power_state{0};
  uint8_t chassis_ultrasonic_radar_power_state{0};
  uint8_t chassis_tof_camera_power_state{0};
  uint8_t chassis_ethernet_switch_power_state{0};
  uint8_t chassis_external_power_output_state{0};
  uint8_t battery_main_power_output_switch_state{0};
  std::vector<BatteryState> battery_states{};
  uint8_t charge_plug_insert_state{0};
  float charge_plug_input_voltage{0.0};
  float charge_plug_input_current{0.0};
  uint8_t charge_plug_input_short_circuit_fault_state{0};
  uint8_t charge_plug_input_open_circuit_fault_state{0};
  uint8_t chassis_led_strip_power_state{0};
  float chassis_power_board_temperature{0.0};
  uint8_t power_48v_bus_power_on_fault_state{0};
  uint8_t power_poe_bus_power_on_fault_state{0};
  uint8_t chassis_board_12v_output_fault_state{0};
  uint8_t chassis_board_5v_output_fault_state{0};
  uint32_t chassis_power_board_fault_state{0};
  uint64_t timestamp{0};
};
```

**BatteryState结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `battery_charging_status` | `uint8_t` | 电池充电状态 | 无单位 |
| `battery_output_voltage` | `float` | 电池输出电压 | V |
| `battery_output_current` | `float` | 电池输出电流 | A |
| `battery_charging_current` | `float` | 电池充电电流 | A |
| `battery_temperature` | `float` | 电池温度 | °C |
| `battery_soc` | `float` | 电池电量百分比 (State of Charge) | % |
| `battery_soh` | `uint8_t` | 电池健康度 (State of Health) | % |
| `battery_short_circuit_fault_state` | `uint8_t` | 电池短路故障状态 | 无单位 |
| `battery_open_circuit_fault_state` | `uint8_t` | 电池开路故障状态 | 无单位 |
| `battery_other_fault_state` | `uint8_t` | 电池其他故障状态 | 无单位 |
| `battery_outside_output_voltage` | `float` | 电池外部输出电压 | V |
| `battery_outside_connection` | `uint8_t` | 电池外部连接状态 | 无单位 |
| `battery_outside_open_circuit_fault_state` | `uint8_t` | 电池外部开路故障状态 | 无单位 |
| `battery_switch_state` | `uint8_t` | 电池开关状态 | 无单位 |
| `battery_unlock_state` | `uint8_t` | 电池解锁状态 | 无单位 |
| `battery_input_fault_state` | `uint8_t` | 电池输入故障状态 | 无单位 |
| `battery_charging_mos_switch_state` | `uint8_t` | 电池充电MOS开关状态 | 无单位 |

```
struct BatteryState {
  uint8_t battery_charging_status{0};
  float battery_output_voltage{0.0};
  float battery_output_current{0.0};
  float battery_charging_current{0.0};
  float battery_temperature{0.0};
  float battery_soc{0.0};
  uint8_t battery_soh{0};
  uint8_t battery_short_circuit_fault_state{0};
  uint8_t battery_open_circuit_fault_state{0};
  uint8_t battery_other_fault_state{0};
  float battery_outside_output_voltage{0.0};
  uint8_t battery_outside_connection{0};
  uint8_t battery_outside_open_circuit_fault_state{0};
  uint8_t battery_switch_state{0};
  uint8_t battery_unlock_state{0};
  uint8_t battery_input_fault_state{0};
  uint8_t battery_charging_mos_switch_state{0};
};
```

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Robot robot;
    std::cout << "Robot init" << std::endl;

    agibot::gdk::ChassisPowerState chassis_power_state;
    if (robot.GetChassisPowerState(chassis_power_state) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to get chassis power state" << std::endl;
    } else {
        std::cout << "底盘电源状态信息:" << std::endl;
        std::cout << "  电池主电源开关: " << (int)chassis_power_state.battery_main_power_switch_state << std::endl;
        std::cout << "  急停踏板状态: " << (int)chassis_power_state.emergency_stop_pedal_state << std::endl;
        std::cout << "  底盘电源板状态: " << (int)chassis_power_state.chassis_power_board_state << std::endl;
        std::cout << "  底盘电源板温度: " << chassis_power_state.chassis_power_board_temperature << "°C" << std::endl;
        std::cout << "  充电插头输入电压: " << chassis_power_state.charge_plug_input_voltage << "V" << std::endl;
        std::cout << "  充电插头输入电流: " << chassis_power_state.charge_plug_input_current << "A" << std::endl;
        std::cout << "  电池数量: " << chassis_power_state.battery_states.size() << std::endl;

        for (size_t i = 0; i < chassis_power_state.battery_states.size(); i++) {
            const auto& battery = chassis_power_state.battery_states[i];
            std::cout << "  电池 " << i << ":" << std::endl;
            std::cout << "    电量: " << battery.battery_soc << "%" << std::endl;
            std::cout << "    健康度: " << (int)battery.battery_soh << "%" << std::endl;
            std::cout << "    电压: " << battery.battery_output_voltage << "V" << std::endl;
            std::cout << "    电流: " << battery.battery_output_current << "A" << std::endl;
            std::cout << "    温度: " << battery.battery_temperature << "°C" << std::endl;
        }

        std::cout << "  时间戳: " << chassis_power_state.timestamp << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 8. `GetChestPowerState()`

- **功能**：获取胸部电源状态
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `chest_power_state` | `ChestPowerState&` | 输出参数，胸部电源状态对象 |

- **返回值**：`GDKRes`，操作结果状态码。

#### ChestPowerState对象详细说明

**ChestPowerState结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `power_onoff_req` | `uint8_t` | 开关机请求 | 无单位 |
| `emergency_stop_button_req` | `uint8_t` | 急停按钮请求 | 无单位 |
| `power_switch_fault_state` | `uint8_t` | 电源开关故障状态 | 无单位 |
| `emergency_stop_button_fault_state` | `uint8_t` | 急停按钮故障状态 | 无单位 |
| `power_full_low_req` | `uint8_t` | 电源全/低请求 | 无单位 |
| `chest_power_board_power_state` | `uint8_t` | 胸部电源板电源状态 | 无单位 |
| `domain_controller_power_state` | `uint8_t` | 域控制器电源状态 | 无单位 |
| `head_interactive_board_power_state` | `uint8_t` | 头部交互板电源状态 | 无单位 |
| `curved_screen_power_state` | `uint8_t` | 曲面屏电源状态 | 无单位 |
| `head_yaw_motor_power_state` | `uint8_t` | 头部偏航电机电源状态 | 无单位 |
| `head_pitch_motor_power_state` | `uint8_t` | 头部俯仰电机电源状态 | 无单位 |
| `head_roll_motor_power_state` | `uint8_t` | 头部滚转电机电源状态 | 无单位 |
| `waist_yaw_motor_power_state` | `uint8_t` | 腰部偏航电机电源状态 | 无单位 |
| `head_motor_short_circuit_fault_state` | `uint8_t` | 头部电机短路故障状态 | 无单位 |
| `waist_pitch_motor_power_state` | `uint8_t` | 腰部俯仰电机电源状态 | 无单位 |
| `leg_bending1_motor_power_state` | `uint8_t` | 腿部弯曲1电机电源状态 | 无单位 |
| `leg_bending2_motor_power_state` | `uint8_t` | 腿部弯曲2电机电源状态 | 无单位 |
| `leg_bending3_motor_power_state` | `uint8_t` | 腿部弯曲3电机电源状态 | 无单位 |
| `waist_motor_short_circuit_fault_state` | `uint8_t` | 腰部电机短路故障状态 | 无单位 |
| `left_arm_power_state` | `uint8_t` | 左臂电源状态 | 无单位 |
| `left_arm_motor_short_circuit_fault_state` | `uint8_t` | 左臂电机短路故障状态 | 无单位 |
| `left_arm_brake_enable_state` | `uint8_t` | 左臂刹车使能状态 | 无单位 |
| `right_arm_power_state` | `uint8_t` | 右臂电源状态 | 无单位 |
| `right_arm_motor_short_circuit_fault_state` | `uint8_t` | 右臂电机短路故障状态 | 无单位 |
| `right_arm_brake_enable_state` | `uint8_t` | 右臂刹车使能状态 | 无单位 |
| `fan_power_state` | `uint8_t` | 风扇电源状态 | 无单位 |
| `chest_power_board_fan_fault_state` | `uint8_t` | 胸部电源板风扇故障状态 | 无单位 |
| `body_fan1_fault_state` | `uint8_t` | 身体风扇1故障状态 | 无单位 |
| `body_fan2_fault_state` | `uint8_t` | 身体风扇2故障状态 | 无单位 |
| `body_fan3_fault_state` | `uint8_t` | 身体风扇3故障状态 | 无单位 |
| `body_fan4_fault_state` | `uint8_t` | 身体风扇4故障状态 | 无单位 |
| `upper_body_led_strip_power_state` | `uint8_t` | 上身LED灯带电源状态 | 无单位 |
| `poe_power_state` | `uint8_t` | PoE电源状态 | 无单位 |
| `ipad_power_state` | `uint8_t` | iPad电源状态 | 无单位 |
| `chest_reserved_lidar_power_state` | `uint8_t` | 胸部预留激光雷达电源状态 | 无单位 |
| `chest_power_board_temperature` | `float` | 胸部电源板温度 | °C |
| `chest_power_board_fault_state` | `uint32_t` | 胸部电源板故障状态 | 无单位 |
| `timestamp` | `uint64_t` | 时间戳 | 纳秒 |

```
struct ChestPowerState {
  uint8_t power_onoff_req{0};
  uint8_t emergency_stop_button_req{0};
  uint8_t power_switch_fault_state{0};
  uint8_t emergency_stop_button_fault_state{0};
  uint8_t power_full_low_req{0};
  uint8_t chest_power_board_power_state{0};
  uint8_t domain_controller_power_state{0};
  uint8_t head_interactive_board_power_state{0};
  uint8_t curved_screen_power_state{0};
  uint8_t head_yaw_motor_power_state{0};
  uint8_t head_pitch_motor_power_state{0};
  uint8_t head_roll_motor_power_state{0};
  uint8_t waist_yaw_motor_power_state{0};
  uint8_t head_motor_short_circuit_fault_state{0};
  uint8_t waist_pitch_motor_power_state{0};
  uint8_t leg_bending1_motor_power_state{0};
  uint8_t leg_bending2_motor_power_state{0};
  uint8_t leg_bending3_motor_power_state{0};
  uint8_t waist_motor_short_circuit_fault_state{0};
  uint8_t left_arm_power_state{0};
  uint8_t left_arm_motor_short_circuit_fault_state{0};
  uint8_t left_arm_brake_enable_state{0};
  uint8_t right_arm_power_state{0};
  uint8_t right_arm_motor_short_circuit_fault_state{0};
  uint8_t right_arm_brake_enable_state{0};
  uint8_t fan_power_state{0};
  uint8_t chest_power_board_fan_fault_state{0};
  uint8_t body_fan1_fault_state{0};
  uint8_t body_fan2_fault_state{0};
  uint8_t body_fan3_fault_state{0};
  uint8_t body_fan4_fault_state{0};
  uint8_t upper_body_led_strip_power_state{0};
  uint8_t poe_power_state{0};
  uint8_t ipad_power_state{0};
  uint8_t chest_reserved_lidar_power_state{0};
  float chest_power_board_temperature{0.0};
  uint32_t chest_power_board_fault_state{0};
  uint64_t timestamp{0};
};
```

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Robot robot;
    std::cout << "Robot init" << std::endl;

    agibot::gdk::ChestPowerState chest_state;
    if (robot.GetChestPowerState(chest_state) == agibot::gdk::GDKRes::kSuccess) {
        std::cout << "胸部电源状态信息:" << std::endl;
        std::cout << "  开关机请求: " << (int)chest_state.power_onoff_req << std::endl;
        std::cout << "  急停按钮请求: " << (int)chest_state.emergency_stop_button_req << std::endl;
        std::cout << "  胸部电源板状态: " << (int)chest_state.chest_power_board_power_state << std::endl;
        std::cout << "  胸部电源板温度: " << chest_state.chest_power_board_temperature << "°C" << std::endl;
        std::cout << "  域控制器电源: " << (int)chest_state.domain_controller_power_state << std::endl;
        std::cout << "  左臂电源: " << (int)chest_state.left_arm_power_state << std::endl;
        std::cout << "  右臂电源: " << (int)chest_state.right_arm_power_state << std::endl;
        std::cout << "  左臂刹车使能: " << (int)chest_state.left_arm_brake_enable_state << std::endl;
        std::cout << "  右臂刹车使能: " << (int)chest_state.right_arm_brake_enable_state << std::endl;
        std::cout << "  头部偏航电机电源: " << (int)chest_state.head_yaw_motor_power_state << std::endl;
        std::cout << "  头部俯仰电机电源: " << (int)chest_state.head_pitch_motor_power_state << std::endl;
        std::cout << "  头部滚转电机电源: " << (int)chest_state.head_roll_motor_power_state << std::endl;
        std::cout << "  风扇电源: " << (int)chest_state.fan_power_state << std::endl;
        std::cout << "  时间戳: " << chest_state.timestamp << std::endl;
    } else {
        std::cout << "Failed to get chest power state" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 9. `JointControl()`

- **功能**：关节控制接口（路径规划控制，阻塞式），执行到目标位置后，接口返回。
- **支持的控制模式**：`1`（关节位控）、`3`（关节阻抗），使用关节阻抗模式前，请先通过`SetControlMode()`切换。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `joint_control_req` | `const JointControlReq&` | 关节控制请求对象 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`

#### JointControlReq对象详细说明

**JointControlReq结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `uuid` | `std::string` | 请求唯一标识 | 字符串 |
| `life_time` | `double` | 请求生命周期 | 秒 |
| `joint_names` | `std::vector<std::string>` | 关节名称列表 | 字符串列表 |
| `joint_positions` | `std::vector<double>` | 关节位置列表 | 弧度 |
| `joint_velocities` | `std::vector<double>` | 关节速度列表 | 弧度/秒 |
| `detail` | `std::string` | 详细信息 | 字符串 |

```
struct JointControlReq {
  double life_time{0.0};
  std::vector<std::string> joint_names{};
  std::vector<double> joint_positions{};
  std::vector<double> joint_velocities{};
  std::string uuid{};
  std::string detail{};
};
```

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <random>
#include <sstream>

std::string generate_uuid() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint32_t> dis(0, 0xFFFFFFFF);
    std::stringstream ss;
    ss << std::hex << dis(gen) << "-" << dis(gen) << "-" << dis(gen) << "-" << dis(gen);
    return ss.str();
}

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Robot robot;
    std::cout << "Robot init" << std::endl;

    agibot::gdk::JointControlReq joint_control;
    joint_control.life_time = 5.0;
    joint_control.joint_names = {"idx21_arm_l_joint1","idx22_arm_l_joint2","idx23_arm_l_joint3"};
    joint_control.joint_positions = {0.0, 0.0, 0.0};
    joint_control.joint_velocities = {0.0, 0.0, 0.0};
    joint_control.detail = "左臂关节控制";

    // 该接口支持 control_mode=1（关节位控）/ 3（关节阻抗），需先确保已切换；
    // 默认通常已是位控模式，若想用关节阻抗请先 robot.SetControlMode(3)。
    if (robot.JointControl(joint_control) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to control joints" << std::endl;
    } else {
        std::cout << "关节控制指令发送成功" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 10. `MoveHeadJoint()`

- **功能**：头部关节位置规划控制接口（阻塞式），执行到目标位置后，接口返回。
- **支持的控制模式**：`1`（关节位控）、`3`（关节阻抗），使用关节阻抗模式前，请先通过`SetControlMode()`切换。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `positions` | `std::vector<double>&` | 头部关节位置列表, 按照"idx11\_head\_joint1", "idx12\_head\_joint2", "idx13\_head\_joint3"填写目标关节角 |
| `velocities` | `const std::vector<double>&` | 头部关节速度列表, 按照"idx11\_head\_joint1", "idx12\_head\_joint2", "idx13\_head\_joint3"填写目标关节速度 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Robot robot;
    std::cout << "Robot init" << std::endl;

    std::vector<double> head_positions = {0.0, 0.0, 0.0};  // 头部关节位置
    std::vector<double> head_velocities = {0.3, 0.3, 0.3};  // 头部关节速度

    // 该接口支持 control_mode=1（关节位控）/ 3（关节阻抗），需先确保已切换；
    // 默认通常已是位控模式，若想用关节阻抗请先 robot.SetControlMode(3)。
    if (robot.MoveHeadJoint(head_positions, head_velocities) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to move head position" << std::endl;
    } else {
        std::cout << "头部位置控制成功" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 11. `MoveWaistJoint()`

- **功能**：腰部关节位置规划控制接口（阻塞式），执行到目标位置后，接口返回。
- **支持的控制模式**：`1`（关节位控）、`3`（关节阻抗），使用关节阻抗模式前，请先通过`SetControlMode()`切换。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `positions` | `std::vector<double>&` | 腰部关节位置列表, 按照"idx01\_body\_joint1", "idx02\_body\_joint2", "idx03\_body\_joint3", "idx04\_body\_joint4", "idx05\_body\_joint5"填写目标关节角 |
| `velocities` | `const std::vector<double>&` | 腰部关节速度列表, 按照"idx01\_body\_joint1", "idx02\_body\_joint2", "idx03\_body\_joint3", "idx04\_body\_joint4", "idx05\_body\_joint5"填写目标关节速度 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Robot robot;
    std::cout << "Robot init" << std::endl;

    std::vector<double> waist_positions = {0.0, 0.0, 0.0, 0.0, 0.0};  // 腰部关节位置
    std::vector<double> waist_velocities = {0.3, 0.3, 0.3, 0.3, 0.3};  // 腰部关节速度

    // 该接口支持 control_mode=1（关节位控）/ 3（关节阻抗），需先确保已切换；
    // 默认通常已是位控模式，若想用关节阻抗请先 robot.SetControlMode(3)。
    if (robot.MoveWaistJoint(waist_positions, waist_velocities) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to move waist position" << std::endl;
    } else {
        std::cout << "腰部位置控制成功" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 12. `MoveArmJoint()`

- **功能**：手臂关节位置规划控制接口（阻塞式），执行到目标位置后，接口返回。
- **支持的控制模式**：`1`（关节位控）、`3`（关节阻抗），使用关节阻抗模式前，请先通过`SetControlMode()`切换。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `positions` | `std::vector<double>&` | 手臂关节位置列表, 按照"idx21\_arm\_l\_joint1", "idx22\_arm\_l\_joint2", "idx23\_arm\_l\_joint3", |
| "idx24\_arm\_l\_joint4", "idx25\_arm\_l\_joint5", "idx26\_arm\_l\_joint6", |  |  |
| "idx27\_arm\_l\_joint7", "idx61\_arm\_r\_joint1", "idx62\_arm\_r\_joint2", |  |  |
| "idx63\_arm\_r\_joint3", "idx64\_arm\_r\_joint4", "idx65\_arm\_r\_joint5", |  |  |
| "idx66\_arm\_r\_joint6", "idx67\_arm\_r\_joint7"填写目标关节角 |  |  |
| `velocities` | `const std::vector<double>&` | 手臂关节速度列表, 按照"idx21\_arm\_l\_joint1", "idx22\_arm\_l\_joint2", "idx23\_arm\_l\_joint3", |
| "idx24\_arm\_l\_joint4", "idx25\_arm\_l\_joint5", "idx26\_arm\_l\_joint6", |  |  |
| "idx27\_arm\_l\_joint7", "idx61\_arm\_r\_joint1", "idx62\_arm\_r\_joint2", |  |  |
| "idx63\_arm\_r\_joint3", "idx64\_arm\_r\_joint4", "idx65\_arm\_r\_joint5", |  |  |
| "idx66\_arm\_r\_joint6", "idx67\_arm\_r\_joint7"填写目标关节速度 |  |  |
| `control_group` | `const int` | 控制组，0表示控制左臂，1表示控制右臂，2表示控制双臂 |
| - **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess` |  |  |

- **示例**：

```
#include "gdk/gdk.h"
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Robot robot;
    std::cout << "Robot init" << std::endl;

    std::vector<double> arm_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // 手臂关节位置
    std::vector<double> arm_velocities = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};  // 手臂关节速度

    // 该接口支持 control_mode=1（关节位控）/ 3（关节阻抗），需先确保已切换；
    // 默认通常已是位控模式，若想用关节阻抗请先 robot.SetControlMode(3)。
    if (robot.MoveArmJoint(arm_positions, arm_velocities, 2) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to move arm position" << std::endl;
    } else {
        std::cout << "手臂位置控制成功" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 13. `JointServoControl()`

- **功能**：关节位置伺服控制接口（非阻塞式），需要以100hz的控制频率进行控制，支持正常模式和低延时模式，默认使用正常模式。该接口支持同时控制末端执行器。
- **支持的控制模式**：`1`（关节位控）、`3`（关节阻抗），使用关节阻抗模式前，请先通过`SetControlMode()`切换。
- **注意**：低延时模式下，该接口绕过运控算法保护，直接驱动电机，仅在必要时使用。潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。开发者须自行评估环境与工况，并实现必要的安全保护措施。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `joint_servo_control_req` | `const JointServoControlReq&` | 关节位置伺服请求对象 |
| `enable_low_latency` | `bool` | 是否启用低延时模式，默认为`false` ｜ |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`

#### JointServoControlReq对象详细说明

**JointServoControlReq结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `control_period` | `double` | 控制周期 | 秒 |
| `joint_names` | `std::vector<std::string>` | 关节名称列表 | 字符串列表 |
| `joint_positions` | `std::vector<double>` | 关节位置列表 | 弧度 |
| `joint_velocities` | `std::vector<double>` | 关节速度列表 | 弧度/秒 |

```
struct JointServoControlReq {
  double control_period{0.0};
  std::vector<std::string> joint_names{};
  std::vector<double> joint_positions{};
  std::vector<double> joint_velocities{};
};
```

**参数说明**：
- `control_period` 表示控制周期，单位为秒，建议比当前控制频率稍大
- `joint_names`、`joint_positions`、`joint_velocities` 三个列表的长度必须相同
- `joint_positions` 中的值必须在对应关节的限位范围内（限位参考值见 [`GetJointLimits()`](#2-getjointlimits)），否则返回 `ErrorCode::kInvalidInput`
- `joint_names`、`joint_positions`、`joint_velocities` 不能为空，否则返回 `ErrorCode::kInvalidInput`
- `joint_velocities` 预留参数，目前不使用，可以为空

- **错误处理**：
- 如果 `joint_names`、`joint_positions`为空，返回 `ErrorCode::kInvalidInput`
- 如果 `joint_positions` 中的值超出对应关节的限位范围，返回 `ErrorCode::kInvalidInput`
- 如果配置解析器获取失败，返回 `ErrorCode::kRuntimeError`
- 其他错误情况返回相应的错误码
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <algorithm>

using namespace agibot::gdk;

// 控制参数
const double CONTROL_PERIOD = 0.01;  // 控制周期（秒）
const double RATE_HZ = 100.0;        // 发送频率（Hz）
const double DURATION = 5.0;         // 控制持续时间（秒）

class JointServoControlController {
private:
    Robot robot_;

    double getJointPositionByName(const JointStates& joint_states, const std::string& joint_name) {
        for (const auto& state : joint_states.states) {
            if (state.name == joint_name) {
                return state.motor_position;
            }
        }
        throw std::runtime_error("Joint name " + joint_name + " not found");
    }

    double interpolatePosition(double start_pos, double target_pos, double t) {
        return start_pos + t * (target_pos - start_pos);
    }

public:
    void executeJointServoControl(const std::vector<std::string>& target_joint_names,
                                   const std::vector<double>& target_positions) {
        std::this_thread::sleep_for(std::chrono::seconds(1));  // 等待1秒

        // 获取当前关节状态
        JointStates current_joint_states;
        if (robot_.GetJointStates(current_joint_states) != GDKRes::kSuccess) {
            std::cout << "获取关节状态失败" << std::endl;
            return;
        }
        std::cout << "当前关节数量: " << current_joint_states.nums << std::endl;

        // 获取起始位置
        std::vector<double> start_positions;
        for (const auto& joint_name : target_joint_names) {
            try {
                double pos = getJointPositionByName(current_joint_states, joint_name);
                start_positions.push_back(pos);
                std::cout << "关节 " << joint_name << " 当前位置: " << pos << " 弧度" << std::endl;
            } catch (const std::exception& e) {
                std::cout << "错误: " << e.what() << std::endl;
                return;
            }
        }

        // 计算步数
        int n_steps = static_cast<int>(DURATION * RATE_HZ);
        std::cout << "总步数: " << n_steps << ", 持续时间: " << DURATION << " 秒" << std::endl;

        // 执行轨迹
        double dt = 1.0 / RATE_HZ;
        auto start_time = std::chrono::steady_clock::now();

        for (int i = 0; i < n_steps; i++) {
            double t = static_cast<double>(i) / (n_steps - 1);

            // 创建关节位置伺服请求
            JointServoControlReq joint_position_servo_req;
            joint_position_servo_req.control_period = CONTROL_PERIOD;

            // 计算当前目标位置（线性插值）
            std::vector<double> current_positions;
            for (size_t j = 0; j < target_joint_names.size(); j++) {
                double interp_pos = interpolatePosition(
                    start_positions[j], target_positions[j], t
                );
                current_positions.push_back(interp_pos);
            }

            joint_position_servo_req.joint_names = target_joint_names;
            joint_position_servo_req.joint_positions = current_positions;

            // 使用普通模式（enable_low_latency=false，默认值）
            auto res = robot_.JointServoControl(joint_position_servo_req);
            // 如需使用低延时模式，可传入 enable_low_latency=true：
            // 注意：低延时模式下，该接口绕过运控算法保护，直接驱动电机。
            // 潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。
            // auto res = robot_.JointServoControl(joint_position_servo_req, true);
            if (res != GDKRes::kSuccess) {
                std::cout << "控制命令发送失败，步数: " << i << std::endl;
                return;
            }

            // 控制发送频率
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            auto expected_time = std::chrono::milliseconds(static_cast<int>((i + 1) * dt * 1000));
            auto sleep_time = expected_time - elapsed;
            if (sleep_time.count() > 0) {
                std::this_thread::sleep_for(sleep_time);
            }
        }

        std::cout << "关节位置伺服控制完成" << std::endl;

        // 保持最终位置
        std::cout << "进入最终位置保持（Ctrl+C 结束）..." << std::endl;
        try {
            while (true) {
                JointServoControlReq joint_position_servo_req;
                joint_position_servo_req.control_period = CONTROL_PERIOD;
                joint_position_servo_req.joint_names = target_joint_names;
                joint_position_servo_req.joint_positions = target_positions;

                // 使用普通模式（enable_low_latency=false，默认值）
                auto res = robot_.JointServoControl(joint_position_servo_req);
                // 如需使用低延时模式，可传入 enable_low_latency=true：
                // 注意：低延时模式下，该接口绕过运控算法保护，直接驱动电机。
                // 潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。
                // auto res = robot_.JointServoControl(joint_position_servo_req, true);
                if (res != GDKRes::kSuccess) {
                    std::cout << "保持位置失败" << std::endl;
                    break;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
            }
        } catch (const std::exception& e) {
            std::cout << "已中断保持: " << e.what() << std::endl;
        }
    }
};

int main() {
    // 初始化GDK系统
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    Robot robot;
    std::cout << "Robot init" << std::endl;

    // 获取当前关节状态
    JointStates current_joint_states;
    if (robot.GetJointStates(current_joint_states) != GDKRes::kSuccess) {
        std::cout << "获取关节状态失败" << std::endl;
        GDKRelease();
        return -1;
    }
    std::cout << "当前关节数量: " << current_joint_states.nums << std::endl;

    // 定义要控制的关节（示例：左臂前3个关节）
    std::vector<std::string> target_joint_names = {
        "idx21_arm_l_joint1",
        "idx22_arm_l_joint2",
        "idx23_arm_l_joint3"
    };

    // 获取当前位置作为起始位置（用于插值）
    std::vector<double> start_positions;
    for (const auto& joint_name : target_joint_names) {
        bool found = false;
        for (const auto& state : current_joint_states.states) {
            if (state.name == joint_name) {
                start_positions.push_back(state.motor_position);
                std::cout << "关节 " << joint_name << " 当前位置: " << state.motor_position << " 弧度" << std::endl;
                found = true;
                break;
            }
        }
        if (!found) {
            std::cout << "错误: 关节 " << joint_name << " 未找到" << std::endl;
            GDKRelease();
            return -1;
        }
    }

    // 设置目标角度（直接指定目标角度值，单位：弧度）
    std::vector<double> target_positions = {
        0.0,  // idx21_arm_l_joint1: 目标角度 0.0 弧度
        0.0,  // idx22_arm_l_joint2: 目标角度 0.0 弧度
        0.0   // idx23_arm_l_joint3: 目标角度 0.0 弧度
    };

    std::cout << "\n目标角度:" << std::endl;
    for (size_t i = 0; i < target_joint_names.size(); i++) {
        std::cout << "  " << target_joint_names[i] << ": " << target_positions[i] << " 弧度" << std::endl;
    }

    // 执行关节位置伺服控制
    // 该接口支持 control_mode=1（关节位控）/ 3（关节阻抗），需先确保已切换；
    // 默认通常已是位控模式，若想用关节阻抗请先 robot.SetControlMode(3)。
    JointServoControlController controller;
    controller.executeJointServoControl(
        target_joint_names, target_positions);

    // 释放GDK系统资源
    if (GDKRelease() != GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

- **同时控制机械臂与末端的示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>

using namespace agibot::gdk;

const double CONTROL_PERIOD = 0.01;
const double RATE_HZ = 100.0;
const double DURATION = 3.0;
const double HOLD_DURATION = 0.5;    // 在 -0.785 或 0 处保持的时间（秒）
const int NUM_CYCLES = 3;            // 往复运动次数

static const std::vector<std::string> ARM_L_JOINT_NAMES = {
    "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3",
    "idx24_arm_l_joint4", "idx25_arm_l_joint5", "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
};

// 从 GetJointStates 的返回中按关节名取出位置列表
std::vector<double> getArmPositionsByName(const JointStates& joint_states,
                                          const std::vector<std::string>& joint_names) {
    std::vector<double> positions;
    for (const auto& name : joint_names) {
        for (const auto& state : joint_states.states) {
            if (state.name == name) {
                positions.push_back(state.motor_position);
                break;
            }
        }
    }
    return positions;
}

// 从 GetEndState 的返回中取出指定侧末端的关节名列表和位置列表
// use_left 为 true 取 left_end_state，为 false 取 right_end_state
void getEENamesAndPositions(const DualEndState& end_state,
                            bool use_left,
                            std::vector<std::string>& names,
                            std::vector<double>& positions) {
    const EndState& state = use_left ? end_state.left_end_state : end_state.right_end_state;
    names = state.names;
    positions.clear();
    for (const auto& motor : state.end_states) {
        positions.push_back(motor.position);
    }
}

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    Robot robot;

    JointStates joint_states;
    if (robot.GetJointStates(joint_states) != GDKRes::kSuccess) {
        std::cout << "获取关节状态失败" << std::endl;
        GDKRelease();
        return -1;
    }

    DualEndState end_state;
    if (robot.GetEndState(end_state) != GDKRes::kSuccess) {
        std::cout << "获取末端状态失败" << std::endl;
        GDKRelease();
        return -1;
    }

    std::vector<std::string> ee_names;
    std::vector<double> ee_positions;
    getEENamesAndPositions(end_state, true, ee_names, ee_positions);  // 左侧末端

    std::vector<std::string> all_names = ARM_L_JOINT_NAMES;
    all_names.insert(all_names.end(), ee_names.begin(), ee_names.end());

    std::vector<double> arm_positions = getArmPositionsByName(joint_states, ARM_L_JOINT_NAMES);
    if (arm_positions.size() != ARM_L_JOINT_NAMES.size()) {
        std::cout << "左臂关节数量不匹配" << std::endl;
        GDKRelease();
        return -1;
    }

    // 机械臂按之前方式做小幅关节空间运动（来回），末端关节在 -0.785 和 0 之间往复运动
    std::vector<double> arm_start = arm_positions;                    // 机械臂初始姿态
    std::vector<double> arm_target = arm_positions;                   // 机械臂目标姿态（小幅偏移）
    for (auto& v : arm_target) v += 0.05;
    const double ee_low = -0.785;                                     // 末端下限
    const double ee_high = 0.0;                                       // 末端上限

    std::cout << "控制关节数: 机械臂 " << ARM_L_JOINT_NAMES.size()
              << " + 末端 " << ee_names.size() << " = " << all_names.size() << std::endl;
    std::cout << "机械臂将在关节空间 arm_start ↔ arm_target 之间运动，末端关节在 "
              << ee_low << " 和 " << ee_high << " 之间往复 " << NUM_CYCLES << " 次" << std::endl;

    int n_steps = static_cast<int>(DURATION * RATE_HZ);           // 单次移动的步数
    int hold_steps = static_cast<int>(HOLD_DURATION * RATE_HZ);   // 保持阶段的步数
    double dt = 1.0 / RATE_HZ;

    // 辅助函数：发送控制命令
    // JointServoControl 支持 control_mode=1（关节位控）/ 3（关节阻抗），
    // 默认通常已是位控模式，若想用关节阻抗请先 robot.SetControlMode(3)。
    auto sendControlCommand = [&](const std::vector<double>& arm_vals, const std::vector<double>& ee_vals) -> bool {
        std::vector<double> current = arm_vals;
        current.insert(current.end(), ee_vals.begin(), ee_vals.end());
        JointServoControlReq req;
        req.control_period = CONTROL_PERIOD;
        req.joint_names = all_names;
        req.joint_positions = current;
        req.joint_velocities = std::vector<double>(current.size(), 0.0);
        return robot.JointServoControl(req) == GDKRes::kSuccess;
    };

    for (int cycle = 0; cycle < NUM_CYCLES; cycle++) {
        std::cout << "\n=== 周期 " << (cycle + 1) << "/" << NUM_CYCLES
                  << ": 机械臂 arm_start -> arm_target, 末端 " << ee_low << " -> " << ee_high << " ===" << std::endl;
        auto start_time = std::chrono::steady_clock::now();

        // 第一段：机械臂 arm_start -> arm_target，末端 ee_low -> ee_high
        for (int i = 0; i < n_steps; i++) {
            double t = (n_steps > 1) ? static_cast<double>(i) / (n_steps - 1) : 1.0;
            std::vector<double> current_arm;
            for (size_t j = 0; j < arm_start.size(); j++) {
                current_arm.push_back(arm_start[j] + t * (arm_target[j] - arm_start[j]));
            }
            std::vector<double> current_ee(ee_names.size(), ee_low + t * (ee_high - ee_low));

            if (!sendControlCommand(current_arm, current_ee)) {
                std::cout << "发送失败，周期=" << (cycle + 1) << ", 阶段=0->1, 步数=" << i << std::endl;
                GDKRelease();
                return -1;
            }

            auto elapsed = std::chrono::steady_clock::now() - start_time;
            auto expected_ms = static_cast<int>((i + 1) * dt * 1000);
            auto sleep_time = std::chrono::milliseconds(expected_ms) - elapsed;
            if (sleep_time.count() > 0) {
                std::this_thread::sleep_for(sleep_time);
            }
        }

        // 第二段：在 arm_target / ee_high 处保持
        std::cout << "=== 周期 " << (cycle + 1) << "/" << NUM_CYCLES
                  << ": 在 arm_target / " << ee_high << " 处保持 " << HOLD_DURATION << " 秒 ===" << std::endl;
        start_time = std::chrono::steady_clock::now();
        std::vector<double> current_arm = arm_target;
        std::vector<double> current_ee(ee_names.size(), ee_high);

        for (int i = 0; i < hold_steps; i++) {
            if (!sendControlCommand(current_arm, current_ee)) {
                std::cout << "发送失败，周期=" << (cycle + 1) << ", 阶段=保持@1, 步数=" << i << std::endl;
                GDKRelease();
                return -1;
            }

            auto elapsed = std::chrono::steady_clock::now() - start_time;
            auto expected_ms = static_cast<int>((i + 1) * dt * 1000);
            auto sleep_time = std::chrono::milliseconds(expected_ms) - elapsed;
            if (sleep_time.count() > 0) {
                std::this_thread::sleep_for(sleep_time);
            }
        }

        // 第三段：机械臂 arm_target -> arm_start，末端 ee_high -> ee_low
        std::cout << "=== 周期 " << (cycle + 1) << "/" << NUM_CYCLES
                  << ": 机械臂 arm_target -> arm_start, 末端 " << ee_high << " -> " << ee_low << " ===" << std::endl;
        start_time = std::chrono::steady_clock::now();

        for (int i = 0; i < n_steps; i++) {
            double t = (n_steps > 1) ? static_cast<double>(i) / (n_steps - 1) : 1.0;
            std::vector<double> current_arm;
            for (size_t j = 0; j < arm_target.size(); j++) {
                current_arm.push_back(arm_target[j] + t * (arm_start[j] - arm_target[j]));
            }
            std::vector<double> current_ee(ee_names.size(), ee_high + t * (ee_low - ee_high));

            if (!sendControlCommand(current_arm, current_ee)) {
                std::cout << "发送失败，周期=" << (cycle + 1) << ", 阶段=1->0, 步数=" << i << std::endl;
                GDKRelease();
                return -1;
            }

            auto elapsed = std::chrono::steady_clock::now() - start_time;
            auto expected_ms = static_cast<int>((i + 1) * dt * 1000);
            auto sleep_time = std::chrono::milliseconds(expected_ms) - elapsed;
            if (sleep_time.count() > 0) {
                std::this_thread::sleep_for(sleep_time);
            }
        }

        // 第四段：在 arm_start / ee_low 处保持
        std::cout << "=== 周期 " << (cycle + 1) << "/" << NUM_CYCLES
                  << ": 在 arm_start / " << ee_low << " 处保持 " << HOLD_DURATION << " 秒 ===" << std::endl;
        start_time = std::chrono::steady_clock::now();
        current_arm = arm_start;
        current_ee = std::vector<double>(ee_names.size(), ee_low);

        for (int i = 0; i < hold_steps; i++) {
            if (!sendControlCommand(current_arm, current_ee)) {
                std::cout << "发送失败，周期=" << (cycle + 1) << ", 阶段=保持@0, 步数=" << i << std::endl;
                GDKRelease();
                return -1;
            }

            auto elapsed = std::chrono::steady_clock::now() - start_time;
            auto expected_ms = static_cast<int>((i + 1) * dt * 1000);
            auto sleep_time = std::chrono::milliseconds(expected_ms) - elapsed;
            if (sleep_time.count() > 0) {
                std::this_thread::sleep_for(sleep_time);
            }
        }
    }

    std::cout << "\n机械臂 arm_start↔arm_target、末端 " << ee_low << "↔" << ee_high << " 往复控制结束" << std::endl;

    if (GDKRelease() != GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;
    return 0;
}
```

#### 14. `MoveHeadJointServo()`

- **功能**：头部关节位置伺服控制接口（非阻塞式），需要以100hz的控制频率进行控制，支持正常模式和低延时模式，默认使用正常模式。
- **支持的控制模式**：`1`（关节位控）、`3`（关节阻抗），使用关节阻抗模式前，请先通过`SetControlMode()`切换。
- **注意**：低延时模式下，该接口绕过运控算法保护，直接驱动电机，仅在必要时使用。潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。开发者须自行评估环境与工况，并实现必要的安全保护措施。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `positions` | `const std::vector<double>&` | 头部关节位置列表（弧度），按照"idx11\_head\_joint1", "idx12\_head\_joint2", "idx13\_head\_joint3"顺序 |
| `control_period` | `const double` | 控制周期（秒），建议比控制频率稍大 |
| `enable_low_latency` | `const bool` | 是否启用低延时模式，默认为`false` |

**参数说明**：
- `positions` 长度必须为3
- `positions` 中的值必须在对应关节的限位范围内（限位参考值见 [`GetJointLimits()`](#2-getjointlimits)），否则返回 `ErrorCode::kInvalidInput`
- `control_period` 表示控制周期，单位为秒，建议比控制频率稍大
- `enable_low_latency` 用于选择控制通道，低延时模式适用于对实时性要求更高的场景

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **错误处理**：
- 如果 `positions` 的长度不为3，返回 `ErrorCode::kInvalidInput`
- 如果 `positions` 中的值超出对应关节的限位范围，返回 `ErrorCode::kInvalidInput`
- 其他错误情况返回相应的错误码
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <algorithm>

using namespace agibot::gdk;

// 控制参数
const double CONTROL_PERIOD = 0.01;  // 控制周期（秒）
const double RATE_HZ = 100.0;        // 发送频率（Hz）
const double DURATION = 5.0;         // 控制持续时间（秒）

class HeadJointServoController {
private:
    Robot robot_;

    double getJointPositionByName(const JointStates& joint_states, const std::string& joint_name) {
        for (const auto& state : joint_states.states) {
            if (state.name == joint_name) {
                return state.motor_position;
            }
        }
        throw std::runtime_error("Joint name " + joint_name + " not found");
    }

    double interpolatePosition(double start_pos, double target_pos, double t) {
        return start_pos + t * (target_pos - start_pos);
    }

public:
    void executeHeadJointServoControl(const std::vector<double>& target_positions) {
        std::this_thread::sleep_for(std::chrono::seconds(1));  // 等待1秒

        // 获取当前关节状态
        JointStates current_joint_states;
        if (robot_.GetJointStates(current_joint_states) != GDKRes::kSuccess) {
            std::cout << "获取关节状态失败" << std::endl;
            return;
        }

        // 获取起始位置
        std::vector<std::string> head_joint_names = {
            "idx11_head_joint1", "idx12_head_joint2", "idx13_head_joint3"
        };
        std::vector<double> start_positions;
        for (const auto& joint_name : head_joint_names) {
            try {
                double pos = getJointPositionByName(current_joint_states, joint_name);
                start_positions.push_back(pos);
                std::cout << "关节 " << joint_name << " 当前位置: " << pos << " 弧度" << std::endl;
            } catch (const std::exception& e) {
                std::cout << "错误: " << e.what() << std::endl;
                return;
            }
        }

        // 计算步数
        int n_steps = static_cast<int>(DURATION * RATE_HZ);
        std::cout << "总步数: " << n_steps << ", 持续时间: " << DURATION << " 秒" << std::endl;

        // 执行轨迹
        double dt = 1.0 / RATE_HZ;
        auto start_time = std::chrono::steady_clock::now();

        for (int i = 0; i < n_steps; i++) {
            double t = static_cast<double>(i) / (n_steps - 1);

            // 计算当前目标位置（线性插值）
            std::vector<double> current_positions;
            for (size_t j = 0; j < 3; j++) {
                double interp_pos = interpolatePosition(
                    start_positions[j], target_positions[j], t
                );
                current_positions.push_back(interp_pos);
            }

            // 使用普通模式（enable_low_latency=false，默认值）
            auto res = robot_.MoveHeadJointServo(
                current_positions, CONTROL_PERIOD);
            // 如需使用低延时模式，可传入 enable_low_latency=true：
            // 注意：低延时模式下，该接口绕过运控算法保护，直接驱动电机。
            // 潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。
            // auto res = robot_.MoveHeadJointServo(
            //     current_positions, target_velocities, CONTROL_PERIOD, true
            // );
            if (res != GDKRes::kSuccess) {
                std::cout << "控制命令发送失败，步数: " << i << std::endl;
                return;
            }

            // 控制发送频率
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            auto expected_time = std::chrono::milliseconds(static_cast<int>((i + 1) * dt * 1000));
            auto sleep_time = expected_time - elapsed;
            if (sleep_time.count() > 0) {
                std::this_thread::sleep_for(sleep_time);
            }
        }

        std::cout << "头部关节位置伺服控制完成" << std::endl;

        // 保持最终位置
        std::cout << "进入最终位置保持（Ctrl+C 结束）..." << std::endl;
        try {
            while (true) {
                // 使用普通模式（enable_low_latency=false，默认值）
                auto res = robot_.MoveHeadJointServo(
                    target_positions, CONTROL_PERIOD);
                // 如需使用低延时模式，可传入 enable_low_latency=true：
                // 注意：低延时模式下，该接口绕过运控算法保护，直接驱动电机。
                // 潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。
                // auto res = robot_.MoveHeadJointServo(
                //     target_positions, target_velocities, CONTROL_PERIOD, true
                // );
                if (res != GDKRes::kSuccess) {
                    std::cout << "保持位置失败" << std::endl;
                    break;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
            }
        } catch (const std::exception& e) {
            std::cout << "已中断保持: " << e.what() << std::endl;
        }
    }
};

int main() {
    // 初始化GDK系统
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    Robot robot;
    std::cout << "Robot init" << std::endl;

    // 获取当前关节状态
    JointStates current_joint_states;
    if (robot.GetJointStates(current_joint_states) != GDKRes::kSuccess) {
        std::cout << "获取关节状态失败" << std::endl;
        GDKRelease();
        return -1;
    }

    // 定义目标位置（示例：头部关节）
    std::vector<double> target_positions = {
        0.0,  // idx11_head_joint1: 目标角度 0.0 弧度
        0.0,  // idx12_head_joint2: 目标角度 0.0 弧度
        0.0   // idx13_head_joint3: 目标角度 0.0 弧度
    };

    std::cout << "\n目标角度:" << std::endl;
    std::vector<std::string> head_joint_names = {
        "idx11_head_joint1", "idx12_head_joint2", "idx13_head_joint3"
    };
    for (size_t i = 0; i < head_joint_names.size(); i++) {
        std::cout << "  " << head_joint_names[i] << ": " << target_positions[i] << " 弧度" << std::endl;
    }

    // 执行头部关节位置伺服控制
    // 该接口支持 control_mode=1（关节位控）/ 3（关节阻抗），需先确保已切换；
    // 默认通常已是位控模式，若想用关节阻抗请先 robot.SetControlMode(3)。
    HeadJointServoController controller;
    controller.executeHeadJointServoControl(target_positions);

    // 释放GDK系统资源
    if (GDKRelease() != GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 15. `MoveWaistJointServo()`

- **功能**：腰部关节位置伺服控制接口（非阻塞式），需要以100hz的控制频率进行控制，支持正常模式和低延时模式，默认使用正常模式。
- **支持的控制模式**：`1`（关节位控）、`3`（关节阻抗），使用关节阻抗模式前，请先通过`SetControlMode()`切换。
- **注意**：低延时模式下，该接口绕过运控算法保护，直接驱动电机，仅在必要时使用。潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。开发者须自行评估环境与工况，并实现必要的安全保护措施。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `positions` | `const std::vector<double>&` | 腰部关节位置列表（弧度），按照"idx01\_body\_joint1", "idx02\_body\_joint2", "idx03\_body\_joint3", "idx04\_body\_joint4", "idx05\_body\_joint5"顺序 |
| `control_period` | `const double` | 控制周期（秒），建议比控制频率稍大 |
| `enable_low_latency` | `const bool` | 是否启用低延时模式，默认为`false` |

**参数说明**：
- `positions` 的长度必须为5
- `positions` 中的值必须在对应关节的限位范围内（限位参考值见 [`GetJointLimits()`](#2-getjointlimits)），否则返回 `ErrorCode::kInvalidInput`
- `control_period` 表示控制周期，单位为秒，建议比控制频率稍大
- `enable_low_latency` 用于选择控制通道，低延时模式适用于对实时性要求更高的场景

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **错误处理**：
- 如果 `positions` 的长度不为5，返回 `ErrorCode::kInvalidInput`
- 如果 `positions` 中的值超出对应关节的限位范围，返回 `ErrorCode::kInvalidInput`
- 其他错误情况返回相应的错误码
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <algorithm>

using namespace agibot::gdk;

// 控制参数
const double CONTROL_PERIOD = 0.01;  // 控制周期（秒）
const double RATE_HZ = 100.0;        // 发送频率（Hz）
const double DURATION = 5.0;         // 控制持续时间（秒）

class WaistJointServoController {
private:
    Robot robot_;

    double getJointPositionByName(const JointStates& joint_states, const std::string& joint_name) {
        for (const auto& state : joint_states.states) {
            if (state.name == joint_name) {
                return state.motor_position;
            }
        }
        throw std::runtime_error("Joint name " + joint_name + " not found");
    }

    double interpolatePosition(double start_pos, double target_pos, double t) {
        return start_pos + t * (target_pos - start_pos);
    }

public:
    void executeWaistJointServoControl(const std::vector<double>& target_positions) {
        std::this_thread::sleep_for(std::chrono::seconds(1));  // 等待1秒

        // 获取当前关节状态
        JointStates current_joint_states;
        if (robot_.GetJointStates(current_joint_states) != GDKRes::kSuccess) {
            std::cout << "获取关节状态失败" << std::endl;
            return;
        }

        // 获取起始位置
        std::vector<std::string> waist_joint_names = {
            "idx01_body_joint1", "idx02_body_joint2", "idx03_body_joint3",
            "idx04_body_joint4", "idx05_body_joint5"
        };
        std::vector<double> start_positions;
        for (const auto& joint_name : waist_joint_names) {
            try {
                double pos = getJointPositionByName(current_joint_states, joint_name);
                start_positions.push_back(pos);
                std::cout << "关节 " << joint_name << " 当前位置: " << pos << " 弧度" << std::endl;
            } catch (const std::exception& e) {
                std::cout << "错误: " << e.what() << std::endl;
                return;
            }
        }

        // 计算步数
        int n_steps = static_cast<int>(DURATION * RATE_HZ);
        std::cout << "总步数: " << n_steps << ", 持续时间: " << DURATION << " 秒" << std::endl;

        // 执行轨迹
        double dt = 1.0 / RATE_HZ;
        auto start_time = std::chrono::steady_clock::now();

        for (int i = 0; i < n_steps; i++) {
            double t = static_cast<double>(i) / (n_steps - 1);

            // 计算当前目标位置（线性插值）
            std::vector<double> current_positions;
            for (size_t j = 0; j < 5; j++) {
                double interp_pos = interpolatePosition(
                    start_positions[j], target_positions[j], t
                );
                current_positions.push_back(interp_pos);
            }

            // 使用普通模式（enable_low_latency=false，默认值）
            auto res = robot_.MoveWaistJointServo(
                current_positions, CONTROL_PERIOD);
            // 如需使用低延时模式，可传入 enable_low_latency=true：
            // 注意：低延时模式下，该接口绕过运控算法保护，直接驱动电机。
            // 潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。
            // auto res = robot_.MoveWaistJointServo(
            //     current_positions, target_velocities, CONTROL_PERIOD, true
            // );
            if (res != GDKRes::kSuccess) {
                std::cout << "控制命令发送失败，步数: " << i << std::endl;
                return;
            }

            // 控制发送频率
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            auto expected_time = std::chrono::milliseconds(static_cast<int>((i + 1) * dt * 1000));
            auto sleep_time = expected_time - elapsed;
            if (sleep_time.count() > 0) {
                std::this_thread::sleep_for(sleep_time);
            }
        }

        std::cout << "腰部关节位置伺服控制完成" << std::endl;

        // 保持最终位置
        std::cout << "进入最终位置保持（Ctrl+C 结束）..." << std::endl;
        try {
            while (true) {
                // 使用普通模式（enable_low_latency=false，默认值）
                auto res = robot_.MoveWaistJointServo(
                    target_positions, CONTROL_PERIOD);
                // 如需使用低延时模式，可传入 enable_low_latency=true：
                // 注意：低延时模式下，该接口绕过运控算法保护，直接驱动电机。
                // 潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。
                // auto res = robot_.MoveWaistJointServo(
                //     target_positions, target_velocities, CONTROL_PERIOD, true
                // );
                if (res != GDKRes::kSuccess) {
                    std::cout << "保持位置失败" << std::endl;
                    break;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
            }
        } catch (const std::exception& e) {
            std::cout << "已中断保持: " << e.what() << std::endl;
        }
    }
};

int main() {
    // 初始化GDK系统
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    Robot robot;
    std::cout << "Robot init" << std::endl;

    // 获取当前关节状态
    JointStates current_joint_states;
    if (robot.GetJointStates(current_joint_states) != GDKRes::kSuccess) {
        std::cout << "获取关节状态失败" << std::endl;
        GDKRelease();
        return -1;
    }

    // 定义目标位置（示例：腰部关节）
    std::vector<double> target_positions = {
        0.0,  // idx01_body_joint1: 目标角度 0.0 弧度
        0.0,  // idx02_body_joint2: 目标角度 0.0 弧度
        0.0,  // idx03_body_joint3: 目标角度 0.0 弧度
        0.0,  // idx04_body_joint4: 目标角度 0.0 弧度
        0.0   // idx05_body_joint5: 目标角度 0.0 弧度
    };

    std::cout << "\n目标角度:" << std::endl;
    std::vector<std::string> waist_joint_names = {
        "idx01_body_joint1", "idx02_body_joint2", "idx03_body_joint3",
        "idx04_body_joint4", "idx05_body_joint5"
    };
    for (size_t i = 0; i < waist_joint_names.size(); i++) {
        std::cout << "  " << waist_joint_names[i] << ": " << target_positions[i] << " 弧度" << std::endl;
    }

    // 执行腰部关节位置伺服控制
    // 该接口支持 control_mode=1（关节位控）/ 3（关节阻抗），需先确保已切换；
    // 默认通常已是位控模式，若想用关节阻抗请先 robot.SetControlMode(3)。
    WaistJointServoController controller;
    controller.executeWaistJointServoControl(target_positions);

    // 释放GDK系统资源
    if (GDKRelease() != GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 16. `MoveArmJointServo()`

- **功能**：手臂关节位置伺服控制接口（非阻塞式），需要以100hz的控制频率进行控制，支持正常模式和低延时模式，默认使用正常模式。
- **支持的控制模式**：`1`（关节位控）、`3`（关节阻抗），使用关节阻抗模式前，请先通过`SetControlMode()`切换。
- **注意**：低延时模式下，该接口绕过运控算法保护，直接驱动电机，仅在必要时使用。潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。开发者须自行评估环境与工况，并实现必要的安全保护措施。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `positions` | `const std::vector<double>&` | 手臂关节位置列表（弧度），按照"idx21\_arm\_l\_joint1", "idx22\_arm\_l\_joint2", "idx23\_arm\_l\_joint3", "idx24\_arm\_l\_joint4", "idx25\_arm\_l\_joint5", "idx26\_arm\_l\_joint6", "idx27\_arm\_l\_joint7", "idx61\_arm\_r\_joint1", "idx62\_arm\_r\_joint2", "idx63\_arm\_r\_joint3", "idx64\_arm\_r\_joint4", "idx65\_arm\_r\_joint5", "idx66\_arm\_r\_joint6", "idx67\_arm\_r\_joint7"顺序 |
| `control_period` | `const double` | 控制周期（秒），建议比控制频率稍大 |
| `control_group` | `const int` | 控制组，0表示控制左臂，1表示控制右臂，2表示控制双臂 |
| `enable_low_latency` | `const bool` | 是否启用低延时模式，默认为`false` |

**参数说明**：
- `positions` 的长度为7(左臂或右臂)或14(双臂)
- `positions` 中的值必须在对应关节的限位范围内（限位参考值见 [`GetJointLimits()`](#2-getjointlimits)），否则返回 `ErrorCode::kInvalidInput`
- `control_period` 表示控制周期，单位为秒，建议比控制频率稍大
- `control_group` 表示控制组，0表示控制左臂，1表示控制右臂，2表示控制双臂
- `enable_low_latency` 用于选择控制通道，低延时模式适用于对实时性要求更高的场景

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **错误处理**：
- 如果 `positions` 的长度不为7(左臂或右臂)或14(双臂)，返回 `ErrorCode::kInvalidInput`
- 如果 `positions` 中的值超出对应关节的限位范围，返回 `ErrorCode::kInvalidInput`
- 其他错误情况返回相应的错误码
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <algorithm>

using namespace agibot::gdk;

// 控制参数
const double CONTROL_PERIOD = 0.01;  // 控制周期（秒）
const double RATE_HZ = 100.0;        // 发送频率（Hz）
const double DURATION = 5.0;         // 控制持续时间（秒）

class ArmJointServoController {
private:
    Robot robot_;

    double getJointPositionByName(const JointStates& joint_states, const std::string& joint_name) {
        for (const auto& state : joint_states.states) {
            if (state.name == joint_name) {
                return state.motor_position;
            }
        }
        throw std::runtime_error("Joint name " + joint_name + " not found");
    }

    double interpolatePosition(double start_pos, double target_pos, double t) {
        return start_pos + t * (target_pos - start_pos);
    }

public:
    void executeArmJointServoControl(const std::vector<double>& target_positions, const int control_group) {
        std::this_thread::sleep_for(std::chrono::seconds(1));  // 等待1秒

        // 获取当前关节状态
        JointStates current_joint_states;
        if (robot_.GetJointStates(current_joint_states) != GDKRes::kSuccess) {
            std::cout << "获取关节状态失败" << std::endl;
            return;
        }

        // 获取起始位置
        std::vector<std::string> arm_joint_names = {
            "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3",
            "idx24_arm_l_joint4", "idx25_arm_l_joint5", "idx26_arm_l_joint6",
            "idx27_arm_l_joint7",  // 左臂7个关节
            "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3",
            "idx64_arm_r_joint4", "idx65_arm_r_joint5", "idx66_arm_r_joint6",
            "idx67_arm_r_joint7"   // 右臂7个关节
        };
        std::vector<double> start_positions;
        for (const auto& joint_name : arm_joint_names) {
            try {
                double pos = getJointPositionByName(current_joint_states, joint_name);
                start_positions.push_back(pos);
                std::cout << "关节 " << joint_name << " 当前位置: " << pos << " 弧度" << std::endl;
            } catch (const std::exception& e) {
                std::cout << "错误: " << e.what() << std::endl;
                return;
            }
        }

        // 计算步数
        int n_steps = static_cast<int>(DURATION * RATE_HZ);
        std::cout << "总步数: " << n_steps << ", 持续时间: " << DURATION << " 秒" << std::endl;

        // 执行轨迹
        double dt = 1.0 / RATE_HZ;
        auto start_time = std::chrono::steady_clock::now();

        for (int i = 0; i < n_steps; i++) {
            double t = static_cast<double>(i) / (n_steps - 1);

            // 计算当前目标位置（线性插值）
            std::vector<double> current_positions;
            for (size_t j = 0; j < 14; j++) {
                double interp_pos = interpolatePosition(
                    start_positions[j], target_positions[j], t
                );
                current_positions.push_back(interp_pos);
            }

            // 使用普通模式（enable_low_latency=false，默认值）
            auto res = robot_.MoveArmJointServo(
                current_positions, CONTROL_PERIOD, control_group
            );
            // 如需使用低延时模式，可传入 enable_low_latency=true：
            // 注意：低延时模式下，该接口绕过运控算法保护，直接驱动电机。
            // 潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。
            // auto res = robot_.MoveArmJointServo(
            //     current_positions, CONTROL_PERIOD, control_group, true
            // );
            if (res != GDKRes::kSuccess) {
                std::cout << "控制命令发送失败，步数: " << i << std::endl;
                return;
            }

            // 控制发送频率
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            auto expected_time = std::chrono::milliseconds(static_cast<int>((i + 1) * dt * 1000));
            auto sleep_time = expected_time - elapsed;
            if (sleep_time.count() > 0) {
                std::this_thread::sleep_for(sleep_time);
            }
        }

        std::cout << "手臂关节位置伺服控制完成" << std::endl;

        // 保持最终位置
        std::cout << "进入最终位置保持（Ctrl+C 结束）..." << std::endl;
        try {
            while (true) {
                // 使用普通模式（enable_low_latency=false，默认值）
                auto res = robot_.MoveArmJointServo(
                    target_positions, CONTROL_PERIOD, control_group
                );
                // 如需使用低延时模式，可传入 enable_low_latency=true：
                // 注意：低延时模式下，该接口绕过运控算法保护，直接驱动电机。
                // 潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。
                // auto res = robot_.MoveArmJointServo(
                //     target_positions, CONTROL_PERIOD, control_group, true
                // );
                if (res != GDKRes::kSuccess) {
                    std::cout << "保持位置失败" << std::endl;
                    break;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
            }
        } catch (const std::exception& e) {
            std::cout << "已中断保持: " << e.what() << std::endl;
        }
    }
};

int main() {
    // 初始化GDK系统
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    Robot robot;
    std::cout << "Robot init" << std::endl;

    // 获取当前关节状态
    JointStates current_joint_states;
    if (robot.GetJointStates(current_joint_states) != GDKRes::kSuccess) {
        std::cout << "获取关节状态失败" << std::endl;
        GDKRelease();
        return -1;
    }

    // 定义目标位置（示例：手臂关节）
    std::vector<double> target_positions = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // 左臂7个关节
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0   // 右臂7个关节
    };

    std::cout << "\n目标角度:" << std::endl;
    std::vector<std::string> arm_joint_names = {
        "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3",
        "idx24_arm_l_joint4", "idx25_arm_l_joint5", "idx26_arm_l_joint6",
        "idx27_arm_l_joint7",  // 左臂7个关节
        "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3",
        "idx64_arm_r_joint4", "idx65_arm_r_joint5", "idx66_arm_r_joint6",
        "idx67_arm_r_joint7"   // 右臂7个关节
    };
    for (size_t i = 0; i < arm_joint_names.size(); i++) {
        std::cout << "  " << arm_joint_names[i] << ": " << target_positions[i] << " 弧度" << std::endl;
    }

    // 执行手臂关节位置伺服控制
    // 该接口支持 control_mode=1（关节位控）/ 3（关节阻抗），需先确保已切换；
    // 默认通常已是位控模式，若想用关节阻抗请先 robot.SetControlMode(3)。
    ArmJointServoController controller;
    controller.executeArmJointServoControl(target_positions, 2);

    // 释放GDK系统资源
    if (GDKRelease() != GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 17. `EndEffectorPoseControl()`

- **功能**：末端执行器位姿控制接口（非阻塞式）
- **支持的控制模式**：`1`（关节位控）、`2`（笛卡尔阻抗）、`3`（关节阻抗），使用关节阻抗模式或笛卡尔阻抗模式前，请先通过`SetControlMode()`切换。
- **不同模式下的行为**：
- `1` / `3`（位置控制 / 关节阻抗控制）：目标位姿经运控服务全身逆解后，在关节空间执行；关节阻抗控制模式下，底层会按`SetJointImpedance()`配置的阻抗参数柔顺跟踪。
- `2`（笛卡尔阻抗控制）：目标位姿直接在笛卡尔空间执行，底层会按`SetCartesianImpedance()`配置的阻抗参数柔顺跟踪。
- 位置控制或关节阻抗控制时，末端位姿的参考坐标系是 end\_link（末端法兰中心位置）；笛卡尔阻抗控制时，末端位姿的参考坐标系为 tcp（末端执行器中心位置）。未特别配置时，tcp 与 end\_link 默认重合。
- **注意**：
- 该接口要求发送端以50hz频率发布控制命令，不允许阶越信号；该接口无碰撞检测，注意使用时环境。该接口的控制器不提供规划能力，只适合用于末端小范围的移动，请勿在奇异点位置使用该接口。
- 支持末端位姿与夹爪关节同步控制：填入 `end_effector_joint_names` / `end_effector_joint_positions` 时同步驱动对应关节，留空则只控制末端位姿。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `end_pose` | `const EndEffectorPose&` | 末端执行器位姿控制对象 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`

#### EndEffectorPose对象详细说明

**EndEffectorPose结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `life_time` | `double` | 控制生命周期 | 秒 |
| `group` | `int32_t` | 控制组，0: 未知，4: 左臂，8: 右臂，12: 双臂 | 无单位 |
| `left_end_effector_pose` | `Pose` | 左末端执行器位姿（base\_link坐标系下） | 位姿 |
| `right_end_effector_pose` | `Pose` | 右末端执行器位姿（base\_link坐标系下） | 位姿 |
| `end_effector_joint_names` | `std::vector<std::string>` | 同步控制的末端关节名称列表（如夹爪关节），与 `end_effector_joint_positions` 一一对应，为空时不控制末端关节 | 无单位 |
| `end_effector_joint_positions` | `std::vector<double>` | 末端关节目标位置列表，须与 `end_effector_joint_names` 长度相同 | 弧度 |

```
struct EndEffectorPose {
  double life_time{0.0};
  int32_t group{0};
  Pose left_end_effector_pose{};
  Pose right_end_effector_pose{};
  // 同步控制末端关节（如夹爪），为空时不控制末端关节
  std::vector<std::string> end_effector_joint_names{};
  std::vector<double> end_effector_joint_positions{};
};
```

**Pose结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `position` | `Position` | 位置信息 | 米 |
| `orientation` | `Orientation` | 方向信息 | 四元数 |

**Position结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `x` | `double` | X坐标 | 米 |
| `y` | `double` | Y坐标 | 米 |
| `z` | `double` | Z坐标 | 米 |

**Orientation结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `x` | `double` | 四元数X分量 | 无单位 |
| `y` | `double` | 四元数Y分量 | 无单位 |
| `z` | `double` | 四元数Z分量 | 无单位 |
| `w` | `double` | 四元数W分量 | 无单位 |

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace agibot::gdk;

// 默认以法兰中心 end_link 为末端参考坐标系。
// 若需以工具坐标系 tool0（TCP）为参考，请改用 left_tool0_body / right_tool0_body，
// 并配合 SwitchEndPoseControlFrame()，详见本节末尾“工具坐标系（tool0）用例”。
const std::string LEFT_NAME = "arm_l_end_link";
const std::string RIGHT_NAME = "arm_r_end_link";
const int32_t CONTROL_GROUP = 12;  // kBothArms

const Pose TARGET_LEFT = {
    {0.516, 0.433, 1.081},  // position
    {0.382, -0.146, 0.663, 0.626}  // orientation
};

const Pose TARGET_RIGHT = {
    {0.579, -0.306, 1.158},  // position
    {0.320, 0.655, 0.651, 0.206}  // orientation
};

const double MAX_STEP_CM = 0.1;  // 最大步长（厘米）
const double LIFETIME = 0.02;    // 生命周期（秒）
const double RATE_HZ = 50.0;     // 发送频率（Hz）
const bool HOLD_FINAL = true;    // 是否保持最终位姿

class EndEffectorController {
private:
    Robot robot_;

    void slerp(const double q0[4], const double q1[4], double t, double result[4]) {
        double dot = q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3];

        // 如果点积为负，取反q1以确保最短路径（取反一次，后续所有分支都用 q1c）
        double q1c[4] = {q1[0], q1[1], q1[2], q1[3]};
        if (dot < 0.0) {
            dot = -dot;
            for (int i = 0; i < 4; i++) {
                q1c[i] = -q1[i];
            }
        }

        // 限制点积范围
        dot = std::clamp(dot, -1.0, 1.0);

        if (dot > 0.9995) {
            // 线性插值
            for (int i = 0; i < 4; i++) {
                result[i] = q0[i] + t * (q1c[i] - q0[i]);
            }
            double norm = 0.0;
            for (int i = 0; i < 4; i++) {
                norm += result[i] * result[i];
            }
            norm = std::sqrt(norm);
            if (norm > 0.0) {
                for (int i = 0; i < 4; i++) {
                    result[i] /= norm;
                }
            }
        } else {
            // 球面线性插值
            double theta_0 = std::acos(dot);
            double sin_theta_0 = std::sin(theta_0);
            double theta = theta_0 * t;
            double sin_theta = std::sin(theta);
            double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
            double s1 = sin_theta / sin_theta_0;

            for (int i = 0; i < 4; i++) {
                result[i] = s0 * q0[i] + s1 * q1c[i];
            }
        }
    }

    double distanceBetweenPoints(const Vector3& p1, const Vector3& p2) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double dz = p2.z - p1.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    int calculateNSteps(const Vector3& start, const Vector3& goal, double max_step_cm) {
        double dist_cm = distanceBetweenPoints(start, goal) * 100.0;
        return std::max(static_cast<int>(std::ceil(dist_cm / max_step_cm)), 1);
    }

    std::vector<Pose> planTrajectory(const Pose& start, const Pose& goal, int n_steps) {
        std::vector<Pose> trajectory;

        for (int i = 0; i < n_steps; i++) {
            double t = static_cast<double>(i) / (n_steps - 1);
            Pose pose;

            // 位置线性插值
            pose.position.x = start.position.x + t * (goal.position.x - start.position.x);
            pose.position.y = start.position.y + t * (goal.position.y - start.position.y);
            pose.position.z = start.position.z + t * (goal.position.z - start.position.z);

            // 四元数SLERP插值
            double q0[4] = {start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w};
            double q1[4] = {goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w};
            double result[4];
            slerp(q0, q1, t, result);

            pose.orientation.x = result[0];
            pose.orientation.y = result[1];
            pose.orientation.z = result[2];
            pose.orientation.w = result[3];

            trajectory.push_back(pose);
        }

        return trajectory;
    }

    Pose findPoseByName(const std::vector<std::string>& frame_names,
                       const std::vector<Pose>& frame_poses,
                       const std::string& target_name) {
        for (size_t i = 0; i < frame_names.size(); i++) {
            if (frame_names[i] == target_name) {
                return frame_poses[i];
            }
        }
        throw std::runtime_error("Frame name " + target_name + " not found");
    }

public:
    void executeEndPoseControl() {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 等待1秒

        // EndEffectorPoseControl 支持 control_mode=1 / 2 / 3：
        //   1（关节位控）/ 3（关节阻抗）—— 经全身逆解后在关节空间执行
        //   2（笛卡尔阻抗）              —— 直接在笛卡尔空间执行，按 SetCartesianImpedance 配置柔顺跟踪
        // 默认通常已是位控模式（1）；想用关节阻抗 / 笛卡尔阻抗请先 robot_.SetControlMode(3 / 2)。

        // 获取当前状态
        MotionControlStatus status;
        GDKRes result = robot_.GetMotionControlStatus(status);
        if (result != GDKRes::kSuccess) {
            std::cout << "获取运动控制状态失败" << std::endl;
            return;
        }

        // 获取起始位姿
        Pose start_left_pose = findPoseByName(status.frame_names, status.frame_poses, LEFT_NAME);
        Pose start_right_pose = findPoseByName(status.frame_names, status.frame_poses, RIGHT_NAME);

        // 计算步数
        int n_left = calculateNSteps(start_left_pose.position, TARGET_LEFT.position, MAX_STEP_CM);
        int n_right = calculateNSteps(start_right_pose.position, TARGET_RIGHT.position, MAX_STEP_CM);
        int n_steps = std::max(n_left, n_right);

        std::cout << "左臂步数: " << n_left << ", 右臂步数: " << n_right << ", 总步数: " << n_steps << std::endl;

        // 规划轨迹
        std::vector<Pose> traj_left = planTrajectory(start_left_pose, TARGET_LEFT, n_steps);
        std::vector<Pose> traj_right = planTrajectory(start_right_pose, TARGET_RIGHT, n_steps);

        // 执行轨迹
        double dt = 1.0 / RATE_HZ;
        for (int i = 0; i < n_steps; i++) {
            EndEffectorPose end_pose;
            end_pose.life_time = LIFETIME;
            end_pose.group = CONTROL_GROUP;
            end_pose.left_end_effector_pose = traj_left[i];
            end_pose.right_end_effector_pose = traj_right[i];

            result = robot_.EndEffectorPoseControl(end_pose);
            if (result != GDKRes::kSuccess) {
                std::cout << "控制命令发送失败，步数: " << i << std::endl;
                return;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
        }

        // 保持最终位姿（对应Python版本）
        if (HOLD_FINAL) {
            std::cout << "进入末端位姿保持（Ctrl+C 结束）..." << std::endl;
            try {
                while (true) {
                    EndEffectorPose end_pose;
                    end_pose.life_time = LIFETIME;
                    end_pose.group = CONTROL_GROUP;
                    end_pose.left_end_effector_pose = traj_left.back();
                    end_pose.right_end_effector_pose = traj_right.back();

                    result = robot_.EndEffectorPoseControl(end_pose);
                    if (result != GDKRes::kSuccess) {
                        std::cout << "保持位姿失败" << std::endl;
                        break;
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
                }
            } catch (const std::exception& e) {
                std::cout << "已中断保持: " << e.what() << std::endl;
            }
        }
    }
};

int main() {
    EndEffectorController controller;
    controller.executeEndPoseControl();
    return 0;
}
```

- **工具坐标系（`tool0`）用例**：上面的示例以 `arm_l_end_link` / `arm_r_end_link`（法兰中心 end\_link）为末端参考坐标系。下面是以\*\*工具坐标系\*\* `left_tool0_body` / `right_tool0_body`（即 TCP）为参考点进行\*\*笛卡尔阻抗\*\*末端位姿控制的\*\*完整示例\*\*。与原示例的区别集中在 4 处（已在代码中用 `区别 N` 标注），轨迹规划与下发逻辑完全一致：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace agibot::gdk;

// 区别 1：末端参考坐标系改用工具坐标系 tool0
//         （原示例为 arm_l_end_link / arm_r_end_link）
const std::string LEFT_NAME = "left_tool0_body";
const std::string RIGHT_NAME = "right_tool0_body";
const int32_t CONTROL_GROUP = 12;  // kBothArms

// 目标位姿——请按 tool0（TCP）坐标系下的期望位姿设置
const Pose TARGET_LEFT = {
    {0.516, 0.433, 1.081},  // position
    {0.382, -0.146, 0.663, 0.626}  // orientation
};

const Pose TARGET_RIGHT = {
    {0.579, -0.306, 1.158},  // position
    {0.320, 0.655, 0.651, 0.206}  // orientation
};

const double MAX_STEP_CM = 0.1;  // 最大步长（厘米）
const double LIFETIME = 0.02;    // 生命周期（秒）
const double RATE_HZ = 50.0;     // 发送频率（Hz）
const bool HOLD_FINAL = true;    // 是否保持最终位姿

class EndEffectorController {
private:
    Robot robot_;

    void slerp(const double q0[4], const double q1[4], double t, double result[4]) {
        double dot = q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3];

        // 如果点积为负，取反q1以确保最短路径（取反一次，后续所有分支都用 q1c）
        double q1c[4] = {q1[0], q1[1], q1[2], q1[3]};
        if (dot < 0.0) {
            dot = -dot;
            for (int i = 0; i < 4; i++) {
                q1c[i] = -q1[i];
            }
        }

        // 限制点积范围
        dot = std::clamp(dot, -1.0, 1.0);

        if (dot > 0.9995) {
            // 线性插值
            for (int i = 0; i < 4; i++) {
                result[i] = q0[i] + t * (q1c[i] - q0[i]);
            }
            double norm = 0.0;
            for (int i = 0; i < 4; i++) {
                norm += result[i] * result[i];
            }
            norm = std::sqrt(norm);
            if (norm > 0.0) {
                for (int i = 0; i < 4; i++) {
                    result[i] /= norm;
                }
            }
        } else {
            // 球面线性插值
            double theta_0 = std::acos(dot);
            double sin_theta_0 = std::sin(theta_0);
            double theta = theta_0 * t;
            double sin_theta = std::sin(theta);
            double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
            double s1 = sin_theta / sin_theta_0;

            for (int i = 0; i < 4; i++) {
                result[i] = s0 * q0[i] + s1 * q1c[i];
            }
        }
    }

    double distanceBetweenPoints(const Vector3& p1, const Vector3& p2) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double dz = p2.z - p1.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    int calculateNSteps(const Vector3& start, const Vector3& goal, double max_step_cm) {
        double dist_cm = distanceBetweenPoints(start, goal) * 100.0;
        return std::max(static_cast<int>(std::ceil(dist_cm / max_step_cm)), 1);
    }

    std::vector<Pose> planTrajectory(const Pose& start, const Pose& goal, int n_steps) {
        std::vector<Pose> trajectory;

        for (int i = 0; i < n_steps; i++) {
            double t = static_cast<double>(i) / (n_steps - 1);
            Pose pose;

            // 位置线性插值
            pose.position.x = start.position.x + t * (goal.position.x - start.position.x);
            pose.position.y = start.position.y + t * (goal.position.y - start.position.y);
            pose.position.z = start.position.z + t * (goal.position.z - start.position.z);

            // 四元数SLERP插值
            double q0[4] = {start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w};
            double q1[4] = {goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w};
            double result[4];
            slerp(q0, q1, t, result);

            pose.orientation.x = result[0];
            pose.orientation.y = result[1];
            pose.orientation.z = result[2];
            pose.orientation.w = result[3];

            trajectory.push_back(pose);
        }

        return trajectory;
    }

    Pose findPoseByName(const std::vector<std::string>& frame_names,
                       const std::vector<Pose>& frame_poses,
                       const std::string& target_name) {
        for (size_t i = 0; i < frame_names.size(); i++) {
            if (frame_names[i] == target_name) {
                return frame_poses[i];
            }
        }
        throw std::runtime_error("Frame name " + target_name + " not found");
    }

public:
    // 进入工具坐标系（tool0）笛卡尔阻抗控制前的初始化
    void setupToolFrame() {
        // 区别 2（可选）：配置 TCP 相对法兰 end_link 的变换，
        // 例如沿 Z 轴前移 0.1m；不配置则 tool0 与 end_link 默认重合。
        // 注意：为确保安全，禁止在笛卡尔阻抗模式下设置 TCP；
        //       必须先设置 TCP 再 SetControlMode(2)，否则 TCP 设置会失败。
        DualArmTcpTransform tcp;
        tcp.left_arm.position = {0.0, 0.0, 0.1};
        tcp.left_arm.euler_zyx = {0.0, 0.0, 0.0};
        tcp.right_arm.position = {0.0, 0.0, 0.1};
        tcp.right_arm.euler_zyx = {0.0, 0.0, 0.0};
        robot_.SetTcpTransform(tcp);

        // 区别 3：进入笛卡尔阻抗控制模式（tool_frame 仅在该模式下生效）
        robot_.SetControlMode(2);

        // 区别 4：将目标坐标系切换到 tool0（left_tool0_body / right_tool0_body）。
        // 切换后 TCP 设置才会在末端位姿控制中生效；日志会打印当前生效的坐标系
        robot_.SwitchEndPoseControlFrame();
    }

    void executeEndPoseControl() {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 等待1秒

        // 获取当前状态
        MotionControlStatus status;
        GDKRes result = robot_.GetMotionControlStatus(status);
        if (result != GDKRes::kSuccess) {
            std::cout << "获取运动控制状态失败" << std::endl;
            return;
        }

        // 获取起始位姿（LEFT_NAME / RIGHT_NAME 已指向 tool0）
        Pose start_left_pose = findPoseByName(status.frame_names, status.frame_poses, LEFT_NAME);
        Pose start_right_pose = findPoseByName(status.frame_names, status.frame_poses, RIGHT_NAME);

        // 计算步数
        int n_left = calculateNSteps(start_left_pose.position, TARGET_LEFT.position, MAX_STEP_CM);
        int n_right = calculateNSteps(start_right_pose.position, TARGET_RIGHT.position, MAX_STEP_CM);
        int n_steps = std::max(n_left, n_right);

        std::cout << "左臂步数: " << n_left << ", 右臂步数: " << n_right << ", 总步数: " << n_steps << std::endl;

        // 规划轨迹
        std::vector<Pose> traj_left = planTrajectory(start_left_pose, TARGET_LEFT, n_steps);
        std::vector<Pose> traj_right = planTrajectory(start_right_pose, TARGET_RIGHT, n_steps);

        // 执行轨迹
        double dt = 1.0 / RATE_HZ;
        for (int i = 0; i < n_steps; i++) {
            EndEffectorPose end_pose;
            end_pose.life_time = LIFETIME;
            end_pose.group = CONTROL_GROUP;
            end_pose.left_end_effector_pose = traj_left[i];
            end_pose.right_end_effector_pose = traj_right[i];

            result = robot_.EndEffectorPoseControl(end_pose);
            if (result != GDKRes::kSuccess) {
                std::cout << "控制命令发送失败，步数: " << i << std::endl;
                return;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
        }

        // 保持最终位姿
        if (HOLD_FINAL) {
            std::cout << "进入末端位姿保持（Ctrl+C 结束）..." << std::endl;
            try {
                while (true) {
                    EndEffectorPose end_pose;
                    end_pose.life_time = LIFETIME;
                    end_pose.group = CONTROL_GROUP;
                    end_pose.left_end_effector_pose = traj_left.back();
                    end_pose.right_end_effector_pose = traj_right.back();

                    result = robot_.EndEffectorPoseControl(end_pose);
                    if (result != GDKRes::kSuccess) {
                        std::cout << "保持位姿失败" << std::endl;
                        break;
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
                }
            } catch (const std::exception& e) {
                std::cout << "已中断保持: " << e.what() << std::endl;
            }
        }
    }
};

int main() {
    EndEffectorController controller;
    controller.setupToolFrame();      // 切换到 tool0 工具坐标系（区别 2/3/4）
    controller.executeEndPoseControl();
    return 0;
}
```

- **同时控制夹爪用例**：在末端位姿控制的同时，通过 `end_effector_joint_names` / `end_effector_joint_positions` 字段同步驱动夹爪关节。夹爪关节名从 `GetEndState` 动态获取（与 `JointServoControl` 同时控制末端示例保持一致），末端行程范围同样取 `-0.785`（下限）↔ `0.0`（上限）。与纯位姿示例相比，新增部分已用注释标出。

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace agibot::gdk;

const std::string LEFT_NAME  = "arm_l_end_link";
const std::string RIGHT_NAME = "arm_r_end_link";
const int32_t CONTROL_GROUP  = 12;  // kBothArms

// 新增：末端行程范围（与 JointServoControl 同时控制末端示例保持一致）
const double EE_LOW  = -0.785;  // 末端行程下限
const double EE_HIGH =  0.0;    // 末端行程上限

const Pose TARGET_LEFT = {
    {0.516, 0.433, 1.081},
    {0.382, -0.146, 0.663, 0.626}
};
const Pose TARGET_RIGHT = {
    {0.579, -0.306, 1.158},
    {0.320, 0.655, 0.651, 0.206}
};

const double MAX_STEP_CM = 0.1;
const double LIFETIME    = 0.02;
const double RATE_HZ     = 50.0;
const bool   HOLD_FINAL  = true;

// 新增：从 GetEndState 中取出指定侧末端关节名与当前位置（同 JointServoControl 示例）
void getEENamesAndPositions(const DualEndState& end_state,
                            bool use_left,
                            std::vector<std::string>& names,
                            std::vector<double>& positions) {
    const EndState& state = use_left ? end_state.left_end_state : end_state.right_end_state;
    names = state.names;
    positions.clear();
    for (const auto& motor : state.end_states) {
        positions.push_back(motor.position);
    }
}

class EndEffectorWithGripperController {
private:
    Robot robot_;

    void slerp(const double q0[4], const double q1[4], double t, double result[4]) {
        double dot = q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3];
        double q1c[4] = {q1[0], q1[1], q1[2], q1[3]};
        if (dot < 0.0) { dot = -dot; for (int i = 0; i < 4; i++) q1c[i] = -q1[i]; }
        dot = std::clamp(dot, -1.0, 1.0);
        if (dot > 0.9995) {
            for (int i = 0; i < 4; i++) result[i] = q0[i] + t * (q1c[i] - q0[i]);
            double norm = 0.0;
            for (int i = 0; i < 4; i++) norm += result[i] * result[i];
            norm = std::sqrt(norm);
            if (norm > 0.0) for (int i = 0; i < 4; i++) result[i] /= norm;
        } else {
            double theta_0 = std::acos(dot);
            double sin_theta_0 = std::sin(theta_0);
            double theta = theta_0 * t;
            double s0 = std::cos(theta) - dot * std::sin(theta) / sin_theta_0;
            double s1 = std::sin(theta) / sin_theta_0;
            for (int i = 0; i < 4; i++) result[i] = s0 * q0[i] + s1 * q1c[i];
        }
    }

    int calculateNSteps(const Position& start, const Position& goal, double max_step_cm) {
        double dx = goal.x - start.x, dy = goal.y - start.y, dz = goal.z - start.z;
        double dist_cm = std::sqrt(dx*dx + dy*dy + dz*dz) * 100.0;
        return std::max(static_cast<int>(std::ceil(dist_cm / max_step_cm)), 1);
    }

    std::vector<Pose> planTrajectory(const Pose& start, const Pose& goal, int n_steps) {
        std::vector<Pose> traj;
        for (int i = 0; i < n_steps; i++) {
            double t = static_cast<double>(i) / (n_steps - 1);
            Pose p;
            p.position.x = start.position.x + t * (goal.position.x - start.position.x);
            p.position.y = start.position.y + t * (goal.position.y - start.position.y);
            p.position.z = start.position.z + t * (goal.position.z - start.position.z);
            double q0[4] = {start.orientation.x, start.orientation.y,
                            start.orientation.z, start.orientation.w};
            double q1[4] = {goal.orientation.x,  goal.orientation.y,
                            goal.orientation.z,  goal.orientation.w};
            double r[4]; slerp(q0, q1, t, r);
            p.orientation = {r[0], r[1], r[2], r[3]};
            traj.push_back(p);
        }
        return traj;
    }

    Pose findPoseByName(const std::vector<std::string>& names,
                        const std::vector<Pose>& poses,
                        const std::string& target) {
        for (size_t i = 0; i < names.size(); i++)
            if (names[i] == target) return poses[i];
        throw std::runtime_error("Frame not found: " + target);
    }

public:
    void execute() {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        MotionControlStatus status;
        if (robot_.GetMotionControlStatus(status) != GDKRes::kSuccess) {
            std::cout << "获取运动控制状态失败" << std::endl;
            return;
        }

        // 新增：获取末端关节名与当前位置（左侧 + 右侧）
        DualEndState end_state;
        if (robot_.GetEndState(end_state) != GDKRes::kSuccess) {
            std::cout << "获取末端状态失败" << std::endl;
            return;
        }
        std::vector<std::string> ee_names_l, ee_names_r;
        std::vector<double> ee_pos_l, ee_pos_r;
        getEENamesAndPositions(end_state, true,  ee_names_l, ee_pos_l);
        getEENamesAndPositions(end_state, false, ee_names_r, ee_pos_r);

        // 合并左右夹爪名称与起始位置
        std::vector<std::string> gripper_names = ee_names_l;
        gripper_names.insert(gripper_names.end(), ee_names_r.begin(), ee_names_r.end());
        std::vector<double> gripper_start = ee_pos_l;
        gripper_start.insert(gripper_start.end(), ee_pos_r.begin(), ee_pos_r.end());
        // 目标：夹爪运动至 EE_LOW
        std::vector<double> gripper_target(gripper_names.size(), EE_LOW);

        Pose start_left  = findPoseByName(status.frame_names, status.frame_poses, LEFT_NAME);
        Pose start_right = findPoseByName(status.frame_names, status.frame_poses, RIGHT_NAME);

        int n_steps = std::max(
            calculateNSteps(start_left.position,  TARGET_LEFT.position,  MAX_STEP_CM),
            calculateNSteps(start_right.position, TARGET_RIGHT.position, MAX_STEP_CM));
        n_steps = std::max(n_steps, static_cast<int>(RATE_HZ * 1.0));  // 至少 1s

        std::vector<Pose> traj_left  = planTrajectory(start_left,  TARGET_LEFT,  n_steps);
        std::vector<Pose> traj_right = planTrajectory(start_right, TARGET_RIGHT, n_steps);

        double dt = 1.0 / RATE_HZ;
        for (int i = 0; i < n_steps; i++) {
            double t = static_cast<double>(i) / (n_steps - 1);

            EndEffectorPose end_pose;
            end_pose.life_time               = LIFETIME;
            end_pose.group                   = CONTROL_GROUP;
            end_pose.left_end_effector_pose  = traj_left[i];
            end_pose.right_end_effector_pose = traj_right[i];

            // 新增：同步驱动夹爪，随轨迹进度从当前位置线性插值到 EE_LOW
            end_pose.end_effector_joint_names = gripper_names;
            end_pose.end_effector_joint_positions.resize(gripper_names.size());
            for (size_t j = 0; j < gripper_names.size(); j++) {
                end_pose.end_effector_joint_positions[j] =
                    gripper_start[j] + t * (gripper_target[j] - gripper_start[j]);
            }

            if (robot_.EndEffectorPoseControl(end_pose) != GDKRes::kSuccess) {
                std::cout << "控制命令发送失败，步数: " << i << std::endl;
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
        }

        if (HOLD_FINAL) {
            std::cout << "进入末端+夹爪保持（Ctrl+C 结束）..." << std::endl;
            try {
                while (true) {
                    EndEffectorPose end_pose;
                    end_pose.life_time               = LIFETIME;
                    end_pose.group                   = CONTROL_GROUP;
                    end_pose.left_end_effector_pose  = traj_left.back();
                    end_pose.right_end_effector_pose = traj_right.back();
                    // 新增：保持夹爪目标位置（EE_LOW）
                    end_pose.end_effector_joint_names      = gripper_names;
                    end_pose.end_effector_joint_positions  = gripper_target;
                    robot_.EndEffectorPoseControl(end_pose);
                    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
                }
            } catch (...) {}
        }
    }
};

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    EndEffectorWithGripperController controller;
    controller.execute();
    GDKRelease();
    return 0;
}
```

#### 18. `GetEndPoseControlFrame()`

- **功能**：获取 `EndEffectorPoseControl()` 在\*\*笛卡尔阻抗控制（`control_mode=2`）\*\*下使用的目标参考坐标系。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `frame_names` | `std::vector<std::string>&` | 输出参数，当前目标参考坐标系名称，按左臂 / 右臂顺序返回 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回 `GDKRes::kSuccess`
- **说明**：
- 默认返回 `arm_l_end_link` / `arm_r_end_link`（末端法兰中心）。
- 调用 [`SwitchEndPoseControlFrame()`](#19-switchendposecontrolframe) 切换到工具坐标系后，返回 `left_tool0_body` / `right_tool0_body`。
- 该接口仅查询当前配置的目标坐标系，不会改变控制状态。
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <vector>

using namespace agibot::gdk;

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    Robot robot;
    std::vector<std::string> frame_names;
    if (robot.GetEndPoseControlFrame(frame_names) == GDKRes::kSuccess) {
        std::cout << "left frame: " << frame_names[0]
                  << ", right frame: " << frame_names[1] << std::endl;
    }

    GDKRelease();
    return 0;
}
```

#### 19. `SwitchEndPoseControlFrame()`

- **功能**：切换 `EndEffectorPoseControl()` 在\*\*笛卡尔阻抗控制（`control_mode=2`）\*\*下使用的目标参考坐标系。每次调用在以下两组坐标系之间切换，并在日志中打印切换后当前使用的坐标系：
- `arm_l_end_link` / `arm_r_end_link`（默认，末端法兰中心）
- `left_tool0_body` / `right_tool0_body`（末端执行器/工具坐标系）
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码。成功时返回 `GDKRes::kSuccess`
- **说明**：
- 仅影响笛卡尔阻抗控制（`control_mode=2`）下的目标坐标系；关节位控（`1`）/ 关节阻抗（`3`）模式不受影响。
- 切换后会在日志中打印当前生效的坐标系，也可以调用 [`GetEndPoseControlFrame()`](#18-getendposecontrolframe) 查询当前坐标系。
- **与 TCP 设置配合使用**：通过 [`SetTcpTransform()`](#35-settcptransform) 设置的 TCP 变换作用于 `tool0`（`left_tool0_body` / `right_tool0_body`）坐标系。设置 TCP 后必须调用本接口切换到工具坐标系，TCP 变换才会在笛卡尔阻抗末端位姿控制中生效。
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>

using namespace agibot::gdk;

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    Robot robot;

    // 切换笛卡尔阻抗末端位姿控制的目标坐标系，
    // 在 arm_l_end_link/arm_r_end_link 与 left_tool0_body/right_tool0_body 间切换
    if (robot.SwitchEndPoseControlFrame() == GDKRes::kSuccess) {
        std::cout << "已切换末端位姿控制坐标系（详见日志）" << std::endl;
    } else {
        std::cerr << "切换末端位姿控制坐标系失败" << std::endl;
    }

    GDKRelease();
    return 0;
}
```

#### 20. `MoveEEPos()`

- **功能**：控制末端执行器位置(非阻塞式)
- **注意**：该接口不可与伺服接口同时使用，若有与伺服接口同时使用的需求请通过伺服接口控制末端执行器
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `joint_states` | `const JointStates&` | 关节状态控制参数 |

#### JointStates对象详细说明（用于MoveEEPos）

**JointStates结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `nums` | `size_t` | 关节数量，应等于 `states.size()` | 无单位 |
| `group` | `std::string` | 控制组，必须为 "left\_tool"、"right\_tool" 或 "dual\_tool" | 字符串 |
| `states` | `std::vector<JointState>` | 关节状态列表（必需） | 关节状态列表 |

**各末端类型的关节位置取值范围**：

- **注意**：关节取值范围与版本绑定，使用时注意当前版本的取值范围，当前版本取值范围以文档为主
- **omnipicker**：`position` 取值范围为 `[-0.785,0]`，其中 `-0.785` 表示打开，`0` 表示关闭
- **dahuan**：`position` 取值范围为 `[0,0.025]`，其中 `0` 表示打开，`0.025` 表示关闭
- **ctek90d**：`position` 取值范围为 `[-0.91, 0]`，其中 `-0.91` 表示打开，`0` 表示关闭
- **o10\_t2**：各关节位置取值范围如下（单位：弧度）

**左手关节限位值**：

| 关节索引 | 关节名称 | 最小值 | 最大值 |
| --- | --- | --- | --- |
| 0 | `idx31_hand_l_thumb_roll_joint` | -1.1213740444063567 | 0.029670597283903602 |
| 1 | `idx32_hand_l_thumb_abad_joint` | -0.04537856055185257 | 1.642354826126664 |
| 2 | `idx33_hand_l_thumb_mcp_joint` | -0.8415977653116657 | 0.0 |
| 3 | `idx36_hand_l_index_abad_joint` | 0.0 | 0.16406094968746698 |
| 4 | `idx37_hand_l_index_pip_joint` | 0.0 | 1.4835298641951802 |
| 5 | `idx39_hand_l_middle_pip_joint` | 0.0 | 1.4835298641951802 |
| 6 | `idx41_hand_l_ring_abad_joint` | -0.16929693744344995 | 0.0 |
| 7 | `idx42_hand_l_ring_pip_joint` | 0.0 | 1.4835298641951802 |
| 8 | `idx44_hand_l_pinky_abad_joint` | -0.1850049007113989 | 0.0 |
| 9 | `idx45_hand_l_pinky_pip_joint` | 0.0 | 1.4835298641951802 |

**右手关节限位值**：

| 关节索引 | 关节名称 | 最小值 | 最大值 |
| --- | --- | --- | --- |
| 0 | `idx71_hand_r_thumb_roll_joint` | -0.029670597283903602 | 1.1213740444063567 |
| 1 | `idx72_hand_r_thumb_abad_joint` | -1.642354826126664 | 0.04537856055185257 |
| 2 | `idx73_hand_r_thumb_mcp_joint` | 0.0 | 0.8415977653116657 |
| 3 | `idx76_hand_r_index_abad_joint` | -0.16406094968746698 | 0.0 |
| 4 | `idx77_hand_r_index_pip_joint` | 0.0 | 1.4835298641951802 |
| 5 | `idx79_hand_r_middle_pip_joint` | 0.0 | 1.4835298641951802 |
| 6 | `idx81_hand_r_ring_abad_joint` | 0.0 | 0.16929693744344995 |
| 7 | `idx82_hand_r_ring_pip_joint` | 0.0 | 1.4835298641951802 |
| 8 | `idx84_hand_r_pinky_abad_joint` | 0.0 | 0.1850049007113989 |
| 9 | `idx85_hand_r_pinky_pip_joint` | 0.0 | 1.4835298641951802 |

**典型状态值**：
- **左手开**：`[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`
- **左手握**：`[-0.2, 1.45, -0.75, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 1.0]`
- **右手开**：`[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`
- **右手握**：`[0.2, -1.45, 0.75, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 1.0]`

- **o12\_t2**：各关节位置取值范围如下（单位：弧度）

**左手关节限位值**：

| 关节索引 | 关节名称 | 最小值 | 最大值 |
| --- | --- | --- | --- |
| 0 | `idx31_hand_l_thumb_roll_joint` | -0.9425 | 0.0 |
| 1 | `idx32_hand_l_thumb_abad_joint` | 0.0 | 1.3875 |
| 2 | `idx33_hand_l_thumb_mcp_joint` | -0.8273 | 0.0 |
| 3 | `idx34_hand_l_thumb_pip_joint` | -1.2915 | 0.0 |
| 4 | `idx36_hand_l_index_abad_joint` | -0.2618 | 0.2618 |
| 5 | `idx37_hand_l_index_mcp_joint` | 0.0 | 1.3526 |
| 6 | `idx38_hand_l_index_pip_joint` | 0.0 | 1.5307 |
| 7 | `idx40_hand_l_middle_abad_joint` | -0.2618 | 0.2618 |
| 8 | `idx41_hand_l_middle_mcp_joint` | 0.0 | 1.3579 |
| 9 | `idx42_hand_l_middle_pip_joint` | 0.0 | 1.8151 |
| 10 | `idx44_hand_l_ring_mcp_joint` | 0.0 | 1.5359 |
| 11 | `idx47_hand_l_pinky_mcp_joint` | 0.0 | 1.5359 |

**右手关节限位值**：

| 关节索引 | 关节名称 | 最小值 | 最大值 |
| --- | --- | --- | --- |
| 0 | `idx71_hand_r_thumb_roll_joint` | 0.0 | 0.9425 |
| 1 | `idx72_hand_r_thumb_abad_joint` | -1.3875 | 0.0 |
| 2 | `idx73_hand_r_thumb_mcp_joint` | -0.8273 | 0.0 |
| 3 | `idx74_hand_r_thumb_pip_joint` | -1.2915 | 0.0 |
| 4 | `idx76_hand_r_index_abad_joint` | -0.2618 | 0.2618 |
| 5 | `idx77_hand_r_index_mcp_joint` | 0.0 | 1.3526 |
| 6 | `idx78_hand_r_index_pip_joint` | 0.0 | 1.5307 |
| 7 | `idx80_hand_r_middle_abad_joint` | -0.2618 | 0.2618 |
| 8 | `idx81_hand_r_middle_mcp_joint` | 0.0 | 1.3579 |
| 9 | `idx82_hand_r_middle_pip_joint` | 0.0 | 1.8151 |
| 10 | `idx84_hand_r_ring_mcp_joint` | 0.0 | 1.5359 |
| 11 | `idx87_hand_r_pinky_mcp_joint` | 0.0 | 1.5359 |

**典型状态值**：
- **左手开**：`[-0.53, 0.42, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`
- **左手握**：`[-0.77, 0.5, -0.4, -0.36, -0.12, 0.69, 0.46, 0.0, 0.72, 0.5, 0.63, 0.63]`
- **右手开**：`[0.53, -0.42, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`
- **右手握**：`[0.77, -0.5, -0.4, -0.36, 0.12, 0.69, 0.46, 0.0, 0.72, 0.5, 0.63, 0.63]`

**参数说明**：
- `group` 必须为 "left\_tool"（左末端执行器）、"right\_tool"（右末端执行器）或 "dual\_tool"（双末端执行器），其他值将返回 `ErrorCode::kInvalidInput`
- 当 `group` 为 "left\_tool" 或 "right\_tool" 时，`states.size()` 必须与末端执行器对应的关节数量要求完全匹配
- 当 `group` 为 "dual\_tool" 时，`states.size()` 必须为左右末端执行器对应的关节数量之和（前半部分用于左末端执行器，后半部分用于右末端执行器）
- `nums` 必须等于 `states` 向量的大小
- `states` 不能为空，`states.size()` 必须大于0，否则返回 `ErrorCode::kInvalidInput`

**JointState结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `position` | `double` | 关节位置（必需） | 开合度 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **错误处理**：
- 如果 `joint_states.states.size() <= 0`，返回 `ErrorCode::kInvalidInput`
- 如果 `joint_states.states.size()` 与末端执行器对应的关节数量要求不匹配（对于 "dual\_tool"，必须是末端执行器关节数量之和），返回 `ErrorCode::kInvalidInput`
- 如果 `group` 不是 "left\_tool"、"right\_tool" 或 "dual\_tool"，返回 `ErrorCode::kInvalidInput`
- 如果关节位置值超出对应末端类型的取值范围，返回 `ErrorCode::kInvalidInput`
- 其他错误情况返回相应的错误码
- **示例**：

**示例1：控制左夹爪（omnipicker类型，需要1个关节）**

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Robot robot;
    std::cout << "Robot init" << std::endl;

    // 控制左夹爪（omnipicker类型，需要1个关节）
    agibot::gdk::JointStates left_joint_states;
    left_joint_states.group = "left_tool";
    left_joint_states.states.resize(1);
    left_joint_states.states[0].position = 0;  // 取值范围 [-0.785, 0]
    left_joint_states.nums = left_joint_states.states.size();

    if (robot.MoveEEPos(left_joint_states) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to move left end effector position" << std::endl;
    } else {
        std::cout << "左末端执行器位置控制成功" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

**示例2：控制右夹爪（dahuan类型，需要1个关节）**

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Robot robot;
    std::cout << "Robot init" << std::endl;

    // 控制右夹爪（dahuan类型，需要1个关节）
    agibot::gdk::JointStates right_joint_states;
    right_joint_states.group = "right_tool";
    right_joint_states.states.resize(1);
    right_joint_states.states[0].position = 0;  // 取值范围 [0, 0.025]
    right_joint_states.nums = right_joint_states.states.size();

    if (robot.MoveEEPos(right_joint_states) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to move right end effector position" << std::endl;
    } else {
        std::cout << "右末端执行器位置控制成功" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

**示例3：控制左夹爪（ctek90d类型，需要1个关节）**

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Robot robot;
    std::cout << "Robot init" << std::endl;

    // 控制左夹爪（ctek90d类型，需要1个关节）
    agibot::gdk::JointStates left_joint_states;
    left_joint_states.group = "left_tool";
    left_joint_states.states.resize(1);
    left_joint_states.states[0].position = 0;  // 取值范围 [-0.91, 0]
    left_joint_states.nums = left_joint_states.states.size();

    if (robot.MoveEEPos(left_joint_states) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to move left end effector position" << std::endl;
    } else {
        std::cout << "左末端执行器位置控制成功" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

**示例4：控制左末端执行器（o10\_t2类型，需要10个关节）**

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Robot robot;
    std::cout << "Robot init" << std::endl;

    // 控制左末端执行器（o10_t2类型，需要10个关节）
    agibot::gdk::JointStates left_o10_states;
    left_o10_states.group = "left_tool";
    left_o10_states.states.resize(10);
    for (size_t i = 0; i < 10; ++i) {
        left_o10_states.states[i].position = 0.0;
    }
    left_o10_states.nums = left_o10_states.states.size();

    if (robot.MoveEEPos(left_o10_states) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to move left end effector (o10_t2)" << std::endl;
    } else {
        std::cout << "左末端执行器（o10_t2）位置控制成功" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

**示例5：控制右末端执行器（o12\_t2类型，需要12个关节）**

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Robot robot;
    std::cout << "Robot init" << std::endl;

    // 控制右末端执行器（o12_t2类型，需要12个关节）
    agibot::gdk::JointStates right_o12_states;
    right_o12_states.group = "right_tool";
    right_o12_states.states.resize(12);
    for (size_t i = 0; i < 12; ++i) {
        right_o12_states.states[i].position = 0.0;
    }
    right_o12_states.nums = right_o12_states.states.size();

    if (robot.MoveEEPos(right_o12_states) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to move right end effector (o12_t2)" << std::endl;
    } else {
        std::cout << "右末端执行器（o12_t2）位置控制成功" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

**示例6：控制双末端执行器（dual\_tool，需要2倍关节数量）**

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Robot robot;
    std::cout << "Robot init" << std::endl;

    // 控制双末端执行器（dual_tool，需要2倍关节数量）
    // 例如：使用 omnipicker 类型，需要 2 个关节（左1个 + 右1个）
    agibot::gdk::JointStates dual_states;
    dual_states.group = "dual_tool";
    dual_states.states.resize(2);
    // 前半部分用于左末端执行器
    dual_states.states[0].position = 0.0;
    // 后半部分用于右末端执行器
    dual_states.states[1].position = 0.0;
    dual_states.nums = dual_states.states.size();

    if (robot.MoveEEPos(dual_states) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to move dual end effector" << std::endl;
    } else {
        std::cout << "双末端执行器位置控制成功" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 21. `MoveEndEffectorJoint()`

- **功能**：控制末端执行器关节位置
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `positions` | `const std::vector<double>&` | 目标位置，顺序需与实际关节执行器顺序相同 |
| `velocities` | `const std::vector<double>&` | 与 `positions` 等长，执行速度 |
| `group` | `const std::string&` | `left_tool`、`right_tool` 或 `dual_tool` |

- **返回值**：`GDKRes`。成功为 `GDKRes::kSuccess`。
- **说明**：`dual_tool` 时 `positions` 为先左后右拼接。控制消息发布至 `/wbc/joint_control`。

**示例**：

```
#include "gdk/gdk.h"
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

int main()
{
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::Robot robot;
    std::cout << "Robot init" << std::endl;

    std::vector<double> positions = {0.0};
    std::vector<double> velocities = {0.0};
    if (robot.MoveEndEffectorJoint(positions, velocities, "left_tool")
        != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "MoveEndEffectorJoint 失败" << std::endl;
    } else {
        std::cout << "末端执行器关节控制成功" << std::endl;
    }

    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 22. `SetControlMode()`

- **功能**：设置机器人运动控制模式（同步阻塞式）。关于各模式的含义与工作流，参见[关于运动控制模式的说明](#关于运动控制模式的说明)。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `control_mode` | `uint8_t` | 控制模式整数值，仅允许 ½/3，见下方说明 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回 `GDKRes::kSuccess`；输入越界返回 `GDKRes::kInvalidInput`；请求超时或被底层拒绝时返回 `GDKRes::kInvalidOutput`。
- **说明**：`MotionControlMode` 中的其他字段（`target`、`safe_mode`、`priority`）以及内部使用的 `input_source` 均使用默认值（`0` / `0` / `10` 以及固定的 `6`），调用方无需也无法在此接口中覆盖。

#### control\_mode 取值说明

`control_mode` 仅允许以下整数：

| 数值 | 描述 |
| --- | --- |
| 1 | 关节位控（PD/轨迹） |
| 2 | 笛卡尔阻抗 / 笛卡尔力控 |
| 3 | 关节阻抗 / 关节力控 |

- **示例**：切换到笛卡尔力控模式

```
#include "gdk/gdk.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <thread>
#include <vector>

using namespace agibot::gdk;

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    Robot robot;

    // 1) 先切换到笛卡尔力控模式
    if (robot.SetControlMode(2) != GDKRes::kSuccess) {
        std::cerr << "切换到笛卡尔力控模式失败" << std::endl;
        GDKRelease();
        return -1;
    }
    std::cout << "已进入笛卡尔力控模式，可通过EndEffectorPoseControl下发目标位姿" << std::endl;

    // 2) 设置笛卡尔阻抗参数
    // 双臂共 12 维，仅 kp 生效；前 6 维为左臂，后 6 维为右臂
    // 每臂顺序为 x, y, z, roll, pitch, yaw
    // x/y/z 推荐范围: 600~6000 N/m；roll/pitch/yaw 推荐范围: 30~300 N·m/rad
    std::vector<double> single_arm_kp{1000.0, 1000.0, 1000.0, 50.0, 50.0, 50.0};
    std::vector<Impedance> impedances;
    impedances.reserve(12);
    for (int arm = 0; arm < 2; ++arm) {
        for (double kp : single_arm_kp) {
            impedances.push_back(Impedance{kp});
        }
    }
    if (robot.SetCartesianImpedance(impedances, 2) != GDKRes::kSuccess) {
        std::cerr << "设置笛卡尔阻抗失败" << std::endl;
        GDKRelease();
        return -1;
    }

    GDKRelease();
    return 0;
}
```

#### 23. `GetCartesianImpedance()`

- **功能**：获取笛卡尔阻抗参数
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `impedances` | `std::vector<Impedance>&` | 输出参数，每个笛卡尔轴的阻抗参数 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回 `GDKRes::kSuccess`

**Impedance结构体**：

| 成员名 | 类型 | 描述 |
| --- | --- | --- |
| `kp` | `double` | 刚度系数 |
| `ki` | `double` | 积分系数，当前实现暂未启用 |
| `kv` | `double` | 阻尼系数（速度反馈），当前实现暂未启用 |

- **示例**：

```
#include "gdk/gdk.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>

using namespace agibot::gdk;

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    Robot robot;

    std::vector<Impedance> impedances;
    if (robot.GetCartesianImpedance(impedances) == GDKRes::kSuccess) {
        for (size_t i = 0; i < impedances.size(); ++i) {
            std::cout << "axis " << i
                      << " kp=" << impedances[i].kp
                      << " ki=" << impedances[i].ki
                      << " kv=" << impedances[i].kv << std::endl;
        }
    }

    GDKRelease();
    return 0;
}
```

#### 24. `SetCartesianImpedance()`

- **功能**：设置笛卡尔阻抗参数（必须先切换到 `control_mode=2`（笛卡尔阻抗）后再调用）。当前实现仅支持设置 `kp`（刚度系数），`ki` / `kv` 暂未启用。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `impedances` | `const std::vector<Impedance>&` | 每个笛卡尔轴的阻抗参数；当前实现仅读取并应用 `kp`，`ki` / `kv` 字段会被忽略。`control_group=0/1` 时长度为 6（单臂），`control_group=2` 时长度为 12（双臂，前 6 个为左臂，后 6 个为右臂） |
| `control_group` | `const int` | 控制组，0: 左臂，1: 右臂，2: 双臂 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回 `GDKRes::kSuccess`
- **kp 取值说明**：

单臂的 6 维 `kp` 按顺序对应笛卡尔末端的 **`x, y, z, roll, pitch, yaw`**（前三维为平动、后三维为转动）。建议取值范围：

| 维度 | 物理意义 | 下限 | 上限 | 单位 |
| --- | --- | --- | --- | --- |
| `x` / `y` / `z` | 平动刚度 | 600 | 6000 | N/m |
| `roll` / `pitch` / `yaw` | 转动刚度 | 30 | 300 | N·m/rad |

即单臂 `kp` 下限为 `(600, 600, 600, 30, 30, 30)`，上限为 `(6000, 6000, 6000, 300, 300, 300)`。`control_group=2`（双臂）时长度为 12，前 6 维为左臂、后 6 维为右臂，每一臂分别遵循上述顺序与范围。

- `kp` 越大刚度越高，对位姿误差的恢复力越强，但抗冲击/柔顺性下降；
- `kp` 越小越柔顺，适合接触类任务（贴合、装配等），但对扰动跟踪偏差更大；
- 超出推荐范围可能引发系统振荡或柔顺度严重不足，请按工况在区间内调试。
- **示例**：

```
#include "gdk/gdk.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>

using namespace agibot::gdk;

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    Robot robot;

    // 先切换到笛卡尔阻抗控制模式（control_mode=2），再设置对应参数
    robot.SetControlMode(2);

    // 双臂共 12 维，仅 kp 生效；前 6 维为左臂，后 6 维为右臂
    // 每臂顺序为 x, y, z, roll, pitch, yaw
    // x/y/z 推荐范围: 600~6000 N/m；roll/pitch/yaw 推荐范围: 30~300 N·m/rad
    std::vector<double> single_arm_kp{1000.0, 1000.0, 1000.0, 50.0, 50.0, 50.0};
    std::vector<Impedance> impedances;
    impedances.reserve(12);
    for (int arm = 0; arm < 2; ++arm) {
        for (double kp : single_arm_kp) {
            impedances.push_back(Impedance{kp});
        }
    }
    robot.SetCartesianImpedance(impedances, 2);

    GDKRelease();
    return 0;
}
```

#### 25. `GetCollisionDetectionConfig()`

- **功能**：获取碰撞检测配置
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `config` | `CollisionDetectionConfig&` | 输出参数，碰撞检测配置 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回 `GDKRes::kSuccess`

**CollisionDetectionConfig结构体**：

| 成员名 | 类型 | 描述 |
| --- | --- | --- |
| `is_enabled` | `bool` | 是否启用碰撞检测 |
| `sensitivity` | `uint8_t` | 灵敏度等级，1-3，数值越大越灵敏 |
| `checkout_timeout_ms` | `int32_t` | 恢复时间（毫秒） |

- **碰撞检测行为说明**：

开启碰撞检测后，控制器会持续监测各关节的外力扰动（参见 `GetMotionControlStatus()` 返回的 `arm_disturbance_force`）。
- **检测到碰撞**：当外力超过当前灵敏度对应的阈值时，**自动切换到位置控制模式（`control_mode=1`）以保障安全**，停止当前柔顺/规划运动；
- **自动恢复**：在 `checkout_timeout_ms` 指定的恢复时间窗内若再未触发新的碰撞，**自动切换回触发前的原控制模式**（如笛卡尔阻抗 / 关节阻抗）；
- **灵敏度选择**：`sensitivity` 越大越容易触发，建议根据负载/末端工具实际工况调试，默认从 `2` 开始；
- **触发后期间**：被切换到位控期间，下发笛卡尔阻抗 / 关节阻抗模式专属的接口会因模式不匹配而被拒绝，应等待自动恢复或显式重新 `SetControlMode()` 后再继续。

- **示例**：

```
#include "gdk/gdk.h"
#include <chrono>
#include <thread>
#include <iostream>

using namespace agibot::gdk;

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    Robot robot;

    CollisionDetectionConfig config;
    if (robot.GetCollisionDetectionConfig(config) == GDKRes::kSuccess) {
        std::cout << "enabled=" << config.is_enabled
                  << " sensitivity=" << static_cast<int>(config.sensitivity)
                  << std::endl;
    }

    GDKRelease();
    return 0;
}
```

#### 26. `SetCollisionDetectionConfig()`

- **功能**：设置碰撞检测配置
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `config` | `const CollisionDetectionConfig&` | 碰撞检测配置 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回 `GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <chrono>
#include <thread>
#include <iostream>

using namespace agibot::gdk;

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    Robot robot;

    CollisionDetectionConfig config;
    config.is_enabled = true;
    config.sensitivity = 2;
    config.checkout_timeout_ms = 500;
    robot.SetCollisionDetectionConfig(config);

    GDKRelease();
    return 0;
}
```

#### 27. `GetJointImpedance()`

- **功能**：获取关节阻抗参数
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `impedances` | `std::vector<Impedance>&` | 输出参数，每个关节的阻抗参数 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回 `GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>

using namespace agibot::gdk;

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    Robot robot;

    std::vector<Impedance> impedances;
    if (robot.GetJointImpedance(impedances) == GDKRes::kSuccess) {
        for (size_t i = 0; i < impedances.size(); ++i) {
            std::cout << "joint " << i
                      << " kp=" << impedances[i].kp
                      << " ki=" << impedances[i].ki
                      << " kv=" << impedances[i].kv << std::endl;
        }
    }

    GDKRelease();
    return 0;
}
```

#### 28. `SetJointImpedance()`

- **功能**：设置关节阻抗参数（必须先切换到 `control_mode=3`（关节阻抗）后再调用）。当前实现仅支持设置 `kp`（刚度系数），`ki` / `kv` 暂未启用。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `impedances` | `const std::vector<Impedance>&` | 每个关节的阻抗参数；当前实现仅读取并应用 `kp`，`ki` / `kv` 字段会被忽略。`control_group=0/1` 时长度为单臂关节数，`control_group=2` 时长度为双臂关节数之和（前 N 个为左臂，后 N 个为右臂） |
| `control_group` | `const int` | 控制组，0: 左臂，1: 右臂，2: 双臂 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回 `GDKRes::kSuccess`
- **kp 取值说明**：

单臂的 7 维 `kp` 按顺序对应该臂从基座到末端的 **关节 1 ~ 关节 7**。各关节建议取值范围：

| 关节序号 | 下限 | 上限 | 单位 |
| --- | --- | --- | --- |
| 关节 1 / 2 / 3（大臂） | 50 | 500 | N·m/rad |
| 关节 4（肘） | 60 | 600 | N·m/rad |
| 关节 5 / 6 / 7（小臂/腕） | 30 | 300 | N·m/rad |

即单臂 `kp` 下限为 `(50, 50, 50, 60, 30, 30, 30)`，上限为 `(500, 500, 500, 600, 300, 300, 300)`。`control_group=2`（双臂）时长度为 14，前 7 维为左臂、后 7 维为右臂，每一臂分别遵循上述顺序与范围。

- `kp` 越大刚度越高，关节对位置误差恢复力强，但抗冲击/柔顺性下降；
- `kp` 越小越柔顺，更适合接触类或拖拽类任务，但跟踪误差更大；
- 超出推荐范围可能引发振荡或柔顺度严重不足，请按工况在区间内调试。
- **示例**：

```
#include "gdk/gdk.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>

using namespace agibot::gdk;

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    Robot robot;

    // 先切换到关节阻抗控制模式（control_mode=3），再设置对应参数
    robot.SetControlMode(3);

    std::vector<Impedance> impedances(7, Impedance{100.0});  // 单臂 7 个关节，仅 kp 生效
    robot.SetJointImpedance(impedances, 0);  // 左臂

    GDKRelease();
    return 0;
}
```

#### 29. `GetPayload()`

- **功能**：获取双臂末端负载信息
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `info` | `PayloadInfo&` | 输出参数，负载信息 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回 `GDKRes::kSuccess`

**PayloadInfo结构体**：

| 成员名 | 类型 | 描述 |
| --- | --- | --- |
| `left_arm` | `InertiaInfo` | 左臂末端负载信息 |
| `right_arm` | `InertiaInfo` | 右臂末端负载信息 |

**InertiaInfo结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `mass` | `double` | 负载质量 | kg |
| `center_of_mass` | `std::array<double, 3>` | 负载重心位置 | m |
| `inertia` | `std::array<double, 9>` | 负载惯性矩阵（3x3 按行展开） | kg·m² |

- **示例**：

```
#include "gdk/gdk.h"
#include <chrono>
#include <thread>
#include <iostream>

using namespace agibot::gdk;

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    Robot robot;

    PayloadInfo info;
    if (robot.GetPayload(info) == GDKRes::kSuccess) {
        std::cout << "left mass=" << info.left_arm.mass
                  << ", right mass=" << info.right_arm.mass << std::endl;
    }

    GDKRelease();
    return 0;
}
```

#### 30. `SetPayload()`

- **功能**：设置双臂末端负载信息。负载参数会进入控制器的动力学补偿，是力控（笛卡尔阻抗 / 关节阻抗）下力反馈与重力补偿正确性的前提。
- **参考坐标系**：负载的质心（`center_of_mass`）与惯性张量（`inertia`）均以末端 `end_link` 坐标系（末端法兰中心位置）为参考，填写时请确保数值在该坐标系下表达。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `info` | `const PayloadInfo&` | 负载信息 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回 `GDKRes::kSuccess`
- **使用建议**：
- **仅在末端确实存在负载时设置**：空载状态不应保留旧的非零负载参数，否则控制器会按虚构的额外重力进行补偿，导致力控柔顺跟踪偏移、力反馈失真。空载或负载移除时，请显式调用本接口将质量/质心/惯量重置为零。
- **持久负载（长期挂载）**：当末端常驻同一负载（如固定工具、长期挂载的物体）时，调用 `SetPayload()` 后应立即调用一次 [`TorqueSensorCalibration()`](#31-torquesensorcalibration)，补偿当前装配下传感器的零点偏置。标定一次后即可持续使用。
- **短期负载（搬运类场景）**：搬重物前先 `SetPayload()` 配置当前负载；放下后再次 `SetPayload()` 重置为零负载。这种短周期切换通常不需要每次都重新做传感器零点标定，但负载参数必须随实际抓持状态同步更新。
- **静止下设置**：建议在机械臂静止时调用，避免与运动控制指令产生竞争。
- **最大负载**：G02机器标称为5kg，请不要超过最大负载，否则会导致机械臂运动异常。
- **示例**：

```
#include "gdk/gdk.h"
#include <chrono>
#include <thread>
#include <iostream>

using namespace agibot::gdk;

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    Robot robot;

    PayloadInfo info;
    info.left_arm.mass = 0.5;
    info.left_arm.center_of_mass = {0.0, 0.0, 0.05};
    info.left_arm.inertia = {0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001};
    info.right_arm.mass = 0.5;
    info.right_arm.center_of_mass = {0.0, 0.0, 0.05};
    info.right_arm.inertia = {0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001};
    robot.SetPayload(info);

    GDKRelease();
    return 0;
}
```

#### 31. `TorqueSensorCalibration()`

- **功能**：标定关节力矩传感器零点。调用前请正确设置负载，并确保机械臂处于静止状态，不要在规划、拖拽、其他标定过程中重复发起。标定期间请勿下发运动类指令，标定完成后会自动恢复正常控制模式。
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码。成功时返回 `GDKRes::kSuccess`
- **什么情况下需要在线标定力矩传感器**：
- 在无外力接触，负载设置正确的情况下，motion control status消息中的arm\_disturbance\_force和frame\_wrenchs的值较大，说明传感器存在漂移，需要标定；
- 机械臂发生碰撞之后，需要标定；
- 长时间使用或者温度变化较大，建议标定；
- **使用建议**
- 建议不带负载，在初始化之后进行标定；
- 标定时，机器人必须处于静止状态；
- 不要在规划、拖拽、其他标定过程中重复发起；
- 标定期间不要再下发运动类指令；
- 如需带负载标定，请正确设置负载，否则标定误差过大或者超出检测阈值会导致标定失败。
- **持久化说明**：

本接口的标定结果会被\*\*固化\*\*到机器人本体，机器人重启后依然有效，无需每次开机重做。
与之相对，下列配置项\*\*不会\*\*被持久化，每次机器人重启后都会回到默认值，需要在使用前重新设置：

| 配置项 | 相关接口 |
| --- | --- |
| 笛卡尔阻抗参数 | [`SetCartesianImpedance()`](#24-setcartesianimpedance) |
| 关节阻抗参数 | [`SetJointImpedance()`](#28-setjointimpedance) |
| 末端负载 | [`SetPayload()`](#30-setpayload) |
| 碰撞检测开关与阈值 | [`SetCollisionDetectionConfig()`](#26-setcollisiondetectionconfig) |
| 运动控制模式 | [`SetControlMode()`](#22-setcontrolmode) |

实际工程中建议在应用启动流程内统一一次性下发上述配置，避免遗漏。

- **示例**：

```
#include "gdk/gdk.h"
#include <chrono>
#include <thread>
#include <iostream>

using namespace agibot::gdk;

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    Robot robot;

    if (robot.TorqueSensorCalibration() == GDKRes::kSuccess) {
        std::cout << "torque sensor calibration succeeded" << std::endl;
    } else {
        std::cerr << "torque sensor calibration failed" << std::endl;
    }

    GDKRelease();
    return 0;
}
```

#### 32. `ClearMotionControlError()`

- **功能**：清除运动控制错误。运动控制触发安全保护机制后，会进入错误态并停止响应运动指令，此时需调用本接口清除错误后才能继续控制。
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码。成功时返回 `GDKRes::kSuccess`
- **支持的控制模式**：所有控制模式（`1` 关节位控、`2` 笛卡尔阻抗、`3` 关节阻抗）均可调用。
- **使用建议**：
- 通过 [`GetMotionControlStatus()`](#6-getmotioncontrolstatus) 的 `error_code`（非 0）/ `error_msg` 判断是否处于错误态；
- 清错前请先排除引发错误的根因（如移除外部干涉、确认负载配置正确、机械臂处于静止状态），否则清错后可能立即再次触发；
- 清错成功后建议再次读取 `GetMotionControlStatus()` 确认 `error_code` 已恢复为 0，再继续下发运动指令。
- **示例**：

```
#include "gdk/gdk.h"
#include <chrono>
#include <thread>
#include <iostream>

using namespace agibot::gdk;

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    Robot robot;

    // 检查运动控制状态，发现错误时清除
    MotionControlStatus status;
    if (robot.GetMotionControlStatus(status) == GDKRes::kSuccess &&
        status.error_code != 0) {
        std::cout << "检测到运动控制错误: code=" << static_cast<int>(status.error_code)
                  << ", msg=" << status.error_msg << std::endl;
        // 清除前请先排除引发错误的根因
        if (robot.ClearMotionControlError() == GDKRes::kSuccess) {
            std::cout << "运动控制错误已清除" << std::endl;
        } else {
            std::cerr << "清除运动控制错误失败" << std::endl;
        }
    } else {
        std::cout << "运动控制状态正常，无需清错" << std::endl;
    }

    GDKRelease();
    return 0;
}
```

#### 33. `ClearHalError()`

- **功能**：清除硬件（HAL）层故障。机器人头部、腰部、手臂等电机在报错后会进入故障态并锁停，本接口向各电机下发一次清错（故障复位）请求，使其恢复可用。
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码。请求成功下发时返回 `GDKRes::kSuccess`
- **与 [`ClearMotionControlError()`](#32-clearmotioncontrolerror) 的区别**：本接口作用于 **HAL 硬件层**（电机驱动层的故障复位）；`ClearMotionControlError()` 作用于\*\*运动控制层\*\*（力控安全保护错误）。两者面向不同层级，按需调用。
- **使用建议**：
- 清错前请先排除引发故障的根因（如机械干涉、过载、急停未释放等），否则清错后可能立即再次触发；
- 若清错后电机仍无法运动，请检查硬件状态与故障日志。
- **示例**：

```
#include "gdk/gdk.h"
#include <chrono>
#include <thread>
#include <iostream>

using namespace agibot::gdk;

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    Robot robot;

    // 清除硬件层故障（清错前请先排除引发故障的根因）
    if (robot.ClearHalError() == GDKRes::kSuccess) {
        std::cout << "HAL清错请求已下发" << std::endl;
    } else {
        std::cerr << "HAL清错请求下发失败" << std::endl;
    }
    // 异步下发，留出时间让底层完成复位
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    GDKRelease();
    return 0;
}
```

#### 34. `GetTcpTransform()`

- **功能**：获取双臂末端 TCP（工具中心点）变换。TCP 变换描述工具坐标系相对末端 `end_link`（法兰中心）的位姿。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `transform` | `DualArmTcpTransform&` | 输出参数，双臂 TCP 变换 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回 `GDKRes::kSuccess`

**DualArmTcpTransform结构体**：

| 成员名 | 类型 | 描述 |
| --- | --- | --- |
| `left_arm` | `TcpTransform` | 左臂末端 TCP 变换 |
| `right_arm` | `TcpTransform` | 右臂末端 TCP 变换 |

**TcpTransform结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `position` | `std::vector<double>`（长度 3） | TCP 相对 flange 的平移 `[x, y, z]` | m |
| `euler_zyx` | `std::vector<double>`（长度 3） | 欧拉角 `[roll, pitch, yaw]`，旋转 `R = Rz(yaw) * Ry(pitch) * Rx(roll)`（绕固定轴 X→Y→Z 外旋） | rad |

- **示例**：

```
#include "gdk/gdk.h"
#include <chrono>
#include <thread>
#include <iostream>

using namespace agibot::gdk;

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    Robot robot;

    DualArmTcpTransform transform;
    if (robot.GetTcpTransform(transform) == GDKRes::kSuccess) {
        std::cout << "left tcp position: "
                  << transform.left_arm.position[0] << ", "
                  << transform.left_arm.position[1] << ", "
                  << transform.left_arm.position[2] << std::endl;
    }

    GDKRelease();
    return 0;
}
```

#### 35. `SetTcpTransform()`

- **功能**：设置双臂末端 TCP（工具中心点）变换。设置后，末端位姿类指令（如末端执行器位姿控制、笛卡尔规划）会以 TCP 为参考点，便于在工具尖端坐标系下进行控制。
- **参考坐标系**：TCP 变换以末端 `end_link` 坐标系（法兰中心位置）为参考，`position` 为平移、`euler_zyx` 为姿态欧拉角。
- **注意（安全限制 / 调用顺序）**：为确保安全，**禁止在笛卡尔阻抗模式（`control_mode=2`）下设置TCP 坐标**。先调用 `SetTcpTransform()` 设置 TCP，再通过 `SetControlMode(2)` 进入笛卡尔阻抗模式；如果在笛卡尔阻抗模式下调用本接口，TCP 设置会\*\*失败\*\*。设置完成并进入笛卡尔阻抗模式后，还需调用 [`SwitchEndPoseControlFrame()`](#19-switchendposecontrolframe) 将末端位姿控制的目标 frame 切换到工具坐标系（`left_tool0_body` / `right_tool0_body`），TCP 变换才会在 `EndEffectorPoseControl()` 中生效。推荐顺序：`SetTcpTransform()` → `SetControlMode(2)` → `SwitchEndPoseControlFrame()`。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `transform` | `const DualArmTcpTransform&` | 双臂 TCP 变换 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回 `GDKRes::kSuccess`
- **使用建议**：
- **静止下设置**：建议在机械臂静止时调用，避免与运动控制指令产生竞争。
- **更换工具后更新**：每次更换末端工具或工具安装位置变化后，应重新设置 TCP 变换，确保末端位姿控制以正确的工具尖端为参考。
- **恢复默认**：若需以法兰中心为参考点，可将 `position` 与 `euler_zyx` 全部置零。
- **先设置 TCP，再进入笛卡尔模式**：为确保安全，禁止在笛卡尔阻抗模式（`control_mode=2`）下设置/切换 TCP，必须先 `SetTcpTransform()` 再 `SetControlMode(2)`，否则 TCP 设置会\*\*失败\*\*。
- **需切换到工具坐标系后生效**：`SetTcpTransform()` 配置的是 `tool0`（`left_tool0_body` / `right_tool0_body`）相对法兰的变换。`EndEffectorPoseControl()` 在笛卡尔阻抗控制下默认仍以 `arm_l_end_link` / `arm_r_end_link`（法兰中心）为目标 frame，**TCP 设置不会自动生效**。需调用 [`SwitchEndPoseControlFrame()`](#19-switchendposecontrolframe) 切换到工具坐标系（`left_tool0_body` / `right_tool0_body`）后，新的 TCP 变换才会作用于末端位姿控制。
- **示例**：

```
#include "gdk/gdk.h"
#include <chrono>
#include <thread>
#include <iostream>

using namespace agibot::gdk;

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    Robot robot;

    DualArmTcpTransform transform;
    // TCP 沿 flange Z 轴前移 0.1m，无姿态偏转
    transform.left_arm.position = {0.0, 0.0, 0.1};
    transform.left_arm.euler_zyx = {0.0, 0.0, 0.0};
    transform.right_arm.position = {0.0, 0.0, 0.1};
    transform.right_arm.euler_zyx = {0.0, 0.0, 0.0};
    robot.SetTcpTransform(transform);

    GDKRelease();
    return 0;
}
```

## 使用注意事项

1. **GDK初始化**：使用Robot功能前必须先调用`agibot::gdk::GDKInit()`初始化GDK系统
2. **GDK释放**：程序结束前必须调用`agibot::gdk::GDKRelease()`释放GDK系统资源
3. **关节名称**：使用前请确认关节名称的正确性，可通过`GetJointStates()`获取
4. **位置范围**：关节位置应在安全范围内，避免超出机械限制
5. **速度限制**：设置合理的关节速度，避免过快运动造成危险
6. **生命周期**：合理设置请求生命周期，避免命令过期
7. **错误处理**：及时检查返回值，处理可能的错误情况
8. **控制组选择**：末端执行器控制时选择合适的控制组（左臂/右臂/双臂）
9. **位姿设置**：设置末端执行器位姿时注意坐标系的正确性
10. **状态监控**：定期检查错误码，确保机器人安全运行
11. **温度监控**：注意电机温度，避免过热损坏
12. **急停状态**：检查急停状态，确保机器人可以正常控制
13. **数据同步**：基于时间戳进行多传感器数据同步
14. **错误处理**：始终检查GDKRes返回值，确保操作成功

## 应用场景

- **机器人控制**：实现机器人的全面运动控制
- **状态监控**：实时监控机器人各部件状态
- **动作执行**：执行复杂的机器人动作序列
- **路径规划**：结合状态信息进行路径规划
- **安全检测**：监控机器人异常状态，确保安全运行
- **末端控制**：精确控制末端执行器的位姿
- **双臂协调**：实现双臂机器人的协调控制
- **关节控制**：实现高精度的关节位置控制
- **故障诊断**：通过错误码进行故障诊断和处理
- **数据融合**：结合多传感器数据进行机器人控制



---

<!-- source: contents_reference_C++_slam -->

# SLAM

# GDK SLAM 接口文档（C++）

## 概述

SLAM（Simultaneous Localization and Mapping，同时定位与地图构建）模块为G02机器人提供了实时建图和定位功能。通过C++接口，开发者可以方便地实现机器人的环境感知、地图构建、位置估计等功能，适用于自主导航、环境建模、定位服务等多种场景。

## 接口说明

### Slam 类

该类封装了SLAM系统的主要功能接口。

#### 1. `GetSlamState()`

- **功能**：获取SLAM系统当前状态
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `state` | `uint32_t&` | 输出参数，SLAM状态码 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`state`参数包含SLAM状态信息 （1：开始建图 2：停止建图 0：取消建图）
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
using namespace agibot::gdk;

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    Slam slam;
    std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待1秒以确保DDS连接建立
    uint32_t state;
    GDKRes result = slam.GetSlamState(state);

    if (result == GDKRes::kSuccess) {
        std::cout << "SLAM状态: " << state << std::endl;
    } else {
        std::cout << "获取SLAM状态失败" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 2. `StartMapping()`

- **功能**：开始建图
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
using namespace agibot::gdk;

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    Slam slam;

    // 开始建图
    GDKRes result = slam.StartMapping();

    if (result == GDKRes::kSuccess) {
        std::cout << "开始建图成功" << std::endl;

        // 检查建图状态
        sleep(2);
        uint32_t state;
        result = slam.GetSlamState(state);
        if (result == GDKRes::kSuccess) {
            std::cout << "建图状态: " << state << std::endl;
        }
    } else {
        std::cout << "开始建图失败" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 3. `StopMapping()`

- **功能**：停止且保存建图
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
using namespace agibot::gdk;

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    Slam slam;

    // 停止建图
    GDKRes result = slam.StopMapping();

    if (result == GDKRes::kSuccess) {
        std::cout << "停止建图成功" << std::endl;

        // 检查建图状态
        sleep(2);
        uint32_t state;
        result = slam.GetSlamState(state);
        if (result == GDKRes::kSuccess) {
            std::cout << "建图状态: " << state << std::endl;
        }
    } else {
        std::cout << "停止建图失败" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 4. `CancelMapping()`

- **功能**：取消建图
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
using namespace agibot::gdk;

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    Slam slam;

    // 取消建图
    GDKRes result = slam.CancelMapping();

    if (result == GDKRes::kSuccess) {
        std::cout << "取消建图成功" << std::endl;

        // 检查建图状态
        sleep(2);
        uint32_t state;
        result = slam.GetSlamState(state);
        if (result == GDKRes::kSuccess) {
            std::cout << "建图状态: " << state << std::endl;
        }
    } else {
        std::cout << "取消建图失败" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 5. `GetOdomInfo()`

- **功能**：获取里程计信息
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `odom_info` | `OdomInfo&` | 输出参数，里程计信息对象 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`odom_info`参数包含里程计信息

#### OdomInfo对象详细说明

**OdomInfo结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `pose` | `PoseWithCovariance` | 机器人当前位姿（带协方差） | 位姿 |
| `twist` | `TwistWithCovariance` | 机器人当前速度（带协方差） | 速度 |
| `is_stationary` | `bool` | 机器人是否静止 | 布尔值 |
| `is_sliping` | `bool` | 机器人是否打滑 | 布尔值 |
| `loc_confidence` | `int32_t` | 定位置信度 | 无单位 |
| `loc_state` | `int32_t` | 定位状态 | 无单位 |
| `velocity` | `Vector3` | 机器人当前速度 | 米/秒 |
| `velocity_body` | `Vector3` | 机器人本体速度 | 米/秒 |
| `acceleration` | `Vector3` | 机器人当前加速度 | 米/秒² |
| `ang_vel` | `Vector3` | 机器人当前角速度 | 弧度/秒 |
| `orientation_euler` | `Vector3` | 机器人当前欧拉角 | 弧度 |

```
struct OdomInfo {
  PoseWithCovariance pose{};    ///< robot current pose
  TwistWithCovariance twist{};  ///< robot current twist
  bool is_stationary{};         ///< robot if stationary
  bool is_sliping{};            ///< robot if slipping
  int32_t loc_confidence{};     ///< robot location confidence
  int32_t loc_state{};          ///< robot location state
  Vector3 velocity{};           ///< robot current velocity
  Vector3 velocity_body{};      ///< robot current body velocity
  Vector3 acceleration{};       ///< robot current acceleration
  Vector3 ang_vel{};            ///< robot current angular velocity
  Vector3 orientation_euler{};  ///< robot current orientation euler
};
```

**PoseWithCovariance结构体**：

| 成员名 | 类型 | 描述 |
| --- | --- | --- |
| `pose` | `Pose` | 位姿信息 |
| `covariance` | `std::vector<double>` | 协方差矩阵 |

```
struct PoseWithCovariance {
  Pose pose{};
  std::vector<double> covariance{};
};
```

**TwistWithCovariance结构体**：

| 成员名 | 类型 | 描述 |
| --- | --- | --- |
| `twist` | `Twist` | 速度信息 |
| `covariance` | `std::vector<double>` | 协方差矩阵 |

```
struct TwistWithCovariance {
  Twist twist{};
  std::vector<double> covariance{};
};
```

**Pose结构体**：

| 成员名 | 类型 | 描述 |
| --- | --- | --- |
| `position` | `Position` | 位置信息 |
| `orientation` | `Orientation` | 方向信息 |

```
struct Pose {
  Position position{};
  Orientation orientation{};
};
```

**Position结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `x` | `double` | X坐标 | 米 |
| `y` | `double` | Y坐标 | 米 |
| `z` | `double` | Z坐标 | 米 |

**Orientation结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `x` | `double` | 四元数X分量 | 无单位 |
| `y` | `double` | 四元数Y分量 | 无单位 |
| `z` | `double` | 四元数Z分量 | 无单位 |
| `w` | `double` | 四元数W分量 | 无单位 |

**Twist结构体**：

| 成员名 | 类型 | 描述 |
| --- | --- | --- |
| `linear` | `Vector3` | 线速度 |
| `angular` | `Vector3` | 角速度 |

```
struct Twist {minear{};   ///< linear velocity (m/s)
  Vector3 angular{};  ///< angular velocity (rad/s)
};
```

**Vector3结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `x` | `double` | X轴分量 | 米/秒或弧度/秒 |
| `y` | `double` | Y轴分量 | 米/秒或弧度/秒 |
| `z` | `double` | Z轴分量 | 米/秒或弧度/秒 |

- **示例**：

```
  #include <chrono>
  #include <iostream>
  #include <thread>
  #include "gdk/gdk.h"

  int main()
  {
      // 初始化GDK系统
      if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
          std::cout << "GDK初始化失败" << std::endl;
          return -1;
      }
      std::cout << "GDK初始化成功" << std::endl;

      agibot::gdk::Slam slam;
      std::cout << "Slam init" << std::endl;

      slam.StartMapping();

      std::cout << "Start mapping" << std::endl;
      for(int i = 0; i < 10; i++)
      {
          agibot::gdk::OdomInfo odom;
          if (slam.GetOdomInfo(odom) != agibot::gdk::GDKRes::kSuccess) {
              std::cout << "Failed to get odom info" << std::endl;
          } else {
              std::cout << "pose: (" << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << ", " << odom.pose.pose.position.z << ")" << std::endl;
              std::cout << "orientation (quaternion): x=" << odom.pose.pose.orientation.x
                      << ", y=" << odom.pose.pose.orientation.y
                      << ", z=" << odom.pose.pose.orientation.z
                      << ", w=" << odom.pose.pose.orientation.w << std::endl;
          }
          std::this_thread::sleep_for(std::chrono::seconds(1));
      }
      slam.StopMapping();

      // 释放GDK系统资源
      if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
          std::cout << "GDK释放失败" << std::endl;
          return -1;
      }
      std::cout << "GDK释放成功" << std::endl;

      return 0;
  }
```

#### 6. `RecordSpecLoc()`

- **功能**：记录当前位置为特定位置
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
using namespace agibot::gdk;

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    Slam slam;

    GDKRes result = slam.RecordSpecLoc();

    if (result == GDKRes::kSuccess) {
        std::cout << "记录特定位置成功" << std::endl;
    } else {
        std::cout << "记录特定位置失败" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 7. `GetCurrPose()`

- **功能**：获取机器人当前位置
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `pose` | `Pose&` | 输出参数，机器人当前位姿 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`pose`参数包含机器人当前位姿
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
using namespace agibot::gdk;

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    Slam slam;

    Pose current_pose;
    GDKRes result = slam.GetCurrPose(current_pose);

    if (result == GDKRes::kSuccess) {
        std::cout << "当前位置: (" << current_pose.position.x
                  << ", " << current_pose.position.y
                  << ", " << current_pose.position.z << ")" << std::endl;
        std::cout << "当前方向: x=" << current_pose.orientation.x
                  << ", y=" << current_pose.orientation.y
                  << ", z=" << current_pose.orientation.z
                  << ", w=" << current_pose.orientation.w << std::endl;
    } else {
        std::cout << "获取当前位置失败" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 8. `GlobalLocal()`

- **功能**：全局重定位，用于在已知地图中初始化机器人位姿
- **注意**: 当前仅支持在建图开始时的初始位姿进行重定位，其他位姿进行重定位无法保证后续定位导航精度
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `grid_map_pt` | `const Vector3&` | 栅格地图原点在 map 坐标系下的坐标；`x,y` 为坐标(m)，`z` 为分辨率 |
| `map_pt` | `const Vector3&` | 初始位姿；`x` 为 init\_x，`y` 为 init\_y，`z` 为 init\_yaw(rad) |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回 `GDKRes::kSuccess`
- **示例**（结合 Map 获取 `grid_map.origin` 作为栅格原点）：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>
using namespace agibot::gdk;

int main() {
    if (GDKInit() != GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }

    Slam slam;
    Map map;

    // 获取当前使用地图信息
    MapName current_map;
    if (map.GetCurrMap(current_map) != GDKRes::kSuccess) {
        std::cout << "获取当前地图失败" << std::endl;
        return -1;
    }
    MapInfo map_info;
    if (map.GetMap(current_map.id, map_info) != GDKRes::kSuccess) {
        std::cout << "获取地图失败" << std::endl;
        return -1;
    }

    // 从 grid_map.origin 取出栅格原点坐标
    double origin_x = map_info.grid_map.origin.position.x;
    double origin_y = map_info.grid_map.origin.position.y;
    float resolution = map_info.grid_map.resolution;

    Vector3 grid_map_pt{origin_x, origin_y, static_cast<double>(resolution)};
    Vector3 map_pt{0.0, 0.0, 0.0};  // 初始位姿 (x, y, yaw)

    if (slam.GlobalLocal(grid_map_pt, map_pt) == GDKRes::kSuccess) {
        std::cout << "全局重定位成功" << std::endl;
    } else {
        std::cout << "全局重定位失败" << std::endl;
    }

    if (GDKRelease() != GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    return 0;
}
```

#### 9. 完整使用示例

- **功能**：演示SLAM模块的完整使用流程
- **示例**：

```
  #include <chrono>
  #include <iostream>
  #include <thread>
  #include "gdk/gdk.h"

  int main()
  {
      // 初始化GDK系统
      if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
          std::cout << "GDK初始化失败" << std::endl;
          return -1;
      }
      std::cout << "GDK初始化成功" << std::endl;

      agibot::gdk::Slam slam;
      agibot::gdk::Map map;
      std::cout << "Slam init" << std::endl;

      slam.StartMapping();
      std::cout << "Start mapping" << std::endl;
      for(int i = 0; i < 10; i++)
      {
          agibot::gdk::OdomInfo odom;
          if (slam.GetOdomInfo(odom) != agibot::gdk::GDKRes::kSuccess) {
              std::cout << "Failed to get odom info" << std::endl;
          } else {
              std::cout << "pose: (" << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << ", " << odom.pose.pose.position.z << ")" << std::endl;
              std::cout << "orientation (quaternion): x=" << odom.pose.pose.orientation.x
                      << ", y=" << odom.pose.pose.orientation.y
                      << ", z=" << odom.pose.pose.orientation.z
                      << ", w=" << odom.pose.pose.orientation.w << std::endl;
          }
          uint32_t state;
          if (slam.GetSlamState(state) != agibot::gdk::GDKRes::kSuccess) {
              std::cout << "Failed to get slam state" << std::endl;
          } else {
              std::cout << "slam state: " << state << std::endl;
          }

          std::this_thread::sleep_for(std::chrono::seconds(1));
      }
      agibot::gdk::MapName map_name;
      map.GetCurrMap(map_name);
      std::cout << "Get current map: " << map_name.id << std::endl;

      std::vector<agibot::gdk::MapName> map_names;
      map.GetAllMap(map_names);
      for (const auto& name : map_names) {
          std::cout << "Map: " << name.id << ", " << name.name << ", is current: " << name.is_curr_map << std::endl;
      }
      slam.StopMapping();

      // 释放GDK系统资源
      if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
          std::cout << "GDK释放失败" << std::endl;
          return -1;
      }
      std::cout << "GDK释放成功" << std::endl;

      return 0;
  }
```

## 使用注意事项

1. **GDK初始化**：使用SLAM功能前必须先调用`agibot::gdk::GDKInit()`初始化GDK系统
2. **GDK释放**：程序结束前必须调用`agibot::gdk::GDKRelease()`释放GDK系统资源
3. **建图顺序**：建议先开始建图，再进行其他操作
4. **状态检查**：操作前后建议检查SLAM状态确保操作成功
5. **里程计信息**：获取里程计信息时确保SLAM或PNC正在运行
6. **重定位**：全局重定位需要在地图构建完成后进行
7. **资源管理**：注意SLAM系统的资源使用，避免过度请求
8. **错误处理**：始终检查GDKRes返回值，确保操作成功

## 应用场景

- **环境建图**：构建机器人工作环境的地图
- **实时定位**：获取机器人在环境中的实时位置
- **导航支持**：为导航系统提供定位和地图数据
- **环境建模**：建立环境的空间模型
- **位置记录**：记录和标记重要位置点
- **多地图管理**：支持多个环境的地图构建和管理



---

<!-- source: contents_reference_C++_tf -->

# TF

# GDK TF 接口文档（C++）

## 概述

TF（坐标变换）模块为G02机器人提供了坐标变换查询功能。通过C++接口，开发者可以方便地获取机器人各部件之间的坐标变换关系，适用于坐标变换、传感器标定、多传感器融合、SLAM建图等多种场景。

## 接口说明

### TF 类

该类封装了坐标变换的主要查询接口。

#### 1. `GetAllTfFromBaseLink()`

- **功能**：获取从base\_link到所有子坐标系的变换关系
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `transforms` | `std::vector<TransformStamped>&` | 输出参数，变换关系列表 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`transforms`参数包含所有变换关系

#### TransformStamped对象详细说明

**TransformStamped结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `frame_id` | `std::string` | 父坐标系ID | 字符串 |
| `child_frame_id` | `std::string` | 子坐标系ID | 字符串 |
| `transform` | `Transform` | 变换信息 | 变换对象 |
| `timestamp_ns` | `uint64_t` | 时间戳 | 纳秒 |

```
struct TransformStamped {
  std::string frame_id{};        ///< frame id
  std::string child_frame_id{};  ///< child frame id
  Transform transform{};         ///< transform
  uint64_t timestamp_ns{0};      ///< transform timestamp(ns)
};
```

**Transform结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `translation` | `Vector3` | 平移向量 | 米 |
| `rotation` | `Quaternion` | 旋转四元数 | 无单位 |

```
struct Transform {
  Vector3 translation{};  ///< translation
  Quaternion rotation{};  ///< rotation
};
```

**Vector3结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `x` | `double` | X轴分量 | 米 |
| `y` | `double` | Y轴分量 | 米 |
| `z` | `double` | Z轴分量 | 米 |

**Quaternion结构体**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `x` | `double` | 四元数X分量 | 无单位 |
| `y` | `double` | 四元数Y分量 | 无单位 |
| `z` | `double` | 四元数Z分量 | 无单位 |
| `w` | `double` | 四元数W分量 | 无单位 |

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::TF tf;
    std::cout << "TF init" << std::endl;

    std::vector<agibot::gdk::TransformStamped> transforms;
    if (tf.GetAllTfFromBaseLink(transforms) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to get all transforms" << std::endl;
    } else {
        std::cout << "获取到 " << transforms.size() << " 个变换关系:" << std::endl;
        for (const auto& transform_stamped : transforms) {
            std::cout << "坐标系: " << transform_stamped.frame_id
                      << " -> " << transform_stamped.child_frame_id << std::endl;
            std::cout << "  平移: x=" << transform_stamped.transform.translation.x
                      << ", y=" << transform_stamped.transform.translation.y
                      << ", z=" << transform_stamped.transform.translation.z << std::endl;
            std::cout << "  旋转: x=" << transform_stamped.transform.rotation.x
                      << ", y=" << transform_stamped.transform.rotation.y
                      << ", z=" << transform_stamped.transform.rotation.z
                      << ", w=" << transform_stamped.transform.rotation.w << std::endl;
            std::cout << "  时间戳: " << transform_stamped.timestamp_ns << std::endl;
            std::cout << std::endl;
        }
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 2. `GetTfFromBaseLink()`

- **功能**：获取从base\_link到指定子坐标系的变换关系
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `child_frame_id` | `const std::string&` | 子坐标系ID |
| `transform` | `Transform&` | 输出参数，变换信息对象 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`transform`参数包含变换信息
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::TF tf;
    std::cout << "TF init" << std::endl;

    agibot::gdk::Transform transform;
    std::string child_frame_id = "arm_l_end_link";

    if (tf.GetTfFromBaseLink(child_frame_id, transform) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to get transform for " << child_frame_id << std::endl;
    } else {
        std::cout << "从base_link到 " << child_frame_id << " 的变换:" << std::endl;
        std::cout << "  平移: x=" << transform.translation.x
                  << ", y=" << transform.translation.y
                  << ", z=" << transform.translation.z << std::endl;
        std::cout << "  旋转: x=" << transform.rotation.x
                  << ", y=" << transform.rotation.y
                  << ", z=" << transform.rotation.z
                  << ", w=" << transform.rotation.w << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 3. `GetTfFromSensor()`

- **功能**：获取传感器外参变换关系
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `sensor_extrinsic_type` | `const SensorExtrinsicType&` | 传感器外参类型枚举 |
| `transform` | `Transform&` | 输出参数，变换信息对象 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`transform`参数包含传感器外参变换信息

#### SensorExtrinsicType枚举详细说明

**支持的传感器外参类型**：

| 枚举值 | 描述 |
| --- | --- |
| `kUnknown` | 未知类型 |
| `kHeadLeftStereoToHeadRightStereo` | 头部左立体相机到右立体相机 |
| `kLeftHandDepthToLeftHandColor` | 左手深度相机到彩色相机 |
| `kRightHandDepthToRightHandColor` | 右手深度相机到彩色相机 |
| `kHeadDepthToHeadColor` | 头部深度相机到彩色相机 |
| `kHeadLeftStereoToHeadLink3` | 头部左立体相机到Headlink3（头部第三连杆） |
| `kHeadRightStereoToHeadLink3` | 头部右立体相机到Headlink3（头部第三连杆） |
| `kHeadLeftFisheyeToHeadLink3` | 头部左鱼眼相机到Headlink3（头部第三连杆） |
| `kHeadRightFisheyeToHeadLink3` | 头部右鱼眼相机到Headlink3（头部第三连杆） |
| `kHeadBackFisheyeToHeadLink3` | 头部后鱼眼相机到Headlink3（头部第三连杆） |
| `kChassisFrontLidarToBaseLink` | 底盘前激光雷达到base\_link |
| `kChassisBackLidarToBaseLink` | 底盘后激光雷达到base\_link |
| `kChassisBackLidarToChassisFrontLidar` | 底盘后激光雷达到前激光雷达 |
| `kChassisMid360ImuToChassisMid360Lidar` | 底盘Mid360 [IMU](../../../concepts/sensors/#imu)到底盘Mid360激光雷达 |
| `kChassisImuToBaseLink` | 底盘IMU到base\_link |
| `kLeftHandRGBDToArmLEndLink` | 左手[RGBD](../../../concepts/sensors/#rgbd)到左臂末端链接 |
| `kRightHandRGBDToArmREndLink` | 右手RGBD到右臂末端链接 |
| `kHeadRGBDToHeadLink3` | 头部RGBD到Headlink3（头部第三连杆） |

```
enum class SensorExtrinsicType {
  kUnknown = 0,
  kHeadLeftStereoToHeadRightStereo,
  kLeftHandDepthToLeftHandColor,
  kRightHandDepthToRightHandColor,
  kHeadDepthToHeadColor,
  kHeadLeftStereoToHeadLink3,
  kHeadRightStereoToHeadLink3,
  kHeadLeftFisheyeToHeadLink3,
  kHeadRightFisheyeToHeadLink3,
  kHeadBackFisheyeToHeadLink3,
  kChassisFrontLidarToBaseLink,
  kChassisBackLidarToBaseLink,
  kChassisBackLidarToChassisFrontLidar,
  kChassisMid360ImuToChassisMid360Lidar,
  kChassisImuToBaseLink,
  kLeftHandRGBDToArmLEndLink,
  kRightHandRGBDToArmREndLink,
  kHeadRGBDToHeadLink3
};
```

- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::TF tf;
    std::cout << "TF init" << std::endl;

    agibot::gdk::Transform transform;
    agibot::gdk::SensorExtrinsicType sensor_type =
        agibot::gdk::SensorExtrinsicType::kHeadLeftStereoToHeadRightStereo;

    if (tf.GetTfFromSensor(sensor_type, transform) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to get sensor extrinsic transform" << std::endl;
    } else {
        std::cout << "传感器外参变换:" << std::endl;
        std::cout << "  平移: x=" << transform.translation.x
                  << ", y=" << transform.translation.y
                  << ", z=" << transform.translation.z << std::endl;
        std::cout << "  旋转: x=" << transform.rotation.x
                  << ", y=" << transform.rotation.y
                  << ", z=" << transform.rotation.z
                  << ", w=" << transform.rotation.w << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 4. `LookupTransformLatest()`

- **功能**：查询两个坐标系之间的最新变换关系
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `target_frame` | `const std::string&` | 目标坐标系ID |
| `source_frame` | `const std::string&` | 源坐标系ID |
| `transform` | `Transform&` | 输出参数，变换信息对象（从source到target） |
| `timestamp_ns` | `uint64_t*` | 输出参数，时间戳指针（可选，可为nullptr） |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`transform`参数包含变换信息
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::TF tf;
    std::cout << "TF init" << std::endl;

    agibot::gdk::Transform transform;
    uint64_t timestamp_ns = 0;

    // 查询从arm_l_end_link到base_link的最新变换
    if (tf.LookupTransformLatest("base_link", "arm_l_end_link", transform, &timestamp_ns)
        != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to lookup transform" << std::endl;
    } else {
        std::cout << "从arm_l_end_link到base_link的最新变换:" << std::endl;
        std::cout << "  平移: x=" << transform.translation.x
                  << ", y=" << transform.translation.y
                  << ", z=" << transform.translation.z << std::endl;
        std::cout << "  旋转: x=" << transform.rotation.x
                  << ", y=" << transform.rotation.y
                  << ", z=" << transform.rotation.z
                  << ", w=" << transform.rotation.w << std::endl;
        std::cout << "  时间戳: " << timestamp_ns << " ns" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 5. `LookupTransform()`

- **功能**：查询两个坐标系在特定时间的变换关系（支持时间插值）
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `target_frame` | `const std::string&` | 目标坐标系ID |
| `source_frame` | `const std::string&` | 源坐标系ID |
| `time_ns` | `uint64_t` | 查询时间（纳秒时间戳） |
| `transform` | `Transform&` | 输出参数，变换信息对象（从source到target） |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`transform`参数包含变换信息
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::TF tf;
    std::cout << "TF init" << std::endl;

    // 获取当前时间戳
    auto now = std::chrono::system_clock::now();
    uint64_t current_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();

    // 查询1秒前的变换
    uint64_t target_time_ns = current_time_ns - 1000000000ULL; // 1秒前

    agibot::gdk::Transform transform;
    if (tf.LookupTransform("base_link", "arm_l_end_link", target_time_ns, transform)
        != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to lookup transform at time " << target_time_ns << std::endl;
    } else {
        std::cout << "从arm_l_end_link到base_link在时间 " << target_time_ns << " 的变换:" << std::endl;
        std::cout << "  平移: x=" << transform.translation.x
                  << ", y=" << transform.translation.y
                  << ", z=" << transform.translation.z << std::endl;
        std::cout << "  旋转: x=" << transform.rotation.x
                  << ", y=" << transform.rotation.y
                  << ", z=" << transform.rotation.z
                  << ", w=" << transform.rotation.w << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 6. `CanTransform()`

- **功能**：检查两个坐标系之间是否存在变换关系
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `target_frame` | `const std::string&` | 目标坐标系ID |
| `source_frame` | `const std::string&` | 源坐标系ID |

- **返回值**：`bool`，如果存在变换关系返回`true`，否则返回`false`
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::TF tf;
    std::cout << "TF init" << std::endl;

    // 检查是否存在变换
    if (tf.CanTransform("base_link", "arm_l_end_link")) {
        std::cout << "存在从arm_l_end_link到base_link的变换" << std::endl;
    } else {
        std::cout << "不存在从arm_l_end_link到base_link的变换" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 7. `GetAllFrameNames()`

- **功能**：获取所有可用的坐标系名称
- **参数**：无
- **返回值**：`std::vector<std::string>`，包含所有坐标系名称的列表
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::TF tf;
    std::cout << "TF init" << std::endl;

    // 获取所有坐标系名称
    std::vector<std::string> frame_names = tf.GetAllFrameNames();
    std::cout << "所有可用坐标系 (" << frame_names.size() << " 个):" << std::endl;
    for (const auto& name : frame_names) {
        std::cout << "  - " << name << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 8. `GetLatestTimestamp()`

- **功能**：获取指定坐标系的最新时间戳
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `frame_id` | `const std::string&` | 坐标系ID |
| `timestamp_ns` | `uint64_t&` | 输出参数，最新时间戳（纳秒） |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`timestamp_ns`参数包含最新时间戳
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::TF tf;
    std::cout << "TF init" << std::endl;

    uint64_t timestamp_ns = 0;
    if (tf.GetLatestTimestamp("arm_l_end_link", timestamp_ns)
        != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "Failed to get latest timestamp" << std::endl;
    } else {
        std::cout << "arm_l_end_link 最新时间戳: " << timestamp_ns << " ns" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 9. `Clear()`

- **功能**：清空TF缓存中的所有变换关系
- **参数**：无
- **返回值**：无（`void`）
- **示例**：

```
#include "gdk/gdk.h"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::TF tf;
    std::cout << "TF init" << std::endl;

    // 清空TF缓存
    tf.Clear();
    std::cout << "TF缓存已清空" << std::endl;

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

## 使用注意事项

1. **GDK初始化**：使用TF功能前必须先调用`agibot::gdk::GDKInit()`初始化GDK系统
2. **GDK释放**：程序结束前必须调用`agibot::gdk::GDKRelease()`释放GDK系统资源
3. **坐标系命名**：使用正确的坐标系名称，可通过`GetAllFrameNames()`获取所有可用坐标系的名称
4. **时间戳精度**：时间戳单位为纳秒，可用于精确的时间同步
5. **变换矩阵**：变换信息包含平移和旋转，可用于坐标转换计算
6. **传感器标定**：使用`GetTfFromSensor()`获取传感器外参，用于多传感器数据融合
7. **实时性**：变换关系会实时更新，反映机器人当前状态
8. **错误处理**：始终检查GDKRes返回值，确保操作成功
9. **变换查询**：`LookupTransformLatest()`查询最新变换，`LookupTransform()`支持时间插值查询历史变换
10. **变换检查**：使用`CanTransform()`在查询前检查变换是否存在，避免查询失败
11. **时间插值**：`LookupTransform()`支持时间插值，可以查询历史任意时刻的变换关系
12. **缓存管理**：使用`Clear()`可以清空TF缓存，适用于需要重置变换关系的场景
13. **时间戳查询**：`GetLatestTimestamp()`可以获取指定坐标系的最新更新时间

## 应用场景

- **坐标变换**：实现不同坐标系之间的坐标转换
- **传感器标定**：获取传感器外参，用于多传感器标定
- **多传感器融合**：结合多个传感器的数据，提高感知精度
- **SLAM建图**：为SLAM算法提供坐标系变换信息
- **路径规划**：在规划路径时考虑不同部件的坐标系关系
- **视觉处理**：将图像坐标转换为机器人坐标系
- **运动控制**：在控制机器人运动时考虑坐标系变换
- **数据同步**：基于时间戳进行多传感器数据同步



---

<!-- source: contents_reference_C++_uss -->

# UltrasonicRadar

# GDK UltrasonicRadar 接口文档（C++）

## 概述

UltrasonicRadar（超声波雷达）模块为G02机器人提供了获取实时超声波雷达数据的功能。通过C++接口，开发者可以方便地获取机器人的障碍物检测数据，适用于避障、导航、安全检测、近距离障碍物感知等多种场景。

## 接口说明

### UltrasonicRadar 类

该类封装了超声波雷达传感器的主要数据获取接口。

#### 1. `GetLatestUltrasonicRadar()`

- **功能**：获取最新的超声波雷达数据
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `ultrasonic_radar` | `std::shared_ptr<UltrasonicRadars>&` | 输出参数，超声波雷达数据指针 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`ultrasonic_radar`参数包含超声波雷达数据

#### UltrasonicRadars对象详细说明

**UltrasonicRadars结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `timestamp_ns` | `uint64_t` | 超声波雷达数据采集的时间戳（底盘传感器时间戳） | 纳秒 |
| `ultrasonic_radar_datas` | `std::vector<UltrasonicRadarData>` | 超声波雷达数据列表 | 无单位 |

```
struct UltrasonicRadars{
  uint64_t timestamp_ns{0};  ///< timestamp in nanoseconds
  std::vector<UltrasonicRadarData> ultrasonic_radar_datas;  ///< ultrasonic radar datas
};
```

**ultrasonic\_radar\_datas (雷达数据列表)**：

每个`UltrasonicRadarData`包含以下属性：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `id` | `uint32_t` | 超声波雷达ID | 无 |
| `distance_mm` | `uint32_t` | 检测到的距离 | 毫米 |
| `fault_state` | `uint8_t` | 故障状态（0表示正常，非0表示存在故障） | 无 |

```
struct UltrasonicRadarData {
  uint32_t id{};           ///< ultrasonic_radar_id
  uint32_t distance_mm{};  ///< distance in millimeters
  uint8_t fault_state{};  ///< fault state
};
```

- **示例**：

```
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::UltrasonicRadar radar;
    std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待1秒以确保DDS连接建立

    std::shared_ptr<agibot::gdk::UltrasonicRadars> ultrasonic_radar;
    auto res = radar.GetLatestUltrasonicRadar(ultrasonic_radar);

    if (res == agibot::gdk::GDKRes::kSuccess && ultrasonic_radar != nullptr) {
        std::cout << "✅ 时间戳: " << ultrasonic_radar->timestamp_ns << std::endl;
        std::cout << "超声波雷达数量: " << ultrasonic_radar->ultrasonic_radar_datas.size() << std::endl;

        for (const auto& data : ultrasonic_radar->ultrasonic_radar_datas) {
            std::cout << "  雷达[" << data.id << "]: "
                      << "距离=" << data.distance_mm << " mm, "
                      << "故障状态=" << static_cast<int>(data.fault_state) << std::endl;
        }
    } else {
        std::cout << "未获取到超声波雷达数据" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 2. `GetNearestUltrasonicRadar()`

- **功能**：获取指定时间戳附近最近的超声波雷达数据
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `timestamp_ns` | `const uint64_t` | 目标时间戳（纳秒） |
| `ultrasonic_radar` | `std::shared_ptr<UltrasonicRadars>&` | 输出参数，超声波雷达数据指针 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`ultrasonic_radar`参数包含超声波雷达数据
- **示例**：

```
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::UltrasonicRadar radar;

    // 先获取最新数据
    std::shared_ptr<agibot::gdk::UltrasonicRadars> latest_radar;
    radar.GetLatestUltrasonicRadar(latest_radar);

    if (latest_radar != nullptr) {
        std::cout << "✅ 最新数据时间戳: " << latest_radar->timestamp_ns << std::endl;

        // 查找最近的数据（往前1秒）
        std::shared_ptr<agibot::gdk::UltrasonicRadars> nearest_radar;
        agibot::gdk::GDKRes res = radar.GetNearestUltrasonicRadar(
            latest_radar->timestamp_ns - 1000000000LL, // 往前1秒
            nearest_radar
        );
        if (res == agibot::gdk::GDKRes::kSuccess && nearest_radar != nullptr) {
            std::cout << "✅ 最近数据时间戳: " << nearest_radar->timestamp_ns << std::endl;
            std::cout << "时间差: " << (nearest_radar->timestamp_ns > latest_radar->timestamp_ns - 1000000000LL ?
                                          nearest_radar->timestamp_ns - (latest_radar->timestamp_ns - 1000000000LL) :
                                          (latest_radar->timestamp_ns - 1000000000LL) - nearest_radar->timestamp_ns)
                      << " ns" << std::endl;
            std::cout << "超声波雷达数量: " << nearest_radar->ultrasonic_radar_datas.size() << std::endl;
        } else {
            std::cout << "❌ 未找到最近的超声波雷达数据" << std::endl;
        }
    } else {
        std::cout << "未获取到超声波雷达数据" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 3. `GetUltrasonicRadarFps()`

- **功能**：获取超声波雷达数据采集帧率
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `fps` | `float&` | 输出参数，超声波雷达帧率（FPS） |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`fps`参数包含帧率值
- **示例**：

```
#include <iostream>
#include <chrono>
#include <thread>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::UltrasonicRadar radar;

    float fps;
    if (radar.GetUltrasonicRadarFps(fps) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "获取帧率失败" << std::endl;
    } else {
        std::cout << "超声波雷达帧率: " << fps << " fps" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 4. `GetUltrasonicRadarLatency()`

- **功能**：获取超声波雷达数据延迟统计信息
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `window_seconds` | `const float` | 统计窗口时间（秒） |
| `latency` | `LatencyStats&` | 输出参数，延迟统计信息 |

- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`，`latency`参数包含延迟统计信息
- **LatencyStats结构体说明**：

```
struct LatencyStats {
  double max_latency_ms{0.0};    ///< max latency(ms)
  double avg_latency_ms{0.0};    ///< average latency(ms)
  double p99_latency_ms{0.0};    ///< 99th percentile latency(ms)
  double p999_latency_ms{0.0};   ///< 99.9th percentile latency(ms)
  double p9999_latency_ms{0.0};  ///< 99.99th percentile latency(ms)
};
```

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `max_latency_ms` | `double` | 最大延迟 | 毫秒 |
| `avg_latency_ms` | `double` | 平均延迟 | 毫秒 |
| `p99_latency_ms` | `double` | 99分位延迟 | 毫秒 |
| `p999_latency_ms` | `double` | 99.9分位延迟 | 毫秒 |
| `p9999_latency_ms` | `double` | 99.99分位延迟 | 毫秒 |

- **示例**：

```
#include <iostream>
#include <chrono>
#include <thread>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::UltrasonicRadar radar;
    // 等待一段时间收集数据
    std::this_thread::sleep_for(std::chrono::seconds(10));

    agibot::gdk::LatencyStats latency;
    if (radar.GetUltrasonicRadarLatency(10.0, latency) != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "获取延迟统计失败" << std::endl;
    } else {
        std::cout << "超声波雷达延迟统计:" << std::endl;
        std::cout << "  最大延迟: " << latency.max_latency_ms << "ms" << std::endl;
        std::cout << "  平均延迟: " << latency.avg_latency_ms << "ms" << std::endl;
        std::cout << "  P99延迟: " << latency.p99_latency_ms << "ms" << std::endl;
        std::cout << "  P99.9延迟: " << latency.p999_latency_ms << "ms" << std::endl;
        std::cout << "  P99.99延迟: " << latency.p9999_latency_ms << "ms" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

#### 5. `Close()`

- **功能**：关闭超声波雷达DDS连接
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes::kSuccess`
- **示例**：

```
#include <iostream>
#include <chrono>
#include <thread>
#include "gdk/gdk.h"

int main() {
    // 初始化GDK系统
    if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK初始化失败" << std::endl;
        return -1;
    }
    std::cout << "GDK初始化成功" << std::endl;

    agibot::gdk::UltrasonicRadar radar;
    std::cout << "UltrasonicRadar init" << std::endl;

    // 使用超声波雷达...

    // 关闭超声波雷达
    if (radar.Close() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "关闭超声波雷达失败" << std::endl;
    } else {
        std::cout << "超声波雷达关闭成功" << std::endl;
    }

    // 释放GDK系统资源
    if (agibot::gdk::GDKRelease() != agibot::gdk::GDKRes::kSuccess) {
        std::cout << "GDK释放失败" << std::endl;
        return -1;
    }
    std::cout << "GDK释放成功" << std::endl;

    return 0;
}
```

## 使用注意事项

1. **GDK初始化**：使用UltrasonicRadar功能前必须先调用`agibot::gdk::GDKInit()`初始化GDK系统
2. **GDK释放**：程序结束前必须调用`agibot::gdk::GDKRelease()`释放GDK系统资源
3. **返回值检查**：使用前请检查GDKRes返回值是否为kSuccess
4. **智能指针管理**：UltrasonicRadars对象使用shared\_ptr管理，注意生命周期
5. **时间戳精度**：时间戳单位为纳秒，是底盘传感器的时间戳，可用于精确的时间同步
6. **距离单位**：距离单位为毫米（mm），使用时注意单位转换
7. **故障状态**：`fault_state`为0表示正常，非0值表示存在故障，使用时需要检查
8. **数据获取**：`GetLatestUltrasonicRadar()`返回当前最新数据，如果没有新数据可能返回失败
9. **时间戳查找**：`GetNearestUltrasonicRadar()`根据时间戳查找最接近的数据，如果时间戳超出范围可能返回失败
10. **帧率统计**：`GetUltrasonicRadarFps()`需要等待一段时间（建议至少2秒）让数据积累后才能获得准确的帧率
11. **延迟统计**：`GetUltrasonicRadarLatency()`需要等待一段时间（建议至少10秒）让数据积累后才能获得准确的统计信息
12. **资源释放**：使用完毕后调用`Close()`释放资源
13. **错误处理**：始终检查GDKRes返回值，确保操作成功

## 应用场景

- **避障检测**：实时检测机器人周围的障碍物，用于避障决策
- **近距离感知**：检测近距离的障碍物，补充激光雷达的盲区
- **安全检测**：监控机器人周围的安全区域，防止碰撞
- **导航辅助**：为机器人导航提供近距离障碍物信息
- **停车辅助**：辅助机器人进行精确停车和定位
- **低速导航**：在低速移动时提供可靠的障碍物检测
- **多传感器融合**：与其他传感器（如激光雷达、相机）数据进行融合，提高感知精度
- **安全区域监控**：监控机器人周围的安全区域，确保安全运行
- **障碍物分类**：结合距离信息进行障碍物分类和识别
- **路径规划**：基于超声波雷达数据规划安全路径



---

<!-- source: contents_reference_Python_camera -->

# Camera

# GDK Camera 接口文档（Python）

## 概述

Camera（相机）模块为G02机器人提供了获取实时图像数据的功能。通过Python接口，开发者可以方便地获取机器人的视觉感知数据，适用于目标检测、图像识别、视觉导航、SLAM建图、环境监控等多种场景。主要的相机类别如下所示。

- RGB-D 相机：
  同时输出彩色图（RGB）和深度图（Depth）的相机。深度值表示每个像素到相机的距离，通常通过结构光或 ToF（飞行时间）技术实现。GDK 中 RGB-D 相机安装于机器人头部和手部，用于近距离感知与抓取任务。
- 立体相机
  由两个固定基线距离的镜头组成，通过左右图像的视差计算深度。GDK 中立体相机用于头部的远距离三维感知。
- 鱼眼相机
  超广角镜头，视角可达 180° 以上，以较大的图像畸变换取宽广的视野范围。GDK 中鱼眼相机安装于头部，用于环境全景感知。

## 接口说明

### Camera 类

该类封装了相机传感器的主要数据获取接口。

#### 构造函数 / 初始化

- **无参构造**：`Camera()`
  启动所有相机
- **指定相机构造**：`Camera(camera_types)`
  仅启动传入的相机类型，可减少资源占用。

| 参数名 | 类型 | 必填 | 描述 |
| --- | --- | --- | --- |
| `camera_types` | `list[CameraType]` | 否 | 要启动的相机类型列表；不传或传空列表时等价于启动全部相机 |

**示例**：

```
import agibot_gdk

# 启动所有相机（默认）
camera = agibot_gdk.Camera()

# 仅启动头部彩色与左右手彩色
# camera = agibot_gdk.Camera([
#     agibot_gdk.CameraType.kHeadColor,
#     agibot_gdk.CameraType.kHandLeftColor,
#     agibot_gdk.CameraType.kHandRightColor,
# ])
```

#### 1. `get_latest_image()`

- **功能**：获取最新的图像数据
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `type` | `CameraType` | 相机类型枚举值 |
| `timeout` | `float` | 超时时间（毫秒） |

- **返回值**：`Image`对象，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `timestamp_ns` | `int` | 图像采集的时间戳 | 纳秒 |
| `width` | `int` | 图像的宽度（像素） | 像素 |
| `height` | `int` | 图像的高度（像素） | 像素 |
| `encoding` | `Encoding` | 图像的编码格式 | 枚举 |
| `color_format` | `ColorFormat` | 图像的颜色格式 | 枚举 |
| `bit_depth` | `int` | 每个像素的位数 | 位 |
| `data` | `numpy.ndarray` | 图像的原始像素数据 | 字节 |

##### Image对象详细说明

**encoding (编码格式)**：

- **类型**：`Encoding`
- **常见值**：
- `UNCOMPRESSED`: 未压缩原始图像数据
- `JPEG`: JPEG压缩格式
- `PNG`: PNG压缩格式

**color\_format (颜色格式)**：

- **类型**：`ColorFormat`
- **全部枚举值**：

| 枚举值 | 描述 |
| --- | --- |
| `RGB` | 红绿蓝（8位每通道） |
| `BGR` | 蓝绿红（8位每通道，OpenCV 默认格式） |
| `RGBA` | 红绿蓝+透明通道 |
| `BGRA` | 蓝绿红+透明通道 |
| `YUV420` | YUV 4:2:0 |
| `YUV422` | YUV 4:2:2 |
| `YUV444` | YUV 4:4:4 |
| `NV12` | YUV 4:2:0 半平面（Y + 交错 UV） |
| `NV21` | YUV 4:2:0 半平面（Y + 交错 VU） |
| `GRAY8` | 8位灰度 |
| `GRAY16` | 16位灰度 |
| `BAYER_RGGB` | Bayer 阵列 RGGB |
| `BAYER_BGGR` | Bayer 阵列 BGGR |
| `BAYER_GBRG` | Bayer 阵列 GBRG |
| `BAYER_GRBG` | Bayer 阵列 GRBG |
| `RS2_FORMAT_Z16` | RealSense 深度图（16位无符号整数，单位毫米） |

**bit\_depth (位深度)**：

- **类型**：`int`
- **常见值**：
- `8`: 8位（0-255）
- `16`: 16位（0-65535）
- `32`: 32位（浮点）

**data (图像数据)**：

- **类型**：`numpy.ndarray`
- **描述**：图像的原始像素数据
- **用途**：图像处理、显示、保存
- **注意**：需要根据encoding和尺寸进行解析

**相机类型**：

- `kHeadBackFisheye`: 头部背部鱼眼相机
- `kHeadLeftFisheye`: 头部左侧鱼眼相机
- `kHeadRightFisheye`: 头部右侧鱼眼相机
- `kHeadStereoLeft`: 头部立体左相机
- `kHeadStereoRight`: 头部立体右相机
- `kHandLeftColor`: 左手彩色相机
- `kHandRightColor`: 右手彩色相机
- `kHeadColor`: 头部彩色相机
- `kHeadDepth`: 头部深度相机（输出为深度图）
- `kHandLeftDepth`: 左手深度相机（输出为深度图）
- `kHandRightDepth`: 右手深度相机（输出为深度图）
- `kHandLeftUpperColor`: 左手上部彩色相机 (预留)
- `kHandRightUpperColor`: 右手上部彩色相机 (预留)
- `kHandLeftLowerColor`: 左手下部彩色相机 (预留)
- `kHandRightLowerColor`: 右手下部彩色相机 (预留)
- `kHandLeftUpperDepth`: 左手上部深度相机（输出为深度图） (预留)
- `kHandRightUpperDepth`: 右手上部深度相机（输出为深度图） (预留)
- `kHandLeftLowerDepth`: 左手下部深度相机（输出为深度图） (预留)
- `kHandRightLowerDepth`: 右手下部深度相机（输出为深度图） (预留)
- 在常规模式下，默认打开头部立体左相机，头部立体右相机，左右彩色相机，右手彩色相机，头部彩色相机，头部深度相机，其余相机默认关闭，且不建议在常规模式下打开其余相机
- 可以在develop模式下开启或关闭其余相机
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

camera = agibot_gdk.Camera()

image = camera.get_latest_image(agibot_gdk.CameraType.kHeadStereoLeft, 1000.0)
if image is not None:
    print(f"✅ 时间戳: {image.timestamp_ns}")
    print(f"图像尺寸: {image.width} x {image.height}")
    print(f"编码格式: {image.encoding}")
    print(f"颜色格式: {image.color_format}")
    print(f"位深度: {image.bit_depth}")
else:
    print("未获取到图像数据")

# 关闭相机
camera.close_camera()

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 2. `get_nearest_image()`

- **功能**：获取指定时间戳附近最近的图像数据
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `type` | `CameraType` | 相机类型枚举值 |
| `timestamp` | `int` | 目标时间戳（纳秒） |
| `timeout` | `float` | 超时时间（毫秒） |

- **返回值**：`Image`对象，结构与`get_latest_image()`相同
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

camera = agibot_gdk.Camera()

# 先获取最新图像数据
image = camera.get_latest_image(agibot_gdk.CameraType.kHeadStereoLeft, 1000.0)

if image is not None:
    # 获取历史图像数据（往前1秒）
    image_nearest = camera.get_nearest_image(
        agibot_gdk.CameraType.kHeadStereoLeft,
        image.timestamp_ns - 1000000000,
        1000.0
    )

    if image_nearest is not None:
        print(f"✅ 最近图像数据: {image_nearest.timestamp_ns}")
        print(f"图像尺寸: {image_nearest.width} x {image_nearest.height}")
        print(f"编码格式: {image_nearest.encoding}")
    else:
        print("❌ 未找到最近的图像数据")
else:
    print("未获取到最新图像数据")

# 关闭相机
camera.close_camera()

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 3. `get_image_shape()`

- **功能**：获取图像数据大小
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `type` | `CameraType` | 相机类型枚举值 |

- **返回值**：`tuple`，包含图像宽度和高度的元组 `(width, height)`
- **示例**：

```
import agibot_gdk

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

camera = agibot_gdk.Camera()

shape = camera.get_image_shape(agibot_gdk.CameraType.kHeadStereoLeft)
print(f"图像尺寸: {shape[0]} x {shape[1]}")

# 关闭相机
camera.close_camera()

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 4. `get_image_fps()`

- **功能**：获取图像捕获帧率
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `type` | `CameraType` | 相机类型枚举值 |

- **返回值**：`float`，图像帧率（FPS）
- **示例**：

```
import agibot_gdk

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

camera = agibot_gdk.Camera()

fps = camera.get_image_fps(agibot_gdk.CameraType.kHeadStereoLeft)
print(f"图像帧率: {fps} FPS")

# 关闭相机
camera.close_camera()

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 5. `get_image_latency()`

- **注意事项**：获取图像延迟统计信息前，需要先进行时间同步，否则延迟统计结果不准确
- **功能**：获取图像延迟统计信息
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `type` | `CameraType` | 相机类型枚举值 |
| `window_seconds` | `float` | 统计窗口时间（秒） |

- **返回值**：`LatencyStats`对象，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `max_latency_ms` | `float` | 最大延迟 | 毫秒 |
| `avg_latency_ms` | `float` | 平均延迟 | 毫秒 |
| `p99_latency_ms` | `float` | 99分位延迟 | 毫秒 |
| `p999_latency_ms` | `float` | 99.9分位延迟 | 毫秒 |
| `p9999_latency_ms` | `float` | 99.99分位延迟 | 毫秒 |

- **示例**：

```
import agibot_gdk

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

camera = agibot_gdk.Camera()

latency = camera.get_image_latency(agibot_gdk.CameraType.kHeadStereoLeft, 1.0)
print(f"最大延迟: {latency.max_latency_ms} ms")
print(f"平均延迟: {latency.avg_latency_ms} ms")
print(f"99分位延迟: {latency.p99_latency_ms} ms")
print(f"99.9分位延迟: {latency.p999_latency_ms} ms")
print(f"99.99分位延迟: {latency.p9999_latency_ms} ms")

# 关闭相机
camera.close_camera()

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 6. `get_camera_intrinsic()`

- **功能**：获取相机内参信息
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `type` | `CameraType` | 相机类型枚举值 |

- **返回值**：`CameraIntrinsic`对象，包含相机内参信息
- **注意**：并非所有相机类型都支持内参获取。支持内参的相机类型包括：`kHeadBackFisheye`、`kHeadLeftFisheye`、`kHeadRightFisheye`、`kHeadStereoLeft`、`kHeadStereoRight`、`kHandLeftColor`、`kHandRightColor`、`kHeadColor`、`kHeadDepth`、`kHandLeftDepth`、`kHandRightDepth`。对于不支持的相机类型，调用此接口将抛出异常。

#### CameraIntrinsic对象详细说明

**CameraIntrinsic结构体包含以下成员**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `intrinsic` | `list[float]` | 相机内参 [fx, fy, cx, cy] | 像素 |
| `distortion` | `list[float]` | 畸变参数 [k1, k2, p1, p2, k3, k4, k5, k6] | 无单位 |

**不同相机类型的内参支持**：

| 相机类型 | intrinsic长度 | distortion长度 | 说明 |
| --- | --- | --- | --- |
| 双目相机 | 4 | 8 | fx,fy,cx,cy,k1,k2,p1,p2,k3,k4,k5,k6 |
| RGB-D相机 | 4 | 5 | fx,fy,cx,cy,k1,k2,p1,p2,k3 |
| 鱼眼相机 | 4 | 6 | fx,fy,cx,cy,k1,k2,p1,p2,k3,k4 |

- **示例**：

```
import agibot_gdk

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

camera = agibot_gdk.Camera()

intrinsic = camera.get_camera_intrinsic(agibot_gdk.CameraType.kHeadStereoLeft)
print(f"相机内参:")
print(f"  fx: {intrinsic.intrinsic[0]}")
print(f"  fy: {intrinsic.intrinsic[1]}")
print(f"  cx: {intrinsic.intrinsic[2]}")
print(f"  cy: {intrinsic.intrinsic[3]}")

print(f"畸变参数:")
for i, dist in enumerate(intrinsic.distortion):
    print(f"  k{i+1}: {dist}")

# 关闭相机
camera.close_camera()

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 7. `set_dev_camera_config()`

- **功能**：设置相机开发配置
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `cam_conf_path` | `str` | 相机配置文件路径 |

- **返回值**：无。成功时不返回，失败时抛出`RuntimeError`异常
- **注意**：
- 配置文件路径必须存在，否则会抛出异常
- 每次修改客制化相机配置文件且调用接口后，需要重新切换模式至develop模式

##### 配置选项

GDK支持配置相机的开关与设置的帧率，相机客制化的配置文件在部署包下，其绝对路径通常为 `~/.cache/agibot/app/gdk/config/r1_camera_conf.json`或`~/.cache/agibot/app/gdk/config/thor_camera_conf.json`（其中 `~` 表示用户主目录，如 `/home/your_name`）
r1机器的配置文件为r1\_camera\_conf.json，thor机器的配置文件为thor\_camera\_conf.json，使用时注意文件名称
设置publish为true开启相机，false关闭相机，设置fps来控制相机帧率
相机具体配置形式如下

```
{
    "cam0": {
        "fps": "30",
        "name": "head_stereo_right",
        "publish": true
    },
    "cam3": {
        "fps": "30",
        "name": "head_stereo_left",
        "publish": true
    },
    "cam4": {
        "fps": "30",
        "name": "hand_left_depth",
        "publish": false
    },
    "cam5": {
        "fps": "30",
        "name": "hand_left_color",
        "publish": true
    },
    "cam6": {
        "fps": "30",
        "name": "hand_right_depth",
        "publish": false
    },
    "cam7": {
        "fps": "30",
        "name": "hand_right_color",
        "publish": true
    },
    "cam10": {
        "fps": "30",
        "name": "head_right_fisheye",
        "publish": false
    },
    "cam11": {
        "fps": "30",
        "name": "head_left_fisheye",
        "publish": false
    },
    "cam12": {
        "fps": "30",
        "name": "head_back_fisheye",
        "publish": false
    },
    "cam14": {
        "fps": "30",
        "name": "head_depth",
        "publish": true
    },
    "cam15": {
        "fps": "30",
        "name": "head_color",
        "publish": true
    }
}
```

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

camera = agibot_gdk.Camera()

# 设置相机配置
config_path = "/home/<your_name>/.cache/agibot/app/gdk/config/r1_camera_conf.json"
try:
    camera.set_dev_camera_config(config_path)
    print("相机配置设置成功")
except RuntimeError as e:
    print(f"设置相机配置失败: {e}")

# 关闭相机
camera.close_camera()

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

##### 模式切换

```
cd ~/.cache/agibot/app/bin
./mode_switch --mode develop # 切换到develop模式
```

如需切换回之前base模式，则执行

```
./mode_switch --mode base # 切换到base模式
```

#### 8. `close_camera()`

- **功能**：关闭相机连接
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码
- **示例**：

```
import agibot_gdk

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

camera = agibot_gdk.Camera()

# 使用相机...

# 关闭相机
result = camera.close_camera()
if result == agibot_gdk.GDKRes.kSuccess:
    print("相机关闭成功")
else:
    print("相机关闭失败")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

## 使用注意事项

1. **GDK初始化**：使用Camera功能前必须先调用`agibot_gdk.gdk_init()`初始化GDK系统
2. **GDK释放**：程序结束前必须调用`agibot_gdk.gdk_release()`释放GDK系统资源
3. **超时设置**：根据实际需求设置合适的超时时间，避免长时间阻塞
4. **数据有效性**：使用前请检查返回的图像数据是否为None
5. **时间戳精度**：时间戳单位为纳秒，可用于精确的时间同步
6. **图像处理**：图像数据量较大，处理时注意内存使用
7. **相机选择**：根据应用场景选择合适的相机类型（鱼眼/立体/深度等）
8. **帧率控制**：注意相机的帧率限制，避免过度请求
9. **相机内参**：使用`get_camera_intrinsic()`获取相机内参，用于图像校正和3D重建
10. **相机配置**：使用`set_dev_camera_config()`设置相机配置时，确保配置文件路径有效
11. **资源管理**：使用完毕后调用`close_camera()`释放相机资源
12. **异常处理**：除`close_camera()`外，其他接口在失败时会抛出`RuntimeError`异常，需要适当处理。`close_camera()`返回`GDKRes`状态码，不会抛出异常
13. **相机开关**：在常规模式/develop模式下，若打开更多相机，存在性能风险

## 应用场景

- **目标检测**：利用图像数据进行目标识别和检测
- **视觉导航**：为机器人导航提供视觉信息
- **SLAM建图**：结合图像数据进行同时定位与地图构建
- **环境监控**：实时监控周围环境变化
- **深度感知**：使用深度相机获取3D环境信息
- **立体视觉**：利用双目相机进行距离测量
- **数据融合**：与其他传感器数据进行融合，提高感知精度
- **相机标定**：使用内参进行图像校正和畸变补偿
- **3D重建**：结合内参进行三维点云重建
- **视觉测量**：利用相机内参进行精确的尺寸测量
- **AR/VR应用**：基于相机内参实现增强现实和虚拟现实功能



---

<!-- source: contents_reference_Python_common -->

# Common

# GDK 通用接口文档（Python）

## 概述

GDK（Genie Development Kit）通用接口提供了系统初始化和资源释放的核心功能。通过Python接口，开发者可以方便地管理GDK系统的生命周期，确保所有GDK功能正常使用。

## 接口说明

### 1. `gdk_init()`

- **功能**：初始化GDK系统
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码

#### GDKRes状态码说明

| 状态码 | 描述 |
| --- | --- |
| `kSuccess` | 操作成功 |
| `kInvalidInput` | 输入参数无效 |
| `kInvalidOutput` | 输出参数无效 |
| `kRuntimeError` | 运行时错误 |
| `kUnknown` | 未知错误 |

- **示例**：

```
import agibot_gdk

# 初始化GDK系统
result = agibot_gdk.gdk_init()
if result == agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化成功")
else:
    print(f"GDK初始化失败，错误码: {result}")
    exit(1)
```

### 2. `gdk_release()`

- **功能**：释放GDK系统资源
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码
- **示例**：

```
import agibot_gdk

# 使用GDK功能...

# 释放GDK系统资源
result = agibot_gdk.gdk_release()
if result == agibot_gdk.GDKRes.kSuccess:
    print("GDK释放成功")
else:
    print(f"GDK释放失败，错误码: {result}")
```

### 3. `get_app_version()`

- **功能**：获取应用版本号
- **前置条件**：需先调用 `gdk_init()` 初始化 GDK 系统
- **参数**：无
- **返回值**：`str`，应用版本
- **异常**：失败时抛出 `runtime_error`
- **示例**：

```
import agibot_gdk
import time
init_result = agibot_gdk.gdk_init()
if init_result != agibot_gdk.GDKRes.kSuccess:
    print(f"GDK初始化失败，错误码: {init_result}")
    exit(1)
try:
    version = agibot_gdk.get_app_version()
    print(f"应用版本: {version}")
except RuntimeError as e:
    print(f"获取版本失败: {e}")
```

### 4. `get_robot_aid()`

- **功能**：获取机器人 AID
- **前置条件**：需先调用 `gdk_init()` 初始化 GDK 系统
- **参数**：无
- **返回值**：`str`，机器人 AID
- **异常**：失败时抛出 `runtime_error`
- **示例**：

```
import agibot_gdk
import time
init_result = agibot_gdk.gdk_init()
if init_result != agibot_gdk.GDKRes.kSuccess:
    print(f"GDK初始化失败，错误码: {init_result}")
    exit(1)
try:
    aid = agibot_gdk.get_robot_aid()
    print(f"机器人 AID: {aid}")
except RuntimeError as e:
    print(f"获取 AID 失败: {e}")
```

## 使用注意事项

1. **必须初始化**：使用任何GDK功能（包括 `get_app_version`、`get_robot_aid`）前必须先调用`gdk_init()`初始化系统
2. **必须释放**：程序结束前必须调用`gdk_release()`释放系统资源
3. **资源管理**：确保在异常情况下也能正确释放资源
4. **错误处理**：始终检查返回值，确保操作成功
5. **初始化顺序**：先初始化GDK，再创建具体的功能对象（如Camera、Robot等）
6. **释放顺序**：先关闭具体功能对象，再释放GDK系统

## 应用场景

- **系统管理**：管理GDK系统的完整生命周期
- **资源控制**：确保系统资源的正确分配和释放
- **异常处理**：在异常情况下保证系统资源的清理
- **多模块开发**：为多个GDK模块提供统一的初始化入口



---

<!-- source: contents_reference_Python_env -->

# Env

# GDK Env 接口文档（Python）

## 概述

Env（环境）模块为 G02 机器人提供统一的观测与控制接口，适用于强化学习、VLA、遥操作等需要对机器人进行 step(action) → observation 式交互的场景。Env 将相机图像、机器人状态、TF 等数据聚合为统一的 `dict` 格式观测，并支持通过 `dict` 格式动作执行轨迹控制。

## 接口说明

### Env 类

该类封装了观测获取、轨迹执行与参考系设置等功能。

#### 1. 构造函数

- **无参构造**：`Env()`
- 使用默认相机组（所有可用相机）
- **指定相机类型**：`Env(camera_types: list[CameraType])`
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `camera_types` | `list[CameraType]` | 相机类型列表；空列表表示使用默认相机组 |

- **返回值**：`Env` 实例
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

# 使用默认相机
env = agibot_gdk.Env()

# 或指定相机类型
# env = agibot_gdk.Env(camera_types=[
#     agibot_gdk.CameraType.kHeadColor,
#     agibot_gdk.CameraType.kHandLeftColor,
#     agibot_gdk.CameraType.kHandRightColor,
# ])

agibot_gdk.gdk_release()
```

#### 2. `get_observation(include_images, include_states)`

- **功能**：获取最新观测数据，返回 `dict` 格式
- **参数**：

| 参数名 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| `include_images` | `bool` | `True` | 是否包含相机图像 |
| `include_states` | `bool` | `True` | 是否包含机器人关节与状态数据 |

- **返回值**：`dict`，观测数据结构如下

**观测 dict 顶层字段**：

| 字段名 | 类型 | 描述 |
| --- | --- | --- |
| `protocol_version` | `str` | 协议版本，如 "1.1.2" |
| `timestamps` | `dict[str, int]` | 各传感器时间戳（纳秒） |
| `images` | `dict` | 图像数据，见下表 |
| `states` | `dict` | 机器人状态，见下表 |

**images 子结构**（各键为可选，相机未开启时该键不存在）：

| 键名 | 类型 | 描述 |
| --- | --- | --- |
| `head` | `dict` | 头部彩色相机 |
| `head_depth` | `dict` | 头部深度相机 |
| `hand_left` | `dict` | 左手相机 |
| `hand_right` | `dict` | 右手相机 |
| `head_left_fisheye` | `dict` | 头部左鱼眼 |
| `head_right_fisheye` | `dict` | 头部右鱼眼 |
| … | … | 其他相机键 |

**单个图像 dict 结构**：

| 字段名 | 类型 | 描述 |
| --- | --- | --- |
| `encoding` | `str` | 编码格式（如 "JPEG"） |
| `color_format` | `str` | 颜色格式（如 "RGB"） |
| `width` | `int` | 图像宽度 |
| `height` | `int` | 图像高度 |
| `image_data` | `bytes` | 图像原始字节数据 |

**states 子结构**（各键为可选，未获取到数据时该键不存在）：

**关节位置**

| 键名 | 类型 | 描述 |
| --- | --- | --- |
| `head_joint_states` | `list[float]` | 头部关节角（弧度） |
| `arm_joint_states` | `list[float]` | 双臂关节角（弧度），左右臂拼接 |
| `waist_joint_states` | `list[float]` | 腰部关节角（弧度） |
| `gripper_states` | `list[float]` | 夹爪状态 |
| `hand_joint_states` | `list[float]` | 灵巧手关节角（弧度） |
| `effector_states` | `list[float]` | 末端执行器状态 |

**关节速度 / 模式 / 力矩**

| 键名 | 类型 | 描述 |
| --- | --- | --- |
| `arm_velocity` | `list[float]` | 双臂关节速度（弧度/秒） |
| `arm_mode` | `list[int]` | 双臂关节控制模式 |
| `arm_effort` | `list[float]` | 双臂关节力矩（牛顿·米） |
| `head_velocity` | `list[float]` | 头部关节速度 |
| `head_mode` | `list[int]` | 头部关节模式 |
| `head_effort` | `list[float]` | 头部关节力矩 |
| `waist_velocity` | `list[float]` | 腰部关节速度 |
| `waist_mode` | `list[int]` | 腰部关节模式 |
| `waist_effort` | `list[float]` | 腰部关节力矩 |
| `motor_velocity` | `list[float]` | 全身电机速度（旧版，已逐步废弃） |
| `motor_mode` | `list[int]` | 全身电机模式 |
| `motor_effort` | `list[float]` | 全身电机力矩 |

**末端位姿（结构化格式）**

末端执行器位姿、六维力、六维速度均包含 `arm_base_link`（相对于机械臂基座）和 `base_link`（相对于机器人基座）两个参考系版本，每个版本下有 `left_arm` / `right_arm` 两个方向。

| 键名 | 类型 | 描述 |
| --- | --- | --- |
| `end_pose` | `dict` | 末端执行器位姿 |
| `end_wrench` | `dict` | 末端执行器六维力 |
| `end_twist` | `dict` | 末端执行器六维速度 |
| `tool_pose` | `dict` | 工具坐标系位姿 |
| `tool_wrench` | `dict` | 工具坐标系六维力 |
| `tool_twist` | `dict` | 工具坐标系六维速度 |
| `waist_pose` | `dict` | 腰部位姿 |

以 `end_pose` 为例，结构如下（`end_wrench` / `end_twist` 等结构相同）：

```
obs["states"]["end_pose"] = {
    "arm_base_link": {           # 相对于机械臂基座
        "left_arm":  {"position": [x, y, z], "orientation": [qx, qy, qz, qw]},
        "right_arm": {"position": [x, y, z], "orientation": [qx, qy, qz, qw]},
    },
    "base_link": {               # 相对于机器人基座
        "left_arm":  {"position": [x, y, z], "orientation": [qx, qy, qz, qw]},
        "right_arm": {"position": [x, y, z], "orientation": [qx, qy, qz, qw]},
    },
}
```

`end_wrench` 中每个方向为 `{"force": [fx, fy, fz], "torque": [tx, ty, tz]}`；
`end_twist` 中每个方向为 `{"linear": [vx, vy, vz], "angular": [wx, wy, wz]}`；
`waist_pose` 直接为 `{"position": [x, y, z], "orientation": [qx, qy, qz, qw]}`。

**TF**

| 键名 | 类型 | 描述 |
| --- | --- | --- |
| `tf` | `dict[str, dict]` | 坐标变换树，键为子坐标系名称（`child_frame_id`） |

每条 TF 数据结构：

```
obs["states"]["tf"]["some_frame"] = {
    "timestamp_ns": 1234567890,
    "frame_id": "base_link",
    "child_frame_id": "some_frame",
    "position": [x, y, z],
    "orientation": [qx, qy, qz, qw],
}
```

**触觉传感器（末端力传感器阵列）**

| 键名 | 类型 | 描述 |
| --- | --- | --- |
| `left_ee_force` | `dict` | 左末端触觉传感器阵列 |
| `right_ee_force` | `dict` | 右末端触觉传感器阵列 |

```
obs["states"]["left_ee_force"] = {
    "controlled": False,          # 是否处于力控模式
    "sensors": {
        "sensor_name": {
            "rows": 4,
            "cols": 4,
            "resolution_x": 0.005,    # X 方向分辨率（米）
            "resolution_y": 0.005,    # Y 方向分辨率（米）
            "normal_force":  [...],   # 法向力，长度 rows×cols（牛顿）
            "shear_force_x": [...],   # 切向力 X，长度 rows×cols
            "shear_force_y": [...],   # 切向力 Y，长度 rows×cols
            "contact": [...],         # 接触状态（1=接触），长度 rows×cols
            "valid":   [...],         # 数据有效性标记，长度 rows×cols
            "err_code": 0,            # 错误码，0 表示正常
        }
    }
}
```

**机器人位姿与状态**

| 键名 | 类型 | 描述 |
| --- | --- | --- |
| `robot_position` | `list[float]` | 机器人底盘全局位置 [x, y, z]（米） |
| `robot_orientation` | `list[float]` | 机器人底盘全局姿态 [qx, qy, qz, qw] |
| `copilot_state` | `str` | copilot 状态描述字符串 |
| `copilot_reward` | `str` | copilot reward 信息 |
| `joint_current_values` | `list[float]` | 关节电流值 |

**电机错误状态**

| 键名 | 类型 | 描述 |
| --- | --- | --- |
| `end_mode` | `list[int]` | 末端控制模式 |
| `err_code` | `list[int]` | 各关节错误码，0 表示正常 |
| `err_msg` | `list[str]` | 各关节错误描述 |

**已弃用字段**

| 键名 | 描述 |
| --- | --- |
| `end_wrench_states` | 原始 12 维六维力，已被 `end_wrench` 替代 |
| `end_velocity` | 原始 12 维末端速度，已被 `end_twist` 替代 |

- **示例**：

```
import os
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

env = agibot_gdk.Env()

# 获取完整观测
obs = env.get_observation()
print(f"协议版本: {obs['protocol_version']}")
print(f"时间戳: {obs['timestamps']}")

# 将图像保存到当前路径
for name, img in obs.get("images", {}).items():
    if img and img.get("image_data"):
        enc = img.get("encoding", "").upper()
        ext = "jpg" if enc in ("JPEG", "JPG") else "png" if enc == "PNG" else "bin"
        path = os.path.join(os.getcwd(), f"{name}.{ext}")
        with open(path, "wb") as f:
            f.write(img["image_data"])
        print(f"已保存: {path}")

# 仅获取状态，不包含图像（节省带宽）
obs_states_only = env.get_observation(include_images=False, include_states=True)

# 打印观测（obs 为纯 dict，可直接 print）
print(obs_states_only)

agibot_gdk.gdk_release()
```

#### 3. `execute_trajectory(action)`

- **功能**：执行轨迹动作
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `action` | `dict` | 轨迹动作，结构与 Action 协议对应 |

**action dict 结构**（各键为可选）：

| 键名 | 类型 | 描述 |
| --- | --- | --- |
| `trajectory_reference_time` | `float` | 从当前位置执行轨迹的参考时间（秒） |
| `left_arm` | `dict` | 左臂控制，见下表 |
| `right_arm` | `dict` | 右臂控制 |
| `head` | `list[list[float]]` | 头部关节轨迹 |
| `waist` | `dict` | 腰部控制 |
| `left_effector` | `list[list[float]]` | 左末端执行器 |
| `right_effector` | `list[list[float]]` | 右末端执行器 |
| `frame_id` | `str` | 指定目标坐标系；不填写时默认为 `base_link` |

> **安全提示：**`trajectory_reference_time` 表示从机器人当前位置执行该轨迹所使用的时间。当轨迹较长时，应适当增大该值；如果设置过小，机器人可能以过快速度执行轨迹，存在碰撞风险。

**左臂/右臂控制 dict**（`left_arm` / `right_arm`）：

| 键名 | 类型 | 描述 |
| --- | --- | --- |
| `kind` | `str` | 控制类型，如 `EEF_ABS`、`JOINT_ABS` |
| `values` | `list[list[float]]` | 控制值序列 |

- `JOINT_ABS` 表示绝对关节角控制，`values` 中的每一项为一组关节角。
- `EEF_ABS` 表示绝对末端位姿控制，`values` 中的每一项按 `[x, y, z, qx, qy, qz, qw]` 排列；前 3 项为末端位置，后 4 项为 `xyzw` 顺序的四元数。该模式仅手部控制支持（对应 `left_arm` / `right_arm`）。

**腰部控制 dict**（`waist`）：

| 键名 | 类型 | 描述 |
| --- | --- | --- |
| `kind` | `str` | 控制类型，如 "ABS\_POSE"、"ABS\_JOINT" |
| `values` | `list[list[float]]` | 控制值序列 |

> `EEF_ABS` 仅手部控制支持，腰部不支持该模式。

- **返回值**：无，失败时抛出 `std::runtime_error`
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

env = agibot_gdk.Env()

# 构造轨迹动作（双臂关节空间控制）
action = {
    "trajectory_reference_time": 1.0,
    "left_arm": {
        "kind": "JOINT_ABS",
        "values": [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]  # 左臂 7 关节
    },
    "right_arm": {
        "kind": "JOINT_ABS",
        "values": [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]  # 右臂 7 关节
    }
}

try:
    env.execute_trajectory(action)
    print("轨迹执行成功")
except Exception as e:
    print(f"轨迹执行失败: {e}")

agibot_gdk.gdk_release()
```

#### 4. `set_reference_frame_poses(l_reference_frame_name, r_reference_frame_name, l_qx, l_qy, l_qz, l_qw, r_qx, r_qy, r_qz, r_qw)`

- **功能**：设置左右末端执行器参考坐标系名称及姿态（四元数 x,y,z,w），用于后续解算
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `l_reference_frame_name` | `str` | 左臂参考坐标系名称 |
| `r_reference_frame_name` | `str` | 右臂参考坐标系名称 |
| `l_qx` | `float` | 左末端参考系四元数 x |
| `l_qy` | `float` | 左末端参考系四元数 y |
| `l_qz` | `float` | 左末端参考系四元数 z |
| `l_qw` | `float` | 左末端参考系四元数 w |
| `r_qx` | `float` | 右末端参考系四元数 x |
| `r_qy` | `float` | 右末端参考系四元数 y |
| `r_qz` | `float` | 右末端参考系四元数 z |
| `r_qw` | `float` | 右末端参考系四元数 w |

- **返回值**：无，失败时抛出 `std::runtime_error`
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

env = agibot_gdk.Env()

# 设置左右末端参考系名称及姿态（单位四元数示例）
try:
    env.set_reference_frame_poses("arm_l_link3", "arm_r_link3",
                                  0.0, 0.0, 0.0, 1.0,
                                  0.0, 0.0, 0.0, 1.0)
    print("参考系设置成功")
except Exception as e:
    print(f"参考系设置失败: {e}")

agibot_gdk.gdk_release()
```

## 使用注意事项

1. **GDK 初始化**：使用 Env 前必须先调用 `gdk_init()` 初始化 GDK 系统
2. **GDK 释放**：程序结束前必须调用 `gdk_release()` 释放资源
3. **观测格式**：`get_observation` 返回纯 Python `dict`，可直接 `print(obs)` 或序列化为 JSON
4. **动作格式**：`execute_trajectory` 接受 `dict`，与 CoRobot 协议 Action 结构一致
5. **可选字段**：观测与动作中的各字段均为可选，未提供的键不会出现在 dict 中

## 应用场景

- **强化学习**：作为 Gym-like 环境的 step 接口
- **算法验证**：快速搭建 obs–action 闭环测试



---

<!-- source: contents_reference_Python_imu -->

# IMU

# GDK [IMU](../../../concepts/sensors/#imu) 接口文档（Python）

## 概述

IMU（惯性测量单元）模块为G02机器人提供了获取实时惯性数据的功能。通过Python接口，开发者可以方便地获取机器人的方向、角速度和线性加速度信息，适用于姿态检测、运动分析、导航等多种场景。

## 接口说明

### Imu 类

该类封装了IMU传感器的主要数据获取接口。

#### 构造函数 / 初始化

- **无参构造**：`Imu()`
  启动所有 IMU
- **指定 IMU 构造**：`Imu(imu_types)`
  仅启动传入的 IMU 类型，可减少资源占用。

| 参数名 | 类型 | 必填 | 描述 |
| --- | --- | --- | --- |
| `imu_types` | `list[ImuType]` | 否 | 要启动的 IMU 类型列表；不传或传空列表时等价于启动全部 IMU |

**示例**：

```
import agibot_gdk

# 启动所有 IMU（默认）
imu = agibot_gdk.Imu()

# 仅启动前部与底盘 IMU
# imu = agibot_gdk.Imu([
#     agibot_gdk.ImuType.kImuFront,
#     agibot_gdk.ImuType.kImuChassis,
# ])
```

#### 1. `get_latest_imu()`

- **功能**：获取最新的IMU数据
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `type` | `ImuType` | IMU类型枚举值 |
| `timeout` | `float` | 超时时间（毫秒） |

- **返回值**：`ImuData`对象，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `timestamp_ns` | `int` | 数据采集的时间戳，精度为纳秒 | 纳秒 |
| `angular_velocity` | `Vector3` | 角速度，机器人在三个轴上的角速度 | 弧度/秒 |
| `linear_acceleration` | `Vector3` | 线性加速度，机器人在三个轴上的线性加速度 | 米/秒² |

#### ImuData对象详细说明

**angular\_velocity (角速度对象)**：

- `x`: X轴角速度
- `y`: Y轴角速度
- `z`: Z轴角速度

**linear\_acceleration (线性加速度对象)**：

- `x`: X轴加速度
- `y`: Y轴加速度
- `z`: Z轴加速度

**IMU类型**：
- `kImuUnknown`: 未知IMU
- `kImuFront`: 前部IMU
- `kImuBack`: 后部IMU
- `kImuChassis`: 底盘IMU
- `kImuHeadRgbd`: 头部[RGBD](../../../concepts/sensors/#rgbd) IMU

**示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

imu = agibot_gdk.Imu()

for i in range(10):
    # 获取最新IMU数据
    imu_data = imu.get_latest_imu(agibot_gdk.ImuType.kImuFront, 1000.0)

    if imu_data is not None:
        print(f"\n--- IMU数据 #{i+1} ---")
        print(f"时间戳: {imu_data.timestamp_ns}")

        # 角速度
        print(f"角速度: x={imu_data.angular_velocity.x:.4f}, "
              f"y={imu_data.angular_velocity.y:.4f}, "
              f"z={imu_data.angular_velocity.z:.4f}")
        # 线性加速度
        print(f"线性加速度: x={imu_data.linear_acceleration.x:.4f}, "
              f"y={imu_data.linear_acceleration.y:.4f}, "
              f"z={imu_data.linear_acceleration.z:.4f}")
    else:
        print(f"未收到IMU数据 #{i+1}")
    time.sleep(1.0)

# 关闭IMU
imu.close_imu()

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 2. `get_nearest_imu()`

- **功能**：获取指定时间戳附近最近的IMU数据
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `type` | `ImuType` | IMU类型枚举值 |
| `timestamp` | `int` | 目标时间戳（纳秒） |
| `timeout` | `float` | 超时时间（毫秒） |

- **返回值**：`ImuData`对象，结构与`get_latest_imu()`相同
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

imu = agibot_gdk.Imu()
imu_type = agibot_gdk.ImuType.kImuFront

# 先获取最新IMU数据
imu_data = imu.get_latest_imu(imu_type, 1000.0)

if imu_data is not None:
    # 获取历史IMU数据（往前1秒）
    imu_data_nearest = imu.get_nearest_imu(imu_type, imu_data.timestamp_ns-1000000000, 1000.0)

    if imu_data_nearest is not None:
        print(f"✅ 最近IMU数据: {imu_data_nearest.timestamp_ns}")
        print(f"角速度: x={imu_data_nearest.angular_velocity.x:.4f}, "
              f"y={imu_data_nearest.angular_velocity.y:.4f}, "
              f"z={imu_data_nearest.angular_velocity.z:.4f}")
        print(f"线性加速度: x={imu_data_nearest.linear_acceleration.x:.4f}, "
              f"y={imu_data_nearest.linear_acceleration.y:.4f}, "
              f"z={imu_data_nearest.linear_acceleration.z:.4f}")
    else:
        print(f"❌ 未找到最近的 {imu_type} 数据")

# 关闭IMU
imu.close_imu()

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 3. `get_imu_fps()`

- **功能**：获取IMU数据采集帧率
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `type` | `ImuType` | IMU类型枚举值 |

- **返回值**：`float`，IMU帧率（FPS）
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

imu = agibot_gdk.Imu()

imu_type = agibot_gdk.ImuType.kImuChassis
try:
    fps = imu.get_imu_fps(imu_type)
    print(f"IMU帧率: {fps} FPS")
except RuntimeError as e:
    print(f"获取帧率失败: {e}")

# 关闭IMU
imu.close_imu()

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 4. `get_imu_latency()`

- **注意事项**：获取IMU数据延迟统计信息前，需要先进行时间同步，否则延迟统计结果不准确
- **功能**：获取IMU数据延迟统计信息
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `type` | `ImuType` | IMU类型枚举值 |
| `window_seconds` | `float` | 统计窗口时间（秒） |

- **返回值**：`LatencyStats`对象，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `max_latency_ms` | `float` | 最大延迟 | 毫秒 |
| `avg_latency_ms` | `float` | 平均延迟 | 毫秒 |
| `p99_latency_ms` | `float` | 99分位延迟 | 毫秒 |
| `p999_latency_ms` | `float` | 99.9分位延迟 | 毫秒 |
| `p9999_latency_ms` | `float` | 99.99分位延迟 | 毫秒 |

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

imu = agibot_gdk.Imu()

imu_type = agibot_gdk.ImuType.kImuChassis

try:
    latency = imu.get_imu_latency(imu_type, 1.0)
    print("IMU延迟统计:")
    print(f"  最大延迟: {latency.max_latency_ms}ms")
    print(f"  平均延迟: {latency.avg_latency_ms}ms")
    print(f"  P99延迟: {latency.p99_latency_ms}ms")
    print(f"  P99.9延迟: {latency.p999_latency_ms}ms")
    print(f"  P99.99延迟: {latency.p9999_latency_ms}ms")
except RuntimeError as e:
    print(f"获取延迟统计失败: {e}")

# 关闭IMU
imu.close_imu()

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 5. `close_imu()`

- **功能**：关闭IMU连接
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码
- **示例**：

```
import agibot_gdk

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

imu = agibot_gdk.Imu()

# 使用IMU...

# 关闭IMU
result = imu.close_imu()
if result == agibot_gdk.GDKRes.kSuccess:
    print("IMU关闭成功")
else:
    print("IMU关闭失败")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

## 使用注意事项

1. **GDK初始化**：使用IMU功能前必须先调用`agibot_gdk.gdk_init()`初始化GDK系统
2. **GDK释放**：程序结束前必须调用`agibot_gdk.gdk_release()`释放GDK系统资源
3. **超时设置**：根据实际需求设置合适的超时时间，避免长时间阻塞
4. **数据有效性**：使用前请检查返回的IMU数据是否为None
5. **时间戳精度**：时间戳单位为纳秒，可用于精确的时间同步
6. **资源管理**：使用完毕后调用`close_imu()`释放IMU资源
7. **异常处理**：所有接口在失败时会抛出`RuntimeError`异常，需要适当处理
8. **未实现方法**：`get_imu_fps()`和`get_imu_latency()`当前未实现，使用时需注意

## 应用场景

- **运动分析**：利用角速度和线性加速度分析机器人运动状态
- **导航定位**：结合其他传感器数据进行SLAM和定位
- **平衡控制**：基于IMU数据实现机器人的平衡控制
- **数据融合**：与其他传感器数据进行融合，提高定位精度
- **振动监测**：通过加速度数据监测机器人振动状态



---

<!-- source: contents_reference_Python_interaction -->

# Interaction

# GDK 交互接口文档（Python）

## 概述

交互模块为G02机器人提供了语音交互、显示控制、音频播放等功能。通过Python接口，开发者可以方便地实现对机器人的语音控制、TTS播放、音频视频播放、显示控制等功能，适用于语音交互、多媒体展示、人机交互等多种场景。

## 注意事项

交互接口依赖跨网段通信模块，因此如果在隔离环境中使用该类接口（如docker容器），需要确保容器可以修改宿主机的网络配置。（启动容器时添加参数‘--privileged’）

## 接口说明

### Interaction 类

该类封装了机器人交互的主要功能接口。

#### 1. `set_language(language)`

- **功能**：设置语音语言
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `language` | `Language` | 语言类型，`Language.kLanguageChinese` 表示中文，`Language.kLanguageEnglish` 表示英文 |

- **返回值**：无，失败时抛出异常

**Language枚举值**：

| 枚举值 | 描述 |
| --- | --- |
| `Language.kLanguageChinese` | 中文 |
| `Language.kLanguageEnglish` | 英文 |

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

interaction = agibot_gdk.Interaction()

# 设置语言为中文
try:
    interaction.set_language(agibot_gdk.Language.kLanguageChinese)
    print("设置语言成功")
except Exception as e:
    print(f"设置语言失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
    exit(1)
print("GDK释放成功")
```

#### 2. `set_call_mode(is_on)`

- **功能**：设置通话模式
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `is_on` | `bool` | `True` 表示开启通话模式，`False` 表示关闭通话模式 |

- **返回值**：无，失败时抛出异常
- **注意事项**：
- 开启通话模式后，打开降噪模式，自动关闭唤醒状态，无法捕获asr文本
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

interaction = agibot_gdk.Interaction()

# 开启通话模式
try:
    interaction.set_call_mode(True)
    print("开启通话模式成功")
except Exception as e:
    print(f"开启通话模式失败: {e}")

# 关闭通话模式
try:
    interaction.set_call_mode(False)
    print("关闭通话模式成功")
except Exception as e:
    print(f"关闭通话模式失败: {e}")

agibot_gdk.gdk_release()
```

#### 3. `set_volume(volume)`

- **功能**：设置音量
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `volume` | `int` | 音量值，范围通常为0-100 |

- **返回值**：无，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

interaction = agibot_gdk.Interaction()

# 设置音量为50
try:
    interaction.set_volume(50)
    print("设置音量成功")
except Exception as e:
    print(f"设置音量失败: {e}")

agibot_gdk.gdk_release()
```

#### 4. `set_wakeup_switch(is_on)`

- **功能**：设置唤醒开关
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `is_on` | `bool` | `True` 表示开启唤醒，`False` 表示关闭唤醒 |

- **返回值**：无，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

interaction = agibot_gdk.Interaction()

# 开启唤醒功能
try:
    interaction.set_wakeup_switch(True)
    print("开启唤醒功能成功")
except Exception as e:
    print(f"设置唤醒开关失败: {e}")

agibot_gdk.gdk_release()
```

#### 5. `set_audio_switch(is_on)`

- **功能**：设置音频开关
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `is_on` | `bool` | `True` 表示开启音频，`False` 表示关闭音频 |

- **返回值**：无，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

interaction = agibot_gdk.Interaction()

# 开启音频功能
try:
    interaction.set_audio_switch(True)
    print("开启音频功能成功")
except Exception as e:
    print(f"设置音频开关失败: {e}")

agibot_gdk.gdk_release()
```

#### 6. `set_display_switch(is_on)`

- **功能**：设置显示开关
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `is_on` | `bool` | `True` 表示开启显示，`False` 表示关闭显示 |

- **返回值**：无，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

interaction = agibot_gdk.Interaction()

# 开启显示功能
try:
    interaction.set_display_switch(True)
    print("开启显示功能成功")
except Exception as e:
    print(f"设置显示开关失败: {e}")

agibot_gdk.gdk_release()
```

#### 7. `play_tts(text)`

- **功能**：播放TTS（文本转语音）
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `text` | `str` | 要播放的文本内容 |

- **返回值**：无，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

interaction = agibot_gdk.Interaction()

# 播放TTS
try:
    interaction.play_tts("你好，我是精灵G2")
    print("TTS播放成功")
    time.sleep(3)  # 等待播放完成
except Exception as e:
    print(f"播放TTS失败: {e}")

agibot_gdk.gdk_release()
```

#### 8. `play_audio(audio_path)`

- **功能**：播放音频文件
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `audio_path` | `str` | 音频文件路径 |

- **返回值**：无，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

interaction = agibot_gdk.Interaction()

# 播放音频文件
try:
    #音频文件需要使用官方工具配置路径，可联系运维人员获取工具，使用方法可见附录中data_push使用说明
    interaction.play_audio("/path/to/audio.wav")
    print("音频播放成功")
    time.sleep(5)  # 等待播放完成
except Exception as e:
    print(f"播放音频失败: {e}")

agibot_gdk.gdk_release()
```

#### 9. `play_video(video_path, loop_count)`

- **功能**：播放视频文件
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `video_path` | `str` | 视频文件路径 |
| `loop_count` | `int` | 循环播放次数，-1表示无限循环 |

- **返回值**：无，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

interaction = agibot_gdk.Interaction()

# 播放视频文件，循环1次
try:
    #音频文件需要使用官方工具配置路径，可联系运维人员获取工具，使用方法可见附录中data_push使用说明
    interaction.play_video("/path/to/video.mp4", 1)
    print("视频播放成功")
    time.sleep(10)  # 等待播放完成
except Exception as e:
    print(f"播放视频失败: {e}")

# 无限循环播放
try:
    interaction.play_video("/path/to/video.mp4", -1)
    print("视频开始循环播放")
except Exception as e:
    print(f"播放视频失败: {e}")

agibot_gdk.gdk_release()
```

#### 10. `get_func_status()`

- **功能**：获取语音功能状态
- **参数**：无
- **返回值**：`VoiceFuncStatus` 对象，包含以下属性：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| `func_status` | `int` | 功能状态 |
| `wakeup_status` | `int` | 唤醒状态 |
| `requester` | `str` | 请求者 |
| `wakeup_enabled` | `bool` | 唤醒功能是否启用 |
| `display_enabled` | `bool` | 显示功能是否启用 |
| `audio_enabled` | `bool` | 音频功能是否启用 |
| `en_settings` | `VoiceSettings` | 英文语音设置 |
| `cn_settings` | `VoiceSettings` | 中文语音设置 |
| `timestamp` | `int` | 时间戳（纳秒） |

**VoiceSettings对象属性**：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| `volume` | `int` | 音量 |
| `speech_rate` | `float` | 语速 |
| `voice_tone` | `str` | 音色 |
| `is_curr_setting` | `bool` | 是否为当前设置 |

- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

interaction = agibot_gdk.Interaction()

# 获取功能状态
try:
    func_status = interaction.get_func_status()
    print("功能状态信息:")
    print(f"  功能状态: {func_status.func_status}")
    print(f"  唤醒状态: {func_status.wakeup_status}")
    print(f"  请求者: {func_status.requester}")
    print(f"  唤醒功能启用: {func_status.wakeup_enabled}")
    print(f"  显示功能启用: {func_status.display_enabled}")
    print(f"  音频功能启用: {func_status.audio_enabled}")
    print(f"  中文设置 - 音量: {func_status.cn_settings.volume}, "
          f"语速: {func_status.cn_settings.speech_rate}, "
          f"音色: {func_status.cn_settings.voice_tone}")
    print(f"  英文设置 - 音量: {func_status.en_settings.volume}, "
          f"语速: {func_status.en_settings.speech_rate}, "
          f"音色: {func_status.en_settings.voice_tone}")
    print(f"  时间戳: {func_status.timestamp}")
except Exception as e:
    print(f"获取功能状态失败: {e}")

agibot_gdk.gdk_release()
```

#### 11. `get_asr_text()`

- **功能**：获取ASR（自动语音识别）文本
- **参数**：无
- **返回值**：`str`，识别到的文本内容，失败时抛出异常
- **注意事项**：
- 在唤醒状态下，且语音唤醒之后，可以获取asr文本
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

interaction = agibot_gdk.Interaction()

# 获取ASR文本
try:
    asr_text = interaction.get_asr_text()
    print(f"识别到的文本: {asr_text}")
except Exception as e:
    print(f"获取ASR文本失败: {e}")

agibot_gdk.gdk_release()
```

#### 12. `register_callback(type, callback)`

- **功能**：注册回调函数
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `type` | `str` | 回调类型 |
| `callback` | `function` | 回调函数 |

- **返回值**：无，失败时抛出异常
- **注意事项**：
- 在唤醒状态下，且语音唤醒之后，可以获取asr文本
- **当前支持的回调类型**：
- `get_asr_text`：回调函数参数类型为`str`，唤醒状态下，识别到新的语音输入时，会调用回调函数，并传递识别到的文本内容
- 其余功能回调暂不支持

#### 13. `unregister_callback(type)`

- **功能**：注销回调函数
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `type` | `str` | 回调类型 |

- **返回值**：无，失败时抛出异常
- **当前支持的回调类型**：
- `get_asr_text`：注销回调函数，注销后，唤醒状态下，识别到新的语音输入时，不会调用回调函数
- 其余功能回调暂不支持
- **示例**：

```
  import agibot_gdk
  import time

  class ASRHandler:
      def __init__(self, interaction):
          self.interaction = interaction

      def callback(self, text):
          print(f"callback text: {text}")

  if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
      print("GDK初始化失败")
      exit(1)

  interaction = agibot_gdk.Interaction()

  asr_handler = ASRHandler(interaction)

  # 注册回调函数
  try:
      interaction.register_callback("get_asr_text", asr_handler.callback)
      print("注册回调函数成功")
  except Exception as e:
      print(f"注册回调函数失败: {e}")

  try:
      while True:
          time.sleep(1)
  except KeyboardInterrupt:
      interaction.unregister_callback("get_asr_text")
      agibot_gdk.gdk_release()
```

#### 14. `init_led_strip_control()`

- **功能**：申请灯带控制权，申请成功后调用 `set_led_strips()` 下发的命令才会生效
- **参数**：无
- **返回值**：`int`，0 表示成功，失败时抛出 `RuntimeError`
- **注意事项**：
- 程序结束或不再需要控制灯带时，应调用 `release_led_strip_control()` 释放控制权
- 获取灯带控制权后，此期间整机故障、急停、低电量等状态不会通过灯带或屏幕给出默认告警提示
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

interaction = agibot_gdk.Interaction()

try:
    interaction.init_led_strip_control()
    print("已获取灯带控制权")
except RuntimeError as e:
    print(f"获取灯带控制权失败: {e}")

agibot_gdk.gdk_release()
```

#### 15. `release_led_strip_control()`

- **功能**：释放灯带控制权。
- **参数**：无
- **返回值**：`int`，0 表示成功，失败时抛出 `RuntimeError`
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

interaction = agibot_gdk.Interaction()

try:
    interaction.init_led_strip_control()
    # ... 此处下发若干 set_led_strips() 命令 ...
    interaction.release_led_strip_control()
    print("已释放灯带控制权")
except RuntimeError as e:
    print(f"操作失败: {e}")

agibot_gdk.gdk_release()
```

#### 16. `set_led_strips(led_strip_controls)`

- **功能**：批量下发灯带控制命令。共 7 路灯带可控，一次调用可独立配置 1–7 路灯带的模式、颜色和动画周期参数。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `led_strip_controls` | `list[LedStripControl]` | 灯带控制项列表，长度 1–7 |

- **返回值**：`int`，0 表示成功，失败时抛出 `RuntimeError`
- **注意事项**：
- 列表长度必须在 `[1, 7]` 之间，空列表或超过 7 项均会抛 `RuntimeError`
- 即使只控制一条灯带，也需要传入列表，例如 `interaction.set_led_strips([ctrl])`；不支持直接传 `interaction.set_led_strips(ctrl)`
- 每个控制项只对自己的 `strip_id` 生效；如需让多路灯带同时显示同一效果，逐路下发即可
- `mode` 取值非法会返回 `GDKRes::kInvalidInput`
- 在调用前需要先获取控制权

**LedStripControl对象详细说明**

单路灯带控制项。一次 `set_led_strips()` 调用最多包含 7 项（每路一条）。

| 属性名 | 类型 | 描述 | 取值范围 |
| --- | --- | --- | --- |
| `strip_id` | `LedStripId` | 灯带 ID | 见下表 |
| `mode` | `LedStripMode` | 显示模式 | 见下表 |
| `red` | `int` | 红色分量 | 0–255 |
| `green` | `int` | 绿色分量 | 0–255 |
| `blue` | `int` | 蓝色分量 | 0–255 |
| `speed` | `int` | 动画周期参数，单位 25 ms | 0–255（对应 0–6.375 s） |

**说明**：

- `speed` 仅对动效类模式（流水 / 反流水 / 三角波呼吸 / 正弦呼吸 / 闪烁）起作用；取值 0–255，对应 0–6.375 s，单位 25 ms；常亮模式可设为 0。流水 / 反流水模式下更容易观察到动画周期参数的变化；呼吸模式下变化可能不明显。

**LedStripId枚举详细说明**

灯带 ID，对应机器人本体上的 7 路灯带物理位置。

| 枚举值 | 数值 | 物理位置 |
| --- | --- | --- |
| `LedStripId.kChassisFront` | 1 | 底盘前 |
| `LedStripId.kChassisLeft` | 2 | 底盘左 |
| `LedStripId.kChassisRight` | 3 | 底盘右 |
| `LedStripId.kChestLeft` | 4 | 胸左 |
| `LedStripId.kChestRight` | 5 | 胸右 |
| `LedStripId.kBack` | 6 | 背部 |
| `LedStripId.kHead` | 7 | 头部 |

**LedStripMode枚举详细说明**

灯带显示模式。模式只对当前控制项中指定的 `strip_id` 生效；如需让所有灯带显示同一效果，逐路下发即可。

| 枚举值 | 数值 | 描述 |
| --- | --- | --- |
| `LedStripMode.kHold` | 0 | 暂停/冻结当前显示帧 |
| `LedStripMode.kSync` | 1 | 同步点亮（常亮） |
| `LedStripMode.kSequence` | 2 | 流水 |
| `LedStripMode.kReverseSequence` | 3 | 反流水 |
| `LedStripMode.kTriangleBreath` | 4 | 三角波呼吸 |
| `LedStripMode.kFlash` | 5 | 闪烁 |
| `LedStripMode.kSineBreath` | 6 | 正弦呼吸 |

**说明**：

- `kHold` 模式下 RGB 数值不起作用；后续发送其他模式可正常恢复控制。
- 需要灭灯时，请使用 `kSync + RGB=(0,0,0)`。
- 三角波呼吸和正弦呼吸对红、绿、蓝、白表现正常；对黄、品红、青等双通道复合色会退化为单通道主色（如黄 R+G → 红），不建议在呼吸模式下使用双通道复合色。
- **示例：控制 7 路灯带**

```
import time
import agibot_gdk

ALL_STRIP_IDS = [
    agibot_gdk.LedStripId.kChassisFront,
    agibot_gdk.LedStripId.kChassisLeft,
    agibot_gdk.LedStripId.kChassisRight,
    agibot_gdk.LedStripId.kChestLeft,
    agibot_gdk.LedStripId.kChestRight,
    agibot_gdk.LedStripId.kBack,
    agibot_gdk.LedStripId.kHead,
]

def make_ctrl(strip_id, mode, r=0, g=0, b=0, speed=0):
    c = agibot_gdk.LedStripControl()
    c.strip_id = strip_id
    c.mode = mode
    c.red = r
    c.green = g
    c.blue = b
    c.speed = speed
    return c

def make_uniform(mode, r=0, g=0, b=0, speed=0):
    return [make_ctrl(sid, mode, r, g, b, speed) for sid in ALL_STRIP_IDS]

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

interaction = agibot_gdk.Interaction()

try:
    interaction.init_led_strip_control()

    # 1. 7 路灯带独立配置：不同位置显示不同常亮颜色
    controls = [
        make_ctrl(agibot_gdk.LedStripId.kChassisFront, agibot_gdk.LedStripMode.kSync, 255,   0,   0),
        make_ctrl(agibot_gdk.LedStripId.kChassisLeft,  agibot_gdk.LedStripMode.kSync,   0, 255,   0),
        make_ctrl(agibot_gdk.LedStripId.kChassisRight, agibot_gdk.LedStripMode.kSync,   0,   0, 255),
        make_ctrl(agibot_gdk.LedStripId.kChestLeft,    agibot_gdk.LedStripMode.kSync, 255,   0,   0),
        make_ctrl(agibot_gdk.LedStripId.kChestRight,   agibot_gdk.LedStripMode.kSync,   0, 255,   0),
        make_ctrl(agibot_gdk.LedStripId.kBack,         agibot_gdk.LedStripMode.kSync,   0,   0, 255),
        make_ctrl(agibot_gdk.LedStripId.kHead,         agibot_gdk.LedStripMode.kSync, 255, 255, 255),
    ]
    interaction.set_led_strips(controls)
    time.sleep(3)

    # 2. 全部 7 路显示同一效果：蓝色流水
    # speed 单位为 25 ms，流水 / 反流水模式下 speed 变化最易观察
    interaction.set_led_strips(
        make_uniform(agibot_gdk.LedStripMode.kSequence, b=255, speed=50)
    )
    time.sleep(5)

    # 3. 全部 7 路熄灭：使用 kSync + RGB=(0,0,0)
    interaction.set_led_strips(
        make_uniform(agibot_gdk.LedStripMode.kSync, r=0, g=0, b=0)
    )

except RuntimeError as e:
    print(f"灯带控制失败: {e}")
finally:
    try:
        interaction.release_led_strip_control()
    except RuntimeError as e:
        print(f"释放灯带控制权失败: {e}")
    agibot_gdk.gdk_release()
```

## 使用注意事项

1. **GDK初始化**：使用Interaction功能前必须先调用`agibot_gdk.gdk_init()`初始化GDK系统
2. **GDK释放**：程序结束前必须调用`agibot_gdk.gdk_release()`释放GDK系统资源
3. **异常处理**：所有接口在失败时会抛出异常，请使用try-except进行异常处理
4. **文件路径**：播放音频和视频时，确保文件路径正确且文件存在
5. **语言设置**：设置语言后会影响TTS的语言，请根据实际需求设置
6. **音量范围**：设置音量时注意合理范围，避免过大或过小
7. **循环播放**：视频循环播放时，使用-1表示无限循环，注意及时停止
8. **状态查询**：定期查询功能状态，确保各功能正常工作
9. **ASR文本**：获取ASR文本时，需要确保语音识别功能已启用

## 应用场景

- **语音交互**：实现机器人的语音识别和语音合成功能
- **多媒体展示**：播放音频、视频内容，进行信息展示
- **人机交互**：通过语音和显示进行人机交互
- **状态监控**：监控语音功能状态，确保系统正常运行
- **多语言支持**：支持中英文切换，适应不同场景需求
- **音量控制**：动态调整音量，适应不同环境
- **功能开关**：灵活控制唤醒、音频、显示等功能开关



---

<!-- source: contents_reference_Python_lidar -->

# Lidar

# GDK Lidar 接口文档（Python）

## 概述

Lidar（激光雷达）模块为G02机器人提供了获取实时点云数据的功能。通过Python接口，开发者可以方便地获取机器人的环境感知数据，适用于SLAM建图、障碍物检测、导航避障、环境建模等多种场景。

## 接口说明

### Lidar 类

该类封装了激光雷达传感器的主要数据获取接口。

#### 构造函数 / 初始化

- **无参构造**：`Lidar()`
  启动所有激光雷达（与旧版本行为一致）。
- **指定雷达构造**：`Lidar(lidar_types)`
  仅启动传入的雷达类型

| 参数名 | 类型 | 必填 | 描述 |
| --- | --- | --- | --- |
| `lidar_types` | `list[LidarType]` | 否 | 要启动的雷达类型列表；不传或传空列表时等价于启动全部雷达 |

**示例**：

```
import agibot_gdk

# 启动所有雷达（默认）
lidar = agibot_gdk.Lidar()

# 仅启动前部雷达
# lidar = agibot_gdk.Lidar([agibot_gdk.LidarType.kLidarFront])
```

#### 1. `get_latest_pointcloud()`

- **功能**：获取最新的点云数据
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `type` | `LidarType` | 雷达类型枚举值 |
| `timeout` | `float` | 超时时间（毫秒） |

- **返回值**：`PointCloud`对象，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `timestamp_ns` | `int` | 点云数据采集的时间戳 | 纳秒 |
| `width` | `int` | 点云的宽度（点数） | 点数 |
| `height` | `int` | 点云的高度（点数） | 点数 |
| `point_step` | `int` | 每个点占用的字节数 | 字节 |
| `row_step` | `int` | 每行占用的字节数 | 字节 |
| `is_bigendian` | `bool` | 数据是否为大端序 | 布尔值 |
| `is_dense` | `bool` | 是否为密集点云（无无效点） | 布尔值 |
| `fields` | `list` | 字段信息列表，定义点云中每个点的属性结构 | 无单位 |
| `data` | `numpy.ndarray` | 点云的原始二进制数据 | 字节 |
| `data_size` | `int` | 点云原始数据的字节数 | 字节 |

#### PointCloud对象详细说明

**fields (字段信息列表)**：

每个字段包含以下属性：

- `name`: 字段名称（如"x", "y", "z", "intensity"）
- `offset`: 在点数据中的偏移量
- `datatype`: 数据类型
- `count`: 该字段的元素数量

**data (原始数据)**：

- **类型**：`numpy.ndarray`
- **描述**：点云的原始二进制数据
- **注意**：需要根据fields信息进行解析；`data_size` 为该数据的字节数

**雷达类型**：
- `kLidarUnknown`: 未知雷达
- `kLidarFront`: 前部雷达
- `kLidarBack`: 后部雷达

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

lidar = agibot_gdk.Lidar()

pointcloud = lidar.get_latest_pointcloud(agibot_gdk.LidarType.kLidarFront, 1000.0)

if pointcloud is not None:
    print(f"✅ 时间戳: {pointcloud.timestamp_ns}")
    print(f"点云尺寸: {pointcloud.width} x {pointcloud.height}")
    print(f"点步长: {pointcloud.point_step}")
    print(f"行步长: {pointcloud.row_step}")
    print(f"是否大端序: {pointcloud.is_bigendian}")
    print(f"是否密集: {pointcloud.is_dense}")
    print(f"数据大小: {pointcloud.data_size} 字节")

    # 打印字段信息
    print(f"字段数量: {len(pointcloud.fields)}")
    for j, field in enumerate(pointcloud.fields):
        print(f"  字段 {j+1}: {field.name} (偏移: {field.offset}, "
              f"类型: {field.datatype}, 数量: {field.count})")
else:
    print("未获取到点云数据")

# 关闭激光雷达
lidar.close_lidar()

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 2. `get_nearest_pointcloud()`

- **功能**：获取指定时间戳附近最近的点云数据
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `type` | `LidarType` | 雷达类型枚举值 |
| `timestamp` | `int` | 目标时间戳（纳秒） |
| `timeout` | `float` | 超时时间（毫秒） |

- **返回值**：`PointCloud`对象，结构与`get_latest_pointcloud()`相同
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

lidar = agibot_gdk.Lidar()

# 先获取最新点云数据
pointcloud = lidar.get_latest_pointcloud(agibot_gdk.LidarType.kLidarFront, 1000.0)

if pointcloud is not None:
    # 获取历史点云数据（往前1秒）
    pointcloud_nearest = lidar.get_nearest_pointcloud(
        agibot_gdk.LidarType.kLidarFront,
        pointcloud.timestamp_ns - 1000000000,
        1000.0
    )

    if pointcloud_nearest is not None:
        print(f"✅ 最近点云数据: {pointcloud_nearest.timestamp_ns}")
        print(f"点云尺寸: {pointcloud_nearest.width} x {pointcloud_nearest.height}")
        print(f"点步长: {pointcloud_nearest.point_step}")
        print(f"行步长: {pointcloud_nearest.row_step}")
        print(f"数据大小: {pointcloud_nearest.data_size} 字节")
    else:
        print("❌ 未找到最近的点云数据")
else:
    print("未获取到最新点云数据")

# 关闭激光雷达
lidar.close_lidar()

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 3. `get_lidar_fps()`

- **功能**：获取激光雷达数据采集帧率
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `type` | `LidarType` | 雷达类型枚举值 |

- **返回值**：`float`，激光雷达帧率（FPS）
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

lidar = agibot_gdk.Lidar()

lidar_type = agibot_gdk.LidarType.kLidarFront

try:
    fps = lidar.get_lidar_fps(lidar_type)
    print(f"激光雷达帧率: {fps} FPS")
except RuntimeError as e:
    print(f"获取帧率失败: {e}")

# 关闭激光雷达
lidar.close_lidar()

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 4. `get_lidar_latency()`

- **注意事项**：获取激光雷达数据延迟统计信息前，需要先进行时间同步，否则延迟统计结果不准确
- **功能**：获取激光雷达数据延迟统计信息
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `type` | `LidarType` | 雷达类型枚举值 |
| `window_seconds` | `float` | 统计窗口时间（秒） |

- **返回值**：`LatencyStats`对象，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `max_latency_ms` | `float` | 最大延迟 | 毫秒 |
| `avg_latency_ms` | `float` | 平均延迟 | 毫秒 |
| `p99_latency_ms` | `float` | 99分位延迟 | 毫秒 |
| `p999_latency_ms` | `float` | 99.9分位延迟 | 毫秒 |
| `p9999_latency_ms` | `float` | 99.99分位延迟 | 毫秒 |

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

lidar = agibot_gdk.Lidar()

lidar_type = agibot_gdk.LidarType.kLidarFront

try:
    latency = lidar.get_lidar_latency(lidar_type, 1.0)
    print("激光雷达延迟统计:")
    print(f"  最大延迟: {latency.max_latency_ms}ms")
    print(f"  平均延迟: {latency.avg_latency_ms}ms")
    print(f"  P99延迟: {latency.p99_latency_ms}ms")
    print(f"  P99.9延迟: {latency.p999_latency_ms}ms")
    print(f"  P99.99延迟: {latency.p9999_latency_ms}ms")
except RuntimeError as e:
    print(f"获取延迟统计失败: {e}")

# 关闭激光雷达
lidar.close_lidar()

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 5. `close_lidar()`

- **功能**：关闭激光雷达连接
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码
- **示例**：

```
import agibot_gdk

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

lidar = agibot_gdk.Lidar()

# 使用激光雷达...

# 关闭激光雷达
result = lidar.close_lidar()
if result == agibot_gdk.GDKRes.kSuccess:
    print("激光雷达关闭成功")
else:
    print("激光雷达关闭失败")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

## 使用注意事项

1. **GDK初始化**：使用激光雷达功能前必须先调用`agibot_gdk.gdk_init()`初始化GDK系统
2. **GDK释放**：程序结束前必须调用`agibot_gdk.gdk_release()`释放GDK系统资源
3. **超时设置**：根据实际需求设置合适的超时时间，避免长时间阻塞
4. **数据有效性**：使用前请检查返回的点云数据是否为None
5. **时间戳精度**：时间戳单位为纳秒，可用于精确的时间同步
6. **点云处理**：点云数据量较大，处理时注意内存使用
7. **雷达选择**：根据应用场景选择合适的雷达类型（前部/后部）
8. **资源管理**：使用完毕后调用`close_lidar()`释放激光雷达资源
9. **异常处理**：所有接口在失败时会抛出`RuntimeError`异常，需要适当处理
10. **未实现方法**：`get_lidar_fps()`和`get_lidar_latency()`当前未实现，使用时需注意

## 应用场景

- **SLAM建图**：利用点云数据进行同时定位与地图构建
- **障碍物检测**：实时检测环境中的障碍物
- **导航避障**：为机器人导航提供环境感知信息
- **环境建模**：构建3D环境模型
- **目标识别**：结合点云数据进行目标检测和识别
- **路径规划**：基于点云数据规划安全路径
- **数据融合**：与其他传感器数据进行融合，提高感知精度



---

<!-- source: contents_reference_Python_map -->

# Map

# GDK Map 接口文档（Python）

## 概述

Map（地图）模块为G02机器人提供了地图管理功能。通过Python接口，开发者可以方便地获取、切换、管理机器人构建的地图，适用于地图存储、地图切换、地图管理等多种场景。

## 接口说明

### Map 类

该类封装了地图管理的主要功能接口。

#### 1. `get_curr_map()`

- **功能**：获取当前使用的地图信息
- **参数**：无
- **返回值**：`MapName`对象，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `id` | `int` | 地图ID | 整数 |
| `name` | `str` | 地图名称 | 字符串 |
| `is_curr_map` | `bool` | 是否为当前地图 | 布尔值 |

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

map_manager = agibot_gdk.Map()

# 获取当前地图
current_map = map_manager.get_curr_map()
print(f"当前地图:")
print(f"  地图ID: {current_map.id}")
print(f"  地图名称: {current_map.name}")
print(f"  是否为当前地图: {current_map.is_curr_map}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 2. `get_map()`

- **功能**：根据地图ID获取地图详细信息（含栅格地图、点云、墙壁、引导点等）
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `map_id` | `int` | 地图ID（0-255） |

- **返回值**：`MapInfo`对象，包含以下主要属性：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| `id` | `int` | 地图ID |
| `name` | `str` | 地图名称 |
| `gravity` | `Vector3` | 重力向量 |
| `grid_map` | `OccupancyGrid` | 栅格地图（含 `width`、`height`、`resolution`、`origin`、`data`） |
| `walls` | `list` | 墙壁点列表 |
| `infeasible_areas` | `list` | 不可行区域 |
| `guide_pts` | `list` | 引导点列表 |

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

map_manager = agibot_gdk.Map()

# 获取当前地图ID后，获取完整地图信息
current_map = map_manager.get_curr_map()
map_info = map_manager.get_map(current_map.id)

print(f"地图名称: {map_info.name}, ID: {map_info.id}")
print(f"栅格尺寸: {map_info.grid_map.width} x {map_info.grid_map.height}")
print(f"分辨率: {map_info.grid_map.resolution}")
print(f"原点: ({map_info.grid_map.origin.position.x}, {map_info.grid_map.origin.position.y})")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 3. `get_all_map()`

- **功能**：获取所有可用地图列表
- **参数**：无
- **返回值**：`list[MapName]`，地图名称列表
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

map_manager = agibot_gdk.Map()

# 获取所有地图
all_maps = map_manager.get_all_map()
print(f"所有地图数量: {len(all_maps)}")

for i, map_name in enumerate(all_maps):
    print(f"地图 {i+1}:")
    print(f"  ID: {map_name.id}")
    print(f"  名称: {map_name.name}")
    print(f"  是否为当前地图: {map_name.is_curr_map}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 4. `switch_map()`

- **功能**：切换到指定地图
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `map_id` | `int` | 目标地图ID |

- **返回值**：无（成功时无返回值，失败时抛出异常）
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

map_manager = agibot_gdk.Map()

try:
    # 切换到指定地图
    map_manager.switch_map(1)
    print("地图切换成功")

    # 验证切换结果
    current_map = map_manager.get_curr_map()
    print(f"当前地图ID: {current_map.id}")
    print(f"当前地图名称: {current_map.name}")
except Exception as e:
    print(f"地图切换失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 5. `remove_map()`

- **功能**：删除指定地图
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `map_id` | `int` | 要删除的地图ID |

- **返回值**：无（成功时无返回值，失败时抛出异常）
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

map_manager = agibot_gdk.Map()

try:
    # 删除指定地图
    map_manager.remove_map(2)
    print("地图删除成功")

    # 验证删除结果
    all_maps = map_manager.get_all_map()
    print(f"剩余地图数量: {len(all_maps)}")
except Exception as e:
    print(f"地图删除失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

## 使用注意事项

1. **GDK初始化**：使用地图功能前必须先调用`agibot_gdk.gdk_init()`初始化GDK系统
2. **GDK释放**：程序结束前必须调用`agibot_gdk.gdk_release()`释放GDK系统资源
3. **地图ID**：确保使用有效的地图ID，避免访问不存在的地图
4. **地图切换**：切换地图时确保当前没有正在进行的导航任务
5. **地图删除**：删除地图前确认该地图不再需要，删除操作不可逆
6. **存储空间**：注意地图文件占用的存储空间，及时清理不需要的地图
7. **异常处理**：所有接口在失败时会抛出`RuntimeError`异常，需要适当处理
8. **数据类型**：地图ID使用`int`类型，范围为0-255
9. **时间戳精度**：时间戳单位为纳秒，可用于精确的时间同步

## 应用场景

- **地图管理**：管理机器人构建的多个地图
- **环境切换**：在不同工作环境间切换地图
- **地图备份**：保存和恢复重要地图数据
- **存储优化**：清理不需要的地图以节省存储空间
- **环境感知**：利用地图中的墙壁和不可行区域信息
- **引导点导航**：使用地图中的引导点进行精确导航



---

<!-- source: contents_reference_Python_pnc -->

# PNC

# GDK PNC 接口文档（Python）

## 概述

PNC（Planning and Control，规划与控制）模块为G02机器人提供了路径规划和导航控制功能。通过Python接口，开发者可以方便地实现机器人的自主导航、路径规划、任务状态管理等功能，适用于自主导航、路径规划、任务调度等多种场景。

## 接口说明

### Pnc 类

该类封装了机器人路径规划和导航控制的主要接口。

#### 1. `get_task_state()`

- **功能**：获取当前任务状态
- **参数**：无
- **返回值**：`PNCTaskState`对象，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `state` | `int` | 任务状态码 | 整数 |
| `message` | `str` | 状态描述信息 | 字符串 |
| `id` | `int` | 任务ID | 整数 |
| `type` | `int` | 任务类型 | 整数 |

**任务状态码说明**：
- `0`: 空闲
- `1`: 启动中
- `2`: 运行中
- `3`: 暂停中
- `4`: 已暂停
- `5`: 恢复中
- `6`: 取消中
- `7`: 已取消
- `8`: 失败
- `9`: 成功

**任务类型说明**：
- `0`: 空闲
- `1`: 点对点导航
- `2`: 高精度导航
- `3`: 远程控制
- `4`: 相对移动导航
- `5`: 路径规划导航

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

pnc = agibot_gdk.Pnc()

# 获取任务状态
task_state = pnc.get_task_state()
print(f"PNC任务状态: {task_state.state}")
print(f"PNC任务ID: {task_state.id}")
print(f"PNC任务内容: {task_state.message}")
print(f"PNC任务种类: {task_state.type}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 2. `normal_navi()`

- **功能**：执行正常导航到指定目标点, 执行之前需要在G02 Pad上进行重定位
- **参数**：`NaviReq`对象(map坐标系下)，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `target.position.x` | `float` | 目标位置X坐标 | 米 |
| `target.position.y` | `float` | 目标位置Y坐标 | 米 |
| `target.position.z` | `float` | 目标位置Z坐标 | 米 |
| `target.orientation.x` | `float` | 目标方向四元数X分量 | 无单位 |
| `target.orientation.y` | `float` | 目标方向四元数Y分量 | 无单位 |
| `target.orientation.z` | `float` | 目标方向四元数Z分量 | 无单位 |
| `target.orientation.w` | `float` | 目标方向四元数W分量 | 无单位 |

- **返回值**：无，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

pnc = agibot_gdk.Pnc()

# 创建导航目标
target = agibot_gdk.NaviReq()
target.target.position.x = -0.14157561800950003
target.target.position.y = 0.015152394013126735
target.target.position.z = 0.0040338473100211417
target.target.orientation.x = 0.01146358383671978
target.target.orientation.y = -0.01720065085681078
target.target.orientation.z = 0.83847410642918951
target.target.orientation.w = 0.54454926012574167

# 执行导航
try:
    pnc.normal_navi(target)
    print("正常导航请求发送成功")
except Exception as e:
    print(f"正常导航失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 3. `high_precision_navi()` (该接口暂未上线)

- **功能**：执行高精度导航到指定目标点, 执行之前需要在G02 Pad上进行重定位
- **参数**：`NaviReq`对象(map坐标系下)
- **返回值**：无，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

pnc = agibot_gdk.Pnc()

# 创建高精度导航目标
target = agibot_gdk.NaviReq()
target.target.position.x = 1.0
target.target.position.y = 2.0
target.target.position.z = 0.0
target.target.orientation.x = 0.0
target.target.orientation.y = 0.0
target.target.orientation.z = 0.0
target.target.orientation.w = 1.0

# 执行高精度导航
try:
    pnc.high_precision_navi(target)
    print("高精度导航请求发送成功")
except Exception as e:
    print(f"高精度导航失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 4. `relative_move()`

- **功能**：执行小范围平移, 简单停障，无避障
- **参数**：`NaviReq`对象(base\_link坐标系下)
- **返回值**：无，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

pnc = agibot_gdk.Pnc()

# 创建相对移动目标
target = agibot_gdk.NaviReq()
target.target.position.x = 0.5  # 相对前进0.5米
target.target.position.y = 0.0
target.target.position.z = 0.0
target.target.orientation.x = 0.0
target.target.orientation.y = 0.0
target.target.orientation.z = 0.0
target.target.orientation.w = 1.0

# 执行相对移动
try:
    pnc.relative_move(target)
    print("相对移动请求发送成功")
except Exception as e:
    print(f"相对移动失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 5. `cancel_task()`

- **功能**：取消指定ID的导航任务
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `task_id` | `int` | 要取消的任务ID |

- **返回值**：无，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

pnc = agibot_gdk.Pnc()

# 先获取当前任务状态以获取任务ID
try:
    task_state = pnc.get_task_state()
    task_id = task_state.id

    # 取消指定ID的导航任务
    pnc.cancel_task(task_id)
    print("取消任务请求发送成功")
except Exception as e:
    print(f"取消任务失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 6. `pause_task()`

- **功能**：暂停指定ID的导航任务
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `task_id` | `int` | 要暂停的任务ID |

- **返回值**：无，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

pnc = agibot_gdk.Pnc()

# 先获取当前任务状态以获取任务ID
try:
    task_state = pnc.get_task_state()
    task_id = task_state.id

    # 暂停指定ID的任务
    pnc.pause_task(task_id)
    print("暂停任务请求发送成功")
except Exception as e:
    print(f"暂停任务失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 7. `resume_task()`

- **功能**：恢复指定ID的导航任务
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `task_id` | `int` | 要恢复的任务ID |

- **返回值**：无，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

pnc = agibot_gdk.Pnc()

# 先获取当前任务状态以获取任务ID
try:
    task_state = pnc.get_task_state()
    task_id = task_state.id

    # 恢复指定ID的任务
    pnc.resume_task(task_id)
    print("恢复任务请求发送成功")
except Exception as e:
    print(f"恢复任务失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 8. `request_chassis_control()`

- **功能**：请求底盘控制权限，用于远程控制模式
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `control_mode` | `int` | 控制请求：0 |

**控制模式说明**：
- **阿克曼模式**：适用于前轮转向的车辆运动学模型，通过线速度(`linear.x`)和角速度(`angular.z`)控制
- **蟹行模式**：支持全向移动，通过`linear.x`和`linear.y`分别控制前后和左右移动
- **返回值**：无，失败时抛出异常

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

pnc = agibot_gdk.Pnc()

# 请求底盘控制权限
try:
    pnc.request_chassis_control(0)
    print("底盘控制权限请求发送成功")
except Exception as e:
    print(f"底盘控制权限请求失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 9. `move_chassis()`

- **功能**：移动底盘，用于远程控制模式下的底盘运动控制
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `twist` | `Twist` | 速度指令对象，包含线速度和角速度 |

- **返回值**：无，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

pnc = agibot_gdk.Pnc()

# 请求底盘控制权限
try:
    pnc.request_chassis_control(0)
    print("底盘控制权限请求发送成功")
except Exception as e:
    print(f"底盘控制权限请求失败: {e}")

# 创建速度指令(阿克曼行走)
twist = agibot_gdk.Twist()
twist.linear.x = 0.5  # 线速度0.5米/秒
twist.angular.z = 0.1  # 角速度0.1弧度/秒

# 创建速度指令（蟹行行走）
#twist.linear.y = 0.5; // 向左平移速度 0.5 m/s

# 移动底盘
try:
    for i in range(10):
        pnc.move_chassis(twist)
        print(f"底盘移动请求发送成功 ({i+1}/10)")
        time.sleep(0.1)
except Exception as e:
    print(f"底盘移动失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

## 使用注意事项

1. **GDK初始化**：使用PNC功能前必须先调用`agibot_gdk.gdk_init()`初始化GDK系统
2. **GDK释放**：程序结束前必须调用`agibot_gdk.gdk_release()`释放GDK系统资源
3. **目标设置**：确保目标位置在机器人可达范围内
4. **坐标系**：注意使用正确的坐标系（通常是机器人本体坐标系）
5. **任务状态**：及时检查任务状态，处理可能的错误情况
6. **安全考虑**：在导航过程中注意周围环境安全
7. **异常处理**：所有接口在失败时会抛出`RuntimeError`异常，需要适当处理
8. **资源管理**：在程序退出前确保调用`cancel_task()`停止正在进行的任务

## 应用场景

- **自主导航**：实现机器人在环境中的自主移动
- **路径规划**：规划从起点到终点的最优路径
- **任务调度**：管理多个导航任务的执行
- **位置控制**：精确控制机器人到达指定位置和姿态
- **避障导航**：在动态环境中实现安全导航
- **高精度导航**：需要精确定位的应用场景
- **相对移动**：基于当前位置的相对位移控制
- **任务控制**：动态暂停、恢复或取消导航任务



---

<!-- source: contents_reference_Python_robot -->

# Robot

# GDK 机器人控制接口文档（Python）

## 概述

机器人控制模块为G02机器人提供了获取关节状态、控制机器人运动、获取全身状态等功能。通过Python接口，开发者可以方便地实现对机器人运动的精细化控制，适用于基础控制、状态检测、动作录制回放等多种场景。

## 关于运动控制模式的说明

机器人双臂的运动控制接口支持不同的\*\*运动控制模式\*\*：`1`（关节位控/位置控制）、`2`（笛卡尔阻抗控制）、`3`（关节阻抗控制）。机器人启动后默认处于力控模式；使用其他控制模式时，需通过`set_control_mode()`切换。

机器人的运动控制模式是持久的，即一旦切换（无论是通过GDK还是通过其他方式）后会一直保持，直到再次切换或机器人重启。调用相关控制接口前，GDK 会检查机器人\*\*当前实际\*\*的运动控制模式是否在该接口支持的模式列表中，若不在则会抛出运行时异常，并在日志中打印原因。各接口的支持情况请参考下方"支持的控制模式"说明。

### 力控使用工作流

笛卡尔/关节力控（阻抗控制）依赖于准确的动力学模型，控制器会基于"机器人本体 + 末端负载"的整体动力学进行重力补偿与柔顺跟踪。若末端实际负载与配置不一致，会导致力反馈失真、柔顺跟踪偏移、甚至触发误报警。建议按以下顺序进入力控：

1. **设置负载**（仅当末端确实存在负载时）：通过 `set_payload()` 配置末端负载的质量、质心、惯性张量。空载场景应保持/重置为零负载，详见 [`set_payload()`](#30-set_payload) 中的"使用建议"。
2. **传感器零点标定**（持久负载场景）：负载长期挂载（如固定工具、夹爪上常驻物）时，设置完 `set_payload()` 后调用一次 `torque_sensor_calibration()`，补偿传感器零点偏置。短期搬运类任务无需每次标定。
3. **切换控制模式**：`set_control_mode(2)` 进入笛卡尔力控，或 `set_control_mode(3)` 进入关节力控。
4. **配置阻抗参数**：进入对应控制模式后，再调用 `set_cartesian_impedance()` 或 `set_joint_impedance()` 设置参数；仅 `kp` 字段当前生效。
5. **下发目标位姿/关节量**：使用 `end_effector_pose_control()`、`joint_control()` 等接口下发目标，底层按已配置的阻抗参数柔顺跟踪。

### 笛卡尔阻抗 vs 关节阻抗

两类阻抗都用于实现"柔顺跟踪 + 力反馈"，区别在于刚度/柔顺性是定义在哪个空间，进而影响控制目标的下发形式与适合的任务类型：

| 维度 | 笛卡尔阻抗（`control_mode=2`） | 关节阻抗（`control_mode=3`） |
| --- | --- | --- |
| **刚度定义空间** | 末端笛卡尔空间（`x/y/z/roll/pitch/yaw` 6 维） | 关节空间（每个关节单独配置，单臂 7 维） |
| **执行轨迹** | 目标位姿直接在笛卡尔空间执行，不经过全身逆解 | 目标关节量在关节空间执行；如下发的是位姿，会先经全身逆解再到关节空间柔顺跟踪 |
| **适合任务** | 接触类任务、装配、贴合、沿笛卡尔某轴方向柔顺（如 z 向贴合） | 拖拽示教、关节级柔顺、对单关节刚度有差异化要求的任务 |
| **可观察柔顺性方向** | 在笛卡尔轴上更直观（如降低 z 向 `kp` 让末端在垂直方向更"软"） | 在关节维度上更直观（如降低肘关节 `kp` 让肘部更"软"） |
| **配置接口** | [`set_cartesian_impedance()`](#24-set_cartesian_impedance) | [`set_joint_impedance()`](#28-set_joint_impedance) |
| **`kp` 维度** | 单臂 6 维：`x/y/z/roll/pitch/yaw` | 单臂 7 维：关节 1~7 |

**共同要点**：

- 设置阻抗参数前，必须先切换到对应控制模式：笛卡尔阻抗参数需先 `set_control_mode(2)`，关节阻抗参数需先 `set_control_mode(3)`。
- 当前实现仅 `kp` 字段生效（刚度系数），`ki` / `kv` 暂未启用。
- 重启后参数会丢失，每次需重新下发；详见 [`torque_sensor_calibration()`](#31-torque_sensor_calibration) 的"持久化说明"。
- 两种阻抗模式互斥：同一时刻只能处于其中一种，切换需重新调用 `set_control_mode()`。
- 进入阻抗模式前请先按"力控使用工作流"配置好负载，避免因动力学不匹配导致柔顺跟踪偏移。

**注意事项**：

- 控制关节角度的相关接口，支持位控模式和关节阻抗模式；
- 控制末端位姿势的相关接口，支持位控模式，笛卡尔阻抗模式和关节阻抗模式；
- 在不同的控制模式下控制末端位姿时，需要使用不同的参考坐标系，详细可参考end\_effector\_pose\_control()的使用说明。

## 接口说明

### Robot 类

该类封装了机器人控制的主要功能接口。

#### 1. `get_joint_states()`

- **注意事项**：使用motor\_position和motor\_velocity获取电机位置和速度，position和velocity为低速电机预留字段，当前版本无需关注
- **功能**：获取关节状态信息
- **参数**：无
- **返回值**：`dict`，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `timestamp` | `int` | 时间戳 | 纳秒 |
| `nums` | `int` | 关节数量 | 整数 |
| `states` | `list[dict]` | 关节状态列表 | 字典列表 |

**states中每个关节状态结构**：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `name` | `str` | 关节名称 | 字符串 |
| `mode` | `int` | 关节模式 | 整数 |
| `position` | `float` | 关节位置 | 弧度 |
| `velocity` | `float` | 关节速度 | 弧度/秒 |
| `effort` | `float` | 关节力矩 | 牛顿·米 |
| `motor_position` | `float` | 电机位置 | 弧度 |
| `motor_velocity` | `float` | 电机速度 | 弧度/秒 |
| `motor_current` | `float` | 电机电流 | 安培 |
| `error_code` | `int` | 错误码 | 整数 |

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

robot = agibot_gdk.Robot()

# 获取关节状态
joint_states = robot.get_joint_states()
print(f"关节数量: {joint_states['nums']}")
print(f"时间戳: {joint_states['timestamp']}")

for state in joint_states['states']:
    print(f"关节: {state['name']}")
    print(f"  位置: {state['position']:.3f} 弧度")
    print(f"  速度: {state['velocity']:.3f} 弧度/秒")
    print(f"  力矩: {state['effort']:.3f} 牛顿·米")
    print(f"  电机位置: {state['motor_position']:.3f} 弧度")
    print(f"  电机电流: {state['motor_current']:.3f} 安培")
    print(f"  错误码: {state['error_code']}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 2. `get_joint_limits()`

- **功能**：获取机器人关节限位信息
- **参数**：无
- **返回值**：`dict[str, dict]`，键为关节名，值为关节限位信息字典，结构如下：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `min` | `float` | 关节最小位置限位 | 弧度 |
| `max` | `float` | 关节最大位置限位 | 弧度 |

- **说明**：当前返回结果包含机器人本体关节与末端执行器关节（`left_tool`/`right_tool`）的限位信息。

G02 各关节限位参考值（单位：弧度）：

| 关节名 | 分组 | 最小值 | 最大值 |
| --- | --- | --- | --- |
| `idx01_body_joint1` | 躯干 | -1.0821 | 0.0002 |
| `idx02_body_joint2` | 躯干 | -0.0002 | 2.6529 |
| `idx03_body_joint3` | 躯干 | -1.9199 | 1.5710 |
| `idx04_body_joint4` | 躯干 | -0.4363 | 0.4363 |
| `idx05_body_joint5` | 躯干 | -3.0456 | 3.0456 |
| `idx11_head_joint1` | 头部 | -1.5710 | 1.5710 |
| `idx12_head_joint2` | 头部 | -0.3492 | 0.3492 |
| `idx13_head_joint3` | 头部 | -0.5348 | 0.5348 |
| `idx21_arm_l_joint1` | 左臂 | -3.0718 | 3.0718 |
| `idx22_arm_l_joint2` | 左臂 | -2.0595 | 2.0595 |
| `idx23_arm_l_joint3` | 左臂 | -3.0718 | 3.0718 |
| `idx24_arm_l_joint4` | 左臂 | -2.4958 | 1.0123 |
| `idx25_arm_l_joint5` | 左臂 | -3.0718 | 3.0718 |
| `idx26_arm_l_joint6` | 左臂 | -1.0123 | 1.0123 |
| `idx27_arm_l_joint7` | 左臂 | -1.5359 | 1.5359 |
| `idx61_arm_r_joint1` | 右臂 | -3.0718 | 3.0718 |
| `idx62_arm_r_joint2` | 右臂 | -2.0595 | 2.0595 |
| `idx63_arm_r_joint3` | 右臂 | -3.0718 | 3.0718 |
| `idx64_arm_r_joint4` | 右臂 | -2.4958 | 1.0123 |
| `idx65_arm_r_joint5` | 右臂 | -3.0718 | 3.0718 |
| `idx66_arm_r_joint6` | 右臂 | -1.0123 | 1.0123 |
| `idx67_arm_r_joint7` | 右臂 | -1.5359 | 1.5359 |

- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

robot = agibot_gdk.Robot()

joint_limits = robot.get_joint_limits()
for name, limit in joint_limits.items():
    print(f"{name}: [{limit['min']}, {limit['max']}]")

agibot_gdk.gdk_release()
```

#### 3. `get_whole_body_status()`

- **功能**：获取全身状态信息
- **参数**：无
- **返回值**：`dict`，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `timestamp` | `int` | 时间戳 | 纳秒 |
| `right_arm_error` | `int` | 右臂错误码 | 整数 |
| `left_arm_error` | `int` | 左臂错误码 | 整数 |
| `right_arm_control` | `bool` | 右臂控制状态 | 布尔值 |
| `left_arm_control` | `bool` | 左臂控制状态 | 布尔值 |
| `right_arm_estop` | `bool` | 右臂急停状态 | 布尔值 |
| `left_arm_estop` | `bool` | 左臂急停状态 | 布尔值 |
| `right_end_error` | `int` | 右执行器错误码 | 整数 |
| `left_end_error` | `int` | 左执行器错误码 | 整数 |
| `right_end_model` | `str` | 右执行器型号 | 字符串 |
| `left_end_model` | `str` | 左执行器型号 | 字符串 |
| `waist_error` | `int` | 腰部错误码 | 整数 |
| `lift_error` | `int` | 升降错误码 | 整数 |
| `neck_error` | `int` | 头部错误码 | 整数 |
| `chassis_error` | `int` | 底盘错误码 | 整数 |

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

robot = agibot_gdk.Robot()

# 获取全身状态
status = robot.get_whole_body_status()

# 打印基本信息
print(f"时间戳: {status['timestamp']}")
print(f"右执行器型号: {status['right_end_model']}")
print(f"左执行器型号: {status['left_end_model']}")

# 检查错误状态
print("\n=== 错误状态检查 ===")
if status['right_arm_error'] == 0:
    print("✅ 右臂正常")
else:
    print(f"❌ 右臂错误码: {status['right_arm_error']}")

if status['left_arm_error'] == 0:
    print("✅ 左臂正常")
else:
    print(f"❌ 左臂错误码: {status['left_arm_error']}")

if status['waist_error'] == 0:
    print("✅ 腰部正常")
else:
    print(f"❌ 腰部错误码: {status['waist_error']}")

if status['neck_error'] == 0:
    print("✅ 头部正常")
else:
    print(f"❌ 头部错误码: {status['neck_error']}")

if status['chassis_error'] == 0:
    print("✅ 底盘正常")
else:
    print(f"❌ 底盘错误码: {status['chassis_error']}")

# 检查控制状态
print("\n=== 控制状态 ===")
print(f"右臂控制: {'是' if status['right_arm_control'] else '否'}")
print(f"左臂控制: {'是' if status['left_arm_control'] else '否'}")
print(f"右臂急停: {'是' if status['right_arm_estop'] else '否'}")
print(f"左臂急停: {'是' if status['left_arm_estop'] else '否'}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 4. `get_motion_control_status()`

- **功能**：获取末端运动控制状态
- **参数**：无
- **返回值**：`MotionControlStatus`对象，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `mode` | `int` | 运动模式，0为停止，1为伺服，2为规划 | 无单位 |
| `control_mode` | `int` | 控制模式，1为关节位控（PD/轨迹），2为笛卡尔阻抗，3为关节阻抗 | 无单位 |
| `error_code` | `int` | 错误码，0表示正常 | 无单位 |
| `error_msg` | `str` | 错误信息 | 字符串 |
| `frame_names` | `list[str]` | 末端关节名称列表 | 无单位 |
| `frame_poses` | `list[Pose]` | 末端关节位姿列表 | 米 |
| `twists` | `list[Twist]` | 末端关节速度列表 | 米/秒 |
| `wrenches` | `list[Wrench]` | 末端关节力矩列表 | 牛米（torque）、牛（force） |
| `arm_disturbance_force` | `list[float]` | 各关节所受外力 | 牛米 |
| `collision_pairs_1` | `list[str]` | 碰撞对列表1 | 无单位 |
| `collision_pairs_2` | `list[str]` | 碰撞对列表2 | 无单位 |

**碰撞对说明**：
碰撞对是指当前状态下发生碰撞的关节对，如有碰撞会在列表中注明碰撞信息（碰撞的link（urdf的link）），否则为空。

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

robot = agibot_gdk.Robot()

# 获取运动控制状态
status = robot.get_motion_control_status()

print("=== 运动控制状态 ===")
print(f"运动模式(mode): {status.mode}")
print(f"控制模式(control_mode): {status.control_mode}")
print(f"错误码(error_code): {status.error_code}")
print(f"错误信息(error_msg): {status.error_msg}")

print("\n=== 末端关节名称(frame_names) ===")
print(f"数量: {len(status.frame_names)}")
for i, frame_name in enumerate(status.frame_names):
    print(f"  [{i}] {frame_name}")

print("\n=== 末端关节位姿(frame_poses) ===")
print(f"数量: {len(status.frame_poses)}")
for i, pose in enumerate(status.frame_poses):
    frame_name = status.frame_names[i] if i < len(status.frame_names) else f"frame_{i}"
    print(f"  [{i}] {frame_name}:")
    print(f"    position: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}")
    print(f"    orientation: x={pose.orientation.x}, y={pose.orientation.y}, "
          f"z={pose.orientation.z}, w={pose.orientation.w}")

print("\n=== 末端关节速度(twists) ===")
print(f"数量: {len(status.twists)}")
for i, twist in enumerate(status.twists):
    frame_name = status.frame_names[i] if i < len(status.frame_names) else f"frame_{i}"
    print(f"  [{i}] {frame_name}:")
    print(f"    linear: x={twist.linear.x}, y={twist.linear.y}, z={twist.linear.z}")
    print(f"    angular: x={twist.angular.x}, y={twist.angular.y}, z={twist.angular.z}")

print("\n=== 末端关节力/力矩(wrenches) ===")
print(f"数量: {len(status.wrenches)}")
for i, wrench in enumerate(status.wrenches):
    frame_name = status.frame_names[i] if i < len(status.frame_names) else f"frame_{i}"
    print(f"  [{i}] {frame_name}:")
    print(f"    force: x={wrench.force.x}, y={wrench.force.y}, z={wrench.force.z}")
    print(f"    torque: x={wrench.torque.x}, y={wrench.torque.y}, z={wrench.torque.z}")

print("\n=== 各关节所受外力(arm_disturbance_force) ===")
print(f"数量: {len(status.arm_disturbance_force)}")
for i, force in enumerate(status.arm_disturbance_force):
    print(f"  [{i}] {force}")

print("\n=== 碰撞对(collision_pairs_1 / collision_pairs_2) ===")
print(f"碰撞对数量: {len(status.collision_pairs_1)}")
for i, (link_1, link_2) in enumerate(zip(status.collision_pairs_1, status.collision_pairs_2)):
    print(f"  [{i}] {link_1} <-> {link_2}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 5. `get_end_state()`

- **功能**：获取末端执行器状态信息
- **参数**：无
- **返回值**：`dict`，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `left_end_state` | `dict` | 左执行器状态信息 | 字典 |
| `right_end_state` | `dict` | 右执行器状态信息 | 字典 |

**left\_end\_state/right\_end\_state结构**：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `controlled` | `bool` | 是否被控制 | 布尔值 |
| `type` | `int` | 执行器类型 | 整数 |
| `names` | `list[str]` | 关节名称列表 | 字符串列表 |
| `end_states` | `list[dict]` | 关节状态列表 | 字典列表 |

**end\_states中每个关节状态结构**：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `id` | `int` | 关节ID | 整数 |
| `enable` | `bool` | 是否启用 | 布尔值 |
| `position` | `float` | 关节位置 | 弧度 |
| `velocity` | `float` | 关节速度 | 弧度/秒 |
| `effort` | `float` | 关节力矩 | 牛顿·米 |
| `current` | `float` | 电机电流 | 安培 |
| `voltage` | `float` | 电机电压 | 伏特 |
| `temperature` | `float` | 电机温度 | 摄氏度 |
| `status` | `int` | 状态码 | 整数 |
| `err_code` | `int` | 错误码 | 整数 |

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

robot = agibot_gdk.Robot()

# 获取末端执行器状态
end_state = robot.get_end_state()

# 打印左执行器状态
left_state = end_state['left_end_state']
print(f"左执行器控制状态: {left_state['controlled']}")
print(f"左执行器类型: {left_state['type']}")
print(f"左执行器关节: {left_state['names']}")

# 打印左执行器关节详细信息
for i, joint_state in enumerate(left_state['end_states']):
    print(f"\n左执行器关节 {i+1}:")
    print(f"  关节ID: {joint_state['id']}")
    print(f"  启用状态: {joint_state['enable']}")
    print(f"  位置: {joint_state['position']:.3f} 行程值")
    print(f"  速度: {joint_state['velocity']:.3f} 行程值/秒")
    print(f"  力矩: {joint_state['effort']:.3f} 牛顿·米")
    print(f"  电流: {joint_state['current']:.3f} 安培")
    print(f"  电压: {joint_state['voltage']:.3f} 伏特")
    print(f"  温度: {joint_state['temperature']:.1f} 摄氏度")
    print(f"  状态码: {joint_state['status']}")
    print(f"  错误码: {joint_state['err_code']}")

# 打印右执行器状态
right_state = end_state['right_end_state']
print(f"\n右执行器控制状态: {right_state['controlled']}")
print(f"右执行器类型: {right_state['type']}")
print(f"右执行器关节: {right_state['names']}")

# 检查执行器状态
print("\n=== 执行器状态检查 ===")
for side in ['left', 'right']:
    state = end_state[f'{side}_end_state']
    if state['controlled']:
        print(f"✅ {side}执行器正在控制中")
    else:
        print(f"❌ {side}执行器未控制")

    for joint_state in state['end_states']:
        if joint_state['err_code'] == 0:
            print(f"✅ {side}执行器关节{joint_state['id']}正常")
        else:
            print(f"❌ {side}执行器关节{joint_state['id']}错误码: {joint_state['err_code']}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 6. `get_end_force_state()`

- **功能**：获取灵巧手触觉数据
- **参数**：无
- **返回值**：`dict`，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `left_end_force_state` | `dict` | 左末端触觉数据 | 字典 |
| `right_end_force_state` | `dict` | 右末端触觉数据 | 字典 |

**left\_end\_force\_state/right\_end\_force\_state结构**：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `timestamp` | `int` | 时间戳 | 纳秒 |
| `controlled` | `bool` | 是否受控 | 布尔值 |
| `type` | `int` | 末端执行器类型 | 整数 |
| `force` | `list[dict]` | 触觉传感器数据列表 | 传感器数据列表 |
| `status` | `int` | 状态码 | 整数 |
| `err_code` | `int` | 错误码，0表示正常 | 整数 |

**force中每个传感器数据结构**：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `sensor_type` | `int` | 传感器类型：0 Unknown，1 force，2 tactile，3 torque，4 fts | 整数 |
| `sensor_name` | `str` | 传感器名称，格式通常为 `{frame}_{surface}_{sensor_type}`，`surface` 可为 inner、outer、palmar、dorsal | 字符串 |
| `frame` | `str` | 传感器安装所在的 link 名 | 字符串 |
| `rows` | `int` | 触觉阵列行数 M | 整数 |
| `cols` | `int` | 触觉阵列列数 N | 整数 |
| `resolution_x` | `float` | 相邻两个 cell 在 x 方向的间距；没有时为 0 | mm |
| `resolution_y` | `float` | 相邻两个 cell 在 y 方向的间距；没有时为 0 | mm |
| `normal_force` | `list[float]` | 法向力，必选；按 row-major 展开，长度为 `rows * cols` | mN |
| `shear_force_x` | `list[float]` | 切向力 x；没有时为空数组或同长度 0，按右手定则 | mN |
| `shear_force_y` | `list[float]` | 切向力 y；没有时为空数组或同长度 0，按右手定则 | mN |
| `contact` | `list[int]` | 是否接触 | 无单位 |
| `valid` | `list[int]` | 是否有效；若对应 index 处布局无传感器，则为 false | 无单位 |
| `err_code` | `int` | 传感器错误码 | 整数 |

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

robot = agibot_gdk.Robot()

# 获取触觉传感器数据
result = robot.get_end_force_state()

# 遍历左右两侧触觉数据
for side in ["left", "right"]:
    key = f"{side}_end_force_state"
    state = result[key]
    print(f"\n{side} 末端触觉传感器数据:")
    print(f"  时间戳: {state['timestamp']}")
    print(f"  受控状态: {state['controlled']}")
    print(f"  类型: {state['type']}")
    print(f"  状态码: {state['status']}")
    print(f"  错误码: {state['err_code']}")
    print(f"  触觉传感器数量: {len(state['force'])}")

    for i, sensor in enumerate(state['force']):
        print(f"\n  传感器 {i}:")
        print(f"    名称: {sensor['sensor_name']}")
        print(f"    类型: {sensor['sensor_type']}")
        print(f"    坐标系: {sensor['frame']}")
        print(f"    阵列: {sensor['rows']}x{sensor['cols']}")
        print(f"    分辨率: {sensor['resolution_x']} x {sensor['resolution_y']}")
        print(f"    错误码: {sensor['err_code']}")

        # 根据 rows/cols 判断是否为有效触觉阵列传感器
        if sensor['rows'] > 0 and sensor['cols'] > 0:
            print(f"    法向力 normal_force: {sensor['normal_force']}")
            print(f"    切向力 shear_force_x: {sensor['shear_force_x']}")
            print(f"    切向力 shear_force_y: {sensor['shear_force_y']}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 7. `get_chassis_power_state()`

- **功能**：获取底盘电源状态
- **返回值**：`ChassisPowerState`对象，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `battery_main_power_switch_state` | `int` | 电池主电源开关状态 | 无单位 |
| `emergency_stop_pedal_state` | `int` | 急停踏板状态 | 无单位 |
| `battery_main_power_switch_fault_state` | `int` | 电池主电源开关故障状态 | 无单位 |
| `emergency_stop_pedal_fault_state` | `int` | 急停踏板故障状态 | 无单位 |
| `chassis_power_board_state` | `int` | 底盘电源板状态 | 无单位 |
| `chassis_left_traction_motor_power_state` | `int` | 底盘左牵引电机电源状态 | 无单位 |
| `chassis_right_traction_motor_power_state` | `int` | 底盘右牵引电机电源状态 | 无单位 |
| `chassis_left_steering_motor_power_state` | `int` | 底盘左转向电机电源状态 | 无单位 |
| `chassis_right_steering_motor_power_state` | `int` | 底盘右转向电机电源状态 | 无单位 |
| `chassis_lidar1_power_state` | `int` | 底盘激光雷达1电源状态 | 无单位 |
| `chassis_lidar2_power_state` | `int` | 底盘激光雷达2电源状态 | 无单位 |
| `chassis_ultrasonic_radar_power_state` | `int` | 底盘超声波雷达电源状态 | 无单位 |
| `chassis_tof_camera_power_state` | `int` | 底盘ToF相机电源状态 | 无单位 |
| `chassis_ethernet_switch_power_state` | `int` | 底盘以太网交换机电源状态 | 无单位 |
| `chassis_external_power_output_state` | `int` | 底盘外部电源输出状态 | 无单位 |
| `battery_main_power_output_switch_state` | `int` | 电池主电源输出开关状态 | 无单位 |
| `battery_states` | `list[BatteryState]` | 电池状态列表 | 电池状态列表 |
| `charge_plug_insert_state` | `int` | 充电插头插入状态 | 无单位 |
| `charge_plug_input_voltage` | `float` | 充电插头输入电压 | V |
| `charge_plug_input_current` | `float` | 充电插头输入电流 | A |
| `charge_plug_input_short_circuit_fault_state` | `int` | 充电插头输入短路故障状态 | 无单位 |
| `charge_plug_input_open_circuit_fault_state` | `int` | 充电插头输入开路故障状态 | 无单位 |
| `chassis_led_strip_power_state` | `int` | 底盘LED灯带电源状态 | 无单位 |
| `chassis_power_board_temperature` | `float` | 底盘电源板温度 | °C |
| `power_48v_bus_power_on_fault_state` | `int` | 48V总线电源故障状态 | 无单位 |
| `power_poe_bus_power_on_fault_state` | `int` | PoE总线电源故障状态 | 无单位 |
| `chassis_board_12v_output_fault_state` | `int` | 底盘板12V输出故障状态 | 无单位 |
| `chassis_board_5v_output_fault_state` | `int` | 底盘板5V输出故障状态 | 无单位 |
| `chassis_power_board_fault_state` | `int` | 底盘电源板故障状态 | 无单位 |

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

robot = agibot_gdk.Robot()

# 获取底盘电源状态
chassis_power_state = robot.get_chassis_power_state()
print(chassis_power_state)

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 8. `get_chest_power_state()`

- **功能**：获取胸部电源状态
- **返回值**：`ChestPowerState`对象，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `power_onoff_req` | `int` | 开关机请求 | 无单位 |
| `emergency_stop_button_req` | `int` | 急停按钮请求 | 无单位 |
| `power_switch_fault_state` | `int` | 电源开关故障状态 | 无单位 |
| `emergency_stop_button_fault_state` | `int` | 急停按钮故障状态 | 无单位 |
| `power_full_low_req` | `int` | 电源满低请求 | 无单位 |
| `chest_power_board_power_state` | `int` | 胸部电源板状态 | 无单位 |
| `domain_controller_power_state` | `int` | 域控制器电源状态 | 无单位 |
| `head_interactive_board_power_state` | `int` | 头部交互板电源状态 | 无单位 |
| `curved_screen_power_state` | `int` | 曲面屏电源状态 | 无单位 |
| `head_yaw_motor_power_state` | `int` | 头部偏航电机电源状态 | 无单位 |
| `head_pitch_motor_power_state` | `int` | 头部俯仰电机电源状态 | 无单位 |
| `head_roll_motor_power_state` | `int` | 头部滚转电机电源状态 | 无单位 |
| `fan_power_state` | `int` | 风扇电源状态 | 无单位 |
| `chest_power_board_fan_fault_state` | `int` | 胸部电源板风扇故障状态 | 无单位 |
| `body_fan1_fault_state` | `int` | 身体风扇1故障状态 | 无单位 |
| `body_fan2_fault_state` | `int` | 身体风扇2故障状态 | 无单位 |
| `body_fan3_fault_state` | `int` | 身体风扇3故障状态 | 无单位 |
| `body_fan4_fault_state` | `int` | 身体风扇4故障状态 | 无单位 |
| `upper_body_led_strip_power_state` | `int` | 上体LED灯带电源状态 | 无单位 |
| `poe_power_state` | `int` | PoE电源状态 | 无单位 |
| `ipad_power_state` | `int` | iPad电源状态 | 无单位 |
| `chest_reserved_lidar_power_state` | `int` | 胸部保留激光雷达电源状态 | 无单位 |
| `chest_power_board_temperature` | `float` | 胸部电源板温度 | °C |
| `chest_power_board_fault_state` | `int` | 胸部电源板故障状态 | 无单位 |

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

robot = agibot_gdk.Robot()

# 获取胸部电源状态
chest_power_state = robot.get_chest_power_state()
print(chest_power_state)

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 9. `joint_control_request()`

- **功能**：关节位置规划控制接口（阻塞式），执行到目标位置后，接口返回。
- **支持的控制模式**：`1`（关节位控）、`3`（关节阻抗），使用关节阻抗模式前，请先通过`set_control_mode()`切换。
- **参数**：`JointControlReq`对象，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `life_time` | `float` | 生命周期 | 秒 |
| `joint_names` | `list[str]` | 关节名称列表 | 字符串列表 |
| `joint_positions` | `list[float]` | 关节位置列表 | 弧度 |
| `joint_velocities` | `list[float]` | 关节速度列表 | 弧度/秒 |
| `detail` | `str` | 详细信息 | 字符串 |

- **返回值**：`int`，0表示成功，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

robot = agibot_gdk.Robot()

# 创建关节控制请求
joint_control_req = agibot_gdk.JointControlReq()
joint_control_req.joint_names = ["idx01_body_joint1", "idx02_body_joint2"]
joint_control_req.joint_positions = [0.0, 0.0]
joint_control_req.joint_velocities = [0.1, 0.1]
joint_control_req.life_time = 5.0
joint_control_req.detail = "测试关节控制"

try:
    # 该接口支持 control_mode=1（关节位控）/ 3（关节阻抗），需先确保已切换；
    # 默认通常已是位控模式，若想用关节阻抗请先 robot.set_control_mode(3)。
    result = robot.joint_control_request(joint_control_req)
    print("关节控制请求成功")
except Exception as e:
    print(f"关节控制请求失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 10. `move_head_joint()`

- **功能**：头部关节位置规划控制接口（阻塞式），执行到目标位置后，接口返回。
- **支持的控制模式**：`1`（关节位控）、`3`（关节阻抗），使用关节阻抗模式前，请先通过`set_control_mode()`切换。
- **参数**：
- `positions`：`list[float]`，头部关节位置列表，按照"idx11\_head\_joint1", "idx12\_head\_joint2", "idx13\_head\_joint3"顺序
- `velocities`：`list[float]`，头部关节速度列表（弧度/秒），按照"idx11\_head\_joint1", "idx12\_head\_joint2", "idx13\_head\_joint3"顺序
- **返回值**：`int`，0表示成功，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

robot = agibot_gdk.Robot()

# 控制头部位置（按照关节顺序：idx11_head_joint1, idx12_head_joint2, idx13_head_joint3）
head_positions = [0.0, 0.0, 0.0]  # 头部关节位置列表
head_velocities = [0.3, 0.3, 0.3]  # 头部关节速度列表

try:
    # 该接口支持 control_mode=1（关节位控）/ 3（关节阻抗），需先确保已切换；
    # 默认通常已是位控模式，若想用关节阻抗请先 robot.set_control_mode(3)。
    result = robot.move_head_joint(head_positions, head_velocities)
    print("头部控制成功")
except Exception as e:
    print(f"头部控制失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 11. `move_waist_joint()`

- **功能**：腰部关节位置规划控制接口（阻塞式），执行到目标位置后，接口返回。
- **支持的控制模式**：`1`（关节位控）、`3`（关节阻抗），使用关节阻抗模式前，请先通过`set_control_mode()`切换。
- **参数**：
- `positions`：`list[float]`，腰部关节位置列表，按照"idx01\_body\_joint1", "idx02\_body\_joint2", "idx03\_body\_joint3", "idx04\_body\_joint4", "idx05\_body\_joint5"顺序
- `velocities`：`list[float]`，腰部关节速度列表（弧度/秒），按照"idx01\_body\_joint1", "idx02\_body\_joint2", "idx03\_body\_joint3", "idx04\_body\_joint4", "idx05\_body\_joint5"顺序
- **返回值**：`int`，0表示成功，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

robot = agibot_gdk.Robot()

# 控制腰部位置（按照关节顺序：idx01_body_joint1, idx02_body_joint2, idx03_body_joint3, idx04_body_joint4, idx05_body_joint5）
waist_positions = [0.0, 0.0, 0.0, 0.0, 0.0]  # 腰部关节位置列表
waist_velocities = [0.3, 0.3, 0.3, 0.3, 0.3]  # 腰部关节速度列表

try:
    # 该接口支持 control_mode=1（关节位控）/ 3（关节阻抗），需先确保已切换；
    # 默认通常已是位控模式，若想用关节阻抗请先 robot.set_control_mode(3)。
    result = robot.move_waist_joint(waist_positions, waist_velocities)
    print("腰部控制成功")
except Exception as e:
    print(f"腰部控制失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 12. `move_arm_joint()`

- **功能**：手臂关节位置规划控制接口（阻塞式），执行到目标位置后，接口返回。
- **支持的控制模式**：`1`（关节位控）、`3`（关节阻抗），使用关节阻抗模式前，请先通过`set_control_mode()`切换。
- **参数**：
- `positions`：`list[float]`，手臂关节位置列表，按照"idx21\_arm\_l\_joint1", "idx22\_arm\_l\_joint2", "idx23\_arm\_l\_joint3", "idx24\_arm\_l\_joint4", "idx25\_arm\_l\_joint5", "idx26\_arm\_l\_joint6", "idx27\_arm\_l\_joint7", "idx61\_arm\_r\_joint1", "idx62\_arm\_r\_joint2", "idx63\_arm\_r\_joint3", "idx64\_arm\_r\_joint4", "idx65\_arm\_r\_joint5", "idx66\_arm\_r\_joint6", "idx67\_arm\_r\_joint7"顺序
- `velocities`：`list[float]`，手臂关节速度列表（弧度/秒），按照"idx21\_arm\_l\_joint1", "idx22\_arm\_l\_joint2", "idx23\_arm\_l\_joint3", "idx24\_arm\_l\_joint4", "idx25\_arm\_l\_joint5", "idx26\_arm\_l\_joint6", "idx27\_arm\_l\_joint7", "idx61\_arm\_r\_joint1", "idx62\_arm\_r\_joint2", "idx63\_arm\_r\_joint3", "idx64\_arm\_r\_joint4", "idx65\_arm\_r\_joint5", "idx66\_arm\_r\_joint6", "idx67\_arm\_r\_joint7"顺序
- `control_group`：`int`，控制组，0表示控制左臂，1表示控制右臂，2表示控制双臂
- **返回值**：`int`，0表示成功，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

robot = agibot_gdk.Robot()

# 控制手臂位置（按照关节顺序：左臂7个关节 + 右臂7个关节）
arm_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # 左臂7个关节
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 右臂7个关节
arm_velocities = [0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3,  # 左臂7个关节速度
                  0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]  # 右臂7个关节速度

try:
    # 该接口支持 control_mode=1（关节位控）/ 3（关节阻抗），需先确保已切换；
    # 默认通常已是位控模式，若想用关节阻抗请先 robot.set_control_mode(3)。
    result = robot.move_arm_joint(arm_positions, arm_velocities, 2)
    print("手臂控制成功")
except Exception as e:
    print(f"手臂控制失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 13. `joint_servo_control()`

- **功能**：关节位置伺服控制接口（非阻塞式），需要以100hz的控制频率进行控制，支持正常模式和低延时模式，默认使用正常模式。该接口支持同时控制末端执行器。
- **支持的控制模式**：`1`（关节位控）、`3`（关节阻抗），使用关节阻抗模式前，请先通过`set_control_mode()`切换。
- **注意**：低延时模式下，该接口绕过运控算法保护，直接驱动电机，仅在必要时使用。潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。开发者须自行评估环境与工况，并实现必要的安全保护措施。如需同时控制末端执行器，需要使用该接口下发控制指令，具体参考以下示例。
- **参数**：
- `joint_servo_control_req`：`JointServoControlReq`对象，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `control_period` | `float` | 控制周期 | 秒 |
| `joint_names` | `list[str]` | 关节名称列表 | 字符串列表 |
| `joint_positions` | `list[float]` | 关节位置列表 | 弧度 |
| `joint_velocities` | `list[float]` | 关节速度列表 | 弧度/秒 |

- `enable_low_latency`：`bool`，可选参数，默认为`False`。

**参数说明**：
- `control_period` 表示控制周期，单位为秒,建议比控制频率稍大
- `joint_names`、`joint_positions` 两个列表的长度必须相同
- `joint_positions` 中的值必须在对应关节的限位范围内（限位参考值见 [`get_joint_limits()`](#get_joint_limits)），否则将抛出异常
- `joint_names`、`joint_positions` 不能为空，否则将抛出异常
- `joint_velocities` 预留参数，目前不使用，可以为空
- `enable_low_latency` 用于选择控制通道，低延时模式适用于对实时性要求更高的场景
- **返回值**：`int`，0表示成功，失败时抛出异常

- **异常说明**：
- 如果 `joint_names`、`joint_positions` 为空，底层将返回 `ErrorCode::kInvalidInput`，Python 将抛出 `RuntimeError("JointServoControlRequest failed")`
- 如果 `joint_positions` 中的值超出对应关节的限位范围，底层将返回 `ErrorCode::kInvalidInput`，Python 将抛出异常
- 如果配置解析器获取失败，底层将返回 `ErrorCode::kRuntimeError`，Python 将抛出异常
- **示例**：

```
import agibot_gdk
import time
import math

# 控制参数
CONTROL_PERIOD = 0.01  # 控制周期（秒）
RATE_HZ = 100.0        # 发送频率（Hz）
DURATION = 5.0         # 控制持续时间（秒）
MAX_POSITION_DELTA = 0.1  # 最大位置变化量（弧度）

class JointServoControlController:
    def __init__(self, robot):
        self.robot = robot

    def get_joint_position_by_name(self, joint_states, joint_name):
        """根据关节名称获取关节位置"""
        for state in joint_states['states']:
            if state['name'] == joint_name:
                return state['motor_position']
        raise RuntimeError(f"Joint name {joint_name} not found")

    def interpolate_position(self, start_pos, target_pos, t):
        """线性插值计算中间位置"""
        return start_pos + t * (target_pos - start_pos)

    def execute_joint_servo_control(self, target_joint_names, target_positions):
        """执行关节位置伺服控制"""
        time.sleep(1.0)  # 等待1秒

        # 获取当前关节状态
        current_joint_states = self.robot.get_joint_states()
        print(f"当前关节数量: {current_joint_states['nums']}")

        # 获取起始位置
        start_positions = []
        for joint_name in target_joint_names:
            try:
                pos = self.get_joint_position_by_name(current_joint_states, joint_name)
                start_positions.append(pos)
                print(f"关节 {joint_name} 当前位置: {pos:.3f} 弧度")
            except RuntimeError as e:
                print(f"错误: {e}")
                return

        # 计算步数
        n_steps = int(DURATION * RATE_HZ)
        print(f"总步数: {n_steps}, 持续时间: {DURATION} 秒")

        # 执行轨迹
        dt = 1.0 / RATE_HZ
        start_time = time.time()

        for i in range(n_steps):
            t = float(i) / (n_steps - 1) if n_steps > 1 else 0.0

            # 创建关节位置伺服请求
            joint_servo_control_req = agibot_gdk.JointServoControlReq()
            joint_servo_control_req.control_period = CONTROL_PERIOD

            # 计算当前目标位置（线性插值）
            current_positions = []
            for j, joint_name in enumerate(target_joint_names):
                interp_pos = self.interpolate_position(
                    start_positions[j], target_positions[j], t
                )
                current_positions.append(interp_pos)

            joint_servo_control_req.joint_names = target_joint_names
            joint_servo_control_req.joint_positions = current_positions

            try:
                # 使用普通模式（enable_low_latency=False，默认值）
                result = self.robot.joint_servo_control(joint_servo_control_req)
                # 如需使用低延时模式，可传入 enable_low_latency=True：
                # 注意：低延时模式下，该接口绕过运控算法保护，直接驱动电机。
                # 潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。
                # result = self.robot.joint_servo_control(joint_servo_control_req, enable_low_latency=True)
                if result != 0:
                    print(f"控制命令发送失败，步数: {i}")
                    return
            except Exception as e:
                print(f"控制命令发送异常，步数: {i}, 错误: {e}")
                return

            # 控制发送频率
            elapsed = time.time() - start_time
            expected_time = (i + 1) * dt
            sleep_time = expected_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        print("关节位置伺服控制完成")

        # 保持最终位置
        print("进入最终位置保持（Ctrl+C 结束）...")
        try:
            while True:
                joint_servo_control_req = agibot_gdk.JointServoControlReq()
                joint_servo_control_req.control_period = CONTROL_PERIOD
                joint_servo_control_req.joint_names = target_joint_names
                joint_servo_control_req.joint_positions = target_positions

                try:
                    # 使用普通模式（enable_low_latency=False，默认值）
                    result = self.robot.joint_servo_control(joint_servo_control_req)
                    # 如需使用低延时模式，可传入 enable_low_latency=True：
                    # 注意：低延时模式下，该接口绕过运控算法保护，直接驱动电机。
                    # 潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。
                    # result = self.robot.joint_servo_control(joint_servo_control_req, enable_low_latency=True)
                    if result != 0:
                        print("保持位置失败")
                        break
                except Exception as e:
                    print(f"保持位置异常: {e}")
                    break

                time.sleep(dt)
        except KeyboardInterrupt:
            print("\n已中断保持")

def main():
    # 初始化GDK系统
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK初始化失败")
        return

    print("GDK初始化成功")

    try:
        robot = agibot_gdk.Robot()

        # 获取当前关节状态
        current_joint_states = robot.get_joint_states()
        print(f"当前关节数量: {current_joint_states['nums']}")

        # 定义要控制的关节（示例：左臂前3个关节）
        target_joint_names = [
            "idx21_arm_l_joint1",
            "idx22_arm_l_joint2",
            "idx23_arm_l_joint3"
        ]

        # 获取当前位置作为起始位置（用于插值）
        controller = JointServoControlController(robot)
        start_positions = []
        for joint_name in target_joint_names:
            pos = controller.get_joint_position_by_name(current_joint_states, joint_name)
            start_positions.append(pos)
            print(f"关节 {joint_name} 当前位置: {pos:.3f} 弧度")

        # 设置目标角度（直接指定目标角度值，单位：弧度）
        target_positions = [
            0.0,  # idx21_arm_l_joint1: 目标角度 0.0 弧度
            0.0,  # idx22_arm_l_joint2: 目标角度 0.0 弧度
            0.0   # idx23_arm_l_joint3: 目标角度 0.0 弧度
        ]

        print(f"\n目标角度:")
        for i, joint_name in enumerate(target_joint_names):
            print(f"  {joint_name}: {target_positions[i]:.3f} 弧度")

        # 执行关节位置伺服控制
        # 该接口支持 control_mode=1（关节位控）/ 3（关节阻抗），需先确保已切换；
        # 默认通常已是位控模式，若想用关节阻抗请先 robot.set_control_mode(3)。
        controller.execute_joint_servo_control(
            target_joint_names, target_positions)

    except Exception as e:
        print(f"执行过程中发生错误: {e}")
    finally:
        # 释放GDK系统资源
        if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
            print("GDK释放失败")
        else:
            print("GDK释放成功")

if __name__ == "__main__":
    main()
```

- **同时控制机械臂与末端的示例**：

```
import agibot_gdk
import time

# 控制参数
CONTROL_PERIOD = 0.01  # 控制周期（秒）
RATE_HZ = 100.0        # 发送频率（Hz）
DURATION = 3.0         # 单次移动持续时间（秒）
HOLD_DURATION = 0.5    # 在 0 或 1 处保持的时间（秒）
NUM_CYCLES = 3         # 往复运动次数

# 左臂 7 个关节名称（与 get_joint_states 中一致）
ARM_L_JOINT_NAMES = [
    "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3",
    "idx24_arm_l_joint4", "idx25_arm_l_joint5", "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
]

def get_arm_positions_by_name(joint_states, joint_names):
    """从 get_joint_states 的返回中按关节名取出位置列表"""
    name_to_pos = {s["name"]: s["motor_position"] for s in joint_states["states"]}
    return [name_to_pos[name] for name in joint_names]

def get_ee_names_and_positions(end_state, side="left"):
    """从 get_end_state 的返回中取出指定侧末端的关节名列表和位置列表"""
    key = f"{side}_end_state"
    if key not in end_state:
        raise RuntimeError(f"get_end_state 中未找到 {key}")
    state = end_state[key]
    names = state.get("names", [])
    positions = [s["position"] for s in state.get("end_states", [])]
    if len(names) != len(positions):
        raise RuntimeError(f"{key} 中 names 与 end_states 长度不一致")
    return names, positions

def main():
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK初始化失败")
        return
    print("GDK初始化成功")

    try:
        robot = agibot_gdk.Robot()

        # 1) 从 get_joint_states 获取全身关节（含机械臂）
        joint_states = robot.get_joint_states()
        # 2) 从 get_end_state 获取末端关节名与当前位置（如左夹爪/左手）
        end_state = robot.get_end_state()
        ee_names, ee_positions = get_ee_names_and_positions(end_state, "left")

        # 合并：机械臂关节 + 末端关节
        all_names = ARM_L_JOINT_NAMES + ee_names
        arm_positions = get_arm_positions_by_name(joint_states, ARM_L_JOINT_NAMES)

        # 机械臂小幅运动，末端关节在 -0.785 和 0 之间往复运动(omnipicker)
        arm_start = arm_positions[:]                           # 机械臂初始姿态
        arm_target = [p + 0.05 for p in arm_positions]         # 机械臂目标姿态（小幅偏移）
        ee_low = -0.785                                        # 末端下限
        ee_high = 0.0                                          # 末端上限

        print(f"控制关节数: 机械臂 {len(ARM_L_JOINT_NAMES)} + 末端 {len(ee_names)} = {len(all_names)}")
        print(f"机械臂将在关节空间 arm_start ↔ arm_target 之间运动，末端关节在 {ee_low} 和 {ee_high} 之间往复 {NUM_CYCLES} 次")

        n_steps = int(DURATION * RATE_HZ)           # 单次移动的步数
        hold_steps = int(HOLD_DURATION * RATE_HZ)   # 保持阶段的步数
        dt = 1.0 / RATE_HZ

        def send_control_command(arm_target_values, ee_target_values):
            """发送控制命令的辅助函数

            joint_servo_control 支持 control_mode=1（关节位控）/ 3（关节阻抗），
            默认通常已是位控模式，若想用关节阻抗请先 robot.set_control_mode(3)。
            """
            current = arm_target_values + ee_target_values
            req = agibot_gdk.JointServoControlReq()
            req.control_period = CONTROL_PERIOD
            req.joint_names = all_names
            req.joint_positions = current
            return robot.joint_servo_control(req)

        for cycle in range(NUM_CYCLES):
            print(f"\n=== 周期 {cycle+1}/{NUM_CYCLES}: 机械臂 arm_start -> arm_target, 末端 {ee_low} -> {ee_high} ===")
            start_time = time.time()

            # 第一段：机械臂 arm_start -> arm_target，末端 ee_low -> ee_high
            for i in range(n_steps):
                t = float(i) / (n_steps - 1) if n_steps > 1 else 1.0
                current_arm = [
                    arm_start[j] + t * (arm_target[j] - arm_start[j])
                    for j in range(len(ARM_L_JOINT_NAMES))
                ]
                current_ee = [ee_low + t * (ee_high - ee_low) for _ in ee_names]

                result = send_control_command(current_arm, current_ee)
                if result != 0:
                    print(f"发送失败，周期={cycle+1}, 阶段=0->1, 步数={i}")
                    raise RuntimeError("joint_servo_control failed")

                elapsed = time.time() - start_time
                sleep_time = (i + 1) * dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

            # 第二段：在 arm_target / ee_high 处保持
            print(f"=== 周期 {cycle+1}/{NUM_CYCLES}: 在 arm_target / {ee_high} 处保持 {HOLD_DURATION} 秒 ===")
            start_time = time.time()
            current_arm = arm_target[:]
            current_ee = [ee_high for _ in ee_names]

            for i in range(hold_steps):
                result = send_control_command(current_arm, current_ee)
                if result != 0:
                    print(f"发送失败，周期={cycle+1}, 阶段=保持@1, 步数={i}")
                    raise RuntimeError("joint_servo_control failed")

                elapsed = time.time() - start_time
                sleep_time = (i + 1) * dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

            # 第三段：机械臂 arm_target -> arm_start，末端 ee_high -> ee_low
            print(f"=== 周期 {cycle+1}/{NUM_CYCLES}: 机械臂 arm_target -> arm_start, 末端 {ee_high} -> {ee_low} ===")
            start_time = time.time()

            for i in range(n_steps):
                t = float(i) / (n_steps - 1) if n_steps > 1 else 1.0
                current_arm = [
                    arm_target[j] + t * (arm_start[j] - arm_target[j])
                    for j in range(len(ARM_L_JOINT_NAMES))
                ]
                current_ee = [ee_high + t * (ee_low - ee_high) for _ in ee_names]

                result = send_control_command(current_arm, current_ee)
                if result != 0:
                    print(f"发送失败，周期={cycle+1}, 阶段=1->0, 步数={i}")
                    raise RuntimeError("joint_servo_control failed")

                elapsed = time.time() - start_time
                sleep_time = (i + 1) * dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

            # 第四段：在 arm_start / ee_low 处保持
            print(f"=== 周期 {cycle+1}/{NUM_CYCLES}: 在 arm_start / {ee_low} 处保持 {HOLD_DURATION} 秒 ===")
            start_time = time.time()
            current_arm = arm_start[:]
            current_ee = [ee_low for _ in ee_names]

            for i in range(hold_steps):
                result = send_control_command(current_arm, current_ee)
                if result != 0:
                    print(f"发送失败，周期={cycle+1}, 阶段=保持@0, 步数={i}")
                    raise RuntimeError("joint_servo_control failed")

                elapsed = time.time() - start_time
                sleep_time = (i + 1) * dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        print(f"\n机械臂 arm_start↔arm_target、末端 {ee_low}↔{ee_high} 往复控制结束")
    except Exception as e:
        print(f"执行错误: {e}")
    finally:
        if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
            print("GDK释放失败")
        else:
            print("GDK释放成功")

if __name__ == "__main__":
    main()
```

#### 14. `move_head_joint_servo()`

- **功能**：头部关节位置伺服控制接口（非阻塞式），需要以100hz的控制频率进行控制，支持正常模式和低延时模式，默认使用正常模式。
- **支持的控制模式**：`1`（关节位控）、`3`（关节阻抗），使用关节阻抗模式前，请先通过`set_control_mode()`切换。
- **注意**：低延时模式下，该接口绕过运控算法保护，直接驱动电机，仅在必要时使用。潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。开发者须自行评估环境与工况，并实现必要的安全保护措施。
- **参数**：
- `positions`：`list[float]`，头部关节位置列表（弧度），按照"idx11\_head\_joint1", "idx12\_head\_joint2", "idx13\_head\_joint3"顺序
- `control_period`：`float`，控制周期（秒），建议比控制频率稍大
- `enable_low_latency`：`bool`，可选参数，默认为`False`，用于选择控制通道

**参数说明**：
- `positions` 长度必须为3
- `positions` 中的值必须在对应关节的限位范围内（限位参考值见 [`get_joint_limits()`](#get_joint_limits)），否则将抛出异常
- `control_period` 表示控制周期，单位为秒，建议比控制频率稍大
- `enable_low_latency` 用于选择控制通道，低延时模式适用于对实时性要求更高的场景

- **返回值**：`int`，0表示成功，失败时抛出异常
- **异常说明**：
- 如果 `positions` 的长度不为3，底层将返回 `ErrorCode::kInvalidInput`，Python 将抛出异常
- 如果 `positions` 中的值超出对应关节的限位范围，底层将返回 `ErrorCode::kInvalidInput`，Python 将抛出异常
- **示例**：

```
import agibot_gdk
import time
import math

# 控制参数
CONTROL_PERIOD = 0.01  # 控制周期（秒）
RATE_HZ = 100.0        # 发送频率（Hz）
DURATION = 5.0         # 控制持续时间（秒）

class HeadJointServoController:
    def __init__(self, robot):
        self.robot = robot

    def get_joint_position_by_name(self, joint_states, joint_name):
        """根据关节名称获取关节位置"""
        for state in joint_states['states']:
            if state['name'] == joint_name:
                return state['motor_position']
        raise RuntimeError(f"Joint name {joint_name} not found")

    def interpolate_position(self, start_pos, target_pos, t):
        """线性插值计算中间位置"""
        return start_pos + t * (target_pos - start_pos)

    def execute_head_joint_servo_control(self, target_positions):
        """执行头部关节位置伺服控制"""
        time.sleep(1.0)  # 等待1秒

        # 获取当前关节状态
        current_joint_states = self.robot.get_joint_states()

        # 获取起始位置
        head_joint_names = ["idx11_head_joint1", "idx12_head_joint2", "idx13_head_joint3"]
        start_positions = []
        for joint_name in head_joint_names:
            try:
                pos = self.get_joint_position_by_name(current_joint_states, joint_name)
                start_positions.append(pos)
                print(f"关节 {joint_name} 当前位置: {pos:.3f} 弧度")
            except RuntimeError as e:
                print(f"错误: {e}")
                return

        # 计算步数
        n_steps = int(DURATION * RATE_HZ)
        print(f"总步数: {n_steps}, 持续时间: {DURATION} 秒")

        # 执行轨迹
        dt = 1.0 / RATE_HZ
        start_time = time.time()

        for i in range(n_steps):
            t = float(i) / (n_steps - 1) if n_steps > 1 else 0.0

            # 计算当前目标位置（线性插值）
            current_positions = []
            for j in range(3):
                interp_pos = self.interpolate_position(
                    start_positions[j], target_positions[j], t
                )
                current_positions.append(interp_pos)

            try:
                # 使用普通模式（enable_low_latency=False，默认值）
                result = self.robot.move_head_joint_servo(
                    current_positions, CONTROL_PERIOD)
                # 如需使用低延时模式，可传入 enable_low_latency=True：
                # 注意：低延时模式下，该接口绕过运控算法保护，直接驱动电机。
                # 潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。
                # result = self.robot.move_head_joint_servo(
                #     current_positions, CONTROL_PERIOD, enable_low_latency=True
                # )
                if result != 0:
                    print(f"控制命令发送失败，步数: {i}")
                    return
            except Exception as e:
                print(f"控制命令发送异常，步数: {i}, 错误: {e}")
                return

            # 控制发送频率
            elapsed = time.time() - start_time
            expected_time = (i + 1) * dt
            sleep_time = expected_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        print("头部关节位置伺服控制完成")

        # 保持最终位置
        print("进入最终位置保持（Ctrl+C 结束）...")
        try:
            while True:
                try:
                    # 使用普通模式（enable_low_latency=False，默认值）
                    result = self.robot.move_head_joint_servo(
                        target_positions, CONTROL_PERIOD
                    )
                    # 如需使用低延时模式，可传入 enable_low_latency=True：
                    # 注意：低延时模式下，该接口绕过运控算法保护，直接驱动电机。
                    # 潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。
                    # result = self.robot.move_head_joint_servo(
                    #     target_positions, CONTROL_PERIOD, enable_low_latency=True
                    # )
                    if result != 0:
                        print("保持位置失败")
                        break
                except Exception as e:
                    print(f"保持位置异常: {e}")
                    break

                time.sleep(dt)
        except KeyboardInterrupt:
            print("\n已中断保持")

def main():
    # 初始化GDK系统
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK初始化失败")
        return

    print("GDK初始化成功")

    try:
        robot = agibot_gdk.Robot()

        # 获取当前关节状态
        current_joint_states = robot.get_joint_states()

        # 定义目标位置（示例：头部关节）
        target_positions = [
            0.0,  # idx11_head_joint1: 目标角度 0.0 弧度
            0.0,  # idx12_head_joint2: 目标角度 0.0 弧度
            0.0   # idx13_head_joint3: 目标角度 0.0 弧度
        ]

        print(f"\n目标角度:")
        head_joint_names = ["idx11_head_joint1", "idx12_head_joint2", "idx13_head_joint3"]
        for i, joint_name in enumerate(head_joint_names):
            print(f"  {joint_name}: {target_positions[i]:.3f} 弧度")

        # 执行头部关节位置伺服控制
        # 该接口支持 control_mode=1（关节位控）/ 3（关节阻抗），需先确保已切换；
        # 默认通常已是位控模式，若想用关节阻抗请先 robot.set_control_mode(3)。
        controller = HeadJointServoController(robot)
        controller.execute_head_joint_servo_control(target_positions)

    except Exception as e:
        print(f"执行过程中发生错误: {e}")
    finally:
        # 释放GDK系统资源
        if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
            print("GDK释放失败")
        else:
            print("GDK释放成功")

if __name__ == "__main__":
    main()
```

#### 15. `move_waist_joint_servo()`

- **功能**：腰部关节位置伺服控制接口（非阻塞式），需要以100hz的控制频率进行控制，支持正常模式和低延时模式，默认使用正常模式。
- **支持的控制模式**：`1`（关节位控）、`3`（关节阻抗），使用关节阻抗模式前，请先通过`set_control_mode()`切换。
- **注意**：低延时模式下，该接口绕过运控算法保护，直接驱动电机，仅在必要时使用。潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。开发者须自行评估环境与工况，并实现必要的安全保护措施。
- **参数**：
- `positions`：`list[float]`，腰部关节位置列表（弧度），按照"idx01\_body\_joint1", "idx02\_body\_joint2", "idx03\_body\_joint3", "idx04\_body\_joint4", "idx05\_body\_joint5"顺序
- `control_period`：`float`，控制周期（秒），建议比控制频率稍大
- `enable_low_latency`：`bool`，可选参数，默认为`False`，用于选择控制通道

**参数说明**：
- `positions` 长度必须为5
- `positions` 中的值必须在对应关节的限位范围内（限位参考值见 [`get_joint_limits()`](#get_joint_limits)），否则将抛出异常
- `control_period` 表示控制周期，单位为秒，建议比控制频率稍大
- `enable_low_latency` 用于选择控制通道，低延时模式适用于对实时性要求更高的场景

- **返回值**：`int`，0表示成功，失败时抛出异常
- **异常说明**：
- 如果 `positions` 的长度不为5，底层将返回 `ErrorCode::kInvalidInput`，Python 将抛出异常
- 如果 `positions` 中的值超出对应关节的限位范围，底层将返回 `ErrorCode::kInvalidInput`，Python 将抛出异常
- **示例**：

```
import agibot_gdk
import time
import math

# 控制参数
CONTROL_PERIOD = 0.01  # 控制周期（秒）
RATE_HZ = 100.0        # 发送频率（Hz）
DURATION = 5.0         # 控制持续时间（秒）

class WaistJointServoController:
    def __init__(self, robot):
        self.robot = robot

    def get_joint_position_by_name(self, joint_states, joint_name):
        """根据关节名称获取关节位置"""
        for state in joint_states['states']:
            if state['name'] == joint_name:
                return state['motor_position']
        raise RuntimeError(f"Joint name {joint_name} not found")

    def interpolate_position(self, start_pos, target_pos, t):
        """线性插值计算中间位置"""
        return start_pos + t * (target_pos - start_pos)

    def execute_waist_joint_servo_control(self, target_positions):
        """执行腰部关节位置伺服控制"""
        time.sleep(1.0)  # 等待1秒

        # 获取当前关节状态
        current_joint_states = self.robot.get_joint_states()

        # 获取起始位置
        waist_joint_names = [
            "idx01_body_joint1", "idx02_body_joint2", "idx03_body_joint3",
            "idx04_body_joint4", "idx05_body_joint5"
        ]
        start_positions = []
        for joint_name in waist_joint_names:
            try:
                pos = self.get_joint_position_by_name(current_joint_states, joint_name)
                start_positions.append(pos)
                print(f"关节 {joint_name} 当前位置: {pos:.3f} 弧度")
            except RuntimeError as e:
                print(f"错误: {e}")
                return

        # 计算步数
        n_steps = int(DURATION * RATE_HZ)
        print(f"总步数: {n_steps}, 持续时间: {DURATION} 秒")

        # 执行轨迹
        dt = 1.0 / RATE_HZ
        start_time = time.time()

        for i in range(n_steps):
            t = float(i) / (n_steps - 1) if n_steps > 1 else 0.0

            # 计算当前目标位置（线性插值）
            current_positions = []
            for j in range(5):
                interp_pos = self.interpolate_position(
                    start_positions[j], target_positions[j], t
                )
                current_positions.append(interp_pos)

            try:
                # 使用普通模式（enable_low_latency=False，默认值）
                result = self.robot.move_waist_joint_servo(
                    current_positions, CONTROL_PERIOD
                )
                # 如需使用低延时模式，可传入 enable_low_latency=True：
                # 注意：低延时模式下，该接口绕过运控算法保护，直接驱动电机。
                # 潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。
                # result = self.robot.move_waist_joint_servo(
                #     current_positions, CONTROL_PERIOD, enable_low_latency=True
                # )
                if result != 0:
                    print(f"控制命令发送失败，步数: {i}")
                    return
            except Exception as e:
                print(f"控制命令发送异常，步数: {i}, 错误: {e}")
                return

            # 控制发送频率
            elapsed = time.time() - start_time
            expected_time = (i + 1) * dt
            sleep_time = expected_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        print("腰部关节位置伺服控制完成")

        # 保持最终位置
        print("进入最终位置保持（Ctrl+C 结束）...")
        try:
            while True:
                try:
                    # 使用普通模式（enable_low_latency=False，默认值）
                    result = self.robot.move_waist_joint_servo(
                        target_positions, CONTROL_PERIOD
                    )
                    # 如需使用低延时模式，可传入 enable_low_latency=True：
                    # 注意：低延时模式下，该接口绕过运控算法保护，直接驱动电机。
                    # 潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。
                    # result = self.robot.move_waist_joint_servo(
                    #     target_positions, CONTROL_PERIOD, enable_low_latency=True
                    # )
                    if result != 0:
                        print("保持位置失败")
                        break
                except Exception as e:
                    print(f"保持位置异常: {e}")
                    break

                time.sleep(dt)
        except KeyboardInterrupt:
            print("\n已中断保持")

def main():
    # 初始化GDK系统
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK初始化失败")
        return

    print("GDK初始化成功")

    try:
        robot = agibot_gdk.Robot()

        # 获取当前关节状态
        current_joint_states = robot.get_joint_states()

        # 定义目标位置（示例：腰部关节）
        target_positions = [
            0.0,  # idx01_body_joint1: 目标角度 0.0 弧度
            0.0,  # idx02_body_joint2: 目标角度 0.0 弧度
            0.0,  # idx03_body_joint3: 目标角度 0.0 弧度
            0.0,  # idx04_body_joint4: 目标角度 0.0 弧度
            0.0   # idx05_body_joint5: 目标角度 0.0 弧度
        ]

        print(f"\n目标角度:")
        waist_joint_names = [
            "idx01_body_joint1", "idx02_body_joint2", "idx03_body_joint3",
            "idx04_body_joint4", "idx05_body_joint5"
        ]
        for i, joint_name in enumerate(waist_joint_names):
            print(f"  {joint_name}: {target_positions[i]:.3f} 弧度")

        # 执行腰部关节位置伺服控制
        # 该接口支持 control_mode=1（关节位控）/ 3（关节阻抗），需先确保已切换；
        # 默认通常已是位控模式，若想用关节阻抗请先 robot.set_control_mode(3)。
        controller = WaistJointServoController(robot)
        controller.execute_waist_joint_servo_control(target_positions)

    except Exception as e:
        print(f"执行过程中发生错误: {e}")
    finally:
        # 释放GDK系统资源
        if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
            print("GDK释放失败")
        else:
            print("GDK释放成功")

if __name__ == "__main__":
    main()
```

#### 16. `move_arm_joint_servo()`

- **功能**：手臂关节位置伺服控制接口（非阻塞式），需要以100hz的控制频率进行控制，支持正常模式和低延时模式，默认使用正常模式。
- **支持的控制模式**：`1`（关节位控）、`3`（关节阻抗），使用关节阻抗模式前，请先通过`set_control_mode()`切换。
- **注意**：低延时模式下，该接口绕过运控算法保护，直接驱动电机，仅在必要时使用。潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。开发者须自行评估环境与工况，并实现必要的安全保护措施。
- **参数**：
- `positions`：`list[float]`，手臂关节位置列表（弧度），按照"idx21\_arm\_l\_joint1", "idx22\_arm\_l\_joint2", "idx23\_arm\_l\_joint3", "idx24\_arm\_l\_joint4", "idx25\_arm\_l\_joint5", "idx26\_arm\_l\_joint6", "idx27\_arm\_l\_joint7", "idx61\_arm\_r\_joint1", "idx62\_arm\_r\_joint2", "idx63\_arm\_r\_joint3", "idx64\_arm\_r\_joint4", "idx65\_arm\_r\_joint5", "idx66\_arm\_r\_joint6", "idx67\_arm\_r\_joint7"顺序
- `control_period`：`float`，控制周期（秒），建议比控制频率稍大
- `control_group`：`int`，控制组，0表示控制左臂，1表示控制右臂，2表示控制双臂
- `enable_low_latency`：`bool`，可选参数，默认为`False`，用于选择控制通道

**参数说明**：
- `positions` 长度必须为7(左臂或右臂)或14(双臂)
- `positions` 中的值必须在对应关节的限位范围内（限位参考值见 [`get_joint_limits()`](#get_joint_limits)），否则将抛出异常
- `control_period` 表示控制周期，单位为秒，建议比控制频率稍大
- `control_group` 表示控制组，0表示控制左臂，1表示控制右臂，2表示控制双臂
- `enable_low_latency` 用于选择控制通道，低延时模式适用于对实时性要求更高的场景

- **返回值**：`int`，0表示成功，失败时抛出异常
- **异常说明**：
- 如果 `positions` 的长度不为7(左臂或右臂)或14(双臂)，底层将返回 `ErrorCode::kInvalidInput`，Python 将抛出异常
- 如果 `positions` 中的值超出对应关节的限位范围，底层将返回 `ErrorCode::kInvalidInput`，Python 将抛出异常
- **示例**：

```
import agibot_gdk
import time
import math

# 控制参数
CONTROL_PERIOD = 0.01  # 控制周期（秒）
RATE_HZ = 100.0        # 发送频率（Hz）
DURATION = 5.0         # 控制持续时间（秒）

class ArmJointServoController:
    def __init__(self, robot):
        self.robot = robot

    def get_joint_position_by_name(self, joint_states, joint_name):
        """根据关节名称获取关节位置"""
        for state in joint_states['states']:
            if state['name'] == joint_name:
                return state['motor_position']
        raise RuntimeError(f"Joint name {joint_name} not found")

    def interpolate_position(self, start_pos, target_pos, t):
        """线性插值计算中间位置"""
        return start_pos + t * (target_pos - start_pos)

    def execute_arm_joint_servo_control(self, target_positions, control_group):
        """执行手臂关节位置伺服控制"""
        time.sleep(1.0)  # 等待1秒

        # 获取当前关节状态
        current_joint_states = self.robot.get_joint_states()

        # 获取起始位置
        arm_joint_names = [
            "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3",
            "idx24_arm_l_joint4", "idx25_arm_l_joint5", "idx26_arm_l_joint6",
            "idx27_arm_l_joint7",  # 左臂7个关节
            "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3",
            "idx64_arm_r_joint4", "idx65_arm_r_joint5", "idx66_arm_r_joint6",
            "idx67_arm_r_joint7"   # 右臂7个关节
        ]
        start_positions = []
        for joint_name in arm_joint_names:
            try:
                pos = self.get_joint_position_by_name(current_joint_states, joint_name)
                start_positions.append(pos)
                print(f"关节 {joint_name} 当前位置: {pos:.3f} 弧度")
            except RuntimeError as e:
                print(f"错误: {e}")
                return

        # 计算步数
        n_steps = int(DURATION * RATE_HZ)
        print(f"总步数: {n_steps}, 持续时间: {DURATION} 秒")

        # 执行轨迹
        dt = 1.0 / RATE_HZ
        start_time = time.time()

        for i in range(n_steps):
            t = float(i) / (n_steps - 1) if n_steps > 1 else 0.0

            # 计算当前目标位置（线性插值）
            current_positions = []
            for j in range(14):
                interp_pos = self.interpolate_position(
                    start_positions[j], target_positions[j], t
                )
                current_positions.append(interp_pos)

            try:
                # 使用普通模式（enable_low_latency=False，默认值）
                result = self.robot.move_arm_joint_servo(
                    current_positions, CONTROL_PERIOD, control_group
                )
                # 如需使用低延时模式，可传入 enable_low_latency=True：
                # 注意：低延时模式下，该接口绕过运控算法保护，直接驱动电机。
                # 潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。
                # result = self.robot.move_arm_joint_servo(
                #     current_positions, CONTROL_PERIOD, control_group, enable_low_latency=True
                # )
                if result != 0:
                    print(f"控制命令发送失败，步数: {i}")
                    return
            except Exception as e:
                print(f"控制命令发送异常，步数: {i}, 错误: {e}")
                return

            # 控制发送频率
            elapsed = time.time() - start_time
            expected_time = (i + 1) * dt
            sleep_time = expected_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        print("手臂关节位置伺服控制完成")

        # 保持最终位置
        print("进入最终位置保持（Ctrl+C 结束）...")
        try:
            while True:
                try:
                    # 使用普通模式（enable_low_latency=False，默认值）
                    result = self.robot.move_arm_joint_servo(
                        target_positions, CONTROL_PERIOD, control_group
                    )
                    # 如需使用低延时模式，可传入 enable_low_latency=True：
                    # 注意：低延时模式下，该接口绕过运控算法保护，直接驱动电机。
                    # 潜在风险：自碰撞、机体倾倒、与环境碰撞，可能导致财产损失或人员伤害。
                    # result = self.robot.move_arm_joint_servo(
                    #     target_positions, CONTROL_PERIOD, control_group, enable_low_latency=True
                    # )
                    if result != 0:
                        print("保持位置失败")
                        break
                except Exception as e:
                    print(f"保持位置异常: {e}")
                    break

                time.sleep(dt)
        except KeyboardInterrupt:
            print("\n已中断保持")

def main():
    # 初始化GDK系统
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK初始化失败")
        return

    print("GDK初始化成功")

    try:
        robot = agibot_gdk.Robot()

        # 获取当前关节状态
        current_joint_states = robot.get_joint_states()

        # 定义目标位置（示例：手臂关节）
        target_positions = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # 左臂7个关节
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0   # 右臂7个关节
        ]

        print(f"\n目标角度:")
        arm_joint_names = [
            "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3",
            "idx24_arm_l_joint4", "idx25_arm_l_joint5", "idx26_arm_l_joint6",
            "idx27_arm_l_joint7",  # 左臂7个关节
            "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3",
            "idx64_arm_r_joint4", "idx65_arm_r_joint5", "idx66_arm_r_joint6",
            "idx67_arm_r_joint7"   # 右臂7个关节
        ]
        for i, joint_name in enumerate(arm_joint_names):
            print(f"  {joint_name}: {target_positions[i]:.3f} 弧度")

        # 执行手臂关节位置伺服控制
        # 该接口支持 control_mode=1（关节位控）/ 3（关节阻抗），需先确保已切换；
        # 默认通常已是位控模式，若想用关节阻抗请先 robot.set_control_mode(3)。
        controller = ArmJointServoController(robot)
        controller.execute_arm_joint_servo_control(target_positions, 2)

    except Exception as e:
        print(f"执行过程中发生错误: {e}")
    finally:
        # 释放GDK系统资源
        if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
            print("GDK释放失败")
        else:
            print("GDK释放成功")

if __name__ == "__main__":
    main()
```

#### 17. `move_ee_pos()`

- **功能**：控制末端执行器位置（夹爪开合，非阻塞式）
- **注意**：该接口不可与伺服接口同时使用，若有与伺服接口同时使用的需求请通过伺服接口控制末端执行器
- **参数**：`JointStates` 对象

**JointStates对象结构**：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `group` | `str` | 控制组，必须为 "left\_tool"、"right\_tool" 或 "dual\_tool" | 字符串 |
| `states` | `list[JointState]` | 关节状态列表（必需） | JointState列表 |
| `nums` | `int` | 关节数量，应等于 `len(states)` | 无单位 |

**各末端类型的关节位置取值范围**：

- **注意**：关节取值范围与版本绑定，使用时注意当前版本的取值范围，当前版本取值范围以文档为主
- **omnipicker**：`position` 取值范围为 `[-0.785, 0]`，其中 `-0.785` 表示打开，`0` 表示关闭
- **dahuan**：`position` 取值范围为 `[0, 0.025]`，其中 `0` 表示打开，`0.025` 表示关闭
- **ctek90d**：`position` 取值范围为 `[-0.91, 0]`，其中 `-0.91` 表示打开，`0` 表示关闭
- **o10\_t2**：各关节位置取值范围如下（单位：弧度）

**左手关节限位值**：

| 关节索引 | 关节名称 | 最小值 | 最大值 |
| --- | --- | --- | --- |
| 0 | `idx31_hand_l_thumb_roll_joint` | -1.1213740444063567 | 0.029670597283903602 |
| 1 | `idx32_hand_l_thumb_abad_joint` | -0.04537856055185257 | 1.642354826126664 |
| 2 | `idx33_hand_l_thumb_mcp_joint` | -0.8415977653116657 | 0.0 |
| 3 | `idx36_hand_l_index_abad_joint` | 0.0 | 0.16406094968746698 |
| 4 | `idx37_hand_l_index_pip_joint` | 0.0 | 1.4835298641951802 |
| 5 | `idx39_hand_l_middle_pip_joint` | 0.0 | 1.4835298641951802 |
| 6 | `idx41_hand_l_ring_abad_joint` | -0.16929693744344995 | 0.0 |
| 7 | `idx42_hand_l_ring_pip_joint` | 0.0 | 1.4835298641951802 |
| 8 | `idx44_hand_l_pinky_abad_joint` | -0.1850049007113989 | 0.0 |
| 9 | `idx45_hand_l_pinky_pip_joint` | 0.0 | 1.4835298641951802 |

**右手关节限位值**：

| 关节索引 | 关节名称 | 最小值 | 最大值 |
| --- | --- | --- | --- |
| 0 | `idx71_hand_r_thumb_roll_joint` | -0.029670597283903602 | 1.1213740444063567 |
| 1 | `idx72_hand_r_thumb_abad_joint` | -1.642354826126664 | 0.04537856055185257 |
| 2 | `idx73_hand_r_thumb_mcp_joint` | 0.0 | 0.8415977653116657 |
| 3 | `idx76_hand_r_index_abad_joint` | -0.16406094968746698 | 0.0 |
| 4 | `idx77_hand_r_index_pip_joint` | 0.0 | 1.4835298641951802 |
| 5 | `idx79_hand_r_middle_pip_joint` | 0.0 | 1.4835298641951802 |
| 6 | `idx81_hand_r_ring_abad_joint` | 0.0 | 0.16929693744344995 |
| 7 | `idx82_hand_r_ring_pip_joint` | 0.0 | 1.4835298641951802 |
| 8 | `idx84_hand_r_pinky_abad_joint` | 0.0 | 0.1850049007113989 |
| 9 | `idx85_hand_r_pinky_pip_joint` | 0.0 | 1.4835298641951802 |

**典型状态值**：
- **左手开**：`[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`
- **左手握**：`[-0.2, 1.45, -0.75, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 1.0]`
- **右手开**：`[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`
- **右手握**：`[0.2, -1.45, 0.75, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 1.0]`

- **o12\_t2**：各关节位置取值范围如下（单位：弧度）

**左手关节限位值**：

| 关节索引 | 关节名称 | 最小值 | 最大值 |
| --- | --- | --- | --- |
| 0 | `idx31_hand_l_thumb_roll_joint` | -0.9425 | 0.0 |
| 1 | `idx32_hand_l_thumb_abad_joint` | 0.0 | 1.3875 |
| 2 | `idx33_hand_l_thumb_mcp_joint` | -0.8273 | 0.0 |
| 3 | `idx34_hand_l_thumb_pip_joint` | -1.2915 | 0.0 |
| 4 | `idx36_hand_l_index_abad_joint` | -0.2618 | 0.2618 |
| 5 | `idx37_hand_l_index_mcp_joint` | 0.0 | 1.3526 |
| 6 | `idx38_hand_l_index_pip_joint` | 0.0 | 1.5307 |
| 7 | `idx40_hand_l_middle_abad_joint` | -0.2618 | 0.2618 |
| 8 | `idx41_hand_l_middle_mcp_joint` | 0.0 | 1.3579 |
| 9 | `idx42_hand_l_middle_pip_joint` | 0.0 | 1.8151 |
| 10 | `idx44_hand_l_ring_mcp_joint` | 0.0 | 1.5359 |
| 11 | `idx47_hand_l_pinky_mcp_joint` | 0.0 | 1.5359 |

**右手关节限位值**：

| 关节索引 | 关节名称 | 最小值 | 最大值 |
| --- | --- | --- | --- |
| 0 | `idx71_hand_r_thumb_roll_joint` | 0.0 | 0.9425 |
| 1 | `idx72_hand_r_thumb_abad_joint` | -1.3875 | 0.0 |
| 2 | `idx73_hand_r_thumb_mcp_joint` | -0.8273 | 0.0 |
| 3 | `idx74_hand_r_thumb_pip_joint` | -1.2915 | 0.0 |
| 4 | `idx76_hand_r_index_abad_joint` | -0.2618 | 0.2618 |
| 5 | `idx77_hand_r_index_mcp_joint` | 0.0 | 1.3526 |
| 6 | `idx78_hand_r_index_pip_joint` | 0.0 | 1.5307 |
| 7 | `idx80_hand_r_middle_abad_joint` | -0.2618 | 0.2618 |
| 8 | `idx81_hand_r_middle_mcp_joint` | 0.0 | 1.3579 |
| 9 | `idx82_hand_r_middle_pip_joint` | 0.0 | 1.8151 |
| 10 | `idx84_hand_r_ring_mcp_joint` | 0.0 | 1.5359 |
| 11 | `idx87_hand_r_pinky_mcp_joint` | 0.0 | 1.5359 |

**典型状态值**：
- **左手开**：`[-0.53, 0.42, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`
- **左手握**：`[-0.77, 0.5, -0.4, -0.36, -0.12, 0.69, 0.46, 0.0, 0.72, 0.5, 0.63, 0.63]`
- **右手开**：`[0.53, -0.42, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`
- **右手握**：`[0.77, -0.5, -0.4, -0.36, 0.12, 0.69, 0.46, 0.0, 0.72, 0.5, 0.63, 0.63]`

**JointState对象结构**：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `position` | `float` | 关节位置（必需） | 开合度 |

**参数说明**：
- `group` 必须为 "left\_tool"（左末端执行器）、"right\_tool"（右末端执行器）或 "dual\_tool"（双末端执行器）
- 当 `group` 为 "left\_tool" 或 "right\_tool" 时，`states` 列表的长度必须与末端执行器对应的关节数量要求完全匹配
- 当 `group` 为 "dual\_tool" 时，`states` 列表的长度必须为左右末端执行器对应的关节数量之和（前半部分用于左末端执行器，后半部分用于右末端执行器）
- `states` 是一个 `JointState` 对象列表，每个 `JointState` 只需要设置 `position` 字段
- `nums` 必须等于 `len(states)`

- **返回值**：`int`，0表示成功，失败时抛出异常
- **异常说明**：
- 如果 `states` 的长度与末端执行器对应的关节数量不匹配（对于 "dual\_tool"，必须是左右末端执行器关节数量之和），底层将返回 `ErrorCode::kInvalidInput`
- 如果 `group` 不是 "left\_tool"、"right\_tool" 或 "dual\_tool"，底层将返回 `ErrorCode::kInvalidInput`
- 如果 `states` 为空，底层将返回 `ErrorCode::kInvalidInput`
- 如果关节位置值超出对应末端类型的取值范围，底层将返回 `ErrorCode::kInvalidInput`
- 如果控制命令执行失败，抛出 `RuntimeError("Failed to move end effector position")`
- **示例**：

**示例1：控制左夹爪（omnipicker类型，需要1个关节）**

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

robot = agibot_gdk.Robot()

# 控制左夹爪（omnipicker类型，需要1个关节）
joint_states_left = agibot_gdk.JointStates()
joint_states_left.group = "left_tool"

joint_state = agibot_gdk.JointState()
joint_state.position = 0  # 取值范围 [-0.785, 0]
joint_states_left.states = [joint_state]
joint_states_left.nums = len(joint_states_left.states)

try:
    result = robot.move_ee_pos(joint_states_left)
    print("左夹爪控制成功")
except Exception as e:
    print(f"左夹爪控制失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

**示例2：控制右夹爪（dahuan类型，需要1个关节）**

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

robot = agibot_gdk.Robot()

# 控制右夹爪（dahuan类型，需要1个关节）
joint_states_right = agibot_gdk.JointStates()
joint_states_right.group = "right_tool"

joint_state = agibot_gdk.JointState()
joint_state.position = 0  # 右手夹爪位置，取值范围 [0, 0.025]
joint_states_right.states = [joint_state]
joint_states_right.nums = len(joint_states_right.states)

try:
    result = robot.move_ee_pos(joint_states_right)
    print("右夹爪控制成功")
except Exception as e:
    print(f"右夹爪控制失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

**示例3：控制左夹爪（ctek90d类型，需要1个关节）**

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

robot = agibot_gdk.Robot()

# 控制左夹爪（ctek90d类型，需要1个关节）
joint_states_left = agibot_gdk.JointStates()
joint_states_left.group = "left_tool"

joint_state = agibot_gdk.JointState()
joint_state.position = -0.5  # 取值范围 [-0.91, 0]
joint_states_left.states = [joint_state]
joint_states_left.nums = len(joint_states_left.states)

try:
    result = robot.move_ee_pos(joint_states_left)
    print("左夹爪（ctek90d）控制成功")
except Exception as e:
    print(f"左夹爪控制失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

**示例4：控制左末端执行器（o10\_t2类型，需要10个关节）**

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

robot = agibot_gdk.Robot()

# 控制左末端执行器（o10_t2类型，需要10个关节）
joint_states_left_o10 = agibot_gdk.JointStates()
joint_states_left_o10.group = "left_tool"

# 创建关节状态列表
positions = [-0.2, 1.45, -0.75, 0, 1, 1, 0, 1, 0, 1]
states_list = []
for i in range(10):
    joint_state = agibot_gdk.JointState()
    joint_state.position = positions[i]
    states_list.append(joint_state)
# 直接赋值整个列表
joint_states_left_o10.states = states_list
joint_states_left_o10.nums = len(joint_states_left_o10.states)

try:
    result = robot.move_ee_pos(joint_states_left_o10)
    print("左末端执行器（o10_t2）控制成功")
except Exception as e:
    print(f"左末端执行器控制失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

**示例5：控制右末端执行器（o12\_t2类型，需要12个关节）**

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

robot = agibot_gdk.Robot()

# 控制右末端执行器（o12_t2类型，需要12个关节）
joint_states_right_o12 = agibot_gdk.JointStates()
joint_states_right_o12.group = "right_tool"

# 创建关节状态列表
states_list = []
for i in range(12):
    joint_state = agibot_gdk.JointState()
    joint_state.position = 0
    states_list.append(joint_state)
# 直接赋值整个列表
joint_states_right_o12.states = states_list
joint_states_right_o12.nums = len(joint_states_right_o12.states)

try:
    result = robot.move_ee_pos(joint_states_right_o12)
    print("右末端执行器（o12_t2）控制成功")
except Exception as e:
    print(f"右末端执行器控制失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

**示例6：控制双末端执行器（dual\_tool，需要2倍关节数量）**

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

robot = agibot_gdk.Robot()

# 控制双末端执行器（dual_tool，需要2倍关节数量）
# 例如：使用 omnipicker 类型，需要 2 个关节（左1个 + 右1个）
joint_states_dual = agibot_gdk.JointStates()
joint_states_dual.group = "dual_tool"

# 前半部分用于左末端执行器
joint_state_left = agibot_gdk.JointState()
joint_state_left.position = 0
# 后半部分用于右末端执行器
joint_state_right = agibot_gdk.JointState()
joint_state_right.position = 0
# 直接赋值整个列表
joint_states_dual.states = [joint_state_left, joint_state_right]
joint_states_dual.nums = len(joint_states_dual.states)

try:
    result = robot.move_ee_pos(joint_states_dual)
    print("双末端执行器控制成功")
except Exception as e:
    print(f"双末端执行器控制失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 18. `move_end_effector_joint()`

- **功能**：控制末端执行器关节位置。
- **注意**：`group` 与关节数量需与当前末端类型一致（如 omnipicker 单手可传 1 个位置；`dual_tool` 为左右拼接，长度为单侧的 2 倍）。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `positions` | `list[float]` | 目标位置列表（ |
| `velocities` | `list[float]` | 与 `positions` 等长； |
| `group` | `str` | `left_tool`、`right_tool` 或 `dual_tool` |

**参数说明**：
- `positions` 与 `velocities` 长度必须相同，否则底层返回错误。

- **返回值**：`int`，成功为 `0`，失败时抛出 `RuntimeError`。
- **异常说明**：
- 若 `positions` 与 `velocities` 长度不一致、或 `group` 非法、或位置超限时，底层返回错误，Python 侧抛出 `RuntimeError`（信息为 `"Failed to move end effector joint "`）。
- **示例**：

**示例1：左手单关节（如 omnipicker）**

```
import agibot_gdk
import time

def main():

    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK初始化失败")
        return
    print("GDK初始化成功")

    robot = agibot_gdk.Robot()

    positions = [0.0]
    velocities = [0.0]

    try:
        robot.move_end_effector_joint(positions, velocities, "left_tool")
        print("末端关节伺服成功")
    except Exception as e:
        print(f"末端关节伺服失败: {e}")

    if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
        print("GDK释放失败")
    else:
        print("GDK释放成功")

if __name__ == "__main__":
    main()
```

**示例2：双手 `dual_tool`（以 omnipicker 为例：左 1 个关节 + 右 1 个关节，共 2 个位置）**

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

robot = agibot_gdk.Robot()

positions = [0.0, 0.0]   # 先左后右
velocities = [0.0, 0.0]  # 与 positions 等长

try:
    robot.move_end_effector_joint(positions, velocities, "dual_tool")
    print("dual_tool 末端关节伺服成功")
except Exception as e:
    print(f"dual_tool 失败: {e}")

if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 19. `end_effector_pose_control()`

- **功能**：末端执行器位姿控制（非阻塞式）
- **支持的控制模式**：`1`（关节位控）、`2`（笛卡尔阻抗）、`3`（关节阻抗），使用关节阻抗模式或笛卡尔阻抗模式前，请先通过`set_control_mode()`切换。
- **不同模式下的行为**：
- `1` / `3`（位置控制 / 关节阻抗控制）：目标位姿经运控服务全身逆解后，在关节空间执行；关节阻抗控制模式下，底层会按`set_joint_impedance()`配置的阻抗参数柔顺跟踪。
- `2`（笛卡尔阻抗控制）：目标位姿直接在笛卡尔空间执行，底层会按`set_cartesian_impedance()`配置的阻抗参数柔顺跟踪。
- 位置控制时或关节阻抗控制时，末端位姿的参考坐标系是 end\_link（末端法兰中心位置）；笛卡尔阻抗控制时，末端位姿的参考坐标系为 tcp（末端执行器中心位置）。未特别配置时，tcp 与 end\_link 默认重合。
- **注意**：
- 该接口要求发送端以50hz频率发布控制命令，不允许阶越信号；该接口无碰撞检测，注意使用时环境。该接口的控制器不提供规划能力，只适合用于末端小范围的移动，请勿在奇异点位置使用该接口。
- 支持末端位姿与夹爪关节同步控制：填入 `end_effector_joint_names` / `end_effector_joint_positions` 时同步驱动对应关节，留空则只控制末端位姿。
- **参数**：`EndEffectorPose`对象，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `life_time` | `float` | 生命周期 | 秒 |
| `group` | `int` | 控制组 | 整数 |
| `left_end_effector_pose` | `Pose` | 左末端执行器位姿（base\_link坐标系下） | 位姿 |
| `right_end_effector_pose` | `Pose` | 右末端执行器位姿（base\_link坐标系下） | 位姿 |
| `end_effector_joint_names` | `list[str]` | 同步控制的末端关节名称列表（如夹爪关节），与 `end_effector_joint_positions` 一一对应，为空时不控制末端关节 | 无单位 |
| `end_effector_joint_positions` | `list[float]` | 末端关节目标位置列表，须与 `end_effector_joint_names` 长度相同 | 弧度 |

- **返回值**：`int`，0表示成功，失败时抛出异常
- **示例**：（注意：为保证安全，该示例会将机器移动到初始位姿（站立），再调用末端位姿控制，请在确保机器人周围空间安全的情况下运行）

```
import agibot_gdk
import time
import math

# 常量定义
# 默认以法兰中心 end_link 为末端参考坐标系。
# 若需以工具坐标系 tool0（TCP）为参考，请改用 left_tool0_body / right_tool0_body，
# 并配合 switch_end_pose_control_frame()，详见本节末尾“工具坐标系（tool0）用例”。
LEFT_NAME = "arm_l_end_link"
RIGHT_NAME = "arm_r_end_link"

# 目标位姿（位置和四元数）
TARGET_LEFT_POSITION = [0.76755,0.301127306,1.08398]
TARGET_LEFT_ORIENTATION =  [0.5858436, -0.02614889,0.80973243,0.02089957]

TARGET_RIGHT_POSITION = [0.6099763,-0.3903393, 1.050190472]
TARGET_RIGHT_ORIENTATION =  [0.648815,-0.001096,0.76094223696, 0.0020416]

# 控制参数
MAX_STEP_CM = 0.1  # 最大步长（厘米）
LIFETIME = 0.02    # 生命周期（秒）
RATE_HZ = 50.0     # 发送频率（Hz）
HOLD_FINAL = True  # 是否保持最终位姿

class EndEffectorController:
    def __init__(self, robot):
        self.robot = robot

    def slerp(self, q0, q1, t):
        """
        四元数球面线性插值
        q0, q1: 四元数 [x, y, z, w]
        t: 插值参数 [0, 1]
        返回: 插值后的四元数 [x, y, z, w]
        """
        # 计算点积
        dot = q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3]

        # 如果点积为负，取反q1以确保最短路径（取反一次，后续所有分支都用 q1c）
        q1c = list(q1)
        if dot < 0.0:
            dot = -dot
            q1c = [-q1[i] for i in range(4)]

        # 限制点积范围
        dot = max(-1.0, min(1.0, dot))

        if dot > 0.9995:
            # 线性插值
            result = [q0[i] + t * (q1c[i] - q0[i]) for i in range(4)]
            norm = math.sqrt(sum(r*r for r in result))
            if norm > 0.0:
                result = [r / norm for r in result]
        else:
            # 球面线性插值
            theta_0 = math.acos(dot)
            sin_theta_0 = math.sin(theta_0)
            theta = theta_0 * t
            sin_theta = math.sin(theta)
            s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
            s1 = sin_theta / sin_theta_0

            result = [s0 * q0[i] + s1 * q1c[i] for i in range(4)]

        return result

    def distance_between_points(self, p1, p2):
        """计算两点之间的距离"""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dz = p2[2] - p1[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def calculate_n_steps(self, start_pos, goal_pos, max_step_cm):
        """计算需要的步数"""
        dist_cm = self.distance_between_points(start_pos, goal_pos) * 100.0
        return max(int(math.ceil(dist_cm / max_step_cm)), 1)

    def plan_trajectory(self, start_pose, goal_pose, n_steps):
        """
        规划轨迹
        start_pose, goal_pose: 包含position和orientation的字典
        n_steps: 步数
        返回: 轨迹列表
        """
        trajectory = []

        for i in range(n_steps):
            t = float(i) / (n_steps - 1) if n_steps > 1 else 0.0

            # 位置线性插值
            pos = [
                start_pose['position'][0] + t * (goal_pose['position'][0] - start_pose['position'][0]),
                start_pose['position'][1] + t * (goal_pose['position'][1] - start_pose['position'][1]),
                start_pose['position'][2] + t * (goal_pose['position'][2] - start_pose['position'][2])
            ]

            # 四元数SLERP插值
            q0 = start_pose['orientation']
            q1 = goal_pose['orientation']
            quat = self.slerp(q0, q1, t)

            trajectory.append({
                'position': pos,
                'orientation': quat
            })

        return trajectory

    def find_pose_by_name(self, status, target_name):
        """根据名称查找位姿"""
        for i, frame_name in enumerate(status.frame_names):
            if frame_name == target_name:
                pose = status.frame_poses[i]
                # 提取位置和四元数
                position = [pose.position.x, pose.position.y, pose.position.z]
                orientation = [pose.orientation.x, pose.orientation.y,
                              pose.orientation.z, pose.orientation.w]
                return {'position': position, 'orientation': orientation}
        raise RuntimeError(f"Frame name {target_name} not found")

    def execute_end_pose_control(self):
        """执行末端位姿控制

        end_effector_pose_control 支持 control_mode=1 / 2 / 3：
            1（关节位控）/ 3（关节阻抗）—— 经全身逆解后在关节空间执行
            2（笛卡尔阻抗）              —— 直接在笛卡尔空间执行，按 set_cartesian_impedance 配置柔顺跟踪
        默认通常已是位控模式（1）；想用关节阻抗 / 笛卡尔阻抗请先 self.robot.set_control_mode(3 / 2)。
        """
        time.sleep(1.0)  # 等待1秒

        # 获取当前状态
        status = self.robot.get_motion_control_status()

        # 获取起始位姿
        start_left_pose = self.find_pose_by_name(status, LEFT_NAME)
        start_right_pose = self.find_pose_by_name(status, RIGHT_NAME)

        # 目标位姿
        goal_left_pose = {
            'position': TARGET_LEFT_POSITION,
            'orientation': TARGET_LEFT_ORIENTATION
        }
        goal_right_pose = {
            'position': TARGET_RIGHT_POSITION,
            'orientation': TARGET_RIGHT_ORIENTATION
        }

        # 计算步数
        n_left = self.calculate_n_steps(start_left_pose['position'],
                                        goal_left_pose['position'],
                                        MAX_STEP_CM)
        n_right = self.calculate_n_steps(start_right_pose['position'],
                                        goal_right_pose['position'],
                                        MAX_STEP_CM)
        n_steps = max(n_left, n_right)

        print(f"左臂步数: {n_left}, 右臂步数: {n_right}, 总步数: {n_steps}")
        # 规划轨迹
        traj_left = self.plan_trajectory(start_left_pose, goal_left_pose, n_steps)
        traj_right = self.plan_trajectory(start_right_pose, goal_right_pose, n_steps)

        # 执行轨迹
        dt = 1.0 / RATE_HZ
        for i in range(n_steps):
            # 创建末端执行器位姿控制请求
            end_pose = agibot_gdk.EndEffectorPose()
            end_pose.life_time = LIFETIME
            end_pose.group = agibot_gdk.EndEffectorControlGroup.kBothArms

            # 设置左臂位姿
            end_pose.left_end_effector_pose.position.x = traj_left[i]['position'][0]
            end_pose.left_end_effector_pose.position.y = traj_left[i]['position'][1]
            end_pose.left_end_effector_pose.position.z = traj_left[i]['position'][2]
            end_pose.left_end_effector_pose.orientation.x = traj_left[i]['orientation'][0]
            end_pose.left_end_effector_pose.orientation.y = traj_left[i]['orientation'][1]
            end_pose.left_end_effector_pose.orientation.z = traj_left[i]['orientation'][2]
            end_pose.left_end_effector_pose.orientation.w = traj_left[i]['orientation'][3]

            # 设置右臂位姿
            end_pose.right_end_effector_pose.position.x = traj_right[i]['position'][0]
            end_pose.right_end_effector_pose.position.y = traj_right[i]['position'][1]
            end_pose.right_end_effector_pose.position.z = traj_right[i]['position'][2]
            end_pose.right_end_effector_pose.orientation.x = traj_right[i]['orientation'][0]
            end_pose.right_end_effector_pose.orientation.y = traj_right[i]['orientation'][1]
            end_pose.right_end_effector_pose.orientation.z = traj_right[i]['orientation'][2]
            end_pose.right_end_effector_pose.orientation.w = traj_right[i]['orientation'][3]

            try:
                result = self.robot.end_effector_pose_control(end_pose)
                if result != 0:
                    print(f"控制命令发送失败，步数: {i}")
                    return
                print(f"控制命令发送成功，步数: {i}")
            except Exception as e:
                print(f"控制命令发送异常，步数: {i}, 错误: {e}")
                return

            time.sleep(dt)

        # 保持最终位姿
        if HOLD_FINAL:
            print("进入末端位姿保持（Ctrl+C 结束）...")
            try:
                final_left = traj_left[-1]
                final_right = traj_right[-1]

                while True:
                    end_pose = agibot_gdk.EndEffectorPose()
                    end_pose.life_time = LIFETIME
                    end_pose.group = agibot_gdk.EndEffectorControlGroup.kBothArms

                    # 设置左臂位姿
                    end_pose.left_end_effector_pose.position.x = final_left['position'][0]
                    end_pose.left_end_effector_pose.position.y = final_left['position'][1]
                    end_pose.left_end_effector_pose.position.z = final_left['position'][2]
                    end_pose.left_end_effector_pose.orientation.x = final_left['orientation'][0]
                    end_pose.left_end_effector_pose.orientation.y = final_left['orientation'][1]
                    end_pose.left_end_effector_pose.orientation.z = final_left['orientation'][2]
                    end_pose.left_end_effector_pose.orientation.w = final_left['orientation'][3]

                    # 设置右臂位姿
                    end_pose.right_end_effector_pose.position.x = final_right['position'][0]
                    end_pose.right_end_effector_pose.position.y = final_right['position'][1]
                    end_pose.right_end_effector_pose.position.z = final_right['position'][2]
                    end_pose.right_end_effector_pose.orientation.x = final_right['orientation'][0]
                    end_pose.right_end_effector_pose.orientation.y = final_right['orientation'][1]
                    end_pose.right_end_effector_pose.orientation.z = final_right['orientation'][2]
                    end_pose.right_end_effector_pose.orientation.w = final_right['orientation'][3]

                    try:
                        result = self.robot.end_effector_pose_control(end_pose)
                        if result != 0:
                            print("保持位姿失败")
                            break
                    except Exception as e:
                        print(f"保持位姿异常: {e}")
                        break

                    time.sleep(dt)
            except KeyboardInterrupt:
                print("\n已中断保持")

def main():
    # 初始化GDK系统
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK初始化失败")
        return

    print("GDK初始化成功")

    try:
        robot = agibot_gdk.Robot()

        print("正在移动到初始位姿...")
        joint_control_req = agibot_gdk.JointControlReq()
        joint_control_req.joint_names = [
            "idx01_body_joint1", "idx02_body_joint2", "idx03_body_joint3", "idx04_body_joint4", "idx05_body_joint5",
            "idx11_head_joint1", "idx12_head_joint2", "idx13_head_joint3",
            "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3",
            "idx24_arm_l_joint4", "idx25_arm_l_joint5", "idx26_arm_l_joint6",
            "idx27_arm_l_joint7",  # 左臂7个关节
            "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3",
            "idx64_arm_r_joint4", "idx65_arm_r_joint5", "idx66_arm_r_joint6",
            "idx67_arm_r_joint7"   # 右臂7个关节
        ]
        joint_control_req.joint_positions = [-0.12526109443163597,
                                            0.8407635483115572,
                                            -0.971927299903638,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0.0036426282540360732,
                                            1.7301117664332919,
                                            -1.1500181062743811,
                                            -1.5999388458662482,
                                            -1.79993398793182,
                                            -0.41992232715148303,
                                            5.8962386534354331e-05,
                                            3.8709046444301724e-05,
                                            -1.7301030279359648,
                                            -1.1500377304426637,
                                            1.5999483134039236,
                                            -1.6252835640650924,
                                            0.42000178256260562,
                                            1.3781858641160056e-05,
                                            3.7151097206605374e-06]
        joint_control_req.joint_velocities = [0.3] * 22 # 设置速度
        joint_control_req.life_time = 5.0

        result = robot.joint_control_request(joint_control_req)
        if result != 0:
            print("移动到初始位姿失败")
            return
        print("移动到初始位姿成功，等待到位...")
        time.sleep(2)

        controller = EndEffectorController(robot)
        controller.execute_end_pose_control()

    except Exception as e:
        print(f"执行过程中发生错误: {e}")
    finally:
        # 释放GDK系统资源
        if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
            print("GDK释放失败")
        else:
            print("GDK释放成功")

if __name__ == "__main__":
    main()
```

- **工具坐标系（`tool0`）用例**：上面的示例以 `arm_l_end_link` / `arm_r_end_link`（法兰中心 end\_link）为末端参考坐标系。下面是以\*\*工具坐标系\*\* `left_tool0_body` / `right_tool0_body`（即 TCP）为参考点进行\*\*笛卡尔阻抗\*\*末端位姿控制的\*\*完整示例\*\*。与原示例的区别集中在 4 处（已在代码中用 `# 区别 N` 标注），轨迹规划与下发逻辑完全一致：

```
import agibot_gdk
import time
import math

# 常量定义
# 区别 1：末端参考坐标系改用工具坐标系 tool0
#         （原示例为 arm_l_end_link / arm_r_end_link）
LEFT_NAME = "left_tool0_body"
RIGHT_NAME = "right_tool0_body"

# 目标位姿（位置和四元数）——请按 tool0（TCP）坐标系下的期望位姿设置
TARGET_LEFT_POSITION = [0.626755, 0.301127306, 1.08398]
TARGET_LEFT_ORIENTATION = [0.5858436, -0.02614889, 0.80973243, 0.02089957]

TARGET_RIGHT_POSITION = [0.6099763, -0.3903393, 1.050190472]
TARGET_RIGHT_ORIENTATION = [0.648815, -0.001096, 0.76094223696, 0.0020416]

# 控制参数
MAX_STEP_CM = 0.1  # 最大步长（厘米）
LIFETIME = 0.02    # 生命周期（秒）
RATE_HZ = 50.0     # 发送频率（Hz）
HOLD_FINAL = True  # 是否保持最终位姿

class EndEffectorController:
    def __init__(self, robot):
        self.robot = robot

    def slerp(self, q0, q1, t):
        """
        四元数球面线性插值
        q0, q1: 四元数 [x, y, z, w]
        t: 插值参数 [0, 1]
        返回: 插值后的四元数 [x, y, z, w]
        """
        # 计算点积
        dot = q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3]

        # 如果点积为负，取反q1以确保最短路径（取反一次，后续所有分支都用 q1c）
        q1c = list(q1)
        if dot < 0.0:
            dot = -dot
            q1c = [-q1[i] for i in range(4)]

        # 限制点积范围
        dot = max(-1.0, min(1.0, dot))

        if dot > 0.9995:
            # 线性插值
            result = [q0[i] + t * (q1c[i] - q0[i]) for i in range(4)]
            norm = math.sqrt(sum(r*r for r in result))
            if norm > 0.0:
                result = [r / norm for r in result]
        else:
            # 球面线性插值
            theta_0 = math.acos(dot)
            sin_theta_0 = math.sin(theta_0)
            theta = theta_0 * t
            sin_theta = math.sin(theta)
            s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
            s1 = sin_theta / sin_theta_0

            result = [s0 * q0[i] + s1 * q1c[i] for i in range(4)]

        return result

    def distance_between_points(self, p1, p2):
        """计算两点之间的距离"""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dz = p2[2] - p1[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def calculate_n_steps(self, start_pos, goal_pos, max_step_cm):
        """计算需要的步数"""
        dist_cm = self.distance_between_points(start_pos, goal_pos) * 100.0
        return max(int(math.ceil(dist_cm / max_step_cm)), 1)

    def plan_trajectory(self, start_pose, goal_pose, n_steps):
        """
        规划轨迹
        start_pose, goal_pose: 包含position和orientation的字典
        n_steps: 步数
        返回: 轨迹列表
        """
        trajectory = []

        for i in range(n_steps):
            t = float(i) / (n_steps - 1) if n_steps > 1 else 0.0

            # 位置线性插值
            pos = [
                start_pose['position'][0] + t * (goal_pose['position'][0] - start_pose['position'][0]),
                start_pose['position'][1] + t * (goal_pose['position'][1] - start_pose['position'][1]),
                start_pose['position'][2] + t * (goal_pose['position'][2] - start_pose['position'][2])
            ]

            # 四元数SLERP插值
            q0 = start_pose['orientation']
            q1 = goal_pose['orientation']
            quat = self.slerp(q0, q1, t)

            trajectory.append({
                'position': pos,
                'orientation': quat
            })

        return trajectory

    def find_pose_by_name(self, status, target_name):
        """根据名称查找位姿"""
        for i, frame_name in enumerate(status.frame_names):
            if frame_name == target_name:
                pose = status.frame_poses[i]
                # 提取位置和四元数
                position = [pose.position.x, pose.position.y, pose.position.z]
                orientation = [pose.orientation.x, pose.orientation.y,
                              pose.orientation.z, pose.orientation.w]
                return {'position': position, 'orientation': orientation}
        raise RuntimeError(f"Frame name {target_name} not found")

    def execute_end_pose_control(self):
        """执行末端位姿控制（工具坐标系 tool0 / 笛卡尔阻抗）"""
        time.sleep(1.0)  # 等待1秒

        # 获取当前状态
        status = self.robot.get_motion_control_status()

        # 获取起始位姿（LEFT_NAME / RIGHT_NAME 已指向 tool0）
        start_left_pose = self.find_pose_by_name(status, LEFT_NAME)
        start_right_pose = self.find_pose_by_name(status, RIGHT_NAME)

        # 目标位姿
        goal_left_pose = {
            'position': TARGET_LEFT_POSITION,
            'orientation': TARGET_LEFT_ORIENTATION
        }
        goal_right_pose = {
            'position': TARGET_RIGHT_POSITION,
            'orientation': TARGET_RIGHT_ORIENTATION
        }

        # 计算步数
        n_left = self.calculate_n_steps(start_left_pose['position'],
                                        goal_left_pose['position'],
                                        MAX_STEP_CM)
        n_right = self.calculate_n_steps(start_right_pose['position'],
                                        goal_right_pose['position'],
                                        MAX_STEP_CM)
        n_steps = max(n_left, n_right)

        print(f"左臂步数: {n_left}, 右臂步数: {n_right}, 总步数: {n_steps}")
        # 规划轨迹
        traj_left = self.plan_trajectory(start_left_pose, goal_left_pose, n_steps)
        traj_right = self.plan_trajectory(start_right_pose, goal_right_pose, n_steps)

        # 执行轨迹
        dt = 1.0 / RATE_HZ
        for i in range(n_steps):
            # 创建末端执行器位姿控制请求
            end_pose = agibot_gdk.EndEffectorPose()
            end_pose.life_time = LIFETIME
            end_pose.group = agibot_gdk.EndEffectorControlGroup.kBothArms

            # 设置左臂位姿
            end_pose.left_end_effector_pose.position.x = traj_left[i]['position'][0]
            end_pose.left_end_effector_pose.position.y = traj_left[i]['position'][1]
            end_pose.left_end_effector_pose.position.z = traj_left[i]['position'][2]
            end_pose.left_end_effector_pose.orientation.x = traj_left[i]['orientation'][0]
            end_pose.left_end_effector_pose.orientation.y = traj_left[i]['orientation'][1]
            end_pose.left_end_effector_pose.orientation.z = traj_left[i]['orientation'][2]
            end_pose.left_end_effector_pose.orientation.w = traj_left[i]['orientation'][3]

            # 设置右臂位姿
            end_pose.right_end_effector_pose.position.x = traj_right[i]['position'][0]
            end_pose.right_end_effector_pose.position.y = traj_right[i]['position'][1]
            end_pose.right_end_effector_pose.position.z = traj_right[i]['position'][2]
            end_pose.right_end_effector_pose.orientation.x = traj_right[i]['orientation'][0]
            end_pose.right_end_effector_pose.orientation.y = traj_right[i]['orientation'][1]
            end_pose.right_end_effector_pose.orientation.z = traj_right[i]['orientation'][2]
            end_pose.right_end_effector_pose.orientation.w = traj_right[i]['orientation'][3]

            try:
                result = self.robot.end_effector_pose_control(end_pose)
                if result != 0:
                    print(f"控制命令发送失败，步数: {i}")
                    return
                print(f"控制命令发送成功，步数: {i}")
            except Exception as e:
                print(f"控制命令发送异常，步数: {i}, 错误: {e}")
                return

            time.sleep(dt)

        # 保持最终位姿
        if HOLD_FINAL:
            print("进入末端位姿保持（Ctrl+C 结束）...")
            try:
                final_left = traj_left[-1]
                final_right = traj_right[-1]

                while True:
                    end_pose = agibot_gdk.EndEffectorPose()
                    end_pose.life_time = LIFETIME
                    end_pose.group = agibot_gdk.EndEffectorControlGroup.kBothArms

                    # 设置左臂位姿
                    end_pose.left_end_effector_pose.position.x = final_left['position'][0]
                    end_pose.left_end_effector_pose.position.y = final_left['position'][1]
                    end_pose.left_end_effector_pose.position.z = final_left['position'][2]
                    end_pose.left_end_effector_pose.orientation.x = final_left['orientation'][0]
                    end_pose.left_end_effector_pose.orientation.y = final_left['orientation'][1]
                    end_pose.left_end_effector_pose.orientation.z = final_left['orientation'][2]
                    end_pose.left_end_effector_pose.orientation.w = final_left['orientation'][3]

                    # 设置右臂位姿
                    end_pose.right_end_effector_pose.position.x = final_right['position'][0]
                    end_pose.right_end_effector_pose.position.y = final_right['position'][1]
                    end_pose.right_end_effector_pose.position.z = final_right['position'][2]
                    end_pose.right_end_effector_pose.orientation.x = final_right['orientation'][0]
                    end_pose.right_end_effector_pose.orientation.y = final_right['orientation'][1]
                    end_pose.right_end_effector_pose.orientation.z = final_right['orientation'][2]
                    end_pose.right_end_effector_pose.orientation.w = final_right['orientation'][3]

                    try:
                        result = self.robot.end_effector_pose_control(end_pose)
                        if result != 0:
                            print("保持位姿失败")
                            break
                    except Exception as e:
                        print(f"保持位姿异常: {e}")
                        break

                    time.sleep(dt)
            except KeyboardInterrupt:
                print("\n已中断保持")

def main():
    # 初始化GDK系统
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK初始化失败")
        return

    print("GDK初始化成功")

    try:
        robot = agibot_gdk.Robot()

        print("正在移动到初始位姿...")
        joint_control_req = agibot_gdk.JointControlReq()
        joint_control_req.joint_names = [
            "idx01_body_joint1", "idx02_body_joint2", "idx03_body_joint3", "idx04_body_joint4", "idx05_body_joint5",
            "idx11_head_joint1", "idx12_head_joint2", "idx13_head_joint3",
            "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3",
            "idx24_arm_l_joint4", "idx25_arm_l_joint5", "idx26_arm_l_joint6",
            "idx27_arm_l_joint7",  # 左臂7个关节
            "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3",
            "idx64_arm_r_joint4", "idx65_arm_r_joint5", "idx66_arm_r_joint6",
            "idx67_arm_r_joint7"   # 右臂7个关节
        ]
        joint_control_req.joint_positions = [-0.12526109443163597,
                                            0.8407635483115572,
                                            -0.971927299903638,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0.0036426282540360732,
                                            1.7301117664332919,
                                            -1.1500181062743811,
                                            -1.5999388458662482,
                                            -1.79993398793182,
                                            -0.41992232715148303,
                                            5.8962386534354331e-05,
                                            3.8709046444301724e-05,
                                            -1.7301030279359648,
                                            -1.1500377304426637,
                                            1.5999483134039236,
                                            -1.6252835640650924,
                                            0.42000178256260562,
                                            1.3781858641160056e-05,
                                            3.7151097206605374e-06]
        joint_control_req.joint_velocities = [0.3] * 22  # 设置速度
        joint_control_req.life_time = 5.0

        result = robot.joint_control_request(joint_control_req)
        if result != 0:
            print("移动到初始位姿失败")
            return
        print("移动到初始位姿成功，等待到位...")
        time.sleep(2)

        # 区别 2（可选）：配置 TCP 相对法兰 end_link 的变换，
        # 例如沿 Z 轴前移 0.1m；不配置则 tool0 与 end_link 默认重合。
        # 注意：为确保安全，禁止在笛卡尔阻抗模式下设置 TCP；
        #       必须先设置 TCP 再 set_control_mode(2)，否则 TCP 设置会失败。
        tcp = agibot_gdk.DualArmTcpTransform()
        tcp.left_arm.position = [0.0, 0.0, 0.1]
        tcp.left_arm.euler_zyx = [0.0, 0.0, 0.0]
        tcp.right_arm.position = [0.0, 0.0, 0.1]
        tcp.right_arm.euler_zyx = [0.0, 0.0, 0.0]
        robot.set_tcp_transform(tcp)

        # 区别 3：进入笛卡尔阻抗控制模式（tool_frame 仅在该模式下生效）
        robot.set_control_mode(2)

        # 区别 4：将目标坐标系切换到 tool0（left_tool0_body / right_tool0_body）。
        # 切换后 TCP 设置才会在末端位姿控制中生效；日志会打印当前生效的坐标系
        robot.switch_end_pose_control_frame()

        controller = EndEffectorController(robot)
        controller.execute_end_pose_control()

    except Exception as e:
        print(f"执行过程中发生错误: {e}")
    finally:
        # 释放GDK系统资源
        if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
            print("GDK释放失败")
        else:
            print("GDK释放成功")

if __name__ == "__main__":
    main()
```

- **同时控制夹爪用例**：在末端位姿控制的同时，通过 `end_effector_joint_names` / `end_effector_joint_positions` 字段同步驱动夹爪关节。夹爪关节名从 `get_end_state()` 动态获取（与 `joint_servo_control()` 同时控制末端示例保持一致），末端行程范围同样取 `-0.785`（下限）↔ `0.0`（上限）。与纯位姿示例相比，新增部分已用注释标出。

```
import agibot_gdk
import time
import math

LEFT_NAME  = "arm_l_end_link"
RIGHT_NAME = "arm_r_end_link"

# 新增：末端行程范围（与 joint_servo_control 同时控制末端示例保持一致）
EE_LOW  = -0.785  # 末端行程下限
EE_HIGH =  0.0    # 末端行程上限

TARGET_LEFT_POSITION    = [0.76755, 0.301127306, 1.08398]
TARGET_LEFT_ORIENTATION = [0.5858436, -0.02614889, 0.80973243, 0.02089957]

TARGET_RIGHT_POSITION    = [0.6099763, -0.3903393, 1.050190472]
TARGET_RIGHT_ORIENTATION = [0.648815, -0.001096, 0.76094223696, 0.0020416]

MAX_STEP_CM = 0.1
LIFETIME    = 0.02
RATE_HZ     = 50.0
HOLD_FINAL  = True

# 新增：从 get_end_state() 中取出指定侧末端关节名与当前位置（同 joint_servo_control 示例）
def get_ee_names_and_positions(end_state, side="left"):
    key = f"{side}_end_state"
    state = end_state[key]
    names     = state.get("names", [])
    positions = [s["position"] for s in state.get("end_states", [])]
    return names, positions

class EndEffectorWithGripperController:
    def __init__(self, robot):
        self.robot = robot

    def slerp(self, q0, q1, t):
        dot = sum(q0[i]*q1[i] for i in range(4))
        q1c = list(q1)
        if dot < 0.0:
            dot = -dot
            q1c = [-v for v in q1]
        dot = max(-1.0, min(1.0, dot))
        if dot > 0.9995:
            result = [q0[i] + t * (q1c[i] - q0[i]) for i in range(4)]
            norm = math.sqrt(sum(r*r for r in result))
            return [r / norm for r in result] if norm > 0 else result
        theta_0 = math.acos(dot)
        s0 = math.cos(theta_0 * t) - dot * math.sin(theta_0 * t) / math.sin(theta_0)
        s1 = math.sin(theta_0 * t) / math.sin(theta_0)
        return [s0 * q0[i] + s1 * q1c[i] for i in range(4)]

    def calculate_n_steps(self, p0, p1, max_step_cm):
        dist_cm = math.sqrt(sum((p1[i]-p0[i])**2 for i in range(3))) * 100.0
        return max(int(math.ceil(dist_cm / max_step_cm)), 1)

    def plan_trajectory(self, start, goal, n_steps):
        traj = []
        for i in range(n_steps):
            t = float(i) / (n_steps - 1) if n_steps > 1 else 0.0
            pos  = [start['position'][j]  + t * (goal['position'][j]  - start['position'][j])  for j in range(3)]
            quat = self.slerp(start['orientation'], goal['orientation'], t)
            traj.append({'position': pos, 'orientation': quat})
        return traj

    def find_pose_by_name(self, status, name):
        for i, n in enumerate(status.frame_names):
            if n == name:
                p = status.frame_poses[i]
                return {'position': [p.position.x, p.position.y, p.position.z],
                        'orientation': [p.orientation.x, p.orientation.y,
                                        p.orientation.z, p.orientation.w]}
        raise RuntimeError(f"Frame not found: {name}")

    def execute(self):
        time.sleep(1.0)

        status = self.robot.get_motion_control_status()

        # 新增：获取末端关节名与当前位置（左侧 + 右侧）
        end_state = self.robot.get_end_state()
        ee_names_l, ee_pos_l = get_ee_names_and_positions(end_state, "left")
        ee_names_r, ee_pos_r = get_ee_names_and_positions(end_state, "right")

        # 合并左右夹爪名称与起始位置
        gripper_names = ee_names_l + ee_names_r
        gripper_start = ee_pos_l  + ee_pos_r
        # 目标：夹爪运动至 EE_LOW
        gripper_target = [EE_LOW] * len(gripper_names)

        start_left  = self.find_pose_by_name(status, LEFT_NAME)
        start_right = self.find_pose_by_name(status, RIGHT_NAME)
        goal_left   = {'position': TARGET_LEFT_POSITION,  'orientation': TARGET_LEFT_ORIENTATION}
        goal_right  = {'position': TARGET_RIGHT_POSITION, 'orientation': TARGET_RIGHT_ORIENTATION}

        n_steps = max(
            self.calculate_n_steps(start_left['position'],  goal_left['position'],  MAX_STEP_CM),
            self.calculate_n_steps(start_right['position'], goal_right['position'], MAX_STEP_CM),
            int(RATE_HZ * 1.0),  # 至少 1s
        )
        traj_left  = self.plan_trajectory(start_left,  goal_left,  n_steps)
        traj_right = self.plan_trajectory(start_right, goal_right, n_steps)

        dt = 1.0 / RATE_HZ
        for i in range(n_steps):
            t = float(i) / (n_steps - 1)

            end_pose = agibot_gdk.EndEffectorPose()
            end_pose.life_time = LIFETIME
            end_pose.group     = agibot_gdk.EndEffectorControlGroup.kBothArms

            end_pose.left_end_effector_pose.position.x    = traj_left[i]['position'][0]
            end_pose.left_end_effector_pose.position.y    = traj_left[i]['position'][1]
            end_pose.left_end_effector_pose.position.z    = traj_left[i]['position'][2]
            end_pose.left_end_effector_pose.orientation.x = traj_left[i]['orientation'][0]
            end_pose.left_end_effector_pose.orientation.y = traj_left[i]['orientation'][1]
            end_pose.left_end_effector_pose.orientation.z = traj_left[i]['orientation'][2]
            end_pose.left_end_effector_pose.orientation.w = traj_left[i]['orientation'][3]

            end_pose.right_end_effector_pose.position.x    = traj_right[i]['position'][0]
            end_pose.right_end_effector_pose.position.y    = traj_right[i]['position'][1]
            end_pose.right_end_effector_pose.position.z    = traj_right[i]['position'][2]
            end_pose.right_end_effector_pose.orientation.x = traj_right[i]['orientation'][0]
            end_pose.right_end_effector_pose.orientation.y = traj_right[i]['orientation'][1]
            end_pose.right_end_effector_pose.orientation.z = traj_right[i]['orientation'][2]
            end_pose.right_end_effector_pose.orientation.w = traj_right[i]['orientation'][3]

            # 新增：同步驱动夹爪，随轨迹进度从当前位置线性插值到 EE_LOW
            end_pose.end_effector_joint_names = gripper_names
            end_pose.end_effector_joint_positions = [
                gripper_start[j] + t * (gripper_target[j] - gripper_start[j])
                for j in range(len(gripper_names))
            ]

            try:
                result = self.robot.end_effector_pose_control(end_pose)
                if result != 0:
                    print(f"控制命令发送失败，步数: {i}")
                    return
            except Exception as e:
                print(f"控制命令发送异常，步数: {i}, 错误: {e}")
                return

            time.sleep(dt)

        if HOLD_FINAL:
            print("进入末端+夹爪保持（Ctrl+C 结束）...")
            try:
                while True:
                    end_pose = agibot_gdk.EndEffectorPose()
                    end_pose.life_time = LIFETIME
                    end_pose.group     = agibot_gdk.EndEffectorControlGroup.kBothArms

                    final_l = traj_left[-1]
                    final_r = traj_right[-1]
                    end_pose.left_end_effector_pose.position.x    = final_l['position'][0]
                    end_pose.left_end_effector_pose.position.y    = final_l['position'][1]
                    end_pose.left_end_effector_pose.position.z    = final_l['position'][2]
                    end_pose.left_end_effector_pose.orientation.x = final_l['orientation'][0]
                    end_pose.left_end_effector_pose.orientation.y = final_l['orientation'][1]
                    end_pose.left_end_effector_pose.orientation.z = final_l['orientation'][2]
                    end_pose.left_end_effector_pose.orientation.w = final_l['orientation'][3]
                    end_pose.right_end_effector_pose.position.x    = final_r['position'][0]
                    end_pose.right_end_effector_pose.position.y    = final_r['position'][1]
                    end_pose.right_end_effector_pose.position.z    = final_r['position'][2]
                    end_pose.right_end_effector_pose.orientation.x = final_r['orientation'][0]
                    end_pose.right_end_effector_pose.orientation.y = final_r['orientation'][1]
                    end_pose.right_end_effector_pose.orientation.z = final_r['orientation'][2]
                    end_pose.right_end_effector_pose.orientation.w = final_r['orientation'][3]
                    # 新增：保持夹爪目标位置（EE_LOW）
                    end_pose.end_effector_joint_names      = gripper_names
                    end_pose.end_effector_joint_positions  = gripper_target

                    self.robot.end_effector_pose_control(end_pose)
                    time.sleep(dt)
            except KeyboardInterrupt:
                print("\n已中断保持")

def main():
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK初始化失败")
        return
    try:
        robot = agibot_gdk.Robot()
        controller = EndEffectorWithGripperController(robot)
        controller.execute()
    except Exception as e:
        print(f"执行过程中发生错误: {e}")
    finally:
        agibot_gdk.gdk_release()

if __name__ == "__main__":
    main()
```

#### 20. `get_end_pose_control_frame()`

- **功能**：获取 `end_effector_pose_control()` 在\*\*笛卡尔阻抗控制（`control_mode=2`）\*\*下使用的目标参考坐标系。
- **参数**：无
- **返回值**：`list[str]`，当前目标参考坐标系名称，按左臂 / 右臂顺序返回；失败时抛出异常。
- **说明**：
- 默认返回 `arm_l_end_link` / `arm_r_end_link`（末端法兰中心）。
- 调用 [`switch_end_pose_control_frame()`](#21-switch_end_pose_control_frame) 切换到工具坐标系后，返回 `left_tool0_body` / `right_tool0_body`。
- 该接口仅查询当前配置的目标坐标系，不会改变控制状态。
- **示例**：

```
import agibot_gdk

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(-1)

try:
    robot = agibot_gdk.Robot()
    frame_names = robot.get_end_pose_control_frame()
    print("left frame:", frame_names[0])
    print("right frame:", frame_names[1])
except Exception as e:
    print(f"获取末端位姿控制坐标系失败: {e}")
finally:
    agibot_gdk.gdk_release()
```

#### 21. `switch_end_pose_control_frame()`

- **功能**：切换 `end_effector_pose_control()` 在\*\*笛卡尔阻抗控制（`control_mode=2`）\*\*下使用的目标参考坐标系。每次调用在以下两组坐标系之间切换，并在日志中打印切换后当前使用的坐标系：
- `arm_l_end_link` / `arm_r_end_link`（默认，末端法兰中心）
- `left_tool0_body` / `right_tool0_body`（末端执行器/工具坐标系）
- **参数**：无
- **返回值**：`int`，成功返回 `0`；失败抛出异常。
- **说明**：
- 仅影响笛卡尔阻抗控制（`control_mode=2`）下的目标坐标系；关节位控（`1`）/ 关节阻抗（`3`）模式不受影响。
- 切换后会在日志中打印当前生效的坐标系，也可以调用 [`get_end_pose_control_frame()`](#20-get_end_pose_control_frame) 查询当前坐标系。
- **示例**：

```
import agibot_gdk

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(-1)

try:
    robot = agibot_gdk.Robot()

    # 切换笛卡尔阻抗末端位姿控制的目标坐标系，
    # 在 arm_l_end_link/arm_r_end_link 与 left_tool0_body/right_tool0_body 间切换
    robot.switch_end_pose_control_frame()
    print("已切换末端位姿控制坐标系（详见日志）")
except Exception as e:
    print(f"切换末端位姿控制坐标系失败: {e}")
finally:
    agibot_gdk.gdk_release()
```

#### 22. `set_control_mode()`

- **功能**：设置机器人运动控制模式（同步阻塞式）。关于各模式的含义与工作流，参见[关于运动控制模式的说明](#关于运动控制模式的说明)。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `control_mode` | `int`（uint8） | 控制模式整数值，仅允许 ½/3，见下方说明 |

- **返回值**：`int`，0 表示成功；请求超时或被底层拒绝时抛出异常。

#### control\_mode 取值说明

`control_mode` 仅允许以下整数：

| 数值 | 描述 |
| --- | --- |
| 1 | 关节位控（PD/轨迹） |
| 2 | 笛卡尔阻抗 / 笛卡尔力控 |
| 3 | 关节阻抗 / 关节力控 |

- **示例**：切换到笛卡尔力控模式

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

robot = agibot_gdk.Robot()

# 1) 先切换到笛卡尔力控模式
try:
    robot.set_control_mode(2)
    print("已进入笛卡尔力控模式，可通过end_effector_pose_control下发目标位姿")
except Exception as e:
    print(f"切换到笛卡尔力控模式失败: {e}")
    agibot_gdk.gdk_release()
    exit(1)

# 2) 设置笛卡尔阻抗参数
# 双臂共 12 维，仅 kp 生效；前 6 维为左臂，后 6 维为右臂
# 每臂顺序为 x, y, z, roll, pitch, yaw
# x/y/z 推荐范围: 600~6000 N/m；roll/pitch/yaw 推荐范围: 30~300 N·m/rad
single_arm_kp = [1000.0, 1000.0, 1000.0, 50.0, 50.0, 50.0]
dual_arm_kp = single_arm_kp + single_arm_kp
impedances = [agibot_gdk.Impedance(kp=kp) for kp in dual_arm_kp]
try:
    robot.set_cartesian_impedance(impedances, 2)
except Exception as e:
    print(f"设置笛卡尔阻抗失败: {e}")

agibot_gdk.gdk_release()
```

#### 23. `get_cartesian_impedance()`

- **功能**：获取笛卡尔阻抗参数
- **参数**：无
- **返回值**：`list[Impedance]`，每个笛卡尔轴的阻抗参数

**Impedance对象详细说明**：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| `kp` | `float` | 刚度系数 |
| `ki` | `float` | 积分系数，当前实现暂未启用 |
| `kv` | `float` | 阻尼系数（速度反馈），当前实现暂未启用 |

- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

robot = agibot_gdk.Robot()

impedances = robot.get_cartesian_impedance()
for i, imp in enumerate(impedances):
    print(f"axis {i}: kp={imp.kp}, ki={imp.ki}, kv={imp.kv}")

agibot_gdk.gdk_release()
```

#### 24. `set_cartesian_impedance()`

- **功能**：设置笛卡尔阻抗参数（必须先切换到 `control_mode=2`（笛卡尔阻抗）后再调用）。当前实现仅支持设置 `kp`（刚度系数），`ki` / `kv` 暂未启用。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `impedances` | `list[Impedance]` | 每个笛卡尔轴的阻抗参数；当前实现仅读取并应用 `kp`，`ki` / `kv` 字段会被忽略。`control_group=0/1` 时长度为 6（单臂），`control_group=2` 时长度为 12（双臂，前 6 个为左臂，后 6 个为右臂） |
| `control_group` | `int` | 控制组，0: 左臂，1: 右臂，2: 双臂 |

- **返回值**：`int`，0 表示成功，失败时抛出异常
- **kp 取值说明**：

单臂的 6 维 `kp` 按顺序对应笛卡尔末端的 **`x, y, z, roll, pitch, yaw`**（前三维为平动、后三维为转动）。建议取值范围：

| 维度 | 物理意义 | 下限 | 上限 | 单位 |
| --- | --- | --- | --- | --- |
| `x` / `y` / `z` | 平动刚度 | 600 | 6000 | N/m |
| `roll` / `pitch` / `yaw` | 转动刚度 | 30 | 300 | N·m/rad |

即单臂 `kp` 下限为 `(600, 600, 600, 30, 30, 30)`，上限为 `(6000, 6000, 6000, 300, 300, 300)`。`control_group=2`（双臂）时长度为 12，前 6 维为左臂、后 6 维为右臂，每一臂分别遵循上述顺序与范围。

- `kp` 越大刚度越高，对位姿误差的恢复力越强，但抗冲击/柔顺性下降；
- `kp` 越小越柔顺，适合接触类任务（贴合、装配等），但对扰动跟踪偏差更大；
- 超出推荐范围可能引发系统振荡或柔顺度严重不足，请按工况在区间内调试。
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

robot = agibot_gdk.Robot()

# 先切换到笛卡尔阻抗控制模式（control_mode=2），再设置对应参数
robot.set_control_mode(2)

# 双臂共 12 维，仅 kp 生效；前 6 维为左臂，后 6 维为右臂
# 每臂顺序为 x, y, z, roll, pitch, yaw
# x/y/z 推荐范围: 600~6000 N/m；roll/pitch/yaw 推荐范围: 30~300 N·m/rad
single_arm_kp = [1000.0, 1000.0, 1000.0, 50.0, 50.0, 50.0]
dual_arm_kp = single_arm_kp + single_arm_kp
impedances = [agibot_gdk.Impedance(kp=kp) for kp in dual_arm_kp]
robot.set_cartesian_impedance(impedances, 2)

agibot_gdk.gdk_release()
```

#### 25. `get_collision_detection_config()`

- **功能**：获取碰撞检测配置
- **参数**：无
- **返回值**：`CollisionDetectionConfig` 对象

**CollisionDetectionConfig对象详细说明**：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| `is_enabled` | `bool` | 是否启用碰撞检测 |
| `sensitivity` | `int` | 灵敏度等级，1-3，数值越大越灵敏 |
| `checkout_timeout_ms` | `int` | 恢复时间（毫秒） |

- **碰撞检测行为说明**：

开启碰撞检测后，控制器会持续监测各关节的外力扰动（参见 `get_motion_control_status()` 返回的 `arm_disturbance_force`）。
- **检测到碰撞**：当外力超过当前灵敏度对应的阈值时，**自动切换到位置控制模式（`control_mode=1`）以保障安全**，停止当前柔顺/规划运动；
- **自动恢复**：在 `checkout_timeout_ms` 指定的恢复时间窗内若再未触发新的碰撞，**自动切换回触发前的原控制模式**（如笛卡尔阻抗 / 关节阻抗）；
- **灵敏度选择**：`sensitivity` 越大越容易触发，建议根据负载/末端工具实际工况调试，默认从 `2` 开始；
- **触发后期间**：被切换到位控期间，下发笛卡尔阻抗 / 关节阻抗模式专属的接口会因模式不匹配而被拒绝，应等待自动恢复或显式重新 `set_control_mode()` 后再继续。

- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

robot = agibot_gdk.Robot()

config = robot.get_collision_detection_config()
print(f"enabled={config.is_enabled}, sensitivity={config.sensitivity}")

agibot_gdk.gdk_release()
```

#### 26. `set_collision_detection_config()`

- **功能**：设置碰撞检测配置
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `config` | `CollisionDetectionConfig` | 碰撞检测配置 |

- **返回值**：`int`，0 表示成功，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

robot = agibot_gdk.Robot()

config = agibot_gdk.CollisionDetectionConfig()
config.is_enabled = True
config.sensitivity = 2
config.checkout_timeout_ms = 500
robot.set_collision_detection_config(config)

agibot_gdk.gdk_release()
```

#### 27. `get_joint_impedance()`

- **功能**：获取关节阻抗参数
- **参数**：无
- **返回值**：`list[Impedance]`，每个关节的阻抗参数
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

robot = agibot_gdk.Robot()

impedances = robot.get_joint_impedance()
for i, imp in enumerate(impedances):
    print(f"joint {i}: kp={imp.kp}, ki={imp.ki}, kv={imp.kv}")

agibot_gdk.gdk_release()
```

#### 28. `set_joint_impedance()`

- **功能**：设置关节阻抗参数（必须先切换到 `control_mode=3`（关节阻抗）后再调用）。当前实现仅支持设置 `kp`（刚度系数），`ki` / `kv` 暂未启用。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `impedances` | `list[Impedance]` | 每个关节的阻抗参数；当前实现仅读取并应用 `kp`，`ki` / `kv` 字段会被忽略。`control_group=0/1` 时长度为单臂关节数，`control_group=2` 时长度为双臂关节数之和（前 N 个为左臂，后 N 个为右臂） |
| `control_group` | `int` | 控制组，0: 左臂，1: 右臂，2: 双臂 |

- **返回值**：`int`，0 表示成功，失败时抛出异常
- **kp 取值说明**：

单臂的 7 维 `kp` 按顺序对应该臂从基座到末端的 **关节 1 ~ 关节 7**。各关节建议取值范围：

| 关节序号 | 下限 | 上限 | 单位 |
| --- | --- | --- | --- |
| 关节 1 / 2 / 3（大臂） | 50 | 500 | N·m/rad |
| 关节 4（肘） | 60 | 600 | N·m/rad |
| 关节 5 / 6 / 7（小臂/腕） | 30 | 300 | N·m/rad |

即单臂 `kp` 下限为 `(50, 50, 50, 60, 30, 30, 30)`，上限为 `(500, 500, 500, 600, 300, 300, 300)`。`control_group=2`（双臂）时长度为 14，前 7 维为左臂、后 7 维为右臂，每一臂分别遵循上述顺序与范围。

- `kp` 越大刚度越高，关节对位置误差恢复力强，但抗冲击/柔顺性下降；
- `kp` 越小越柔顺，更适合接触类或拖拽类任务，但跟踪误差更大；
- 超出推荐范围可能引发振荡或柔顺度严重不足，请按工况在区间内调试。
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

robot = agibot_gdk.Robot()

# 先切换到关节阻抗控制模式（control_mode=3），再设置对应参数
robot.set_control_mode(3)

impedances = [agibot_gdk.Impedance(kp=100.0) for _ in range(7)]  # 单臂 7 个关节，仅 kp 生效
robot.set_joint_impedance(impedances, 0)  # 左臂

agibot_gdk.gdk_release()
```

#### 29. `get_payload()`

- **功能**：获取双臂末端负载信息
- **参数**：无
- **返回值**：`PayloadInfo` 对象

**PayloadInfo对象详细说明**：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| `left_arm` | `InertiaInfo` | 左臂末端负载信息 |
| `right_arm` | `InertiaInfo` | 右臂末端负载信息 |

**InertiaInfo对象详细说明**：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `mass` | `float` | 负载质量 | kg |
| `center_of_mass` | `list[float]` | 负载重心位置（长度 3） | m |
| `inertia` | `list[float]` | 负载惯性矩阵（长度 9，3x3 按行展开） | kg·m² |

- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

robot = agibot_gdk.Robot()

info = robot.get_payload()
print(f"left mass={info.left_arm.mass}, right mass={info.right_arm.mass}")

agibot_gdk.gdk_release()
```

#### 30. `set_payload()`

- **功能**：设置双臂末端负载信息。负载参数会进入控制器的动力学补偿，是力控（笛卡尔阻抗 / 关节阻抗）下力反馈与重力补偿正确性的前提。
- **参考坐标系**：负载的质心（`center_of_mass`）与惯性张量（`inertia`）均以末端 `end_link` 坐标系（末端法兰中心位置）为参考，填写时请确保数值在该坐标系下表达。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `payload_info` | `PayloadInfo` | 负载信息 |

- **返回值**：`int`，0 表示成功，失败时抛出异常
- **使用建议**：
- **仅在末端确实存在负载时设置**：空载状态不应保留旧的非零负载参数，否则控制器会按虚构的额外重力进行补偿，导致力控柔顺跟踪偏移、力反馈失真。空载或负载移除时，请显式调用本接口将质量/质心/惯量重置为零。
- **持久负载（长期挂载）**：当末端常驻同一负载（如固定工具、长期挂载的物体）时，调用 `set_payload()` 后应立即调用一次 [`torque_sensor_calibration()`](#31-torque_sensor_calibration)，补偿当前装配下传感器的零点偏置。标定一次后即可持续使用。
- **短期负载（搬运类场景）**：搬重物前先 `set_payload()` 配置当前负载；放下后再次 `set_payload()` 重置为零负载。这种短周期切换通常不需要每次都重新做传感器零点标定，但负载参数必须随实际抓持状态同步更新。
- **静止下设置**：建议在机械臂静止时调用，避免与运动控制指令产生竞争。
- **最大负载**：G02机器标称为5kg，请不要超过最大负载，否则会导致机械臂运动异常。
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

robot = agibot_gdk.Robot()

info = agibot_gdk.PayloadInfo()
info.left_arm.mass = 0.5
info.left_arm.center_of_mass = [0.0, 0.0, 0.05]
info.left_arm.inertia = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
info.right_arm.mass = 0.5
info.right_arm.center_of_mass = [0.0, 0.0, 0.05]
info.right_arm.inertia = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
robot.set_payload(info)

agibot_gdk.gdk_release()
```

#### 31. `torque_sensor_calibration()`

- **功能**：标定关节力矩传感器零点。调用前请正确设置负载，并确保机械臂处于静止状态，不要在规划、拖拽、其他标定过程中重复发起。标定期间请勿下发运动类指令，标定完成后会自动恢复正常控制模式。
- **参数**：无
- **返回值**：`int`，0 表示成功，失败时抛出异常
- **什么情况下需要在线标定力矩传感器**：
- 在无外力接触，负载设置正确的情况下，motion control status消息中的arm\_disturbance\_force和frame\_wrenchs的值较大，说明传感器存在漂移，需要标定；
- 机械臂发生碰撞之后，需要标定；
- 长时间使用或者温度变化较大，建议标定；
- **使用建议**
- 建议不带负载，在初始化之后进行标定；
- 标定时，机器人必须处于静止状态；
- 不要在规划、拖拽、其他标定过程中重复发起；
- 标定期间不要再下发运动类指令；
- 如需带负载标定，请正确设置负载，否则标定误差过大或者超出检测阈值会导致标定失败。
- **持久化说明**：

本接口的标定结果会被\*\*固化\*\*到机器人本体，机器人重启后依然有效，无需每次开机重做。
与之相对，下列配置项\*\*不会\*\*被持久化，每次机器人重启后都会回到默认值，需要在使用前重新设置：

| 配置项 | 相关接口 |
| --- | --- |
| 笛卡尔阻抗参数 | [`set_cartesian_impedance()`](#24-set_cartesian_impedance) |
| 关节阻抗参数 | [`set_joint_impedance()`](#28-set_joint_impedance) |
| 末端负载 | [`set_payload()`](#30-set_payload) |
| 末端 TCP 变换 | [`set_tcp_transform()`](#35-set_tcp_transform) |
| 碰撞检测开关与阈值 | [`set_collision_detection_config()`](#26-set_collision_detection_config) |
| 运动控制模式 | [`set_control_mode()`](#22-set_control_mode) |

实际工程中建议在应用启动流程内统一一次性下发上述配置，避免遗漏。

- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

robot = agibot_gdk.Robot()

robot.torque_sensor_calibration()
print("torque sensor calibration succeeded")

agibot_gdk.gdk_release()
```

#### 32. `clear_motion_control_error()`

- **功能**：清除运动控制错误。运动控制触发安全保护机制后，会进入错误态并停止响应运动指令，此时需调用本接口清除错误后才能继续控制。
- **参数**：无
- **返回值**：`int`，0 表示成功，失败时抛出异常
- **支持的控制模式**：所有控制模式（`1` 关节位控、`2` 笛卡尔阻抗、`3` 关节阻抗）均可调用。
- **使用建议**：
- 通过 [`get_motion_control_status()`](#4-get_motion_control_status) 的 `error_code`（非 0）/ `error_msg` 判断是否处于错误态；
- 清错前请先排除引发错误的根因（如移除外部干涉、确认负载配置正确、机械臂处于静止状态），否则清错后可能立即再次触发；
- 清错成功后建议再次读取 `get_motion_control_status()` 确认 `error_code` 已恢复为 0，再继续下发运动指令。
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

robot = agibot_gdk.Robot()

# 检查运动控制状态，发现错误时清除
status = robot.get_motion_control_status()
if status.error_code != 0:
    print(f"检测到运动控制错误: code={status.error_code}, msg={status.error_msg}")
    try:
        # 清除前请先排除引发错误的根因
        robot.clear_motion_control_error()
        print("运动控制错误已清除")
    except Exception as e:
        print(f"清除运动控制错误失败: {e}")
else:
    print("运动控制状态正常，无需清错")

agibot_gdk.gdk_release()
```

#### 33. `clear_hal_error()`

- **功能**：清除硬件（HAL）层故障。机器人头部、腰部、手臂等电机在报错后会进入故障态并锁停，本接口向各电机下发一次清错（故障复位）请求，使其恢复可用。
- **参数**：无
- **返回值**：`int`，0 表示请求成功下发，失败时抛出异常
- **与 [`clear_motion_control_error()`](#32-clear_motion_control_error) 的区别**：本接口作用于 **HAL 硬件层**（电机驱动层的故障复位）；`clear_motion_control_error()` 作用于\*\*运动控制层\*\*（力控安全保护错误）。两者面向不同层级，按需调用。
- **使用建议**：
- 清错前请先排除引发故障的根因（如机械干涉、过载、急停未释放等），否则清错后可能立即再次触发；
- 若清错后电机仍无法运动，请检查硬件状态与故障日志。
- **示例**：

```
import agibot_gdk
import time

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

robot = agibot_gdk.Robot()

# 清除硬件层故障（清错前请先排除引发故障的根因）
try:
    robot.clear_hal_error()
    print("HAL清错请求已下发")
except Exception as e:
    print(f"HAL清错请求下发失败: {e}")

# 异步下发，留出时间让底层完成复位
time.sleep(0.5)

agibot_gdk.gdk_release()
```

#### 34. `get_tcp_transform()`

- **功能**：获取双臂末端 TCP（工具中心点）变换。TCP 变换描述工具坐标系相对末端 `end_link`（法兰中心）的位姿。
- **参数**：无
- **返回值**：`DualArmTcpTransform` 对象，失败时抛出异常

**DualArmTcpTransform对象详细说明**：

| 成员名 | 类型 | 描述 |
| --- | --- | --- |
| `left_arm` | `TcpTransform` | 左臂末端 TCP 变换 |
| `right_arm` | `TcpTransform` | 右臂末端 TCP 变换 |

**TcpTransform对象详细说明**：

| 成员名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `position` | `list[float]`（长度 3） | TCP 相对 flange 的平移 `[x, y, z]` | m |
| `euler_zyx` | `list[float]`（长度 3） | 欧拉角 `[roll, pitch, yaw]`，旋转 `R = Rz(yaw) * Ry(pitch) * Rx(roll)`（绕固定轴 X→Y→Z 外旋） | rad |

- **示例**：

```
import agibot_gdk

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

robot = agibot_gdk.Robot()

transform = robot.get_tcp_transform()
print("left tcp position:", transform.left_arm.position)
print("left tcp euler_zyx:", transform.left_arm.euler_zyx)

agibot_gdk.gdk_release()
```

#### 35. `set_tcp_transform()`

- **功能**：设置双臂末端 TCP（工具中心点）变换。设置后，末端位姿类指令（如末端执行器位姿控制、笛卡尔规划）会以 TCP 为参考点，便于在工具尖端坐标系下进行控制。
- **参考坐标系**：TCP 变换以末端 `end_link` 坐标系（法兰中心位置）为参考，`position` 为平移、`euler_zyx` 为姿态欧拉角。
- **注意（安全限制 / 调用顺序）**：为确保安全，**禁止在笛卡尔阻抗模式（`control_mode=2`）下设置或切换 TCP 坐标**。因此必须先调用 `set_tcp_transform()` 设置 TCP，再通过 `set_control_mode(2)` 进入笛卡尔阻抗模式；若已处于笛卡尔阻抗模式下再调用本接口，TCP 设置会\*\*失败\*\*。设置完成并进入笛卡尔阻抗模式后，还需调用 [`switch_end_pose_control_frame()`](#21-switch_end_pose_control_frame) 将末端位姿控制的目标 frame 切换到工具坐标系（`left_tool0_body` / `right_tool0_body`），TCP 变换才会在 `end_effector_pose_control()` 中生效。推荐顺序：`set_tcp_transform()` → `set_control_mode(2)` → `switch_end_pose_control_frame()`。
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `transform` | `DualArmTcpTransform` | 双臂 TCP 变换 |

- **返回值**：`int`，0 表示成功，失败时抛出异常
- **使用建议**：
- **静止下设置**：建议在机械臂静止时调用，避免与运动控制指令产生竞争。
- **更换工具后更新**：每次更换末端工具或工具安装位置变化后，应重新设置 TCP 变换，确保末端位姿控制以正确的工具尖端为参考。
- **恢复默认**：若需以法兰中心为参考点，可将 `position` 与 `euler_zyx` 全部置零。
- **先设置 TCP，再进入笛卡尔模式**：为确保安全，禁止在笛卡尔阻抗模式（`control_mode=2`）下设置TCP，必须先 `set_tcp_transform()` 再 `set_control_mode(2)`，否则 TCP 设置会\*\*失败\*\*。
- **需切换到工具坐标系后生效**：`set_tcp_transform()` 配置的是 `tool0`（`left_tool0_body` / `right_tool0_body`）相对法兰的变换。`end_effector_pose_control()` 在笛卡尔阻抗控制下默认仍以 `arm_l_end_link` / `arm_r_end_link`（法兰中心）为目标 frame，**TCP 设置不会自动生效**。需调用 [`switch_end_pose_control_frame()`](#21-switch_end_pose_control_frame) 切换到工具坐标系（`left_tool0_body` / `right_tool0_body`）后，新的 TCP 变换才会作用于末端位姿控制。
- **示例**：

```
import agibot_gdk

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)

robot = agibot_gdk.Robot()

transform = agibot_gdk.DualArmTcpTransform()
# TCP 沿 flange Z 轴前移 0.1m，无姿态偏转
transform.left_arm.position = [0.0, 0.0, 0.1]
transform.left_arm.euler_zyx = [0.0, 0.0, 0.0]
transform.right_arm.position = [0.0, 0.0, 0.1]
transform.right_arm.euler_zyx = [0.0, 0.0, 0.0]
robot.set_tcp_transform(transform)

agibot_gdk.gdk_release()
```

## 使用注意事项

1. **GDK初始化**：使用机器人控制功能前必须先调用`agibot_gdk.gdk_init()`初始化GDK系统
2. **GDK释放**：程序结束前必须调用`agibot_gdk.gdk_release()`释放GDK系统资源
3. **关节名称**：使用前请确认关节名称的正确性，可通过`get_motion_control_status()`获取
4. **位置范围**：关节位置应在安全范围内，避免超出机械限制
5. **速度限制**：设置合理的关节速度，避免过快运动造成危险
6. **生命周期**：合理设置请求生命周期，避免命令过期
7. **异常处理**：所有接口在失败时会抛出`RuntimeError`异常，需要适当处理
8. **数据类型**：位置参数使用弧度单位，速度参数使用弧度/秒单位
9. **时间戳精度**：时间戳单位为纳秒，可用于精确的时间同步

## 应用场景

- **基础控制**：实现机器人的基本运动控制
- **状态监控**：实时监控机器人各关节状态
- **动作录制**：录制和回放机器人动作序列
- **精确控制**：实现高精度的关节位置控制
- **安全检测**：监控机器人异常状态，确保安全运行
- **末端执行器控制**：控制夹爪等末端执行器的开合
- **多关节协调**：实现多个关节的协调运动
- **位姿控制**：基于笛卡尔坐标系的末端执行器位姿控制



---

<!-- source: contents_reference_Python_slam -->

# SLAM

# GDK SLAM 接口文档（Python）

## 概述

SLAM（Simultaneous Localization and Mapping，同时定位与地图构建）模块为G02机器人提供了实时建图和定位功能。通过Python接口，开发者可以方便地实现机器人的环境感知、地图构建、位置估计等功能，适用于自主导航、环境建模、定位服务等多种场景。

## 接口说明

### Slam 类

该类封装了SLAM系统的主要功能接口。

#### 1. `get_slam_state()`

- **功能**：获取SLAM系统当前状态
- **参数**：无
- **返回值**：`int`，SLAM状态码，失败时抛出异常（1：开始建图 2：停止建图 0：取消建图）
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

slam = agibot_gdk.Slam()

# 获取SLAM状态
slam_state = slam.get_slam_state()
print(f"SLAM状态: {slam_state}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 2. `start_mapping()`

- **功能**：开始建图
- **参数**：无
- **返回值**：无，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

slam = agibot_gdk.Slam()

# 开始建图
try:
    slam.start_mapping()
    print("开始建图成功")
except Exception as e:
    print(f"开始建图失败: {e}")

# 检查建图状态
time.sleep(2)
slam_state = slam.get_slam_state()
print(f"建图状态: {slam_state}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 3. `stop_mapping()`

- **功能**：停止且保存建图
- **参数**：无
- **返回值**：无，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

slam = agibot_gdk.Slam()

# 停止建图
try:
    slam.stop_mapping()
    print("停止建图成功")
except Exception as e:
    print(f"停止建图失败: {e}")

# 检查建图状态
time.sleep(2)
slam_state = slam.get_slam_state()
print(f"建图状态: {slam_state}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 4. `cancel_mapping()`

- **功能**：取消建图
- **参数**：无
- **返回值**：无，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

slam = agibot_gdk.Slam()

# 取消建图
try:
    slam.cancel_mapping()
    print("取消建图成功")
except Exception as e:
    print(f"取消建图失败: {e}")

# 检查建图状态
time.sleep(2)
slam_state = slam.get_slam_state()
print(f"建图状态: {slam_state}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 5. `get_odom_info()`

- **功能**：获取里程计信息
- **参数**：无
- **返回值**：`OdomInfo`对象，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `pose.pose.position.x` | `float` | 当前位置X坐标 | 米 |
| `pose.pose.position.y` | `float` | 当前位置Y坐标 | 米 |
| `pose.pose.position.z` | `float` | 当前位置Z坐标 | 米 |
| `pose.pose.orientation.x` | `float` | 当前方向四元数X分量 | 无单位 |
| `pose.pose.orientation.y` | `float` | 当前方向四元数Y分量 | 无单位 |
| `pose.pose.orientation.z` | `float` | 当前方向四元数Z分量 | 无单位 |
| `pose.pose.orientation.w` | `float` | 当前方向四元数W分量 | 无单位 |
| `twist.twist.linear.x` | `float` | 线速度X分量 | 米/秒 |
| `twist.twist.linear.y` | `float` | 线速度Y分量 | 米/秒 |
| `twist.twist.linear.z` | `float` | 线速度Z分量 | 米/秒 |
| `twist.twist.angular.x` | `float` | 角速度X分量 | 弧度/秒 |
| `twist.twist.angular.y` | `float` | 角速度Y分量 | 弧度/秒 |
| `twist.twist.angular.z` | `float` | 角速度Z分量 | 弧度/秒 |
| `is_stationary` | `bool` | 是否静止 | 布尔值 |
| `is_sliping` | `bool` | 是否打滑 | 布尔值 |
| `loc_confidence` | `int` | 定位置信度 | 无单位 |
| `loc_state` | `int` | 定位状态 | 整数 |
| `velocity` | `Vector3` | 速度向量 | 米/秒 |
| `velocity_body` | `Vector3` | 机体坐标系速度 | 米/秒 |
| `acceleration` | `Vector3` | 加速度向量 | 米/秒² |
| `ang_vel` | `Vector3` | 角速度向量 | 弧度/秒 |
| `orientation_euler` | `Vector3` | 欧拉角方向 | 弧度 |

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

slam = agibot_gdk.Slam()

# 获取里程计信息
odom_info = slam.get_odom_info()
print(f"位置: ({odom_info.pose.pose.position.x:.3f}, {odom_info.pose.pose.position.y:.3f}, {odom_info.pose.pose.position.z:.3f})")
print(f"方向: ({odom_info.pose.pose.orientation.x:.3f}, {odom_info.pose.pose.orientation.y:.3f}, {odom_info.pose.pose.orientation.z:.3f}, {odom_info.pose.pose.orientation.w:.3f})")
print(f"线速度: ({odom_info.twist.twist.linear.x:.3f}, {odom_info.twist.twist.linear.y:.3f}, {odom_info.twist.twist.linear.z:.3f})")
print(f"角速度: ({odom_info.twist.twist.angular.x:.3f}, {odom_info.twist.twist.angular.y:.3f}, {odom_info.twist.twist.angular.z:.3f})")
print(f"是否静止: {odom_info.is_stationary}")
print(f"是否打滑: {odom_info.is_sliping}")
print(f"定位置信度: {odom_info.loc_confidence:.3f}")
print(f"定位状态: {odom_info.loc_state}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 6. `record_spec_loc()` （该接口暂未上线）

- **功能**：记录当前位置为充电点位置
- **参数**：无
- **返回值**：无，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

slam = agibot_gdk.Slam()

# 记录特定位置
try:
    slam.record_spec_loc()
    print("记录特定位置成功")
except Exception as e:
    print(f"记录特定位置失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 7. `get_curr_pose()`

- **功能**：获取当前位姿
- **参数**：无
- **返回值**：`Pose`对象，包含以下属性：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `position.x` | `float` | 位置X坐标 | 米 |
| `position.y` | `float` | 位置Y坐标 | 米 |
| `position.z` | `float` | 位置Z坐标 | 米 |
| `orientation.x` | `float` | 方向四元数X分量 | 无单位 |
| `orientation.y` | `float` | 方向四元数Y分量 | 无单位 |
| `orientation.z` | `float` | 方向四元数Z分量 | 无单位 |
| `orientation.w` | `float` | 方向四元数W分量 | 无单位 |

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

slam = agibot_gdk.Slam()

# 获取当前位姿
pose = slam.get_curr_pose()
print(f"当前位置: ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})")
print(f"当前方向: ({pose.orientation.x:.3f}, {pose.orientation.y:.3f}, {pose.orientation.z:.3f}, {pose.orientation.w:.3f})")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 8. `global_local()`

- **功能**：全局重定位，用于在已知地图中初始化机器人位姿
- **注意**: 当前仅支持在建图开始时的初始位姿进行重定位，其他位姿进行重定位无法保证后续定位导航精度
- **参数**：
- `grid_map_pt`：`list [x, y, z]`，栅格地图原点在 map 坐标系下的坐标；`x,y` 为坐标(m)，`z` 为分辨率
- `map_pt`：`list [x, y, z]`，初始位姿；`x,y` 为坐标(m)，`z` 为航向角(rad)
- **返回值**：无，失败时抛出异常
- **示例**（结合 Map 获取 `grid_map.origin` 作为栅格原点）：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

slam = agibot_gdk.Slam()
map_obj = agibot_gdk.Map()

# 获取当前使用地图信息
current_map = map_obj.get_curr_map()
map_info = map_obj.get_map(current_map.id)

# 从 grid_map.origin 取出栅格原点坐标
# MapInfo.grid_map 为 OccupancyGrid，origin 为 Pose，position 为 Vector3
origin_x = map_info.grid_map.origin.position.x
origin_y = map_info.grid_map.origin.position.y
resolution = map_info.grid_map.resolution

# 构造参数：grid_map_pt 为栅格原点，map_pt 为初始位姿
grid_map_pt = [origin_x, origin_y, resolution]
map_pt = [0.0, 0.0, 0.0]  # 初始位姿 (x, y, yaw)，yaw=0 表示朝向 x 轴

slam.global_local(grid_map_pt, map_pt)
print("全局重定位成功")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 9. 完整使用示例

- **功能**：演示SLAM模块的完整使用流程
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

# 初始化SLAM模块
slam = agibot_gdk.Slam()
# 开始建图
try:
    slam.start_mapping()
    print("开始建图成功")
except Exception as e:
    print(f"开始建图失败: {e}")

# 建图过程中监控状态和位置
for i in range(10):
    try:
        odom_info = slam.get_odom_info()
        print(f"里程计位置: ({odom_info.pose.pose.position.x:.3f}, {odom_info.pose.pose.position.y:.3f}, {odom_info.pose.pose.position.z:.3f})")
        print(f"里程计方向: ({odom_info.pose.pose.orientation.x:.3f}, {odom_info.pose.pose.orientation.y:.3f}, {odom_info.pose.pose.orientation.z:.3f}, {odom_info.pose.pose.orientation.w:.3f})")

        slam_state = slam.get_slam_state()
        print(f"SLAM状态: {slam_state}")

        pose = slam.get_curr_pose()
        print(f"当前位姿: ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})")

    except Exception as e:
        print(f"获取信息失败: {e}")

    time.sleep(1)

# 记录特定位置
try:
    slam.record_spec_loc()
    print("记录特定位置成功")
except Exception as e:
    print(f"记录特定位置失败: {e}")

# 停止建图
try:
    slam.stop_mapping()
    print("停止建图成功")
except Exception as e:
    print(f"停止建图失败: {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

## 使用注意事项

1. **GDK初始化**：使用SLAM功能前必须先调用`agibot_gdk.gdk_init()`初始化GDK系统
2. **GDK释放**：程序结束前必须调用`agibot_gdk.gdk_release()`释放GDK系统资源
3. **建图环境**：确保建图环境有足够的特征点，避免在空旷或重复性强的环境中建图
4. **状态监控**：及时检查SLAM状态，确保系统正常运行
5. **数据质量**：保证传感器数据质量，避免在传感器故障时进行建图
6. **计算资源**：SLAM算法计算量较大，注意系统资源使用情况
7. **异常处理**：所有接口在失败时会抛出`RuntimeError`异常，需要适当处理
8. **位姿精度**：SLAM位姿估计可能存在漂移，建议定期校正
9. **地图保存**：建图完成后及时保存地图，避免数据丢失

## 应用场景

- **环境建图**：构建机器人工作环境的详细地图
- **自主定位**：在已知环境中实现精确定位
- **导航服务**：为路径规划提供环境信息
- **环境监控**：实时监控环境变化
- **数据采集**：收集环境数据用于后续分析
- **位置记录**：标记和记录重要位置点
- **轨迹跟踪**：跟踪机器人的运动轨迹
- **地图更新**：动态更新环境地图信息



---

<!-- source: contents_reference_Python_tf -->

# TF

# GDK TF 接口文档（Python）

## 概述

TF（Transform，坐标变换）模块为G02机器人提供了坐标系变换功能。通过Python接口，开发者可以方便地获取机器人各部件之间的坐标变换关系，适用于坐标变换、传感器标定、运动学计算等多种场景。

## 接口说明

### TF 类

该类封装了坐标变换的主要功能接口。

#### 1. `get_all_tf_from_base_link()`

- **功能**：获取从base\_link到所有子坐标系的变换关系
- **参数**：无
- **返回值**：`list[TransformStamped]`，包含所有变换关系的列表，失败时抛出异常
- **TransformStamped 对象属性**：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `frame_id` | `str` | 父坐标系ID | 字符串 |
| `child_frame_id` | `str` | 子坐标系ID | 字符串 |
| `transform.translation.x` | `float` | 平移X分量 | 米 |
| `transform.translation.y` | `float` | 平移Y分量 | 米 |
| `transform.translation.z` | `float` | 平移Z分量 | 米 |
| `transform.rotation.x` | `float` | 旋转四元数X分量 | 无单位 |
| `transform.rotation.y` | `float` | 旋转四元数Y分量 | 无单位 |
| `transform.rotation.z` | `float` | 旋转四元数Z分量 | 无单位 |
| `transform.rotation.w` | `float` | 旋转四元数W分量 | 无单位 |
| `timestamp_ns` | `int` | 时间戳 | 纳秒 |

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

tf = agibot_gdk.TF()

# 获取所有变换关系
transforms = tf.get_all_tf_from_base_link()
print(f"获取到 {len(transforms)} 个变换关系:")

for transform_stamped in transforms:
    print(f"坐标系: {transform_stamped.frame_id} -> {transform_stamped.child_frame_id}")
    print(f"  平移: x={transform_stamped.transform.translation.x:.3f}, "
          f"y={transform_stamped.transform.translation.y:.3f}, "
          f"z={transform_stamped.transform.translation.z:.3f}")
    print(f"  旋转: x={transform_stamped.transform.rotation.x:.3f}, "
          f"y={transform_stamped.transform.rotation.y:.3f}, "
          f"z={transform_stamped.transform.rotation.z:.3f}, "
          f"w={transform_stamped.transform.rotation.w:.3f}")
    print(f"  时间戳: {transform_stamped.timestamp_ns}")
    print()

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 2. `get_tf_from_base_link()`

- **功能**：获取从base\_link到指定子坐标系的变换关系
- **参数**：
- `child_frame_id`：子坐标系ID（字符串）
- **返回值**：`Transform`对象，包含变换信息，失败时抛出异常
- **Transform 对象属性**：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `translation.x` | `float` | 平移X分量 | 米 |
| `translation.y` | `float` | 平移Y分量 | 米 |
| `translation.z` | `float` | 平移Z分量 | 米 |
| `rotation.x` | `float` | 旋转四元数X分量 | 无单位 |
| `rotation.y` | `float` | 旋转四元数Y分量 | 无单位 |
| `rotation.z` | `float` | 旋转四元数Z分量 | 无单位 |
| `rotation.w` | `float` | 旋转四元数W分量 | 无单位 |

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

tf = agibot_gdk.TF()

# 获取指定坐标系的变换关系
child_frame_id = "head_link3"
transform = tf.get_tf_from_base_link(child_frame_id)

print(f"从base_link到{child_frame_id}的变换:")
print(f"  平移: x={transform.translation.x:.3f}, "
      f"y={transform.translation.y:.3f}, "
      f"z={transform.translation.z:.3f}")
print(f"  旋转: x={transform.rotation.x:.3f}, "
      f"y={transform.rotation.y:.3f}, "
      f"z={transform.rotation.z:.3f}, "
      f"w={transform.rotation.w:.3f}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 3. `get_tf_from_sensor()`

- **功能**：获取从base\_link到指定传感器的变换关系
- **参数**：
- `sensor_extrinsic_type`：传感器外参类型（枚举值）
- **返回值**：`Transform`对象，包含变换信息，失败时抛出异常
- **SensorExtrinsicType 枚举值**：

| 枚举值 | 描述 |
| --- | --- |
| `kUnknown` | 未知类型 |
| `kHeadLeftStereoToHeadRightStereo` | 头部左立体相机到右立体相机 |
| `kLeftHandDepthToLeftHandColor` | 左手深度相机到彩色相机 |
| `kRightHandDepthToRightHandColor` | 右手深度相机到彩色相机 |
| `kHeadDepthToHeadColor` | 头部深度相机到彩色相机 |
| `kHeadLeftStereoToHeadLink3` | 头部左立体相机到Headlink3（头部第三连杆） |
| `kHeadRightStereoToHeadLink3` | 头部右立体相机到Headlink3（头部第三连杆） |
| `kHeadLeftFisheyeToHeadLink3` | 头部左鱼眼相机到Headlink3（头部第三连杆） |
| `kHeadRightFisheyeToHeadLink3` | 头部右鱼眼相机到Headlink3（头部第三连杆） |
| `kHeadBackFisheyeToHeadLink3` | 头部后鱼眼相机到Headlink3（头部第三连杆） |
| `kChassisFrontLidarToBaseLink` | 底盘前激光雷达到base\_link |
| `kChassisBackLidarToBaseLink` | 底盘后激光雷达到base\_link |
| `kChassisBackLidarToChassisFrontLidar` | 底盘后激光雷达到前激光雷达 |
| `kChassisMid360ImuToChassisMid360Lidar` | 底盘Mid360 [IMU](../../../concepts/sensors/#imu)到底盘Mid360激光雷达 |
| `kChassisImuToBaseLink` | 底盘IMU到base\_link |
| `kLeftHandRGBDToArmLEndLink` | 左手[RGBD](../../../concepts/sensors/#rgbd)到左臂末端链接 |
| `kRightHandRGBDToArmREndLink` | 右手RGBD到右臂末端链接 |
| `kHeadRGBDToHeadLink3` | 头部RGBD到Headlink3（头部第三连杆） |

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

tf = agibot_gdk.TF()

# 获取头部左立体相机到右立体相机的变换关系
sensor_type = agibot_gdk.SensorExtrinsicType.kHeadLeftStereoToHeadRightStereo
transform = tf.get_tf_from_sensor(sensor_type)

print(f"从头部左立体相机到右立体相机的变换:")
print(f"  平移: x={transform.translation.x:.3f}, "
      f"y={transform.translation.y:.3f}, "
      f"z={transform.translation.z:.3f}")
print(f"  旋转: x={transform.rotation.x:.3f}, "
      f"y={transform.rotation.y:.3f}, "
      f"z={transform.rotation.z:.3f}, "
      f"w={transform.rotation.w:.3f}")

# 获取底盘前激光雷达到base_link的变换关系
lidar_transform = tf.get_tf_from_sensor(agibot_gdk.SensorExtrinsicType.kChassisFrontLidarToBaseLink)
print(f"\n从底盘前激光雷达到base_link的变换:")
print(f"  平移: x={lidar_transform.translation.x:.3f}, "
      f"y={lidar_transform.translation.y:.3f}, "
      f"z={lidar_transform.translation.z:.3f}")
print(f"  旋转: x={lidar_transform.rotation.x:.3f}, "
      f"y={lidar_transform.rotation.y:.3f}, "
      f"z={lidar_transform.rotation.z:.3f}, "
      f"w={lidar_transform.rotation.w:.3f}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 4. `lookup_transform_latest()`

- **功能**：查询两个坐标系之间的最新变换关系
- **参数**：
- `target_frame`：目标坐标系ID（字符串）
- `source_frame`：源坐标系ID（字符串）
- `return_timestamp`：是否返回时间戳（布尔值，可选，默认为`False`）
- **返回值**：
- 如果`return_timestamp=False`：返回`tuple(Transform, None)`
- 如果`return_timestamp=True`：返回`tuple(Transform, int)`，其中第二个元素是时间戳（纳秒）
- 失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

tf = agibot_gdk.TF()

# 查询最新变换（不返回时间戳）
transform, _ = tf.lookup_transform_latest("base_link", "arm_l_end_link")
print("从arm_l_end_link到base_link的最新变换:")
print(f"  平移: x={transform.translation.x:.3f}, "
      f"y={transform.translation.y:.3f}, "
      f"z={transform.translation.z:.3f}")
print(f"  旋转: x={transform.rotation.x:.3f}, "
      f"y={transform.rotation.y:.3f}, "
      f"z={transform.rotation.z:.3f}, "
      f"w={transform.rotation.w:.3f}")

# 查询最新变换（返回时间戳）
transform, timestamp_ns = tf.lookup_transform_latest(
    "base_link", "arm_l_end_link", return_timestamp=True
)
print(f"\n时间戳: {timestamp_ns} ns")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 5. `lookup_transform()`

- **功能**：查询两个坐标系在特定时间的变换关系（支持时间插值）
- **参数**：
- `target_frame`：目标坐标系ID（字符串）
- `source_frame`：源坐标系ID（字符串）
- `time_ns`：查询时间（纳秒时间戳，整数）
- **返回值**：`Transform`对象，包含变换信息，失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

tf = agibot_gdk.TF()

# 获取当前时间戳（纳秒）
current_time_ns =  tf.get_latest_timestamp("arm_l_end_link")
# 查询1秒前的变换
target_time_ns = current_time_ns - 1_000_000_000  # 1秒前

transform = tf.lookup_transform("base_link", "arm_l_end_link", target_time_ns)
print(f"从arm_l_end_link到base_link在时间 {target_time_ns} 的变换:")
print(f"  平移: x={transform.translation.x:.3f}, "
      f"y={transform.translation.y:.3f}, "
      f"z={transform.translation.z:.3f}")
print(f"  旋转: x={transform.rotation.x:.3f}, "
      f"y={transform.rotation.y:.3f}, "
      f"z={transform.rotation.z:.3f}, "
      f"w={transform.rotation.w:.3f}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 6. `can_transform()`

- **功能**：检查两个坐标系之间是否存在变换关系
- **参数**：
- `target_frame`：目标坐标系ID（字符串）
- `source_frame`：源坐标系ID（字符串）
- **返回值**：`bool`，如果存在变换关系返回`True`，否则返回`False`
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

tf = agibot_gdk.TF()

# 检查是否存在变换
if tf.can_transform("base_link", "arm_l_end_link"):
    print("存在从arm_l_end_link到base_link的变换")
else:
    print("不存在从arm_l_end_link到base_link的变换")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 7. `get_all_frame_names()`

- **功能**：获取所有可用的坐标系名称
- **参数**：无
- **返回值**：`list[str]`，包含所有坐标系名称的列表
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

tf = agibot_gdk.TF()

# 获取所有坐标系名称
frame_names = tf.get_all_frame_names()
print(f"所有可用坐标系 ({len(frame_names)} 个):")
for name in frame_names:
    print(f"  - {name}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 8. `get_latest_timestamp()`

- **功能**：获取指定坐标系的最新时间戳
- **参数**：
- `frame_id`：坐标系ID（字符串）
- **返回值**：`int`，最新时间戳（纳秒），失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

tf = agibot_gdk.TF()

timestamp_ns = tf.get_latest_timestamp("arm_l_end_link")
print(f"arm_l_end_link 最新时间戳: {timestamp_ns} ns")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 9. `clear()`

- **功能**：清空TF缓存中的所有变换关系
- **参数**：无
- **返回值**：无
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

tf = agibot_gdk.TF()

# 清空TF缓存
tf.clear()
print("TF缓存已清空")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

## 完整使用示例

### 坐标系变换查询示例

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

tf = agibot_gdk.TF()

# 1. 获取所有变换关系
print("=== 获取所有变换关系 ===")
transforms = tf.get_all_tf_from_base_link()
print(f"总共有 {len(transforms)} 个坐标系变换关系")

for i, transform_stamped in enumerate(transforms):
    print(f"{i+1}. {transform_stamped.frame_id} -> {transform_stamped.child_frame_id}")
    print(f"   平移: ({transform_stamped.transform.translation.x:.3f}, "
          f"{transform_stamped.transform.translation.y:.3f}, "
          f"{transform_stamped.transform.translation.z:.3f})")
    print(f"   旋转: ({transform_stamped.transform.rotation.x:.3f}, "
          f"{transform_stamped.transform.rotation.y:.3f}, "
          f"{transform_stamped.transform.rotation.z:.3f}, "
          f"{transform_stamped.transform.rotation.w:.3f})")
    print()

# 2. 查询特定坐标系的变换
print("=== 查询特定坐标系变换 ===")
target_frames = ["head_camera_link", "left_hand_link", "right_hand_link"]

for frame_id in target_frames:
    try:
        transform = tf.get_tf_from_base_link(frame_id)
        print(f"{frame_id}:")
        print(f"  平移: ({transform.translation.x:.3f}, "
              f"{transform.translation.y:.3f}, "
              f"{transform.translation.z:.3f})")
        print(f"  旋转: ({transform.rotation.x:.3f}, "
              f"{transform.rotation.y:.3f}, "
              f"{transform.rotation.z:.3f}, "
              f"{transform.rotation.w:.3f})")
    except Exception as e:
        print(f"{frame_id}: 获取失败 - {e}")
    print()

# 3. 查询传感器标定坐标变换（传感器间 / 传感器到 base_link）
print("=== 传感器标定坐标变换（SensorExtrinsicType）===")
sensor_types = [
    (agibot_gdk.SensorExtrinsicType.kHeadLeftStereoToHeadRightStereo, "头部左立体相机到右立体相机"),
    (agibot_gdk.SensorExtrinsicType.kHeadDepthToHeadColor, "头部深度相机到彩色相机"),
    (agibot_gdk.SensorExtrinsicType.kChassisFrontLidarToBaseLink, "底盘前激光雷达到base_link"),
    (agibot_gdk.SensorExtrinsicType.kChassisImuToBaseLink, "底盘IMU到base_link")
]

for sensor_type, sensor_name in sensor_types:
    try:
        transform = tf.get_tf_from_sensor(sensor_type)
        print(f"{sensor_name}:")
        print(f"  平移: ({transform.translation.x:.3f}, "
              f"{transform.translation.y:.3f}, "
              f"{transform.translation.z:.3f})")
        print(f"  旋转: ({transform.rotation.x:.3f}, "
              f"{transform.rotation.y:.3f}, "
              f"{transform.rotation.z:.3f}, "
              f"{transform.rotation.w:.3f})")
    except Exception as e:
        print(f"{sensor_name}: 获取失败 - {e}")
    print()

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

### 传感器标定辅助示例

```
import agibot_gdk
import time
import math

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

tf = agibot_gdk.TF()

def quaternion_to_euler(x, y, z, w):
    """将四元数转换为欧拉角（弧度）"""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def print_transform_info(transform, name):
    """打印变换信息"""
    print(f"{name}:")
    print(f"  平移: ({transform.translation.x:.6f}, "
          f"{transform.translation.y:.6f}, "
          f"{transform.translation.z:.6f}) 米")

    # 转换为欧拉角
    roll, pitch, yaw = quaternion_to_euler(
        transform.rotation.x, transform.rotation.y,
        transform.rotation.z, transform.rotation.w
    )

    print(f"  旋转四元数: ({transform.rotation.x:.6f}, "
          f"{transform.rotation.y:.6f}, "
          f"{transform.rotation.z:.6f}, "
          f"{transform.rotation.w:.6f})")
    print(f"  旋转欧拉角: Roll={math.degrees(roll):.2f}°, "
          f"Pitch={math.degrees(pitch):.2f}°, "
          f"Yaw={math.degrees(yaw):.2f}°")
    print()

# 获取所有相机的变换关系
print("=== 相机外参标定信息 ===")
camera_sensors = [
    (agibot_gdk.SensorExtrinsicType.kHeadLeftStereoToHeadRightStereo, "头部左立体相机到右立体相机"),
    (agibot_gdk.SensorExtrinsicType.kLeftHandDepthToLeftHandColor, "左手深度相机到彩色相机"),
    (agibot_gdk.SensorExtrinsicType.kRightHandDepthToRightHandColor, "右手深度相机到彩色相机"),
    (agibot_gdk.SensorExtrinsicType.kHeadDepthToHeadColor, "头部深度相机到彩色相机"),
    (agibot_gdk.SensorExtrinsicType.kHeadLeftStereoToHeadLink3, "头部左立体相机到Headlink3（头部第三连杆）"),
    (agibot_gdk.SensorExtrinsicType.kHeadRightStereoToHeadLink3, "头部右立体相机到Headlink3（头部第三连杆）"),
    (agibot_gdk.SensorExtrinsicType.kHeadLeftFisheyeToHeadLink3, "头部左鱼眼相机到Headlink3（头部第三连杆）"),
    (agibot_gdk.SensorExtrinsicType.kHeadRightFisheyeToHeadLink3, "头部右鱼眼相机到Headlink3（头部第三连杆）"),
    (agibot_gdk.SensorExtrinsicType.kHeadBackFisheyeToHeadLink3, "头部后鱼眼相机到Headlink3（头部第三连杆）")
]

for sensor_type, sensor_name in camera_sensors:
    try:
        transform = tf.get_tf_from_sensor(sensor_type)
        print_transform_info(transform, sensor_name)
    except Exception as e:
        print(f"{sensor_name}: 获取失败 - {e}")

# 获取其他传感器的变换关系
print("=== 其他传感器外参信息 ===")
other_sensors = [
    (agibot_gdk.SensorExtrinsicType.kChassisFrontLidarToBaseLink, "底盘前激光雷达到base_link"),
    (agibot_gdk.SensorExtrinsicType.kChassisBackLidarToBaseLink, "底盘后激光雷达到base_link"),
    (agibot_gdk.SensorExtrinsicType.kChassisBackLidarToChassisFrontLidar, "底盘后激光雷达到前激光雷达"),
    (agibot_gdk.SensorExtrinsicType.kChassisMid360ImuToChassisMid360Lidar, "底盘Mid360 IMU到激光雷达"),
    (agibot_gdk.SensorExtrinsicType.kChassisImuToBaseLink, "底盘IMU到base_link"),
    (agibot_gdk.SensorExtrinsicType.kLeftHandRGBDToArmLEndLink, "左手RGBD到左臂末端链接"),
    (agibot_gdk.SensorExtrinsicType.kRightHandRGBDToArmREndLink, "右手RGBD到右臂末端链接"),
    (agibot_gdk.SensorExtrinsicType.kHeadRGBDToHeadLink3, "头部RGBD到Headlink3（头部第三连杆）")
]

for sensor_type, sensor_name in other_sensors:
    try:
        transform = tf.get_tf_from_sensor(sensor_type)
        print_transform_info(transform, sensor_name)
    except Exception as e:
        print(f"{sensor_name}: 获取失败 - {e}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

## 使用注意事项

1. **GDK初始化**：使用TF功能前必须先调用`agibot_gdk.gdk_init()`初始化GDK系统
2. **GDK释放**：程序结束前必须调用`agibot_gdk.gdk_release()`释放GDK系统资源
3. **坐标系约定**：遵循机器人标准坐标系约定（右手坐标系）
4. **时间戳精度**：时间戳以纳秒为单位，注意精度处理
5. **异常处理**：所有接口在失败时会抛出`RuntimeError`异常，需要适当处理
6. **传感器类型**：使用传感器外参查询时，确保使用正确的传感器类型枚举
7. **变换矩阵**：四元数表示旋转，注意与旋转矩阵的转换
8. **坐标系ID**：使用字符串查询时，确保坐标系ID正确且存在，可通过`get_all_frame_names()`获取所有可用坐标系
9. **实时性**：TF数据可能随时间变化，注意数据的时效性
10. **变换查询**：`lookup_transform_latest()`查询最新变换，`lookup_transform()`支持时间插值查询历史变换
11. **变换检查**：使用`can_transform()`在查询前检查变换是否存在，避免查询失败
12. **时间插值**：`lookup_transform()`支持时间插值，可以查询历史任意时刻的变换关系
13. **缓存管理**：使用`clear()`可以清空TF缓存，适用于需要重置变换关系的场景
14. **时间戳查询**：`get_latest_timestamp()`可以获取指定坐标系的最新更新时间
15. **返回值处理**：`lookup_transform_latest()`根据`return_timestamp`参数返回不同的元组格式

## 应用场景

- **坐标变换**：获取机器人各部件之间的坐标变换关系
- **传感器标定**：获取传感器相对于机器人本体的外参
- **运动学计算**：为机器人运动学计算提供坐标变换数据
- **视觉处理**：为相机图像处理提供坐标系信息
- **导航定位**：为SLAM和导航提供坐标系变换
- **机械臂控制**：为机械臂运动规划提供坐标系信息
- **多传感器融合**：统一不同传感器的坐标系
- **标定验证**：验证传感器标定结果的正确性



---

<!-- source: contents_reference_Python_types -->

# Types

# GDK 数据类型文档（Python）

## 概述

GDK（Genie Development Kit）提供了丰富的数据类型，用于处理机器人系统中的各种数据。通过Python接口，开发者可以方便地使用这些数据类型进行机器人控制、传感器数据处理、地图管理等功能。

## 枚举类型

### 1. GDKRes

GDK操作结果状态码。

| 枚举值 | 描述 |
| --- | --- |
| `kSuccess` | 操作成功 |
| `kInvalidInput` | 输入参数无效 |
| `kInvalidOutput` | 输出参数无效 |
| `kRuntimeError` | 运行时错误 |
| `kUnknown` | 未知错误 |

**示例**：

```
import agibot_gdk

result = agibot_gdk.gdk_init()
if result == agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化成功")
else:
    print(f"GDK初始化失败，错误码: {result}")
```

### 2. CameraType

相机类型枚举。

| 枚举值 | 描述 |
| --- | --- |
| `kCameraUnknown` | 未知相机 |
| `kHeadBackFisheye` | 头部后鱼眼相机 |
| `kHeadLeftFisheye` | 头部左鱼眼相机 |
| `kHeadRightFisheye` | 头部右鱼眼相机 |
| `kHeadStereoLeft` | 头部立体左相机 |
| `kHeadStereoRight` | 头部立体右相机 |
| `kHandLeftColor` | 左手相机 |
| `kHandRightColor` | 右手相机 |
| `kHeadColor` | 头部彩色相机 |
| `kHeadDepth` | 头部深度相机 |

**示例**：

```
import agibot_gdk

camera_type = agibot_gdk.CameraType.kHeadStereoLeft
print(f"相机类型: {camera_type}")
```

### 3. LidarType

激光雷达类型枚举。

| 枚举值 | 描述 |
| --- | --- |
| `kLidarUnknown` | 未知激光雷达 |
| `kLidarFront` | 前激光雷达 |
| `kLidarBack` | 后激光雷达 |

**示例**：

```
import agibot_gdk

lidar_type = agibot_gdk.LidarType.kLidarFront
print(f"激光雷达类型: {lidar_type}")
```

### 4. ImuType

[IMU](../../../concepts/sensors/#imu)类型枚举。

| 枚举值 | 描述 |
| --- | --- |
| `kImuUnknown` | 未知IMU |
| `kImuFront` | 前IMU |
| `kImuBack` | 后IMU |
| `kImuChassis` | 底盘IMU |
| `kImuHeadRgbd` | 头部[RGBD](../../../concepts/sensors/#rgbd) IMU |

**示例**：

```
import agibot_gdk

imu_type = agibot_gdk.ImuType.kImuChassis
print(f"IMU类型: {imu_type}")
```

### 5. EndEffectorControlGroup

末端执行器控制组枚举。

| 枚举值 | 描述 |
| --- | --- |
| `kUnknown` | 未知组 |
| `kLeftArm` | 左臂 |
| `kRightArm` | 右臂 |
| `kBothArms` | 双臂 |
| `kLeftArmWaistLift` | 左臂+腰部+升降 |
| `kRightArmWaistLift` | 右臂+腰部+升降 |
| `kBothArmsWaistLift` | 双臂+腰部+升降 |
| `kLeftArmWaistPitch` | 左臂+腰部俯仰 |
| `kRightArmWaistPitch` | 右臂+腰部俯仰 |
| `kBothArmsWaistPitch` | 双臂+腰部俯仰 |
| `kLeftArmWaist` | 左臂+腰部 |
| `kRightArmWaist` | 右臂+腰部 |
| `kBothArmsWaist` | 双臂+腰部 |

**示例**：

```
import agibot_gdk

control_group = agibot_gdk.EndEffectorControlGroup.kBothArms
print(f"控制组: {control_group}")
```

### 6. SensorExtrinsicType

传感器外参类型枚举。

| 枚举值 | 描述 |
| --- | --- |
| `kUnknown` | 未知类型 |
| `kHeadLeftStereoToHeadRightStereo` | 头部左立体相机到右立体相机 |
| `kLeftHandDepthToLeftHandColor` | 左手深度相机到彩色相机 |
| `kRightHandDepthToRightHandColor` | 右手深度相机到彩色相机 |
| `kHeadDepthToHeadColor` | 头部深度相机到彩色相机 |
| `kHeadLeftStereoToHeadLink3` | 头部左立体相机到Headlink3（头部第三连杆） |
| `kHeadRightStereoToHeadLink3` | 头部右立体相机到Headlink3（头部第三连杆） |
| `kHeadLeftFisheyeToHeadLink3` | 头部左鱼眼相机到Headlink3（头部第三连杆） |
| `kHeadRightFisheyeToHeadLink3` | 头部右鱼眼相机到Headlink3（头部第三连杆） |
| `kHeadBackFisheyeToHeadLink3` | 头部后鱼眼相机到Headlink3（头部第三连杆） |
| `kChassisFrontLidarToBaseLink` | 底盘前激光雷达到base\_link |
| `kChassisBackLidarToBaseLink` | 底盘后激光雷达到base\_link |
| `kChassisBackLidarToChassisFrontLidar` | 底盘后激光雷达到前激光雷达 |
| `kChassisMid360ImuToChassisMid360Lidar` | 底盘Mid360 IMU到激光雷达 |
| `kChassisImuToBaseLink` | 底盘IMU到base\_link |
| `kLeftHandRGBDToArmLEndLink` | 左手RGBD到左臂末端链接 |
| `kRightHandRGBDToArmREndLink` | 右手RGBD到右臂末端链接 |
| `kHeadRGBDToHeadLink3` | 头部RGBD到Headlink3（头部第三连杆） |

**示例**：

```
import agibot_gdk

sensor_type = agibot_gdk.SensorExtrinsicType.kHeadLeftStereoToHeadRightStereo
print(f"传感器外参类型: {sensor_type}")
```

### 7. LedStripId

灯带 ID 枚举，对应机器人本体上的 7 路灯带物理位置。

| 枚举值 | 数值 | 物理位置 |
| --- | --- | --- |
| `kChassisFront` | 1 | 底盘前 |
| `kChassisLeft` | 2 | 底盘左 |
| `kChassisRight` | 3 | 底盘右 |
| `kChestLeft` | 4 | 胸左 |
| `kChestRight` | 5 | 胸右 |
| `kBack` | 6 | 背部 |
| `kHead` | 7 | 头部 |

**示例**：

```
import agibot_gdk

strip_id = agibot_gdk.LedStripId.kHead
print(f"灯带ID: {strip_id}")
```

### 8. LedStripMode

灯带显示模式枚举。模式只对当前控制项中指定的 `strip_id` 生效；如需让所有灯带显示同一效果，逐路下发即可。

| 枚举值 | 数值 | 描述 |
| --- | --- | --- |
| `kHold` | 0 | 暂停/冻结当前显示帧 |
| `kSync` | 1 | 同步点亮（常亮） |
| `kSequence` | 2 | 流水 |
| `kReverseSequence` | 3 | 反流水 |
| `kTriangleBreath` | 4 | 三角波呼吸 |
| `kFlash` | 5 | 闪烁 |
| `kSineBreath` | 6 | 正弦呼吸 |

**说明**：

- `kHold` 模式下 RGB 数值不起作用；后续发送其他模式可正常恢复控制。
- 需要灭灯时，请使用 `kSync + RGB=(0,0,0)`。
- 三角波呼吸和正弦呼吸对红、绿、蓝、白表现正常；对黄、品红、青等双通道复合色会退化为单通道主色（如黄 R+G → 红），不建议在呼吸模式下使用双通道复合色。

**示例**：

```
import agibot_gdk

mode = agibot_gdk.LedStripMode.kSineBreath
print(f"灯带模式: {mode}")
```

## 基础数据类型

### 1. Vector3

3D向量结构。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `x` | `float` | X轴分量 | 米 |
| `y` | `float` | Y轴分量 | 米 |
| `z` | `float` | Z轴分量 | 米 |

**示例**：

```
import agibot_gdk

vector = agibot_gdk.Vector3()
vector.x = 1.0
vector.y = 2.0
vector.z = 3.0
print(f"向量: {vector}")
```

### 2. Quaternion

四元数结构。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `x` | `float` | 四元数X分量 | 无单位 |
| `y` | `float` | 四元数Y分量 | 无单位 |
| `z` | `float` | 四元数Z分量 | 无单位 |
| `w` | `float` | 四元数W分量 | 无单位 |

**示例**：

```
import agibot_gdk

quat = agibot_gdk.Quaternion()
quat.x = 0.0
quat.y = 0.0
quat.z = 0.0
quat.w = 1.0
print(f"四元数: {quat}")
```

### 3. Pose

位姿结构。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `position` | `Vector3` | 位置 | 米 |
| `orientation` | `Quaternion` | 方向 | 无单位 |

**示例**：

```
import agibot_gdk

pose = agibot_gdk.Pose()
pose.position.x = 1.0
pose.position.y = 2.0
pose.position.z = 3.0
pose.orientation.w = 1.0
print(f"位姿: {pose}")
```

### 4. Twist

速度结构。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `linear` | `Vector3` | 线速度 | 米/秒 |
| `angular` | `Vector3` | 角速度 | 弧度/秒 |

**示例**：

```
import agibot_gdk

twist = agibot_gdk.Twist()
twist.linear.x = 0.5
twist.angular.z = 0.1
print(f"速度: {twist}")
```

### 5. Wrench

力/力矩结构。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `force` | `Vector3` | 力 | 牛顿 |
| `torque` | `Vector3` | 力矩 | 牛顿·米 |

**示例**：

```
import agibot_gdk

wrench = agibot_gdk.Wrench()
wrench.force.z = 10.0
wrench.torque.z = 5.0
print(f"力/力矩: {wrench}")
```

## 传感器数据类型

### 1. Image

图像数据结构。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `width` | `int` | 图像宽度 | 像素 |
| `height` | `int` | 图像高度 | 像素 |
| `timestamp_ns` | `int` | 时间戳 | 纳秒 |
| `data` | `numpy.ndarray` | 图像数据 | 字节 |
| `encoding` | `Encoding` | 编码格式 | 枚举 |
| `color_format` | `ColorFormat` | 颜色格式 | 枚举 |
| `bit_depth` | `int` | 位深度 | 位 |

**Encoding 枚举值**：

| 枚举值 | 描述 |
| --- | --- |
| `UNCOMPRESSED` | 未压缩原始数据 |
| `JPEG` | JPEG 压缩 |
| `PNG` | PNG 压缩 |

**ColorFormat 枚举值**：

| 枚举值 | 描述 |
| --- | --- |
| `RGB` | 红绿蓝（8位每通道） |
| `BGR` | 蓝绿红（OpenCV 默认格式） |
| `RGBA` | 红绿蓝+透明通道 |
| `BGRA` | 蓝绿红+透明通道 |
| `YUV420` | YUV 4:2:0 |
| `YUV422` | YUV 4:2:2 |
| `YUV444` | YUV 4:4:4 |
| `NV12` | YUV 4:2:0 半平面（Y + 交错 UV） |
| `NV21` | YUV 4:2:0 半平面（Y + 交错 VU） |
| `GRAY8` | 8位灰度 |
| `GRAY16` | 16位灰度 |
| `BAYER_RGGB` | Bayer 阵列 RGGB |
| `BAYER_BGGR` | Bayer 阵列 BGGR |
| `BAYER_GBRG` | Bayer 阵列 GBRG |
| `BAYER_GRBG` | Bayer 阵列 GRBG |
| `RS2_FORMAT_Z16` | RealSense 深度图（16位无符号整数，单位毫米） |

**示例**：

```
import agibot_gdk
import time

# 获取图像数据
camera = agibot_gdk.Camera()
image = camera.get_latest_image(agibot_gdk.CameraType.kHeadStereoLeft, 1000.0)

if image is not None:
    print(f"图像尺寸: {image.width} x {image.height}")
    print(f"时间戳: {image.timestamp_ns}")
    print(f"数据大小: {image.data.size} 字节")
    print(f"编码格式: {image.encoding}")
    print(f"颜色格式: {image.color_format}")
    print(f"位深度: {image.bit_depth}")
```

### 2. PointCloud

点云数据结构。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `width` | `int` | 点云宽度 | 点 |
| `height` | `int` | 点云高度 | 点 |
| `fields` | `list[PointField]` | 点云字段 | 列表 |
| `point_step` | `int` | 点步长 | 字节 |
| `row_step` | `int` | 行步长 | 字节 |
| `is_bigendian` | `bool` | 是否大端序 | 布尔 |
| `is_dense` | `bool` | 是否稠密 | 布尔 |
| `data` | `numpy.ndarray` | 点云数据 | 字节 |
| `timestamp_ns` | `int` | 时间戳 | 纳秒 |

**示例**：

```
import agibot_gdk
import time

# 获取点云数据
lidar = agibot_gdk.Lidar()
pointcloud = lidar.get_latest_pointcloud(agibot_gdk.LidarType.kLidarFront, 1000.0)

if pointcloud is not None:
    print(f"点云尺寸: {pointcloud.width} x {pointcloud.height}")
    print(f"时间戳: {pointcloud.timestamp_ns}")
    print(f"数据大小: {pointcloud.data_size} 字节")
    print(f"字段数量: {len(pointcloud.fields)}")
    for field in pointcloud.fields:
        print(f"  字段: {field.name}, 偏移: {field.offset}, 类型: {field.datatype}")
```

### 3. ImuData

IMU数据结构。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `angular_velocity` | `Vector3` | 角速度 | 弧度/秒 |
| `linear_acceleration` | `Vector3` | 线加速度 | 米/秒² |
| `timestamp_ns` | `int` | 时间戳 | 纳秒 |

**示例**：

```
import agibot_gdk
import time

# 获取IMU数据
imu = agibot_gdk.Imu()
imu_data = imu.get_latest_imu(agibot_gdk.ImuType.kImuChassis, 1000.0)

if imu_data is not None:
    print(f"角速度: ({imu_data.angular_velocity.x}, {imu_data.angular_velocity.y}, {imu_data.angular_velocity.z})")
    print(f"线加速度: ({imu_data.linear_acceleration.x}, {imu_data.linear_acceleration.y}, {imu_data.linear_acceleration.z})")
    print(f"时间戳: {imu_data.timestamp_ns}")
```

### 4. CameraIntrinsic

相机内参结构。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `intrinsic` | `list[float]` | 内参 [fx, fy, cx, cy] | 像素 |
| `distortion` | `list[float]` | 畸变参数 [k1, k2, p1, p2, k3, k4, k5, k6] | 无单位 |

**示例**：

```
import agibot_gdk
import time

# 获取相机内参
camera = agibot_gdk.Camera()
intrinsic = camera.get_camera_intrinsic(agibot_gdk.CameraType.kHeadStereoLeft)

print(f"内参:")
print(f"  fx: {intrinsic.intrinsic[0]}")
print(f"  fy: {intrinsic.intrinsic[1]}")
print(f"  cx: {intrinsic.intrinsic[2]}")
print(f"  cy: {intrinsic.intrinsic[3]}")

print(f"畸变参数:")
for i, dist in enumerate(intrinsic.distortion):
    print(f"  k{i+1}: {dist}")
```

## 机器人控制数据类型

### 1. JointState

关节状态结构。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `name` | `str` | 关节名称 | 字符串 |
| `mode` | `int` | 关节模式 | 整数 |
| `position` | `float` | 关节位置 | 弧度 |
| `velocity` | `float` | 关节速度 | 弧度/秒 |
| `effort` | `float` | 关节力矩 | 牛顿·米 |
| `motor_position` | `float` | 电机位置 | 弧度 |
| `motor_velocity` | `float` | 电机速度 | 弧度/秒 |
| `motor_current` | `float` | 电机电流 | 安培 |
| `error_code` | `int` | 错误码 | 整数 |

**示例**：

```
import agibot_gdk
import time

# 获取关节状态
robot = agibot_gdk.Robot()
joint_states = robot.get_joint_states()

print(f"关节数量: {joint_states['nums']}")
for state in joint_states['states']:
    print(f"关节: {state['name']}")
    print(f"  位置: {state['position']}")
    print(f"  速度: {state['velocity']}")
    print(f"  力矩: {state['effort']}")
```

### 2. JointControlReq

关节控制请求结构。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `joint_names` | `list[str]` | 关节名称列表 | 字符串列表 |
| `joint_positions` | `list[float]` | 关节位置列表 | 弧度 |
| `joint_velocities` | `list[float]` | 关节速度列表 | 弧度/秒 |
| `life_time` | `float` | 生命周期 | 秒 |
| `detail` | `str` | 详细信息 | 字符串 |

**示例**：

```
import agibot_gdk
import time

# 创建关节控制请求
joint_control_req = agibot_gdk.JointControlReq()
joint_control_req.joint_names = ["idx01_body_joint1", "idx02_body_joint2"]
joint_control_req.joint_positions = [0.0, 0.0]
joint_control_req.joint_velocities = [0.1, 0.1]
joint_control_req.life_time = 5.0

# 执行关节控制
robot = agibot_gdk.Robot()
result = robot.joint_control_request(joint_control_req)
if result == agibot_gdk.GDKRes.kSuccess:
    print("关节控制成功")
```

### 3. EndEffectorPose

末端执行器位姿控制结构。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `group` | `int` | 控制组 | 整数 |
| `left_end_effector_pose` | `Pose` | 左末端执行器位姿 | 位姿 |
| `right_end_effector_pose` | `Pose` | 右末端执行器位姿 | 位姿 |
| `life_time` | `float` | 生命周期 | 秒 |

**示例**：

注意：end\_effector\_pose\_control()需要对action进行插值处理才可正常响应，本示例仅说明EndEffectorPose的数据结构

```
import agibot_gdk
import time

# 创建末端执行器位姿控制请求
# 该请求需要结合当前末端执行器位姿进行插补，本示例仅作说明
end_pose = agibot_gdk.EndEffectorPose()
end_pose.group = agibot_gdk.EndEffectorControlGroup.kBothArms
end_pose.left_end_effector_pose.position.x = 0.3
end_pose.left_end_effector_pose.position.y = 0.2
end_pose.left_end_effector_pose.position.z = 0.4
end_pose.left_end_effector_pose.orientation.x = 0.0
end_pose.left_end_effector_pose.orientation.y = 0.0
end_pose.left_end_effector_pose.orientation.z = 0.0
end_pose.left_end_effector_pose.orientation.w = 1.0
end_pose.right_end_effector_pose.position.x = 0.3
end_pose.right_end_effector_pose.position.y = 0.3
end_pose.right_end_effector_pose.position.z = -0.2
end_pose.right_end_effector_pose.orientation.x = 0.0
end_pose.right_end_effector_pose.orientation.y = 0.0
end_pose.right_end_effector_pose.orientation.z = 0.0
end_pose.right_end_effector_pose.orientation.w = 1.0
end_pose.life_time = 5.0

# 执行末端执行器位姿控制
robot = agibot_gdk.Robot()
result = robot.end_effector_pose_control(end_pose)
if result == agibot_gdk.GDKRes.kSuccess:
    print("末端执行器位姿控制成功")
```

### 4. Impedance

阻抗参数结构，用于描述关节或笛卡尔轴的阻抗控制参数。

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| `kp` | `float` | 刚度系数 |
| `ki` | `float` | 积分系数，当前实现暂未启用 |
| `kv` | `float` | 阻尼系数（速度反馈），当前实现暂未启用 |

**示例**：

```
import agibot_gdk

imp = agibot_gdk.Impedance()
imp.kp = 200.0
imp.ki = 0.0
imp.kv = 20.0
```

### 5. CollisionDetectionConfig

碰撞检测配置结构。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `is_enabled` | `bool` | 是否启用碰撞检测 | 布尔值 |
| `sensitivity` | `int` | 灵敏度等级，1-3，数值越大越灵敏 | 整数 |
| `checkout_timeout_ms` | `int` | 状态机候选 / 恢复时间窗 | 毫秒 |

**示例**：

```
import agibot_gdk

config = agibot_gdk.CollisionDetectionConfig()
config.is_enabled = True
config.sensitivity = 2
config.checkout_timeout_ms = 500
```

### 6. InertiaInfo

惯性信息结构，用于描述单个臂的末端负载惯性参数。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `mass` | `float` | 负载质量 | kg |
| `center_of_mass` | `list[float]` | 负载重心位置（长度 3） | m |
| `inertia` | `list[float]` | 负载惯性矩阵（长度 9，3x3 按行展开） | kg·m² |

**示例**：

```
import agibot_gdk

inertia = agibot_gdk.InertiaInfo()
inertia.mass = 0.5
inertia.center_of_mass = [0.0, 0.0, 0.05]
inertia.inertia = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
```

### 7. PayloadInfo

负载信息结构，包含双臂末端负载。

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| `left_arm` | `InertiaInfo` | 左臂末端负载信息 |
| `right_arm` | `InertiaInfo` | 右臂末端负载信息 |

**示例**：

```
import agibot_gdk

info = agibot_gdk.PayloadInfo()
info.left_arm.mass = 0.5
info.left_arm.center_of_mass = [0.0, 0.0, 0.05]
info.left_arm.inertia = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
info.right_arm.mass = 0.5
info.right_arm.center_of_mass = [0.0, 0.0, 0.05]
info.right_arm.inertia = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
```

### 8. TcpTransform

TCP（工具中心点）变换结构，描述单个臂的工具坐标系相对末端 `end_link`（法兰中心）的位姿。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `position` | `list[float]` | TCP 相对 flange 的平移 `[x, y, z]`（长度 3） | m |
| `euler_zyx` | `list[float]` | 欧拉角 `[roll, pitch, yaw]`（长度 3），旋转 `R = Rz(yaw) * Ry(pitch) * Rx(roll)` | rad |

**示例**：

```
import agibot_gdk

tcp = agibot_gdk.TcpTransform()
tcp.position = [0.0, 0.0, 0.1]
tcp.euler_zyx = [0.0, 0.0, 0.0]
```

### 9. DualArmTcpTransform

双臂 TCP 变换结构，包含左右臂末端 TCP 变换。

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| `left_arm` | `TcpTransform` | 左臂末端 TCP 变换 |
| `right_arm` | `TcpTransform` | 右臂末端 TCP 变换 |

**示例**：

```
import agibot_gdk

transform = agibot_gdk.DualArmTcpTransform()
transform.left_arm.position = [0.0, 0.0, 0.1]
transform.left_arm.euler_zyx = [0.0, 0.0, 0.0]
transform.right_arm.position = [0.0, 0.0, 0.1]
transform.right_arm.euler_zyx = [0.0, 0.0, 0.0]
```

### 10. LedStripControl

单路灯带控制项，用于 `Interaction.set_led_strips()` 接口。一次调用最多包含 7 项（每路一条）。

| 属性名 | 类型 | 描述 | 取值范围 |
| --- | --- | --- | --- |
| `strip_id` | `LedStripId` | 灯带 ID | 见 [LedStripId](#7-ledstripid) |
| `mode` | `LedStripMode` | 显示模式 | 见 [LedStripMode](#8-ledstripmode) |
| `red` | `int` | 红色分量 | 0–255 |
| `green` | `int` | 绿色分量 | 0–255 |
| `blue` | `int` | 蓝色分量 | 0–255 |
| `speed` | `int` | 动画周期参数，单位 25 ms | 0–255（对应 0–6.375 s） |

**说明**：
- `speed` 仅对动画类模式（流水 / 反流水 / 三角波呼吸 / 正弦呼吸 / 闪烁）起作用；取值 0–255，对应 0–6.375 s，单位 25 ms；常亮模式可设为 0。流水 / 反流水模式下更容易观察到动画周期参数的变化；呼吸模式下变化可能不明显。

**示例**：

```
import agibot_gdk

ctrl = agibot_gdk.LedStripControl()
ctrl.strip_id = agibot_gdk.LedStripId.kHead
ctrl.mode = agibot_gdk.LedStripMode.kSequence
ctrl.red = 0
ctrl.green = 0
ctrl.blue = 255
ctrl.speed = 50  # 单位 25ms，流水类模式下 speed 变化更容易观察

# 下发时，即使只控制一条灯带，也需要传入列表：
# interaction.set_led_strips([ctrl])
```

## 地图数据类型

### 1. MapInfo

地图信息结构。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `id` | `int` | 地图ID | 整数 |
| `name` | `str` | 地图名称 | 字符串 |
| `status` | `int` | 地图状态 | 整数 |
| `counter` | `int` | 地图计数器 | 整数 |
| `timestamp_ns` | `int` | 时间戳 | 纳秒 |
| `gravity` | `Vector3` | 重力向量 | 米/秒² |
| `ground_cloud` | `PointCloud` | 地面点云地图 | 点云 |
| `nonground_cloud` | `PointCloud` | 非地面点云地图 | 点云 |
| `grid_map` | `OccupancyGrid` | 栅格地图 | 栅格 |
| `walls` | `list[list[Point3d]]` | 墙壁 | 点列表 |
| `infeasible_areas` | `list[list[Point3d]]` | 不可行区域 | 点列表 |
| `guide_pts` | `list[GuidePtInfo]` | 引导点 | 引导点列表 |

**示例**：

```
import agibot_gdk
import time

# 获取地图信息
map_manager = agibot_gdk.Map()

# 获取地图信息前，先完成建图
try:
    map_info = map_manager.get_map(1)
except Exception as e:
    print(f"获取地图信息失败: {e}")
    exit(1)

print(f"地图名称: {map_info.name}")
print(f"地图ID: {map_info.id}")
print(f"地图状态: {map_info.status}")
print(f"重力向量: ({map_info.gravity.x}, {map_info.gravity.y}, {map_info.gravity.z})")
print(f"墙壁数量: {len(map_info.walls)}")
print(f"不可行区域数量: {len(map_info.infeasible_areas)}")
print(f"引导点数量: {len(map_info.guide_pts)}")
```

### 2. GuidePtInfo

导航引导点信息结构，表示地图中预先标注的路径引导点。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `id` | `int` | 引导点 ID | 整数 |
| `pt` | `Pose` | 引导点位姿（位置 + 朝向） | 位姿 |
| `type` | `int` | 引导点类型（保留字段，当前固定为 0） | 整数 |
| `timestamp_ns` | `int` | 时间戳 | 纳秒 |

**示例**：

```
for gp in map_info.guide_pts:
    print(f"引导点 {gp.id}: pos=({gp.pt.position.x:.2f}, {gp.pt.position.y:.2f})")
```

### 3. OccupancyGrid

占用栅格地图结构。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `width` | `int` | 地图宽度 | 栅格 |
| `height` | `int` | 地图高度 | 栅格 |
| `resolution` | `float` | 分辨率 | 米/栅格 |
| `origin` | `Pose` | 原点位姿 | 位姿 |
| `data` | `list[int]` | 栅格数据 | 整数列表 |
| `timestamp_ns` | `int` | 时间戳 | 纳秒 |

**示例**：

```
import agibot_gdk
import time

# 获取栅格地图
map_manager = agibot_gdk.Map()
map_info = map_manager.get_map(1)
grid_map = map_info.grid_map

print(f"栅格地图尺寸: {grid_map.width} x {grid_map.height}")
print(f"分辨率: {grid_map.resolution} 米/栅格")
print(f"原点位置: ({grid_map.origin.position.x}, {grid_map.origin.position.y}, {grid_map.origin.position.z})")
print(f"栅格数据大小: {len(grid_map.data)}")
```

## 坐标变换数据类型

### 1. Transform

坐标变换结构。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `translation` | `Vector3` | 平移向量 | 米 |
| `rotation` | `Quaternion` | 旋转四元数 | 无单位 |

**示例**：

```
import agibot_gdk
import time

# 获取坐标变换
tf = agibot_gdk.TF()
transform = tf.get_tf_from_base_link("arm_l_end_link")

print(f"平移: ({transform.translation.x}, {transform.translation.y}, {transform.translation.z})")
print(f"旋转: ({transform.rotation.x}, {transform.rotation.y}, {transform.rotation.z}, {transform.rotation.w})")
```

### 2. TransformStamped

带时间戳的坐标变换结构。

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `frame_id` | `str` | 父坐标系ID | 字符串 |
| `child_frame_id` | `str` | 子坐标系ID | 字符串 |
| `transform` | `Transform` | 变换信息 | 变换 |
| `timestamp_ns` | `int` | 时间戳 | 纳秒 |

**示例**：

```
import agibot_gdk
import time

# 获取所有坐标变换
tf = agibot_gdk.TF()
transforms = tf.get_all_tf_from_base_link()

for transform_stamped in transforms:
    print(f"坐标系: {transform_stamped.frame_id} -> {transform_stamped.child_frame_id}")
    print(f"  平移: ({transform_stamped.transform.translation.x}, {transform_stamped.transform.translation.y}, {transform_stamped.transform.translation.z})")
    print(f"  时间戳: {transform_stamped.timestamp_ns}")
```

## 使用注意事项

1. **数据有效性**：使用前请检查返回的数据是否为None
2. **枚举值使用**：使用枚举值而不是整数值，提高代码可读性
3. **数组访问**：使用Python列表语法访问数组元素
4. **字符串编码**：字符串使用UTF-8编码

## 应用场景

- **机器人控制**：使用关节状态和控制请求进行机器人运动控制
- **传感器数据处理**：处理图像、点云、IMU等传感器数据
- **地图管理**：使用地图信息进行导航和路径规划
- **坐标变换**：处理不同坐标系之间的变换关系
- **状态监控**：监控机器人各部件的工作状态
- **数据融合**：融合多种传感器数据进行环境感知
- **路径规划**：基于地图数据进行路径规划
- **视觉处理**：使用相机内参进行图像校正和3D重建



---

<!-- source: contents_reference_Python_uss -->

# UltrasonicRadar

# GDK UltrasonicRadar 接口文档（Python）

## 概述

UltrasonicRadar（超声波雷达）模块为G02机器人提供了获取实时超声波雷达数据的功能。通过Python接口，开发者可以方便地获取机器人的障碍物检测数据，适用于避障、导航、安全检测、近距离障碍物感知等多种场景。

## 接口说明

### UltrasonicRadar 类

该类封装了超声波雷达传感器的主要数据获取接口。

#### 1. `get_latest_ultrasonic_radar()`

- **功能**：获取最新的超声波雷达数据
- **参数**：无
- **返回值**：`dict`，包含超声波雷达数据的字典，失败时抛出异常

**返回值字典结构**：

| 键名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `timestamp_ns` | `int` | 时间戳（底盘传感器时间戳） | 纳秒 |
| `ultrasonic_radar_datas` | `list[dict]` | 超声波雷达数据列表 | 无单位 |

**`ultrasonic_radar_datas` 列表中每个字典的字段**：

| 键名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `id` | `int` | 超声波雷达ID | 无 |
| `distance_mm` | `int` | 检测到的距离 | 毫米 |
| `fault_state` | `int` | 故障状态（0表示正常，非0表示存在故障） | 无 |

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

radar = agibot_gdk.UltrasonicRadar()

# 获取最新数据
radar_data = radar.get_latest_ultrasonic_radar()

print(f"✅ 时间戳: {radar_data['timestamp_ns']} ns")
print(f"超声波雷达数量: {len(radar_data['ultrasonic_radar_datas'])}")

for data in radar_data['ultrasonic_radar_datas']:
    print(f"  雷达[{data['id']}]: "
          f"距离={data['distance_mm']} mm, "
          f"故障状态={data['fault_state']}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 2. `get_nearest_ultrasonic_radar()`

- **功能**：获取指定时间戳附近最近的超声波雷达数据
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `timestamp_ns` | `int` | 目标时间戳（纳秒） |

- **返回值**：`dict`，包含超声波雷达数据的字典，失败时抛出异常

**返回值字典结构**：

| 键名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `timestamp_ns` | `int` | 时间戳（底盘传感器时间戳） | 纳秒 |
| `ultrasonic_radar_datas` | `list[dict]` | 超声波雷达数据列表 | 无单位 |

**`ultrasonic_radar_datas` 列表中每个字典的字段**：

| 键名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `distance_mm` | `int` | 检测到的距离 | 毫米 |
| `fault_state` | `int` | 故障状态（0表示正常，非0表示存在故障） | 无 |

**注意**：`get_nearest_ultrasonic_radar()` 返回的雷达数据字典中不包含 `id` 字段。

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

radar = agibot_gdk.UltrasonicRadar()

# 先获取最新数据
latest_data = radar.get_latest_ultrasonic_radar()
print(f"✅ 最新数据时间戳: {latest_data['timestamp_ns']} ns")

# 查找最近的数据（往前1秒）
target_timestamp = latest_data['timestamp_ns'] - 1000000000  # 1秒 = 1,000,000,000 纳秒
nearest_data = radar.get_nearest_ultrasonic_radar(target_timestamp)

print(f"✅ 最近数据时间戳: {nearest_data['timestamp_ns']} ns")
time_diff = abs(nearest_data['timestamp_ns'] - target_timestamp)
print(f"时间差: {time_diff} ns")
print(f"超声波雷达数量: {len(nearest_data['ultrasonic_radar_datas'])}")

for i, data in enumerate(nearest_data['ultrasonic_radar_datas']):
    print(f"  雷达[{i}]: "
          f"距离={data['distance_mm']} mm, "
          f"故障状态={data['fault_state']}")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 3. `get_ultrasonic_radar_fps()`

- **功能**：获取超声波雷达数据采集帧率
- **参数**：无
- **返回值**：`float`，超声波雷达帧率（FPS），失败时抛出异常
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

radar = agibot_gdk.UltrasonicRadar()

# 获取帧率
fps = radar.get_ultrasonic_radar_fps()
print(f"超声波雷达帧率: {fps} fps")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 4. `get_ultrasonic_radar_latency()`

- **功能**：获取超声波雷达数据延迟统计信息
- **参数**：

| 参数名 | 类型 | 描述 |
| --- | --- | --- |
| `window_seconds` | `float` | 统计窗口时间（秒），默认10.0秒 |

- **返回值**：`LatencyStats`对象，包含延迟统计信息，失败时抛出异常

**LatencyStats 对象属性**：

| 属性名 | 类型 | 描述 | 单位 |
| --- | --- | --- | --- |
| `max_latency_ms` | `float` | 最大延迟 | 毫秒 |
| `avg_latency_ms` | `float` | 平均延迟 | 毫秒 |
| `p99_latency_ms` | `float` | 99分位延迟 | 毫秒 |
| `p999_latency_ms` | `float` | 99.9分位延迟 | 毫秒 |
| `p9999_latency_ms` | `float` | 99.99分位延迟 | 毫秒 |

- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

radar = agibot_gdk.UltrasonicRadar()

# 等待一段时间收集数据
time.sleep(10)

# 获取延迟统计
latency = radar.get_ultrasonic_radar_latency(10.0)

print("超声波雷达延迟统计:")
print(f"  最大延迟: {latency.max_latency_ms} ms")
print(f"  平均延迟: {latency.avg_latency_ms} ms")
print(f"  P99延迟: {latency.p99_latency_ms} ms")
print(f"  P99.9延迟: {latency.p999_latency_ms} ms")
print(f"  P99.99延迟: {latency.p9999_latency_ms} ms")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

#### 5. `close_ultrasonic_radar()`

- **功能**：关闭超声波雷达DDS连接
- **参数**：无
- **返回值**：`GDKRes`，操作结果状态码。成功时返回`GDKRes.kSuccess`
- **示例**：

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

radar = agibot_gdk.UltrasonicRadar()
print("UltrasonicRadar init")

# 使用超声波雷达...

# 关闭超声波雷达
if radar.close_ultrasonic_radar() != agibot_gdk.GDKRes.kSuccess:
    print("关闭超声波雷达失败")
else:
    print("超声波雷达关闭成功")

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

## 完整示例

```
import agibot_gdk
import time

# 初始化GDK系统
if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    print("GDK初始化失败")
    exit(1)
print("GDK初始化成功")

# 创建超声波雷达对象
radar = agibot_gdk.UltrasonicRadar()

# 获取最新数据
radar_data = radar.get_latest_ultrasonic_radar()

print("=== 超声波雷达数据 ===")
print(f"时间戳: {radar_data['timestamp_ns']} ns")
print(f"雷达数量: {len(radar_data['ultrasonic_radar_datas'])}")

for data in radar_data['ultrasonic_radar_datas']:
    print(f"  雷达[{data['id']}]: "
          f"距离={data['distance_mm']} mm, "
          f"故障状态={data['fault_state']}")

# 获取帧率
fps = radar.get_ultrasonic_radar_fps()
print(f"帧率: {fps} fps")

# 获取延迟统计
time.sleep(5)
latency = radar.get_ultrasonic_radar_latency(5.0)
print("延迟统计:")
print(f"  最大延迟: {latency.max_latency_ms} ms")
print(f"  平均延迟: {latency.avg_latency_ms} ms")

# 关闭接口
radar.close_ultrasonic_radar()

# 释放GDK系统资源
if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
    print("GDK释放失败")
else:
    print("GDK释放成功")
```

## 使用注意事项

1. **GDK初始化**：使用UltrasonicRadar功能前必须先调用`agibot_gdk.gdk_init()`初始化GDK系统
2. **GDK释放**：程序结束前必须调用`agibot_gdk.gdk_release()`释放GDK系统资源
3. **异常处理**：所有接口在失败时会抛出异常，建议使用try-except进行异常处理
4. **时间戳精度**：时间戳单位为纳秒，是底盘传感器的时间戳，可用于精确的时间同步
5. **距离单位**：距离单位为毫米（mm），使用时注意单位转换
6. **故障状态**：`fault_state`为0表示正常，非0值表示存在故障，使用时需要检查
7. **数据获取**：`get_latest_ultrasonic_radar()`返回当前最新数据，如果没有新数据可能抛出异常
8. **时间戳查找**：`get_nearest_ultrasonic_radar()`根据时间戳查找最接近的数据，如果时间戳超出范围可能抛出异常
9. **数据格式差异**：`get_latest_ultrasonic_radar()`返回的雷达数据包含`id`字段，而`get_nearest_ultrasonic_radar()`返回的数据不包含`id`字段
10. **帧率统计**：`get_ultrasonic_radar_fps()`需要等待一段时间（建议至少2秒）让数据积累后才能获得准确的帧率
11. **延迟统计**：`get_ultrasonic_radar_latency()`需要等待一段时间（建议至少10秒）让数据积累后才能获得准确的统计信息
12. **资源释放**：使用完毕后调用`close_ultrasonic_radar()`释放资源
13. **字典访问**：返回值是字典类型，使用字典键名访问数据，注意键名的大小写和拼写

## 应用场景

- **避障检测**：实时检测机器人周围的障碍物，用于避障决策
- **近距离感知**：检测近距离的障碍物，补充激光雷达的盲区
- **安全检测**：监控机器人周围的安全区域，防止碰撞
- **导航辅助**：为机器人导航提供近距离障碍物信息
- **停车辅助**：辅助机器人进行精确停车和定位
- **低速导航**：在低速移动时提供可靠的障碍物检测
- **多传感器融合**：与其他传感器（如激光雷达、相机）数据进行融合，提高感知精度
- **安全区域监控**：监控机器人周围的安全区域，确保安全运行
- **障碍物分类**：结合距离信息进行障碍物分类和识别
- **路径规划**：基于超声波雷达数据规划安全路径



---

<!-- source: contents_reference_ROS2_camera_node -->

# Camera

# Camera Node

相机相关话题说明

## Topic

| Topic | 描述 | 消息类型 |
| --- | --- | --- |
| /gdk/camera/hand\_left\_color | 腕部左侧彩色相机 | `sensor_msgs/msg/CompressedImage` |
| /gdk/camera/hand\_right\_color | 腕部右侧彩色相机 | `sensor_msgs/msg/CompressedImage` |
| /gdk/camera/head\_stereo\_left | 头部左目相机 | `sensor_msgs/msg/CompressedImage` |
| /gdk/camera/head\_stereo\_right | 头部右目相机 | `sensor_msgs/msg/CompressedImage` |
| /gdk/camera/head\_color | 头部 RGB 相机 | `sensor_msgs/msg/CompressedImage` |
| /gdk/camera/head\_depth | 头部深度相机 | `sensor_msgs/msg/Image` |

## 消息类型

### sensor\_msgs/msg/CompressedImage

| Field | Type | Description |
| --- | --- | --- |
| header | std\_msgs/Header | 消息头，包含时间戳和坐标系信息 |
| format | string | 压缩格式描述，例如 `jpeg`、`png` 或业务自定义格式 |
| data | uint8[] | 压缩后的字节数据 |

### sensor\_msgs/msg/Image

| Field | Type | Description |
| --- | --- | --- |
| header | std\_msgs/Header | 消息头，包含时间戳和坐标系信息 |
| height | uint32 | 图像高度 |
| width | uint32 | 图像宽度 |
| encoding | string | 图像编码格式 |
| is\_bigendian | uint8 | 字节序标记 |
| step | uint32 | 每行字节数 |
| data | uint8[] | 图像原始数据 |

## 使用方式

默认可查看的相机 topics：

- `/gdk/camera/hand_left_color`
- `/gdk/camera/hand_right_color`
- `/gdk/camera/head_stereo_left`
- `/gdk/camera/head_stereo_right`
- `/gdk/camera/head_color`
- `/gdk/camera/head_depth`

### ros2cli 使用示例

```
ros2 topic type /gdk/camera/head_stereo_left
ros2 topic hz /gdk/camera/head_stereo_left
ros2 topic echo /gdk/camera/head_stereo_left --once
```



---

<!-- source: contents_reference_ROS2_control_node -->

# Control

# Control Node

机器人控制相关话题。

## 关节状态获取

- **Topic:** `/gdk/joint_state`
- **消息类型:** `gdk_msgs::msg::JointState`

```
# The state of each joint (revolute or prismatic) is defined by:
#  * the control mode (csp 0, cst 1)
#  * the position of the joint (rad or m)
#  * the velocity of the joint (rad/s or m/s)
#  * the effort that is applied in the joint (Nm or N)
#  * the position of the motor (rad or m)
#  * the velocity of the motor (rad/s or m/s)
#  * the current of the motor (A)
#  * the error_code of the joint

std_msgs/Header header

string[] name
uint32[] mode
float64[] position
float64[] velocity
float64[] effort
float64[] motor_position
float64[] motor_velocity
float64[] motor_current
uint32[] error_code
```

### ros2cli 使用示例

```
ros2 topic type /gdk/joint_state
ros2 topic hz /gdk/joint_state
ros2 topic echo /gdk/joint_state --once
```

## 关节位置控制命令请求

- **Topic:** `/gdk/joint_pos_request`
- **消息类型:** `gdk_msgs::msg::JointPositionRequst`

```
# frame id 无用, timestamp发送时间
std_msgs/Header header
float64 lifetime

string[] joint_names
float64[] joint_positions
float64[] joint_velocities

string uuid
string detail
```

### 使用说明

- 功能：关节位置规划控制接口。控制命令发出后，目标位置执行完成后再返回结果。
- 参数说明：
- `lifetime` 表示生命周期，单位为秒。
- `joint_names`、`joint_positions`、`joint_velocities` 不应为空。
- `joint_names`、`joint_positions`、`joint_velocities` 三个列表长度应保持一致。
- `joint_positions` 中的目标值应位于对应关节限位范围内。
- 结果反馈：执行结果通过 `/gdk/joint_pos_response` 反馈。

### ros2cli 使用示例

```
ros2 topic type /gdk/joint_pos_request
ros2 topic hz /gdk/joint_pos_request
ros2 topic echo /gdk/joint_pos_request --once
```

### 发布示例

```
ros2 topic pub -t 2 /gdk/joint_pos_request gdk_msgs/msg/JointPositionRequst \
"{lifetime: 0.0, joint_names: ['idx21_arm_l_joint1'], joint_positions: [0.5], joint_velocities: [0.3], uuid: 'test_uuid', detail: 'ros2 cli test'}"
```

## 关节位置控制命令反馈

- **Topic:** `/gdk/joint_pos_response`
- **消息类型:** `gdk_msgs::msg::CommonResponse`

```
std_msgs/Header header

# 0: 命令执行成功
# 1: 命令执行失败
uint8 SUCCESS=0
uint8 FAIL=1
uint8 result

string uuid
string reason
```

### ros2cli 使用示例

```
ros2 topic type /gdk/joint_pos_response
ros2 topic hz /gdk/joint_pos_response
ros2 topic echo /gdk/joint_pos_response --once
```

## 末端控制状态信息

- **Topic:** `/gdk/motion_control_status`
- **消息类型:** `gdk_msgs::msg::MotionControlStatus`

```
std_msgs/Header header  # frame id 默认填写base_link, timestamp发送时间

# 反馈末端位姿状态
string[] frame_names   # 运控参数配置定义
geometry_msgs/Pose[] frame_poses

# 反馈自碰撞状态
# [collision_pair_1， collision_pair_2]组成一组碰撞对的frame名称
string[] collision_pairs_1
string[] collision_pairs_2

# 运控模式
uint8 MODE_STOP=0
uint8 MODE_SERVO=1
uint8 MODE_PLANNING=2
uint8 mode

# 实时控制的运控错误码
# 0: no error
uint8 error_code
string error_msg

geometry_msgs/Twist[] frame_twists
geometry_msgs/Wrench[] frame_wrenchs
```

### ros2cli 使用示例

```
ros2 topic type /gdk/motion_control_status
ros2 topic hz /gdk/motion_control_status
ros2 topic echo /gdk/motion_control_status --once
```

## 关节位置伺服控制

- **Topic:** `/gdk/joint_control`
- **消息类型:** `gdk_msgs::msg::JointPositionServo`

```
# frame id 无用, timestamp发送时间
std_msgs/Header header

float64 control_period

string[] joint_names
float64[] joint_positions
float64[] joint_velocities
```

### 使用说明

- 功能：关节位置伺服控制接口（非阻塞式）。
- 控制频率：建议以 100 Hz 的控制频率持续发送控制命令。
- 参数说明：
- `control_period` 表示控制周期，单位为秒，建议与控制频率一致或略大于控制周期。
- `joint_names`、`joint_positions` 不应为空。
- `joint_names` 与 `joint_positions` 的长度应保持一致。
- `joint_positions` 中的目标值应位于对应关节限位范围内。
- `joint_velocities` 为预留字段，可根据业务需要填写；若未使用，可填 `0.0`。

### 示例程序

```
#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from gdk_msgs.msg import JointState, JointPositionServo, MotionControlStatus

CONTROL_PERIOD = 0.01
RATE_HZ = 100.0
DURATION = 5.0

class JointServoLowDelayRosController(Node):
    def __init__(self):
        super().__init__("gdk_wbc_joint_servo_low_delay_controller")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.joint_state_msg = None
        self.motion_status_msg = None

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/gdk/joint_state",
            self.on_joint_state,
            qos,
        )

        self.motion_status_sub = self.create_subscription(
            MotionControlStatus,
            "/gdk/motion_control_status",
            self.on_motion_status,
            qos,
        )

        self.joint_servo_pub = self.create_publisher(
            JointPositionServo,
            "/gdk/joint_control",
            qos,
        )

    def on_joint_state(self, msg: JointState):
        self.joint_state_msg = msg

    def on_motion_status(self, msg: MotionControlStatus):
        self.motion_status_msg = msg

    def wait_for_joint_state(self, timeout_sec: float = 5.0):
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_state_msg is not None:
                return
        raise RuntimeError("等待 /gdk/joint_state 超时")

    def get_joint_position_by_name(self, joint_name: str) -> float:
        if self.joint_state_msg is None:
            raise RuntimeError("joint_state 还没收到")

        for i, name in enumerate(self.joint_state_msg.name):
            if name == joint_name:
                return self.joint_state_msg.motor_position[i]

        raise RuntimeError(f"Joint name {joint_name} not found")

    @staticmethod
    def interpolate_position(start_pos: float, target_pos: float, t: float) -> float:
        return start_pos + t * (target_pos - start_pos)

    def publish_servo(self, joint_names, joint_positions):
        msg = JointPositionServo()
        msg.control_period = CONTROL_PERIOD
        msg.joint_names = list(joint_names)
        msg.joint_positions = list(joint_positions)
        msg.joint_velocities = [0.0] * len(joint_names)
        self.joint_servo_pub.publish(msg)

    def execute_joint_servo_control(self, target_joint_names, target_positions):
        self.wait_for_joint_state()

        start_positions = []
        for joint_name in target_joint_names:
            pos = self.get_joint_position_by_name(joint_name)
            start_positions.append(pos)
            self.get_logger().info(f"{joint_name} 当前位姿: {pos:.3f} rad")

        n_steps = int(DURATION * RATE_HZ)
        dt = 1.0 / RATE_HZ
        start_time = time.time()

        self.get_logger().info(f"总步数: {n_steps}, 持续时间: {DURATION}s")

        for i in range(n_steps):
            t = float(i) / (n_steps - 1) if n_steps > 1 else 0.0
            current_positions = [
                self.interpolate_position(start_positions[j], target_positions[j], t)
                for j in range(len(target_joint_names))
            ]

            self.publish_servo(target_joint_names, current_positions)

            elapsed = time.time() - start_time
            expected_time = (i + 1) * dt
            sleep_time = expected_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

            rclpy.spin_once(self, timeout_sec=0.0)

        self.get_logger().info("轨迹发送完成，进入保持阶段")

        try:
            while rclpy.ok():
                self.publish_servo(target_joint_names, target_positions)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(dt)
        except KeyboardInterrupt:
            self.get_logger().info("收到 Ctrl+C，停止保持")

def main():
    rclpy.init()
    node = JointServoLowDelayRosController()

    try:
        target_joint_names = [
            "idx21_arm_l_joint1",
            "idx22_arm_l_joint2",
            "idx23_arm_l_joint3",
        ]
        target_positions = [0.0, 0.0, 0.0]

        node.get_logger().info("目标关节:")
        for name, pos in zip(target_joint_names, target_positions):
            node.get_logger().info(f"  {name}: {pos:.3f} rad")

        node.execute_joint_servo_control(target_joint_names, target_positions)

    except Exception as e:
        node.get_logger().error(f"执行失败: {e}")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == "__main__":
    main()
```

## 低时延关节位置伺服控制

- **Topic:** `/gdk/joint_control_low_delay`
- **消息类型:** `gdk_msgs::msg::JointPositionServo`

```
# frame id 无用, timestamp发送时间
std_msgs/Header header

float64 control_period

string[] joint_names
float64[] joint_positions
float64[] joint_velocities
```

### 使用说明

- 功能：低时延关节位置伺服控制接口（非阻塞式）
- 控制频率：建议以 100 Hz 的控制频率持续发送控制命令
- 注意：低延时模式无碰撞保护，使用时注意安全
- 参数说明：同 `/gdk/joint_control`

### 示例程序

同上 /gdk/joint\_control 的一致，只需更换 create\_publisher 中的 topic 名字即可



---

<!-- source: contents_reference_ROS2_imu_node -->

# IMU

# Imu Node

[IMU](../../../concepts/sensors/#imu) 相关话题说明

## Topic

| 话题名称 | 含义 |
| --- | --- |
| /gdk/imu/livox\_front | 前侧雷达 IMU |
| /gdk/imu/livox\_back | 背部雷达 IMU |
| /gdk/imu/chassis | 底盘 IMU |
| /gdk/imu/head\_rgbd | 头部 [RGBD](../../../concepts/sensors/#rgbd) IMU |

## 消息类型

### sensor\_msgs/msg/Imu

| Field | Type | Description |
| --- | --- | --- |
| header | std\_msgs/Header | 消息头，包含时间戳和坐标系信息 |
| angular\_velocity | geometry\_msgs/Vector3 | 三维角速度（rad/s） |
| linear\_acceleration | geometry\_msgs/Vector3 | 三维线性加速度（m/s²） |

## 使用方式

### 查看默认话题

默认可查看的 IMU topics：

- `/gdk/imu/livox_front`
- `/gdk/imu/livox_back`
- `/gdk/imu/chassis`
- `/gdk/imu/head_rgbd`

### ros2cli 使用示例

```
ros2 topic type /gdk/imu/chassis
ros2 topic hz /gdk/imu/chassis
ros2 topic echo /gdk/imu/chassis --once
```



---

<!-- source: contents_reference_ROS2_lidar_node -->

# Lidar

# Lidar Node

雷达相关话题说明

## Topic

| 话题名称 | 含义 |
| --- | --- |
| /gdk/lidar/livox\_front | 前侧激光雷达 |
| /gdk/lidar/livox\_back | 后侧激光雷达 |

## 消息类型

### sensor\_msgs/msg/PointCloud2

| Field | Type | Description |
| --- | --- | --- |
| header | std\_msgs/Header | 消息头，包含时间戳和坐标系信息 |
| height | uint32 | 点云的高度（如果点云是有组织的，否则为1） |
| width | uint32 | 点云的宽度（点的数量） |
| fields | PointField[] | 描述点云中每个点的字段结构 |
| is\_bigendian | bool | 数据字节序（true表示大端序） |
| point\_step | uint32 | 单个点占用的字节数 |
| row\_step | uint32 | 一行数据占用的字节数 |
| data | uint8[] | 点云的原始二进制数据 |
| is\_dense | bool | 如果为true，表示所有点都包含有效数据 |

#### PointField

| Field | Type | Description |
| --- | --- | --- |
| name | string | 字段名称（如：`x`、`y`、`z`、`rgb` 等） |
| offset | uint32 | 字段在点数据中的字节偏移量 |
| datatype | uint8 | 数据类型（1表示INT8，2表示UINT8，3表示INT16，4表示UINT16，5表示INT32，6表示UINT32，7表示FLOAT32，8表示FLOAT64） |
| count | uint32 | 该字段的元素数量 |

## 使用方式

### 查看默认话题

默认可查看的 Lidar topics：

- `/gdk/lidar/livox_front`
- `/gdk/lidar/livox_back`

### ros2cli 使用示例

```
ros2 topic type /gdk/lidar/livox_front
ros2 topic hz /gdk/lidar/livox_front
ros2 topic echo /gdk/lidar/livox_front --once
```



---

<!-- source: contents_reference_ROS2_odom_node -->

# Odom

# Odom Node

里程计状态相关话题

> 该话题需要在定位有效后才会持续输出数据

## Topic

| 话题名称 | 含义 |
| --- | --- |
| /gdk/odom\_info | 机器人定位与运动状态 |

## 消息类型

### gdk\_msgs/msg/OdomInfo

```
std_msgs/Header header
string child_frame_id

geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

bool is_stationary
bool is_sliping
int32 loc_confidence
int32 loc_state
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 velocity_body
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 ang_vel
geometry_msgs/Vector3 orientation_euler
```

## 使用方式

### ros2cli 使用示例

```
ros2 topic type /gdk/odom_info
ros2 topic hz /gdk/odom_info
ros2 topic echo /gdk/odom_info --once
```

### 字段说明

- `pose`：六自由度位姿及协方差；
- `twist`：线速度与角速度及协方差；
- `is_stationary`：机器人是否静止；
- `is_sliping`：机器人是否正在打滑；
- `loc_confidence`：定位置信度，范围 1~100，数值越大表示置信度越高；
- `loc_state`：定位状态，`0` 表示定位成功，`1` 表示定位丢失；
- `velocity`：世界坐标系下速度；
- `velocity_body`：机体坐标系下速度；
- `acceleration`：加速度；
- `ang_vel`：角速度；
- `orientation_euler`：欧拉角姿态，单位为弧度。



---

<!-- source: contents_reference_ROS2_tf_node -->

# TF

# TF Node

TF 坐标变换相关话题

## Topic

| 话题名称 | 含义 |
| --- | --- |
| /gdk/tf | 机器人 TF 坐标变换 |

## 消息类型

### tf2\_msgs/msg/TFMessage

| Field | Type | Description |
| --- | --- | --- |
| transforms | geometry\_msgs/TransformStamped[] | 一组坐标系变换 |

## 使用方式

### ros2cli 使用示例

```
ros2 topic type /gdk/tf
ros2 topic hz /gdk/tf
ros2 topic echo /gdk/tf --once
```

### 说明

- `/gdk/tf` 用于发布机器人坐标系之间的 TF 变换关系；
- 消息中的 `transforms` 数组包含多组 `geometry_msgs/TransformStamped`；
- 使用 TF 相关工具或程序时，应先完成 ROS2 环境初始化。



---

<!-- source: contents_reference_error_handling -->

# 错误处理

# 错误处理

Python接口在发生错误时会抛出异常，同时打印错误信息；
C++接口不抛出异常，所有接口通过\*\*返回值\*\*传递执行结果。

## GDKRes 返回码

所有 GDK 接口（Python 和 C++）的主返回类型为 `GDKRes`：

| 枚举值 | 整数值 | 含义 |
| --- | --- | --- |
| `GDKRes.kSuccess` | 0 | 调用成功 |
| `GDKRes.kInvalidInput` | 1 | 输入参数无效（空指针、超限、类型错误等） |
| `GDKRes.kInvalidOutput` | 2 | 输出参数无效（输出缓冲区为空等） |
| `GDKRes.kRuntimeError` | 3 | 运行时错误（网络超时、内部状态异常等） |
| `GDKRes.kUnknown` | 4 | 未知错误 |

PythonC++

```
import agibot_gdk

robot = agibot_gdk.Robot()
res = robot.move_head_joint([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

if res != agibot_gdk.GDKRes.kSuccess:
    print(f"调用失败: {res}")
```

```
agibot::gdk::Robot robot;
auto res = robot.MoveHeadJoint({0, 0, 0}, {0, 0, 0});

if (res != agibot::gdk::GDKRes::kSuccess) {
    std::cerr << "调用失败: " << static_cast<int>(res) << std::endl;
}
```

## 常见错误原因与处理

| 错误码 | 常见触发场景 | 处理建议 |
| --- | --- | --- |
| `kInvalidInput` | 关节角度超出限位、位置列表长度不匹配、空指针传参 | 检查参数值是否在合法范围内，参见各接口的「注意事项」 |
| `kRuntimeError` | 网络超时（`Config request timeout`）、[机器人端](../../concepts/hybrid_deploy/#机器人端)服务未就绪 | 确认 GDK 服务运行中；检查网络连通性；参见[开发规范](../guidelines/)中的「Robot 对象的异步初始化」 |
| `kRuntimeError` | `Failed to get config parser` | 通常是 `Robot` 对象初始化未完成即调用方法，需等待约 1 秒 |
| `kInvalidOutput` | 输出缓冲区未分配或为空 | 检查传入的输出参数是否已正确初始化 |

## 关节错误码

关节级的硬件错误码（与 `GDKRes` 无关）记录在 `JointState.error_code` 字段中，完整列表见[关节错误码附录](../../appendices/joint_error_code/)。



---

<!-- source: contents_reference_guidelines -->

# 开发规范

# 开发规范

本页列出适用于所有 GDK 模块的通用开发规则。各接口页的「注意事项」只记录该接口特有的限制，通用规则不再重复。

## 初始化与释放

**所有模块使用前必须先调用 `gdk_init()`**，否则创建任何 GDK 对象都会报 `GDK is not initialized`。

PythonC++

```
import agibot_gdk

if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
    raise RuntimeError("GDK 初始化失败")

# 之后才能创建模块对象
robot = agibot_gdk.Robot()
```

```
if (agibot::gdk::GDKInit() != agibot::gdk::GDKRes::kSuccess) {
    return -1;
}

agibot::gdk::Robot robot;
```

程序退出前调用 `gdk_release()` / `GDKRelease()` 释放资源。

## Robot 对象的异步初始化

`Robot` 构造函数异步完成初始化。创建对象后\*\*立即调用方法会超时失败\*\*，需等待初始化完成：

PythonC++

```
robot = agibot_gdk.Robot()
time.sleep(1)  # 等待异步初始化
```

```
agibot::gdk::Robot robot;
std::this_thread::sleep_for(std::chrono::seconds(1));
```

## 运行前环境配置

运行任何 GDK 程序前，需先声明动态库路径：

```
source ~/.cache/agibot/app/env.sh
```

## 线程安全

各模块对象\*\*非线程安全\*\*，避免多线程并发调用同一对象的方法。如需多线程使用，请在调用侧自行加锁。



---

<!-- source: contents_reference -->

# 接口文档

# 接口导引

按使用场景快速定位对应的类和方法。同一功能在 Python / C++ 接口中名称一致（Python 用下划线命名，C++ 用大驼峰），ROS2 通过话题/服务交互，详见各语言页面。

使用任何接口前，请先阅读[开发规范](guidelines/)了解初始化、线程安全等通用前提。

## 系统生命周期

| 意图 | Python | C++ |
| --- | --- | --- |
| 初始化 GDK | `gdk_init()` | `GDKInit()` |
| 释放 GDK 资源 | `gdk_release()` | `GDKRelease()` |

→ [Python Common](Python/common/) · [C++ Common](C%2B%2B/common/)

---

## 机器人状态查询

**`Robot` 类**：直接操作硬件层，负责关节级读写——读取状态、发关节指令、控制末端执行器，无任务概念。

| 意图 | 类 · 方法 |
| --- | --- |
| 获取所有关节角度 / 力矩 | `Robot.get_joint_states()` |
| 获取关节限位 | `Robot.get_joint_limits()` |
| 获取全身运动状态 | `Robot.get_whole_body_status()` |
| 获取末端执行器状态（夹爪类型等） | `Robot.get_end_state()` |
| 获取末端位姿控制当前参考坐标系 | `Robot.get_end_pose_control_frame()` |
| 获取底盘电源状态 | `Robot.get_chassis_power_state()` |

→ [Python Robot](Python/robot/) · [C++ Robot](C%2B%2B/robot/)

---

## 关节运动控制

| 意图 | 类 · 方法 |
| --- | --- |
| 控制头部关节 | `Robot.move_head_joint()` |
| 控制腰部关节 | `Robot.move_waist_joint()` |
| 控制手臂关节 | `Robot.move_arm_joint()` |
| 批量发送关节指令 | `Robot.joint_control_request()` |
| 伺服模式关节控制（高频） | `Robot.joint_servo_control()` |

→ [Python Robot](Python/robot/) · [C++ Robot](C%2B%2B/robot/)

---

## 末端执行器控制

| 意图 | 类 · 方法 |
| --- | --- |
| 按关节角度控制末端 | `Robot.move_end_effector_joint()` |
| 按笛卡尔位置控制末端 | `Robot.move_ee_pos()` |
| 末端位姿控制 | `Robot.end_effector_pose_control()` |

→ [Python Robot](Python/robot/) · [C++ Robot](C%2B%2B/robot/)

---

## 底盘导航与移动

**`Pnc` 类**（Planning and Control）：任务层规划与控制，负责自主导航、路径规划、底盘移动等有状态的任务调度。

| 意图 | 类 · 方法 |
| --- | --- |
| 申请底盘控制权 | `Pnc.RequestChassisControl()` |
| 速度控制底盘移动 | `Pnc.MoveChassis()` |
| 导航到目标点（普通精度） | `Pnc.NormalNavi()` |
| 导航到目标点（高精度） | `Pnc.HighPrecisionNavi()` |
| 相对位移移动 | `Pnc.RelativeMove()` |
| 取消当前导航任务 | `Pnc.CancelTask()` |
| 暂停 / 恢复任务 | `Pnc.PauseTask()` / `Pnc.ResumeTask()` |
| 查询任务状态 | `Pnc.GetTaskState()` |

→ [Python PNC](Python/pnc/) · [C++ PNC](C%2B%2B/pnc/)

---

## SLAM 定位与建图

| 意图 | 类 · 方法 |
| --- | --- |
| 启动 / 停止 SLAM | `Slam` 类 |
| 获取当前地图列表 | `Map.get_map_list()` |
| 切换地图 | `Map.switch_map()` |

→ [Python SLAM](Python/slam/) · [Python Map](Python/map/) · [C++ SLAM](C%2B%2B/slam/) · [C++ Map](C%2B%2B/map/)

---

## 传感器数据

| 意图 | 类 · 方法 |
| --- | --- |
| 获取相机图像 | `Camera.get_latest_image()` |
| 获取激光雷达点云 | `Lidar.get_latest_pointcloud()` |
| 获取 [IMU](../concepts/sensors/#imu) 数据（加速度 / 角速度） | `Imu.get_latest_imu()` |
| 获取超声波雷达距离 | `UltrasonicRadar.get_latest_ultrasonic_radar()` |

→ [Python Camera](Python/camera/) · [Python Lidar](Python/lidar/) · [Python IMU](Python/imu/) · [Python USS](Python/uss/)

---

## 坐标变换

| 意图 | 类 · 方法 |
| --- | --- |
| 获取某坐标系到 base\_link 的变换 | `TF.get_tf_from_base_link(frame_id)` |
| 获取所有坐标系变换 | `TF.get_all_tf_from_base_link()` |
| 查询传感器标定坐标变换 | `TF.get_tf_from_sensor(SensorExtrinsicType)` |
| 查询任意两坐标系间变换 | `TF.lookup_transform_latest()` |

→ [Python TF](Python/tf/) · [C++ TF](C%2B%2B/tf/)

---

## 人机交互

| 意图 | 类 · 方法 |
| --- | --- |
| 语音播报（TTS） | `Interaction` 类 |
| 屏幕显示控制 | `Interaction` 类 |
| 音视频播放 | `Interaction` 类 |

→ [Python Interaction](Python/interaction/) · [C++ Interaction](C%2B%2B/interaction/)

---

## 数据类型与枚举

常用枚举（`GDKRes`、`CameraType`、`SensorExtrinsicType` 等）和数据结构（`Transform`、`JointState`、`Image` 等）的完整定义。

→ [Python Types](Python/types/) · [C++ Common](C%2B%2B/common/)



---

<!-- source: contents_tools_camera_web_viewer -->

# Web相机查看器 (camera_web_viewer.py)

# Web相机查看器 (camera\_web\_viewer.py)

`camera_web_viewer.py` 是一个基于 Web 的相机图像查看工具，可以在浏览器中实时查看机器人所有相机的图像画面，支持多相机同时显示。

## 功能特性

- Web 界面，无需安装客户端
- 支持多相机同时显示（最多9个相机）
- 自动检测可用相机
- 实时图像刷新（10fps）
- 智能缓存机制，减少重复处理
- 支持多种图像格式（JPEG、PNG、未压缩RGB/BGR/灰度、深度图）
- 深度相机伪彩色显示
- 显示相机信息（尺寸、帧率）

## 安装要求

### 必需依赖

- Python 3.10+
- GDK Python 包 (`agibot_gdk`)
- Flask（用于 Web 服务器）

### 可选依赖

- OpenCV（推荐，用于图像处理和优化）
- NumPy（推荐，用于图像数据处理）

### 安装依赖

```
# 安装 Flask
pip install flask

# 安装 OpenCV（推荐）
pip install opencv-python

# 安装 NumPy（通常随 OpenCV 一起安装）
pip install numpy
```

## 使用方法

### 1. 运行工具

```
cd ~/.cache/agibot/app/gdk/examples/python
source ~/.cache/agibot/app/env.sh
python3 camera_web_viewer.py
```

### 2. 访问 Web 界面

工具启动后，会在终端显示访问地址：

```
访问地址: http://0.0.0.0:5000
```

在浏览器中打开该地址即可查看相机画面。

### 3. 支持的相机类型

工具支持以下9种相机类型：

1. **头部后视鱼眼相机** (`kHeadBackFisheye`)
2. **头部左侧鱼眼相机** (`kHeadLeftFisheye`)
3. **头部右侧鱼眼相机** (`kHeadRightFisheye`)
4. **头部双目左相机** (`kHeadStereoLeft`)
5. **头部双目右相机** (`kHeadStereoRight`)
6. **左手彩色相机** (`kHandLeftColor`)
7. **右手彩色相机** (`kHandRightColor`)
8. **头部深度相机** (`kHeadDepth`)
9. **头部彩色相机** (`kHeadColor`)

### 4. 功能说明

#### 自动检测可用相机

工具会自动检测哪些相机可用，只显示可用的相机画面。如果某个相机不可用，不会在界面上显示。

#### 实时图像刷新

- 所有可用相机的图像每 100ms 自动刷新一次（10fps）
- 使用智能缓存机制，避免重复处理相同帧
- 显示处理统计信息（新帧数、缓存数、错误数）

#### 相机信息显示

每个相机卡片显示：
- 相机名称
- 图像尺寸（宽 x 高）
- 实时帧率（FPS）

#### 深度相机特殊处理

深度相机会自动进行以下处理：
- 深度值归一化到 0-255 范围
- 应用伪彩色映射（JET 色彩方案）
- 显示深度范围信息（最小/最大深度值）

## 配置选项

### 修改服务器地址和端口

在代码中修改 `WebCameraViewer` 的初始化参数：

```
viewer = WebCameraViewer(host='0.0.0.0', port=5000)
```

- `host`: 服务器监听地址（默认：`0.0.0.0`，允许外部访问）
- `port`: 服务器端口（默认：`5000`）

### 修改图像质量

在 `image_to_base64()` 方法中修改 `quality` 参数：

```
image_b64 = self.image_to_base64(decoded_image, quality=75)
```

- 范围：1-100
- 默认：75
- 值越大，图像质量越高，但文件也越大

### 修改刷新频率

在 HTML 模板中修改 `setInterval` 的时间间隔：

```
// 每100ms更新一次（10fps）
setInterval(updateAllImages, 100);
```

## 使用示例

### 示例1：本地访问

```
# 运行工具
python3 camera_web_viewer.py

# 在浏览器中打开
# http://localhost:5000
```

### 示例2：远程访问

#### 注意：需要确保机器人与开发机在同一局域网内

```
# 运行工具（监听所有网络接口）
python3 camera_web_viewer.py

# 在其他设备上访问
# http://<机器人IP>:5000
```

### 示例3：自定义端口

修改代码中的端口号：

```
viewer = WebCameraViewer(host='0.0.0.0', port=8080)
```

然后访问：`http://localhost:8080`



---

<!-- source: contents_tools_mc_example -->

# 关节控制工具 (mc_example.py)

# 关节控制工具 (mc\_example.py)

`mc_example.py` 是一个交互式的关节位置控制工具，可以通过键盘实时控制机器人关节位置，并支持播放预录制的动作序列。

## 功能特性

- 交互式键盘控制关节位置
- 支持所有22个关节的控制
- 播放预录制的动作序列
- 实时查看关节状态
- 支持多个动作文件切换

## 安装要求

在使用此工具前，请确保：

1. 已安装 GDK Python 包
2. 已配置好机器人连接（[混合部署](../../concepts/hybrid_deploy/)或机内部署）
3. 已准备好动作序列文件（JSON格式）

## 使用方法

### 1. 准备动作序列文件

工具会从 `saved_commands/` 目录读取 JSON 格式的动作序列文件。确保该目录存在并包含动作文件：

```
cd ~/.cache/agibot/app/gdk/examples/python
mkdir -p saved_commands
```

动作序列文件格式示例：

```
{
  "recorded_commands": [
    {
      "joint_names": ["idx21_arm_l_joint1", "idx22_arm_l_joint2"],
      "joint_positions": [0.5, -0.3]
    },
    {
      "joint_names": ["idx21_arm_l_joint1", "idx22_arm_l_joint2"],
      "joint_positions": [0.8, -0.5]
    }
  ]
}
```

### 2. 运行工具

```
cd ~/.cache/agibot/app/gdk/examples/python
source ~/.cache/agibot/app/env.sh
python3 mc_example.py
```

### 3. 键盘控制

工具启动后，使用以下键盘按键进行控制：

| 按键 | 功能 |
| --- | --- |
| `w` / `W` | 当前关节正向移动（+0.1 弧度） |
| `s` / `S` | 当前关节负向移动（-0.1 弧度） |
| `a` / `A` | 切换到上一个关节 |
| `d` / `D` | 切换到下一个关节 |
| `p` / `P` | 播放当前动作序列的下一帧 |
| `m` / `M` | 切换到下一个动作文件 |
| `q` / `Q` | 退出程序 |

### 4. 支持的关节列表

工具支持控制以下22个关节：

**腰部关节（5个）：**

- `idx01_body_joint1` - 腰部关节1
- `idx02_body_joint2` - 腰部关节2
- `idx03_body_joint3` - 腰部关节3
- `idx04_body_joint4` - 腰部关节4
- `idx05_body_joint5` - 腰部关节5

**头部关节（3个）：**

- `idx11_head_joint1` - 头部关节1
- `idx12_head_joint2` - 头部关节2
- `idx13_head_joint3` - 头部关节3

**左手臂关节（7个）：**

- `idx21_arm_l_joint1` - 左手臂关节1
- `idx22_arm_l_joint2` - 左手臂关节2
- `idx23_arm_l_joint3` - 左手臂关节3
- `idx24_arm_l_joint4` - 左手臂关节4
- `idx25_arm_l_joint5` - 左手臂关节5
- `idx26_arm_l_joint6` - 左手臂关节6
- `idx27_arm_l_joint7` - 左手臂关节7

**右手臂关节（7个）：**

- `idx61_arm_r_joint1` - 右手臂关节1
- `idx62_arm_r_joint2` - 右手臂关节2
- `idx63_arm_r_joint3` - 右手臂关节3
- `idx64_arm_r_joint4` - 右手臂关节4
- `idx65_arm_r_joint5` - 右手臂关节5
- `idx66_arm_r_joint6` - 右手臂关节6
- `idx67_arm_r_joint7` - 右手臂关节7

## 配置参数

可以在代码中修改以下参数：

- `_step`: 每次按键移动的步长（默认：0.1 弧度）
- `_speed`: 关节运动速度（默认：0.3）

## 使用示例

### 示例1：控制单个关节

1. 运行程序
2. 使用 `a`/`d` 键选择要控制的关节（例如：`idx21_arm_l_joint1`）
3. 使用 `w`/`s` 键调整关节位置
4. 观察机器人关节运动

### 示例2：播放动作序列

1. 准备动作序列文件并放在 `saved_commands/` 目录
2. 运行程序
3. 使用 `m` 键切换到要播放的动作文件
4. 按 `p` 键逐帧播放动作序列
5. 连续按 `p` 键可以播放完整动作

## 注意事项

**安全提示：**

- 使用前请确保机器人周围有足够的空间
- 注意关节的运动范围，避免超出限位
- 建议先在小幅度范围内测试
- 紧急情况下按 `q` 键退出程序

**使用限制：**

- 工具需要从 `saved_commands/` 目录读取 JSON 文件，如果目录为空或文件格式错误，程序会报错
- 动作序列文件必须是有效的 JSON 格式
- 确保机器人已正确连接并初始化



---

<!-- source: root -->

# What is GDK for?

# 主页

GDK（Genie Development Kit）是为Genie机器人及其相关产品提供的开发工具包。它的主要作用是为用户和开发者提供丰富的二次开发接口，方便用户根据自身需求快速实现机器人在移动、感知、操作等方面的定制化功能。

通过GDK，用户可以：

- 集成调用机器人的底层控制、传感器数据获取、任务调度等功能接口；
- 可基于 C++、Python 等主流开发语言开发，或直接使用 ROS2，满足不同开发者的技术栈需求；
- 实现机器人与外部系统的高效对接，比如与VR、仿真、云端等模块的集成；
- 便捷地进行算法验证、功能扩展和系统集成，加速产品落地和迭代。

本文档包含以下主要章节：

- **快速开始**：环境搭建与最小可运行示例，支持 C++、Python、ROS2 三种开发方式；
- **基本概念**：文档中一些难懂的关键术语的解释，或者一些英文缩写的具体描述；
- **如何使用**：GDK 部署、时间同步、模式切换，以及 C++/Python/ROS2 的开发流程；
- **工具**：关节控制工具与 Web 相机查看器的使用说明；
- **接口文档**：Python、C++、ROS2 三套 SDK 的完整 API 参考；
- **附录**：关节错误码查询表等参考资料。

## 架构概览

GDK 采用[混合部署](contents/concepts/hybrid_deploy/)模式：用户代码运行在开发机（[用户侧](contents/concepts/hybrid_deploy/#用户侧)），通过有线网络调用机器人本体上的 GDK 服务（[机器人端](contents/concepts/hybrid_deploy/#机器人端)）。

```
flowchart LR
    subgraph 用户侧 Client Side
        A[用户应用代码\nC++ / Python / ROS2]
        B[GDK SDK]
        A --> B
    end
    subgraph 机器人端 Robot Side
        C[GDK 服务]
        D[硬件驱动\n运动控制 / 传感器]
        C --> D
    end
    B -- 有线网络 --> C
```

详见[混合部署](contents/concepts/hybrid_deploy/)。

## 示例场景

以下是GDK可能用到的一些典型场景：

- **教育与科研**：高校、研究机构可利用GDK快速搭建机器人实验平台，进行算法验证、课程教学、学术研究等。
- **工业自动化**：在工厂、仓储等场景下，开发者可基于GDK实现机器人自动搬运、巡检、分拣等功能。
- **服务机器人**：如酒店、医院、商场等公共场所的服务机器人，通过GDK实现定制化的人机交互、导航、物品递送等。
- **智能家居**：家庭服务机器人可通过GDK集成语音、视觉等多模态感知，实现智能清扫、安防巡逻等。
- **多机器人协作**：GDK支持多机器人系统的开发，便于实现机器人之间的协同作业、信息共享与任务分配。
- **感知与导航算法开发**：开发者可基于GDK快速接入传感器数据，进行SLAM、目标检测、路径规划等算法的开发与测试。
- **远程运维与监控**：通过GDK实现机器人远程控制、状态监测、故障诊断和远程升级。
- **二次开发与系统集成**：企业或个人可基于GDK进行功能扩展，将机器人与云平台、物联网、第三方系统无缝集成。

这些场景仅为部分示例，GDK的灵活性和开放性使其能够适应更多创新应用场景，助力机器人行业的快速发展。

---

**文档版本:** 4.1.5  |  **构建时间:** 2026-08-20 03:33:53
