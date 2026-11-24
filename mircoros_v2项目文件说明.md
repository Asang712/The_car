# mircoros_v2 项目文件说明文档

> 分析对象：`/home/asang/mircoros_v2-1.0.1/mircoros_v2`
> 运行环境：Ubuntu 24.04 + ROS 2 Jazzy + Python 3.12
> 说明范围：`src/` 下 16 个 ROS 2 包、`.system_cfg/` 配置与图形工具，以及 `build/`、`install/`、`log/` 三个构建产物目录

---

## 一、项目总览

### 1.1 项目定位

这是一套 **micro-ROS 智能小车**的完整 ROS 2 软件栈。上位机（Ubuntu 电脑）通过 micro-ROS Agent 用 UDP 与 ESP32 控制板通信，向上提供 ROS 2 话题；在此之上叠加了 SLAM / 导航 / 视觉识别跟随 / 大模型语音控制四层能力。

**两种运行模式**：

- **建图导航模式**（厂家默认）：SLAM 建图 → nav2 全局规划 → roadnet 路网导航，需要先建图再导航。
- **不建图视觉伺服模式**（推荐）：用相机画面中的目标框位置/大小直接闭环控制 `/cmd_vel`，无需地图、无需全局定位。适合室内找东西、跟人、AprilTag 视觉路标等场景。`bringup_app.launch.py` 加参数 `slam_mode:=none` 即可切到此模式。

核心数据链路：

```
ESP32 控制板（电机、IMU、雷达、云台）
        ↕  micro-ROS 通信（UDP 8090）
micro-ROS Agent（电脑进程）
        ↕  ROS 2 话题（Domain ID 必须两端一致）
┌───────────────────────────────────────────┐
│  感知层：/scan /imu /odom_raw /camera/image_raw │
│  决策层：SLAM / nav2 / roadnet（建图路线）          │
│         视觉伺服 / YOLO / AprilTag（不建图路线）    │
│         multi_brains LLM Agent（语音/文字理解）     │
│  执行层：/cmd_vel /cmd_ptz_angle /beep          │
└───────────────────────────────────────────┘
```

### 1.2 顶层目录

| 目录 | 作用 |
|---|---|
| `src/` | 全部 ROS 2 功能包源码，colcon 的编译输入 |
| `build/` | colcon 中间编译产物（CMake 缓存、.o、autogen），可安全删除 |
| `install/` | colcon 安装产物（可执行文件、库、share 资源），运行时 `source install/setup.bash` 的来源 |
| `log/` | 每次 `colcon build` 的详细日志，`log/latest_build/events.log` 是排查编译失败的第一入口 |
| `.system_cfg/` | 厂家图形化配置工具 RobotManager、桌面快捷方式、机器人配置文件（不在 src 内，不参与编译） |

### 1.3 包清单

| 包名 | 语言 | 角色 | 是否必装 |
|---|---|---|---|
| `interfaces` | IDL | 自定义 msg/srv/action 契约，全项目共享 | 是（被多数包依赖） |
| `robot_description` | Xacro/URDF | 机器人模型、TF 树、RViz/Gazebo 显示 | 是 |
| `microros_v2_bringup` | Python | 总启动编排、相机驱动、底盘驱动、SLAM/导航参数与 RViz 配置 | 是（核心） |
| `ros_robot_app` | Python | 应用层：图像压缩转发、位姿发布、地图保存 | 是（App 联动） |
| `multi_brains` | Python | 大模型（Dify）智能体，自然语言 → ROS 动作 | 选装 |
| `multi_brains_pre` | Python | 大模型前置层：MCP 服务、外部动作封装 | 选装 |
| `roadnet` | C++/Qt | nav2 路网（Route Graph）规划服务与 RViz 编辑插件 | 选装（导航增强） |
| `microros_v2_no_ptz_visual_control_pkg` | Python | 固定摄像头车型的视觉识别 + 跟随/控制节点 | 按车型选装 |
| `microros_v2_ptz_visual_control_pkg` | Python | 带云台车型的视觉识别 + 跟随/控制节点 | 按车型选装 |
| `microros_v2_mediapipe_pkg` | Python | MediaPipe 手势/姿态/人脸识别 ROS 化示例 | 选装 |
| `microros_v2_opencv_visual_pkg` | Python | OpenCV 经典视觉识别（颜色/人脸/二维码/ArUco/物体） | 选装 |
| `microros_v2_opencv_base_pkg` | Python | OpenCV 基础教程脚本（无 ROS 依赖的练习代码） | 选装 |
| `microros_v2_yolo_pkg` | Python | YOLO 检测/分割/姿态/OBB/分类通用节点集 | 选装 |
| `yolo_onnx_follow_pkg` | Python | YOLOv11 ONNX 检测 + 云台/底盘跟随（纯 OpenCV DNN，无需 ultralytics） | 选装 |
| `microros_v2_kcf_follow_pkg` | Python | 消费 `/tracking_bbox` 的通用跟随控制器（跟踪结果由外部提供） | 选装 |
| `common_demo` | Python | 教学示例集：标定、雷达、麦轮、手柄、多机同步 | 选装 |

> 说明：`microros_v2_no_ptz_visual_control_pkg` 与 `microros_v2_ptz_visual_control_pkg` 是**互斥的车型方案**，由 `CAR_TYPE` 环境变量决定实际启动哪一套。

### 1.4 分层视图

```
┌── 交互层 ──────────────────────────────────────────────┐
│ multi_brains（Dify LLM + terminal_client）              │
│ multi_brains_pre（MCP Server，外部 Agent 调用 ROS 工具）│
├── 应用层 ──────────────────────────────────────────────┤
│ ros_robot_app（图像压缩、位姿、存图）                    │
│ microros_v2_*_visual_control_pkg（视觉跟随）             │
│ microros_v2_yolo_pkg / yolo_onnx_follow_pkg（检测）      │
├── 决策层 ──────────────────────────────────────────────┤
│ nav2（planner/controller/bt_navigator）                 │
│ roadnet（Route Graph 路网规划，route_server）            │
│ SLAM：slam_toolbox / cartographer / gmapping            │
│ 定位：robot_localization EKF + AMCL                      │
├── 驱动层 ──────────────────────────────────────────────┤
│ microros_v2_bringup：camera_driver / base_driver        │
│ micro-ROS Agent（外部，工作空间外）                      │
├── 描述层 ──────────────────────────────────────────────┤
│ robot_description（URDF + TF + mesh）                   │
│ interfaces（msg / srv / action）                        │
└────────────────────────────────────────────────────────┘
```

---

## 二、interfaces

**职责**：全项目唯一的自定义接口定义包，不包含可执行节点，只生成 C++/Python 消息代码。

| 文件 | 作用 |
|---|---|
| `CMakeLists.txt` | 用 `rosidl_generate_interfaces` 批量生成 msg/srv/action，声明对 `geometry_msgs`、`builtin_interfaces` 的依赖 |
| `package.xml` | 声明为 IDL-only 包，导出 `rosidl_default_runtime` |
| `msg/LlmTextRequest.msg` | LLM 文字请求：`text` 用户输入、`is_feedback` 标记是否为动作执行反馈 |
| `msg/LlmTextResponse.msg` | LLM 响应：`response_text` 回复文本、`action_list_json` 动作列表 JSON、`conversation_id` Dify 会话 ID、`msg_type` 区分 progress/result |
| `msg/TrackingBbox.msg` | 目标跟踪框：时间戳 + `bbox[4]`（x1,y1,x2,y2）+ 置信度，是 KCF/跟随包的输入契约 |
| `srv/GetPTZAngle.srv` | 查询云台当前水平/俯仰角度 |
| `srv/GetRobotPose.srv` | 查询机器人当前 `geometry_msgs/Pose` |
| `srv/SetTracking.srv` | 开启/关闭目标跟随（`enable` 布尔） |
| `srv/WebSaveMap.srv` | Web/App 端按名称保存地图 |
| `action/MoveByOdom.action` | 按里程计移动：距离 + 方向（前/后/左转/右转）+ 角度，含已移动距离/已旋转角度反馈 |
| `action/PreciseAdjustPose.action` | 精确位姿调整：目标 Pose 进，位置误差/角度误差反馈出 |

---

## 三、robot_description

**职责**：提供机器人 URDF/Xacro 模型、TF 树定义与显示/仿真启动入口。所有需要 `base_link → camera_link → laser_link` 变换的节点都依赖它。

| 文件 | 作用 |
|---|---|
| `CMakeLists.txt` | 安装 `urdf/`、`meshes/`、`launch/`、`rviz/`、`config/` 到 share 目录 |
| `package.xml` | 声明依赖 `xacro`、`robot_state_publisher`、`joint_state_publisher`、`rviz2`、`gazebo_ros` |
| `urdf/microros_v2.urdf.xacro` | 主模型：底盘 `base_link`、左右驱动轮、万向轮、雷达、相机安装位；按 `camera_type` 参数切换不同相机 mesh |
| `urdf/camera.urdf.xacro` | 相机连杆与光学坐标系 `camera_link` / `camera_optical_frame` 定义，被主 xacro include |
| `config/joint_names.yaml` | 关节名清单，供 `joint_state_publisher` 在无真实关节状态时生成占位 TF |
| `launch/display.launch.py` | 无仿真快速查看模型：`robot_state_publisher` + `joint_state_publisher_gui` + `rviz2` |
| `launch/gazebo.launch.py` | 在 Gazebo 中加载机器人模型，用于纯仿真调试算法 |
| `rviz/microros_v2.rviz` | 该车型专用 RViz 布局（RobotModel、TF、LaserScan、Camera 显示项） |
| `meshes/` | 12 个 STL 模型：`base_link.STL` 与 `base_link_noptz.STL`（带/不带云台两种底盘）、`bwheel_Link.STL` 驱动轮、`cam01/cam02/cam1/cam2/cam3_Link.STL` 不同相机支架、`IMU_Link.STL`、`laser_Link.STL` 等，由 xacro 以 `package://` 引用 |

---

## 四、microros_v2_bringup（核心启动包）

**职责**：整个系统的"总控台"。负责相机采集、底盘驱动、SLAM/导航/多机的 launch 编排、全部参数 yaml 与 RViz 配置、以及串口底层配置工具。

### 4.1 Python 节点

| 文件 | 作用 |
|---|---|
| `microros_v2_bringup/camera_driver.py` | IP 相机驱动：用 OpenCV 打开 `http://<ip>:81/stream`，发布 `/camera/image_raw` 与 `/camera/camera_info`；支持加载 `ost.yaml` 标定参数、按需落盘截图 |
| `microros_v2_bringup/base_driver.py` | 底盘动作服务节点：提供 `MoveByOdom`、`PreciseAdjustPose` 两个 action server 和 `GetRobotPose`、`GetPTZAngle` 两个 service；内部用 TF 监听与里程计计算位移/角度误差，输出 `Twist`、`Pose`、`Vector3` 指令 |
| `microros_v2_bringup/visual_track.py` | 简单 PID 跟随节点：订阅 `smart_tracker` 的 `TrackBox`，通过 `SetTracking` 服务开关，把目标中心偏差转成 `Twist` / 云台 `Vector3` 指令 |
| `microros_v2_bringup/topic_test.py` | 连通性自检：检查关键话题是否有发布者/数据，用于首连排障 |
| `microros_v2_bringup/calibrate_linear.py` 系列 | `calibrate_linear_save_yaml.py`、`calibrate_angular_save_yaml.py`：跑标定流程并把结果写回 yaml |
| `microros_v2_bringup/calibration_yaml_utils.py` | 标定 yaml 的读写工具函数，供上面两个标定脚本复用 |
| `microros_v2_bringup/calib_verify.py` | 标定结果验证：按标定参数走固定距离，回读实际位移判断误差 |
| `microros_v2_bringup/robot_tools/robot_tool.py` | 全局工具模块：从环境变量读 `CAR_TYPE`、`ROS_DOMAIN_ID`，从 `robot_info.json` 读相机 URL / Agent IP，供各 launch 文件调用 `GetRobotType()`、`GetCameraUrl()` |
| `microros_v2_bringup/__init__.py` | 包标识文件 |

### 4.2 启动文件（launch）

| 文件 | 作用 |
|---|---|
| `launch/bringup_app.launch.py` | **主入口**：启动 camera_driver → rosbridge → `ros_robot_app` 的 transmission/robot_pose_publisher/app_save_map，并按 `slam_mode` 与 `navigation_mode` 条件包含 SLAM 与 nav2 子 launch |
| `launch/bringup_mod.launch.py` | 模块化启动变体，按功能模块分别拉起，便于单独调试 |
| `launch/car_base.launch.py` | 只启动底盘相关（TF、里程计、EKF），不含感知 |
| `launch/controller.launch.py` | 手柄/键盘等遥控输入节点的启动 |
| `launch/state_estimation.launch.py` | 启动 `robot_localization` EKF，融合轮式里程计与 IMU |
| `launch/slam_toolbox.launch.py` | slam_toolbox 在线建图（`mapper_params_online_async.yaml`） |
| `launch/cartographer.launch.py` | Cartographer 建图（`cartographer_2d.lua`） |
| `launch/gmapping.launch.py` | GMapping 建图（`slam_gmapping.yaml`） |
| `launch/save_map.launch.py` | 调用 `map_saver` 保存地图，并与 `ros_robot_app` 的存图服务联动 |
| `launch/nav2.launch.py` | 标准 nav2 导航栈（planner/controller/bt_navigator/behavior 全套） |
| `launch/roadnet_nav.launch.py` | **roadnet 版导航栈**：在 nav2 基础上追加 `route_server` 生命周期节点，加载 `roadnet_nav.yaml` 与 geojson 路网文件 |
| `launch/toolbox_location_nav.launch.py` | slam_toolbox 定位模式 + nav2 导航组合启动 |
| `launch/multi_robot_base.launch.py` | 多机器人底盘（按 namespace 隔离 TF 与话题） |
| `launch/multi_robot_demo.launch.py` | 多机器人协同演示 |
| `launch/display_model.launch.py` | 只做模型显示，用于检查 URDF |
| `launch/lidar_demo.launch.py` | 雷达数据可视化演示 |
| `launch/colorSelect.launch.py` | 启动取色工具（HSV 阈值交互选取） |
| `launch/super_tracker.launch.py` | 综合跟踪启动，串起检测 + 跟随两段 |
| `launch/openclaw_service.launch.py` | 启动 openclaw 相关外部服务（本地 Agent 服务对接） |
| `launch/test_connect.launch.py` | 首次连接的连通性测试组合（相机 + 话题自检） |

### 4.3 参数配置（config）

| 文件 | 作用 |
|---|---|
| `config/common.yaml` | 全项目公共参数（frame 名、话题名等） |
| `config/ekf.yaml` | `robot_localization` EKF 配置：融合 `/odom_raw` 与 `/imu` 输出 `odom→base_link` |
| `config/nav2_params.yaml` | nav2 全套参数：costmap、planner、controller、behavior、bt 行为树路径 |
| `config/nav2_multi_robot_params.yaml` | 多机器人版的 nav2 参数（带 namespace 前缀） |
| `config/roadnet_nav.yaml` | roadnet 导航参数：route_server、edge scorer 列表、operation 列表、图文件路径 |
| `config/mapper_params_online_async.yaml` | slam_toolbox 在线异步建图参数 |
| `config/mapper_params_localization.yaml` | slam_toolbox 纯定位模式参数（先验地图 + `pose_map`） |
| `config/map_mapping.yaml` | 建图相关附加参数（地图分辨率、保存路径） |
| `config/slam_gmapping.yaml` | GMapping 建图参数 |
| `config/cartographer_2d.lua` | Cartographer 2D 建图 Lua 配置 |
| `config/cartographer_2d_localization.lua` | Cartographer 定位模式配置（加载 pbstream） |
| `config/odom_linear_calibration.yaml` | 线速度标定结果（每米脉冲数/比例因子） |
| `config/odom_angular_calibration.yaml` | 角速度标定结果 |
| `config/odom_calibration_common.yaml` | 线/角标定共用的公共参数 |
| `config/ost.yaml` | 相机内参与畸变标定文件（camera_info 来源） |
| `config/fastdds_shm.xml` | FastDDS 共享内存传输配置，降低大图像话题的拷贝开销；launch 中以 `FASTRTPS_DEFAULT_PROFILES_FILE` 注入 |
| `config/multi_brains_pre_setting.yaml` | `multi_brains_pre` 的运行时参数（服务端口、动作白名单等） |

### 4.4 RViz 配置（rviz）

| 文件 | 作用 |
|---|---|
| `rviz/rviz_config.rviz` | 默认综合视图 |
| `rviz/display.rviz` | 纯模型检查视图 |
| `rviz/gmapping_rviz.rviz` | GMapping 建图视图 |
| `rviz/slam_toolbox_rviz.rviz` | slam_toolbox 建图视图 |
| `rviz/slam_cartographer_rviz.rviz` | Cartographer 建图视图 |
| `rviz/state_estimation.rviz` | EKF 状态估计调试图（里程计 vs 融合轨迹） |
| `rviz/nav2.rviz` | nav2 导航视图（costmap、全局/局部路径） |
| `rviz/nav2_multi_robot.rviz` | 多机器人 nav2 视图，按 namespace 分组显示 |
| `rviz/roadnet.rviz` | roadnet 路网导航视图（显示 geojson 图与 route） |
| `rviz/roadnet_edit.rviz` | roadnet 路网编辑视图，配合 Route Tool 插件画图 |
| `rviz/microros_v2_3d.rviz` | 3D 展示视图 |

### 4.5 串口配置工具

| 文件 | 作用 |
|---|---|
| `cli/config_robot` | 命令行串口配置工具：通过串口协议向 ESP32 写入 WiFi SSID/密码、Agent IP、相机 URL、`ROS_DOMAIN_ID`、舵机偏移、电机 PID、IMU PID、namespace 等底层参数 |
| `setup.py` | 声明 5 个 `console_scripts`：`camera_driver`、`calib_verify`、`base_driver`、`visual_track`、`topic_test`，并安装 launch/config/rviz 资源 |
| `setup.cfg` | 脚本安装路径配置 |
| `package.xml` | 依赖声明：`interfaces`、`cv_bridge`、`tf2_ros`、`robot_localization`、`smart_tracker` 等 |
| `resource/` | ament 资源索引标记文件 |
| `test/test_copyright.py` 等 | ament lint 三件套（版权头、flake8、pep257），仅为合规检查 |

---

## 五、ros_robot_app

**职责**：面向手机 App / Web 的应用层节点，负责把机器人端的大数据量话题压缩后转发出去。

| 文件 | 作用 |
|---|---|
| `ros_robot_app/transmission.py` | **核心转发节点**：订阅 `/scan` 转成 `/scan_points`；订阅 `/camera/image_raw`（BEST_EFFORT）压缩后发布 `/camera/rgb/image_raw/compressed`（BEST_EFFORT），降低 App 端带宽压力 |
| `ros_robot_app/robot_pose_publisher.py` | 订阅 TF，对外发布 `PoseStamped` 形式的机器人位姿（供 App 定位显示） |
| `ros_robot_app/odom_pose_bridge.py` | **不建图模式专用**：发布 `map→odom` 静态 TF、`odom→base_footprint` 动态 TF（来自 `/odom_raw`），并发布 `/robot_pose`。无 ros2 run 入口，需直接跑 `python3 odom_pose_bridge.py` 或由 `start_nomap.sh` 启动 |
| `ros_robot_app/app_save_map.py` | 提供 `WebSaveMap` 服务：App 端传地图名，服务端落盘地图文件 |
| `ros_robot_app/__init__.py` | 包标识 |
| `setup.py` | 声明 `transmission`、`robot_pose_publisher`、`app_save_map` 三个入口（`odom_pose_bridge` 未注册） |
| `setup.cfg` | 脚本安装路径 |
| `package.xml` | 依赖 `sensor_msgs`、`nav_msgs`、`interfaces`、`cv_bridge` 等 |
| `data/xgo.db` | 本地 SQLite 库，存放地图/任务等持久化数据 |
| `test/test_copyright.py` 等 | ament lint 三件套 |

---

## 六、multi_brains（大模型智能体）

**职责**：把用户自然语言（文字/语音转写）交给 Dify 大模型，解析出结构化动作，再翻译成 ROS 2 控制指令执行，并把执行结果反馈给模型形成闭环。

| 文件 | 作用 |
|---|---|
| `multi_brains/llm_agent_node.py` | **核心节点**：接收 `LlmTextRequest`，调用 `DifyCoordinator`，解析动作批次，调度各 controller 执行，发布 `LlmTextResponse` 进度/结果 |
| `multi_brains/clients/terminal_client.py` | 终端交互客户端：输入"前进""去A点""沿着红线走"等指令并显示执行计划 |
| `multi_brains/clients/image_preview.py` | 图像预览辅助：把相机画面落盘或弹窗，供调试与反馈模型 |
| `multi_brains/clients/__init__.py` | 客户端子包标识 |
| `multi_brains/services/dify_coordinator.py` | Dify 对接层：调用 Agent SSE 聊天接口传文本/图像/conversation_id，解析返回 JSON 里的 `action` 与 `response` |
| `multi_brains/services/__init__.py` | 子包标识 |
| `multi_brains/controllers/motion_controller.py` | 运动控制：把动作翻译成 `/cmd_vel` |
| `multi_brains/controllers/navigation_controller.py` | 导航控制：调用 nav2 `NavigateToPose` / roadnet action 目标导航 |
| `multi_brains/controllers/ptz_controller.py` | 云台控制：发布 `/cmd_ptz_angle` 并按 `/ptz_angle` 做闭环 |
| `multi_brains/controllers/kcf_tracker.py` | 调用 KCF 跟踪相关服务/话题，实现目标锁定跟随 |
| `multi_brains/controllers/follow_line_controller.py` | 巡线控制：调用 follow_line 动作节点 |
| `multi_brains/controllers/__init__.py` | 子包标识 |
| `multi_brains/core/action_registry.py` | 动作注册表：动作名 → 中文描述 → 执行器映射，是"模型输出→函数调用"的分发表 |
| `multi_brains/core/command_state.py` | 指令状态机：跟踪一个指令从下发到完成/超时/失败的状态 |
| `multi_brains/core/feedback_protocol.py` | 反馈协议：定义回传模型的文案模板与字段格式 |
| `multi_brains/core/hardware_profile.py` | 硬件画像：按 `CAR_TYPE` 决定可用动作集合（有/无云台能力不同） |
| `multi_brains/core/__init__.py` | 子包标识 |
| `multi_brains/utils/dify_client.py` | 轻量 HTTP 客户端封装（请求头、超时、SSE 流读取） |
| `multi_brains/utils/common_tools.py` | 通用工具函数（角度归一化、JSON 安全解析等） |
| `multi_brains/utils/colorSelect.py` | 取色工具，供巡线/颜色跟随前先确定 HSV 阈值 |
| `multi_brains/utils/__init__.py` | 子包标识 |
| `multi_brains/__init__.py` | 包标识 |
| `multi_brains/config/llm_agent_config.yaml` | Dify 接入配置：`DIFY_BASE_URL`、`DIFY_API_KEY`、调试模式、反馈轮次与文案模板 |
| `multi_brains/config/map_mapping.yaml` | 地图/点位名称映射（"A点"→具体坐标） |
| `multi_brains/config/color_hsv.txt` | 预设 HSV 颜色阈值 |
| `multi_brains/language/zh.yaml` | 中文话术：动作描述、反馈模板 |
| `multi_brains/language/en.yaml` | 英文话术 |
| `multi_brains/launch/llm_agent_control.launch.py` | 启动 `llm_agent_node`，加载 `llm_agent_config.yaml` |
| `multi_brains/launch/terminal_client.launch.py` | 启动终端客户端 |
| `setup.py` | 声明 `llm_agent_node`、`terminal_client`、`color_selector` 三个入口 |
| `setup.cfg` / `package.xml` | 安装路径与依赖（`rclpy`、`nav2_msgs`、`tf2_ros`、`cv_bridge`、`interfaces`） |
| `test/test_copyright.py` 等 | ament lint 三件套 |

---

## 七、multi_brains_pre（MCP 前置服务）

**职责**：把机器人能力包装成 **MCP（Model Context Protocol）工具**，让外部 Agent（如 Trae / Claude）可以直接"看到画面""调云台""执行动作"。是 `multi_brains` 的另一种接入形态：不走 Dify，而走 MCP 标准协议。

| 文件 | 作用 |
|---|---|
| `multi_brains_pre/mcp_service.py` | **MCP 服务端**：基于 `FastMCP`，在 `0.0.0.0:8000` 以 `streamable-http` 暴露工具；初始化 ROS 上下文并注册全部工具 |
| `multi_brains_pre/mcp_tools/mcp_action.py` | 工具定义集合：`SeeWhat`（拍照返回路径）、`AdjustCameraView`（按 pitch/yaw 转云台）等动作的 MCP 包装 |
| `multi_brains_pre/mcp_tools/__init__.py` | 子包标识 |
| `multi_brains_pre/action_controller.py` | ROS 节点上下文管理：创建节点、启动 spin 线程、统一提供发布/服务调用入口 |
| `multi_brains_pre/message/mcp_message.py` | MCP 请求/响应的数据结构与序列化 |
| `multi_brains_pre/message/__init__.py` | 子包标识 |
| `multi_brains_pre/external_action/follow_line.py` | 巡线动作实现（可被 MCP 工具直接调用） |
| `multi_brains_pre/external_action/colorSelect.py` | 颜色阈值选取动作 |
| `multi_brains_pre/external_action/follow_common.py` | 跟随类动作的公共逻辑（目标居中、距离保持） |
| `multi_brains_pre/external_action/__init__.py` | 子包标识 |
| `multi_brains_pre/utils/dify_client.py` | Dify 客户端（与 multi_brains 版本共享思路） |
| `multi_brains_pre/utils/robot_vision.py` | 视觉辅助：抓图、编码、按目标裁剪 |
| `multi_brains_pre/utils/common_tools.py` | 通用工具函数 |
| `multi_brains_pre/utils/__init__.py` | 子包标识 |
| `multi_brains_pre/__init__.py` | 包标识 |
| `cli/robot_control.py` | 命令行客户端：`call-tool SeeWhat`、`call-tool AdjustCameraView --pitch/--yaw` 直接调 MCP 工具 |
| `cli/SKILL.md` | MCP 工具的说明文档（工具名、参数、示例命令），供 Agent 侧理解如何调用 |
| `setup.py` | 声明 `mcp_service`、`follow_line`、`colorSelect` 三个入口 |
| `setup.cfg` / `package.xml` | 安装路径与依赖（额外需要 `fastmcp`） |
| `test/test_copyright.py` 等 | ament lint 三件套 |

---

## 八、roadnet（C++ 路网规划包）

**职责**：nav2 生态中的 **Route Graph（路网）规划器**。与传统自由空间 costmap 规划互补：它在一张由节点（Node）和有向边（Edge）组成的拓扑图上找最短路，并可对边打分、在途中触发操作（限速、碰撞监控、重规划）。

### 8.1 构建与插件注册

| 文件 | 作用 |
|---|---|
| `CMakeLists.txt` | 定义 6 个 target：`route_server_core`（核心库）、`route_operations`（操作插件库）、`edge_scorers`（边评分插件库）、`graph_file_loaders` / `graph_file_savers`（图读写插件库）、`route_tool` 与 `roadnet_nav_tool`（RViz 插件）、`route_server`（可执行文件）；调用 `nav2_package()` 会自动开启 `-Wall -Wextra -Werror` |
| `package.xml` | 描述为 "Route Graph planner to compliment the Planner Server"，导出 `plugins.xml`、`route_tool_rviz.xml`、`roadnet_nav_tool_rviz.xml` |
| `plugins.xml` | **插件注册表**：注册 8 个 edge scorer、5 个 route operation、geojson 图读写器 |
| `route_tool_rviz.xml` | RViz 中"路线绘制工具"的插件声明 |
| `roadnet_nav_tool_rviz.xml` | RViz 中"路网导航面板"的插件声明 |
| `README.md` | 上游 roadnet 项目的使用说明（图格式、插件配置示例） |
| `LICENSE` | 开源许可证 |
| `icons/classes/` | RViz 工具面板图标资源 |

### 8.2 核心源文件（src）

| 文件 | 作用 |
|---|---|
| `src/main.cpp` | `route_server` 可执行入口：初始化 rclcpp、构造 `RouteServer` 生命周期节点并 spin |
| `src/route_server.cpp` | 服务主体：创建 `compute_route`、`compute_and_track_route`、`compute_route_and_follow` 三个 action server；组装 GraphLoader、GoalIntentExtractor、RoutePlanner、RouteTracker、OperationsManager、PathConverter |
| `src/route_planner.cpp` | 路径搜索核心：`findRoute()` 校验图 → 定位起终点 → 跑 `findShortestGraphTraversal()` → 转成 `Route`，失败时输出诊断原因 |
| `src/route_tracker.cpp` | 路线跟踪：机器人沿 Route 行进时判定"到达哪个节点/进入哪条边"，触发对应 operation，并在需要时返回重规划请求 |
| `src/graph_loader.cpp` | 图加载实现：解析 geojson → 构建 Node/Edge 结构 → 建立空间索引 |
| `src/graph_saver.cpp` | 图保存实现：把内存中的图写回 geojson |
| `src/edge_scorer.cpp` | 边评分管理器：按参数加载插件、组合各 scorer 的权重得到最终边代价 |
| `src/node_spatial_tree.cpp` | 基于 nanoflann 的节点 KD 树：支持"找最近节点"的快速查询 |
| `src/operations_manager.cpp` | 操作管理器：按边的 metadata 选择并执行 route operation 插件 |
| `src/path_converter.cpp` | `Route` → `nav_msgs/Path` 转换（把拓扑路径变成控制层能消费的密集路径点） |
| `src/goal_intent_extractor.cpp` | 目标意图提取：从用户给定位姿/TF 推断在路网上的起终点节点 |
| `src/route_tool.cpp` | RViz 路线绘制工具实现：鼠标点选节点画路线并下发 action |
| `src/roadnet_nav_tool.cpp` | RViz 路网导航面板实现：设置起点终点、选择图文件、显示执行状态 |

### 8.3 插件实现（src/plugins）

| 文件 | 作用 |
|---|---|
| `src/plugins/edge_cost_functions/distance_scorer.cpp` | 距离评分：以边长度为代价 |
| `src/plugins/edge_cost_functions/time_scorer.cpp` | 时间评分：以预计通行为代价 |
| `src/plugins/edge_cost_functions/penalty_scorer.cpp` | 罚分评分：按边上的 penalty 字段加代价 |
| `src/plugins/edge_cost_functions/costmap_scorer.cpp` | 代价地图评分：边经过不可通行区域时加代价 |
| `src/plugins/edge_cost_functions/dynamic_edges_scorer.cpp` | 动态边评分：运行时禁用的边给高代价 |
| `src/plugins/edge_cost_functions/semantic_scorer.cpp` | 语义评分：按边语义标签（如地面材质、坡度）加权 |
| `src/plugins/edge_cost_functions/goal_orientation_scorer.cpp` | 目标朝向评分：终点朝向与机器人朝向的夹角代价 |
| `src/plugins/edge_cost_functions/start_pose_orientation_scorer.cpp` | 起点朝向评分：起步朝向偏差代价 |
| `src/plugins/route_operations/adjust_speed_limit.cpp` | 限速操作：经过该边时把速度上限压到 metadata 指定值 |
| `src/plugins/route_operations/collision_monitor.cpp` | 碰撞监控操作：在该边上开启/关闭碰撞监控 |
| `src/plugins/route_operations/rerouting_service.cpp` | 重规划操作：调用外部服务触发路线重算 |
| `src/plugins/route_operations/time_marker.cpp` | 时间标记操作：记录经过时间用于性能统计 |
| `src/plugins/route_operations/trigger_event.cpp` | 通用事件触发：在节点/边上调用任意指定服务（Trigger 类型） |
| `src/plugins/graph_file_loaders/geojson_graph_file_loader.cpp` | geojson 图加载插件实现 |
| `src/plugins/graph_file_savers/geojson_graph_file_saver.cpp` | geojson 图保存插件实现 |

### 8.4 头文件（include）

| 文件 | 作用 |
|---|---|
| `include/roadnet/types.hpp` | 基础类型别名：`NodePtr`、`EdgePtr`、`Route`、`Metadata`、`OperationResult` 等 |
| `include/roadnet/utils.hpp` | 通用工具：坐标变换、角度归一化、字符串解析 |
| `include/roadnet/route_exceptions.hpp` | 专用异常类型（`OperationFailed`、`NoValidGraph` 等），对应 action 的 error_code |
| `include/roadnet/graph_loader.hpp` | 图加载器声明与插件加载逻辑 |
| `include/roadnet/graph_saver.hpp` | 图保存器声明 |
| `include/roadnet/route_server.hpp` | `RouteServer` 生命周期节点声明 |
| `include/roadnet/route_planner.hpp` | 规划器接口：`findRoute`、`findShortestGraphTraversal` |
| `include/roadnet/route_tracker.hpp` | 跟踪器接口：进度判定与操作触发 |
| `include/roadnet/edge_scorer.hpp` | 边评分器：加载插件、累加代价 |
| `include/roadnet/node_spatial_tree.hpp` | 节点 KD 树（nanoflann 封装） |
| `include/roadnet/operations_manager.hpp` | 操作管理器：插件加载与执行调度 |
| `include/roadnet/path_converter.hpp` | 路径转换器声明 |
| `include/roadnet/goal_intent_extractor.hpp` | 目标意图提取器声明 |
| `include/roadnet/goal_intent_search.hpp` | 目标意图搜索的辅助算法（按方向/车道筛选候选节点） |
| `include/roadnet/corner_smoothing.hpp` | 拐角平滑：把折线路径在节点处做圆滑处理 |
| `include/roadnet/route_tool.hpp` | RViz Route Tool 声明 |
| `include/roadnet/roadnet_nav_tool.hpp` | RViz 路网导航面板声明 |
| `include/roadnet/interfaces/edge_cost_function.hpp` | 边代价函数插件**抽象接口**（供第三方实现） |
| `include/roadnet/interfaces/graph_file_loader.hpp` | 图加载插件抽象接口（支持 geojson 之外的格式） |
| `include/roadnet/interfaces/graph_file_saver.hpp` | 图保存插件抽象接口 |
| `include/roadnet/interfaces/route_operation.hpp` | 路线操作插件抽象接口（`configure` / `perform`） |
| `include/roadnet/plugins/route_operation_client.hpp` | 通用"调用任意服务型操作"的模板基类，`configure()` 建 client、`perform()` 调服务 |
| `include/roadnet/plugins/edge_cost_functions/*.hpp` | 上述 8 个 scorer 的类声明 |
| `include/roadnet/plugins/route_operations/*.hpp` | 上述 5 个 operation 的类声明 |
| `include/roadnet/plugins/graph_file_loaders/geojson_graph_file_loader.hpp` | geojson 加载器声明 |
| `include/roadnet/plugins/graph_file_savers/geojson_graph_file_saver.hpp` | geojson 保存器声明 |
| `include/nav2_util/service_server.hpp` | **本地复制的 nav2 服务包装模板**：把回调式服务封装成类，本项目的 `RouteServer` 用它创建服务；已适配 Jazzy 的 `create_service(name, cb, rclcpp::QoS, group)` 重载 |

### 8.5 接口定义（interfaces）

| 文件 | 作用 |
|---|---|
| `interfaces/action/ComputeRoute.action` | 只算路不下发：起点/终点（ID 或位姿）→ `nav_msgs/Path` + `nav2_msgs/Route` + 规划耗时；含 400~407 错误码 |
| `interfaces/action/ComputeAndTrackRoute.action` | 算路并跟踪执行，反馈当前节点/边、路径、触发的操作、是否重规划 |
| `interfaces/action/RoadNetToGoal.action` | 一体化目标导航：算路 + 跟踪 + 返回执行结果 |

### 8.6 示例图与脚本

| 文件 | 作用 |
|---|---|
| `graphs/sample_graph.geojson` | 最小示例路网，用于单元测试与入门 |
| `graphs/turtlebot3_graph.geojson` | TurtleBot3 场景路网 |
| `graphs/turtlebot4_graph.geojson` | TurtleBot4 场景路网 |
| `graphs/aws_graph.geojson` | AWS 仓储场景大型路网示例 |
| `graphs/scripts/export_shapefiles.py` | 从 shapefile 导出 geojson 路网的转换脚本 |
| `graphs/scripts/increment_node_id.json` | 节点 ID 自增器状态文件 |
| `graphs/scripts/increment_edge_id.json` | 边 ID 自增器状态文件 |
| `graphs/scripts/README.md` | 图制作脚本的使用说明 |

### 8.7 测试（test）

| 文件 | 作用 |
|---|---|
| `test/CMakeLists.txt` | 注册 gtest 用例与 `ament_add_gtest` |
| `test/test_graph_loader.cpp` | 图加载正确性（正常图、缺失 frame、非法 JSON） |
| `test/test_graph_saver.cpp` | 图保存往返一致性 |
| `test/test_route_planner.cpp` | 最短路径结果的正确性 |
| `test/test_route_server.cpp` | 服务层 action 接口端到端 |
| `test/test_route_tracker.cpp` | 跟踪进度判定与操作触发 |
| `test/test_operations.cpp` | 各 route operation 插件行为 |
| `test/test_collision_operation.cpp` | 碰撞监控操作专项 |
| `test/test_edge_scorers.cpp` | 各 edge scorer 代价计算 |
| `test/test_spatial_tree.cpp` | KD 树最近邻查询 |
| `test/test_path_converter.cpp` | Route → Path 转换 |
| `test/test_goal_intent_extractor.cpp` | 起终点意图提取 |
| `test/test_goal_intent_search.cpp` | 意图搜索算法 |
| `test/test_corner_smoothing.cpp` | 拐角平滑几何正确性 |
| `test/test_geojson_graph_file_loader.cpp` | geojson 加载插件 |
| `test/test_geojson_graph_file_saver.cpp` | geojson 保存插件 |
| `test/test_utils_and_types.cpp` | 工具函数与类型基础行为 |
| `test/performance_benchmarking.cpp` | 大规模图上的规划耗时基准 |
| `test/test_graphs/error_codes.geojson` | 触发错误码的构造图 |
| `test/test_graphs/invalid.json` | 非法 JSON 用例 |
| `test/test_graphs/no_frame.json` | 缺少 frame 的非法图用例 |

---

## 九、视觉控制包族（无云台 / 带云台）

这两个包结构高度对称，`_ptz` 版本多一条云台控制链路（发布 `/cmd_ptz_angle`、订阅 `/ptz_angle`），`_no_ptz` 版本则把目标居中误差转为**底盘转向**。二者由 `CAR_TYPE` 决定使用哪一套。

### 9.1 命名规律

```
xxx_tracker_node      → 只跟踪并发布 /tracking_bbox + 调试图像，不控制底盘
xxx_follow_node       → 跟踪 + 控制（底盘转向 / 云台转动）
xxx_control_node      → 按识别结果执行离散动作（如二维码内容映射成动作）
xxx_dify_node         → 供 multi_brains 大模型调用的版本
*_action_group_control_node → 动作组编排（连续动作序列）
```

### 9.2 microros_v2_no_ptz_visual_control_pkg

| 文件 | 作用 |
|---|---|
| `.../color_tracker_node.py` | 颜色跟踪：HSV 阈值分割 → 取最大轮廓 → 发布 `/tracking_bbox` |
| `.../color_follow_node.py` | 颜色跟随：在 color_tracker 基础上按横向偏差控制底盘角速度 |
| `.../color_follow_dify_node.py` | 颜色跟随的 Dify 版（接受大模型下发的目标颜色与参数） |
| `.../face_tracker_node.py` | 人脸跟踪：Haar 级联检测正面人脸 → 发布跟踪框 |
| `.../face_follow_node.py` | 人脸跟随：人脸居中控制底盘 |
| `.../april_tag_tracker_node.py` | AprilTag 检测跟踪：解算标签 ID 与位姿 |
| `.../april_tag_follow_node.py` | AprilTag 跟随：按标签位姿保持距离与朝向 |
| `.../apriltag_action_group_control_node.py` | 依据识别到的标签 ID 执行预设动作组 |
| `.../qr_tracker_node.py` | 二维码检测跟踪：解码内容并发布位置 |
| `.../qr_follow_node.py` | 二维码跟随 |
| `.../qr_control_node.py` | 二维码内容 → 控制指令映射（扫到什么做什么） |
| `.../gesture_tracker_node.py` | 手势识别跟踪（基于 MediaPipe 手部关键点） |
| `.../gesture_follow_node.py` | 手势跟随：手掌位置驱动底盘 |
| `.../gesture_control_node.py` | 手势 → 离散控制指令（握拳停止、比划转向等） |
| `.../gesture_action_group_control_node.py` | 手势动作组编排 |
| `.../pose_tracker_node.py` | 人体姿态识别跟踪 |
| `.../pose_follow_node.py` | 人体跟随：按躯干中心控制底盘 |
| `.../hand_tracker_node.py` | 单手检测跟踪 |
| `.../hand_follow_node.py` | 手部跟随 |
| `.../kcf_tracker_node.py` | OpenCV KCF 相关跟踪：框选目标后逐帧跟踪并发布 `/tracking_bbox` |
| `.../kcf_tracker_dify_node.py` | KCF 跟踪的 Dify 版（由大模型决定何时锁定/释放目标） |
| `.../follow_line_node.py` | 巡线：颜色阈值分割出引导线 → 计算中心偏移 → 控制底盘沿线路前进 |
| `.../colorSelect.py` | 取色工具节点：鼠标点选画面颜色，输出 HSV 阈值 |
| `.../hand_detector.py` | MediaPipe 手部检测封装（供 gesture/pose 节点复用） |
| `.../pose_detector.py` | MediaPipe 姿态检测封装 |
| `.../image_conversion.py` | 图像编解码与 `cv_bridge` 转换工具 |
| `.../__init__.py` | 包标识 |
| `config/haarcascade_frontalface_default.xml` | OpenCV 正面人脸级联检测器模型 |
| `launch/*.launch.py` | 21 个启动文件，与上述节点一一对应（`xxx_tracker` / `xxx_follow` / `xxx_control` / `xxx_dify`） |
| `setup.py` | 声明 23 个 `console_scripts` 入口 |
| `setup.cfg` / `package.xml` | 安装路径与依赖 |

### 9.3 microros_v2_ptz_visual_control_pkg

在 9.2 全部同名能力上加 `_ptz` 后缀，并额外包含：

| 文件 | 作用 |
|---|---|
| `.../object_tracker_ptz_node.py` | 通用物体检测跟踪（SSD/YOLO 通用封装） |
| `.../object_follow_ptz_node.py` | 物体跟随：云台先对准，底盘再前进 |
| `.../kcf_style_follow_control.py` | KCF 风格的跟随控制核心：像素误差 → 云台 PD → 底盘前进 |
| `.../kcf_style_follow_launch.py` | 上述控制核心的启动参数装配辅助 |
| `config/color_tracker_ptz.yaml` | 云台跟踪参数：图像话题、PTZ 话题、死区、P/I/D 增益、平滑系数、最大步进、目标丢失保护、控制频率 |
| `config/color_hsv.txt` | 预设 HSV 阈值表 |
| `config/haarcascade_frontalface_default.xml` | 人脸级联检测器 |
| `refer/code/follow/colorFollow.py` | 厂家参考实现：颜色跟随（非 ROS 版本，供对照） |
| `refer/code/follow/astra_common.py` | 奥比中光 Astra 深度相机公共封装（参考实现） |
| `refer/code/kcf/monoTracker.py` | 参考实现：单目 KCF 跟踪器 |
| `refer/code/kcf/track_common.py` | 参考实现：跟踪公共逻辑 |
| `launch/*.launch.py` | 23 个启动文件，`_ptz` 版本含 `object_follow_ptz` / `object_tracker_ptz` |
| `setup.py` | 声明 25 个 `console_scripts` 入口 |

---

## 十、microros_v2_mediapipe_pkg

**职责**：把 MediaPipe 的各类识别算法做成 ROS 2 节点并配套 launch，同时保留 `common_demo` 中的纯脚本版本。适合手势/姿态驱动的交互演示。

| 文件 | 作用 |
|---|---|
| `.../HandDetector.py` | 手部 21 关键点检测，发布关键点与位置 |
| `.../PoseDetector.py` | 人体 33 关键点姿态检测 |
| `.../Holistic.py` | 全身体（脸+手+姿态）同时检测 |
| `.../FaceMesh.py` | 人脸 468 点网格 |
| `.../FaceDetection.py` | 人脸框检测 |
| `.../FaceLandmarks.py` | 人脸关键点 |
| `.../Objectron.py` | 3D 物体检测（盒子/鞋子/杯子/椅子） |
| `.../VirtualPaint.py` | 虚拟画笔：食指轨迹在画面上绘制 |
| `.../FingerCtrl.py` | 手指数量/手势 → 控制指令 |
| `.../FindHand.py` | 定位手部中心坐标 |
| `.../FingerTrajectory.py` | 手指运动轨迹追踪 |
| `.../gestureFollow.py` | 手势驱动跟随（手掌居中 → 底盘转向） |
| `.../poseFollow.py` | 人体驱动跟随 |
| `.../HandCtrl.py` | 手势离散指令控制 |
| `.../mediapipe_gesture.py` | 手势语义识别（点赞、OK、停止等）并输出动作 |
| `.../RobotCtrl.py` | 综合机器人控制节点：把手势结果一次性映射为运动指令 |
| `.../media_common.py` | 相机取流、图像绘制、坐标转换等公共函数 |
| `.../media_library.py` | MediaPipe 模型/工具库封装（检测器构建、平滑滤波） |
| `.../__init__.py` | 包标识 |
| `launch/*.launch.py` | 16 个启动文件，每个对应一个识别节点 |
| `setup.py` | 声明 16 个 `console_scripts` 入口 |
| `setup.cfg` / `package.xml` | 安装路径与依赖（`mediapipe`、`cv_bridge`） |
| `test/test_copyright.py` 等 | ament lint 三件套 |

---

## 十一、microros_v2_opencv_visual_pkg

**职责**：OpenCV 经典视觉识别节点的集合，输出检测结果并可选叠加到调试图像话题。

| 文件 | 作用 |
|---|---|
| `.../detect_color.py` | 颜色识别：HSV 阈值 + 形态学，输出色块中心与面积 |
| `.../detect_face.py` | 人脸识别：Haar 级联检测 |
| `.../detect_qr.py` | 二维码检测解码 |
| `.../parse_qrcode.py` | 二维码内容解析：按分隔符拆字段（如坐标指令） |
| `.../create_qrcode.py` | 生成二维码图片（用于制作测试标签） |
| `.../detect_apriltag.py` | AprilTag / ArUco 检测与位姿解算 |
| `.../detect_object.py` | 通用物体检测：加载 `ssd_mobilenet_v2_coco` 做 COCO 80 类检测 |
| `.../detect_pose.py` | 人体姿态检测（OpenPose 轻量模型） |
| `.../simple_ar.py` | 简单 AR：识别标记后在画面上叠加 3D 立方体 |
| `.../__init__.py` | 包标识 |
| `config/camera.yaml` | 相机内参与话题配置 |
| `config/haarcascade_frontalface_default.xml` | 人脸级联检测器 |
| `config/object_detection_coco.txt` | COCO 80 类类别名表 |
| `config/ssd_mobilenet_v2_coco.txt` | SSD MobileNet v2 模型结构描述 |
| `launch/*.launch.py` | 9 个启动文件，对应上述 9 个节点 |
| `setup.py` | 声明 9 个 `console_scripts` 入口 |
| `setup.cfg` / `package.xml` | 安装路径与依赖 |
| `test/test_copyright.py` 等 | ament lint 三件套 |

---

## 十二、microros_v2_opencv_base_pkg

**职责**：**纯 OpenCV 教学脚本**，不依赖 ROS 2，放在包里只是为了随工作空间一起安装。按功能分 4 个子目录。

| 文件 | 作用 |
|---|---|
| `scripts/opencv_getting_started/opencv_read_show.py` | 读图 + 显示（入门） |
| `scripts/opencv_getting_started/opencv_write.py` | 读图 + 保存 |
| `scripts/opencv_getting_started/opencv_pixel_manipulation.py` | 像素级读写与颜色通道操作 |
| `scripts/opencv_geometric_transformation/opencv_image_cropping.py` | 图像裁剪 |
| `scripts/opencv_geometric_transformation/opencv_image_mirroring.py` | 镜像翻转 |
| `scripts/opencv_geometric_transformation/opencv_image_panning.py` | 平移 |
| `scripts/opencv_geometric_transformation/opencv_zoom_out_in.py` | 缩放 |
| `scripts/opencv_image_processing_and_drawing/opencv_draw_line.py` | 画直线 |
| `scripts/opencv_image_processing_and_drawing/opencv_draw_circle.py` | 画圆 |
| `scripts/opencv_image_processing_and_drawing/opencv_draw_rectangle.py` | 画矩形 |
| `scripts/opencv_image_processing_and_drawing/opencv_draw_ellipse.py` | 画椭圆 |
| `scripts/opencv_image_processing_and_drawing/opencv_draw_polygon.py` | 画多边形 |
| `scripts/opencv_image_processing_and_drawing/opencv_draw_text.py` | 写字 |
| `scripts/opencv_image_processing_and_drawing/opencv_image_grayscale_conversion.py` | 灰度化 |
| `scripts/opencv_image_processing_and_drawing/opencv_image_binarization_processing.py` | 二值化 |
| `scripts/opencv_image_processing_and_drawing/opencv_image_edge_detection.py` | 边缘检测（Canny/Sobel） |
| `scripts/opencv_image_enhancement/opencv_image_brightness_enhancement.py` | 亮度增强 |
| `scripts/opencv_image_enhancement/opencv_skin_smoothing_brightening.py` | 磨皮美白 |
| `scripts/opencv_image_enhancement/opencv_create_damaged_image.py` | 人为制造破损图（供修复演示） |
| `scripts/opencv_image_enhancement/opencv_repair_damaged_images.py` | 图像修复（inpaint） |
| `scripts/images/` | 上述脚本使用的示例图片素材 |

---

## 十三、microros_v2_yolo_pkg

**职责**：YOLO 系列的 **ROS 2 节点全集**，按任务类型（检测/分割/姿态/OBB/分类）× 输入源（图片/视频/实时相机）组合出 15 个节点。

| 文件 | 作用 |
|---|---|
| `.../yolo_common.py` | 公共封装：模型加载、预处理、NMS、结果绘制、类别名映射 |
| `.../image_message_utils.py` | ROS 图像消息 ↔ OpenCV Mat 转换（含编码 bgr8/rgb8 兼容） |
| `.../yolo_realtime_detect_node.py` | 实时检测（订阅相机话题） |
| `.../yolo_image_detect_node.py` | 单张图片检测 |
| `.../yolo_video_detect_node.py` | 视频文件检测 |
| `.../yolo_seg_realtime_node.py` | 实时实例分割 |
| `.../yolo_seg_image_node.py` | 图片分割 |
| `.../yolo_seg_video_node.py` | 视频分割 |
| `.../yolo_pose_realtime_node.py` | 实时人体姿态估计 |
| `.../yolo_pose_image_node.py` | 图片姿态估计 |
| `.../yolo_pose_video_node.py` | 视频姿态估计 |
| `.../yolo_cls_realtime_node.py` | 实时图像分类 |
| `.../yolo_cls_image_node.py` | 图片分类 |
| `.../yolo_cls_video_node.py` | 视频分类 |
| `.../yolo_obb_realtime_node.py` | 实时旋转框检测（OBB，适合航拍/密集目标） |
| `.../yolo_obb_image_node.py` | 图片旋转框检测 |
| `.../yolo_obb_video_node.py` | 视频旋转框检测 |
| `.../__init__.py` | 包标识 |
| `requirements.txt` | Python 依赖（`ultralytics` 等） |
| `launch/*.launch.py` | 15 个启动文件，与 15 个节点一一对应 |
| `setup.py` | 声明 15 个 `console_scripts` 入口 |
| `setup.cfg` / `package.xml` | 安装路径与依赖 |
| `test/test_copyright.py` 等 | ament lint 三件套 |

---

## 十四、yolo_onnx_follow_pkg

**职责**：**不依赖 ultralytics** 的轻量方案——用 `cv2.dnn` 直接跑 YOLOv11 ONNX 模型，鼠标点选目标后锁定，再用云台/底盘跟随。适合算力有限的嵌入式上位机。

| 文件 | 作用 |
|---|---|
| `yolo_onnx_follow_pkg/yolo_onnx_follow_node.py` | 主节点：订阅 `/camera/image_raw` → DNN 推理 → 点选锁定目标 → 计算像素误差 → 输出云台角度与底盘角速度 |
| `yolo_onnx_follow_pkg/ptz_controller.py` | 云台 PD 控制器：像素误差 → `/cmd_ptz_angle`，含死区、限幅、平滑 |
| `yolo_onnx_follow_pkg/__init__.py` | 包标识 |
| `config/yolo_onnx_follow_config.yaml` | 全量参数：`model_path`（默认 `~/MODELS/yolo/yolo11n.onnx`）、类别名、相机话题、输入尺寸、置信度/NMS 阈值、CUDA 开关、PTZ 话题、像素容差、水平/俯仰 P+D、最大步进、底盘 yaw 限幅、前馈补偿；并区分 `CAR_TYPE=ptz` 与 `no_ptz` 两种控制策略 |
| `launch/yolo_onnx_follow.launch.py` | 启动参数装配 |
| `README.md` | 使用说明（模型获取、点选操作、参数调优） |
| `setup.py` | 声明 `yolo_onnx_follow_node` 入口 |
| `setup.cfg` / `package.xml` | 安装路径与依赖（仅需 `opencv-python`、`cv_bridge`） |

---

## 十五、microros_v2_kcf_follow_pkg

**职责**：**纯跟随控制器**。它不做跟踪算法本身，只订阅外部发布的 `/tracking_bbox`，据此保持目标居中并可选择按目标框大小维持跟随距离。这样任何跟踪器（KCF/YOLO/MediaPipe）都能复用同一套跟随逻辑。

| 文件 | 作用 |
|---|---|
| `microros_v2_kcf_follow_pkg/tracking_bbox_follow_node.py` | 跟随节点：订阅 `/tracking_bbox` 与 `/JoyState`，发布 `Twist` 到 `/cmd_vel`、`Vector3` 到 `/cmd_ptz_angle`；按框中心偏差做转向控制，按框面积做距离控制 |
| `microros_v2_kcf_follow_pkg/__init__.py` | 包标识 |
| `launch/tracking_bbox_follow.launch.py` | 参数集中入口：话题名（`/tracking_bbox`、`/cmd_vel`、`/cmd_ptz_angle`、`/JoyState`）、跟随使能、PTZ 开关、PID 增益、目标距离、云台限位、控制频率 |
| `setup.py` | 声明 `tracking_bbox_follow_node` 入口 |
| `setup.cfg` / `package.xml` | 安装路径与依赖（`interfaces`、`geometry_msgs`） |

---

## 十六、common_demo

**职责**：按功能分类的**教学示例集**，每个脚本可独立运行，用来验证底盘、雷达、相机、标定、多机等基础能力。

| 文件 | 作用 |
|---|---|
| `control/joy_control.py` | 手柄遥控：读手柄轴值 → `/cmd_vel` |
| `control/keyboard_contol.py` | 键盘遥控：WASD 按键 → `/cmd_vel` |
| `calibration/calibrate_linear.py` | 线速度标定：走固定命令距离，量实际距离算出比例因子 |
| `calibration/calibrate_angular_z.py` | 角速度标定：转固定角度，量实际角度算比例因子 |
| `lidar/laser_Avoidance.py` | 激光避障：前方扇形区域有障碍则减速/停 |
| `lidar/laser_patrol.py` | 激光巡逻：沿墙保持固定距离前进 |
| `lidar/laser_Tracker.py` | 激光跟随：锁定前方最近目标并保持距离 |
| `lidar/laser_Warning.py` | 激光告警：分区距离阈值触发提示/蜂鸣 |
| `camera/Measure_Distance.py` | 单目测距：按目标像素高度与已知尺寸估算距离 |
| `camera/calculate_volume.py` | 体积估算：结合深度与轮廓外接立方体 |
| `camera/Edge_Detection.py` | 边缘检测演示 |
| `camera/GetDepthColor.py` | 深度图伪彩色显示 |
| `mediapipe/01_HandDetector.py` … `19_mediapipe_gesture.py` | MediaPipe 各能力的最小示例（手部、姿态、全身、人脸网格、人脸检测、人脸关键点、Objectron、虚拟画笔、手指控制、手部定位、手指轨迹、手势跟随、姿态跟随、手部控制、手指动作、手势抓取、夹爪控制、手势移动、手势语义） |
| `mediapipe/media_common.py` | 示例共用的相机/绘图工具 |
| `mediapipe/media_library.py` | 示例共用的 MediaPipe 模型封装 |
| `model_display/model_display.py` | 在 RViz 中显示机器人模型与 TF |
| `multi_robot/multi_robot_sync.py` | 多机同步：按 namespace 广播/对齐多台机器人的时间与目标 |
| `yolo/yolo_detect.py` | YOLO 检测最小示例 |
| `yolo/yolo_track.py` | YOLO + 跟踪器（ByteTrack 风格）示例 |
| `setup.py` | 声明 `joy_control_node`、`keyboard_control`、`calibrate_linear`、`calibrate_angular_z`、`laser_*`、`model_display`、`multi_robot_sync` 入口 |
| `setup.cfg` / `package.xml` | 安装路径与依赖 |
| `test/test_copyright.py` 等 | ament lint 三件套 |

---

## 十七、.system_cfg（图形化配置工具）

**职责**：厂家提供的**免命令行**配置入口。RobotManager 是 Textual（TUI）应用，通过串口把参数写入控制板；桌面快捷方式用于一键启动常用功能。

| 文件 | 作用 |
|---|---|
| `robot_manager/robot_manager.py` | 程序入口：构建 Textual App 并 run |
| `robot_manager/panels.py` | 界面面板：控制板 IP 配置、`ROS_DOMAIN_ID` 设置、相机、WiFi、标定等分页表单 |
| `robot_manager/backend.py` | 后端逻辑：串口读写、`~/.bashrc` 中环境变量的读写（`read_env_from_bashrc` / `set_env_in_bashrc`） |
| `robot_manager/config.py` | 出厂默认值：`ROS_DOMAIN_ID = 40`、`ROS_NAMESPACE`、`AGENT_IP_ADDR`、`AGENT_IP_PORT = 8090`、WiFi 账号密码、舵机偏移、电机/IMU PID 等 |
| `robot_manager/config_robot.py` | 串口配置协议实现：把上面各项打包成帧写入 ESP32 |
| `robot_manager/i18n.py` | 中英文文案表 |
| `robot_manager/robot_manager.tcss` | Textual 样式表 |
| `robot_info.json` | 运行时机器人信息（相机 URL、Agent IP、`DIFY_BASE_URL`、`DIFY_API_KEY` 等），被 `robot_tool.py` 与 `multi_brains` 读取 |
| `application/robot_manager.desktop` | 桌面图标：打开 RobotManager |
| `application/navigation.desktop` | 桌面图标：启动导航 |
| `application/slam.desktop` | 桌面图标：启动建图 |
| `application/Joystick.desktop` | 桌面图标：启动手柄遥控 |
| `application/keyboard.desktop` | 桌面图标：启动键盘遥控 |
| `application/robot_info.desktop` | 桌面图标：打开机器人信息 |
| `icon/*.png` | 上述快捷方式使用的图标 |

---

## 十八、关键运行约定

### 18.1 环境变量

| 变量 | 含义 | 一致性要求 |
|---|---|---|
| `ROS_DOMAIN_ID` | ROS 2 域隔离，等价"频道" | **Agent 端与控制端必须相同**，否则 `ros2 topic list` 为空 |
| `RMW_IMPLEMENTATION` | DDS 实现，本项目为 `rmw_fastrtps_cpp` | 全网络一致 |
| `CAR_TYPE` | 车型标识（`ptz` / `no_ptz`），决定加载哪套视觉控制包与模型 | 需与实车硬件一致 |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | 指向 `config/fastdds_shm.xml`，启用共享内存传输 | 由 launch 自动注入 |

> 项目 `config.py` 出厂默认为 `ROS_DOMAIN_ID = 40`；教程文档示例为 `49`。以实车控制板实际写入值为准。

### 18.2 话题约定

| 话题 | 类型 | 方向 | QoS | 说明 |
|---|---|---|---|---|
| `/battery` | sensor_msgs/BatteryState | 控制板 → | RELIABLE | 电池电压 |
| `/imu` | sensor_msgs/Imu | 控制板 → | RELIABLE | IMU 原始数据 |
| `/scan` | sensor_msgs/LaserScan | 控制板 → | RELIABLE | 激光雷达 |
| `/odom_raw` | nav_msgs/Odometry | 控制板 → | RELIABLE | 原始里程计 |
| `/cmd_vel` | geometry_msgs/Twist | → 控制板 | RELIABLE | 底盘速度指令 |
| `/beep` | std_msgs | → 控制板 | RELIABLE | 蜂鸣器 |
| `/cmd_ptz_angle` | geometry_msgs/Vector3 | → 控制板 | RELIABLE | 云台目标角度 |
| `/ptz_angle` | geometry_msgs/Vector3 | 控制板 → | RELIABLE | 云台当前角度 |
| `/camera/image_raw` | sensor_msgs/Image | camera_driver → | **BEST_EFFORT** | bgr8 640×480，**发布端与订阅端 QoS 必须一致**，否则话题无数据 |
| `/camera/camera_info` | sensor_msgs/CameraInfo | camera_driver → | RELIABLE | 来自 `ost.yaml`；直跑 `camera_driver.py` 需手动传 `-p ost_file:=<绝对路径>`，launch 自动带上 |
| `/tracking_bbox` | interfaces/TrackingBbox | 跟踪器 → 跟随器 | RELIABLE | 目标框与置信度 |
| `/scan_points` | sensor_msgs/PointCloud2 | transmission → | BEST_EFFORT | 雷达转点云 |
| `/camera/rgb/image_raw/compressed` | sensor_msgs/CompressedImage | transmission → | BEST_EFFORT | 压缩图，供 App 拉流；发布端/订阅端 QoS 必须匹配 |

> **QoS 一致性是高频坑位**：ROS 2 话题发布端与订阅端的 `RELIABILITY` 策略必须相同，否则不会有数据传输。`camera_driver.py` 和 `transmission.py` 已统一使用 `BEST_EFFORT` 发布相机图像话题。

### 18.3 主要 action / service

| 接口 | 类型 | 提供者 |
|---|---|---|
| `compute_route` | roadnet/ComputeRoute | roadnet `route_server` |
| `compute_and_track_route` | roadnet/ComputeAndTrackRoute | roadnet `route_server` |
| `compute_route_and_follow` | roadnet/RoadNetToGoal | roadnet `route_server` |
| `navigate_to_pose` | nav2_msgs/NavigateToPose | nav2 `bt_navigator` |
| `move_by_odom` | interfaces/MoveByOdom | 底盘动作节点 |
| `precise_adjust_pose` | interfaces/PreciseAdjustPose | 定位调整节点 |
| `get_ptz_angle` / `get_robot_pose` / `set_tracking` / `web_save_map` | interfaces 服务 | 对应功能节点 |

---

## 十九、编译与运行速查

### 19.1 编译

```bash
cd /home/asang/mircoros_v2-1.0.1/mircoros_v2
source /opt/ros/jazzy/setup.bash

# 全量编译
colcon build --symlink-install

# 只编某个包及其依赖
colcon build --symlink-install --packages-up-to roadnet

# 排查失败时看最新事件日志
grep -E "Failed|error:" log/latest_build/events.log
```

> `roadnet` 使用 `nav2_package()` 宏，默认开启 `-Werror`，任何弃用警告都会变成编译错误。ROS 2 Jazzy 下应使用接受 `rclcpp::QoS` 的 `create_client` / `create_service` 重载，**不要传** `.get_rmw_qos_profile()` 或 `rmw_qos_profile_t`。

**错误写法（会触发 Jazzy 弃用警告 → -Werror 编译失败）**：

```cpp
// Jazzy 下 rmw_qos_profile_t 已被标记弃用
auto client = node->create_client<ComputeRoute>("compute_route",
  qos.get_rmw_qos_profile());  // ← 错误
```

**正确写法**：

```cpp
// 直接传 rclcpp::QoS 对象即可
auto client = node->create_client<ComputeRoute>("compute_route", qos);
// 或用传感器 QoS
auto client = node->create_client<ComputeRoute>("compute_route",
  rclcpp::SensorDataQoS());
```

### 19.2 启动前置条件

`bringup_app.launch.py` 会在启动时读取固定路径 `~/microros_ws/.system_cfg/robot_info.json`（由 `robot_tool.py` 的 `GetCameraUrl()` 调用），文件不存在会直接抛 `FileNotFoundError`。首次部署或换电脑必须先做这一步：

```bash
mkdir -p ~/microros_ws/.system_cfg
cp /home/asang/mircoros_v2-1.0.1/mircoros_v2/.system_cfg/robot_info.json \
   ~/microros_ws/.system_cfg/
# 然后把 camera_ip 改成当前相机地址（手机热点 DHCP 每次重启会变）
python3 -c "
import json, os
p = os.path.expanduser('~/microros_ws/.system_cfg/robot_info.json')
d = json.load(open(p, encoding='utf-8'))
d['camera_ip'] = '当前相机IP'  # ← 改成实际地址
json.dump(d, open(p, 'w', encoding='utf-8'), indent=4, ensure_ascii=False)
"
```

### 19.3 一键启动脚本（推荐）

项目根目录已提供两个自动化脚本，覆盖最常用的两种操作：

| 脚本 | 作用 | 用法 |
|---|---|---|
| `start_nomap.sh` | **一键启动不建图模式**：Agent + `bringup_app.launch.py slam_mode:=none` + `odom_pose_bridge.py`，Ctrl+C 全部退出，日志在 `~/.ros/nomap_logs/` | `./start_nomap.sh --check` 先自检，确认无误后 `./start_nomap.sh` |
| `auto_push.sh` | 自动提交并推送到 GitHub，默认 300 秒轮询 | `./auto_push.sh --watch` 常驻，`pkill -f auto_push.sh` 停止 |

> `start_nomap.sh` 会自动完成 19.2 的 `robot_info.json` 检查和复制，`auto_push.sh` 会通过 `.gitignore` 跳过 `build/`、`install/`、密钥文件。

### 19.4 手动启动顺序

#### 模式 A：不建图视觉伺服（推荐）

```bash
# 终端一：Agent（前台进程，关闭终端会终止）
source /opt/ros/jazzy/setup.bash
source ~/microros_agent_ws/install/local_setup.bash   # 按实际 Agent 工作空间路径调整
export ROS_DOMAIN_ID=40
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8090 -v6

# 终端二：主系统（相机 + rosbridge + 图传 + App 位姿桥接，**不启动任何 SLAM**）
source /opt/ros/jazzy/setup.bash
source /home/asang/mircoros_v2-1.0.1/mircoros_v2/install/setup.bash
export ROS_DOMAIN_ID=40
ros2 launch microros_v2_bringup bringup_app.launch.py \
  slam_mode:=none navigation_mode:=False

# 终端三：补位姿 TF（不建图时需要，因为 SLAM 不发布 map→odom）
source /opt/ros/jazzy/setup.bash
source /home/asang/mircoros_v2-1.0.1/mircoros_v2/install/setup.bash
export ROS_DOMAIN_ID=40
python3 /home/asang/mircoros_v2-1.0.1/mircoros_v2/src/ros_robot_app/ros_robot_app/odom_pose_bridge.py

# 终端四（可选）：大模型控制
ros2 launch multi_brains llm_agent_control.launch.py
ros2 run multi_brains terminal_client
```

#### 模式 B：建图导航（厂家默认）

```bash
# Agent 同上，然后：
ros2 launch microros_v2_bringup bringup_app.launch.py
# 默认 slam_mode=slam_toolbox，会自动启动在线建图

# 路网导航（额外需要先建图、再有 geojson 路网文件）
ros2 launch microros_v2_bringup roadnet_nav.launch.py graph:=my_graph.geojson
```

### 19.5 常见故障对照

| 现象 | 优先检查 |
|---|---|
| `bringup_app.launch.py` 启动报 `FileNotFoundError: ~/microros_ws/.system_cfg/robot_info.json` | 执行 19.2 的前置复制，或直接用 `./start_nomap.sh` |
| `ros2 topic list` 为空 | Agent 是否在跑（前台进程不能关）；两端 `ROS_DOMAIN_ID` 是否一致 |
| 控制板显示 False | Agent IP 填错、电脑防火墙、Wi-Fi 不通；手机热点 DHCP 重启后所有 IP 会变 |
| 能收数据但 `/cmd_vel` 不动 | USB 是否插着（插 USB 会切到手柄模式忽略 `/cmd_vel`）；电池电压是否 ≥ 6.50 V |
| 相机话题有发布者但 App 收不到压缩图 | 发布端/订阅端 QoS 是否一致（必须都是 BEST_EFFORT）；见 18.2 |
| 相机节点报打不开 HTTP MJPEG 流 | 先 `ping` 相机 IP，再浏览器打开 `http://<ip>:81/stream`；热点重启后 IP 可能变化 |
| `odom_pose_bridge.py` 启动时报重复 shutdown 警告（已修复） | 更新到最新版本，旧版 SIGTERM 处理有 bug |
| 视觉伺服靠近时来回振荡 | 帧率偏低（3～4 Hz）会导致控制滞后；检查 HTTP MJPEG 解码瓶颈 |
| 串口打不开 | `groups` 里是否有 `dialout` 权限 |
| `roadnet` 编译失败 | `-Werror` 下是否有 `rmw_qos_profile_t` 弃用警告；见 19.1 代码示例 |

### 19.6 硬件约束速记

- **USB 插控制板 → 手柄模式**，忽略 `/cmd_vel`；拔掉 USB 才恢复话题控制。
- **电池 < 6.50 V → 挂起所有模块**，Wi-Fi、相机、串口全部不工作。
- **手机热点 DHCP → 每次重启 IP 全变**（电脑、控制板、相机），需重新扫描并更新 `robot_info.json` 中的 `camera_ip`。
- **MicroROS Agent 是前台进程**，关闭终端 = 断链，必须保持运行。
- **ROS 2 Domain ID 必须两端一致**，不一致会导致节点互相发现失败、话题列表为空。

---

*文档生成时间：2026-11-12；最后更新：2026-11-24（补充不建图模式、启动前置条件、QoS 约定、roadnet Jazzy 修复示例、硬件约束速记、一键启动与自动推送脚本说明）*
