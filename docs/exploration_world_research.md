# 适合 TARE + GBPlanner 的狭小空间探索场景调研

日期：2026-09-13

## 结论

当前最值得优先测试的是 **官方 GBPlanner 仓库自带的 Pittsburgh Mine**，其次是同仓库的 **Virginia Mine**，再其次是 **AWS RoboMaker Hospital World（ROS1 分支）** 和 **Clearpath CPR Office Construction World**。Pittsburgh Mine 与当前 GBPlanner 版本同源，已经随混合镜像下载，几何范围约为 252 m × 327 m，最接近“大范围、长通道和分支空间”的目标。

不建议第一步直接接 DARPA SubT：它的隧道、城市地下和洞穴场景最有挑战性，但当前官方仓库以 Ignition Gazebo 和 `ros_ign_bridge` 为主，不是当前 ROS1 Noetic/Gazebo 11 链路的即插即用 world。

## 候选比较

| 候选 | 场景特点 | 当前工程接入难度 | 推荐用途 |
|---|---|---:|---|
| [官方 GBPlanner Pittsburgh Mine](https://github.com/ntnu-arl/gbplanner_ros/tree/gbplanner2/planner_gazebo_sim) | 官方 ground-robot demo 默认使用的矿井 world；几何约 252 m × 327 m，长距离通道和矿井结构适合大范围探索 | 低（资源已在混合镜像） | 首选压力测试；保持 Gazebo Classic，最能验证 TARE+GBPlanner 的全局/局部切换 |
| [官方 GBPlanner Virginia Mine](https://github.com/ntnu-arl/gbplanner_ros/tree/gbplanner2/planner_gazebo_sim) | 约 138 m × 36 m × 13 m 的矿井几何，通道更集中、垂向结构更多 | 低（资源已在混合镜像） | 第二个矿井压力测试；观察窄通道和高度变化对地面车的影响 |
| [AWS Hospital World ROS1](https://github.com/aws-robotics/aws-robomaker-hospital-world/tree/ros1) | 医院楼层、长走廊、病房/房间、护士站、电梯、病床和大量固定设备；仓库还提供单层、两层、三层 world | 中 | 首选单层大范围室内探索；先用 `hospital.world`，暂不使用多层/楼梯 |
| [Clearpath CPR Office Gazebo](https://github.com/clearpathrobotics/cpr_gazebo/tree/noetic-devel/cpr_office_gazebo) | 官方文档明确包含大空间、窄走廊、会议室/办公室；construction 版本有未完成墙体、不同地面和施工 debris/障碍 | 中 | 测试不规则障碍、狭窄通道和转弯约束；`office_construction_world.launch` 更贴近目标 |
| [AWS Small Warehouse World ROS1](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/tree/ros1) | 货架、箱体簇、托盘车、垃圾桶等，Gazebo world 和 ROS1 launch 齐全 | 低 | 作为规则货架环境的基线；窄空间和拓扑复杂度不如前两者 |
| [AWS Bookstore World ROS1](https://github.com/aws-robotics/aws-robomaker-bookstore-world/tree/ros1) | 大量书架、桌子和零售物品，适合局部密集障碍/信息增益测试 | 低 | 做局部 GBPlanner 对比，不作为最大范围测试场景 |
| [DARPA SubT / OSRF](https://github.com/osrf/subt) | 官方虚拟赛道含 tunnel、urban underground、cave；有大量分支、死胡同、坡道和受限通道，world 数据位于 [`subt_ign/worlds`](https://github.com/osrf/subt/tree/master/subt_ign/worlds) | 高 | 第二阶段压力测试；需要 Ignition/ROS bridge 或把场景几何转成 Gazebo Classic |

## 为什么优先官方 GBPlanner Mine

官方 GBPlanner2 README 的 ground-robot demo 默认加载 `pittsburgh_mine.world`，说明该场景是其自身算法链路的基准环境，而不是为另一套传感器或控制器临时制作的地图。当前混合镜像已经从官方固定提交下载了 `planner_gazebo_sim` 目录；Pittsburgh Mine 的静态 DAE 几何约为 252 m × 327 m，明显比当前 25 m × 25 m 障碍物集中区更适合测量长距离覆盖、回环和局部图搜索。

使用这些矿井 world 时仍需要在新实验 launch 中保留 TB3、Mid-360、FAST-LIO2 和现有控制接口，只替换 Gazebo 的 world/model 路径。原来的 `tb3_tare_fast_lio_cluttered.launch` 不动。

## 为什么其次选择 Hospital World

官方 ROS1 README 给出了 `hospital.world`、`hospital_two_floors.world` 和 `hospital_three_floors.world`，并明确支持 Gazebo 7.14+ / 9.16+。`hospital.world` 中包含医院楼层/墙体以及大量家具和医疗设备模型；从 world 中的布局坐标看，单层场景约为 22 m × 42 m，明显大于当前 25 m × 25 m 障碍物实验区，同时仍有长走廊和房间分支。

当前工程的 Gazebo 入口是 `tb3_tare_fast_lio.launch`，通过 `$(find vehicle_simulator)/world/$(arg world_name).world` 解析 world，而 TARE 规划参数通过 `scenario` 加载。因此不能只把 `world_name` 字符串改成 AWS 包名：需要额外把 Hospital World 的模型目录加入 `GAZEBO_MODEL_PATH`，并让 `vehicle_simulator` 能加载一个外部 world，或在新实验 launch 中直接 include `gazebo_ros/empty_world.launch` 后使用当前 TB3/传感器/FAST-LIO 节点。该实验应保持在新 launch 中，原有 `tb3_tare_fast_lio_cluttered.launch` 不动。

Hospital World 使用了 Fuel 模型，官方 README 要求先运行 `setup.sh` 并把 `models`、`fuel_models` 加入 `GAZEBO_MODEL_PATH`。它是 AWS RoboMaker 的历史资源，目前仓库已归档，适合研究和可重复仿真，不应直接作为生产依赖。

## 为什么第二个选 CPR Office Construction

Clearpath 的官方 ROS1 package 在 `noetic-devel` 分支提供 catkin package、Gazebo Classic launch 和办公室几何。其文档把场景特征直接分为 “Large, open areas”、“Narrow hallways”、“Meeting rooms & offices”；construction 版本另外提供未完成墙体、不同地面纹理和施工 debris/obstacles。场景几何由 `office.dae`/`office_construction.dae` 作为 URDF mesh 生成，不依赖数千个独立 Gazebo model，通常比当前随机生成大量障碍物更适合观察算法行为。

它的完整官方 launch 默认生成 Clearpath 自己的 Husky/Jackal/Ridgeback/Dingo/Boxer，当前项目仍应保留 TB3 模型，只复用办公室几何 URDF，把机器人 spawn 和已有 Mid-360/FAST-LIO 链路接到新 launch 中。这样不会改变原有 launch。

## DARPA SubT 的限制

DARPA 官方说明 SubT 的目标就是在复杂地下环境中进行快速建图、导航和搜索，场景包括人工隧道、地下城市和天然洞穴；OSRF 官方仓库也明确包含 Gazebo simulation assets、ROS interfaces、support scripts and plugins。但仓库当前的核心场景位于 `subt_ign`，启动文件使用 Ignition（例如 `cave_circuit.ign`、`tunnel_circuit_practice.ign`），ROS 接入依赖 `ros_ign_bridge`。当前镜像是 ROS Noetic + Gazebo Classic 11，不能直接把这些 `.sdf`/`.world` 放到现有 TARE launch 中；转换碰撞模型、传感器插件、时钟/TF 和 ROS 话题的成本较高。

## 建议的验证顺序

1. 用 AWS Hospital `hospital.world` 的单层版本，保持当前 TB3、Mid-360、FAST-LIO2 和 TARE/GBPlanner 参数不变，只替换几何与启动入口。记录覆盖率、有效探索距离、局部规划成功率、最小通道净空和是否触发原地旋转。
2. 用 CPR Office Construction，测试非规则墙体、施工障碍和窄走廊；将 `twoWayDrive=false`、底盘速度和最大角速度保持和真实光纤小车一致。
3. 最后再考虑 SubT，作为跨模拟器/高转换成本的极限场景，而不是当前 Noetic 链路的第一候选。

## 官方运行入口（仅用于确认资源）

AWS Hospital ROS1 官方 README 给出的直接 Gazebo 运行方式是：

```bash
chmod +x setup.sh
./setup.sh
export GAZEBO_MODEL_PATH=$PWD/models:$PWD/fuel_models:$GAZEBO_MODEL_PATH
gazebo worlds/hospital.world
```

Clearpath Office 官方 ROS1 运行入口是：

```bash
roslaunch cpr_office_gazebo office_world.launch
roslaunch cpr_office_gazebo office_construction_world.launch
```

上述命令用于验证第三方场景本身可加载；真正接入当前算法时，应另写实验 launch，避免覆盖已有可用链路。
