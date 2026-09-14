# 启动说明

## 1. 启动纯 TARE

先在宿主机终端中启动并进入 `tare_official` 容器：

```bash
cd /home/jack/tare_planner
export DISPLAY=:0
xhost +SI:localuser:root
docker compose up -d --force-recreate tare_official
docker compose exec tare_official bash
```

进入容器后加载 ROS 和 TARE 环境：

```bash
source /opt/ros/noetic/setup.bash
source /opt/tare_planner/devel/setup.bash
```

以下两个场景选择一个启动，不要在同一个终端中连续运行。

启动普通场景：

```bash
roslaunch tb3_tare_sim tb3_tare_fast_lio.launch
```

启动多障碍物场景：

```bash
roslaunch tb3_tare_sim tb3_tare_fast_lio_cluttered.launch
```

## 2. 启动 TARE + GBPlanner 多障碍物场景

先在宿主机终端中创建并进入混合规划容器：

```bash
cd /home/jack/tare_planner
export DISPLAY=${DISPLAY:-:0}
xhost +SI:localuser:root
docker compose run --rm --name tare_gbplanner_run tare_gbplanner_hybrid bash
```

进入容器后加载三个工作空间并启动：

```bash
source /opt/ros/noetic/setup.bash
source /opt/tare_planner/devel/setup.bash
source /opt/gbplanner_ws/devel/setup.bash

roslaunch tare_gbplanner_hybrid \
  tb3_tare_gbplanner_fast_lio_cluttered.launch
```

## 3. 启动 TARE + GBPlanner Pittsburgh 矿井场景

进入上面的混合规划容器后执行：

```bash
source /opt/ros/noetic/setup.bash
source /opt/tare_planner/devel/setup.bash
source /opt/gbplanner_ws/devel/setup.bash

roslaunch tare_gbplanner_hybrid tb3_tare_gbplanner_fast_lio_pittsburgh.launch
```

RViz 打开后，在左侧 `Exploration Control` 面板中点击 `Start Navigation` 开始探索，点击 `Stop` 停车，点击 `Continue` 恢复探索。
# TARE_GBPlanner
