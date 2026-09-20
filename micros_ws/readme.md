本文件夹用于放置 micro-ROS Agent 相关功能包，负责在上位机（Jetson/Ubuntu）与 ESP32 之间通过 micro-ROS 桥接通信。
本项目使用 ROS2 Humble，需要克隆以下两个官方仓库（注意都要切到 humble 分支）：

- micro_ros_msgs（消息定义）：https://github.com/micro-ROS/micro_ros_msgs.git
- micro-ROS-Agent（Agent 节点）：https://github.com/micro-ROS/micro-ROS-Agent.git

请 cd 到 micros_ws/src 目录下，执行以下命令：

```
git clone -b humble https://github.com/micro-ROS/micro_ros_msgs.git
git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git
```

成功克隆后，cd .. 回到 micros_ws 目录编译：

```
cd micros_ws
rosdep install --from-paths src --ignore-src -y   # 安装依赖，可选但建议
colcon build --symlink-install
```

注意：micro-ROS-Agent 第一次编译时会自动从网上拉取并编译 Micro-XRCE-DDS-Agent 等依赖，需要联网、耗时较长；若报网络错误，重试或配置代理即可。

编译完成后 source 环境并验证：

```
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent --help
```