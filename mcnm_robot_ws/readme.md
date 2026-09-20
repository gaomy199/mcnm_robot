这个文件夹包含了所有的机器人模型文件和任务启动文件，其中:
mcnm_robot_ws下面的这个pdf为本项目机器人的TF结构
src文件夹下面的具体各个功能包为:
- cooling_fan_controller：风扇控制功能包，包含风扇控制节点和配置文件，用于控制机器人主控散热风扇。
- mcnm_robot_bringup：机器人启动文件，包含启动机器人各个功能包的launch文件和配置文件，用于启动机器人系统
  task_points.yaml为室内地图的目标点和返航点，还有机械臂抓取的配置文件，这些都需要手动更改，**task_executor.py为机器人任务的状态机定义脚本，请详细阅读**。
- mcnm_robot_cartographer：机器人建图功能包，包含Cartographer建图算法的配置文件和节点，用于实现机器人自主建图。
- mcnm_robot_description：机器人模型文件，包含URDF、SDF、XACRO等文件，用于描述机器人结构和外观，    
  **display_real.launch.py这个launch文件是启动机器人所有的节点文件，请详细阅读，配合task_executor.py阅读即可弄清楚本项目的workspace下面所有的功能包逻辑关系!!!**
- mcnm_robot_navigation2：机器人导航功能包，包含路径规划、地图构建、定位等功能，用于实现机器人自主导航。
- micro_ros_agent和micro_ros_msgs：micro-ROS Agent和消息定义功能包，和micors_ws那里的一样，可以那边克隆后的直接把两个包复制过来覆盖这里的，这里"重复"放置是为了方便通过launch脚本一步启动所有节点。
- rplidar_ros：RPLIDAR激光雷达功能包，同上，放在这里单纯为了方便启动。
- xbox_teleop：机器人底盘遥控功能包，写了一个xbox遥控器的节点脚本，可以代替键盘控制机器人底盘运动。

使用方法:
在mcnm_robot_ws目录下进行编译:
```
colcon build --symlink-install
source install/setup.bash
```

如果同时编译会出现问题，那就对src下面的包一个个执行编译:

```
colcon build --packages-select <包名> --symlink-install
例如 colcon build --packages-select cooling_fan_controller --symlink-install
```

**执行命令:**
如果是使用键盘控制节点遥控机器人底盘运动:
```
# 终端1
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
# 终端2
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

如果是测试雷达功能:
```
# 终端1
rviz2
# 终端2
ros2 launch rplidar_ros rplidar_c1_launch.py
```

如果是使用雷达建图:
```
# 终端1
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
# 终端2
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 终端3，记得把display_real.launch.py里面的use_rviz值改成false，直接在终端3执行rviz2
rviz2
# 终端4，机器人节点启动
ros2 launch mcnm_robot_description display_real.launch.py
# 终端5 使用cartographer建图
ros2 launch mcnm_robot_cartographer cartographer.launch.py
# 或者用slam_toolbox建图，真机False，仿真建图改成True
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=False
```

如果基于已经有的室内地图进行导航，请提前修改mcnm_robot_ws\src\mcnm_robot_bringup\config\task_points.yaml里面的数据:
```
# 上下位机连接，风扇启动，机器人rviz打开……
ros2 launch mcnm_robot_description display_real.launch.py
# 启动机器人任务状态机，执行任务点的导航和机械臂抓取……
ros2 launch mcnm_robot_bringup task.launch.py
```