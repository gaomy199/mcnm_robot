本文件夹为放置雷达的配置文件，本项目使用的雷达为思岚c1m1型号。
思岚系列的雷达驱动在github仓库地址为：https://github.com/Slamtec/rplidar_ros.git
请cd到lidar_ws/src目录下，执行以下命令：

```
git clone https://github.com/Slamtec/rplidar_ros.git
```

成功克隆仓库后，cd ..到lidar_ws目录下然后编译雷达驱动

```
cd lidar_ws          # 进到含 src/ 的这一层
colcon build --symlink-install
```

编译完成后，lidar_ws目录下应该如下所示:

```
lidar_ws -
         -src/rplidar_ros
         -build
         -install
         -log
```

可以执行以下命令来验证是否编译成功：

```
source install/setup.bash
ros2 launch src/rplidar_ros/launch rplidar_c1_launch
```