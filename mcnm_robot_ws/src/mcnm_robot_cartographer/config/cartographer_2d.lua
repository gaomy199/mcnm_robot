include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- TF 坐标系配置 (非常重要，必须与你的小车TF树严格对应)
  map_frame = "map",
  tracking_frame = "base_footprint", -- 机器人的基准坐标系，通常是 base_link 或 base_footprint
  published_frame = "odom",          -- Cartographer 将发布 map -> odom 的变换
  odom_frame = "odom",               -- 里程计坐标系
  provide_odom_frame = false,        -- 如果你的小车底盘已经发布了 odom，这里设为 false
  publish_frame_projected_to_2d = true,

  -- 传感器配置
  use_odometry = true,               -- 是否使用轮式里程计
  use_nav_sat = false,               -- 不使用 GPS
  use_landmarks = false,             -- 不使用地标
  num_laser_scans = 1,               -- 单线/多线 2D 激光雷达数量为 1
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,              -- 设为 0，除非你用 3D 多线雷达或深度相机点云建图

  -- 节点发布频率参数
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  -- 传感器数据采样率
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- 开启 2D SLAM 模式
MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D 轨迹构建器具体参数调优
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- 如果你的小车没有开启 IMU，一定要设为 false
TRAJECTORY_BUILDER_2D.min_range = 0.1       -- 激光雷达最小有效距离 (米)
TRAJECTORY_BUILDER_2D.max_range = 12.0      -- 激光雷达最大有效距离 (米)

-- 子图大小
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120

-- 实时扫描匹配，缩小搜索窗口防止对称误匹配
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.05
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e4

-- 回环检测门槛提高，减少假回环
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.optimize_every_n_nodes = 60

-- 加大里程计信任权重
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options