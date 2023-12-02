/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-09-28 09:48:44
 * @LastEditTime: 2023-09-26 17:22:12
 * @Author: lcfc-desktop
 */
#pragma once

#include <stdint.h>
#include <string>

namespace gomros {
namespace data_process {
namespace mapping_and_location {
/**
 * @brief 建图方式枚举：Online为在线建图，Offline为离线建图
 *
 */
enum MappingPattern { Online, Offline };

enum MappingType {
  Karto = 0,  // Karto 建图及定位
  Fusion = 1,  // 融合 建图及定位
  Reflector = 2,  // 纯反光柱建图
  Trilateral_Mapping = 3,   // 纯多边建图！！！
  Optimize_Mapping = 4
};

enum LocationType {
  AMCL = 0,
  QR = 1,
  LandMarkLocation = 2,
  FusionLocation = 3,
  WanJiLandMarkLocation = 4,       // 为万集雷达定位写接口
  Trilateral = 5,                // 三边定位
  WanJiAndQR = 6
};

enum OdomType {
  DiffWheelModel = 0,       // 双轮差速模型
  SingleSteerModel = 1,     // 单舵轮模型
  DoubleSteerModel = 2,     // 双舵轮模型
  FourSteerModel = 3        // 四舵轮模型
};

enum RecognitionType {
  Cluster = 0,          // 聚类
  FindPeak = 1          // 波峰
};

/**
 * @brief: 传感器安装位置
 */
struct SensorMount {
  // 雷达安装位置
  float radar_position_x;
  float radar_position_y;
  float radar_position_theta;
  // 相机安装位置
  float camera_position_x;
  float camera_position_y;
  float camera_position_theta;

  // odom相关配置
  /* 双轮差速模型 */
  float wheel_distance;  // 轮间距 单位：m
  float wheel_diameter;  // 轮直径 单位：m
  /* 双舵轮模型 */
  float front_wheel_length;   // 前轮距离车体中心的距离 单位：m
  float back_wheel_length;   // 后轮距离车体中心的距离 单位：m
  float front_wheel_alpha;    // 前舵轮与底盘中心的夹角 单位：弧度
  float back_wheel_alpha;     // 后舵轮与底盘中心的夹角 单位：弧度

  // 单舵轮模型参数
  float to_centre_distance;   // 单舵轮到车体坐标中心

  SensorMount();
};

/**
 * @brief: EKF相关参数
 * @return {*}
 */
struct EKFConfig {
  double control_left_error;    // 控制标准差 左轮 单位：m
  double control_right_error;   // 控制标准差 右轮 单位：m
  double observation_noise_dis;      // 观测噪声
  double observation_noise_theta;    // 观测噪声
  double imu_theta_noise;            // imu角度噪声 标准差
  double ma_distance_threshold;     // 马氏距离阈值 TODO(r)
  int odom_type;
  SensorMount sensor_mount;
  EKFConfig();
};

/**
 * @brief: 离线数据记录相关配置参数
 */
struct RecordConfig {
  MappingType mapping_type;   // 建图模式
  int flush_odom_list_size;  // odom list 更新阈值
  float distance_step;       // 激光记录距离步长 单位：m
  float angular_step;  // 激光记录角度步长 单位：角度 TODO(r) 核对一下
  float mapping_start_angle;  // 激光起始角度 单位：角度
  float mapping_end_angle;    // 激光结束角度 单位：角度
  float laser_resolution;     // 激光分辨率 单位：角度
  int odom_type;
  bool is_need_record_data;   // 是否需要记录数据（用于测试定位偏转的问题）
  RecordConfig();
};


/**
 * @brief: landmark配置参数
 */
struct LandMarkConfig {
  float landmark_intensities;       // 反光柱的光强信息
  float landmark_radius;            // 反光柱半径 单位：m
  int landmark_size_threshold;      // 反光柱位置拟合最少个数
  int min_reflector_count_in_map;   // 纯反光柱建图 最少识别次数
  int max_ladar_size_in_sub_map;    // 纯反光柱建图，子图中最多处理的雷达帧
  float laser_resolution;           // 激光雷达分辨率(建图和定位略有不同)

  float distance_for_constraint;  // 纯反光柱建图 相邻子图约束距离阈值 单位：m
  float distance_for_loop;  // 纯反光柱建图 与闭环子图建立约束距离阈值 单位：m
  float distance_for_same_reflector;  // 纯反光版建图 关联时距离阈值 单位：m

  float ladar_min_dis;  // 激光点最小距离 单位：m 为雷达的扫描范围
  float ladar_max_dis;  // 激光点最大距离 单位：m


  float data_association_max_dis;
  float observation_error;
  float map_ma_match_dis_threshold;
  float add_new_center_min_dis;
  float reflector_max_distance;
  float distance_diff_threshold;
  float error_threshold_for_loop;
  RecognitionType recognition_type;
  LandMarkConfig();
};


/**
 * @brief: 融合建图相关参数
 */
struct MappingConfig {
  MappingPattern mapping_pattern;  // 建图模式
  float laser_resolution;         // 激光的分辨率 单位：角度
  float mapping_resolution;       // 建图分辨率 单位：米
  float mapping_laser_min_range;  // 雷达最小观测距离 单位：米
  float mapping_laser_max_range;  // 雷达最大观测距离 单位：米
  float mapping_start_angle;      // 建图开始角度 单位：角度
  float mapping_end_angle;        // 建图结束角度 单位：角度
  std::string map_config_file_path;       // 地图存放地址
  std::string map_data_file_path;         // 地图存放地址
  int odom_type;
  EKFConfig ekf_config;           // EKF 相关参数
  RecordConfig record_config;     // 数据缓存的相关配置参数
  SensorMount sensor_mount;       // 安装位置
  LandMarkConfig land_mark_config;
  MappingConfig();
};


/**
 * @brief 栅格定位配置参数
 *
 */
struct LocationConfig {
  float location_start_angle;       // -95.0
  float location_end_angle;         // 95.0
  float location_laser_resolution;  // 0.5
  bool use_scan_matching;
  float scan_matching_min_distance;     // 0.0  scan_matching
  float scan_matching_max_distance;     // 19.0 scan_matching
  float occupied_space_cost_factor;     // 1.0 scan_matching
  float translation_delta_cost_factor;  // 10.0 scan_matching
  float rotation_delta_cost_factor;     // 10.0 scan_matching
  int num_threads;                      // 2 scan_matching
  int max_num_iterations;               // 10 scan_matching
  float laser_min_range;                // 0.5
  float laser_max_range;                // 19.0

  int odom_type;                        // 0

  bool use_weighted_areas;  // 是否使用权重区域
  bool use_landmark;
  int grid_weight;
  int landmark_weight;

  LocationType location_type;  // 定位模式
  // 初始位姿文件存放路径,必须和地图管理模块的地图文件路径一样
  std::string init_pose_file_path;
  std::string map_data_file_path;
  SensorMount sensor_mount;
  EKFConfig ekf_config;
  LandMarkConfig land_mark_config;
  LocationConfig();
};

/**
 * @brief 建图和栅格定位配置参数
 *
 */
struct MappingAndLocationConfig {
  std::string node_name;
  std::string sub_odom_topic;               // 订阅里程计话题
  std::string sub_radar_topic;              // 订阅雷达话题
  std::string sub_imu_topic;                // 订阅imu话题
  std::string sub_qr_topic;                 // 订阅二维码话题
  std::string sub_global_locate_cmd_topic;  // 订阅全局初始化命令
  std::string sub_start_mapping_cmd_topic;  // 订阅开始建图命令
  std::string sub_stop_mapping_cmd_topic;   // 订阅停止建图命令
  std::string sub_calibration_cmd_topic;    // 订阅标定命令
  std::string ad_state_topic;               // 更改状态发布的话题名
  std::string ad_pose_topic;                // 位姿发布的话题名

  std::string log_file_name;   // 日志文件文件名
  int log_level;               // 日志等级
  bool is_printf_to_terminal;  // 是否将日志输出到终端

  MappingType mapping_type;                 // 建图模式
  LocationConfig location_config;           // 定位配置参数
  MappingConfig mapping_config;  // 融合建图配置参数
  MappingAndLocationConfig();
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
