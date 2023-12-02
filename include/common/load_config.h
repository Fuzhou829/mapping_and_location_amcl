/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-03-10 13:47:58
 * @LastEditors: renjy
 * @LastEditTime: 2023-08-09 06:15:35
 */
#pragma once
#include <string>

#include "agv_config_lib/ConfigUtils.h"
#include "include/config_struct.h"

namespace internal_common {

typedef struct ReadMappingAndConfig {
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
  std::string map_data_file_path;           // 地图存放地址
  std::string map_config_file_path;         // 地图存放地址
  std::string init_pose_file_path;          // 初始位姿存放位置

  std::string log_file_name;   // 日志文件文件名
  int log_level;               // 日志等级
  bool is_printf_to_terminal;  // 是否将日志输出到终端

  int mapping_type;     // 建图模式
  int location_type;    // 定位模式
  int mapping_pattern;  // 建图形式（1：离线建图 else.在线建图）
  int odom_type;        // odom:形式

  float mapping_resolution;    // 建图分辨率 单位：m
  float flush_odom_list_size;  // odom刷新大小
  float distance_step;         // 建图记录数据距离步长 单位：m
  float angular_step;          // 建图记录数据角度步长 单位：弧度

  float mapping_start_angle;       // 建图雷达起始角度 单位：度
  float mapping_end_angle;         // 建图雷达终止角度 单位：度
  float mapping_laser_resolution;  // 建图雷达角度分辨率 单位：度
  float mapping_laser_min_range;   // 建图雷达最小距离 单位：m
  float mapping_laser_max_range;   // 建图雷达最大距离 单位：m

  float location_start_angle;       // 定位雷达起始角度 单位：度
  float location_end_angle;         // 定位雷达终止角度 单位：度
  float location_laser_resolution;  // 定位雷达角度分辨率 单位：度
  float location_laser_min_range;   // 定位雷达最小距离 单位：m
  float location_laser_max_range;   // 定位雷达最大距离 单位：m

  int recognition_type;  // 反光柱识别方式 0 - 聚类 1 - 寻找波峰
  float landmark_intensities;      // 反光柱光强阈值 倍加福 800
  float landmark_size_threshold;   // 反光柱聚类数据阈值
  float landmark_radius;           // 反光柱半径 单位：m
  int min_reflector_count_in_map;  // 纯反光柱建图 最少识别次数
  int max_ladar_size_in_sub_map;  // 纯反光柱建图，子图中最多处理的雷达帧
  float distance_for_constraint;  // 纯反光柱建图 与相邻子图约束距离阈值 单位：m
  float distance_for_loop;  // 纯反光柱建图 与闭环子图建立约束距离阈值 单位：m
  float distance_for_same_reflector;  // 纯反光版建图 关联时距离阈值 单位：m

  float data_association_max_dis;  // 反光柱定位数据关联最大距离 单位：m
  float observation_error;           // 反光柱观测误差
  float map_ma_match_dis_threshold;  // 数据关联马氏距离阈值
  float add_new_center_min_dis;    // 增加新反光柱最小距离 单位：m
  float reflector_max_distance;    // 反光柱最大距离 单位：m
  float distance_diff_threshold;   // 反光柱距离差值阈值 单位：m
  float error_threshold_for_loop;  // 闭环阈值误差 单位：m

  // 雷达安装位置
  float radar_position_x;      // 单位：m
  float radar_position_y;      // 单位：m
  float radar_position_theta;  // 单位：弧度
  // 相机安装位置
  float camera_position_x;      // 单位: m
  float camera_position_y;      // 单位：m
  float camera_position_theta;  // 单位：180度

  // scan match 相关配置
  bool use_scan_matching;
  float scan_matching_min_distance;     // 0.0  scan_matching
  float scan_matching_max_distance;     // 19.0 scan_matching
  float occupied_space_cost_factor;     // 1.0 scan_matching
  float translation_delta_cost_factor;  // 10.0 scan_matching
  float rotation_delta_cost_factor;     // 10.0 scan_matching
  int num_threads;                      // 2 scan_matching
  int max_num_iterations;               // 10 scan_matching

  // EKF 相关配置
  float control_left_error;   // 控制标准差 左轮 单位：m
  float control_right_error;  // 控制标准差 右轮 单位：m
  float wheel_distance;       // 轮间距 单位：m
  float wheel_diameter;       // 轮直径 单位：m

  float observation_noise_dis;    // 观测噪声
  float observation_noise_theta;  // 观测噪声
  float imu_theta_noise;          // imu角度噪声 标准差
  float ma_distance_threshold;    // 马氏距离阈值

  float front_wheel_length;  // 前轮距离车体中心的距离 单位：m
  float back_wheel_length;   // 后轮距离车体中心的距离 单位：m
  float front_wheel_alpha;  // 前舵轮与底盘中心的夹角 单位：弧度
  float back_wheel_alpha;   // 后舵轮与底盘中心的夹角 单位：弧度

  float to_centre_distance;  // 单舵轮到车体中心的距离 单位：m

  bool use_weighted_areas;  // 是否使用权重区域
  bool use_landmark;
  int grid_weight;
  int landmark_weight;
  bool is_need_record_data;  // 是否需要记录定位原始数据

  XPACK(O(node_name, sub_odom_topic, sub_radar_topic, sub_imu_topic,
          sub_qr_topic, sub_global_locate_cmd_topic,
          sub_start_mapping_cmd_topic, sub_stop_mapping_cmd_topic,
          sub_calibration_cmd_topic, ad_state_topic, ad_pose_topic,
          map_data_file_path, map_config_file_path, init_pose_file_path,
          log_file_name, log_level, is_printf_to_terminal, mapping_type,
          location_type, mapping_pattern, odom_type, mapping_resolution,
          flush_odom_list_size, distance_step, angular_step,
          mapping_start_angle, mapping_end_angle, mapping_laser_resolution,
          mapping_laser_min_range, mapping_laser_max_range,
          location_start_angle, location_end_angle, location_laser_resolution,
          location_laser_min_range, location_laser_max_range, recognition_type,
          landmark_intensities, landmark_size_threshold, landmark_radius,
          min_reflector_count_in_map, max_ladar_size_in_sub_map,
          distance_for_constraint, distance_for_loop,
          distance_for_same_reflector, data_association_max_dis,
          observation_error, map_ma_match_dis_threshold, add_new_center_min_dis,
          reflector_max_distance, distance_diff_threshold,
          error_threshold_for_loop, radar_position_x, radar_position_y,
          radar_position_theta, camera_position_x, camera_position_y,
          camera_position_theta, use_scan_matching, scan_matching_min_distance,
          scan_matching_max_distance, occupied_space_cost_factor,
          translation_delta_cost_factor, rotation_delta_cost_factor,
          num_threads, max_num_iterations, control_left_error,
          control_right_error, wheel_distance, observation_noise_dis,
          observation_noise_theta, imu_theta_noise, ma_distance_threshold,
          front_wheel_length, back_wheel_length, front_wheel_alpha,
          back_wheel_alpha, to_centre_distance, use_weighted_areas,
          use_landmark, grid_weight, landmark_weight, is_need_record_data));
} ReadMappingAndConfig;

void MappingAndLocationConfigTransition(
    const ReadMappingAndConfig& config,
    gomros::data_process::mapping_and_location::MappingAndLocationConfig*
        config_result);

bool ReadMappingAndLocationConfig(
    const std::string& config_dir,
    gomros::data_process::mapping_and_location::MappingAndLocationConfig*
        config_result);

}  // namespace internal_common
