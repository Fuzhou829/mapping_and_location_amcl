/*
 * @Description = Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version = 1.0
 * @Author = renjy
 * @Date = 2022-10-29 10:54:02
 * @LastEditTime: 2023-08-09 06:15:15
 */
#include <math.h>

#include "include/config_struct.h"


namespace gomros {
namespace data_process {
namespace mapping_and_location {

EKFConfig::EKFConfig() {
  control_left_error = 0.002;
  control_right_error = 0.002;
  observation_noise_dis = 0.01;
  observation_noise_theta = 0.02;
  imu_theta_noise = 0.01;
  ma_distance_threshold = 15;
  odom_type = 0;
}


RecordConfig::RecordConfig() {
  flush_odom_list_size = 200;
  distance_step = 0.1;
  angular_step = 0.1;
  mapping_start_angle = -95.0;
  mapping_end_angle = 95.0;
  laser_resolution = 0.5;
  odom_type = 0;
  is_need_record_data = false;
}


SensorMount::SensorMount() {
  radar_position_x = 0.0;
  radar_position_y = 0.0;
  radar_position_theta = 0.0;
  // 相机安装位置
  camera_position_x = 0.0f;
  camera_position_y = 0.0f;
  camera_position_theta = 0.5;  // 旋转0.5*M_PI

  wheel_distance = 0.61;
  wheel_diameter = 0.18;

  front_wheel_length = 0.88581;
  back_wheel_length = 0.88581;
  front_wheel_alpha = 0.3335;
  back_wheel_alpha = 0.3335;

  to_centre_distance = 0.8;
}

LandMarkConfig::LandMarkConfig() {
  landmark_intensities = 800.0f;
  landmark_radius = 0.0375;
  landmark_size_threshold = 20;
  min_reflector_count_in_map = 25;
  max_ladar_size_in_sub_map = 160;
  distance_for_constraint = 0.1;
  distance_for_loop = 0.2;
  distance_for_same_reflector = 0.05;

  data_association_max_dis = 30.0f;
  observation_error = 0.03;
  map_ma_match_dis_threshold = 20.0;
  add_new_center_min_dis = 1.0f;
  distance_diff_threshold = 0.1;
  reflector_max_distance = 12.8f;
  error_threshold_for_loop = 0.1;
  recognition_type = RecognitionType::Cluster;
}

// MappingConfig::MappingConfig() {
//   mapping_pattern = MappingPattern::Offline;
//   distance_step = 0.2;
//   angular_step = 0.2;
//   mapping_start_angle = -95.0;
//   mapping_end_angle = 95.0;
//   laser_resolution = 0.5;
//   mapping_resolution = 0.02;
//   radar_position_x = 0.473891;
//   radar_position_y = 0.00692;
//   radar_position_theta = 0.002559;
//   mapping_laser_min_range = 0.5;
//   mapping_laser_max_range = 30;
//   map_data_file_path = "./map";
//   map_config_file_path = "./config";
// }


MappingConfig::MappingConfig() {
  mapping_pattern = MappingPattern::Offline;
  laser_resolution = 0.5;
  mapping_resolution = 0.02;
  mapping_laser_min_range = 0.35;
  mapping_laser_max_range = 20.0;
  mapping_start_angle = -95.0;
  mapping_end_angle = 95.0;
  map_config_file_path = "./config";
  map_data_file_path = "./map";
  odom_type = 0;
}




LocationConfig::LocationConfig() {
  location_start_angle = -95.0f;
  location_end_angle = 95.0f;
  location_laser_resolution = 0.5f;

  use_scan_matching = false;
  scan_matching_min_distance = 0.0;
  scan_matching_max_distance = 29.0f;     // 19.0 scan_matching
  occupied_space_cost_factor = 1.0f;
  translation_delta_cost_factor = 10.0f;
  rotation_delta_cost_factor = 10.0f;
  num_threads = 2;
  max_num_iterations = 10;

  laser_min_range = 0.5;
  laser_max_range = 29.0f;

  odom_type = 0;
  location_type = LocationType::AMCL;            // 定位模式
  // 初始位姿文件存放路径,必须和地图管理模块的地图文件路径一样
  init_pose_file_path = "./map";
  map_data_file_path = "./map";
}



MappingAndLocationConfig::MappingAndLocationConfig() {
  node_name = "mapping_and_location";
  sub_odom_topic = "odom_topic";
  sub_radar_topic = "radar/wj716/sensory";
  sub_imu_topic = "imu_sensory_msg";
  sub_qr_topic = "qr_message";
  sub_global_locate_cmd_topic = "global_locate_cmd";
  sub_start_mapping_cmd_topic = "start_mapping";
  sub_stop_mapping_cmd_topic = "stop_mapping";
  ad_state_topic = "location_state";
  ad_pose_topic = "location_pose";
  log_file_name = "MappingAndLocation";
  log_level = 0;
  is_printf_to_terminal = true;
  mapping_type = MappingType::Fusion;
}

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
