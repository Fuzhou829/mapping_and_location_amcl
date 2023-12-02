/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-03-10 13:48:07
 * @LastEditors: renjy
 * @LastEditTime: 2023-09-02 15:20:27
 */
#include "common/load_config.h"

namespace internal_common {

void MappingAndLocationConfigTransition(
    const ReadMappingAndConfig& config,
    gomros::data_process::mapping_and_location::MappingAndLocationConfig*
        config_result) {
  using namespace gomros::data_process::mapping_and_location;  // NOLINT
  EKFConfig ekf_config;
  RecordConfig record_config;
  SensorMount sensor_mount;
  LandMarkConfig land_mark_config;
  MappingConfig mapping_config;

  sensor_mount.camera_position_theta = config.camera_position_theta;
  sensor_mount.camera_position_x = config.camera_position_x;
  sensor_mount.camera_position_y = config.camera_position_y;

  sensor_mount.radar_position_theta = config.radar_position_theta;
  sensor_mount.radar_position_x = config.radar_position_x;
  sensor_mount.radar_position_y = config.radar_position_y;
  sensor_mount.wheel_distance = config.wheel_distance;
  sensor_mount.wheel_diameter = config.wheel_diameter;

  sensor_mount.front_wheel_length = config.front_wheel_length;
  sensor_mount.back_wheel_length = config.back_wheel_length;
  sensor_mount.front_wheel_alpha = config.front_wheel_alpha;
  sensor_mount.back_wheel_alpha = config.back_wheel_alpha;

  sensor_mount.to_centre_distance = config.to_centre_distance;

  ekf_config.control_left_error = config.control_left_error;
  ekf_config.control_right_error = config.control_right_error;
  ekf_config.imu_theta_noise = config.imu_theta_noise;
  ekf_config.ma_distance_threshold = config.ma_distance_threshold;
  ekf_config.observation_noise_dis = config.observation_noise_dis;
  ekf_config.observation_noise_theta = config.observation_noise_theta;
  ekf_config.odom_type = config.odom_type;
  ekf_config.sensor_mount = sensor_mount;

  record_config.angular_step = config.angular_step;
  record_config.distance_step = config.distance_step;
  record_config.flush_odom_list_size = config.flush_odom_list_size;
  record_config.mapping_start_angle = config.mapping_start_angle;
  record_config.mapping_end_angle = config.mapping_end_angle;
  record_config.laser_resolution = config.mapping_laser_resolution;
  record_config.odom_type = config.odom_type;
  record_config.is_need_record_data = config.is_need_record_data;
  record_config.mapping_type =
      (gomros::data_process::mapping_and_location::MappingType)
          config.mapping_type;

  land_mark_config.recognition_type =
      (gomros::data_process::mapping_and_location::RecognitionType)
          config.recognition_type;
  land_mark_config.landmark_intensities = config.landmark_intensities;
  land_mark_config.landmark_radius = config.landmark_radius;
  land_mark_config.landmark_size_threshold = config.landmark_size_threshold;
  land_mark_config.min_reflector_count_in_map =
      config.min_reflector_count_in_map;
  land_mark_config.max_ladar_size_in_sub_map = config.max_ladar_size_in_sub_map;
  land_mark_config.distance_for_constraint = config.distance_for_constraint;
  land_mark_config.distance_for_loop = config.distance_for_loop;
  land_mark_config.distance_for_same_reflector =
      config.distance_for_same_reflector;

  land_mark_config.ladar_min_dis = config.mapping_laser_min_range;
  land_mark_config.ladar_max_dis = config.mapping_laser_max_range;

  land_mark_config.data_association_max_dis = config.data_association_max_dis;
  land_mark_config.observation_error = config.observation_error;
  land_mark_config.map_ma_match_dis_threshold =
      config.map_ma_match_dis_threshold;
  land_mark_config.add_new_center_min_dis = config.add_new_center_min_dis;
  land_mark_config.reflector_max_distance = config.reflector_max_distance;
  land_mark_config.distance_diff_threshold = config.distance_diff_threshold;
  land_mark_config.error_threshold_for_loop = config.error_threshold_for_loop;

  mapping_config.laser_resolution = config.mapping_laser_resolution;
  mapping_config.mapping_resolution = config.mapping_resolution;
  mapping_config.mapping_laser_min_range = config.mapping_laser_min_range;
  mapping_config.mapping_laser_max_range = config.mapping_laser_max_range;
  mapping_config.mapping_start_angle = config.mapping_start_angle;
  mapping_config.mapping_end_angle = config.mapping_end_angle;
  mapping_config.map_config_file_path = config.map_config_file_path;
  mapping_config.map_data_file_path = config.map_data_file_path;
  mapping_config.ekf_config = ekf_config;
  mapping_config.record_config = record_config;
  mapping_config.sensor_mount = sensor_mount;
  mapping_config.land_mark_config = land_mark_config;
  mapping_config.land_mark_config.laser_resolution =
      config.mapping_laser_resolution;
  mapping_config.odom_type = config.odom_type;

  config_result->mapping_config = mapping_config;

  config_result->node_name = config.node_name;
  config_result->sub_odom_topic = config.sub_odom_topic;
  config_result->sub_radar_topic = config.sub_radar_topic;
  config_result->sub_imu_topic = config.sub_imu_topic;
  config_result->sub_qr_topic = config.sub_qr_topic;
  config_result->sub_global_locate_cmd_topic =
      config.sub_global_locate_cmd_topic;
  config_result->sub_start_mapping_cmd_topic =
      config.sub_start_mapping_cmd_topic;
  config_result->sub_stop_mapping_cmd_topic = config.sub_stop_mapping_cmd_topic;
  config_result->sub_calibration_cmd_topic = config.sub_calibration_cmd_topic;
  config_result->ad_state_topic = config.ad_state_topic;
  config_result->ad_pose_topic = config.ad_pose_topic;
  config_result->log_file_name = config.log_file_name;
  config_result->log_level = config.log_level;
  config_result->is_printf_to_terminal = config.is_printf_to_terminal;
  config_result->mapping_type =
      (gomros::data_process::mapping_and_location::MappingType)
          config.mapping_type;

  config_result->location_config.ekf_config = ekf_config;
  config_result->location_config.sensor_mount = sensor_mount;
  config_result->location_config.location_start_angle =
      config.location_start_angle;
  config_result->location_config.location_end_angle = config.location_end_angle;
  config_result->location_config.location_laser_resolution =
      config.location_laser_resolution;
  config_result->location_config.use_scan_matching = config.use_scan_matching;
  config_result->location_config.scan_matching_min_distance =
      config.scan_matching_min_distance;
  config_result->location_config.scan_matching_max_distance =
      config.scan_matching_max_distance;
  config_result->location_config.occupied_space_cost_factor =
      config.occupied_space_cost_factor;
  config_result->location_config.translation_delta_cost_factor =
      config.translation_delta_cost_factor;
  config_result->location_config.rotation_delta_cost_factor =
      config.rotation_delta_cost_factor;
  config_result->location_config.num_threads = config.num_threads;
  config_result->location_config.max_num_iterations = config.max_num_iterations;
  config_result->location_config.laser_max_range =
      config.location_laser_max_range;
  config_result->location_config.laser_min_range =
      config.location_laser_min_range;

  config_result->location_config.ekf_config = ekf_config;
  config_result->location_config.sensor_mount = sensor_mount;
  config_result->location_config.land_mark_config = land_mark_config;
  config_result->location_config.land_mark_config.laser_resolution =
      config.location_laser_resolution;

  config_result->location_config.odom_type = config.odom_type;
  config_result->location_config.location_type =
      (LocationType)config.location_type;
  config_result->location_config.init_pose_file_path =
      config.init_pose_file_path;
  config_result->location_config.use_weighted_areas = config.use_weighted_areas;
  config_result->location_config.use_landmark = config.use_landmark;
  config_result->location_config.grid_weight = config.grid_weight;
  config_result->location_config.landmark_weight = config.landmark_weight;
  config_result->location_config.map_data_file_path = config.map_data_file_path;
  return;
}

bool ReadMappingAndLocationConfig(
    const std::string& config_dir,
    gomros::data_process::mapping_and_location::MappingAndLocationConfig*
        config_result) {
  ReadMappingAndConfig temp_config;
  if (ConfigUtils::decode(config_dir.c_str(), temp_config)) {
    MappingAndLocationConfigTransition(temp_config, config_result);
    return true;
  }
  std::cerr << "can not open mapping config file" << std::endl;
  return false;
}

}  // namespace internal_common
