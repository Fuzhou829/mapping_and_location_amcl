/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-10-28 14:32:05
 * @LastEditTime: 2023-08-03 15:15:07
 */

#include "reflector_mapping/reflector_mapping.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

ReflectorMapping::ReflectorMapping(const MappingConfig& config)
: MappingInterface(config.record_config) {
  mapping_config_ = config;
  // mapping_config_.record_config.angular_step = 0.0;
  // mapping_config_.record_config.distance_step = 0.0;
  // SLAM_INFO("reflector change record data step %f %f",
  //           mapping_config_.record_config.angular_step,
  //           mapping_config_.record_config.distance_step);
  SetMappingConfig(mapping_config_);
  reflector_map_builder_ =
    std::make_shared<ReflectorMapBuilder>(mapping_config_, 0);
}


void ReflectorMapping::StartMapping() {
  sensor_data_recorder_->StartRecord();
}

void ReflectorMapping::StopMapping(const std::string& map_name) {
  sensor_data_recorder_->FinishRecord();
  // 开始离线建图
  is_record_last_odom_ = false;
  _startReflectorMapping(map_name);
}

void ReflectorMapping::_startReflectorMapping(const std::string& map_name) {
  reflector_map_builder_->Init();
  // 打开记录数据信息
  _openOdomAndImuFile();
  std::ifstream ladar_file;
  ladar_file.open(sensor_data_recorder_->GetLadarFileDir());
  std::string ladar_line;
  while (getline(ladar_file, ladar_line)) {
    std::vector<std::string> ladar_data =
      internal_common::SplitCString(ladar_line, " ");
    RadarSensoryInfo ladar_message;
    _transform2RadarSensoryInfo(ladar_data, &ladar_message);
    _dealOdomAndImuMessage(ladar_message.mstruRadarHeaderData.mlTimeStamp);
    reflector_map_builder_->DispatchLadarData(ladar_message);
  }
  reflector_map_builder_->FinishTrajectory();
  reflector_map_builder_->SaveToFile(map_name);
  _closeOdomAndImuFile();
  return;
}


void ReflectorMapping::_openOdomAndImuFile() {
  odom_file_.open(sensor_data_recorder_->GetOdomFileDir());
  imu_file_.open(sensor_data_recorder_->GetImuFileDir());
}

void ReflectorMapping::_closeOdomAndImuFile() {
  odom_file_.close();
  imu_file_.close();
}

void ReflectorMapping::_transform2RadarSensoryInfo(
  const std::vector<std::string>& ladar_str, RadarSensoryInfo* ladar_message) {
  ladar_message->mstruRadarHeaderData.mlTimeStamp = atof(ladar_str[0].c_str());
  ladar_message->mstruSingleLayerData.mvPoints.clear();
  ladar_message->mstruSingleLayerData.mvIntensities.clear();
  float theta_acc = mapping_config_.mapping_end_angle -
                    mapping_config_.mapping_start_angle;
  int record_count = theta_acc / mapping_config_.laser_resolution + 1;
  int read_id = 2;
  float xp, yp;
  float theta = mapping_config_.mapping_start_angle * M_PI / 180.0f;
  float delta_theta = mapping_config_.laser_resolution * M_PI / 180.0f;
  for (int i = 0; i < record_count; i++) {
    float len = atof(ladar_str[read_id++].c_str());
    xp = len * cos(theta);
    yp = len * sin(theta);
    theta += delta_theta;
    Eigen::Vector2d temp(xp, yp);
    ladar_message->mstruSingleLayerData.mvPoints.push_back(temp);
    ladar_message->mstruSingleLayerData.mvIntensities.push_back(
                                          atof(ladar_str[read_id++].c_str()));
  }
  return;
}

void ReflectorMapping::_dealOdomAndImuMessage(uint64_t time_stemp) {
  std::string imu_line;
  while (getline(imu_file_, imu_line)) {
    ImuSensoryMessage imu_msg;
    std::vector<std::string> imu_data =
        internal_common::SplitCString(imu_line, " ");
    imu_msg.time_stamp = atof(imu_data[0].c_str());
    imu_msg.z_omega = atof(imu_data[1].c_str());
    imu_msg.z_angle = atof(imu_data[2].c_str());
    imu_msg.forward_linear_accel = atof(imu_data[3].c_str());
    // reflector_map_builder_->DispatchImuData(imu_msg);
    if (imu_msg.time_stamp > time_stemp) break;
  }

  std::string odom_line;
  if (is_record_last_odom_) {
    is_record_last_odom_ = false;
    reflector_map_builder_->DispatchOdomData(last_odom_massage_);
  }
  while (getline(odom_file_, odom_line)) {
    // ekf 预测
    OdometerMessage odom_message;
    if (!internal_common::GetOdomMessage(odom_line,
        (OdomType)mapping_config_.odom_type, &odom_message))
      continue;
    if (odom_message.mclDeltaPosition.mlTimestamp > time_stemp) {
      is_record_last_odom_ = true;
      last_odom_massage_ = odom_message;
      break;
    }
    reflector_map_builder_->DispatchOdomData(odom_message);
  }
  return;
}


}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
