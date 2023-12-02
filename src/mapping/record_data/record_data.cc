/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-14 16:40:27
 * @LastEditTime: 2023-10-11 22:30:10
 */

#include "record_data/record_data.h"
#include "common/logger.h"

#ifdef TEST_DEBUG
#include "common/load_config.h"
#endif

namespace Record {


SensorData::SensorData(const RecordConfig& config, const std::string& file_dir)
:config_(config) {
  pthread_mutex_init(&for_scan_match_mutex_, nullptr);
  save_dir_ = file_dir;
  std::string command;
  command = "rm -rf " + save_dir_;
  system(command.c_str());
  command = "mkdir -p " + save_dir_;
  system(command.c_str());

  usleep(2000);
  ladar_file_dir_ = save_dir_ + "ladar_message.smap";
  odom_file_dir_ = save_dir_ + "odom_message.txt";
  imu_file_dir_ = save_dir_ + "imu_message.txt";
  qr_file_dir_ = save_dir_ + "qr_message.txt";
  is_start_record_ = false;
  need_add_record_ = false;
  delta_pos_.mfX = 0.0;
  delta_pos_.mfY = 0.0;
  delta_pos_.mfTheta = 0.0;
  previous_tag_num_ = -1;
  is_stop_record_data_ = true;
  record_ladar_count_ = 0;
}

void SensorData::SetRecordConfig(const RecordConfig& config) {
  config_ = config;
}


void SensorData::StartRecord() {
  SLAM_INFO("begin record data~~~~");
  std::string command;
  command = "rm -rf " + save_dir_ + "*";
  system(command.c_str());
  usleep(20000);
  ladar_file_ = fopen(ladar_file_dir_.c_str(), "w+");
  imu_file_ = fopen(imu_file_dir_.c_str(), "w+");
  qr_file_ = fopen(qr_file_dir_.c_str(), "w+");
  odom_file_ = fopen(odom_file_dir_.c_str(), "w+");
  temp_file = fopen("./compare.txt", "w+");
  usleep(20000);
  is_start_record_ = true;
  need_add_record_ = false;
  ladar_record_scan_match_.clear();
  last_record_for_scan_match_.is_get_qr = false;
  delta_pos_.mfX = 0.0;
  delta_pos_.mfY = 0.0;
  delta_pos_.mfTheta = 0.0;
  previous_tag_num_ = -1;
  is_stop_record_data_ = false;
  record_ladar_count_ = 0;
  pthread_create(&record_ladar_thread_, nullptr, _ladarDataWriteThread, this);
  // SLAM_INFO("create pthread successful!!!!!!!!!!!!");
}

void SensorData::FinishRecord() {
  SLAM_INFO("finish record");
  is_stop_record_data_ = true;
  is_start_record_ = false;
  usleep(6000000);
  _recordScanMatchLadar();
  {
    std::lock_guard<std::recursive_mutex> lk(odom_mutex_);
    fclose(qr_file_);
    fclose(odom_file_);
    fclose(imu_file_);
  }
  while (!radar_sensor_info_.empty()) {}
  usleep(1500000);
}

void SensorData::RecordLadarData(
  const RadarSensoryMessage& message, bool is_move) {
  if (!is_start_record_) return;

#ifndef TEST_DEBUG
  // if (config_.mapping_type == MappingType::Trilateral_Mapping &&
  //     is_move && !is_stop_record_data_)
  //   return;
  if (is_stop_record_data_ && !config_.is_need_record_data) return;
  if (config_.mapping_type == MappingType::Trilateral_Mapping && is_move &&
      !config_.is_need_record_data) {
    is_stop_record_data_ = true;
    SLAM_INFO("Trilateral_Mapping finished record data!");
  }
  if (config_.mapping_type == MappingType::Trilateral_Mapping &&
      record_ladar_count_ > 50 &&
      !config_.is_need_record_data) {
    is_stop_record_data_ = true;
    SLAM_INFO("we have enough ladar data for mapping");
  }
#endif
  if (config_.mapping_type == MappingType::Trilateral_Mapping &&
      !config_.is_need_record_data) {
    // 只记录静止的数据
    std::lock_guard<std::recursive_mutex> lk(record_mutex_);
    radar_sensor_info_.push_back(message.mstruRadarMessage);
    record_ladar_count_++;
    return;
  }
  {
    std::lock_guard<std::recursive_mutex> lk(last_ladar_mutex_);
    last_ladar_data_ = message.mstruRadarMessage;
  }

  _insertLadarMessage(message.mstruRadarMessage);
  _recordLadarData(message);
  return;
}


void SensorData::RecordOdomData(const OdometerMessage& message) {
  if (!is_start_record_) return;
  _recordOdomDataByType(message);
  std::lock_guard<std::recursive_mutex> lk(odom_mutex_);
  if (odom_list_.empty()) {
    odom_list_.push_back(message.mclDeltaPosition);
  } else {
    for (auto iter = odom_list_.begin();; iter++) {
      if (iter == odom_list_.end() ||
          iter->mlTimestamp > message.mclDeltaPosition.mlTimestamp) {
          odom_list_.insert(iter, message.mclDeltaPosition);
          break;
        }
    }
  }
  return;
}

void SensorData::RecordImuData(const ImuSensoryMessage& message) {
  if (!is_start_record_) return;
  fprintf(imu_file_, "%ld %f %f %f\n",
    message.time_stamp, message.z_omega, message.z_angle,
    message.forward_linear_accel);
}

void SensorData::RecordQrData(const DMreaderSensoryMessage& message) {
  if (!is_start_record_) return;
  if (!message.read_success) return;
  pthread_mutex_lock(&for_scan_match_mutex_);
  last_record_for_scan_match_.qr_num = message.tag_num;
  last_record_for_scan_match_.qr_time_stamp = message.time_stamp;
  last_record_for_scan_match_.is_get_qr = true;
  last_record_for_scan_match_.qr_x = message.x;
  last_record_for_scan_match_.qr_y = message.y;
  last_record_for_scan_match_.qr_theta = message.theta;
  pthread_mutex_unlock(&for_scan_match_mutex_);

  {
    std::lock_guard<std::recursive_mutex> lk(last_ladar_mutex_);
    _insertLadarMessage(last_ladar_data_);
  }
  // if (message.tag_num == previous_tag_num_ &&
  //     config_.mapping_type != MappingType::Optimize_Mapping) {
  //   return;
  // }
  need_add_record_ = true;
  previous_tag_num_ = message.tag_num;
  fprintf(qr_file_, "%ld %f %f %f %d %d\n", message.time_stamp,
          message.x, message.y, message.theta, message.tag_num,
          message.read_success);
  return;
}


void* SensorData::_ladarDataWriteThread(void* ptr) {
  pthread_detach(pthread_self());
  SensorData *lp_this = reinterpret_cast<SensorData*>(ptr);
  SLAM_INFO("enter ladar data write thread<<<<<<<<<<<");
  while (1) {
    usleep(20000);
    RadarSensoryInfo radar_sensory_info;
    {
      std::lock_guard<std::recursive_mutex> lk(lp_this->record_mutex_);
      if (lp_this->is_stop_record_data_ &&
          lp_this->radar_sensor_info_.empty()) {
        fclose(lp_this->ladar_file_);
        usleep(1500000);
        SLAM_INFO("finished record ladar data");
        break;
      }
      if (lp_this->radar_sensor_info_.empty()) {
        continue;
      }
      radar_sensory_info = lp_this->radar_sensor_info_.front();
      lp_this->radar_sensor_info_.pop_front();
    }
    lp_this->_recordLadarDataToFile(radar_sensory_info, lp_this->ladar_file_);
  }
  SLAM_INFO("exit ladar data write thread<<<<<<<<<<<");
  pthread_exit(NULL);
}

void SensorData::_recordLadarData(const RadarSensoryMessage& message) {
  bool is_test = false;
#ifdef TEST_DEBUG
  is_test = true;
#endif
  if (config_.is_need_record_data || is_test) {
    std::lock_guard<std::recursive_mutex> lk(record_mutex_);
    radar_sensor_info_.push_back(message.mstruRadarMessage);
    return;
  }
  // 更新当前位姿
  std::lock_guard<std::recursive_mutex> luck(odom_mutex_);
  while (!odom_list_.empty()) {
    delta_pos_.mfX += fabs(odom_list_.front().mfX);
    delta_pos_.mfY += fabs(odom_list_.front().mfY);
    delta_pos_.mfTheta += fabs(odom_list_.front().mfTheta);
    delta_pos_.mlTimestamp = odom_list_.front().mlTimestamp;
    odom_list_.pop_front();
    if (delta_pos_.mlTimestamp >
      message.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp) break;
  }

  if (!need_add_record_) {
    if ((fabs(delta_pos_.mfX) <= config_.distance_step &&
        fabs(delta_pos_.mfY) <= config_.distance_step &&
        fabs(delta_pos_.mfTheta) <= config_.angular_step)) {
      return;
    } else {
      delta_pos_.mfX = 0;
      delta_pos_.mfY = 0;
      delta_pos_.mfTheta = 0;
      delta_pos_.mlTimestamp = 0;
    }
  } else {
    need_add_record_ = false;
  }
  {
    std::lock_guard<std::recursive_mutex> lk(record_mutex_);
    radar_sensor_info_.push_back(message.mstruRadarMessage);
  }
  return;
}


void SensorData::_recordOdomDataByType(const OdometerMessage& message) {
  if (config_.odom_type == OdomType::DiffWheelModel) {
    fprintf(odom_file_, "%ld %f %f\n",
            message.mclDeltaPosition.mlTimestamp,
            message.mstruDiffSteerSts.mfLeftLinearVel,
            message.mstruDiffSteerSts.mfRightLinearVel);
  } else if (config_.odom_type == OdomType::SingleSteerModel) {
    fprintf(odom_file_, "%ld %f %f %f\n",
            message.mclDeltaPosition.mlTimestamp,
            message.mstruSingleSteerSts.mdLinearVel,
            message.mstruSingleSteerSts.mdAngularVel,
            message.mstruSingleSteerSts.mdRotAngle);
  } else if (config_.odom_type == OdomType::DoubleSteerModel) {
    fprintf(odom_file_, "%ld %f %f %f %f %f %f\n",
            message.mclDeltaPosition.mlTimestamp,
            message.mstruDualSteerSts.mdFrontLinearVel,
            message.mstruDualSteerSts.mdBackLinearVel,
            message.mstruDualSteerSts.mdFrontAngularVel,
            message.mstruDualSteerSts.mdBackAngularVel,
            message.mstruDualSteerSts.mdFrontRotAngle,
            message.mstruDualSteerSts.mdBackRotAngle);
  }
  return;
}

void SensorData::_recordLadarDataToFile(
  const RadarSensoryInfo& radar_sensory_info, FILE* file) {
  float laser_min_angle = config_.mapping_start_angle;
  float laser_max_angle = config_.mapping_end_angle;
  float laser_resolution = config_.laser_resolution;
  int record_cnt = fabs((laser_max_angle - laser_min_angle) / laser_resolution);

  float angle_increment =
    radar_sensory_info.mstruRadarHeaderData.mfAngleIncreament;
  if ((laser_resolution - angle_increment) < 1e-6) {
    // SLAM_ERROR("配置文件需要的雷达分辨率比雷达的实际分辨率小\n");
  }

  int stepIncreament =
      std::round(laser_resolution / 180 * M_PI / angle_increment);
  float float_stepIncreament =
      laser_resolution / 180 * M_PI / angle_increment;
  if (fabs(float_stepIncreament - stepIncreament) > 1e-6) {
    // SLAM_ERROR("配置文件需要的雷达分辨率必须是雷达原分辨率的整数倍!\n");
  }

  angle_increment = fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI;

  if ((laser_min_angle / 180 * M_PI) <
          radar_sensory_info.mstruRadarHeaderData.mfAngleMin ||
      (laser_max_angle / 180 * M_PI) >
          radar_sensory_info.mstruRadarHeaderData.mfAngleMax) {
    // SLAM_ERROR("配置文件配置的雷达角度范围超出了雷达数据角度范围!\n");
    return;
  }
  int start_i = fabs(laser_min_angle / 180 * M_PI -
                      radar_sensory_info.mstruRadarHeaderData.mfAngleMin) /
                angle_increment;
  record_cnt = record_cnt * stepIncreament;

  fprintf(file, "%ld %d",
          radar_sensory_info.mstruRadarHeaderData.mlTimeStamp,
          record_cnt);
  double xp, yp, laser_len;
#ifdef TEST_DEBUG
  start_i = 0;
  record_cnt = radar_sensory_info.mstruSingleLayerData.mvPoints.size() - 1;
  stepIncreament = 1;
#endif
  static int record_count = 0;
  if (record_count == 0) {
    fprintf(temp_file, "%ld\n",
          radar_sensory_info.mstruRadarHeaderData.mlTimeStamp);
  }
  for (int i = start_i; i <= (start_i + record_cnt); i += stepIncreament) {
    xp = radar_sensory_info.mstruSingleLayerData.mvPoints[i](0);
    yp = radar_sensory_info.mstruSingleLayerData.mvPoints[i](1);
    laser_len = sqrt(xp * xp + yp * yp);
    fprintf(file, " %f %f", laser_len,
            radar_sensory_info.mstruSingleLayerData.mvIntensities[i]);
    if (record_count == 0) {
      fprintf(temp_file, "%f %f\n", xp, yp);
    }
  }
  if (record_count == 0) {
    fclose(temp_file);
  }
  record_count++;
  fprintf(file, "\n");
  return;
}


void SensorData::_recordScanMatchLadar() {
  std::string scan_match_ladar_dir =  "./mapping_data/scan_match_ladar.txt";
  FILE* scan_match_ladar_file = fopen(scan_match_ladar_dir.c_str(), "w+");

  for (auto iter = ladar_record_scan_match_.begin();
      iter != ladar_record_scan_match_.end(); iter++) {
      fprintf(scan_match_ladar_file, "%ld %f %f %f %d ",
              iter->second.qr_time_stamp,
              iter->second.qr_x, iter->second.qr_y, iter->second.qr_theta,
              iter->second.qr_num);
      _recordLadarDataToFile(iter->second.radar_info, scan_match_ladar_file);
  }
  fclose(scan_match_ladar_file);
}

void SensorData::_insertLadarMessage(const RadarSensoryInfo& message) {
  if (config_.is_need_record_data) return;
  pthread_mutex_lock(&for_scan_match_mutex_);
  if (last_record_for_scan_match_.is_get_qr) {
    last_record_for_scan_match_.radar_info = message;
    if (ladar_record_scan_match_.count(last_record_for_scan_match_.qr_num)) {
      RecordForScanMatch data =
        ladar_record_scan_match_.at(last_record_for_scan_match_.qr_num);
      if (data.GetTimeGap() > last_record_for_scan_match_.GetTimeGap()) {
        ladar_record_scan_match_.at(last_record_for_scan_match_.qr_num) =
        last_record_for_scan_match_;
      }
  } else {
      ladar_record_scan_match_.insert(std::make_pair(
        last_record_for_scan_match_.qr_num, last_record_for_scan_match_));
    }
    last_record_for_scan_match_.is_get_qr = false;
    pthread_mutex_unlock(&for_scan_match_mutex_);
    return;
  } else {
    for (auto iter = ladar_record_scan_match_.begin();
        iter != ladar_record_scan_match_.end(); iter++) {
        uint64_t delta_time =
          iter->second.qr_time_stamp >
          message.mstruRadarHeaderData.mlTimeStamp ?
          iter->second.qr_time_stamp -
          message.mstruRadarHeaderData.mlTimeStamp :
          message.mstruRadarHeaderData.mlTimeStamp -
          iter->second.qr_time_stamp;
        if (iter->second.GetTimeGap() > delta_time) {
          iter->second.radar_info = message;
        }
    }
  }
  pthread_mutex_unlock(&for_scan_match_mutex_);
  return;
}


}  // namespace Record
