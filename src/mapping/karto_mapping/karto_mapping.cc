/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-08-06 11:11:30
 * @LastEditTime: 2023-06-29 11:11:00
 * @Author: lcfc-desktop
 */

#include <fstream>
#include <string>
#include <vector>
#include <memory>

#include "karto_mapping/karto_mapping.h"
#include "common_lib/time_utils.h"
#include "karto_mapping/slam_kar.h"
#include "common/logger.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

KartoMapping::KartoMapping(const MappingConfig &config)
: MappingInterface(config.record_config) {
  mapping_config_ = config;
  pthread_mutex_init(&mutex_pose_raw, nullptr);
  pthread_mutex_init(&mutex_odom_list_raw, nullptr);
}

KartoMapping::~KartoMapping() {
  pthread_mutex_destroy(&mutex_pose_raw);
  pthread_mutex_destroy(&mutex_odom_list_raw);
}


void KartoMapping::HandleOdomData(const OdometerMessage &data) {
  AddOdomDataRaw(data.mclDeltaPosition);
  if (m_OdomListRaw.size() > FLUSH_ODOM_LIST_SIZE) {
    FlushOdomDataRaw();
  }
}

void KartoMapping::HandleLaserData(const RadarSensoryMessage &data) {
  if (mapping_config_.mapping_pattern == MappingPattern::Offline) {
    m_CurrentRecordPose = ExpolateCurrentPositionTimeRaw(
        data.mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp);
    RecordMapRawData(data);
    // // SLAM_INFO("录制雷达数据\n");
  } else if (mapping_config_.mapping_pattern == MappingPattern::Online) {
    // // SLAM_INFO("在线模式无法录制雷达数据\n");
  }
}

void KartoMapping::StartMapping() {
  sensor_data_recorder_->StartRecord();
  m_RecordIndex = 1;
  pthread_mutex_lock(&mutex_odom_list_raw);
  m_OdomListRaw.clear();
  pthread_mutex_unlock(&mutex_odom_list_raw);
  m_MappingOdom.mfX = 0;
  m_MappingOdom.mfY = 0;
  m_MappingOdom.mfTheta = 0;
  m_LastRecordPose = m_MappingOdom;
  m_CurrentRecordPose = m_MappingOdom;
  std::string command;
  command = "mkdir -p " + mapping_config_.map_data_file_path;  
  system(command.c_str());
  std::string raw_map_file = mapping_config_.map_data_file_path + "/map.rawmap";
  m_pMapRawDataFile = fopen(raw_map_file.c_str(), "w+");
  usleep(1500000);  // 等待文件打开完成
  m_FirstRecord = true;
}

void KartoMapping::StopMapping(const std::string& map_name) {
  sensor_data_recorder_->FinishRecord();
  fclose(m_pMapRawDataFile);
  usleep(30000);
  // SLAM_INFO("begin karto mapping");
  internal_common::RunTime runtime;
  std::vector<char> map_data1;
  // 基于karto算法离线建立地图
  std::shared_ptr<SlamKarto> karto_mapping =
    std::make_shared<SlamKarto>(mapping_config_);
  karto_mapping->begin_slam(&map_data1, map_name);
  usleep(30000);
  if (!karto_mapping->map_successed) {
    // SLAM_ERROR("Karto 离线建图失败！\n");
  }
  // SLAM_INFO("end karto mapping");
}

/**
 * @brief 录制一次数据
 *
 * @param data
 */
void KartoMapping::RecordMapRawData(const RadarSensoryMessage &data) {
  double xp, yp, laser_len;
  if (fabs(m_LastRecordPose.mfX - m_CurrentRecordPose.mfX) >
          mapping_config_.record_config.distance_step ||
      fabs(m_LastRecordPose.mfY - m_CurrentRecordPose.mfY) >
          mapping_config_.record_config.distance_step ||
      fabs(m_LastRecordPose.mfTheta - m_CurrentRecordPose.mfTheta) >
          mapping_config_.record_config.angular_step ||
      m_FirstRecord) {
    fprintf(m_pMapRawDataFile, "%d\t%ld\t%ld\t", m_RecordIndex,
            m_CurrentRecordPose.mlTimestamp, m_LastRecordPose.mlTimestamp);
    if (m_FirstRecord) {
      m_FirstRecord = false;
    }
    float laser_min_angle = mapping_config_.mapping_start_angle;
    float laser_max_angle = mapping_config_.mapping_end_angle;
    float laser_resolution = mapping_config_.laser_resolution;
    int record_cnt =
        fabs((laser_max_angle - laser_min_angle) / laser_resolution);

    int range_count =
        fabs((data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMax -
              data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMin) /
             data.mstruRadarMessage.mstruRadarHeaderData.mfAngleIncreament);

    float angle_increment =
        data.mstruRadarMessage.mstruRadarHeaderData.mfAngleIncreament;
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
            data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMin ||
        (laser_max_angle / 180 * M_PI) >
            data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMax) {
      // SLAM_ERROR("配置文件配置的雷达角度范围超出了雷达数据角度范围!\n");
      return;
    }
    int start_i = fabs(laser_min_angle / 180 * M_PI -
                       data.mstruRadarMessage.mstruRadarHeaderData.mfAngleMin) /
                  angle_increment;
    record_cnt = record_cnt * stepIncreament;

    fprintf(m_pMapRawDataFile, "%f\t%f\t%f\t%d\t", m_CurrentRecordPose.mfX,
            m_CurrentRecordPose.mfY, m_CurrentRecordPose.mfTheta, record_cnt);
    for (int i = start_i; i <= (start_i + record_cnt); i += stepIncreament) {
      xp = data.mstruRadarMessage.mstruSingleLayerData.mvPoints[i](0);
      yp = data.mstruRadarMessage.mstruSingleLayerData.mvPoints[i](1);
      laser_len = sqrt(xp * xp + yp * yp);
      fprintf(m_pMapRawDataFile, "%f,", laser_len);
    }
    fprintf(m_pMapRawDataFile, "\n");
    m_RecordIndex++;
    m_LastRecordPose = m_CurrentRecordPose;
  }
}

void KartoMapping::AddOdomDataRaw(const Position &pose_change) {
  uint64_t timestamp = pose_change.mlTimestamp;
  pthread_mutex_lock(&mutex_pose_raw);
  if (timestamp < m_MappingOdom.mlTimestamp) {
    // SLAM_WARN("当前里程计数据时间戳小于最新里程计位姿时间戳\n");
  }
  pthread_mutex_lock(&mutex_odom_list_raw);
  if (m_OdomListRaw.empty()) {
    m_OdomListRaw.push_back(pose_change);
  } else {
    for (auto iter = m_OdomListRaw.begin();; iter++) {
      if (iter == m_OdomListRaw.end() || iter->mlTimestamp >= timestamp) {
        m_OdomListRaw.insert(iter, pose_change);
        break;
      }
    }
  }
  pthread_mutex_unlock(&mutex_odom_list_raw);
  pthread_mutex_unlock(&mutex_pose_raw);
}

void KartoMapping::FlushOdomDataRaw() {
  pthread_mutex_lock(&mutex_pose_raw);
  Position pose = m_MappingOdom;
  pthread_mutex_lock(&mutex_odom_list_raw);
  while (!m_OdomListRaw.empty()) {
    pose = pose * m_OdomListRaw.front();
    m_OdomListRaw.pop_front();
  }
  m_MappingOdom = pose;

  pthread_mutex_unlock(&mutex_odom_list_raw);
  pthread_mutex_unlock(&mutex_pose_raw);
}

/**
 * @brief 将建图里程计的位姿根据里程计数据预测到timestamp时刻
 *
 * @param timestamp
 * @return gomros::message::Position
 */
gomros::message::Position KartoMapping::ExpolateCurrentPositionTimeRaw(
    uint64_t timestamp) {
  pthread_mutex_lock(&mutex_pose_raw);
  Position ret = m_MappingOdom;
  pthread_mutex_lock(&mutex_odom_list_raw);
  for (Position &p : m_OdomListRaw) {
    if (p.mlTimestamp > ret.mlTimestamp && (p.mlTimestamp <= timestamp)) {
      ret = ret * p;
    }
  }
  m_MappingOdom = ret;
  while (!m_OdomListRaw.empty()) {
    if (m_OdomListRaw.front().mlTimestamp > timestamp) {
      break;
    }
    m_OdomListRaw.pop_front();
  }
  pthread_mutex_unlock(&mutex_odom_list_raw);
  pthread_mutex_unlock(&mutex_pose_raw);
  return ret;
}

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
