/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-08-06 11:10:15
 * @LastEditTime: 2023-06-29 02:16:13
 * @Author: lcfc-desktop
 */
#pragma once

#include <assert.h>
#include <pthread.h>

#include <list>
#include <memory>

#include "karto_mapping/slam_kar.h"
#include "include/config_struct.h"
#include "mapping/mapping_interface.h"
#include "message_lib/odometer_message.h"
#include "message_lib/position_message.h"
#include "message_lib/radar_message.h"
#include "common_lib/log.h"
#include "common/run_time.h"

#ifndef FLUSH_ODOM_LIST_SIZE
#define FLUSH_ODOM_LIST_SIZE 200
#endif  // FLUSH_ODOM_LIST_SIZE

namespace gomros {
namespace data_process {
namespace mapping_and_location {

class KartoMapping : public MappingInterface {
 public:
  using Position = gomros::message::Position;
  using OdometerMessage = gomros::message::OdometerMessage;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using RadarSensoryInfo = gomros::message::RadarSensoryInfo;
  using Logger = gomros::common::Logger;

  KartoMapping(const MappingConfig &config);
  ~KartoMapping();

  // 处理里程计数据
  virtual void HandleOdomData(const OdometerMessage &data);
  // 处理雷达数据
  virtual void HandleLaserData(const RadarSensoryMessage &data);
  // 开始建图
  virtual void StartMapping();
  // 停止建图
  virtual void StopMapping(const std::string& map_name) override;

 private:
  void AddOdomDataRaw(const Position &pose_change);

  void FlushOdomDataRaw();

  Position ExpolateCurrentPositionTimeRaw(uint64_t timestamp);

  void RecordMapRawData(const RadarSensoryMessage &data);

  FILE *m_pMapRawDataFile;

  std::list<Position> m_OdomListRaw;  // 原始里程计数据
  pthread_mutex_t mutex_odom_list_raw;
  pthread_mutex_t mutex_pose_raw;

  // 一些状态变量
  int m_RecordIndex;
  Position m_MappingOdom;        // 建图里程计
  Position m_LastRecordPose;     // 上一次记录的位姿
  Position m_CurrentRecordPose;  // 当前位姿
  bool m_FirstRecord;            // 是否第一次录制
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
