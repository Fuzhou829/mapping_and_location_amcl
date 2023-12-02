/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-14 15:03:31
 * @LastEditTime: 2023-09-04 22:46:54
 */
#pragma once
#include <string>
#include <fstream>
#include <list>
#include <mutex>  // NOLINT
#include <map>
#include <memory>

#include "message_lib/odometer_message.h"
#include "message_lib/radar_message.h"
#include "message_lib/dmreader_message.h"
#include "message_lib/imu_message.h"

#include "include/config_struct.h"

#include "common/tool.h"

#ifdef TEST_DEBUG
#include "ekf_calcuator/ekf_calcuator.h"
#endif
namespace Record {

using Position = gomros::message::Position;
using OdometerMessage = gomros::message::OdometerMessage;
using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
using RadarSensoryInfo = gomros::message::RadarSensoryInfo;
using DMreaderSensoryMessage = gomros::message::DMreaderSensoryMessage;
using ImuSensoryMessage = gomros::message::ImuSensoryMessage;

using namespace gomros::data_process::mapping_and_location;  // NOLINT

/**
 * @brief: 用于记录离线数据
 */
class SensorData {
 public:
  SensorData(const RecordConfig& config,
             const std::string& file_dir = "./mapping_data/");
  virtual ~SensorData() {
    // pthread_mutex_destroy(&record_mutex_);
  }
  void SetRecordConfig(const RecordConfig& config);
  void StartRecord();
  void FinishRecord();

  void RecordLadarData(const RadarSensoryMessage& message, bool is_move);
  void RecordOdomData(const OdometerMessage& message);
  void RecordImuData(const ImuSensoryMessage& message);
  void RecordQrData(const DMreaderSensoryMessage& message);

  std::string GetLadarFileDir() const {
    return ladar_file_dir_;
  }
  std::string GetOdomFileDir() const {
    return odom_file_dir_;
  }
  std::string GetImuFileDir() const {
    return imu_file_dir_;
  }
  std::string GetQrFileDir() const {
    return qr_file_dir_;
  }

 private:
  // 增加雷达数据写入线程
  static void* _ladarDataWriteThread(void* ptr);
  void _recordLadarData(const RadarSensoryMessage& message);
  // void _recordLadarDataForFusionTest(
  // const RadarSensoryInfo& message, FILE* file);
  void _recordOdomDataByType(const OdometerMessage& message);
  void _recordLadarDataToFile(const RadarSensoryInfo& message, FILE* file);
  void _recordScanMatchLadar();
  void _insertLadarMessage(const RadarSensoryInfo& message);

 private:
  struct RecordForScanMatch {
    // qr info
    int qr_num;
    uint64_t qr_time_stamp;
    float qr_x;
    float qr_y;
    float qr_theta;
    bool is_get_qr;
    // ladar info
    RadarSensoryInfo radar_info;
    uint64_t GetTimeGap() {
      uint64_t delta_time =
        radar_info.mstruRadarHeaderData.mlTimeStamp > qr_time_stamp ?
        radar_info.mstruRadarHeaderData.mlTimeStamp - qr_time_stamp :
        qr_time_stamp - radar_info.mstruRadarHeaderData.mlTimeStamp;
      return delta_time;
    }
  };

  std::string ladar_file_dir_;
  std::string odom_file_dir_;
  std::string imu_file_dir_;
  std::string qr_file_dir_;
  std::string save_dir_;

  FILE* ladar_file_;
  FILE* odom_file_;
  FILE* imu_file_;
  FILE* qr_file_;
  FILE* temp_file;

  bool is_start_record_;  // true:第一次记录数据
  bool need_add_record_;  // true；需要增加一次记录

  std::map<int, RecordForScanMatch> ladar_record_scan_match_;
  RecordForScanMatch last_record_for_scan_match_;
  RecordConfig config_;

  // 大致估计当前的位姿 里程计累计
  Position delta_pos_;

  std::list<RadarSensoryInfo> radar_sensor_info_;
  RadarSensoryInfo last_ladar_data_;
  std::recursive_mutex last_ladar_mutex_;
  std::recursive_mutex record_mutex_;
  std::recursive_mutex odom_mutex_;
  pthread_mutex_t for_scan_match_mutex_;
  // pthread_mutex_t record_mutex_;
  pthread_t record_ladar_thread_;
  bool is_stop_record_data_;
  // 原始里程计数据集合（最近一段时间）
  std::list<Position> odom_list_;
  // 上一次机器获取到的qr id
  int previous_tag_num_;
  int record_ladar_count_;
};




}  // namespace Record
