/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-10 10:02:47
 * @LastEditTime: 2023-10-16 22:17:24
 */
#pragma once
#include <unistd.h>

#include <string>
#include <fstream>
#include <vector>
#include <map>
#include "common_lib/node.h"
#include "common/logger.h"

#include "message_lib/odometer_message.h"
#include "message_lib/radar_message.h"
#include "message_lib/dmreader_message.h"
#include "message_lib/imu_message.h"
#include "message_lib/cmd_message.h"
#include "message_lib/device_state.h"

#include "common_lib/message.h"

#include "include/config_struct.h"
#include "ekf_calcuator/ekf_calcuator.h"

/**
 * @brief: 模拟数据发布
 */

namespace Simulation {

using namespace gomros::data_process::mapping_and_location;  // NOLINT

enum MessageType {
  LADAR,
  ODOM,
  IMU,
  QR
};

struct MessageInfo {
  MessageType message_type;
  std::vector<std::string> message_info;
};


class SensorData {
 public:
  using Node = gomros::common::Node;
  using RawPublisher = gomros::common::RawPublisher;
  using OdometerMessage = gomros::message::OdometerMessage;
  using Position = gomros::message::Position;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using DMreaderSensoryMessage = gomros::message::DMreaderSensoryMessage;
  using ImuSensoryMessage = gomros::message::ImuSensoryMessage;
  using DeviceState = gomros::message::DeviceState;
  using CustomTaskString = gomros::message::CustomTaskString;
  using CallBackEvent = gomros::common::CallBackEvent;

 public:
  SensorData(const MappingAndLocationConfig& config,
                      const std::string& data_file_dir,
                      bool is_mapping = true);
  virtual ~SensorData() {}


  bool IsFinishSimulation();


 private:
  static void* _publisherThread(void* ptr);
  static void _robotState(void* object, char* buf, const int size);
  static void _robotPose(void *object, char *buf, const int size);
  void _loadMessageFromFile();
  /**
   * @brief: 发布消息
   * @return {*}
   */  
  void _odomPublish(const OdometerMessage& message);
  void _ladarPublish(RadarSensoryMessage message);
  void _imuPublish(const ImuSensoryMessage& message);
  void _qrPublish(const DMreaderSensoryMessage& message);

  void _calRobotVel(OdometerMessage* odom_message);
  ImuSensoryMessage _getImuMessage(const std::vector<std::string>& line);
  bool _getLadarMessage(const std::vector<std::string>& line,
    RadarSensoryMessage* message);
  DMreaderSensoryMessage _getQrMessage(const std::vector<std::string>& line);

 private:
  Node* node_;
  RawPublisher* odom_publisher_;
  RawPublisher* imu_publisher_;
  RawPublisher* qr_publisher_;
  RawPublisher* ladar_publisher_;
  RawPublisher* last_pos_publisher_;

  RawPublisher* start_mapping_publisher_;
  RawPublisher* end_mapping_publisher_;
  std::map<uint64_t, MessageInfo> message_infos_;

  bool is_mapping_;
  pthread_t thread_;
  std::string data_file_dir_;
  MappingAndLocationConfig config_;
  Eigen::Matrix<float, 4, 3> config_mat_;  // 舵轮配置矩阵
  bool is_in_real_time_location_;
  bool is_finish_publish_;
};



}  // namespace Simulation
