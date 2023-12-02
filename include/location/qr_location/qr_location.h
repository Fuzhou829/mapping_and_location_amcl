/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-01-14 15:52:31
 * @LastEditTime: 2023-10-26 11:42:21
 * @Author: renjy
 */

#pragma once
#include <map>
#include <memory>
#include "include/qr_struct.h"

#include "message_lib/dmreader_message.h"
#include "message_lib/position_message.h"
#include "ekf_calcuator/ekf_calcuator.h"
#include "include/config_struct.h"
namespace gomros {
namespace data_process {
namespace mapping_and_location {

class QrLocation {
 public:
  using Position = gomros::message::Position;
  using DMreaderSensoryMessage = gomros::message::DMreaderSensoryMessage;
  using ImuSensoryMessage = gomros::message::ImuSensoryMessage;
  using OdometerMessage = gomros::message::OdometerMessage;

 public:
  explicit QrLocation(const LocationConfig& config) {
    pthread_mutex_init(&robot_pos_mutex_, nullptr);
    config_ = config;
    ekf_cal_ =
      std::make_shared<DataFusion::EKFCalcuator>(config_.ekf_config, "qr_ekf");
    last_qr_id_ = -1;
  }
  ~QrLocation() {}
  void SetQrMap(const std::map<int, QRCoordinate>& qr_map) {
    qr_map_ = qr_map;
  }
  void ClearImuData() {
    ekf_cal_->ClearImuData();
  }
  void HandleOdomData(const OdometerMessage& data);
  void HandleImuData(const ImuSensoryMessage& data);
  bool HandleQrData(const DMreaderSensoryMessage& data, Position* qr_pos);
  void SetInitPos(const Position& pos);
  bool GetCurrentPose(Position* current_pose);

 private:
  Position current_pose_;
  pthread_mutex_t robot_pos_mutex_;

  std::map<int, QRCoordinate> qr_map_;
  int last_qr_id_;
  std::shared_ptr<DataFusion::EKFCalcuator> ekf_cal_;
  LocationConfig config_;
};



}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
