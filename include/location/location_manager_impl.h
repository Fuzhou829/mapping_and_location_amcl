/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-08-18 09:21:18
 * @LastEditTime: 2023-10-27 11:07:16
 * @Author: lcfc-desktop
 */

#pragma once

#include <pthread.h>
#include <unistd.h>

#include <list>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>  // NOLINT

#include "common_lib/node.h"
#include "message_lib/cmd_message.h"
#include "message_lib/device_state.h"
#include "message_lib/odometer_message.h"
#include "message_lib/position_message.h"
#include "message_lib/radar_message.h"
#include "message_lib/simple_grid_map.h"
#include "message_lib/imu_message.h"
#include "message_lib/dmreader_message.h"

#include "ekf_calcuator/ekf_calcuator.h"
#include "location/location_result_calculator.h"
#include "location/location_interface.h"
#include "amcl/grid_location.h"
#include "qr_location/qr_location.h"
#include "reflector_location/reflector_location.h"
#include "reflector_location/wanji_location.h"

#include "include/config_struct.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

#define DEL(p)        \
  if (p != nullptr) { \
    delete p;         \
    p = nullptr;      \
  }

class LocationManagerImpl {
 public:
  using Node = gomros::common::Node;
  using RawPublisher = gomros::common::RawPublisher;
  using DeviceState = gomros::message::DeviceState;
  using OdometerMessage = gomros::message::OdometerMessage;
  using Position = gomros::message::Position;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using DMreaderSensoryMessage = gomros::message::DMreaderSensoryMessage;
  using ImuSensoryMessage = gomros::message::ImuSensoryMessage;
  using SimpleGridMap = gomros::message::SimpleGridMap;
  using CmdData = gomros::message::CmdData;
  using CMDTYPE = gomros::message::CMDTYPE;
  using MAP_CMD_TYPE = gomros::message::MAP_CMD_TYPE;
  using Logger = gomros::common::Logger;

  LocationManagerImpl(const LocationConfig &config,
                               std::shared_ptr<DeviceState> device_state);

  virtual ~LocationManagerImpl();

  /**
   * @description: 急停后的处理
   * @return {*}
   */
  void DealEmerge();
  void SetRadarData(const RadarSensoryMessage &data);
  void ResetPose();
  void SetOdomData(const OdometerMessage &data);
  void SetImuData(const ImuSensoryMessage& data);
  void SetQrData(const DMreaderSensoryMessage& data);
  void SetMapData(SimpleGridMap* simple_grid_map);
  void SetQrMapData(std::map<int, QRCoordinate> qr_map) {
    qr_location_->SetQrMap(qr_map);
  }
  void SetLandMarkMapData(
    const std::map<int, std::map<int, Eigen::Vector2d>>& map) {
    reflector_location_->SetMapData(map);
  }
  void SetLocationArea(
      const std::map<LocationType, std::vector<LocationArea>>& areas);
  void SetIniPose(const Position &pose);
  void SetTotalOdom(double total_odom);
  void SetDeviceState(
      std::shared_ptr<gomros::message::DeviceState> device_state);

  double GetTotalOdom();
  double GetQrOdom();
  Position GetCurrentPose();

  void SetHaveMap(bool have_map);

  void SetHavePose(bool have_pose);
  void SetIsMove(bool is_move) {
    if (is_move_ != is_move) {
      SLAM_INFO("change move form %d to %d", is_move_, is_move);
    }
    is_move_ = is_move;
  }

  bool FinishLocate();

  void StartGlobalLocating(bool is_need_location = true);
  gomros::message::RadarSensoryInfo GetLaserData();
  gomros::message::RadarSensoryInfo GetParticleData();
  gomros::message::Position GetCurrentPos();
  cv::Mat GetImage();

 private:
  static void *_monitorGlobalLocateThreadFunction(void *param);
  static void *_reflectorLocationFunction(void *param);
  static void *_amclLocationFunction(void *param);
  static void *_wanJiLocationFunction(void *param);
  static void *_handleOdomFunction(void *param);
  void _doGlobalLocateMonitor();
  Position _refreshOdom(bool is_pop = true);
  Position _refreshOdom(uint64_t time, bool is_pop = true);
  Position _refreshOdomForAmcl(uint64_t time);
  void _updateAmclPos();
  void _updateReflectorPos();
  void _updateQrPos(const Position& qr_pos);

  // amcl推算得到的当前位姿
  void _calDaltaPos(const Position& last_pos, const Position& now_pos,
                    const Position& now_amcl_pos, Position* current_pose);
  // 判断位姿是否出现跳变 true 出现跳变 false 未出现跳变
  bool _isPosJump(const Position& predictive_pos, const Position& real_pos);

 private:
  bool radar_disconnect = true;
  bool is_finished_global_location_ = false;  // 外部判断是否完成初始化的标志

  bool have_init_pose_ = false;
  bool have_map_ = false;
  bool is_amcl_deal_radar_data_;
  bool is_reflector_deal_radar_data_;
  bool is_finish_global_location;
  bool is_move_;
  bool is_get_total_odom_;

  // 一些状态变量
  Position amcl_last_pos_;
  Position amcl_current_pos_;
  Position reflector_last_pos_;
  Position reflector_current_pos_;
  Position location_last_pose_;     // 上一帧激光雷达得到后的位姿

  std::shared_ptr<DeviceState> device_state_;
  int m_SavePoseFileIndex = 1;
  double total_odom_;
  double qr_distance_;
  // std::recursive_mutex amcl_pos_data_mutex_;
  std::recursive_mutex reflector_pos_data_mutex_;

  pthread_mutex_t amcl_pos_data_mutex_;

  pthread_t m_MonitorGlobalLocateThread;  // 监视初始化定位是否完成
  pthread_t reflector_location_thread_;
  pthread_t amcl_location_thread_;
  pthread_t odom_thread;
  pthread_t file_thread;
  pthread_t wanji_location_thread_;

  pthread_mutex_t odom_data_mutex_;
  pthread_mutex_t last_odom_mutex_;
  pthread_mutex_t odom_data_for_amcl_mutex_;
  pthread_mutex_t ladar_data_mutex_;

  std::list<OdometerMessage> odom_list_;
  std::list<OdometerMessage> odom_list_for_amcl_;
  RadarSensoryMessage radar_data_;
  OdometerMessage last_odom_data_;
  bool get_last_odom_;
  // TODO(r) qr定位模块和反光板定位模块后续需要重构
  std::shared_ptr<DataFusion::EKFCalcuator> ekf_for_amcl_;
  std::shared_ptr<DataFusion::LocationResult> location_data_fusion_cal_;
  std::shared_ptr<GridLocation> grid_location_;
  std::shared_ptr<QrLocation> qr_location_;
  std::shared_ptr<ReflectorLocation> reflector_location_;
  std::shared_ptr<WanJiLocation> wan_ji_location_;

  // config param 配置参数
  LocationConfig location_config_;
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
