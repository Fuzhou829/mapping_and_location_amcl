/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-07-26 15:52:31
 * @LastEditTime: 2023-10-17 14:46:44
 * @Author: lcfc-desktop
 */
#pragma once

#include <assert.h>
#include <jsoncpp/json/json.h>
#include <pthread.h>
#include <limits>
#include <memory>
#include <mutex>  // NOLINT
#include <opencv2/opencv.hpp>

#include <list>
#include <vector>

#include "include/config_struct.h"
#include "location/amcl/amcl_laser.h"
#include "location/amcl/amcl_odom.h"
#include "location/amcl/map.h"
#include "location/amcl/pf.h"
#include "location/location_interface.h"
#include "message_lib/grid_map.h"
#include "message_lib/odometer_message.h"
#include "message_lib/position_message.h"
#include "message_lib/radar_message.h"
#include "message_lib/simple_grid_map.h"

#include "ekf_calcuator/ekf_calcuator.h"
#include "ekf_calcuator/reflector_ekf.h"

#include "common/logger.h"

#ifdef TEST_DEBUG
#include "common_lib/node.h"
#endif

namespace gomros {
namespace data_process {
namespace mapping_and_location {

#define fuzzy_1 0.001
#define fuzzy_mini 0.01

typedef struct {
  // Total weight (weights sum to 1)
  double weight;
  double score;
  // Mean of pose esimate
  amcl::pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  amcl::pf_matrix_t pf_pose_cov;
} amcl_hyp_t;

double normalize(double z);
double angle_diff(double ang_a, double ang_b);
amcl::pf_vector_t uniformPoseGenerator(void* arg);

class GridLocation : public LocationInterface {
 public:
  typedef gomros::message::SimpleGridMap* MapSharedPointer;
  using Position = gomros::message::Position;
  using OdometerMessage = gomros::message::OdometerMessage;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using RadarSensoryInfo = gomros::message::RadarSensoryInfo;
  using SimpleGridMap = gomros::message::SimpleGridMap;
  using MapInfo = gomros::message::MapInfo;
  using Logger = gomros::common::Logger;

  explicit GridLocation(const LocationConfig& config);
  ~GridLocation();

  void HandleLaserData(const RadarSensoryMessage& data,
                       const Position& forecast_pos, bool is_move) override;
  void SetInitPose(float x, float y, float theta) override;
  void SetGlobalLocationPos(const Position& pos) override;
  bool StartGlobalLocating() override;
  bool StartGlobalLocating(bool is_relocation) override;

  void SetMapData(MapSharedPointer grid_map) override;

  bool GetCurrentPosition(Position* pos) override;

  bool IsFinishLocate() override;

  RadarSensoryInfo GetLaserData();
  RadarSensoryInfo GetParticleData();
  Position GetCurrentPose();
  cv::Mat GetImage();

 private:
  static void* _globalLocatingThreadFunction(void* ptr);
  void _freeMapDependentMemory(bool save_map = false);
  amcl::map_t* _doMapOpen();
  void _mainParticleRun();
  bool _initialLocation(bool bin);
  bool _particleFilterRun(char fup);

  bool _disAmong(amcl::pf_vector_t pos1, amcl::pf_vector_t pos2, double dis);

  bool _jhInitialPf();
  int _particleFilterInitialize();
  void _paramInitial();
  void _publishLadarInfoForTest(const RadarSensoryInfo& raw_ladar);

 private:
  MapSharedPointer p_grid_map_;  // 栅格地图
  std::recursive_mutex amcl_mutex_;
  float m_InitPoseX;  // 通过外部传递给amcl算法的位姿
  float m_InitPoseY;
  float m_InitPoseTheta;
  bool ready = false;
  RadarSensoryInfo radar_data_;
  RadarSensoryInfo particle_data_;
  Position cur_pos_;
  int show_cnt_ = 0;
  cv::Mat cv_map_image_;
  cv::Mat cv_init_map_image_;

  std::vector<amcl::pf_vector_t> landmarks_;

  Position m_RadarPose;  // 实时位姿

  // 全局定位状态
  bool m_FirstLocatedInvoked;
  bool finish_location_;
  bool is_relocation_;
  bool is_get_ladar_;
  bool first_laser_ = true;
  bool get_landmark_ = false;
  double min_dis_;
  double min_theta_;

  bool is_get_amcl_pos_;
  float m_LocateScore;
  char m_LocateStep;
  amcl::pf_vector_t m_AmclPose;

  int m_OdomType;

  amcl::pf_t* m_pPf = nullptr;  // 粒子滤波器
  bool m_PfInit;

  amcl::AMCLOdom* m_pOdom = nullptr;
  amcl::AMCLLaser* m_pLaser = nullptr;
  amcl::map_t* m_pMap = nullptr;              // AMCL格式的地图数据
  std::recursive_mutex ladar_data_mutex_;     // 增加ladar数据锁
  amcl::pf_vector_t m_LastOdomPose;           // last odom data
  message::RadarSensoryInfo m_LastRadarData;  // last radar data
  bool m_LasersUpdate;

  Position m_RadarPosition;  // 雷达安装位置

  Position current_posotion_;
  int m_ResampleCount;  // 重采样计数

  amcl::PFParam pf_param_;
  //  bool m_LocatingFlag_ = false;
  pthread_t m_GlobalLocatingThread;
  pthread_mutex_t particle_mutex_;
  pthread_cond_t cond;

  LocationConfig m_LocationConfig;
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
