/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.2
 * @Author: renjy
 * @Date: 2023-06-02 02:02:50
 * @LastEditTime: 2023-10-18 15:27:33
 */

#pragma once
#include <list>
#include <map>
#include <vector>
#include <memory>

#include "Eigen/Dense"

#include "message_lib/odometer_message.h"
#include "message_lib/imu_message.h"
#include "message_lib/radar_message.h"

#include "include/config_struct.h"
#include "landmark_tool/landmark_center_calcuator.h"
#include "common/logger.h"

#include "display_result/display_result.h"

namespace DataFusion {

namespace MAL = gomros::data_process::mapping_and_location;

/**
 * @brief: 观测信息
 */
class Observation {
 public:
  Observation() {
    centers.clear();
  }
  Observation(const uint64_t& time, const std::vector<Eigen::Vector2d> &cloud)
    : time_(time), centers(cloud) {}
  /**
   * @describes: 根据激光帧获取到观测信息（雷达坐标系下）
   * @return {*}
   */  
  void GetObsFromLadarData(const MAL::SensorMount& sensor_mount,
                           const MAL::LandMarkConfig& landmark_config,
                  const gomros::message::RadarSensoryInfo& radar_info);

  uint64_t time_;
  std::vector<Eigen::Vector2d> centers;
  std::map<int, LandMark::Center> land_mark_;
};

/**
 * @brief: 机器状态
 */
struct State {
  uint64_t time;
  Eigen::VectorXd mu;
  Eigen::MatrixXd sigma;
  // 第一个id 对应状态中的反光柱组 i - mu(3 + 2* i)和 mu(3 + 2* ( i+ 1)）
  // 用于建图时判断该反光柱被识别了多少次
  std::map<int, int> obs_in_map_count;
  /**
   * @describes: 初始化
   * @return {*}
   */  
  void Init() {
    sigma.resize(3, 3);
    sigma.setZero();
    mu.resize(3, 1);
    mu(0) = mu(1) = mu(2) = 0.0;
    time = 0;
  }
};

/**
 * @describes: 匹配结果信息
 * @return {*}
 */
struct MatchInfo {
  int obs_id;     // 观测数据中的id
  int match_id;   // 匹配对象id （地图或状态）
  float min_distance;   // 最近匹配距离
  MatchInfo(int obs_id, int match_id, float min_distance) {
    this->obs_id = obs_id;
    this->match_id = match_id;
    this->min_distance = min_distance;
  }
};

/**
 * @brief: 反光柱匹配信息
 */
struct ReflectorMatchResult {
  // map - observation matched id
  // 第一个参数对应在观测数据中的id， 第二个参数关联的反光柱在地图中的id
  std::vector<MatchInfo> map_obs_match_ids;
  // mu_ - observation matched id
  // 第一个参数对应在观测数据中的id， 第二个状态空间下反光柱的id
  std::vector<MatchInfo> state_obs_match_ids;
  // new observed reflector id
  std::vector<int> new_ids;
};

/**
 * @brief: 反光柱地图信息
 */
class ReflectorMap {
 public:
  ReflectorMap() {}
  ReflectorMap(const std::vector<Eigen::Vector2d> &centers,
               const std::vector<Eigen::Matrix2d> &cov) :
               reflectors_(centers), reflector_map_coviarance_(cov) {}
  std::vector<Eigen::Vector2d> reflectors_;
  std::vector<Eigen::Matrix2d> reflector_map_coviarance_;
};

/**
 * @brief: 用于融合雷达和和odom数据信息 用于反光柱定位和建图
 */
class ReflectorEkfCalcuator {
 public:
  using OdometerMessage = gomros::message::OdometerMessage;
  using ImuSensoryMessage = gomros::message::ImuSensoryMessage;

 public:
  ReflectorEkfCalcuator(const MAL::EKFConfig& ekf_config,
                        const MAL::SensorMount& sensor_mount_config,
                        const MAL::LandMarkConfig& landmark_config,
                        bool is_mapping = true);
  virtual ~ReflectorEkfCalcuator() {}
  // 复制-为子图连接
  void Copy(const ReflectorEkfCalcuator &obj);

  void SetInitState(const State& state);
  void SetMapDate(const ReflectorMap& map_info);

  void HandleOdomData(const OdometerMessage& data);
  void HandleImuData(const ImuSensoryMessage& data);
  void HandleObsData(const Observation& obs);
  bool GetState(State* state);
  // for test
  std::vector<int> GetMapMatch() const {
    return show_map_id_;
  }

 private:
  ReflectorMatchResult _calReflectorMatch(const Observation& obs);
  Eigen::Vector2d _ladar2GlobalFrame(const Eigen::Vector2d& obs);

  /**
  * @name: 判断是否与状态中的反光柱相关联
  * @param obs 观测信息（雷达坐标系下)
  * @param obs_id 当前帧观测对应id
  * @param reflector_match_result 反光柱匹配结果
  * @return {*}
  */ 
  bool _isMatchToState(const Eigen::Vector2d& obs, int obs_id,
                       ReflectorMatchResult* reflector_match_result);

  /**
  * @name: 判断是否与地图中的反光柱相关联
  * @param obs 观测信息（雷达坐标系下)
  * @param obs_id 当前帧观测对应id
  * @param index_for_data_association 关联地图中反光柱id
  * @param reflector_match_result 反光柱匹配结果
  * @return {*}
  */ 
  bool _isMatchToMap(const Eigen::Vector2d& obs, int obs_id,
                   const std::vector<int>& index_for_data_association,
                   ReflectorMatchResult* reflector_match_result);

  void _update(const Observation& observation,
                const ReflectorMatchResult& result);
  void _refreshOdom(uint64_t time);

  void _refreshImu(const DataFusion::State& previous_state,
                   const DataFusion::State& current_state);
  void _calOdomMessage(const OdometerMessage& data);
  void _calOdomMessageDiffWheel(const OdometerMessage& data);
  void _calOdomMessageDoubleSteer(const OdometerMessage& data);
  void _calOdomMessageSingleSteer(const OdometerMessage& data);
  void _calPredictedState(float v, float w, double delta_time,
                          uint64_t time_stamp,
                          const Eigen::Vector3d& delta_pos,
                          bool in_global = false);

 private:
  MAL::EKFConfig ekf_config_;
  bool is_mapping_;
  MAL::SensorMount sensor_mount_config_;
  MAL::LandMarkConfig landmark_config_;
  Eigen::Matrix<float, 4, 3> config_mat_;  // 舵轮配置矩阵
  Eigen::Matrix<double, 2, 2, Eigen::DontAlign> Qt_;    // 观测噪声协方差矩阵

  ReflectorMap reflector_map_info_;
  State state_;
  bool is_get_location_pos_;
  bool get_init_state_;
  std::list<OdometerMessage> odom_list_;          // odom待处理数据
  std::list<ImuSensoryMessage> imu_list_;  //  imu待处理数据
  bool get_odom_data_;
  OdometerMessage last_odom_data_;
  // 线程锁
  pthread_mutex_t state_mutex_;
  pthread_mutex_t odom_mutex_;
  pthread_mutex_t imu_mutex_;
  // for test
  std::vector<int> show_map_id_;
  std::shared_ptr<DisPlayResult::SimulationDataPublisher>
    display_reflector_ekf_;
};


}  // namespace DataFusion
