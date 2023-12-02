/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-08-05 17:09:35
 * @LastEditTime: 2023-08-19 14:53:59
 * @Author: renjy
 */
#pragma once

#include <string>
#include <vector>
#include <tuple>
#include <map>

#include "ceres/ceres.h"

#include "message_lib/radar_message.h"
#include "mapping/mapping_interface.h"
#include "landmark_tool/landmark_dbscan_calcuator.h"
#include "reflector_mapping/constraint.h"
#include "include/config_struct.h"
#include "common/tool.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

struct SubmapGlobalPose {
  // 全局坐标系！！
  std::array<double, 3> pose_info;
  bool is_get_global_pose;
  SubmapGlobalPose() {
    is_get_global_pose = false;
    pose_info[0] = 0.0;
    pose_info[1] = 0.0;
    pose_info[2] = 0.0;
  }
};

class LocalcentorCostFunction {
 public:
  explicit LocalcentorCostFunction(
      const std::array<double, 2> &local_landmark_pose)
      : local_landmark_pose_(local_landmark_pose) {}

  template <typename T>
  bool operator()(const T *const start_pose, T *e) const {
    // 信任局部位置，增加权重
    e[0] = (start_pose[0] - local_landmark_pose_[0]) * 1.0;
    e[1] = (start_pose[1] - local_landmark_pose_[1]) * 1.0;
    return true;
  }

 private:
  // 约束, 图结构的边
  const std::array<double, 2> local_landmark_pose_;
};

class SubmapConstraintCostFunction {
 public:
  explicit SubmapConstraintCostFunction(
      const std::array<double, 3> &in_end_map_pos)
      : in_end_map_pos_(in_end_map_pos) {}

  template <typename T>
  bool operator()(const T *const start_pose,
                  const T *const end_pos, T *e) const {
    std::array<T, 3> new_start_pos;
    new_start_pos[0] = T(in_end_map_pos_[0]) * ceres::cos(end_pos[2]) -
                       T(in_end_map_pos_[1]) * ceres::sin(end_pos[2]) +
                       end_pos[0];
    new_start_pos[1] = T(in_end_map_pos_[0]) * ceres::sin(end_pos[2]) +
                       T(in_end_map_pos_[1]) * ceres::cos(end_pos[2]) +
                       end_pos[1];
    new_start_pos[2] =
      SLAMMath::NormalizePITheta(end_pos[2] + T(in_end_map_pos_[2]));
    e[0] = (start_pose[0] - new_start_pos[0]);
    e[1] = (start_pose[1] - new_start_pos[1]);
    e[2] = SLAMMath::NormalizePITheta(start_pose[2] - new_start_pos[2]);
    return true;
  }

 private:
  // 约束, 图结构的边  子图回环之间的约束
  const std::array<double, 3> in_end_map_pos_;
};

class LandmarkCostFunction {
 public:
  template <typename T>
  bool operator()(const T *const start_submap_pose,
                  const T *const end_submap_pose,
                  const T *const start_landmark_pose,
                  const T *const end_landmark_pose, T *e) const {
      T start_world_x =
        start_landmark_pose[0] * ceres::cos(start_submap_pose[2]) -
        start_landmark_pose[1] * ceres::sin(start_submap_pose[2]) +
        start_submap_pose[0];
      T start_world_y =
        start_landmark_pose[0] * ceres::sin(start_submap_pose[2]) +
        start_landmark_pose[1] * ceres::cos(start_submap_pose[2]) +
        start_submap_pose[1];

      T end_world_x =
        end_landmark_pose[0] * ceres::cos(end_submap_pose[2]) -
        end_landmark_pose[1] * ceres::sin(end_submap_pose[2]) +
        end_submap_pose[0];
      T end_world_y =
        end_landmark_pose[0] * ceres::sin(end_submap_pose[2]) +
        end_landmark_pose[1] * ceres::cos(end_submap_pose[2]) +
        end_submap_pose[1];
      e[0] = (start_world_x - end_world_x);
      e[1] = (start_world_y - end_world_y);
      return true;
  }
};


class MergeReflectors {
 public:
  explicit MergeReflectors(const MappingConfig& config);
  virtual ~MergeReflectors() {}

  void SetReflectorMapInfo(const std::vector<Eigen::Vector2d>& map);
  /**
   * @describes: 获取建图结果
   * @param result 建图结果 id : -1 总图 + 全局做标配
   * @return {*}
   */
  bool GetMapResult(
    std::map<int, std::map<int, Eigen::Vector2d>>* result);

 private:
  void _runOptimizer();
  bool _isLoop(const std::vector<Eigen::Vector2d>& map,
               SubmapGlobalPose* global_pose);
  bool _refleashAllSubmap();
  void _calGlobalMap(std::vector<Eigen::Vector2d>* global_map);
  std::array<double, 3> _local2Global(const Eigen::Vector3d& local,
                                      const std::array<double, 3>& global);

 private:
  MappingConfig mapping_config_;
  std::map<int, std::vector<Eigen::Vector2d>> sub_maps_;
  std::map<int, std::vector<Eigen::Vector2d>> actual_sub_maps_;
  std::map<int, SubmapGlobalPose> global_pose_;
  std::vector<Constraint> constraint_;
};


class TrilateralMapping : public MappingInterface {
 public:
  using OdometerMessage = gomros::message::OdometerMessage;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using RadarSensoryInfo = gomros::message::RadarSensoryInfo;
  using Position = gomros::message::Position;

 public:
  explicit TrilateralMapping(const MappingConfig& config);
  virtual ~TrilateralMapping() {}

  /**
   * @brief: 开始建图
   * @return {*}
   */  
  void StartMapping() override;
  /**
   * @brief: 结束建图
   * @param {string} map_name 地图输出名字
   * @return {*}
   */  
  void StopMapping(const std::string& map_name) override;

 private:
  void _transform2RadarSensoryInfo(const std::vector<std::string>& ladar_str,
                                    RadarSensoryInfo* ladar_message);

  void _saveToFile(
    const std::map<int, std::map<int, Eigen::Vector2d>>& result,
    const std::string& map_name);
};



}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
