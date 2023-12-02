/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.2
 * @Author: renjy
 * @Date: 2023-06-01 08:07:39
 * @LastEditTime: 2023-08-01 16:52:05
 */

#pragma once
#include <memory>
#include <map>
#include <vector>

#include "ceres/ceres.h"

#include "common/thread_pool.h"
#include "reflector_mapping/submap.h"
#include "reflector_mapping/constraint.h"
#include "include/mapping_and_location_math.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

class LocalLandmarkCostFunction {
 public:
  explicit LocalLandmarkCostFunction(
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


class GlobalSubmapCostFunction {
 public:
  explicit GlobalSubmapCostFunction(
      const std::array<double, 3> &global_submap_pose)
      : global_submap_pose_(global_submap_pose) {}

  template <typename T>
  bool operator()(const T *const start_pose, T *e) const {
    e[0] = (start_pose[0] - global_submap_pose_[0]);
    e[1] = (start_pose[1] - global_submap_pose_[1]);
    e[2] = SLAMMath::NormalizePITheta(start_pose[2] - global_submap_pose_[2]);
    return true;
  }

 private:
    // 约束, 图结构的边
    const std::array<double, 3> global_submap_pose_;
};

class BetweenSubmapCostFunction {
 public:
  explicit BetweenSubmapCostFunction(
      const std::array<double, 3> &current_submap_pose,
      const std::array<double, 3> &last_submap_pose)
      : current_submap_pose_(current_submap_pose),
        last_submap_pose_(last_submap_pose) {}

  template <typename T>
  bool operator()(const T *const current_pose,
                  const T *const last_pose, T *e) const {
      e[0] = SLAMMath::NormalizePITheta((current_pose[2] - last_pose[2])
                -(current_submap_pose_[2] - last_submap_pose_[2]))/
                ((current_submap_pose_[2] - last_submap_pose_[2]) * 1000.0);
      return true;
  }

 private:
  // 约束, 图结构的边
  const std::array<double, 3> current_submap_pose_;
  const std::array<double, 3> last_submap_pose_;
};

class GlobalLandmarkCostFunction {
 public:
  template <typename T>
  bool operator()(const T *const start_submap_pose,
                  const T *const end_submap_pose,
                  const T *const start_landmark_pose,
                  const T *const end_landmark_pose,
                  T *e) const {
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


/* 为纯反光柱建图提供优化*/

class Optimizer {
 public:
  explicit Optimizer(const MappingConfig& config,
      internal_common::ThreadPool* thread_pool);
  virtual ~Optimizer() {}
  void Init();

  void AddSubMap(std::shared_ptr<Submap> new_finished_submap);
  void RunFinalOptimization();
  /**
   * @describes: 返回子图的全局坐标
   * @return {*}
   */  
  std::map<int, Eigen::Vector3d> GetSubMapsGlobalPose() const {
    return submaps_global_pose_;
  }
  /**
   * @describes: 返回子图中反光柱坐标（子图坐标系下）
   * @return {*}
   */
  std::map<int, std::vector<Eigen::Vector2d>> GetSubMapsReflectors() const {
    return reflectors_in_submaps_;
  }

 private:
  Eigen::Vector3d _transform2Global(const Eigen::Vector3d& relative_pose,
                                    const Eigen::Vector3d& last_submap_global);

 private:
  MappingConfig config_;
  internal_common::ThreadPool* thread_pool_;
  std::vector<std::shared_ptr<Submap>> finished_submaps_;
  std::shared_ptr<SubMapConstraint> constraint_;
  // 不断被优化的subamp
  std::map<int, Eigen::Vector3d> submaps_global_pose_;
  // submap_id- 在submap坐标下的反光柱信息
  std::map<int, std::vector<Eigen::Vector2d>> reflectors_in_submaps_;
  // submap_id - 对应反光柱的边？方差
  std::map<int, std::vector<Eigen::Vector2d>> reflectors_in_submaps_bound_;
};








}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros

