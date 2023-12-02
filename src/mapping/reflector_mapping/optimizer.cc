/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2023-06-01 08:07:39
 * @LastEditTime: 2023-08-01 17:31:33
 */

#include "reflector_mapping/optimizer.h"



namespace gomros {
namespace data_process {
namespace mapping_and_location {

Optimizer::Optimizer(const MappingConfig& config,
  internal_common::ThreadPool* thread_pool)
: config_(config), thread_pool_(thread_pool) {
  constraint_ = std::make_shared<SubMapConstraint>(config_);
}
void Optimizer::Init() {
  finished_submaps_.clear();
  constraint_->Init();
  submaps_global_pose_.clear();
  reflectors_in_submaps_.clear();
  reflectors_in_submaps_bound_.clear();
}

void Optimizer::AddSubMap(std::shared_ptr<Submap> new_finished_submap) {
  int submap_id = new_finished_submap->GetSubMapId();
  if (finished_submaps_.empty()) {
    assert(new_finished_submap->GetGlobalPose().first ==
           Eigen::Vector3d(0, 0, 0));
    submaps_global_pose_[submap_id] =
      new_finished_submap->GetGlobalPose().first;
  } else {
    submaps_global_pose_[submap_id] =
      _transform2Global(new_finished_submap->GetRelativePose(),
      submaps_global_pose_[submap_id - 1]);
    SLAM_DEBUG("relative (%f %f %f), global pos (%f %f %f)",
                submaps_global_pose_[submap_id](0),
                submaps_global_pose_[submap_id](1),
                submaps_global_pose_[submap_id](2),
                new_finished_submap->GetGlobalPose().first(0),
                new_finished_submap->GetGlobalPose().first(1),
                new_finished_submap->GetGlobalPose().first(2));
  }
  finished_submaps_.push_back(new_finished_submap);
  int relector_size =
    new_finished_submap->GetReflectorInSubMap().reflectors_.size();
  SLAM_INFO("in submap %d has %d relectors", submap_id, relector_size);
  reflectors_in_submaps_[submap_id] =
    new_finished_submap->GetReflectorInSubMap().reflectors_;

  for (int i = 0; i < relector_size; i++) {
    Eigen::Vector2d bound(std::sqrt(new_finished_submap->GetReflectorInSubMap().
                                    reflector_map_coviarance_[i](0, 0)),
                          std::sqrt(new_finished_submap->GetReflectorInSubMap().
                                    reflector_map_coviarance_[i](1, 1)));
    reflectors_in_submaps_bound_[submap_id].push_back(bound);
  }

  if (constraint_->ComputeSubMapConstraint(
      reflectors_in_submaps_, submaps_global_pose_, finished_submaps_)) {
    RunFinalOptimization();
  }
  return;
}

void Optimizer::RunFinalOptimization() {
  const std::vector<Constraint> constraints = constraint_->GetConstraint();
  // constraint_->ClearConstraint();
  if (constraints.empty()) {
    SLAM_WARN("there is no constraint");
    return;
  }
  SLAM_INFO("begin run optimization!!!!");
  for (int i = 0; i < constraints.size(); i++) {
    SLAM_DEBUG("constraints info %d %d %d %d %d %f", i,
                constraints[i].start_submap_id,
                constraints[i].start_reflector_id,
                constraints[i].end_submap_id,
                constraints[i].end_reflector_id,
                constraints[i].distance);
  }

  ceres::Problem problem;
  // ceres需要double的指针, std::array能转成原始指针的形式
  std::map<int, std::array<double, 3>> C_submaps;
  std::map<int, std::vector<std::array<double, 2>>> C_landmarks;

  bool first_submap = true;

  // 将需要优化的子图位姿设置为优化参数
  for (const auto &submap_id_pose : submaps_global_pose_) {
    // 将子图的global_pose放入C_submaps中
    C_submaps[submap_id_pose.first] =
      std::array<double, 3>({submap_id_pose.second(0),
                             submap_id_pose.second(1),
                             submap_id_pose.second(2)});
    // c++11: std::array::data() 返回指向数组对象中第一个元素的指针
    // 添加需要优化的数据 这里显式添加参数块,会进行额外的参数块正确性检查
    problem.AddParameterBlock(C_submaps.at(submap_id_pose.first).data(), 3);

    if (first_submap) {
      first_submap = false;
      // 如果是第一幅子图, 不优化这个子图位姿
      problem.SetParameterBlockConstant(
        C_submaps.at(submap_id_pose.first).data());
    }
  }
  // 将需要优化的landmark设置为优化参数
  // std::vector<double*>* parameter_blocks = new std::vector<double*>;
  for (const auto &landmark_id_poses : reflectors_in_submaps_) {
    // 将landmark的local_pose_2d放入C_landmarks中
    const auto &landmarkcenters = landmark_id_poses.second;
    for (int id = 0; id < landmarkcenters.size(); id++) {
      C_landmarks[landmark_id_poses.first].push_back(std::array<double, 2>(
        {landmarkcenters[id](0), landmarkcenters[id](1)}));
      problem.AddParameterBlock(
        C_landmarks.at(landmark_id_poses.first)[id].data(), 2);

      problem.SetParameterUpperBound(
        C_landmarks.at(landmark_id_poses.first)[id].data(), 0,
        C_landmarks.at(landmark_id_poses.first)[id][0] +
        reflectors_in_submaps_bound_.at(landmark_id_poses.first).at(id).x());
      problem.SetParameterLowerBound(
        C_landmarks.at(landmark_id_poses.first)[id].data(), 0,
        C_landmarks.at(landmark_id_poses.first)[id][0] -
        reflectors_in_submaps_bound_.at(landmark_id_poses.first).at(id).x());
      problem.SetParameterUpperBound(
        C_landmarks.at(landmark_id_poses.first)[id].data(), 1,
        C_landmarks.at(landmark_id_poses.first)[id][1] +
        reflectors_in_submaps_bound_.at(landmark_id_poses.first).at(id).y());
      problem.SetParameterLowerBound(
        C_landmarks.at(landmark_id_poses.first)[id].data(), 1,
        C_landmarks.at(landmark_id_poses.first)[id][1] -
        reflectors_in_submaps_bound_.at(landmark_id_poses.first).at(id).y());
    }
  }

  // 第一种约束：反光柱局部位置与初始值的约束
  for (const auto &landmark_id_poses : C_landmarks) {
    const auto &landmarkcenters = landmark_id_poses.second;
    for (unsigned int id = 0; id < landmarkcenters.size(); id++) {
      ceres::CostFunction *cost_function =
          new ceres::AutoDiffCostFunction<LocalLandmarkCostFunction, 2, 2>(
              new LocalLandmarkCostFunction(
                C_landmarks.at(landmark_id_poses.first).at(id)));
      // 设置残差项
      problem.AddResidualBlock(cost_function, NULL,
        C_landmarks.at(landmark_id_poses.first).at(id).data());
    }
  }
  // 第二种约束：子图位姿与初始值的约束以及子图与前一个子图的约束
  bool is_first_submap = true;

  auto last_submap_id = (*C_submaps.begin()).first;
  for (const auto &submap_id_poses : C_submaps) {
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<GlobalSubmapCostFunction, 3, 3>(
            new GlobalSubmapCostFunction(C_submaps.at(submap_id_poses.first)));
    if (is_first_submap) {
      is_first_submap = false;
      last_submap_id = submap_id_poses.first;
      // last_submap_poses = submap_id_poses.second;
    } else {
      ceres::CostFunction *cost_function2 =
          new ceres::AutoDiffCostFunction<BetweenSubmapCostFunction, 1, 3, 3>(
            new BetweenSubmapCostFunction(
            C_submaps.at(submap_id_poses.first), C_submaps.at(last_submap_id)));
      // 设置残差项
      problem.AddResidualBlock(cost_function2, NULL,
                               C_submaps.at(submap_id_poses.first).data(),
                               C_submaps.at(last_submap_id).data());
    }
  }

  // 第三种约束：反光柱与反光柱之间的约束
  for (const auto &constraint : constraints) {
    ceres::CostFunction *cost_function =
    new ceres::AutoDiffCostFunction<GlobalLandmarkCostFunction, 2, 3, 3, 2, 2>(
          new GlobalLandmarkCostFunction());
    // 设置残差项
    problem.AddResidualBlock(cost_function, NULL,
      C_submaps.at(constraint.start_submap_id).data(),
      C_submaps.at(constraint.end_submap_id).data(),
      C_landmarks.at(constraint.start_submap_id).
      at(constraint.start_reflector_id).data(),
      C_landmarks.at(constraint.end_submap_id).
      at(constraint.end_reflector_id).data());
  }
  // 解决问题
  ceres::Solver::Options options;
  options.max_num_iterations = 200;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // std::cout << summary.FullReport() << std::endl;

  // 将优化后的所有数据进行更新 Store the result.
  for (const auto &C_submap_id_data : C_submaps) {
    Eigen::Vector3d temp;
    temp(0) = C_submap_id_data.second[0];
    temp(1) = C_submap_id_data.second[1];
    temp(2) = SLAMMath::NormalizePITheta(C_submap_id_data.second[2]);
    SLAM_INFO("id %d used %f %f %f now %f %f %f", C_submap_id_data.first,
              submaps_global_pose_.at(C_submap_id_data.first)(0),
              submaps_global_pose_.at(C_submap_id_data.first)(1),
              submaps_global_pose_.at(C_submap_id_data.first)(2),
              temp(0), temp(1), temp(2));
    submaps_global_pose_.at(C_submap_id_data.first) = temp;
  }
  // 存储优化后的 landmark 信息
  for (const auto &C_landmark : C_landmarks) {
    for (unsigned int i = 0; i < C_landmark.second.size(); i++) {
      reflectors_in_submaps_[C_landmark.first][i][0] = C_landmark.second[i][0];
      reflectors_in_submaps_[C_landmark.first][i][1] = C_landmark.second[i][1];
    }
  }
  return;
}

Eigen::Vector3d Optimizer::_transform2Global(
  const Eigen::Vector3d& relative_pose,
  const Eigen::Vector3d& last_submap_global) {
  Eigen::Vector3d new_submap_pose;
  new_submap_pose(0) = relative_pose.x() * std::cos(last_submap_global.z()) -
                       relative_pose.y() * std::sin(last_submap_global.z()) +
                       last_submap_global.x();
  new_submap_pose(1) = relative_pose.x() * std::sin(last_submap_global.z()) +
                       relative_pose.y() * std::cos(last_submap_global.z()) +
                       last_submap_global.y();
  new_submap_pose(2) =
    SLAMMath::NormalizePITheta(relative_pose.z() + last_submap_global.z());
  return new_submap_pose;
}


}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros

