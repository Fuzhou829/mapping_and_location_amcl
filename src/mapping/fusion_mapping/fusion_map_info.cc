/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-16 16:34:15
 * @LastEditTime: 2023-11-09 14:38:50
 */

#include <ceres/ceres.h>

#include "fusion_mapping/fusion_map_info.h"
#include "karto_mapping/Macros.h"


#include "ekf_calcuator/ekf_calcuator.h"

#include "Eigen/Dense"
#include "jsoncpp/json/json.h"
#include "common/logger.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

// class LocalCostFunction {
//  public:
//   explicit LocalCostFunction(
//       const std::array<double, 3> &local_landmark_pose)
//       : local_landmark_pose_(local_landmark_pose) {}

//   template <typename T>
//   bool operator()(const T *const start_pose, T *e) const {
//     // 信任局部位置，增加权重
//     e[0] = (start_pose[0] - local_landmark_pose_[0]) * 1.0;
//     e[1] = (start_pose[1] - local_landmark_pose_[1]) * 1.0;
//     e[2] = SLAMMath::NormalizePITheta(
//             start_pose[2] - local_landmark_pose_[2]) * 1.0;
//     return true;
//   }

//  private:
//   // 约束, 图结构的边
//   const std::array<double, 3> local_landmark_pose_;
// };


// class TransCostFunction {
//  public:
//   explicit TransCostFunction(
//       const std::array<double, 2> &delta)
//       : delta_(delta) {}

//   template <typename T>
//   bool operator()(const T *const start_pose,
//                   const T *const end_pos, T *e) const {
//     T dis =
//       SLAMMath::Dist(start_pose[0], start_pose[1], end_pos[0], end_pos[1]);
//     T lin_theta;
//     if (ceres::abs(end_pos[0] - start_pose[0]) < 0.0000000001) {
//       lin_theta = T(M_PI * 0.5);
//     } else {
//      lin_theta = ceres::atan(
//       (end_pos[1] - start_pose[1]) / (end_pos[0] - start_pose[0]));
//     }

//     e[0] = ceres::abs(ceres::cos(lin_theta) * (delta_[0] - dis));
//     e[1] = ceres::abs(ceres::sin(lin_theta) * (delta_[0] - dis));
//     e[2] = ceres::abs(
//       SLAMMath::NormalizePITheta(delta_[1] - start_pose[2] + end_pos[2]));
//     return true;
//   }

//  private:
//   const std::array<double, 2> delta_;
// };


FusionMap::FusionMap(const MappingConfig& config,
  kt_int32s width, kt_int32s height,
  const karto::Vector2<kt_double>& rOffset,
  bool is_qr_calibration)
  : karto::Grid<kt_int8u>(width, height),
    config_(config),
    cell_pass_cnt_(karto::Grid<kt_int32u>::CreateGrid(
                      0, 0, config.mapping_resolution)),
    cell_hits_cnt_(karto::Grid<kt_int32u>::CreateGrid(
                      0, 0, config.mapping_resolution)),
    is_qr_calibration_(is_qr_calibration) {
  if (karto::math::DoubleEqual(config.mapping_resolution, 0.0)) {
    throw karto::Exception("Resolution cannot be 0");
  }
  // TODO(r) 更改为配置参数
  min_pass_through_ = 2;
  occupancy_threshold_ = 0.1;

  GetCoordinateConverter()->SetScale(1.0 / config.mapping_resolution);
  GetCoordinateConverter()->SetOffset(rOffset);

  // 外部输入二维码相对坐标
  _initQrMapInfo();
  show_ladar_for_test_ = std::make_shared<
    DisPlayResult::SimulationDataPublisher>(config_.sensor_mount);
}


void FusionMap::CreateMapFromScans(
  const karto::LocalizedRangeScanVector& scan_points,
  const std::string& map_name) {
  // 创建反光板 二维码 栅格融合地图
  karto::Vector2<kt_double> rOffset = GetCoordinateConverter()->GetOffset();
  cell_pass_cnt_->Resize(GetWidth(), GetHeight());
  cell_pass_cnt_->GetCoordinateConverter()->SetOffset(rOffset);

  cell_hits_cnt_->Resize(GetWidth(), GetHeight());
  cell_hits_cnt_->GetCoordinateConverter()->SetOffset(rOffset);

  std::vector<QRCoordinate> qrs_world;
  std::map<int, LandMark::Center> land_mark_map;
  landmark_points_.clear();
  // karto::LocalizedRangeScanVector new_scanPoint;

  // std::map<int, std::array<double, 3>> poses;
  // karto::LocalizedRangeScan* last_scan = NULL;

  // struct Constraint {
  //   int last_id;
  //   int current_id;
  //   float delta_theta;  // 绝对值
  //   float delta_dis;   // 绝对值
  //   Constraint(int last, int current, float theta,
  //              float dis) {
  //     last_id = last;
  //     current_id = current;
  //     delta_theta = theta;
  //     delta_dis = dis;
  //   }
  // };
  // std::vector<Constraint> constraints;

  // const_forEach(karto::LocalizedRangeScanVector, &scan_points) {
  //   karto::LocalizedRangeScan* current_scan = *iter;
  //   if (last_scan == NULL) {
  //     last_scan = current_scan;
  //   }
  //   karto::Pose2 sensor_pose = current_scan->GetSensorPose();
  //   poses.insert(std::make_pair(current_scan->GetStateId(),
  //       std::array<double, 3>{
  //       sensor_pose.GetPosition().GetX(),
  //       sensor_pose.GetPosition().GetY(),
  //       sensor_pose.GetHeading()}));
  //   karto::LandmarkConstraintsInfo constraint_current, constraint_last;
  //   if (current_scan->IsGetLandmarkConstraints(&constraint_current) &&
  //       last_scan->IsGetLandmarkConstraints(&constraint_last)) {
  //       if (constraint_current.landmark_id == constraint_last.landmark_id &&
  //           constraint_current.type != karto::MoveType::CURVATURE) {
  //         float delta_dis = 0.0, delta_theta = 0.0;
  //         if (constraint_current.type == karto::MoveType::ROTATION) {
  //             delta_theta = SLAMMath::NormalizePITheta(
  //               constraint_current.theta - constraint_last.theta);
  //         } else {
  //           delta_dis = SLAMMath::Dist(
  //                       constraint_current.x, constraint_current.y,
  //                       constraint_last.x, constraint_last.y);
  //         }
  //         constraints.push_back(Constraint(
  //           constraint_current.last_scan_id, constraint_current.current_scan_id,
  //           delta_theta, delta_dis));
  //       }
  //   }
  //   last_scan = current_scan;
  // }

  // ceres::Problem problem;
  //   for (int i = 0; i < 9; i++) {
  //   problem.AddParameterBlock(poses.at(i).data(), 3);
  //   if (i == 0) {
  //     problem.SetParameterBlockConstant(poses.at(i).data());
  //   }
  // }

  // // 1. 建立建图时构建出的约束
  // for (int i = 0; i < 9; i++) {
  //   ceres::CostFunction *cost_function =
  //       new ceres::AutoDiffCostFunction<LocalCostFunction, 3, 3>(
  //           new LocalCostFunction(poses.at(i)));
  //     // 设置残差项
  //     problem.AddResidualBlock(cost_function, NULL, poses.at(i).data());
  // }


  // for (auto iter : constraints) {
  //   // 2. 构建出距离及角度形成节点间的约束
  //   ceres::CostFunction *cost_function_translate =
  //       new ceres::AutoDiffCostFunction<TransCostFunction, 3, 3, 3>(
  //           new TransCostFunction(
  //             std::array<double, 2>{iter.delta_dis, iter.delta_theta}));
  //     problem.AddResidualBlock(cost_function_translate, NULL,
  //                               poses.at(iter.last_id).data(),
  //                               poses.at(iter.current_id).data());
  // }
  // // 处理问题
  // ceres::Solver::Options options;
  // options.max_num_iterations = 200;
  // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  // ceres::Solver::Summary summary;
  // ceres::Solve(options, &problem, &summary);

  // const_forEach(karto::LocalizedRangeScanVector, &scan_points) {
  //   karto::LocalizedRangeScan* current_scan = *iter;
  //   std::array<double, 3> pose = poses.at(current_scan->GetStateId());
  //   karto::Pose2 sensor_pose(pose[0], pose[1], pose[2]);
  //   current_scan->SetSensorPose(sensor_pose);
  //   new_scanPoint.push_back(current_scan);
  // }

  // const_forEach(karto::LocalizedRangeScanVector, &new_scanPoint) {
  //   karto::LocalizedRangeScan* current_scan = *iter;
  //   _createScanMapAndLandMark(current_scan, &land_mark_map);
  // }
  const_forEach(karto::LocalizedRangeScanVector, &scan_points) {
    karto::LocalizedRangeScan* current_scan = *iter;
    _createScanMapAndLandMark(current_scan, &land_mark_map);
  }
  _updateScanMap();
  _recordGridMap(map_name);

  // 二维码建图
  if (!create_qr_map_info_.empty()) _createQrMap(scan_points, &qrs_world);
  land_mark_map = _getLandmarkCenters();
  // 将数据记录到map_name中
  _recordMapInfoToFile(land_mark_map, qrs_world, map_name);
  usleep(1000);
  return;
}

void FusionMap::_initQrMapInfo() {
  // 初始化用于qr建图信息
  std::string qr_ladar_dir = "./mapping_data/scan_match_ladar.txt";
  std::ifstream qr_ladar_file;
  qr_ladar_file.open(qr_ladar_dir);
  std::string line_info;
  CreateQrMapInfo create_map_info;
  while (getline(qr_ladar_file, line_info)) {
    QRInfo qr_info;
    RadarSensoryInfo ladar_info;
    std::vector<std::string> qr_ladar_data =
      internal_common::SplitCString(line_info, " ");
    qr_info.time_stemp = atof(qr_ladar_data[0].c_str());
    qr_info.pos_x = atof(qr_ladar_data[1].c_str());
    qr_info.pos_y = atof(qr_ladar_data[2].c_str());
    qr_info.pos_theta = atof(qr_ladar_data[3].c_str());
    qr_info.pos_theta =
      SLAMMath::NormalizePITheta(-qr_info.pos_theta * M_PI / 180);
    qr_info.tag_num = atoi(qr_ladar_data[4].c_str());
    ladar_info.mstruRadarHeaderData.mlTimeStamp =
      atof(qr_ladar_data[5].c_str());
    float theta = config_.mapping_start_angle * M_PI / 180.0f;
    float delta_theta = config_.laser_resolution * M_PI / 180.0f;
    // 转至robot坐标系下
    ladar_info.mstruSingleLayerData.mvPoints.clear();
    ladar_info.mstruSingleLayerData.mvIntensities.clear();
    for (int i = 7; i < qr_ladar_data.size();) {
      float len = atof(qr_ladar_data[i++].c_str());
      float xp = len * cos(theta);
      float yp = len * sin(theta);
      theta += delta_theta;
      ladar_info.mstruSingleLayerData.mvPoints.push_back(
          Eigen::Vector2d(xp, yp));
      ladar_info.mstruSingleLayerData.mvIntensities.push_back(
        atof(qr_ladar_data[i++].c_str()));
    }
    create_map_info.ladar_info = ladar_info;
    create_map_info.qr_info = qr_info;
    uint64_t delta_time =
      ladar_info.mstruRadarHeaderData.mlTimeStamp > qr_info.time_stemp ?
      ladar_info.mstruRadarHeaderData.mlTimeStamp - qr_info.time_stemp :
      qr_info.time_stemp - ladar_info.mstruRadarHeaderData.mlTimeStamp;
    SLAM_INFO("delta time %ld", delta_time);
    create_qr_map_info_.push_back(create_map_info);
  }
  qr_ladar_file.close();
  return;
}



void FusionMap::_createScanMapAndLandMark(const karto::LocalizedRangeScan* scan,
  std::map<int, LandMark::Center>* land_mark_pos) {
  using namespace karto;  // NOLINT
  kt_double rangeThreshold = scan->GetLaserRangeFinder()->GetRangeThreshold();
  kt_double maxRange = scan->GetLaserRangeFinder()->GetMaximumRange();
  kt_double minRange = scan->GetLaserRangeFinder()->GetMinimumRange();

  Vector2<kt_double> scanPosition = scan->GetSensorPose().GetPosition();
  // LandMark::Point sensor_pos(scanPosition.GetX(), scanPosition.GetY());

  // get scan point readings
  const PointVectorDouble& rPointReadings = scan->GetPointReadings(false);

  // draw lines from scan position to all point readings
  int pointIndex = 0;

  gomros::message::RadarSensoryInfo radar_info;
  float theta = config_.mapping_start_angle * M_PI / 180.0f;
  float delta_theta = config_.laser_resolution * M_PI / 180.0f;
  const_forEachAs(PointVectorDouble, &rPointReadings, pointsIter) {
    Vector2<kt_double> point = *pointsIter;
    kt_double rangeReading = scan->GetRangeReadings()[pointIndex];
    kt_double intensities = scan->GetIntensitiesReadings()[pointIndex];
    bool isEndPointValid = rangeReading < (rangeThreshold - KT_TOLERANCE);
    Eigen::Vector2d temp_point;
    temp_point(0) = rangeReading * cos(theta);
    temp_point(1) = rangeReading * sin(theta);
    theta += delta_theta;
    radar_info.mstruSingleLayerData.mvPoints.push_back(temp_point);
    radar_info.mstruSingleLayerData.mvIntensities.push_back(intensities);

    if (rangeReading <= minRange ||
        rangeReading >= maxRange ||
        std::isnan(rangeReading)) {
      // ignore these readings
      pointIndex++;
      continue;
    } else if (rangeReading >= rangeThreshold) {
      // trace up to range reading
      kt_double ratio = rangeThreshold / rangeReading;
      kt_double dx = point.GetX() - scanPosition.GetX();
      kt_double dy = point.GetY() - scanPosition.GetY();
      point.SetX(scanPosition.GetX() + ratio * dx);
      point.SetY(scanPosition.GetY() + ratio * dy);
    }

    // 判断为反光柱点
    // 建图方式 1.0
    // if (intensities > config_.land_mark_config.landmark_intensities) {
    //   LandMark::Point candidate_point(point.GetX(), point.GetY());
      // if (pointIndex != 0 && pointIndex != (rPointReadings.size() - 1)) {
      //   if (intensities > scan->GetIntensitiesReadings()[pointIndex - 1] &&
      //       intensities > scan->GetIntensitiesReadings()[pointIndex + 1]) {
            // land_marks_calcuator_->AddLandMarkCenter(candidate_point);
        // }
      // }
    // }
    bool isInMap = _rayTrace(scanPosition, point, isEndPointValid);
    pointIndex++;
  }
  // 构建Landmarkmap
  // 建图方式 2.0
  LocationConfig location_config;
  location_config.land_mark_config = config_.land_mark_config;
  location_config.location_laser_resolution = config_.laser_resolution;
  std::vector<Eigen::Vector2d> landmark_point =
    LandMark::CalLandMarkCentor(radar_info, location_config.land_mark_config);
  float current_x = scan->GetSensorPose().GetX();
  float current_y = scan->GetSensorPose().GetY();
  float current_theta = scan->GetSensorPose().GetHeading();
  for (int i = 0; i < landmark_point.size(); i++) {
    LandMark::DbscanPoint candidate_point;
    // 转移至全局坐标系下
    candidate_point.x = landmark_point[i](0) * std::cos(current_theta) -
                    landmark_point[i](1) * std::sin(current_theta) + current_x;
    candidate_point.y = landmark_point[i](0) * std::sin(current_theta) +
                    landmark_point[i](1) * std::cos(current_theta) + current_y;
    candidate_point.clusterID = LandMark::UNCLASSIFIED;
    landmark_points_.push_back(candidate_point);
  }

  return;
}

void FusionMap::_createQrMap(
  const karto::LocalizedRangeScanVector& scan_points,
  std::vector<QRCoordinate>* qrs_world) {
  std::string odom_file_dir = "./mapping_data/odom_message.txt";
  std::string imu_file_dir = "./mapping_data/imu_message.txt";
  std::ifstream odom_file, imu_file;
  odom_file.open(odom_file_dir);
  imu_file.open(imu_file_dir);
  DataFusion::EKFCalcuator ekf_calcuator(config_.ekf_config, "fusion");

  karto::LocalizedRangeScan* last_scan = scan_points[0];

  Position init_pos;

  _ladar2Robot(Position(last_scan->GetTime(), last_scan->GetSensorPose().GetX(),
                        last_scan->GetSensorPose().GetY(),
                        last_scan->GetSensorPose().GetHeading()),
              &init_pos);

  Eigen::Matrix3d covariance;
  covariance.setZero(3, 3);
  ekf_calcuator.SetInitPos(init_pos, covariance);

  ImuSensoryMessage imu_msg;
  OdometerMessage last_odom_msg;

  const_forEach(karto::LocalizedRangeScanVector, &scan_points) {
    karto::LocalizedRangeScan* current_scan = *iter;
    ekf_calcuator.HandleImuData(imu_msg);
    ekf_calcuator.HandleOdomData(last_odom_msg);

    std::string line_info;
    while (getline(imu_file, line_info)) {
      std::vector<std::string> imu_data =
        internal_common::SplitCString(line_info, " ");
      imu_msg.time_stamp = atof(imu_data[0].c_str());
      imu_msg.z_omega = atof(imu_data[1].c_str());
      imu_msg.z_angle = atof(imu_data[2].c_str());
      imu_msg.forward_linear_accel = atof(imu_data[3].c_str());
      ekf_calcuator.HandleImuData(imu_msg);
      if (imu_msg.time_stamp > current_scan->GetTime()) break;
    }

    while (getline(odom_file, line_info)) {
      Position last_pos, now_pos;
      ekf_calcuator.GetCurrentState(&last_pos);

      OdometerMessage odom_msg;
      internal_common::GetOdomMessage(
        line_info, (OdomType)config_.odom_type, &odom_msg);
      ekf_calcuator.HandleOdomData(odom_msg);
      ekf_calcuator.Update();
      ekf_calcuator.GetCurrentState(&now_pos);
      if (_calQrScanMatch(last_pos, now_pos, last_odom_msg)) {
        _showMapLadar(last_scan);
        _showMapLadar(current_scan);
      }
      last_odom_msg = odom_msg;
      if (odom_msg.mclDeltaPosition.mlTimestamp > current_scan->GetTime())
        break;
    }
    last_scan = current_scan;
    _ladar2Robot(Position(last_scan->GetTime(),
                        last_scan->GetSensorPose().GetX(),
                        last_scan->GetSensorPose().GetY(),
                        last_scan->GetSensorPose().GetHeading()), &init_pos);
    ekf_calcuator.SetInitPos(init_pos, covariance);
  }
  _calQRCoordinateInMap(qrs_world);
  odom_file.close();
  imu_file.close();
  return;
}

void FusionMap::_ladar2Robot(const Position& ladar_world,
  Position* robot_word) {
  Eigen::Matrix3d in_robot(3, 3);
  in_robot <<
    cos(config_.sensor_mount.radar_position_theta),
    -sin(config_.sensor_mount.radar_position_theta),
    config_.sensor_mount.radar_position_x,
    sin(config_.sensor_mount.radar_position_theta),
    cos(config_.sensor_mount.radar_position_theta),
    config_.sensor_mount.radar_position_y,
    0, 0, 1.0;
  Eigen::Matrix3d ladar_world_mat(3, 3);
  ladar_world_mat <<
    cos(ladar_world.mfTheta), -sin(ladar_world.mfTheta), ladar_world.mfX,
    sin(ladar_world.mfTheta), cos(ladar_world.mfTheta), ladar_world.mfY,
    0, 0, 1.0;
  Eigen::Matrix3d robot_world = ladar_world_mat * in_robot.inverse();

  robot_word->mfX = robot_world(0, 2);
  robot_word->mfY = robot_world(1, 2);
  robot_word->mfTheta = SLAMMath::NormalizePITheta(
            atan2(robot_world(1, 0), robot_world(0, 0)));
  robot_word->mlTimestamp = ladar_world.mlTimestamp;
  return;
}



bool FusionMap::_calQrScanMatch(const Position& last_pos,
  const Position& now_pos, const OdometerMessage& last_odom_msg) {
  assert(last_pos.mlTimestamp <= now_pos.mlTimestamp);
  bool show_ladar = false;
  for (auto iter = create_qr_map_info_.begin();
      iter != create_qr_map_info_.end(); iter++) {
      uint64_t time_stamp = iter->ladar_info.mstruRadarHeaderData.mlTimeStamp;
      if (last_pos.mlTimestamp <= time_stamp &&
          now_pos.mlTimestamp > time_stamp) {
        if (iter->qr_info.tag_num == 20005 || iter->qr_info.tag_num == 20007) {
          SLAM_INFO("qr info %d", iter->qr_info.tag_num);
          show_ladar = true;
        }
        SLAM_DEBUG("qr tag %d last odom (%f %f)", iter->qr_info.tag_num,
                  last_odom_msg.mstruDiffSteerSts.mfLeftLinearVel,
                  last_odom_msg.mstruDiffSteerSts.mfRightLinearVel);
          iter->ladar_pos = _calTimePos(last_pos, last_odom_msg, time_stamp);
      }
      if (last_pos.mlTimestamp <= iter->qr_info.time_stemp &&
          now_pos.mlTimestamp > iter->qr_info.time_stemp) {
        SLAM_DEBUG("qr tag %d last odom (%f %f)", iter->qr_info.tag_num,
                  last_odom_msg.mstruDiffSteerSts.mfLeftLinearVel,
                  last_odom_msg.mstruDiffSteerSts.mfRightLinearVel);
          iter->qr_pos =
            _calTimePos(last_pos, last_odom_msg, iter->qr_info.time_stemp);
      }
  }
  return show_ladar;
}

void FusionMap::_calQRCoordinateInMap(std::vector<QRCoordinate>* qrs_world) {
  CreataQrMapAndCalibration create_qr_map_cal(config_, this);
  for (auto iter = create_qr_map_info_.begin();
      iter != create_qr_map_info_.end(); iter++) {
    if (iter->qr_pos.mlTimestamp == iter->qr_info.time_stemp &&
        iter->ladar_pos.mlTimestamp ==
        iter->ladar_info.mstruRadarHeaderData.mlTimeStamp) {
      create_qr_map_cal.GetQrMap(*iter, qrs_world);
    }
  }
  if (is_qr_calibration_) {
    create_qr_map_cal.GetQrCalibration();
  }
  return;
}



FusionMap::Position FusionMap::_calTimePos(const Position& last_pos,
  const OdometerMessage& odom_msg, uint64_t time) {
  DataFusion::EKFCalcuator ekf_cal(config_.ekf_config, "delta_pos");
  Eigen::Matrix3d covariance;
  covariance.setZero(3, 3);
  ekf_cal.SetInitPos(last_pos, covariance);
  OdometerMessage message = odom_msg;
  SLAM_DEBUG("delta time %ld", time - message.mclDeltaPosition.mlTimestamp);
  message.mclDeltaPosition.mlTimestamp = time;
  ekf_cal.HandleOdomData(message);
  Position now_pos;
  ekf_cal.GetCurrentState(&now_pos);
  return now_pos;
}



bool FusionMap::_rayTrace(const karto::Vector2<kt_double>& rWorldFrom,
  const karto::Vector2<kt_double>& rWorldTo,
  bool isEndPointValid) {
  using namespace karto;  // NOLINT
  assert(cell_pass_cnt_ != NULL && cell_hits_cnt_ != NULL);
  Vector2<kt_int32s> gridFrom = cell_pass_cnt_->WorldToGrid(rWorldFrom);
  Vector2<kt_int32s> gridTo = cell_pass_cnt_->WorldToGrid(rWorldTo);

  cell_pass_cnt_->TraceLine(gridFrom.GetX(), gridFrom.GetY(),
                            gridTo.GetX(), gridTo.GetY(), NULL);
  if (isEndPointValid) {
    if (cell_pass_cnt_->IsValidGridIndex(gridTo)) {
      kt_int32s index = cell_pass_cnt_->GridIndex(gridTo, false);

      kt_int32u* pCellPassCntPtr = cell_pass_cnt_->GetDataPointer();
      kt_int32u* pCellHitCntPtr = cell_hits_cnt_->GetDataPointer();

      // increment cell pass through and hit count
      pCellPassCntPtr[index]++;
      pCellHitCntPtr[index]++;
    }
  }
  return cell_pass_cnt_->IsValidGridIndex(gridTo);
}


void FusionMap::_updateScanMap() {
  assert(cell_pass_cnt_ != NULL && cell_hits_cnt_ != NULL);

  // clear grid
  Clear();

  // set occupancy status of cells
  kt_int8u* pDataPtr = GetDataPointer();
  kt_int32u* pCellPassCntPtr = cell_pass_cnt_->GetDataPointer();
  kt_int32u* pCellHitCntPtr = cell_hits_cnt_->GetDataPointer();

  kt_int32u nBytes = GetDataSize();
  for (kt_int32u i = 0; i < nBytes; i++, pDataPtr++,
       pCellPassCntPtr++, pCellHitCntPtr++) {
    _updateCell(pDataPtr, *pCellPassCntPtr, *pCellHitCntPtr);
  }
}

void FusionMap::_updateCell(kt_int8u* pCell,
  kt_int32u cellPassCnt, kt_int32u cellHitCnt) {
  if (cellPassCnt > min_pass_through_) {
    kt_double hitRatio =
      static_cast<kt_double>(cellHitCnt) / static_cast<kt_double>(cellPassCnt);
    if (hitRatio > occupancy_threshold_) {
      *pCell = karto::GridStates_Occupied;
    } else {
      *pCell = karto::GridStates_Free;
    }
  }
  return;
}


void FusionMap::_recordGridMap(const std::string& map_name) {
  kt_int32s width = GetWidth();
  kt_int32s height = GetHeight();
  karto::Vector2<kt_double> offset = GetCoordinateConverter()->GetOffset();
  MapInfo info;
  info.miMapHeight = height;
  info.miMapWidth = width;
  info.mdOriginXInWorld = offset.GetX();
  info.mdOriginYInWorld = offset.GetY();
  info.mdResolution = config_.mapping_resolution;

  SLAM_INFO("map info width = %d, height = %d, scale = %f, offset = (%f %f)\n",
            height, width , GetCoordinateConverter()->GetScale(),
            offset.GetX(), offset.GetY());
  SimpleGridMap grid_map(info);
  grid_map.datas.reserve(info.miMapWidth * info.miMapHeight);
  Json::Value map_header;
  Json::Value point_append;
  Json::Value atring_add;
  map_header["mapType"] = "Fusion_map";
  map_header["mapName"] = map_name;
  atring_add["x"] = info.mdOriginXInWorld;
  atring_add["y"] = info.mdOriginYInWorld;
  map_header["minPos"] = atring_add;
  atring_add["x"] = info.mdOriginXInWorld + config_.mapping_resolution * width;
  atring_add["y"] = info.mdOriginYInWorld + config_.mapping_resolution * height;
  map_header["maxPos"] = atring_add;
  map_header["resolution"] = config_.mapping_resolution;
  map_header["version"] = "1.2";
  map_json_["header"] = map_header;
  int index_cnt = 0;
  for (kt_int32s y = 0; y < height; y++) {
    for (kt_int32s x = 0; x < width; x++) {
      Json::Value point_xy;
      // Getting the value at position x,y
      kt_int8u value = GetValue(karto::Vector2<kt_int32s>(x, y));
      switch (value) {
        case karto::GridStates_Unknown:
          grid_map.datas.push_back(255);
          break;
        case karto::GridStates_Occupied:
          grid_map.datas.push_back(100);
          point_xy["x"] =
            info.mdOriginXInWorld + x * config_.mapping_resolution;
          point_xy["y"] =
            info.mdOriginYInWorld + y * config_.mapping_resolution;
          point_append[index_cnt] = point_xy;
          index_cnt++;
           break;
        case karto::GridStates_Free:
          grid_map.datas.push_back(0);
          break;
        default:
          break;
      }
    }
  }
  map_json_["normalPosList"] = point_append;
#ifdef TEST_DEBUG
  show_ladar_for_test_->DisPlayGridMap(grid_map);
#endif
  return;
}

std::map<int, LandMark::Center> FusionMap::_getLandmarkCenters() {
  int minPts = 2;
  float eps = config_.land_mark_config.landmark_radius * 4;
  LandMark::ReflectorDbscan dbscan(minPts, eps, landmark_points_);
  std::vector<LandMark::DbscanPoint> middle = dbscan.Run();  // 存在复制数据较多

  int id = 0;
  std::map<int, LandMark::Center> landmarks;
  std::unordered_map<int, std::vector<Eigen::Vector2d>> clustered_centers;
  for (auto point : middle) {
    SLAM_INFO("point.clusterID: %d (%f %f)", point.clusterID, point.x, point.y);
    if (point.clusterID > 0) {
      clustered_centers[point.clusterID].push_back(
        Eigen::Vector2d(point.x, point.y));
    } else {
      LandMark::Center center(point.x, point.y);
      center.is_cal_center = true;
      landmarks.insert(std::make_pair(id, center));
      id++;
    }
  }
  for (const auto &cluster : clustered_centers) {
    Eigen::Vector2d tmp_pose(0, 0);
    int n = cluster.second.size();
    for (const auto &center : cluster.second) {
      tmp_pose(0) += center(0) / n;
      tmp_pose(1) += center(1) / n;
    }
    LandMark::Center center(tmp_pose(0), tmp_pose(1));
    center.is_cal_center = true;
    landmarks.insert(std::make_pair(id, center));
    id++;
  }
  return landmarks;
}


void FusionMap::_recordMapInfoToFile(
  const std::map<int, LandMark::Center>& land_mark_map,
  const std::vector<QRCoordinate>& qr_map,
  const std::string& map_name) {
  // 显示qr位姿
  Json::Value qr_point;
  int qr_count = 0;

  for (auto iter = qr_map.begin(); iter != qr_map.end(); iter++) {
    Json::Value qr_info;
    qr_info["x"] = iter->mx;
    qr_info["y"] = iter->my;
    // qr_info["theta"] = qr_thetas_ave[iter->tag_num];
    qr_info["theta"] = iter->theta;
    qr_info["tag_num"] = iter->tag_num;
    qr_point[qr_count] = qr_info;
    SLAM_INFO("qr info %d (%f %f %f)",
      iter->tag_num, iter->mx, iter->my, iter->theta);
    qr_count++;
  }
  map_json_["qr_pos_list"] = qr_point;

  // 标记landmark信息
  Json::Value landmark_point;
  int landmark_id = 0;
  for (auto iter = land_mark_map.begin(); iter != land_mark_map.end(); iter++) {
    float mx = iter->second.mx;
    float my = iter->second.my;
    if (!iter->second.is_cal_center) continue;
    Json::Value landmark_info;
    landmark_info["x"] = mx;
    landmark_info["y"] = my;
    landmark_info["radius"] = config_.land_mark_config.landmark_radius;
    landmark_info["landmark_id"] = landmark_id;
    landmark_point[landmark_id] = landmark_info;
    landmark_id++;
  }

  map_json_["landmarks_pos_list"] = landmark_point;


  // grid_map.to_json_char_array(output);
  std::string full_map_data_path =
    config_.map_data_file_path + "/" +
    internal_common::CreatedMapFileName(map_name);
  internal_common::Save(full_map_data_path.c_str(), map_json_);
  SLAM_INFO("save fusion map to %s", full_map_data_path.c_str());
  usleep(200000);
  return;
}


void FusionMap::_showMapLadar(const karto::LocalizedRangeScan* scan) {
  using namespace karto;  // NOLINT
  kt_double rangeThreshold = scan->GetLaserRangeFinder()->GetRangeThreshold();
  kt_double maxRange = scan->GetLaserRangeFinder()->GetMaximumRange();
  kt_double minRange = scan->GetLaserRangeFinder()->GetMinimumRange();

  Vector2<kt_double> scanPosition = scan->GetSensorPose().GetPosition();
  LandMark::Point sensor_pos(scanPosition.GetX(), scanPosition.GetY(),
                            scanPosition.GetIntensities());

  // get scan point readings
  const PointVectorDouble& rPointReadings = scan->GetPointReadings(false);

  // draw lines from scan position to all point readings
  int pointIndex = 0;

  gomros::message::RadarSensoryInfo radar_info;
  float theta = config_.mapping_start_angle * M_PI / 180.0f;
  float delta_theta = config_.laser_resolution * M_PI / 180.0f;
  const_forEachAs(PointVectorDouble, &rPointReadings, pointsIter) {
    Vector2<kt_double> point = *pointsIter;
    kt_double rangeReading = scan->GetRangeReadings()[pointIndex];
    kt_double intensities = scan->GetIntensitiesReadings()[pointIndex];
    Eigen::Vector2d temp_point;
    temp_point(0) = rangeReading * cos(theta);
    temp_point(1) = rangeReading * sin(theta);
    theta += delta_theta;
    radar_info.mstruSingleLayerData.mvPoints.push_back(temp_point);
    radar_info.mstruSingleLayerData.mvIntensities.push_back(intensities);
    pointIndex++;
  }
  radar_info.mstruRadarHeaderData.mfXPos = scan->GetSensorPose().GetX();
  radar_info.mstruRadarHeaderData.mfYPos = scan->GetSensorPose().GetY();
  radar_info.mstruRadarHeaderData.mfTheta = scan->GetSensorPose().GetHeading();
  radar_info.mstruRadarHeaderData.mcPosValid = 1;
  show_ladar_for_test_->DisplayLadarMessage(radar_info);
  return;
}


}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros

