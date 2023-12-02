/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-06-29 06:37:36
 * @LastEditors: renjy
 * @LastEditTime: 2023-10-18 15:43:34
 */

#include "fusion_mapping/qr_map_cal.h"
#include "include/mapping_and_location_math.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {


ceres::CostFunction* CreateOccupiedSpaceCostFunction2D(
    const double scaling_factor, const SensorMount& config,
    const gomros::message::RadarSensoryInfo& point_cloud,
    const karto::Grid<kt_int8u>& grid) {
  return new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunction2D,
                                         ceres::DYNAMIC /* residuals */,
                                         3 /* pose variables */>(
    new OccupiedSpaceCostFunction2D(scaling_factor, config, point_cloud, grid),
      // 比固定残差维度的 多了一个参数
      point_cloud.mstruSingleLayerData.mvPoints.size());
}


CreataQrMapAndCalibration::CreataQrMapAndCalibration(
  const MappingConfig& config, const karto::Grid<kt_int8u>* map_info)
: config_(config), map_info_(map_info) {
  show_ladar_ = std::make_shared<
    DisPlayResult::SimulationDataPublisher>(config.sensor_mount);
}


CreataQrMapAndCalibration::~CreataQrMapAndCalibration() {}

void CreataQrMapAndCalibration::GetQrMap(const CreateQrMapInfo& create_map_info,
  std::vector<QRCoordinate>* qrs_world) {
  Position pose_estimate;
  _scanMatch(create_map_info.ladar_pos, create_map_info.ladar_info,
            *map_info_, &pose_estimate);
  SLAM_INFO("qr id %d init_pos (%f %f %f), pose_estimate (%f %f %f)",
            create_map_info.qr_info.tag_num,
            create_map_info.ladar_pos.mfX, create_map_info.ladar_pos.mfY,
            create_map_info.ladar_pos.mfTheta,
            pose_estimate.mfX, pose_estimate.mfY, pose_estimate.mfTheta);
  _calQrWorldPos(pose_estimate, create_map_info, qrs_world);

#ifdef TEST_DEBUG
  RadarSensoryMessage message;
  message.mstruRadarMessage = create_map_info.ladar_info;
  // show_ladar_->DisplayLadarMessage(message, pose_estimate);
#endif
  return;
}


void CreataQrMapAndCalibration::GetQrCalibration() {
  if (calibration_qrs_info_.size() < 3) {
    SLAM_WARN("qr size is %d, less 3, cannot do calibartion",
      calibration_qrs_info_.size());
    return;
  }
  // FIXME(r) 判断是否获取到真实坐标信息
  std::ifstream infile;
  std::string file_name =
    "../../../../../Dataset/SLAM_Dataset/config/qr_infos.txt";
  infile.open(file_name.c_str(), std::ifstream::in);
  if (!infile.is_open()) {
    SLAM_ERROR("can not open %s, cannot do calibartion", file_name.c_str());
    return;
  }
  std::string line;
  QRCoordinate qr_coordinate;
  std::map<int, QRCoordinate> qrs_coordinate_map;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    if (!(iss >> qr_coordinate.tag_num >> qr_coordinate.mx
              >> qr_coordinate.my >> qr_coordinate.theta)) {
      break;
    }
    qrs_coordinate_map.insert(
      std::make_pair(qr_coordinate.tag_num, qr_coordinate));
  }
  std::vector<cv::Mat> R_gripper2base;
  std::vector<cv::Mat> T_gripper2base;
  std::vector<cv::Mat> R_target2cam;
  std::vector<cv::Mat> T_target2cam;
  for (int i = 0; i < calibration_qrs_info_.size(); i++) {
    if (!qrs_coordinate_map.count(calibration_qrs_info_[i].qr_info.tag_num))
      continue;
    QRCoordinate qr1_coor =
      qrs_coordinate_map.at(calibration_qrs_info_[i].qr_info.tag_num);
    CalibrationQrInfo qr1_info = calibration_qrs_info_[i];
    for (int j = i + 1; j < calibration_qrs_info_.size(); j++) {
      CalibrationQrInfo qr2_info = calibration_qrs_info_[j];
      if (!qrs_coordinate_map.count(calibration_qrs_info_[j].qr_info.tag_num))
        continue;
      QRCoordinate qr2_coor =
        qrs_coordinate_map.at(calibration_qrs_info_[j].qr_info.tag_num);
      cv::Mat T_A, R_A, T_B, R_B;
      _calHandEyeMat(qr1_coor, qr2_coor, qr1_info, qr2_info,
                     &T_A, &R_A, &T_B, &R_B);
      R_gripper2base.push_back(R_A);
      T_gripper2base.push_back(T_A);
      R_target2cam.push_back(R_B);
      T_target2cam.push_back(T_B);
    }
  }
  cv::Mat R_cam2gripper = cv::Mat::eye(3, 3, CV_64FC1);
  cv::Mat T_cam2gripper = cv::Mat::eye(3, 1, CV_64FC1);


  cv::calibrateHandEye(R_gripper2base, T_gripper2base, R_target2cam,
    T_target2cam, R_cam2gripper, T_cam2gripper);
  for (int i = 0; i < R_gripper2base.size(); i++) {
      std::cout << R_gripper2base[i] << std::endl;
      std::cout << T_gripper2base[i] << std::endl;
      std::cout << R_target2cam[i] << std::endl;
      std::cout << T_target2cam[i] << std::endl;
  }

  std::cout << R_cam2gripper << std::endl;
  std::cout << T_cam2gripper << std::endl;

  float theta = SLAMMath::NormalizePITheta(
                atan2(R_cam2gripper.at<double>(1, 0),
                      R_cam2gripper.at<double>(0, 0)));
  float x = T_cam2gripper.at<double>(0, 0);
  float y = T_cam2gripper.at<double>(1, 0);

  SLAM_INFO("calibrateHandEye result (%f %f %f)", x, y, theta);
  return;
}

void CreataQrMapAndCalibration::_calHandEyeMat(const QRCoordinate& qr1_coor,
  const QRCoordinate& qr2_coor, const CalibrationQrInfo& qr1_info,
  const CalibrationQrInfo& qr2_info, cv::Mat* T_A, cv::Mat* R_A,
  cv::Mat* T_B, cv::Mat* R_b) {
  Eigen::Matrix3d W_qr1_mat(3, 3);
  W_qr1_mat <<
  cos(qr1_coor.theta), -sin(qr1_coor.theta), qr1_coor.mx,
  sin(qr1_coor.theta), cos(qr1_coor.theta), qr1_coor.my,
  0, 0, 1;
  Eigen::Matrix3d W_qr2_mat(3, 3);
  W_qr2_mat <<
  cos(qr2_coor.theta), -sin(qr2_coor.theta), qr2_coor.mx,
  sin(qr2_coor.theta), cos(qr2_coor.theta), qr2_coor.my,
  0, 0, 1;

  Eigen::Matrix3d delta_mat = W_qr1_mat.inverse() * W_qr2_mat;

  Eigen::Matrix3d Q_robot1(3, 3);
  Q_robot1 <<
    cos(qr1_info.qr_info.pos_theta),
    -sin(qr1_info.qr_info.pos_theta),
    qr1_info.qr_info.pos_x,
    sin(qr1_info.qr_info.pos_theta),
    cos(qr1_info.qr_info.pos_theta),
    qr1_info.qr_info.pos_y,
    0, 0, 1;
  Eigen::Matrix3d Q_robot2(3, 3);
  Q_robot2 <<
    cos(qr2_info.qr_info.pos_theta),
    -sin(qr2_info.qr_info.pos_theta),
    qr2_info.qr_info.pos_x,
    sin(qr2_info.qr_info.pos_theta),
    cos(qr2_info.qr_info.pos_theta),
    qr2_info.qr_info.pos_y,
    0, 0, 1;

  Eigen::Matrix3d B = Q_robot2.inverse() * delta_mat * Q_robot1;

  _homogeneousMtr2RT(B, R_b, T_B);

  Eigen::Matrix3d W_robot1_mat(3, 3);
  W_robot1_mat <<
    cos(qr1_info.robot_pos.mfTheta),
    -sin(qr1_info.robot_pos.mfTheta),
    qr1_info.robot_pos.mfX,
    sin(qr1_info.robot_pos.mfTheta),
    cos(qr1_info.robot_pos.mfTheta),
    qr1_info.robot_pos.mfY,
    0, 0, 1;

  Eigen::Matrix3d W_robot2_mat(3, 3);
  W_robot2_mat <<
    cos(qr2_info.robot_pos.mfTheta),
    -sin(qr2_info.robot_pos.mfTheta),
    qr2_info.robot_pos.mfX,
    sin(qr2_info.robot_pos.mfTheta),
    cos(qr2_info.robot_pos.mfTheta),
    qr2_info.robot_pos.mfY,
    0, 0, 1;
  Eigen::Matrix3d A = W_robot2_mat.inverse() * W_robot1_mat;

  _homogeneousMtr2RT(A, R_A, T_A);

  return;
}

void CreataQrMapAndCalibration::_homogeneousMtr2RT(
  const Eigen::Matrix3d& mat, cv::Mat* r, cv::Mat* t) {
  cv::Mat homo_mtr = cv::Mat::eye(4, 4, CV_64FC1);
  homo_mtr.at<double>(0, 0) = mat(0, 0);
  homo_mtr.at<double>(0, 1) = mat(0, 1);
  homo_mtr.at<double>(0, 2) = 0;
  homo_mtr.at<double>(0, 3) = mat(0, 2);
  homo_mtr.at<double>(1, 0) = mat(1, 0);
  homo_mtr.at<double>(1, 1) = mat(1, 1);
  homo_mtr.at<double>(1, 2) = 0;
  homo_mtr.at<double>(1, 3) = mat(1, 2);

  homo_mtr.at<double>(2, 0) = 0;
  homo_mtr.at<double>(2, 1) = 0;
  homo_mtr.at<double>(2, 2) = 1;
  homo_mtr.at<double>(2, 3) = 0;

  homo_mtr.at<double>(3, 0) = 0;
  homo_mtr.at<double>(3, 1) = 0;
  homo_mtr.at<double>(3, 2) = 0;
  homo_mtr.at<double>(3, 3) = 1;

  cv::Rect R_rect(0, 0, 3, 3);
  cv::Rect T_rect(3, 0, 1, 3);
  *r = homo_mtr(R_rect);
  *t = homo_mtr(T_rect);
  return;
}



// scan match
void CreataQrMapAndCalibration::_scanMatch(const Position& init_pos,
  const RadarSensoryInfo& point_cloud,
  const karto::Grid<kt_int8u>& map_info,
  Position* pose_estimate) {
  double ceres_pose_estimate[3] = {init_pos.mfX,
                                   init_pos.mfY,
                                   init_pos.mfTheta};
  ceres::Problem problem;

  // 地图部分的残差
  problem.AddResidualBlock(
      CreateOccupiedSpaceCostFunction2D(
          option_.occupied_space_cost_factor,
          config_.sensor_mount,
          point_cloud, map_info),
      nullptr /* loss function */, ceres_pose_estimate);

  // 平移的残差
  Eigen::Vector2d target_translation;
  target_translation(0) = init_pos.mfX;
  target_translation(1) = init_pos.mfY;
  problem.AddResidualBlock(
      TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
           // 平移的目标值, 没有使用校准后的平移
          option_.translation_delta_cost_factor, target_translation),
      nullptr /* loss function */, ceres_pose_estimate);      // 平移的初值

  // 旋转的残差, 固定了角度不变
  problem.AddResidualBlock(
      RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          // 角度的目标值
          option_.rotation_delta_cost_factor, ceres_pose_estimate[2]),
      nullptr /* loss function */, ceres_pose_estimate);       // 角度的初值

  // 根据配置进行求解
  ceres::Solver::Options ceres_solver_options;
  ceres_solver_options.max_num_iterations = option_.max_num_iterations;
  ceres_solver_options.num_threads = option_.num_threads;
  ceres_solver_options.use_nonmonotonic_steps = option_.use_nonmonotonic_steps;
  ceres::Solver::Summary summary;

  ceres::Solve(ceres_solver_options, &problem, &summary);

  pose_estimate->mlTimestamp = point_cloud.mstruRadarHeaderData.mlTimeStamp;
  pose_estimate->mfX = ceres_pose_estimate[0];
  pose_estimate->mfY = ceres_pose_estimate[1];
  pose_estimate->mfTheta = SLAMMath::NormalizePITheta(ceres_pose_estimate[2]);
}

void CreataQrMapAndCalibration::_calQrWorldPos(const Position& pose_estimate,
  const CreateQrMapInfo& create_map_info,
  std::vector<QRCoordinate>* qrs_world) {
  QRCoordinate qr_world;
  qr_world.tag_num = create_map_info.qr_info.tag_num;
  // qr_pos
  Position qr_pos;
  Eigen::Matrix3d ladar_pos_mat(3, 3);
  float ladar_pos_theta = create_map_info.ladar_pos.mfTheta;
  ladar_pos_mat <<
    cos(ladar_pos_theta), -sin(ladar_pos_theta), create_map_info.ladar_pos.mfX,
    sin(ladar_pos_theta), cos(ladar_pos_theta), create_map_info.ladar_pos.mfY,
    0, 0, 1;
  Eigen::Matrix3d qr_pos_mat(3, 3);
  float qr_pos_theta = create_map_info.qr_pos.mfTheta;
  qr_pos_mat <<
    cos(qr_pos_theta), -sin(qr_pos_theta), create_map_info.qr_pos.mfX,
    sin(qr_pos_theta), cos(qr_pos_theta), create_map_info.qr_pos.mfY,
    0, 0, 1;
  Eigen::Matrix3d delta_pos_mat = ladar_pos_mat.inverse() * qr_pos_mat;
  SLAM_INFO("delta pos %f %f", delta_pos_mat(0, 2), delta_pos_mat(1, 2));
  ladar_pos_mat <<
    cos(pose_estimate.mfTheta), -sin(pose_estimate.mfTheta), pose_estimate.mfX,
    sin(pose_estimate.mfTheta), cos(pose_estimate.mfTheta), pose_estimate.mfY,
    0, 0, 1;

  Eigen::Matrix3d robot_world = ladar_pos_mat * delta_pos_mat;
  // 插入标定数据
  CalibrationQrInfo calibration_info;
  calibration_info.qr_info = create_map_info.qr_info;
  calibration_info.robot_pos.mfX = robot_world(0, 2);
  calibration_info.robot_pos.mfY = robot_world(1, 2);
  calibration_info.robot_pos.mfTheta =
    SLAMMath::NormalizePITheta(atan2(robot_world(1, 0), robot_world(0, 0)));;
  calibration_qrs_info_.push_back(calibration_info);

  // 相机在qr坐标系下的坐标
  Eigen::Matrix3d robot_qr(3, 3);
  robot_qr <<
    cos(create_map_info.qr_info.pos_theta),
    -sin(create_map_info.qr_info.pos_theta), create_map_info.qr_info.pos_x,
    sin(create_map_info.qr_info.pos_theta),
    cos(create_map_info.qr_info.pos_theta), create_map_info.qr_info.pos_y,
    0, 0, 1;
  Eigen::Matrix3d camera_robot(3, 3);
  float theta = config_.sensor_mount.camera_position_theta * M_PI;
  camera_robot <<
    cos(theta), -sin(theta), config_.sensor_mount.camera_position_x,
    sin(theta), cos(theta), config_.sensor_mount.camera_position_y,
    0, 0, 1;
  Eigen::Matrix3d qr(3, 3);
  qr = robot_world * camera_robot * robot_qr.inverse();
  qr_world.mx = qr(0, 2);
  qr_world.my = qr(1, 2);
  qr_world.theta = SLAMMath::NormalizePITheta(atan2(qr(1, 0), qr(0, 0)));
  qrs_world->push_back(qr_world);
}


}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros

