/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-14 19:37:32
 * @LastEditTime: 2023-11-07 14:41:51
 */


#include "fusion_mapping/fusion_mapping.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {


FusionMapping::FusionMapping(const MappingConfig& config)
  : MappingInterface(config.record_config) {
  mapping_config_ = config;
  // 配置karto mapper 相关参数配置
  spa_solver_ = nullptr;
  karto_mapper_ = nullptr;
  dataset_ = nullptr;
  _initKartoMapper();
}

FusionMapping::~FusionMapping() {
}


void FusionMapping::StartMapping() {
  sensor_data_recorder_->StartRecord();
  karto_mapper_->Reset();
  if (spa_solver_ != nullptr) {
    delete spa_solver_;
    spa_solver_ = nullptr;
  }
  if (dataset_ != nullptr) {
    delete dataset_;
    dataset_ = nullptr;
  }
  dataset_ = new karto::Dataset();
  spa_solver_ = new SpaSolver();
  karto_mapper_->SetScanSolver(spa_solver_);

  std::string name("laser0");
  laser_range_finder_ = karto::LaserRangeFinder::CreateLaserRangeFinder(
                  karto::LaserRangeFinder_Custom, karto::Name(name));
  laser_range_finder_->SetOffsetPose(
    karto::Pose2(mapping_config_.sensor_mount.radar_position_x,
    mapping_config_.sensor_mount.radar_position_y,
    mapping_config_.sensor_mount.radar_position_theta));
  laser_range_finder_->SetQrOffsetPose(
    karto::Pose2(mapping_config_.sensor_mount.camera_position_x,
    mapping_config_.sensor_mount.camera_position_y,
    mapping_config_.sensor_mount.camera_position_theta));
  laser_range_finder_->SetMinimumRange(mapping_config_.mapping_laser_min_range);
  laser_range_finder_->SetMaximumRange(mapping_config_.mapping_laser_max_range);
  laser_range_finder_->SetMinimumAngle(
    karto::math::DegreesToRadians(mapping_config_.mapping_start_angle));
  laser_range_finder_->SetMaximumAngle
    (karto::math::DegreesToRadians(mapping_config_.mapping_end_angle));
  laser_range_finder_->SetAngularResolution(
    karto::math::DegreesToRadians(mapping_config_.laser_resolution));
  dataset_->Add(laser_range_finder_);
  is_record_last_odom_ = false;
  is_record_last_imu_ = false;
}

void FusionMapping::StopMapping(const std::string& map_name) {
  sensor_data_recorder_->FinishRecord();
  // 开始离线建图
  _startFusionMapping(map_name);
}

void FusionMapping::_initKartoMapper() {
  if (spa_solver_ != nullptr) {
    delete spa_solver_;
    spa_solver_ = nullptr;
  }
  if (karto_mapper_ != nullptr) {
    delete karto_mapper_;
    karto_mapper_ = nullptr;
  }
  if (dataset_ != nullptr) {
    delete dataset_;
    dataset_ = nullptr;
  }

  karto_mapper_ = new karto::Mapper();
  dataset_ = new karto::Dataset();

  karto_mapper_->setParamUseScanMatching(true);
  karto_mapper_->setParamUselandmarkConstraints(false);
  karto_mapper_->setParamUseScanBarycenter(true);
  double time = 3600 * 1e6;
  karto_mapper_->setParamMinimumTimeInterval(time);
  karto_mapper_->setParamMinimumTravelDistance(0.2);
  karto_mapper_->setParamMinimumTravelHeading(0.4);  // 0.175
  karto_mapper_->setParamScanBufferSize(100);       // 70
  karto_mapper_->setParamScanBufferMaximumScanDistance(20.0);
  karto_mapper_->setParamLinkMatchMinimumResponseFine(0.8);
  karto_mapper_->setParamLinkScanMaximumDistance(10.0);
  karto_mapper_->setParamLoopSearchMaximumDistance(4.0);  // 4.0 避免误匹配
  karto_mapper_->setParamDoLoopClosing(true);
  karto_mapper_->setParamLoopMatchMinimumChainSize(20);  // 20 距离上进行
  karto_mapper_->setParamLoopMatchMaximumVarianceCoarse(0.9);
  karto_mapper_->setParamLoopMatchMinimumResponseCoarse(0.5);
  karto_mapper_->setParamLoopMatchMinimumResponseFine(0.6);
  karto_mapper_->setParamCorrelationSearchSpaceDimension(0.3);
  karto_mapper_->setParamCorrelationSearchSpaceResolution(0.01);
  karto_mapper_->setParamCorrelationSearchSpaceSmearDeviation(0.03);
  karto_mapper_->setParamLoopSearchSpaceDimension(9.0);
  karto_mapper_->setParamLoopSearchSpaceResolution(0.05);
  karto_mapper_->setParamLoopSearchSpaceSmearDeviation(0.03);
  karto_mapper_->setParamDistanceVariancePenalty(0.09);  // 0.09
  karto_mapper_->setParamAngleVariancePenalty(0.09);   // 0.349055 * 0.349055
  karto_mapper_->setParamFineSearchAngleOffset(0.00349);  // 0.00349
  karto_mapper_->setParamCoarseSearchAngleOffset(0.349);  // 0.349
  karto_mapper_->setParamCoarseAngleResolution(0.0349);  // 0.0349
  karto_mapper_->setParamMinimumAnglePenalty(0.9);
  karto_mapper_->setParamMinimumDistancePenalty(0.5);
  karto_mapper_->setParamUseResponseExpansion(false);

  spa_solver_ = new SpaSolver();
  karto_mapper_->SetScanSolver(spa_solver_);

  std::string name("laser0");
  laser_range_finder_ = karto::LaserRangeFinder::CreateLaserRangeFinder(
                  karto::LaserRangeFinder_Custom, karto::Name(name));
  laser_range_finder_->SetOffsetPose(
    karto::Pose2(mapping_config_.sensor_mount.radar_position_x,
    mapping_config_.sensor_mount.radar_position_y,
    mapping_config_.sensor_mount.radar_position_theta));
  laser_range_finder_->SetQrOffsetPose(
    karto::Pose2(mapping_config_.sensor_mount.camera_position_x,
    mapping_config_.sensor_mount.camera_position_y,
    mapping_config_.sensor_mount.camera_position_theta));
  laser_range_finder_->SetMinimumRange(mapping_config_.mapping_laser_min_range);
  laser_range_finder_->SetMaximumRange(mapping_config_.mapping_laser_max_range);
  laser_range_finder_->SetMinimumAngle(
    karto::math::DegreesToRadians(mapping_config_.mapping_start_angle));
  laser_range_finder_->SetMaximumAngle
    (karto::math::DegreesToRadians(mapping_config_.mapping_end_angle));
  laser_range_finder_->SetAngularResolution(
    karto::math::DegreesToRadians(mapping_config_.laser_resolution));
  dataset_->Add(laser_range_finder_);
  is_record_last_odom_ = false;
  is_record_last_imu_ = false;
}


void FusionMapping::_startFusionMapping(const std::string& map_name) {
  /*
  * 打开imu odom qr ladar的数据文件 根据时间戳进行处理
  * 1.EKF处理imu和odom数据
  * 2.处理ladar数据
  * 3.更新qr的建图信息
  */
  internal_common::RunTime runtime;
  std::ifstream ladar_file(sensor_data_recorder_->GetLadarFileDir().c_str());
  std::string ladar_line;
  karto::Pose2 pre_odom_data(0.0f, 0.0f, 0.0f);
  karto::Matrix3 pre_covariance;
  // 打开odom和imu的文件 为位姿预测做准备
  _openOdomAndImuFile();
  

  auto tranform = [this](const Eigen::Matrix3d& eigen_matrix,
        karto::Matrix3* karto_matrix) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        (*karto_matrix)(i, j) = eigen_matrix(i, j);
      }
    }
  };

  int count = 0;
  Position pre_pos;
  while (getline(ladar_file, ladar_line)) {
    count++;
    std::vector<std::string> ladar_data =
      internal_common::SplitCString(ladar_line, " ");
    uint64_t time_stemp = atof(ladar_data[0].c_str());
    if (count == 1) {
      pre_pos.mlTimestamp = time_stemp;
    }
    // 利用odom数据进行预测
    Eigen::Matrix3d covariance;
    covariance.setZero(3, 3);
    karto::Pose2 odom_pos =
      _calCurrentOdomPos(time_stemp, &pre_pos, &covariance);
    pre_odom_data = odom_pos;
    tranform(covariance, &pre_covariance);
    if (!_calScanMatchPos(ladar_data, odom_pos, &covariance)) {
      // // SLAM_ERROR("can not cal scan match");
    }
  }
  // 创建地图
  ladar_file.close();
  _closeOdomAndImuFile();
  usleep(1000);
  _createMap(map_name);
}


bool FusionMapping::_isMoveEnough(const karto::Pose2& pre_pos,
 const karto::Pose2& now_pos) {
  if (fabs(pre_pos.GetX() - now_pos.GetX()) <
        mapping_config_.record_config.distance_step &&
      fabs(pre_pos.GetY() - now_pos.GetY()) <
        mapping_config_.record_config.distance_step &&
      fabs(pre_pos.GetHeading() - now_pos.GetHeading()) <
          mapping_config_.record_config.angular_step) {
      return false;
  }
  return true;
}

karto::Pose2 FusionMapping::_calCurrentOdomPos(uint64_t time_stemp,
  Position* last_pos, Eigen::Matrix3d* covariance) {
  DataFusion::EKFCalcuator ekf_calcuator(
    mapping_config_.ekf_config, "fusion_mapping");
  ekf_calcuator.SetInitPos(*last_pos, *covariance);
  std::string odom_line;
  std::string imu_line;
  // // SLAM_DEBUG("!!!get last_pos = %f %f %f",
  //   last_pos->mfX, last_pos->mfY, last_pos->mfTheta);
  if (is_record_last_odom_) {
    is_record_last_odom_ = false;
    ekf_calcuator.HandleOdomData(last_odom_massage_);
  }
  while (getline(odom_file_, odom_line)) {
    // ekf 预测
    OdometerMessage odom_message;
    if (!internal_common::GetOdomMessage(odom_line,
        (OdomType)mapping_config_.odom_type, &odom_message))
      continue;
    if (odom_message.mclDeltaPosition.mlTimestamp > time_stemp) {
      is_record_last_odom_ = true;
      last_odom_massage_ = odom_message;
      break;
    }

    ekf_calcuator.HandleOdomData(odom_message);
    std::vector<ImuSensoryMessage> update_imu_datas;
    bool is_has_last_data = false;
    ImuSensoryMessage imu_message;
    if (is_record_last_imu_) {
      is_record_last_imu_ = false;
      update_imu_datas.push_back(last_imu_massage_);
    }
    while (getline(imu_file_, imu_line)) {
      std::vector<std::string> imu_data =
        internal_common::SplitCString(imu_line, " ");
      uint64_t imu_time_stemp = atof(imu_data[0].c_str());

      if (imu_time_stemp > odom_message.mclDeltaPosition.mlTimestamp) {
        is_record_last_imu_ = true;
        last_imu_massage_.time_stamp = imu_time_stemp;
        last_imu_massage_.z_omega = atof(imu_data[1].c_str());
        last_imu_massage_.z_angle = atof(imu_data[2].c_str());
        break;
      } else if (imu_time_stemp > last_pos->mlTimestamp &&
                 imu_time_stemp <= odom_message.mclDeltaPosition.mlTimestamp) {
        // 用于本次预测的数据
        imu_message.time_stamp = imu_time_stemp;
        imu_message.z_omega = atof(imu_data[1].c_str());
        imu_message.z_angle = atof(imu_data[2].c_str());
        update_imu_datas.push_back(imu_message);
      } else {
        // 小于上次预测位姿的数据不做处理了
        continue;
      }
    }
    for (auto iter = update_imu_datas.begin();
        iter != update_imu_datas.end(); iter++) {
        ekf_calcuator.HandleImuData(*iter, false);
    }
    ekf_calcuator.Update();
    update_imu_datas.clear();
  }
  // 获取位姿信息
  ekf_calcuator.GetCurrentState(last_pos, covariance);
  karto::Pose2 odom_pos(last_pos->mfX, last_pos->mfY, last_pos->mfTheta);
  return odom_pos;
}

bool FusionMapping::_calScanMatchPos(
  const std::vector<std::string>& ladar_data,
  const karto::Pose2& predicted_pos, Eigen::Matrix3d* covariance) {
  // 为karto准备工作
  // 读取激光的数据并纠正位姿 获取最新位姿及其协方差矩阵
  // 增加光强信息的读取
  std::vector<kt_double> scan_points;
  std::vector<kt_double> intensities;
  for (int i = 2; i < static_cast<int>(ladar_data.size());) {
    scan_points.push_back(atof((ladar_data[i++]).c_str()));
    intensities.push_back(atof((ladar_data[i++]).c_str()));
  }
  karto::LocalizedRangeScan* range_scan = new karto::LocalizedRangeScan(
    laser_range_finder_->GetName(), scan_points, intensities);

  range_scan->SetTime(atof(ladar_data[0].c_str()));
  range_scan->SetIntensitiesReadings(intensities);

  range_scan->SetOdometricPose(predicted_pos);
  range_scan->SetCorrectedPose(predicted_pos);

  // 进行karto 定位
  if (karto_mapper_->Process(range_scan)) {
    karto::Pose2 corrected_pose = range_scan->GetCorrectedPose();
    // Add the localized range scan to the dataset (for memory management)
    dataset_->Add(range_scan);
  } else {
    delete range_scan;
    return false;
  }
  // 获取当前位姿信息 这个位姿是激光对应的位姿 转换到odom坐标下
  karto::Pose2 sensor_pos = range_scan->GetCorrectedPose();

  // 获取协方差矩阵
  karto::Matrix3 cov = range_scan->GetCovariance();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      (*covariance)(i, j) = cov(i, j);
    }
  }
  return true;
}


void FusionMapping::_calLocatizedRangeScan(
  const std::vector<std::string>& ladar_data,
  const karto::Pose2& predicted_pos,
  karto::LocalizedRangeScan* range_scan) {
  std::vector<kt_double> scan_points;
  std::vector<kt_double> intensities;
  for (int i = 2; i < static_cast<int>(ladar_data.size());) {
    scan_points.push_back(atof((ladar_data[i++]).c_str()));
    intensities.push_back(atof((ladar_data[i++]).c_str()));
  }
  // // SLAM_DEBUG("scan_points size = %d, intensities size = %d",
  //             scan_points.size(), intensities.size());
  range_scan = new karto::LocalizedRangeScan(laser_range_finder_->GetName(),
                                                  scan_points, intensities);
  range_scan->SetTime(atof(ladar_data[0].c_str()));
  range_scan->SetIntensitiesReadings(intensities);

  // Eigen::Matrix3d transform_robot(3, 3);
  // transform_robot <<
  //   cos(predicted_pos.GetHeading()), -sin(predicted_pos.GetHeading()),
  //   predicted_pos.GetX(),
  //   sin(predicted_pos.GetHeading()), cos(predicted_pos.GetHeading()),
  //   predicted_pos.GetY(),
  //   0, 0, 1;
  // Eigen::Matrix3d ladar_pos(3, 3);
  // ladar_pos << 1, 0, mapping_config_.sensor_mount.radar_position_x,
  //              0, 1, mapping_config_.sensor_mount.radar_position_y,
  //              0, 0, 1;
  // Eigen::Matrix3d ladar_world = transform_robot * ladar_pos;

  // karto::Pose2 ladar_world_pos(ladar_world(0, 2), ladar_world(1, 2),
  //                              atan2(ladar_world(1, 0), ladar_world(0, 0)));

  karto::Pose2 ladar_world_pos(
    predicted_pos.GetX() + mapping_config_.sensor_mount.radar_position_x *
                           cos(predicted_pos.GetHeading()) -
                           mapping_config_.sensor_mount.radar_position_y *
                           sin(predicted_pos.GetHeading()),
    predicted_pos.GetY() + mapping_config_.sensor_mount.radar_position_x *
                           sin(predicted_pos.GetHeading()) +
                           mapping_config_.sensor_mount.radar_position_y *
                           cos(predicted_pos.GetHeading()),
    predicted_pos.GetHeading());

  range_scan->SetOdometricPose(ladar_world_pos);
  range_scan->SetCorrectedPose(ladar_world_pos);
}




void FusionMapping::_createMap(const std::string& slam_map_name) {
  // 创建地图时 将二维码和反光板的信息加入进去
  /*
  * 输入：激光雷达数据信息 分辨率信息 时间戳信息
  */
  // 打开二维码数据信息，将二维码地图插入进去
  // 反光板的数据提取出来
  using namespace karto;  // NOLINT
  LocalizedRangeScanVector all_scans =
    karto_mapper_->GetAllProcessedScans();
  if (all_scans.empty()) {
    // SLAM_ERROR("no scan points, can not create map!!!!!!!!!!!!!");
    return;
  }
  // 计算整张地图的大小
  kt_int32s map_width, map_height;
  Vector2<kt_double> map_offset;
  _computeDimensions(all_scans, mapping_config_.mapping_resolution,
                      &map_width, &map_height, &map_offset);
  // 建立地图信息
  // TODO(r) 需传入融合地图的配置信息
  FusionMap fusion_map(mapping_config_, map_width, map_height,
                      map_offset, is_qr_calibration_);
  fusion_map.CreateMapFromScans(all_scans, slam_map_name);
  return;
}

void FusionMapping::_computeDimensions(
  const karto::LocalizedRangeScanVector& rScans,
  const kt_double& resolution, kt_int32s* rWidth,
  kt_int32s* rHeight, karto::Vector2<kt_double>* rOffset) {
  using namespace karto;  // NOLINT
  // FILE* realpos_file = fopen("./slampos.txt", "w+");
  BoundingBox2 boundingBox;
  const_forEach(LocalizedRangeScanVector, &rScans) {
    boundingBox.Add((*iter)->GetBoundingBox());
    // karto::Pose2 slam_pos = (*iter)->GetCorrectedPose();
    // fprintf(realpos_file, "%f %f %f\n",
    //     slam_pos.GetX(), slam_pos.GetY(), slam_pos.GetHeading());
  }
  // fclose(realpos_file);
  kt_double scale = 1.0 / resolution;
  Size2<kt_double> size = boundingBox.GetSize();

  *rWidth = static_cast<kt_int32s>(math::Round(size.GetWidth() * scale));
  *rHeight = static_cast<kt_int32s>(math::Round(size.GetHeight() * scale));
  *rOffset = boundingBox.GetMinimum();
  return;
}



void FusionMapping::_openOdomAndImuFile() {
  odom_file_.open(sensor_data_recorder_->GetOdomFileDir());
  imu_file_.open(sensor_data_recorder_->GetImuFileDir());
}


void FusionMapping::_closeOdomAndImuFile() {
  odom_file_.close();
  imu_file_.close();
}


}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros

