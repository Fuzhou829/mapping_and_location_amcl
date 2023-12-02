/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-22 14:51:22
 * @LastEditTime: 2023-08-07 18:32:45
 */

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <fstream>
#include <istream>

#include "common_lib/node.h"
#include "message_lib/imu_message.h"
#include "include/config_struct.h"
#include "ekf_calcuator/ekf_calcuator.h"

#include "common/load_config.h"

#include "common/tool.h"
#include "common/logger.h"



class EKFParam
    : public ::testing::TestWithParam<std::string> {};

TEST_P(EKFParam, TrackReckoning_with_imu) {
  gomros::common::InitMaster(1);
  using namespace gomros::data_process::mapping_and_location;  // NOLINT
  std::string data_dir = GetParam();
  SLAM_INFO("data_dir %s", data_dir.c_str());
  MappingAndLocationConfig config;
  std::string config_dir = data_dir + "/mappingandlocation.json";
  if (!internal_common::ReadMappingAndLocationConfig(config_dir, &config)) {
    SLAM_ERROR("can not open config file, please check json!!");
    return;
  }
  gomros::data_process::mapping_and_location::EKFConfig ekf_config =
    config.location_config.ekf_config;
  DataFusion::EKFCalcuator ekf_cal(ekf_config, "TrackReckoning_with_imu");

  FILE* pos_file;
  std::string odom_pos_dir = data_dir + "/odom_pos.txt";
  pos_file = fopen(odom_pos_dir.c_str(), "w+");

  std::string imu_file_name = data_dir + "/imu_message.txt";
  std::string odom_file_name = data_dir + "/odom_message.txt";

  std::ifstream imu_infile, odom_infile;
  imu_infile.open(imu_file_name.c_str(), std::ifstream::in);
  odom_infile.open(odom_file_name.c_str(), std::ifstream::in);

  std::string line_read;

  // 处理odom数据
  bool get_first_data = false;
  while (1) {
    if (getline(odom_infile, line_read)) {
      gomros::message::OdometerMessage odom_message;
      if (!internal_common::GetOdomMessage(
        line_read, OdomType(ekf_config.odom_type), &odom_message)) continue;
      if (!get_first_data) {
        Eigen::Matrix3d covariance;
        covariance.setZero(3, 3);
        gomros::message::Position init_pos(
          odom_message.mclDeltaPosition.mlTimestamp, 0, 0, 0);
        ekf_cal.SetInitPos(init_pos, covariance);
        get_first_data = true;
        // 放置imu数据
        while (1) {
          if (getline(imu_infile, line_read)) {
            gomros::message::ImuSensoryMessage imu_message;
            std::vector<std::string> imu_str =
              internal_common::SplitCString(line_read, " ");
            imu_message.time_stamp = atof(imu_str[0].c_str());
            imu_message.z_omega = atof(imu_str[1].c_str());
            imu_message.z_angle = atof(imu_str[2].c_str());
            imu_message.forward_linear_accel = atof(imu_str[3].c_str());
            ekf_cal.HandleImuData(imu_message, false);
          } else {
            break;
          }
        }
      }
      ekf_cal.HandleOdomData(odom_message);
      ekf_cal.Update();
      gomros::message::Position pos;
      ekf_cal.GetCurrentState(&pos);
      SLAM_INFO("current state is %d, current_pose (%f %f %f)",
              1, pos.mfX, pos.mfY, pos.mfTheta);
      fprintf(pos_file, "%f %f %f\n", pos.mfX, pos.mfY, pos.mfTheta);
    } else {
      break;
    }
  }
  fclose(pos_file);
}

INSTANTIATE_TEST_CASE_P(TrackReckoningWithImutest, EKFParam,
  ::testing::Values(
    "../../../../../Dataset/SLAM_Dataset/ekf_data/data0",
    "../../../../../Dataset/SLAM_Dataset/ekf_data/data1",
    "../../../../../Dataset/SLAM_Dataset/ekf_data/data2",
    "../../../../../Dataset/SLAM_Dataset/ekf_data/data3",
    "../../../../../Dataset/SLAM_Dataset/ekf_data/data4",
    "../../../../../Dataset/SLAM_Dataset/ekf_data/data5",
    "../../../../../Dataset/SLAM_Dataset/ekf_data/data6",
    "../../../../../Dataset/SLAM_Dataset/ekf_data/data7"
  )
);


class TrackReckoningParam
    : public ::testing::TestWithParam<std::string> {};

TEST_P(TrackReckoningParam, TrackReckoning) {
  gomros::common::InitMaster(1);
  using namespace gomros::data_process::mapping_and_location;  // NOLINT
  std::string data_dir = GetParam();
  SLAM_INFO("data_dir %s", data_dir.c_str());
  MappingAndLocationConfig config;
  std::string config_dir = data_dir + "/mappingandlocation.json";
  if (!internal_common::ReadMappingAndLocationConfig(config_dir, &config)) {
    SLAM_ERROR("can not open config file, please check json!!");
    return;
  }
  gomros::data_process::mapping_and_location::EKFConfig ekf_config =
    config.location_config.ekf_config;
  DataFusion::EKFCalcuator ekf_cal(ekf_config, "TrackReckoning");

  FILE* pos_file;
  std::string odom_pos_dir = data_dir + "/odom_pos.txt";
  pos_file = fopen(odom_pos_dir.c_str(), "w+");

  std::string odom_file_name = data_dir + "/odom_message.txt";

  std::ifstream odom_infile;
  odom_infile.open(odom_file_name.c_str(), std::ifstream::in);

  std::string line_read;
  // 处理odom数据
  bool get_first_data = false;
  while (1) {
    if (getline(odom_infile, line_read)) {
      gomros::message::OdometerMessage odom_message;
      if (!internal_common::GetOdomMessage(
        line_read, OdomType(ekf_config.odom_type), &odom_message)) continue;
      if (!get_first_data) {
        Eigen::Matrix3d covariance;
        covariance.setZero(3, 3);
        gomros::message::Position init_pos(
          odom_message.mclDeltaPosition.mlTimestamp, 0, 0, 0);
        ekf_cal.SetInitPos(init_pos, covariance);
        get_first_data = true;
      }
      ekf_cal.HandleOdomData(odom_message);
      ekf_cal.Update();
      gomros::message::Position pos;
      ekf_cal.GetCurrentState(&pos);
      SLAM_INFO("current state is %d, current_pose (%f %f %f)",
              1, pos.mfX, pos.mfY, pos.mfTheta);
      fprintf(pos_file, "%f %f %f\n", pos.mfX, pos.mfY, pos.mfTheta);
    } else {
      break;
    }
  }
  fclose(pos_file);
}

INSTANTIATE_TEST_CASE_P(TrackReckoningtest, TrackReckoningParam,
  ::testing::Values(
    "../../../../../Dataset/SLAM_Dataset/ekf_data/data0",
    "../../../../../Dataset/SLAM_Dataset/ekf_data/data1",
    "../../../../../Dataset/SLAM_Dataset/ekf_data/data2",
    "../../../../../Dataset/SLAM_Dataset/ekf_data/data3",
    "../../../../../Dataset/SLAM_Dataset/ekf_data/data4",
    "../../../../../Dataset/SLAM_Dataset/ekf_data/data5",
    "../../../../../Dataset/SLAM_Dataset/ekf_data/data6",
    "../../../../../Dataset/SLAM_Dataset/ekf_data/data7"
  )
);

