/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-06-26 09:01:27
 * @LastEditors: renjy
 * @LastEditTime: 2023-09-21 15:05:49
 */
#include <gtest/gtest.h>

#include "message_lib/radar_message.h"
#include "message_lib/odometer_message.h"
#include "ekf_calcuator/reflector_ekf.h"
#include "common/load_config.h"
#include "common/tool.h"
#include "common/logger.h"

#include "reflector_mapping/submap.h"
#include "reflector_mapping/reflector_map_builder.h"
#include "common/thread_pool.h"

class ReflectorEKFParam
    : public ::testing::TestWithParam<std::string> {};

TEST_P(ReflectorEKFParam, Mapping) {
  gomros::common::InitMaster(1);
  using namespace DataFusion;  // NOLINT
  using namespace gomros::data_process::mapping_and_location;  // NOLINT
  using namespace gomros::message;  // NOLINT
  std::string file_dir = GetParam();
  SLAM_INFO("file_dir %s", file_dir.c_str());

  // 加载配置文件
  MappingAndLocationConfig config;
  std::string config_dir = file_dir + "/mappingandlocation.json";
  if (!internal_common::ReadMappingAndLocationConfig(config_dir, &config)) {
    SLAM_ERROR("can not open config file, please check json!!");
    return;
  }
  ReflectorEkfCalcuator reflector_ekf_cal(
                                config.mapping_config.ekf_config,
                                config.mapping_config.sensor_mount,
                                config.mapping_config.land_mark_config);

  std::string odom_dir = file_dir + "/odom_message.txt";
  std::string ladar_dir = file_dir + "/ladar_message.smap";

  std::ifstream odom_file;
  std::ifstream ladar_file;

  odom_file.open(odom_dir);
  ladar_file.open(ladar_dir);
  std::string ladar_line;

  auto _transform2Ladar = [&](const std::vector<std::string>& ladar_data,
                                RadarSensoryInfo* ladar_message) {
    ladar_message->mstruRadarHeaderData.mlTimeStamp =
      atof(ladar_data[0].c_str());
    ladar_message->mstruSingleLayerData.mvPoints.clear();
    ladar_message->mstruSingleLayerData.mvIntensities.clear();
    float theta_acc = config.mapping_config.mapping_end_angle -
                      config.mapping_config.mapping_start_angle;
    int record_count =
      theta_acc / config.mapping_config.laser_resolution + 1;
    int read_id = 2;
    float xp, yp;
    float theta =
      config.mapping_config.mapping_start_angle * M_PI / 180.0f;
    float delta_theta =
      config.mapping_config.laser_resolution * M_PI / 180.0f;
    for (int i = 0; i < record_count; i++) {
      float len = atof(ladar_data[read_id++].c_str());
      xp = len * cos(theta);
      yp = len * sin(theta);
      theta += delta_theta;
      Eigen::Vector2d temp(xp, yp);
      ladar_message->mstruSingleLayerData.mvPoints.push_back(temp);
      ladar_message->mstruSingleLayerData.mvIntensities.push_back(
                              atof(ladar_data[read_id++].c_str()));
    }
  };

  DataFusion::State state;

  bool is_record_last_odom = false;
  bool is_first_odom = true;
  OdometerMessage last_odom_massage;
  int num_range_data_ = 0;
  while (getline(ladar_file, ladar_line)) {
    std::vector<std::string> ladar_data =
      internal_common::SplitCString(ladar_line, " ");
    RadarSensoryInfo ladar_message;
    _transform2Ladar(ladar_data, &ladar_message);
    std::string odom_line;
    if (is_record_last_odom) {
      is_record_last_odom = false;
      reflector_ekf_cal.HandleOdomData(last_odom_massage);
    }
    while (getline(odom_file, odom_line)) {
      // ekf 预测
      OdometerMessage odom_message;
      if (!internal_common::GetOdomMessage(odom_line,
          (OdomType)config.mapping_config.odom_type, &odom_message))
        continue;
      if (is_first_odom) {
        is_first_odom = false;
        State state;
        state.Init();
        state.time = odom_message.mclDeltaPosition.mlTimestamp;
        reflector_ekf_cal.SetInitState(state);
      }
      if (odom_message.mclDeltaPosition.mlTimestamp >
          ladar_message.mstruRadarHeaderData.mlTimeStamp) {
        is_record_last_odom = true;
        last_odom_massage = odom_message;
        break;
      }
      reflector_ekf_cal.HandleOdomData(odom_message);
    }
    DataFusion::Observation obs;
    obs.GetObsFromLadarData(config.mapping_config.sensor_mount,
      config.mapping_config.land_mark_config, ladar_message);
    if (is_first_odom) {
        is_first_odom = false;
        State state;
        state.Init();
        state.time = ladar_message.mstruRadarHeaderData.mlTimeStamp;
        reflector_ekf_cal.SetInitState(state);
    }
    reflector_ekf_cal.HandleObsData(obs);
    reflector_ekf_cal.GetState(&state);
    SLAM_DEBUG("SubMapId %d range_data_count %d submap_state %f %f %f",
            0, num_range_data_++, state.mu(0), state.mu(1), state.mu(2));
  }
  int reflector_size = (state.mu.rows() - 3) / 2;
  for (int i = 0; i < reflector_size; i++) {
    if (state.obs_in_map_count[i] < 10) continue;
    float x = state.mu(3 + 2 * i);
    float y = state.mu(3 + 2 * i + 1);
    SLAM_INFO("reflector_mapping index %d %f %f", i, x, y);
  }

  odom_file.close();
  ladar_file.close();
}

INSTANTIATE_TEST_CASE_P(Mapping, ReflectorEKFParam,
  ::testing::Values(
    "../../../../../Dataset/SLAM_Dataset/reflector_ekf_data/data0",
    "../../../../../Dataset/SLAM_Dataset/reflector_ekf_data/data1",
    "../../../../../Dataset/SLAM_Dataset/reflector_ekf_data/data2",
    "../../../../../Dataset/SLAM_Dataset/reflector_ekf_data/data3",
    "../../../../../Dataset/SLAM_Dataset/reflector_ekf_data/data4",
    "../../../../../Dataset/SLAM_Dataset/reflector_ekf_data/data5",
    "../../../../../Dataset/SLAM_Dataset/reflector_ekf_data/data6",
    "../../../../../Dataset/SLAM_Dataset/reflector_ekf_data/data7"
  )
);
