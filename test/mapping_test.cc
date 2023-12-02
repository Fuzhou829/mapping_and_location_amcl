/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-22 14:51:22
 * @LastEditTime: 2023-09-25 14:45:32
 */

#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "mock/sensor_data_simulation.h"
#include "common/load_config.h"

#include "include/config_struct.h"
#include "mapping_and_location/mapping_and_location.h"
#include "mock/display_location_result.h"
#include "landmark_tool/trilateration.h"
#include "common/logger.h"

// 纯反光柱参数化单元测试
class ReflectorMappingParam
    : public ::testing::TestWithParam<std::string> {};


TEST_P(ReflectorMappingParam, ReflectorMapping) {
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
  std::shared_ptr<MappingAndLocation> mapping_and_location =
    std::make_shared<MappingAndLocation>(config_dir);
  std::shared_ptr<Simulation::SensorData> sensor_data_publish =
    std::make_shared<Simulation::SensorData>(config, data_dir);
  std::shared_ptr<Display::LocationResult> display_result =
    std::make_shared<Display::LocationResult>(config);
  while (!sensor_data_publish->IsFinishSimulation() ||
        !display_result->IsFinishedDisply()) {
    usleep(20000);
  }
  usleep(2000000);
  return;
}

INSTANTIATE_TEST_CASE_P(Mapping, ReflectorMappingParam,
  ::testing::Values(
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data0",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data1",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data2",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data3",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data4",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data5",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data6",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data7",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data8",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data9",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data10",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data11",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data12",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data13",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data14",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data15",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data16",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data17",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data18",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data19",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data20",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data21",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data22",
    "../../../../../Dataset/SLAM_Dataset/reflector_mapping/data23"
  )
);


// 融合建图参数化单元测试
class FusionMappingParam
    : public ::testing::TestWithParam<std::string> {};


TEST_P(FusionMappingParam, FusionMapping) {
  // Eigen::Matrix3d robot_world(3, 3);
  // robot_world <<
  //   cos(0.5 * M_PI), -sin(0.5 * M_PI), 0,
  //   sin(0.5 * M_PI), cos(0.5 * M_PI), 0,
  //   0, 0, 1;
  // Eigen::Matrix3d qr_world(3, 3);
  // qr_world <<
  //   cos(0), -sin(0), 0,
  //   sin(0), cos(0), 0,
  //   0, 0, 1;
  // Eigen::Matrix3d qr_info(3, 3);
  // qr_info <<
  //   cos(0.5 * M_PI), -sin(0.5 * M_PI), 0,
  //   sin(0.5 * M_PI), cos(0.5 * M_PI), 0.009,
  //   0, 0, 1;
  // Eigen::Matrix3d carme_robot(3, 3);
  // carme_robot = robot_world.inverse() * qr_world * qr_info;
  // double x  = carme_robot(0, 2);
  // double y = carme_robot(1, 2);
  // double t =
  //   SLAMMath::NormalizePITheta(
  //     atan2(carme_robot(1, 0), carme_robot(0, 0))) / M_PI;


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
  std::shared_ptr<MappingAndLocation> mapping_and_location =
    std::make_shared<MappingAndLocation>(config_dir);
  std::shared_ptr<Simulation::SensorData> sensor_data_publish =
    std::make_shared<Simulation::SensorData>(config, data_dir);
  std::shared_ptr<Display::LocationResult> display_result =
    std::make_shared<Display::LocationResult>(config);
  while (!sensor_data_publish->IsFinishSimulation() ||
        !display_result->IsFinishedDisply()) {
    usleep(20000);
  }
  usleep(2000000);
  return;
}

INSTANTIATE_TEST_CASE_P(Mapping, FusionMappingParam,
  ::testing::Values(
    "../../../../../Dataset/SLAM_Dataset/fusion_mapping/data0",
    "../../../../../Dataset/SLAM_Dataset/fusion_mapping/data1",
    "../../../../../Dataset/SLAM_Dataset/fusion_mapping/data2",
    "../../../../../Dataset/SLAM_Dataset/fusion_mapping/data3",
    "../../../../../Dataset/SLAM_Dataset/fusion_mapping/data4",
    "../../../../../Dataset/SLAM_Dataset/fusion_mapping/data5",
    "../../../../../Dataset/SLAM_Dataset/fusion_mapping/data6",
    "../../../../../Dataset/SLAM_Dataset/fusion_mapping/data7",
    "../../../../../Dataset/SLAM_Dataset/fusion_mapping/data8",
    "../../../../../Dataset/SLAM_Dataset/fusion_mapping/data9"
  )
);


TEST(Precision, trilateral_wanji) {
  using namespace gomros::data_process::mapping_and_location;  // NOLINT
  std::string map_dir =
    "../../../../../Dataset/SLAM_Dataset/Precision/";
  std::string wanji_dir = map_dir + "wanji.txt";
  std::string trilateral_dir = map_dir + "ale.smap";

  std::vector<Eigen::Vector2d> wanji_map, trilateral_map;
  // 加载三边定位地图
  char filename[128];
  SLAM_DEBUG("load map dir is %s", trilateral_dir.c_str());
  std::string map_data;
  if (!internal_common::Read(trilateral_dir.c_str(), &map_data)) {
    SLAM_ERROR("read map file failed....");
    return;
  }
  Json::Reader reader;
  Json::Value map_json;
  if (!reader.parse(map_data, map_json)) {
    SLAM_ERROR("parse map failed.....");
    return;
  }
  for (int i = 0; i < map_json["landmarks_pos_list"].size(); i++) {
    trilateral_map.push_back(Eigen::Vector2d(
      map_json["landmarks_pos_list"][i]["x"].asDouble(),
      map_json["landmarks_pos_list"][i]["y"].asDouble()));
  }

  std::ifstream infile;

  infile.open(wanji_dir.c_str(), std::ifstream::in);
  if (!infile.is_open()) {
    SLAM_ERROR("can not open wanji map %s,", wanji_dir.c_str());
    return;
  }
  std::string line;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    Eigen::Vector2d temp;
    if (!(iss >> temp(0) >> temp(1))) {
      break;
    }
    wanji_map.push_back(temp);
  }

  MappingAndLocationConfig config;
  std::string config_dir = map_dir + "mappingandlocation.json";
  if (!internal_common::ReadMappingAndLocationConfig(config_dir, &config)) {
    SLAM_ERROR("can not open config file, please check json!!");
    return;
  }
  LandMark::Trilateration trilateration_find_sim(
    config.mapping_config.land_mark_config);

  trilateration_find_sim.SetLandMarkMap(
    trilateration_find_sim.Transform(wanji_map));
  Eigen::Vector3d global_pos;
  if (!trilateration_find_sim.IsGetGlobalPos(trilateral_map, &global_pos)) {
    SLAM_ERROR("error!!!!!!!!!!");
    return;
  }
  std::map<int, int> match_id;
  trilateration_find_sim.GetMatchId(&match_id);

  // 匹配好对应的反光柱
  for (auto match_iter1 : match_id) {
    for (auto match_iter2 : match_id) {
      if (match_iter2.first == match_iter1.first) continue;
      double dis_wanji = SLAMMath::Dist(
                        wanji_map[match_iter1.first](0),
                        wanji_map[match_iter1.first](1),
                        wanji_map[match_iter2.first](0),
                        wanji_map[match_iter2.first](1));
      double dis_ale = SLAMMath::Dist(
                        trilateral_map[match_iter1.second](0),
                        trilateral_map[match_iter1.second](1),
                        trilateral_map[match_iter2.second](0),
                        trilateral_map[match_iter2.second](1));
      double delta = dis_wanji - dis_ale;
      SLAM_INFO("match id %d %d %d %d", match_iter1.first, match_iter2.first,
                                      match_iter1.second, match_iter2.second);
      SLAM_INFO("accuracy_comparison wanji %f ale %f delta %f",
                dis_wanji, dis_ale, delta);
    }
  }
}
