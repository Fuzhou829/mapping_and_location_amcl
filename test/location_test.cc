/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-22 14:51:22
 * @LastEditTime: 2023-10-19 16:11:46
 */

#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "mock/sensor_data_simulation.h"
#include "mock/display_location_result.h"
#include "common/load_config.h"

#include "include/config_struct.h"
#include "mapping_and_location/mapping_and_location.h"

#include "common/logger.h"


class AMCLParam
    : public ::testing::TestWithParam<std::string> {};
// amcl 定位
TEST_P(AMCLParam, AMCL) {
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
  if (config.location_config.location_type != LocationType::AMCL) return;
  std::shared_ptr<Display::LocationResult> display_result =
    std::make_shared<Display::LocationResult>(config);

  std::shared_ptr<MappingAndLocation> mapping_and_location =
    std::make_shared<MappingAndLocation>(config_dir);
  // SLAM_INFO("begin FusionMapping test");
  std::shared_ptr<Simulation::SensorData> sensor_data_publish =
    std::make_shared<Simulation::SensorData>(config, data_dir, false);
  while (!sensor_data_publish->IsFinishSimulation() ||
         !display_result->IsFinishedDisply()) {
    usleep(20000);
  }
  usleep(20000);
  return;
}

INSTANTIATE_TEST_CASE_P(Location, AMCLParam,
  ::testing::Values(
    "../../../../../Dataset/SLAM_Dataset/amcl_location/data0",
    "../../../../../Dataset/SLAM_Dataset/amcl_location/data1",
    "../../../../../Dataset/SLAM_Dataset/amcl_location/data2",
    "../../../../../Dataset/SLAM_Dataset/amcl_location/data3",
    "../../../../../Dataset/SLAM_Dataset/amcl_location/data4",
    "../../../../../Dataset/SLAM_Dataset/amcl_location/data5",
    "../../../../../Dataset/SLAM_Dataset/amcl_location/data6",
    "../../../../../Dataset/SLAM_Dataset/amcl_location/data7",
    "../../../../../Dataset/SLAM_Dataset/amcl_location/data8",
    "../../../../../Dataset/SLAM_Dataset/amcl_location/data9"
  )
);



class ReflectorParam
    : public ::testing::TestWithParam<std::string> {};

TEST_P(ReflectorParam, reflector_location) {
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
  // SLAM_INFO("begin FusionMapping test");
  std::shared_ptr<Simulation::SensorData> sensor_data_publish =
    std::make_shared<Simulation::SensorData>(config, data_dir, false);
  std::shared_ptr<Display::LocationResult> display_result =
    std::make_shared<Display::LocationResult>(config);
  while (!sensor_data_publish->IsFinishSimulation() ||
         !display_result->IsFinishedDisply()) {
    usleep(20000);
  }
  usleep(20000);
  return;
}

INSTANTIATE_TEST_CASE_P(Location, ReflectorParam,
  ::testing::Values(
    "../../../../../Dataset/SLAM_Dataset/reflector_location/data0",
    "../../../../../Dataset/SLAM_Dataset/reflector_location/data1",
    "../../../../../Dataset/SLAM_Dataset/reflector_location/data2",
    "../../../../../Dataset/SLAM_Dataset/reflector_location/data3",
    "../../../../../Dataset/SLAM_Dataset/reflector_location/data4",
    "../../../../../Dataset/SLAM_Dataset/reflector_location/data5",
    "../../../../../Dataset/SLAM_Dataset/reflector_location/data6",
    "../../../../../Dataset/SLAM_Dataset/reflector_location/data7",
    "../../../../../Dataset/SLAM_Dataset/reflector_location/data8",
    "../../../../../Dataset/SLAM_Dataset/reflector_location/data9",
    "../../../../../Dataset/SLAM_Dataset/reflector_location/data10",
    "../../../../../Dataset/SLAM_Dataset/reflector_location/data11",
    "../../../../../Dataset/SLAM_Dataset/reflector_location/data12",
    "../../../../../Dataset/SLAM_Dataset/reflector_location/data13",
    "../../../../../Dataset/SLAM_Dataset/reflector_location/data14"
  )
);


class FusionLocationParam
    : public ::testing::TestWithParam<std::string> {};

TEST_P(FusionLocationParam, fusion_location) {
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
  // SLAM_INFO("begin FusionMapping test");
  std::shared_ptr<Simulation::SensorData> sensor_data_publish =
    std::make_shared<Simulation::SensorData>(config, data_dir, false);
  std::shared_ptr<Display::LocationResult> display_result =
    std::make_shared<Display::LocationResult>(config);
  while (!sensor_data_publish->IsFinishSimulation() ||
         !display_result->IsFinishedDisply()) {
    usleep(20000);
  }
  usleep(20000);
  return;
}

INSTANTIATE_TEST_CASE_P(Location, FusionLocationParam,
  ::testing::Values(
    "../../../../../Dataset/SLAM_Dataset/fusion_location/data0",
    "../../../../../Dataset/SLAM_Dataset/fusion_location/data1",
    "../../../../../Dataset/SLAM_Dataset/fusion_location/data2",
    "../../../../../Dataset/SLAM_Dataset/fusion_location/data3",
    "../../../../../Dataset/SLAM_Dataset/fusion_location/data4",
    "../../../../../Dataset/SLAM_Dataset/fusion_location/data5",
    "../../../../../Dataset/SLAM_Dataset/fusion_location/data6",
    "../../../../../Dataset/SLAM_Dataset/fusion_location/data7",
    "../../../../../Dataset/SLAM_Dataset/fusion_location/data8",
    "../../../../../Dataset/SLAM_Dataset/fusion_location/data9",
    "../../../../../Dataset/SLAM_Dataset/fusion_location/data10",
    "../../../../../Dataset/SLAM_Dataset/fusion_location/data11",
    "../../../../../Dataset/SLAM_Dataset/fusion_location/data12",
    "../../../../../Dataset/SLAM_Dataset/fusion_location/data13",
    "../../../../../Dataset/SLAM_Dataset/fusion_location/data14"
  )
);
