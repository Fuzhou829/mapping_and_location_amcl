/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-22 14:51:22
 * @LastEditTime: 2023-08-07 18:15:59
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

TEST(FusionLocation, from_file) {
  gomros::common::InitMaster(1);

  using namespace gomros::data_process::mapping_and_location;  // NOLINT
  MappingAndLocationConfig config;
  std::string config_dir =
    "../../../../../Dataset/SLAM_Dataset/config/mappingandlocation.json";
  if (!internal_common::ReadMappingAndLocationConfig(config_dir, &config)) {
    SLAM_ERROR("can not open config file, please check json!!");
    return;
  }
  std::string file_name = "../../../../../Dataset/SLAM_Dataset/mapping_data";
  std::shared_ptr<MappingAndLocation> mapping_and_location =
    std::make_shared<MappingAndLocation>(config_dir);
  // SLAM_INFO("begin FusionMapping test");
  std::shared_ptr<Simulation::SensorData> sensor_data_publish =
    std::make_shared<Simulation::SensorData>(config, file_name, false);
  std::shared_ptr<Display::LocationResult> display_result =
    std::make_shared<Display::LocationResult>(config);
  while (!sensor_data_publish->IsFinishSimulation() ||
         !display_result->IsFinishedDisply()) {
    usleep(20000);
  }
  usleep(20000);
  return;
}
