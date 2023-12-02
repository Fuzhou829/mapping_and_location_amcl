/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Date: 2023-07-29 07:44:05
 * @LastEditors: renjy
 * @LastEditTime: 2023-08-07 18:16:17
 */
#include <gtest/gtest.h>

#include "mock/sensor_data_simulation.h"
#include "common/load_config.h"

#include "include/config_struct.h"
#include "mapping_and_location/mapping_and_location.h"
#include "common/logger.h"

// 测试接口 -- 建图
TEST(interface_test, mapping) {
  gomros::common::InitMaster(1);
  using namespace gomros::data_process::mapping_and_location;  // NOLINT
  std::string data_dir =
    "../../../../../Dataset/SLAM_Dataset/fusion_mapping/data0";
  SLAM_INFO("data_dir %s", data_dir.c_str());
  MappingAndLocationConfig config;
  std::string config_dir = data_dir + "/mappingandlocation.json";
  if (!internal_common::ReadMappingAndLocationConfig(config_dir, &config)) {
    SLAM_ERROR("can not open config file, please check json!!");
    return;
  }
  std::shared_ptr<MappingAndLocation> mapping_and_location =
    std::make_shared<MappingAndLocation>(config_dir);
  int count = 10;
  while (count--) {
    SLAM_INFO("mapping count %d", count);
    std::shared_ptr<Simulation::SensorData> sensor_data_publish =
      std::make_shared<Simulation::SensorData>(config, data_dir);
    while (!sensor_data_publish->IsFinishSimulation()) {
      usleep(20000);
    }
    usleep(60000000);
  }
}



