/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-12-03 15:51:18
 * @LastEditTime: 2023-09-21 15:03:15
 */
#include <gtest/gtest.h>
#include "mapping_and_location/mapping_and_location.h"
#include "include/config_struct.h"
#include "common_lib/master.h"

#include "karto_mapping/slam_kar.h"


TEST(Mapping, KartoMapping) {
  gomros::common::InitMaster(1);
  using namespace gomros::data_process::mapping_and_location;  // NOLINT
  MappingConfig config;
  std::shared_ptr<SlamKarto> karto_slam = std::make_shared<SlamKarto>(config);
  std::vector<char> map_data1;
  karto_slam->begin_slam(&map_data1, "karto_mapping_map");
}
