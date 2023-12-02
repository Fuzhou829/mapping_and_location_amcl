/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-08-06 17:30:57
 * @LastEditors: renjy
 * @LastEditTime: 2023-08-31 10:38:59
 */
#include <gtest/gtest.h>
#include "include/config_struct.h"
#include "common/load_config.h"
#include "common/logger.h"
#include "common/tool.h"

#include "joint_map/joint_map_interface.h"

TEST(JointMap, ReflectorType) {
  using namespace gomros::data_process::mapping_and_location;  // NOLINT
  MappingAndLocationConfig config;
  std::string data_dir = "../../../../../Dataset/SLAM_Dataset/joint_map/";
  std::string config_dir = data_dir + "/mappingandlocation.json";
  if (!internal_common::ReadMappingAndLocationConfig(config_dir, &config)) {
    SLAM_ERROR("can not open config file, please check json!!");
    return;
  }
  JointMap::JointMapType type = JointMap::JointMapType::Reflector;
  std::shared_ptr<JointMap::JointMapInterface> joint_map_tool =
      JointMap::CreateJointMap(type, config.mapping_config);

  joint_map_tool->SetMajorMapInfo(data_dir + "1.smap");
  std::string second_map =
    "../../../../../Dataset/SLAM_Dataset/joint_map/2.smap";
  std::vector<std::string> second_maps =
      internal_common::SplitCString(second_map, " ");
  for (int i = 0; i < second_maps.size(); i++) {
    if (i == 0) {
      joint_map_tool->SetSecondMap(second_maps[i]);
    } else {
      std::string map_name = data_dir + second_maps[i];
      joint_map_tool->SetSecondMap(map_name);
    }
  }
  if (second_maps.empty()) {
    joint_map_tool->SetSecondMap(second_map);
  }
  // joint_map_tool->SetSecondMap(data_dir + "2.smap");
  // joint_map_tool->SetSecondMap(data_dir + "3.smap");
  // joint_map_tool->SetSecondMap(data_dir + "4.smap");
  // joint_map_tool->SetSecondMap(data_dir + "5.smap");


  joint_map_tool->JointAndSaveMap(data_dir + "result.smap");
}
