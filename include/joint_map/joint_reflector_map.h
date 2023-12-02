/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-07-07 08:01:16
 * @LastEditors: renjy
 * @LastEditTime: 2023-08-06 19:34:35
 */

#pragma once

#include <jsoncpp/json/json.h>
#include <string>
#include <map>
#include <vector>

#include "Eigen/Dense"

#include "joint_map/joint_map_interface.h"
#include "reflector_mapping/trilateral_mapping.h"

namespace JointMap {

class JointReflectorMap : public JointMapInterface {
 public:
  explicit JointReflectorMap(const MAL::MappingConfig& map_config)
        : JointMapInterface(map_config) {}
  virtual ~JointReflectorMap() {}
  void SetMajorMapInfo(const std::string& major_map) override;
  void SetSecondMap(const std::string& second_map) override;
  bool JointAndSaveMap(const std::string& final_map) override;


 private:
  std::vector<Eigen::Vector2d> _getreflectorFromMap(
    const std::string& major_map);

 private:
  std::map<int, std::vector<Eigen::Vector2d>> candidate_map_;
};






}  // namespace JointMap

