/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-07-07 08:01:16
 * @LastEditors: renjy
 * @LastEditTime: 2023-08-06 16:22:17
 */

#pragma once

#include <jsoncpp/json/json.h>
#include <string>

#include "joint_map/joint_map_interface.h"

namespace JointMap {

class JointQRMap : public JointMapInterface {
 public:
  explicit JointQRMap(const MAL::MappingConfig& map_config):
        JointMapInterface(map_config) {}
  virtual ~JointQRMap() {}
  void SetMajorMapInfo(const std::string& major_map) override;
  void SetSecondMap(const std::string& second_map) override;

  bool JointAndSaveMap(const std::string& final_map) override;

 private:
  bool _jointMap();
  bool _getMapJson();
  void _jsonSave(const char* file_name, Json::Value v);
  std::string major_map_;
  std::string second_map_;
  Json::Value major_map_json_;
  Json::Value second_map_json_;
};






}  // namespace JointMap

