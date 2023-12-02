/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 023-07-07 08:04:21
 * @LastEditors: renjy
 * @LastEditTime: 2023-08-07 17:49:39
 */

#pragma once

#include <memory>
#include <string>

#include "include/config_struct.h"
namespace JointMap {

enum JointMapType {
  Qr = 0,
  Reflector = 1
};

namespace MAL = gomros::data_process::mapping_and_location;

class JointMapInterface {
 public:
  explicit JointMapInterface(
    const MAL::MappingConfig& map_config) :
  mapping_config_(map_config) {}
  virtual ~JointMapInterface() {}

  /**
   * @describes: 输入需要合并地图的信息
   * @return {*}
   */
  virtual void SetMajorMapInfo(const std::string& major_map) = 0;
    /**
   * @describes: 输入需要合并地图的信息
   * @return {*}
   */
  virtual void SetSecondMap(const std::string& second_map) = 0;
  /**
   * @describes: 合并地图并进行保存
   * @param final_map 最后保存的地图名字
   * @return {*}
   */ 
  virtual bool JointAndSaveMap(const std::string& final_map) = 0;

 protected:
  MAL::MappingConfig mapping_config_;
};

std::shared_ptr<JointMapInterface> CreateJointMap(const JointMapType& type,
  const MAL::MappingConfig& config);


}  // namespace JointMap
