/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 023-07-07 08:04:21
 * @LastEditors: renjy
 * @LastEditTime: 2023-08-06 19:31:43
 */

#include "joint_map/joint_map_interface.h"
#include "joint_map/joint_qr_map.h"
#include "joint_map/joint_reflector_map.h"

#include "common/logger.h"

namespace JointMap {


std::shared_ptr<JointMapInterface> CreateJointMap(const JointMapType& type,
  const MAL::MappingConfig& config) {
  SLAM_INFO("joint map type now choose %d", type);
  switch (type) {
    case JointMapType::Qr:
      return std::make_shared<JointQRMap>(config);
    case JointMapType::Reflector:
      return std::make_shared<JointReflectorMap>(config);
    default:
     return std::make_shared<JointQRMap>(config);
  }
}



}  // namespace JointMap
















