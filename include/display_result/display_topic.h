/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-09-05 22:56:03
 * @LastEditors: renjy
 * @LastEditTime: 2023-10-18 15:26:37
 */

#pragma once

#include <string>

namespace DisPlayResult {

  // 原始雷达数据 并携带该帧雷达数据在全局坐标下的位姿信息
  const std::string raw_ladar_topic = "raw_ladar";

  /* 
    携带反光柱的信息 激光头携带了反光柱在全局坐标下的位姿  -- 定位
  */
  const std::string reflector_ladar_topic = "reflector_info";
  /* 
    携带ekf反光柱的信息
  */
  const std::string reflector_ekf_topic = "reflector_ekf_info";
  /*
    反光柱SLAM建图时 状态信息
  */
  const std::string ekf_mapping_topic = "state_info";

  // 栅格建图
  const std::string grid_map_topic = "grid_map_info";


}  // namespace DisPlayResult






