/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-09-21 14:36:35
 * @LastEditors: renjy
 * @LastEditTime: 2023-09-21 15:02:14
 */

#include "mock/load_data_for_test.h"

namespace LoadDataForTest {

bool ReadLadar(const std::string& file_dir, LadarInfo* ladar_info) {
  std::ifstream infile;
  infile.open(file_dir.c_str(), std::ifstream::in);
  if (!infile.is_open()) {
    SLAM_WARN("can not open %s", file_dir.c_str());
    return false;
  }
  std::string line;
  uint64_t time = 0;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    if (0 == time) {
      if (!(iss >> time)) break;
      continue;
    }
    float x, y, i;
    if (!(iss >> x >> y >> i))
      break;
    ladar_info->mvPoints.push_back(Eigen::Vector2d(x, y));
    ladar_info->mvIntensities.push_back(i);
  }
  return true;
}

}  // namespace LoadDataForTest











