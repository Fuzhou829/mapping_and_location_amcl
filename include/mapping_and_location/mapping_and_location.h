/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-10-01 16:42:13
 * @LastEditTime: 2023-07-07 07:30:16
 */
#pragma once
#include <memory>
#include <string>

namespace gomros {
namespace data_process {
namespace mapping_and_location {

class MappingAndLocationImpl;
class MappingAndLocation {
 public:
  // 建图及定位构造函数
  explicit MappingAndLocation(const std::string& config_dir);

  ~MappingAndLocation();

  /**
   * @describes: 利用充电点进行重定位
   * @return {*}
   */  
  void ChargeRelocation();


 private:
  std::shared_ptr<MappingAndLocationImpl> impl_;
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
