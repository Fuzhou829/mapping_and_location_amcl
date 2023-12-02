/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-09-30 22:16:15
 * @LastEditTime: 2023-07-07 07:30:52
 */
#include "mapping_and_location/mapping_and_location.h"

#include "include/mapping_and_location_impl.h"
#include "common/load_config.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {
/**
 * @brief Construct a new Mapping And Location:: Mapping And Location object
 *
 * @param config
 */
MappingAndLocation::MappingAndLocation(const std::string& config_dir) {
  MappingAndLocationConfig config;
  if (!internal_common::ReadMappingAndLocationConfig(config_dir, &config)) {
    std::cerr << "can not open config file, please check json!!" << std::endl;
    std::cerr << "can not open config file, please check json!!" << std::endl;
    std::cerr << "can not open config file, please check json!!" << std::endl;
    std::cerr << "can not open config file, please check json!!" << std::endl;
    std::cerr << "can not open config file, please check json!!" << std::endl;
    std::cerr << "can not open config file, please check json!!" << std::endl;
    return;
  }
  impl_ = std::make_shared<MappingAndLocationImpl>(config);
}

/**
 * @brief Destroy the Mapping And Location:: Mapping And Location object
 *
 */
MappingAndLocation::~MappingAndLocation() {}


void MappingAndLocation::ChargeRelocation() {
  impl_->ChargeRelocation();
  return;
}



}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
