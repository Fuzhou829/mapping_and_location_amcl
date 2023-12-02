/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-10-27 20:42:27
 * @LastEditTime: 2023-03-06 16:11:10
 */
#pragma once

#include <memory>
#include <string>

#include "common_lib/log.h"

#ifndef SLAM_ERROR
#define SLAM_ERROR(fmt, ...)                                             \
  gomros::common::SLAMLogger::GetInstance()->GetLogger()->WriteLog( \
  gomros::common::LOG_LEVEL::ERROR, __FILENAME__,                   \
  __LINE__, __FUNCTION__, fmt"\n", ##__VA_ARGS__)
#endif

#ifndef SLAM_INFO
#define SLAM_INFO(fmt, ...)                                              \
  gomros::common::SLAMLogger::GetInstance()->GetLogger()->WriteLog( \
  gomros::common::LOG_LEVEL::INFO, __FILENAME__,                    \
  __LINE__, __FUNCTION__, fmt"\n", ##__VA_ARGS__)
#endif

#ifndef SLAM_WARN
#define SLAM_WARN(fmt, ...)                                              \
  gomros::common::SLAMLogger::GetInstance()->GetLogger()->WriteLog( \
  gomros::common::LOG_LEVEL::WARNING, __FILENAME__,                 \
  __LINE__, __FUNCTION__, fmt"\n", ##__VA_ARGS__)
#endif

#ifndef SLAM_DEBUG
#define SLAM_DEBUG(fmt, ...)                                             \
  gomros::common::SLAMLogger::GetInstance()->GetLogger()->WriteLog( \
  gomros::common::LOG_LEVEL::DEBUG, __FILENAME__,                   \
  __LINE__, __FUNCTION__, fmt"\n", ##__VA_ARGS__)
#endif

namespace gomros {
namespace common {

// 封装日志打印信息
class SLAMLogger {
 public:
  static void SetLoggerConfig(const gomros::common::LOG_LEVEL& level,
            const std::string& logger_name, bool out_terminal);
  static SLAMLogger* GetInstance() {
    if (logger_ == nullptr) {
      logger_ = new SLAMLogger();
    }
    return logger_;
  }
  Logger* GetLogger() {
    if (p_logger_ == nullptr) {
      p_logger_ = new Logger(gomros::common::LOG_LEVEL::DEBUG,
                              "./MappingAndLoaction", true);
    }
    return p_logger_;
  }

 private:
  SLAMLogger();
  virtual ~SLAMLogger() {}
  // 禁止外部复制构造
  SLAMLogger(const SLAMLogger &signal) {}
  // 禁止外部赋值操作
  const SLAMLogger &operator=(const SLAMLogger &signal) {}
  // static std::recursive_mutex mutex_logger_data_;
  static SLAMLogger* logger_;
  static Logger* p_logger_;
};


}  // namespace common
}  // namespace gomros
