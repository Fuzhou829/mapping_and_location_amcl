/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-28 11:49:26
 * @LastEditTime: 2023-06-03 13:42:52
 */

#pragma once

#include <chrono> // NOLINT
#include <iostream>

#include "common/logger.h"

namespace internal_common {

class RunTime {
 public:
  using seconds = std::chrono::seconds;
  using milliseconds = std::chrono::milliseconds;
  using microseconds = std::chrono::microseconds;
  using high_resolution_clock = std::chrono::high_resolution_clock;

 public:
  RunTime() {
    start_time_ = high_resolution_clock::now();
  }
  virtual ~RunTime() {
    // ElapsedMillisecond();
    // ElapsedMicroSecond();
    SpeedTime();
  }

  float ElapsedSecond() {
    int time_s = std::chrono::duration_cast<seconds>(
                high_resolution_clock::now() - start_time_).count();
    // LOGGER("speed run time = %d s", time_s);
    return time_s;
  }
  float ElapsedMillisecond() {
    int time_s = std::chrono::duration_cast<milliseconds>(
                  high_resolution_clock::now() - start_time_).count();
    // LOGGER("speed run time = %d ms", time_s);
    return time_s;
  }
  float ElapsedMicroSecond() {
    int time_s = std::chrono::duration_cast<microseconds>(
      high_resolution_clock::now() - start_time_).count();
    // LOGGER("speed run time = %d us", time_s);
    return time_s;
  }
  void SpeedTime() {
    int time_us = std::chrono::duration_cast<microseconds>(
      high_resolution_clock::now() - start_time_).count();
    int time_ms = std::chrono::duration_cast<milliseconds>(
                  high_resolution_clock::now() - start_time_).count();
    float time_s = time_ms + time_us / 1000.0f;
    // SLAM_INFO("speed time = %f ms", time_s);
  }

 private:
  std::chrono::high_resolution_clock::time_point start_time_;
};


constexpr int64_t kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);

struct UniversalTimeScaleClock {
  using rep = int64_t;
  using period = std::ratio<1, 10000000>;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
using Duration = UniversalTimeScaleClock::duration;
using Time = UniversalTimeScaleClock::time_point;

// Convenience functions to create common::Durations.
inline Duration FromSeconds(double seconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::duration<double>(seconds));
}
inline Duration FromMilliseconds(int64_t milliseconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::milliseconds(milliseconds));
}

// Returns the given duration in seconds.
inline double ToSeconds(Duration duration) {
    return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
      .count();
}

// Creates a time from a Universal Time Scale.
inline Time FromUniversal(int64_t ticks) {
  return Time(Duration(ticks));
}

// Outputs the Universal Time Scale timestamp for a given Time.
inline int64_t ToUniversal(Time time) {
  return time.time_since_epoch().count();
}

// For logging and unit tests, outputs the timestamp integer.
inline std::ostream& operator<<(std::ostream& os, Time time) {
    os << std::to_string(ToUniversal(time));
  return os;
}



}  // namespace internal_common
