/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-11 18:00:05
 * @LastEditTime: 2023-09-01 11:30:27
 */
#include <gtest/gtest.h>

#include <math.h>
#include "mock/logger_test.h"


TEST(TestLog, Printf) {
  int a = 10;
  // SLAM_INFO("a = %d", a);
  // SLAM_INFO("222222222222222");
  // SLAM_INFO("!!!!!!!!!!!!!!!!!");
  // SLAM_DEBUG("11111111111");
  // SLAM_DEBUG("22222222222");
  // SLAM_ERROR("!!!!!!!!!!!!!");
  // SLAM_WARN("error error");
  return;
}

TEST(TestLog, Multithreading) {
  LoggerTest test;
  int n = 100;
  while (n--) {
    test.Printf();
  }
  return;
}

