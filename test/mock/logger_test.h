/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-10 10:02:47
 * @LastEditTime: 2023-06-29 11:18:04
 */
#pragma once
#include <unistd.h>

#include <string>
#include "common/logger.h"

/**
 * @brief: 测试线程下机器是否能进行正常打印
 */

class LoggerTest {
 public:
  LoggerTest() {
    gomros::common::SLAMLogger::SetLoggerConfig(
      gomros::common::LOG_LEVEL::DEBUG, "./slam", true);
    pthread_create(&thread_1_, NULL, _newThread1, this);
    pthread_create(&thread_2_, NULL, _newThread2, this);
  }
  virtual ~LoggerTest() {
    // pthread_join(thread_1_, NULL);
    // pthread_join(thread_2_, NULL);
    // pthread_detach(thread_1_);
    // pthread_detach(thread_2_);
  }
  void Printf() {
    // SLAM_INFO("this is Printf!!!!!!!!");
    usleep(20000);
  }

 private:
  static void* _newThread1(void* ptr) {
    pthread_detach(pthread_self());
    while (1) {
      // SLAM_INFO("this is thread 11111111111111");
    }
    pthread_exit(NULL);
  }
  static void* _newThread2(void* ptr) {
    pthread_detach(pthread_self());
    while (1) {
      // SLAM_INFO("this is thread 22222222222222");
    }
    pthread_exit(NULL);
  }

 private:
  pthread_t thread_1_;
  pthread_t thread_2_;
};
