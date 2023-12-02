/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.2
 * @Author: renjy
 * @Date: 2023-10-22 15:33:47
 * @LastEditTime: 2023-10-26 12:03:11
 */

#include <gtest/gtest.h>
#include <list>


#include "Eigen/Dense"
#include "common/logger.h"


TEST(QR, QR_TEST) {
  // 相机在二维码坐标系下的坐标
  Eigen::Matrix3d camera_qr(3, 3);
  camera_qr <<
    cos(0), -sin(0), 0.0,
    sin(0), cos(0), 0.0,
    0, 0, 1;
  // 相机在车体中的坐标
  Eigen::Matrix3d camera_robot(3, 3);
  float theta = 0.0;
  camera_robot <<
    cos(theta), -sin(theta), 0.755,
    sin(theta), cos(theta), -0.42,
    0, 0, 1;
  // 二维码在全局中的坐标
  Eigen::Matrix3d qr_world(3, 3);
  qr_world <<
    cos(0), -sin(0), 0,
    sin(0), cos(0), 0,
    0, 0, 1;
  Eigen::Matrix3d robot_world(3, 3);
  robot_world = qr_world * camera_qr * camera_robot.inverse();
  SLAM_INFO("0 -> %f %f %f", robot_world(0, 2), robot_world(1, 2),
    atan2(robot_world(1, 0), robot_world(0, 0)));

  theta = 1.0 * M_PI / 180.f;
  camera_robot <<
    cos(theta), -sin(theta), 0.755,
    sin(theta), cos(theta), -0.42,
    0, 0, 1;

  robot_world = qr_world * camera_qr * camera_robot.inverse();
  SLAM_INFO("1 -> %f %f %f", robot_world(0, 2), robot_world(1, 2),
    atan2(robot_world(1, 0), robot_world(0, 0)));
}




TEST(TEST, LIST) {
  std::list<int> list_;
  list_.push_back(1);
  list_.push_back(2);
  list_.push_back(3);
  list_.push_back(4);

  for (auto data : list_) {
    std::cout << data << std::endl;
  }

  while (!list_.empty()) {
    int data = list_.front();
    if (data > 2) break;
    list_.pop_front();
    std::cout << data << std::endl;
  }
}
