/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-05-26 03:00:40
 * @LastEditors: renjy
 * @LastEditTime: 2023-05-26 03:03:36
 */
#pragma once

#include "message_lib/radar_message.h"
#include "message_lib/position_message.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

/**
 * @brief: TODO(r) 二维码的观测信息 
 */
struct QRInfo {
  uint64_t time_stemp;   // QR 时间戳
  int tag_num;        // QR id
  float pos_x;        // QR 观测位置 x 单位：m
  float pos_y;        // QR 观测位置 y 单位：m
  float pos_theta;    // QR 观测位置 theta 单位：弧度
};

/**
 * @brief: 二维码的坐标 全局坐标需要得到安装位姿信息 局部坐标需要外部输入
 */
struct QRCoordinate {
  int tag_num;        // QR id
  float mx;           // 二维码坐标系x 单位：m
  float my;           // 二维码坐标系y 单位：m
  float theta;        // 二维码坐标系朝向 单位：弧度 逆时针为正
  bool operator < (const QRCoordinate &a) const {
    return tag_num < a.tag_num;
  }
};

struct CreateQrMapInfo {
  using RadarSensoryInfo = gomros::message::RadarSensoryInfo;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using Position = gomros::message::Position;
  RadarSensoryInfo ladar_info;
  QRInfo qr_info;
  Position qr_pos;   // qr 对应全局的坐标
  Position ladar_pos;   // 当前激光帧对应的全局坐标
};

struct CalibrationQrInfo {
  using Position = gomros::message::Position;
  QRInfo qr_info;   // 相机在二维码中位姿
  Position robot_pos;   // qr 对应车体全局的坐标
};



}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
