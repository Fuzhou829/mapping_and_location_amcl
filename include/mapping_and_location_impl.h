/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-09-27 15:02:35
 * @LastEditTime: 2023-10-26 10:51:13
 * @Author: lcfc-desktop
 */
#pragma once

#include <arpa/inet.h>
#include <jsoncpp/json/json.h>
#include <math.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Dense"

#include "common_lib/gomros.h"
#include "message_lib/cmd_message.h"
#include "message_lib/device_state.h"
#include "message_lib/iosystem_message.h"
#include "message_lib/odometer_message.h"
#include "message_lib/pose_and_odom_message.h"
#include "message_lib/position_message.h"
#include "message_lib/radar_message.h"
#include "message_lib/simple_grid_map.h"
#include "message_lib/speed_message.h"

#include "include/config_struct.h"
#include "location/location_area.h"
#include "location/location_manager_impl.h"
#include "record_data/record_data.h"

#include "fusion_mapping/fusion_mapping.h"
#include "karto_mapping/karto_mapping.h"
#include "mapping/mapping_interface.h"
#include "optimize_mapping/optimize_mapping.h"
#include "reflector_mapping/reflector_mapping.h"
#include "reflector_mapping/trilateral_mapping.h"

#include "common/logger.h"
#include "common/tool.h"
#include "fusion_mapping/fusion_map_info.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

struct T_COMMU_HEADER {
  char herder_syn;   ////同步头
  char version_num;  /////版本号
  char serial_num1;  /////编号1
  char serial_num2;  /////编号2
  char data_len3;    /////数据长度
  char data_len2;    /////数据长度
  char data_len1;    /////数据长度
  char data_len0;    /////数据长度
  char cmdtype_HB;   ////命令类型高字节
  char cmdtype_LB;   ////命令类型低字节
  char s1;
  char s2;
  char s3;
  char s4;
  char s5;
  char s6;
};  ///////通信结构体头部

enum ROBOT_CMD_REQ {
  GET_MAP = 0,
  GET_PARTICLE = 1,
  GET_LIDAR = 2,
  DO_NOT_GET_PARTICLE = 3,
  DO_NOT_GET_LIDAR = 4,
  MAP_MESSAGE = 10,
  POSE_MESSAGE = 11,
};

class MappingAndLocationImpl {
 public:
  using RawPublisher = gomros::common::RawPublisher;
  using CallBackEvent = gomros::common::CallBackEvent;
  using SimpleGridMap = gomros::message::SimpleGridMap;
  using MapInfo = gomros::message::MapInfo;
  using Position = gomros::message::Position;
  using Node = gomros::common::Node;
  using OdometerMessage = gomros::message::OdometerMessage;
  using DeviceState = gomros::message::DeviceState;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using DMreaderSensoryMessage = gomros::message::DMreaderSensoryMessage;
  using ImuSensoryMessage = gomros::message::ImuSensoryMessage;
  using CustomTaskString = gomros::message::CustomTaskString;
  using RadarSensoryInfo = gomros::message::RadarSensoryInfo;

 public:
  // 建图及定位构造函数
  explicit MappingAndLocationImpl(const MappingAndLocationConfig& config);

  ~MappingAndLocationImpl();

  /**
   * @describes: 利用充电点进行重定位
   * @return {*}
   */
  void ChargeRelocation();

 private:
  /* 加载地图及定位等初始化信息 */
  void _loadPoseFromFile();
  // 从文件中加载地图信息
  void _loadMapNameFromFile(const std::string& file_name);
  /**
   * @describes: 从指定地图文件中读取地图数据
   * @param map_file_name 地图名
   * @param is_need_charge 是否需要充电点信息
   * @return {*}
   */
  bool _loadMapDataFromFile(std::string map_file_name,
                            bool is_need_charge = false);
  void _loadGridMap(const Json::Value& map_json);
  void _loadLandMarkMap(const Json::Value& map_json, SimpleGridMap* grid_map);
  void _loadQrMap(const Json::Value& map_json);
  void _loadLoactionAreaInfo(const Json::Value& map_json);
  bool _loadChargePos(const Json::Value& map_json);

  void _checkLadarMessage(const RadarSensoryMessage& ladar_message);

  // 开始建图模块
  void _startMapping();
  // 建图线程
  static void* _stopMappingThread(void* ptr);
  // 地图合并线程
  static void* _mergeMappingThread(void* ptr);

  // 发布机器目前的状态
  void _publishRobotState();

  /* 消息订阅回调 */
  // 机器定位状态信息发布
  static void* _publishRobotStateThread(void* ptr);
  // 接受机器当前速度
  static void _robotValCallBackFunc(void* object, char* buf, const int size);
  // 接收急停等指令
  static void _emergeCallBackFunc(void* object, char* buf, const int size);
  // 接收odom数据信息
  static void _odomCallBackFunc(void* object, char* buf, const int size);
  // 接收雷达数据
  static void _radarCallBackFunc(void* object, char* buf, const int size);
  // 接收imu数据
  static void _imuCallBackFunc(void* object, char* buf, const int size);
  // 接收qr数据
  static void _qrCallBackFunc(void* object, char* buf, const int size);
  // 接收开始定位消息
  static void _startLocateCallBack(void* object, char* buf, const int size);
  // 接收开始建图消息
  static void _startMappingCallBack(void* object, char* buf, const int size);
  // 接收终止建图消息
  static void _stopMappingCallBack(void* object, char* buf, const int size);
  // 接受标定相关指令
  static void _calibrationQrCameraCallBack(void* object, char* buf,
                                           const int size);
  // 接收切换楼层指令
  static void _upDateMapCallBack(void* object, char* buf, const int size);

  // 保留最新位姿给服务器
  void _savePoseToServer(const Position& current_pose);

  static void* _createTcpServer(void* object);
  static void* _analyszieData(void* object);
  void _sendData();
  bool _socketConnect(int socket);
  int byte2int(char* buff);

 private:
  Node* node_ = nullptr;
  RawPublisher* state_publisher_;
  RawPublisher* pose_publisher_;
  RawPublisher* qr_pos_publisher_;
  Position initial_position_;
  Position radar_pose_base_link_;
  int client_sockfd_;
  bool send_laser_ = false;
  bool send_particle_ = false;
  bool send_map_ = false;

  pthread_mutex_t last_odom_mutex_;
  OdometerMessage last_odom_message_;

  std::shared_ptr<DeviceState> device_state_;
  std::map<int, std::map<int, Eigen::Vector2d>> landmark_map_info_;

  SimpleGridMap* map_shared_pointer_;  // 指向栅格地图的指针
  std::map<int, QRCoordinate> qr_map_;
  Json::Value merge_map_value_;  // 用于合并地图的信息
  std::shared_ptr<MappingInterface> mapping_module_;  // 建图模块

  // TODO(r) 后续需要进行调整代码结构
  std::shared_ptr<LocationManagerImpl> location_module_;  // 定位模块
  std::map<LocationType, std::vector<LocationArea>> location_areas_;

  pthread_t send_pose_thread_;
  pthread_t mapping_thread_;
  pthread_t merge_map_thread_;
  pthread_t tcp_thread_;
  pthread_t analysize_thread_;
  pthread_t send_data_thread_;

  // for test
  pthread_t qr_thread_;

  pthread_mutex_t robot_vel_mutex_;
  pthread_mutex_t send_data_mutex_;
  float robot_line_vel_;
  float robot_angle_vel_;
  bool is_emerge_;
  bool is_move_;
  bool is_calibration_camera_;
  bool is_stop_mapping_;
  int robot_stop_count_;
  double total_odom_;
  std::string save_map_name_;
  string map_file_full_path_;
  MappingAndLocationConfig mapping_and_location_config_;
  // 收集定位测试数据
  std::shared_ptr<Record::SensorData> record_sensor_data_;
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
