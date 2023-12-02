/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-11-12 09:28:10
 * @LastEditTime: 2023-11-02 10:44:47
 */
#include <float.h>
#include <pthread.h>
#include <fstream>
#include <istream>
#include <vector>
#include <utility>

#include "Eigen/Dense"
#include "message_lib/pose_and_odom_message.h"

#include "mock/sensor_data_simulation.h"
#include "common/run_time.h"
#include "common/tool.h"

namespace Simulation {



SensorData::SensorData(const MappingAndLocationConfig& config,
  const std::string& data_file_dir, bool is_mapping)
: config_(config), data_file_dir_(data_file_dir) {
  is_in_real_time_location_ = false;
  is_finish_publish_ = false;
  node_ = new Node(config_.node_name);
  odom_publisher_ = node_->AdvertiseTopic(config_.sub_odom_topic);
  imu_publisher_ = node_->AdvertiseTopic(config_.sub_imu_topic);
  qr_publisher_ = node_->AdvertiseTopic(config_.sub_qr_topic);
  ladar_publisher_ = node_->AdvertiseTopic(config_.sub_radar_topic);

  float back_wheel_length =
    config_.location_config.sensor_mount.back_wheel_length;
  float front_wheel_length =
    config_.location_config.sensor_mount.front_wheel_length;
  float back_wheel_alpha =
    config_.location_config.sensor_mount.back_wheel_alpha;
  float front_wheel_alpha =
    config_.location_config.sensor_mount.front_wheel_alpha;
  config_mat_ <<
    1.0f, 0.0f, -back_wheel_length * sin(back_wheel_alpha),
    0.0f, 1.0f, -back_wheel_length * cos(back_wheel_alpha),
    1.0f, 0.0f, front_wheel_length * sin(front_wheel_alpha),
    0.0f, 1.0f, front_wheel_length * cos(front_wheel_alpha);


  SLAM_INFO("sub_odom_topic = %s, sub_radar_topic= %s",
            config_.sub_odom_topic.c_str(), config_.sub_radar_topic.c_str());
  SLAM_INFO("sub_imu_topic = %s, sub_qr_topic= %s",
            config_.sub_imu_topic.c_str(), config_.sub_qr_topic.c_str());
  node_->Debug();
  is_mapping_ = is_mapping;
    // 发布开始建图
  start_mapping_publisher_ =
    node_->AdvertiseTopic(config_.sub_start_mapping_cmd_topic);

  end_mapping_publisher_ =
    node_->AdvertiseTopic(config_.sub_stop_mapping_cmd_topic);
  CallBackEvent robot_state_event{this, _robotState};
  node_->SubscribeTopic(config_.ad_state_topic,
                        robot_state_event);
  CallBackEvent robot_pose_event{this, _robotPose};
  node_->SubscribeTopic(config_.ad_pose_topic, robot_pose_event);

  _loadMessageFromFile();
  // 启动线程
  pthread_create(&thread_, NULL, _publisherThread, this);
}

bool SensorData::IsFinishSimulation() {
  return is_finish_publish_ && is_in_real_time_location_;
}


void SensorData::_robotState(void* object, char* buf, const int size) {
  SensorData *lp_this = reinterpret_cast<SensorData *>(object);
  gomros::common::Message<DeviceState> task_string;
  task_string.FromCharArray(buf, size);
  if (task_string.pack.mcChassisState == SYSTEM_STATE_FREE) {
    lp_this->is_in_real_time_location_ = true;
  } else {
    lp_this->is_in_real_time_location_ = false;
  }
  if (task_string.pack.mcChassisState == SYSTEM_STATE_OFFLINE) {
    SLAM_WARN("Positioning fails");
  }
}
void SensorData::_robotPose(void *object, char *buf, const int size) {
  SensorData *lp_this = reinterpret_cast<SensorData *>(object);
  gomros::common::Message<gomros::message::TotalOdomMessgae> pose_info;
  pose_info.FromCharArray(buf, size);
  return;
}

void SensorData::_loadMessageFromFile() {
  internal_common::RunTime run_time;
  std::ifstream imu_infile, odom_infile, ladar_infile, qr_infile, qr_map_infile;
  std::string file_name = data_file_dir_;
  std::string imu_file_name = file_name + "/imu_message.txt";
  imu_infile.open(imu_file_name.c_str(), std::ifstream::in);
  std::string odom_file_name = file_name + "/odom_message.txt";
  odom_infile.open(odom_file_name.c_str(), std::ifstream::in);
  std::string ladar_file_name = file_name + "/ladar_message.smap";
  ladar_infile.open(ladar_file_name.c_str(), std::ifstream::in);

  std::string qr_file_name = file_name + "/qr_message.txt";
  qr_infile.open(qr_file_name.c_str(), std::ifstream::in);

  std::string qr_map_name = file_name + "/scan_match_ladar.txt";
  qr_map_infile.open(qr_map_name.c_str(), std::ifstream::in);

  std::string line_read;
  if (!imu_infile.is_open()) {
    SLAM_ERROR("imu infile can not open~~~");
  }
  // 加载imu数据
  while (getline(imu_infile, line_read)) {
    MessageInfo message_info;
    message_info.message_type = MessageType::IMU;
    message_info.message_info = internal_common::SplitCString(line_read, " ");
    uint64_t time = atof(message_info.message_info[0].c_str());
    message_infos_.insert(std::make_pair(time, message_info));
  }
  // 加载odom
  uint64_t last_odom_time = 0;
  while (getline(odom_infile, line_read)) {
    MessageInfo message_info;
    message_info.message_type = MessageType::ODOM;
    message_info.message_info = internal_common::SplitCString(line_read, " ");
    uint64_t time = atof(message_info.message_info[0].c_str());
    message_infos_.insert(std::make_pair(time, message_info));
    // SLAM_INFO("odom delta time %f", (time - last_odom_time) * 1e-6);
    last_odom_time = time;
  }

  int count = 0;
  // 加载雷达
  uint64_t last_ladar_time;
  while (getline(ladar_infile, line_read)) {
    MessageInfo message_info;
    message_info.message_type = MessageType::LADAR;
    message_info.message_info = internal_common::SplitCString(line_read, " ");
    uint64_t time = atof(message_info.message_info[0].c_str());
    message_infos_.insert(std::make_pair(time, message_info));
    if (time < last_ladar_time) {
      SLAM_WARN("time is error!!!!!");
      continue;
    }
    // SLAM_INFO("ladar get time %f", (time - last_ladar_time) * 1e-6);
    last_ladar_time = time;
    count++;
  }
  SLAM_INFO("ladar count %d", count);

  // 加载 qr
  while (getline(qr_infile, line_read)) {
    MessageInfo message_info;
    message_info.message_type = MessageType::QR;
    message_info.message_info = internal_common::SplitCString(line_read, " ");
    uint64_t time = atof(message_info.message_info[0].c_str());
    message_infos_.insert(std::make_pair(time, message_info));
  }

  // 加载qr和ladar
  while (getline(qr_map_infile, line_read)) {
    std::vector<std::string> message_string =
      internal_common::SplitCString(line_read, " ");
    MessageInfo message_info;
    message_info.message_type = MessageType::QR;
    message_info.message_info.insert(message_info.message_info.end(),
            message_string.begin(), message_string.begin() + 5);
    message_info.message_info.push_back(std::string("1"));
    uint64_t time = atof(message_info.message_info[0].c_str());
    message_infos_.insert(std::make_pair(time, message_info));
    message_info.message_type = MessageType::LADAR;
    message_info.message_info.clear();
    message_info.message_info.insert(message_info.message_info.end(),
            message_string.begin() + 5, message_string.end());
    time = atof(message_info.message_info[0].c_str());
    message_infos_.insert(std::make_pair(time, message_info));
  }

  imu_infile.close();
  odom_infile.close();
  ladar_infile.close();
  qr_infile.close();
  qr_map_infile.close();

  SLAM_INFO("load file speend time %f ms", run_time.ElapsedMillisecond());
  return;
}



void* SensorData::_publisherThread(void* ptr) {
  pthread_detach(pthread_self());
  usleep(10000);
  SLAM_INFO("enter sensor data publish thread !!!!!!!!!!");
  SensorData *lp_this = reinterpret_cast<SensorData *>(ptr);
  bool is_stop_mapping = false;

  std::string line_read;
  ImuSensoryMessage imu_message;
  OdometerMessage odom_message;
  RadarSensoryMessage ladar_message;
  DMreaderSensoryMessage qr_message;
  internal_common::RunTime run_time;
  if (lp_this->is_mapping_) {
    CustomTaskString task_cmd;
    gomros::common::Message<CustomTaskString> task_cmd_message;
    task_cmd_message.pack = task_cmd;
    std::vector<char> send_buff;
    task_cmd_message.ToCharArray(&send_buff);
    lp_this->start_mapping_publisher_->Publish(
        send_buff.data(), send_buff.size());
  }
  uint64_t pre_time, current_time;
  pre_time =
    atof(lp_this->message_infos_.begin()->second.message_info[0].c_str());
  bool right_data = false;
  while (!lp_this->message_infos_.empty()) {
    MessageType message_type =
      lp_this->message_infos_.begin()->second.message_type;
    current_time =
      atof(lp_this->message_infos_.begin()->second.message_info[0].c_str());
    assert(current_time >= pre_time);

    if (!lp_this->is_mapping_ && right_data) {
      if (current_time - pre_time > 500000) {
        SLAM_WARN("sleep time too long %d", current_time - pre_time);
      } else {
        usleep(current_time - pre_time);
      }
      if (!lp_this->is_in_real_time_location_ &&
          message_type == MessageType::LADAR) {
        usleep(100000);
      }
    }
    pre_time = current_time;
    switch (message_type) {
      case MessageType::IMU : {
        ImuSensoryMessage imu_message = lp_this->_getImuMessage(
          lp_this->message_infos_.begin()->second.message_info);
        lp_this->message_infos_.erase(lp_this->message_infos_.begin());
        if (right_data) lp_this->_imuPublish(imu_message);
        break;
      };
      case MessageType::ODOM : {
        internal_common::GetOdomMessage(
          lp_this->message_infos_.begin()->second.message_info,
          (OdomType)lp_this->config_.location_config.odom_type , &odom_message);
        if (right_data) lp_this->_odomPublish(odom_message);
        lp_this->message_infos_.erase(lp_this->message_infos_.begin());
        break;
      };
      case MessageType::QR : {
        DMreaderSensoryMessage qr_message = lp_this->_getQrMessage(
          lp_this->message_infos_.begin()->second.message_info);
        if (right_data) lp_this->_qrPublish(qr_message);
        lp_this->message_infos_.erase(lp_this->message_infos_.begin());
        break;
      };
      case MessageType::LADAR : {
        RadarSensoryMessage ladar_message;
        if (!lp_this->_getLadarMessage(lp_this->message_infos_.begin()->
                      second.message_info, &ladar_message)) {
           lp_this->message_infos_.erase(lp_this->message_infos_.begin());
           break;
        }
        right_data = true;
        lp_this->_ladarPublish(ladar_message);
        if (lp_this->is_mapping_ || lp_this->is_in_real_time_location_ ||
            lp_this->config_.location_config.location_type ==
            LocationType::LandMarkLocation ||
            lp_this->config_.location_config.location_type ==
            LocationType::Trilateral) {
          lp_this->message_infos_.erase(lp_this->message_infos_.begin());
        }
        break;
      }
      default:
        break;
    }
  }
  usleep(100000);
  if (lp_this->is_mapping_) {
    SLAM_INFO("~~~~~~stop mapping~~~~~");
    CustomTaskString task_cmd;
    gomros::common::Message<CustomTaskString> task_cmd_message;
    task_cmd_message.pack = task_cmd;
    std::vector<char> send_buff;
    task_cmd_message.ToCharArray(&send_buff);
    lp_this->end_mapping_publisher_->Publish(
          send_buff.data(), send_buff.size());
  }
  lp_this->is_finish_publish_ = true;
  SLAM_INFO("simulation finished!!!!!!!!!");
  pthread_exit(NULL);
}


void SensorData::_odomPublish(const OdometerMessage& message) {
  using namespace gomros::common;  // NOLINT
  Message<OdometerMessage> odom_message;
  odom_message.pack = message;
  std::vector<char> send_buff;
  odom_message.ToCharArray(&send_buff);
  odom_publisher_->Publish(send_buff.data(), send_buff.size());
}

void SensorData::_ladarPublish(RadarSensoryMessage message) {
  using namespace gomros::common;  // NOLINT
  std::vector<char> send_buff;
  message.to_char_array(&send_buff);
  ladar_publisher_->Publish(send_buff.data(), send_buff.size());
}

void SensorData::_imuPublish(const ImuSensoryMessage& message) {
  using namespace gomros::common;  // NOLINT
  Message<ImuSensoryMessage> imu_message;
  imu_message.pack = message;
  std::vector<char> send_buff;
  imu_message.ToCharArray(&send_buff);
  imu_publisher_->Publish(send_buff.data(), send_buff.size());
}

void SensorData::_qrPublish(const DMreaderSensoryMessage& message) {
  using namespace gomros::common;  // NOLINT
  Message<DMreaderSensoryMessage> qr_message;
  qr_message.pack = message;
  std::vector<char> send_buff;
  qr_message.ToCharArray(&send_buff);
  qr_publisher_->Publish(send_buff.data(), send_buff.size());
}


void SensorData::_calRobotVel(OdometerMessage* odom_message) {
  if (config_.location_config.odom_type == OdomType::DiffWheelModel) {
    float left_vel = odom_message->mstruDiffSteerSts.mfLeftLinearVel;
    float right_vel = odom_message->mstruDiffSteerSts.mfRightLinearVel;
    odom_message->mfLinearSpeed = 0.5 * (left_vel + right_vel);
    odom_message->mfAngularSpeed = (right_vel - left_vel) /
        config_.location_config.sensor_mount.wheel_distance;
  } else if (config_.location_config.odom_type == OdomType::DoubleSteerModel) {
    float fvel1 = odom_message->mstruDualSteerSts.mdFrontLinearVel;
    float bvel1 = odom_message->mstruDualSteerSts.mdBackLinearVel;
    float theta_f1 = odom_message->mstruDualSteerSts.mdFrontRotAngle;
    float theta_b1 = odom_message->mstruDualSteerSts.mdBackRotAngle;
    float cf1, sf1, cb1, sb1;
    cf1 = cos(theta_f1);
    sf1 = sin(theta_f1);
    cb1 = cos(theta_b1);
    sb1 = sin(theta_b1);

    Eigen::Matrix<float, 4, 1> matrix_41;
    matrix_41<<
    bvel1 * cb1, bvel1 * sb1, fvel1 * cf1, fvel1 * sf1;

    Eigen::Matrix<float, 3, 3> matrix_33 =
      config_mat_.transpose() * config_mat_;
    Eigen::Matrix<float, 3, 4> matrix_btbnbt =
      matrix_33.inverse() * config_mat_.transpose();
    Eigen::Matrix<float, 3, 1> result =
      matrix_btbnbt * matrix_41;

    float vel_x = result(0);
    float vel_y = result(1);
    float w = result(2);
    odom_message->mfLinearSpeed = std::sqrt(vel_x * vel_x + vel_y * vel_y);
    odom_message->mfAngularSpeed = w;
  } else if (config_.location_config.odom_type == OdomType::SingleSteerModel) {
    float ldLineVel = odom_message->mstruSingleSteerSts.mdLinearVel;
    float ldAngularVel = odom_message->mstruSingleSteerSts.mdAngularVel;
    float ldTurnAng = odom_message->mstruSingleSteerSts.mdRotAngle;

    odom_message->mfLinearSpeed = ldLineVel * (cos(ldTurnAng));
    odom_message->mfAngularSpeed = ldLineVel * (sin(ldTurnAng)) /
      config_.location_config.sensor_mount.to_centre_distance;
  }
}


SensorData::ImuSensoryMessage SensorData::_getImuMessage(
  const std::vector<std::string>& line) {
  ImuSensoryMessage imu_message;
  imu_message.time_stamp = atof(line[0].c_str());
  imu_message.z_omega = atof(line[1].c_str());
  imu_message.z_angle = atof(line[2].c_str());
  return imu_message;
}

bool SensorData::_getLadarMessage(
  const std::vector<std::string>& line, RadarSensoryMessage* message) {
  message->mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp =
    atof(line[0].c_str());
  message->mstruRadarMessage.mstruSingleLayerData.mvPoints.clear();
  message->mstruRadarMessage.mstruSingleLayerData.mvIntensities.clear();
  float theta_acc = config_.mapping_config.mapping_end_angle -
                      config_.mapping_config.mapping_start_angle;
  int record_count =
      theta_acc / config_.mapping_config.laser_resolution + 1;
  int read_id = 2;
  float xp, yp;
  float theta = config_.mapping_config.mapping_start_angle * M_PI / 180.0f;
  float delta_theta = config_.mapping_config.laser_resolution * M_PI / 180.0;
  message->mstruRadarMessage.mstruRadarHeaderData.mfAngleMax = M_PI;
  message->mstruRadarMessage.mstruRadarHeaderData.mfAngleMin = -1.0f * M_PI;
  message->mstruRadarMessage.mstruRadarHeaderData.mfAngleIncreament =
    (config_.location_config.location_laser_resolution) * M_PI / 180.0f;
  message->mstruRadarMessage.mstruRadarHeaderData.mfRangeMax =
    config_.mapping_config.mapping_laser_max_range;
  message->mstruRadarMessage.mstruRadarHeaderData.mfRangeMin =
    config_.mapping_config.mapping_laser_min_range;
  if (record_count != (line.size() - 2) / 2) {
    SLAM_ERROR("ladar data size is error!!!");
    return false;
  }
  static int file_count = 0;
  char open_file_name[512];
  snprintf(open_file_name, sizeof(open_file_name), \
            "./out/ladar_data_%02d.txt", file_count++);
  std::ofstream outfile(open_file_name);
  bool is_print_data = true;
  if (!outfile.is_open()) {
    SLAM_ERROR("can not open ladar data file!!!");
    is_print_data = false;
  }
  if (is_print_data) {
    outfile << message->mstruRadarMessage.mstruRadarHeaderData.mlTimeStamp
            << std::endl;
  }
  for (int i = 0; i < record_count; i++) {
    float len = atof(line[read_id++].c_str());
    xp = len * cos(theta);
    yp = len * sin(theta);
    // SLAM_WARN("id = %d %f %f %f %f\n", i, len, theta, xp, yp);
    theta += delta_theta;
    float intensitie = atof(line[read_id++].c_str());
    if (is_print_data) {
      outfile << xp << " " << yp << " " << intensitie << std::endl;
    }
    Eigen::Vector2d temp(xp, yp);
    message->mstruRadarMessage.mstruSingleLayerData.mvPoints.push_back(temp);
    message->mstruRadarMessage.mstruSingleLayerData.mvIntensities.push_back(
                                          intensitie);
  }
  outfile.close();
  return true;
}

SensorData::DMreaderSensoryMessage SensorData::_getQrMessage(
  const std::vector<std::string>& line) {
  DMreaderSensoryMessage qr_message;
  qr_message.time_stamp = atof(line[0].c_str());
  qr_message.x = atof(line[1].c_str());
  qr_message.y = atof(line[2].c_str());
  qr_message.theta = atof(line[3].c_str());
  qr_message.tag_num = atof(line[4].c_str());
  qr_message.read_success = atof(line[5].c_str());
  return qr_message;
}




}  // namespace Simulation
