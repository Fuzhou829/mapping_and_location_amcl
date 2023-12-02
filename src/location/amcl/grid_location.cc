/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-07-25 10:30:26
 * @LastEditTime: 2023-11-01 11:53:11
 * @Author: lcfc-desktop
 */
#include "amcl/grid_location.h"

#include <fstream>

#include "common_lib/time_utils.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

/**
 * @brief
 *
 * @param z
 * @return double
 */
double normalize(double z) { return atan2(sin(z), cos(z)); }

/**
 * @brief
 *
 * @param ang_a
 * @param ang_b
 * @return double
 */
double angle_diff(double ang_a, double ang_b) {
  double d1_ang, d2_ang;
  ang_a = normalize(ang_a);
  ang_b = normalize(ang_b);
  d1_ang = ang_a - ang_b;
  d2_ang = 2 * M_PI - fabs(d1_ang);
  if (d1_ang > 0) d2_ang *= -1.0;
  if (fabs(d1_ang) < fabs(d2_ang))
    return (d1_ang);
  else
    return (d2_ang);
}

/**
 * @brief 初始均布粒子
 *
 * @param arg
 * @return pf_vector_t
 */
amcl::pf_vector_t uniformPoseGenerator(void *arg) {
  amcl::map_t *map = reinterpret_cast<amcl::map_t *>(arg);

  double min_x, max_x, min_y, max_y;

  min_x = map->origin_x - (map->size_x * map->scale) / 2.0;
  max_x = map->origin_x + (map->size_x * map->scale) / 2.0;
  min_y = map->origin_y - (map->size_y * map->scale) / 2.0;
  max_y = map->origin_y + (map->size_y * map->scale) / 2.0;

  amcl::pf_vector_t p;
  int ii = 0;
  for (;;) {
    ii++;
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int i, j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);

    if (MAP_VALID(map, i, j) &&
        (map->cells[MAP_INDEX(map, i, j)].occ_state == -1))
      break;
  }

  return p;
}

/**
 * @brief Construct a new Grid Location:: Grid Location object
 *
 */
GridLocation::GridLocation(const LocationConfig &config) {
  m_LocationConfig = config;
  is_get_amcl_pos_ = false;
  is_get_ladar_ = false;
  pthread_mutex_init(&particle_mutex_, nullptr);
  pthread_cond_init(&cond, nullptr);
  _paramInitial();
  display_ladar_ = std::make_shared<DisPlayResult::SimulationDataPublisher>(
      config.sensor_mount);
}

/**
 * @brief Destroy the Grid Location:: Grid Location object
 *
 */
GridLocation::~GridLocation() {
  if (m_pMap != nullptr) {
    map_free(m_pMap);
    m_pMap = nullptr;
  }
  if (m_pPf != nullptr) {
    pf_free(m_pPf);
    m_pPf = nullptr;
  }
  if (m_pOdom != nullptr) {
    delete m_pOdom;
    m_pOdom = nullptr;
  }
  if (m_pLaser != nullptr) {
    delete m_pLaser;
    m_pLaser = nullptr;
  }
  pthread_mutex_destroy(&particle_mutex_);
  pthread_cond_destroy(&cond);
}

void GridLocation::SetMapData(MapSharedPointer grid_map_) {
  SLAM_DEBUG("设置地图数据\n");
  p_grid_map_ = grid_map_;
}

bool GridLocation::GetCurrentPosition(Position *pos) {
  *pos = current_posotion_;
  return is_get_amcl_pos_;
}

bool GridLocation::IsFinishLocate() { return finish_location_; }

/**
 * @brief
 *
 * @param data
 */
void GridLocation::HandleLaserData(const RadarSensoryMessage &data,
                                   const Position &forecast_pos, bool is_move) {
  // Position odom_temp;
  is_get_amcl_pos_ = false;
  is_get_ladar_ = true;
  {
    std::lock_guard<std::recursive_mutex> lk(ladar_data_mutex_);
    m_LastRadarData = data.mstruRadarMessage;
    m_LastRadarData.mstruRadarHeaderData.mfXPos = forecast_pos.mfX;
    m_LastRadarData.mstruRadarHeaderData.mfYPos = forecast_pos.mfY;
    m_LastRadarData.mstruRadarHeaderData.mfTheta = forecast_pos.mfTheta;
  }
  if (m_LocationConfig.use_landmark && first_laser_) {
    DataFusion::Observation obs;
    obs.GetObsFromLadarData(m_LocationConfig.sensor_mount,
                            m_LocationConfig.land_mark_config, m_LastRadarData);
    int land_mark_point_count = obs.land_mark_.size();
    if (land_mark_point_count <= 0) {
      SLAM_WARN("没有检测到反光柱\n");
      get_landmark_ = false;
      return;
    }
    SLAM_WARN("检测到反光柱数量为%d\n", land_mark_point_count);
    double min_dis = std::numeric_limits<double>::max();
    double min_theta = 0;
    for (int i = 0; i < land_mark_point_count; i++) {
      double delta_x =
          (obs.centers[i](0) - m_LocationConfig.sensor_mount.radar_position_x);
      double delta_y =
          (obs.centers[i](1) - m_LocationConfig.sensor_mount.radar_position_y);
      double x =
          delta_x * cos(m_LocationConfig.sensor_mount.radar_position_theta) +
          delta_y * sin(m_LocationConfig.sensor_mount.radar_position_theta);
      double y =
          -delta_x * sin(m_LocationConfig.sensor_mount.radar_position_theta) +
          delta_y * cos(m_LocationConfig.sensor_mount.radar_position_theta);
      double dis = sqrt(x * x + y * y);
      if (dis < min_dis) {
        min_dis = dis;
        min_theta = atan2(y, x);
      }
    }
    for (auto it : obs.land_mark_) {
      float x = it.second.mx;
      float y = it.second.my;
      double dis = sqrt(x * x + y * y);
      if (dis < min_dis) {
        min_dis = dis;
        min_theta = atan2(y, x);
      }
    }
    min_dis_ = min_dis;
    min_theta_ = min_theta;
    get_landmark_ = true;
    first_laser_ = false;
    SLAM_DEBUG("最短反光柱距离为%f", min_dis_);
  }

#ifdef TEST_DEBUG
  if (!finish_location_ || !is_get_amcl_pos_) return;
  _publishLadarInfoForTest(data.mstruRadarMessage);
// usleep(100000);
#endif
}

/**
 * @brief
 *
 * @param x
 * @param y
 * @param theta
 */
void GridLocation::SetInitPose(float x, float y, float theta) {
  m_InitPoseX = x;
  m_InitPoseY = y;
  m_InitPoseTheta = theta;
  m_PfInit = false;
}

void GridLocation::SetGlobalLocationPos(const Position &pos) {
  m_FirstLocatedInvoked = false;
  finish_location_ = true;
  m_PfInit = false;
  std::lock_guard<std::recursive_mutex> lk(amcl_mutex_);
  m_InitPoseX = pos.mfX;
  m_InitPoseY = pos.mfY;
  m_InitPoseTheta = pos.mfTheta;
  _freeMapDependentMemory(true);
  _jhInitialPf();
  current_posotion_ = pos;
  SLAM_INFO("get location pos immediately (%f %f %f)", current_posotion_.mfX,
            current_posotion_.mfY, current_posotion_.mfTheta);
}

bool GridLocation::StartGlobalLocating() {
  // if the locate thread is already running , firstly stop the thread,
  // then start another locate thread
  if (m_FirstLocatedInvoked == true) {
    m_FirstLocatedInvoked = false;
    usleep(100000);
  }
  // SLAM_INFO("begin initialize Location thread---------");
  finish_location_ = false;
  m_FirstLocatedInvoked = true;
  is_relocation_ = true;
  pthread_create(&m_GlobalLocatingThread, nullptr,
                 _globalLocatingThreadFunction, this);

  return true;
}

bool GridLocation::StartGlobalLocating(bool is_relocation) {
  is_relocation_ = is_relocation;
  if (m_FirstLocatedInvoked == true) {
    m_FirstLocatedInvoked = false;
    usleep(100000);
  }
  // SLAM_INFO("begin initialize Location thread---------\n");
  finish_location_ = false;
  m_FirstLocatedInvoked = true;
  pthread_create(&m_GlobalLocatingThread, nullptr,
                 _globalLocatingThreadFunction, this);
}

/**
 * @brief
 *
 * @param param
 * @return void*
 */
void *GridLocation::_globalLocatingThreadFunction(void *param) {
  GridLocation *ptr = reinterpret_cast<GridLocation *>(param);
  pthread_detach(pthread_self());
  usleep(500000);
  {
    std::lock_guard<std::recursive_mutex> lk(ptr->amcl_mutex_);
    ptr->_freeMapDependentMemory();
    ptr->m_pMap = ptr->_doMapOpen();
    ptr->_jhInitialPf();
  }

  if (!ptr->is_relocation_) {
    ptr->current_posotion_.mfX = ptr->m_InitPoseX;
    ptr->current_posotion_.mfY = ptr->m_InitPoseY;
    ptr->current_posotion_.mfTheta = ptr->m_InitPoseTheta;
    ptr->current_posotion_.mlTimestamp = gomros::common::GetCurrentTime_us();
    SLAM_INFO("do not relocation, get location pos immediately (%f %f %f)",
              ptr->current_posotion_.mfX, ptr->current_posotion_.mfY,
              ptr->current_posotion_.mfTheta);
    ptr->finish_location_ = true;
    ptr->m_FirstLocatedInvoked = false;
  } else {
    ptr->_mainParticleRun();
  }
  pthread_exit(NULL);
  return nullptr;
}

/**
 * @brief 主粒子滤波运行
 *
 */
void GridLocation::_mainParticleRun() {
  // 开机执行初始化一次，初始化粒子滤波器，odom，激光传感器
  bool force_located = false;  // 超时（LOCATE_TIME_EXCEED）强制定位完成
  bool b_ini_located = false;  // 初始定位初始化变量

  uint64_t init_time = gomros::common::GetCurrentTime_us();
  uint64_t real_time = init_time;
  double delta_t = 0.0;
  int LOCATE_TIME_EXCEED = 70 * 1000;
  m_LocateStep = 1;
  // SLAM_INFO("m_FirstLocatedInvoked = %d......\n", m_FirstLocatedInvoked);
  while (m_FirstLocatedInvoked) {
    if (b_ini_located == false) {
      b_ini_located = true;
      force_located = false;
      init_time = gomros::common::GetCurrentTime_us();
      delta_t = 0;  // 清零delta_t
    }
    real_time = gomros::common::GetCurrentTime_us();
    delta_t = static_cast<double>(real_time - init_time) / 1000;  // 单位ms
    // SLAM_INFO("delta_t = %f......\n", delta_t);
    if (delta_t > LOCATE_TIME_EXCEED) {
      SLAM_WARN("grid location time out!!!! force location!!!\n");
      force_located = true;  // 超时强制定位完成
      delta_t = 0;
    }
    if (!is_get_ladar_) continue;
    _particleFilterRun(true);         // 启动粒子滤波器
    _initialLocation(force_located);  // 机器人初始定位线程函数
    usleep(20000);
  }
#ifdef TEST_DEBUG
  _publishLadarInfoForTest(m_LastRadarData);
#endif

  finish_location_ = true;
  // SLAM_INFO("global location is finished<<<<<<<<<<<<");
  return;
}

/**
 * @brief 机器人初始定位线程函数
 *
 * @param bin true 进行强制定位 false 正常全局定位
 * @return true
 * @return false
 */
bool GridLocation::_initialLocation(bool bin) {
  int index_j;
  float particle_radius = 0.2;
  if (m_FirstLocatedInvoked == true) {  // 定位步骤激活
    if (m_LocateScore > 0.90 || bin) {
      amcl::pf_vector_t pf_init_pose_mean = amcl::pf_vector_zero();
      amcl::pf_matrix_t pf_init_pose_cov = amcl::pf_matrix_zero();
      // 使用得到的位置重新初始化粒子群
      // m_LocateStep 初始值为1, bin代表是否强制结束定位
      if (m_LocateStep < 3 && !bin) {  // 启动二次重定位步骤
        usleep(30000);
        m_LocateStep++;
        current_posotion_.mfX = m_AmclPose.v[0];
        current_posotion_.mfY = m_AmclPose.v[1];
        current_posotion_.mfTheta = m_AmclPose.v[2];
        current_posotion_.mlTimestamp =
            m_LastRadarData.mstruRadarHeaderData.mlTimeStamp;
        usleep(200000);
        pf_init_pose_mean = m_AmclPose;
        pf_init_pose_cov.m[0][0] = particle_radius;
        pf_init_pose_cov.m[1][1] = particle_radius;
        pf_init_pose_cov.m[2][2] = 0.1;
        pf_init(m_pPf, pf_init_pose_mean, pf_init_pose_cov);
        m_PfInit = false;
      } else {
        // 全局定位真正结束的地方
        current_posotion_.mfX = m_AmclPose.v[0];
        current_posotion_.mfY = m_AmclPose.v[1];
        current_posotion_.mfTheta = m_AmclPose.v[2];
        current_posotion_.mlTimestamp =
            m_LastRadarData.mstruRadarHeaderData.mlTimeStamp;
        if (bin) usleep(15000);
        m_FirstLocatedInvoked = false;
      }

      usleep(15000);
    }
  }
}

/**
 * @brief 启动粒子滤波器
 *
 * @param fup
 * @return true
 * @return false
 */
bool GridLocation::_particleFilterRun(char fup) {
  // 定义粒子滤波所需局部变量
  std::lock_guard<std::recursive_mutex> lk(amcl_mutex_);
  amcl::pf_vector_t delta;
  bool force_publication;
  bool resampled;
  double angle_increment, range_min, range_max, angle_min, angle_max;
  double max_weight = 0.0;
  int max_weight_hyp = -1;
  double weight;
  bool update;
  double laser_len;
  char resample_upd = 0;

  amcl::pf_sample_set_t *set;
  delta = amcl::pf_vector_zero();
  amcl::pf_vector_t GetOdomPose;

  GetOdomPose = amcl::pf_vector_zero();
  {
    std::lock_guard<std::recursive_mutex> lk(ladar_data_mutex_);
    GetOdomPose.v[0] = m_LastRadarData.mstruRadarHeaderData.mfXPos;
    GetOdomPose.v[1] = m_LastRadarData.mstruRadarHeaderData.mfYPos;
    GetOdomPose.v[2] = m_LastRadarData.mstruRadarHeaderData.mfTheta;
  }

  // // SLAM_INFO("%f %f %f %f %f %f\n", GetOdomPose.v[0], GetOdomPose.v[1],
  //        GetOdomPose.v[2], m_LastOdomPose.v[0], m_LastOdomPose.v[1],
  //        m_LastOdomPose.v[2]);
  // 如果初始化完成，则判断是否更新
  if (m_PfInit) {
    // 当前里程计与上一里程计的差值
    delta.v[0] = GetOdomPose.v[0] - m_LastOdomPose.v[0];
    delta.v[1] = GetOdomPose.v[1] - m_LastOdomPose.v[1];
    delta.v[2] = angle_diff(GetOdomPose.v[2], m_LastOdomPose.v[2]);

    // See if we should update the filter
    update = fabs(delta.v[0]) > pf_param_.m_DThresh ||
             fabs(delta.v[1]) > pf_param_.m_DThresh ||
             fabs(delta.v[2]) > pf_param_.m_AThresh;

    if (fup == 1) update = true;

    // Set the laser update flags
    if (update) m_LasersUpdate = true;
  }
  force_publication = false;
  if (!m_PfInit) {
    // Pose at last filter update
    m_LastOdomPose = GetOdomPose;
    // Filter is now initialized
    m_PfInit = true;

    // Should update sensor data
    m_LasersUpdate = true;

    force_publication = true;
    m_ResampleCount = 0;
  } else if (m_PfInit && m_LasersUpdate) {
    // If the robot has moved, update the filter
    amcl::AMCLOdomData odata;
    odata.pose = GetOdomPose;
    odata.delta = delta;
    // Use the action data to update the filter
    SLAM_WARN("控制更新\n");
    SLAM_DEBUG("控制更新量为%f %f %f\n", odata.delta.v[0], odata.delta.v[1],
               odata.delta.v[2]);
    m_pOdom->UpdateAction(m_pPf,
                          reinterpret_cast<amcl::AMCLSensorData *>(&odata));
  }
  resampled = false;

  // If the robot has moved, update the filter
  if (m_LasersUpdate) {
    amcl::AMCLLaserData ldata;
    ldata.sensor = m_pLaser;
    if (m_LocationConfig.use_landmark) {
      ldata.use_landmark = true;
      DataFusion::Observation obs;
      obs.GetObsFromLadarData(m_LocationConfig.sensor_mount,
                              m_LocationConfig.land_mark_config,
                              m_LastRadarData);
      int land_mark_point_count = obs.land_mark_.size();
      SLAM_WARN("检测到反光柱点云的数量为%d\n", land_mark_point_count);
      if (land_mark_point_count) {
        ldata.range_count2 = land_mark_point_count;
        ldata.ranges2 = new double[land_mark_point_count][2];
        int i = 0;
        for (auto it : obs.land_mark_) {
          float x = it.second.mx;
          float y = it.second.my;
          ldata.ranges2[i][0] = x;
          ldata.ranges2[i][1] = y;
        }
      } else {
        ldata.use_landmark = false;
        ldata.range_count2 = 0;
      }
    } else {
      ldata.use_landmark = false;
    }
    float laser_min_angle = m_LocationConfig.location_start_angle;
    float laser_max_angle = m_LocationConfig.location_end_angle;
    float laser_resolution = m_LocationConfig.location_laser_resolution;
    angle_min = laser_min_angle / 180 * M_PI;
    angle_max = laser_max_angle / 180 * M_PI;
    angle_increment = m_LastRadarData.mstruRadarHeaderData.mfAngleIncreament;
    angle_increment = fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI;
    int stepIncreament =
        std::round(laser_resolution / 180 * M_PI / angle_increment);
    if (stepIncreament < 1) {
      stepIncreament = 1;
    }
    range_max = pf_param_.m_LaserMaxRange;
    range_min = pf_param_.m_LaserMinRange;

    bool is_test = false;
#ifdef TEST_DEBUG
    is_test = true;
#endif
    if (is_test) {
      if ((laser_min_angle / 180 * M_PI) <
              m_LastRadarData.mstruRadarHeaderData.mfAngleMin ||
          (laser_max_angle / 180 * M_PI) >
              m_LastRadarData.mstruRadarHeaderData.mfAngleMax) {
        SLAM_WARN("AMCL locating angle out of the radar angle range.");
        return false;
      }
      int record_cnt = (laser_max_angle - laser_min_angle) / laser_resolution;
      ldata.range_count = record_cnt + 1;
      ldata.range_max = range_max - 1;
      ldata.ranges = new double[ldata.range_count][2];
      assert(ldata.ranges);
      int ladar_count = m_LastRadarData.mstruSingleLayerData.mvPoints.size();
      for (int i = 0; i < ladar_count; i++) {
        double xp = m_LastRadarData.mstruSingleLayerData.mvPoints[i](0);
        double yp = m_LastRadarData.mstruSingleLayerData.mvPoints[i](1);
        laser_len = sqrt(xp * xp + yp * yp);
        if (laser_len <= range_min)
          ldata.ranges[i][0] = ldata.range_max;
        else
          ldata.ranges[i][0] = laser_len;
        // Compute bearing
        ldata.ranges[i][1] =
            angle_min + (i * laser_resolution * M_PI / 180.0f);  // 角度
        // // SLAM_DEBUG("ldata %d %f %f\n", j, ldata.ranges[j][0],
        // ldata.ranges[j][1]);
      }
    } else {
      laser_resolution = angle_increment * stepIncreament;
      if (angle_min < m_LastRadarData.mstruRadarHeaderData.mfAngleMin ||
          angle_min >= m_LastRadarData.mstruRadarHeaderData.mfAngleMax ||
          angle_min >= angle_max) {
        angle_min = m_LastRadarData.mstruRadarHeaderData.mfAngleMin;
      }
      if (angle_max > m_LastRadarData.mstruRadarHeaderData.mfAngleMax ||
          angle_max <= m_LastRadarData.mstruRadarHeaderData.mfAngleMin ||
          angle_max <= angle_min) {
        angle_max = m_LastRadarData.mstruRadarHeaderData.mfAngleMax;
      }
      if (pf_param_.m_LaserMinRange <
              m_LastRadarData.mstruRadarHeaderData.mfRangeMin ||
          pf_param_.m_LaserMinRange >=
              m_LastRadarData.mstruRadarHeaderData.mfRangeMax ||
          pf_param_.m_LaserMinRange >= pf_param_.m_LaserMaxRange) {
        range_min = m_LastRadarData.mstruRadarHeaderData.mfRangeMin;
      } else {
        range_min = pf_param_.m_LaserMinRange;
      }
      if (pf_param_.m_LaserMaxRange >
              m_LastRadarData.mstruRadarHeaderData.mfRangeMax ||
          pf_param_.m_LaserMaxRange <=
              m_LastRadarData.mstruRadarHeaderData.mfRangeMin ||
          pf_param_.m_LaserMinRange >= pf_param_.m_LaserMaxRange) {
        range_max = m_LastRadarData.mstruRadarHeaderData.mfRangeMax;
      } else {
        range_max = pf_param_.m_LaserMaxRange;
      }
      int start_i =
          (angle_min - m_LastRadarData.mstruRadarHeaderData.mfAngleMin) /
          angle_increment;
      int record_cnt = (angle_max - angle_min) / laser_resolution;
      ldata.range_count = record_cnt + 1;
      ldata.range_max = range_max - 1;
      ldata.ranges = new double[ldata.range_count][2];
      record_cnt = record_cnt * stepIncreament;
      assert(ldata.ranges);
      int j = 0;
      for (int i = start_i; i <= (start_i + record_cnt); i += stepIncreament) {
        // amcl doesn't (yet) have a concept of min range.  So we'll map short
        // readings to max range.
        double xp = m_LastRadarData.mstruSingleLayerData.mvPoints[i](0);
        double yp = m_LastRadarData.mstruSingleLayerData.mvPoints[i](1);
        laser_len = sqrt(xp * xp + yp * yp);
        if (laser_len <= range_min)
          ldata.ranges[j][0] = ldata.range_max;
        else
          ldata.ranges[j][0] = laser_len;
        // Compute bearing
        ldata.ranges[j][1] = angle_min + (j * laser_resolution);  // 角度
        j++;
      }
    }
    // SLAM_WARN("控制更新\n");
    m_pLaser->UpdateSensor(m_pPf,
                           reinterpret_cast<amcl::AMCLSensorData *>(&ldata));
    m_LasersUpdate = false;

    // 保存上一步的位置为激光相对于odo的位置
    // Resample the particles
    m_LastOdomPose = GetOdomPose;
    if (!(++m_ResampleCount % pf_param_.m_ResampleInterval)) {
      pf_update_resample(m_pPf, false);
      show_cnt_++;
      amcl::pf_sample_set_t *particle_set = amcl::pf_get_current_set(m_pPf);
      int width = p_grid_map_->map_info.miMapWidth;
      int height = p_grid_map_->map_info.miMapHeight;
      cv_map_image_ = cv_init_map_image_.clone();
      ;
      int cnt = particle_set->sample_count;
      SLAM_WARN("粒子数量为%d!\n", cnt);
      for (int i = 0; i < cnt; i++) {
        int grid_x, grid_y;
        double map_x = (particle_set->samples + i)->pose.v[0];
        double map_y = (particle_set->samples + i)->pose.v[1];
        double theta = (particle_set->samples + i)->pose.v[2];
        // SLAM_DEBUG("#########map_x %d, map_y %d\n", map_x, map_y);
        grid_x = p_grid_map_->map_info.RealXToMapX(map_x) + width;
        grid_y = p_grid_map_->map_info.RealYToMapY(map_y) + height;
        // SLAM_DEBUG("#########grid_x %d, grid_y %d\n", grid_x, grid_y);
        cv_map_image_.at<cv::Vec3b>(grid_y, grid_x)[0] = 0;
        cv_map_image_.at<cv::Vec3b>(grid_y, grid_x)[1] = 0;
        cv_map_image_.at<cv::Vec3b>(grid_y, grid_x)[2] = 255;
        float end_pos_x = map_x + 0.2 * cos(theta);
        float end_pos_y = map_y + 0.2 * sin(theta);
        int end_grid_x = p_grid_map_->map_info.RealXToMapX(end_pos_x) + width;
        int end_grid_y = p_grid_map_->map_info.RealYToMapY(end_pos_y) + height;
        cv::arrowedLine(cv_map_image_, cv::Point(grid_x, grid_y),
                        cv::Point(end_grid_x, end_grid_y),
                        cv::Scalar(0, 0, 255), 3, 8, 0, 0.1);
      }
      for (int i = 0; i < p_grid_map_->datas.size(); i++) {
        if (p_grid_map_->datas[i] == 1) {
          int grid_x, grid_y;
          p_grid_map_->map_info.ArraryIndexToMapIndex(i, &grid_x, &grid_y);
          cv::Point center(grid_x + width, grid_y + height);
          int radius = 5;
          cv::circle(cv_map_image_, center, radius, cv::Scalar(255, 0, 0), -1);
        }
      }
      if (show_cnt_ < 20) {
        string name = std::string("./") + std::to_string(show_cnt_) + ".png";
        cv::imwrite(name, cv_map_image_);
      }
      SLAM_DEBUG("条件变量唤醒\n");
      pthread_mutex_lock(&particle_mutex_);
      resampled = true;
      pthread_cond_signal(&cond);
      pthread_mutex_unlock(&particle_mutex_);
      m_ResampleCount = 0;
    }
    // amcl::pf_sample_set_t *cur_set = amcl::pf_get_current_set(m_pPf);
    // int cnt = cur_set->sample_count;
    // particle_data_.mstruSingleLayerData.mvPoints.clear();
    // particle_data_.mstruSingleLayerData.mvIntensities.clear();
    // for (int i = 0; i < cnt; i++) {
    //   particle_data_.mstruSingleLayerData.mvPoints.emplace_back(
    //       cur_set->samples->pose.v[0], cur_set->samples->pose.v[1]);
    //   particle_data_.mstruSingleLayerData.mvIntensities.emplace_back(
    //       cur_set->samples->pose.v[2]);
    // }

    set = m_pPf->sets + m_pPf->current_set;
    SLAM_INFO("当前周期的粒子数目: %d\n", set->sample_count);
    if (resampled || force_publication) {
      // Read out the current hypotheses
      // int total_count = 0;
      int score_count = 0;
      std::vector<amcl_hyp_t> hyps;
      hyps.resize(m_pPf->sets[m_pPf->current_set].cluster_count);

      for (int hyp_count = 0;
           hyp_count < m_pPf->sets[m_pPf->current_set].cluster_count;
           hyp_count++) {
        amcl::pf_vector_t pose_mean;
        amcl::pf_matrix_t pose_cov;
        if (!pf_get_cluster_stats(m_pPf, hyp_count, &weight, &pose_mean,
                                  &pose_cov)) {
          SLAM_WARN("Couldn't get stats on cluster %d\n", hyp_count);
          break;
        }
        hyps[hyp_count].weight = weight;
        hyps[hyp_count].pf_pose_mean = pose_mean;
        hyps[hyp_count].pf_pose_cov = pose_cov;

        if (hyps[hyp_count].weight > max_weight) {
          max_weight = hyps[hyp_count].weight;
          max_weight_hyp = hyp_count;
        }
      }
      // SLAM_WARN("权重最高聚类对应的id为 %d\n", max_weight_hyp);
      if (finish_location_ == false && max_weight_hyp != -1) {
        for (int i = 0; i < set->sample_count; i++) {
          amcl::pf_sample_t *sample = set->samples + i;
          if (_disAmong(hyps[max_weight_hyp].pf_pose_mean, sample->pose,
                        0.10)) {
            score_count++;
          }
        }
      }
      if (max_weight_hyp != -1) {
        hyps[max_weight_hyp].score = static_cast<float>(score_count) /
                                     (static_cast<float>(set->sample_count));
        m_LocateScore = hyps[max_weight_hyp].score;
      } else {
        SLAM_WARN("AMCL定位失败\n");
      }
      if (max_weight > 0.0) {
        // 最高得分的位置值赋值给全局变量LWG
        m_AmclPose.v[0] = hyps[max_weight_hyp].pf_pose_mean.v[0];
        m_AmclPose.v[1] = hyps[max_weight_hyp].pf_pose_mean.v[1];
        m_AmclPose.v[2] = hyps[max_weight_hyp].pf_pose_mean.v[2];
        current_posotion_.mfX = m_AmclPose.v[0];
        current_posotion_.mfY = m_AmclPose.v[1];
        current_posotion_.mfTheta = m_AmclPose.v[2];
        current_posotion_.mlTimestamp =
            m_LastRadarData.mstruRadarHeaderData.mlTimeStamp;
        is_get_amcl_pos_ = true;
      } else {
        current_posotion_.mfX = GetOdomPose.v[0];
        current_posotion_.mfY = GetOdomPose.v[1];
        current_posotion_.mfTheta = GetOdomPose.v[2];
        current_posotion_.mlTimestamp =
            m_LastRadarData.mstruRadarHeaderData.mlTimeStamp;
      }
    } else {
      current_posotion_.mfX = GetOdomPose.v[0];
      current_posotion_.mfY = GetOdomPose.v[1];
      current_posotion_.mfTheta = GetOdomPose.v[2];
      current_posotion_.mlTimestamp =
          m_LastRadarData.mstruRadarHeaderData.mlTimeStamp;
    }
    cur_pos_ = current_posotion_;
    ready = true;
    for (Eigen::Vector2d &point : radar_data_.mstruSingleLayerData.mvPoints) {
      double x = cur_pos_.mfX + point[0] * cos(cur_pos_.mfTheta) -
                 point[1] * sin(cur_pos_.mfTheta);
      double y = cur_pos_.mfY + point[0] * sin(cur_pos_.mfTheta) +
                 point[1] * cos(cur_pos_.mfTheta);
      point[0] = x;
      point[1] = y;
    }
  }
}

/**
 * @brief 两个空间点之间的距离
 *
 * @param pos1
 * @param pos2
 * @param dis
 * @return true
 * @return false
 */
bool GridLocation::_disAmong(amcl::pf_vector_t pos1, amcl::pf_vector_t pos2,
                             double dis) {
  double dis_ret;
  double dis_x = pos1.v[0] - pos2.v[0];
  if (fabs(dis_x) > dis) return false;
  double dis_y = pos1.v[1] - pos2.v[1];
  if (fabs(dis_y) > dis) return false;

  double v02 = (dis_x) * (dis_x);
  double v12 = (dis_y) * (dis_y);
  dis_ret = sqrt(v02 + v12);
  if (dis_ret > dis)
    return false;
  else
    return true;
}

/**
 * @brief
 *
 */
void GridLocation::_freeMapDependentMemory(bool save_map) {
  if (m_pMap != nullptr && !save_map) {
    map_free(m_pMap);
    m_pMap = nullptr;
  }
  if (m_pPf != nullptr) {
    pf_free(m_pPf);
    m_pPf = nullptr;
  }

  if (m_pOdom != nullptr) {
    delete m_pOdom;
    m_pOdom = nullptr;
  }

  if (m_pLaser != nullptr) {
    delete m_pLaser;
    m_pLaser = nullptr;
  }
}

amcl::map_t *GridLocation::_doMapOpen() {
  amcl::map_t *map = amcl::map_alloc();
  int index_x, index_y;
  int point_count = 0;
  float max_x, max_y;
  float min_x, min_y;
  float threshold = 0.6;
  int width = p_grid_map_->map_info.miMapWidth;
  int height = p_grid_map_->map_info.miMapHeight;
  // SLAM_INFO("width=%d,height=%d", width, height);
  map->scale = p_grid_map_->map_info.mdResolution;
  // SLAM_INFO("map->resolution:%f\t", map->scale);
  map->size_x = width;
  // SLAM_INFO("map->size_x:%d\t", map->size_x);
  if (map->size_x <= 0) return map;
  map->size_y = height;
  // SLAM_INFO("map->size_y:%d\t", map->size_y);
  if (map->size_y <= 0) return map;

  float m_scale = map->scale;  // 缩放因子==分辨率

  min_x = p_grid_map_->map_info.mdOriginXInWorld;
  min_y = p_grid_map_->map_info.mdOriginYInWorld;
  // SLAM_INFO("min_x = %f, min_y = %f\t", min_x, min_y);

  map->origin_x = (map->size_x / 2) * m_scale + min_x;
  map->origin_y = (map->size_y / 2) * m_scale + min_y;

  // SLAM_INFO("origin_x = %f,origin_y = %f\n", map->origin_x, map->origin_y);

  map->cells = reinterpret_cast<amcl::map_cell_t *>(
      malloc(sizeof(amcl::map_cell_t) * map->size_x * map->size_y));
  cv_init_map_image_ = cv::Mat(cv::Size(width * 3, height * 3), CV_8UC3,
                               cv::Scalar(255, 255, 255));
  char grid_value;
  int xi, yi;
  // FILE* map_data_file;
  // map_data_file = fopen("./test_data/map.txt", "w+");
  usleep(300000);
  for (int i = 0; i < map->size_x * map->size_y; i++) {
    map->cells[i].occ_state = -1;  // 先全部初始化为FREE
  }
  int total_height = height - 1;
  int landmark_cnt = 0;
  for (index_y = 0; index_y < height; index_y++) {
    for (index_x = 0; index_x < width; index_x++) {
      xi = index_x;
      yi = index_y;
      grid_value =
          p_grid_map_
              ->datas[p_grid_map_->get_info().MapIndexToArraryIndex(xi, yi)];
      if (grid_value == 1) {
        landmark_cnt++;
        // 获取反光柱坐标
        amcl::pf_vector_t landmark_pos;
        landmark_pos.v[0] = p_grid_map_->get_info().MapXToRealX(xi);
        landmark_pos.v[1] = p_grid_map_->get_info().MapYToRealY(yi);
        landmarks_.push_back(landmark_pos);
        int grid_x = MAP_GXWX(map, landmark_pos.v[0]);
        int grid_y = MAP_GYWY(map, landmark_pos.v[1]);
        int cell_radius = floor(m_LocationConfig.land_mark_config.landmark_radius / map->scale + 0.5);
        for (int xx = -cell_radius; xx <= cell_radius; xx++) {
          for (int yy = -cell_radius; yy <= cell_radius; yy++) {
            if (abs(xx) + abs(yy) > 3) continue;
            if (xx + grid_x < 0 || xx + grid_x > width || yy + grid_y < 0 || yy + grid_y > height) continue;
            map->landmraks.emplace_back(xx + grid_x, yy + grid_y);
          }
        }
        map->cells[(index_y)*width + (index_x)].occ_state = +2;
      } else if (grid_value == 255) {
        map->cells[(index_y)*width + (index_x)].occ_state = -1;
      } else if (grid_value > 50 && grid_value < 255) {
        map->cells[(index_y)*width + (index_x)].occ_state = +1;
        cv_init_map_image_.at<cv::Vec3b>(index_y + height, index_x + width)[0] =
            0;
        cv_init_map_image_.at<cv::Vec3b>(index_y + height, index_x + width)[1] =
            0;
        cv_init_map_image_.at<cv::Vec3b>(index_y + height, index_x + width)[2] =
            0;
        // if (map_data_file != nullptr) {
        //   fprintf(map_data_file, "%d %d\n", index_x, index_y);
        // } else {
        //   std::cout << "地图文件打开失败!" << std::endl;
        // }
      }
    }
  }

  SLAM_WARN("地图中反光柱的个数为%d\n", landmark_cnt);

  // fclose(map_data_file);
  // SLAM_INFO("load map finished<<<<<<<<<<<<<<<");
  return map;
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool GridLocation::_jhInitialPf() {
  // SLAM_INFO("成功加载地图，执行粒子初始化.");
  _particleFilterInitialize();
  // 设定激光的安装位置变量
  amcl::pf_vector_t laser_pose_v;
  laser_pose_v.v[0] = m_RadarPosition.mfX;
  laser_pose_v.v[1] = m_RadarPosition.mfY;
  laser_pose_v.v[2] = m_RadarPosition.mfTheta;

  SLAM_INFO("ladar mount pos -> %f %f %f\n", m_RadarPosition.mfX,
            m_RadarPosition.mfY, m_RadarPosition.mfTheta);
  m_pLaser->SetLaserPose(laser_pose_v);

  return true;
}

/**
 * @brief 创建初始化粒子滤波器
 *
 * @return int
 */
int GridLocation::_particleFilterInitialize() {
  // Create the particle filter
  if (m_pPf != nullptr) {
    pf_free(m_pPf);
    m_pPf = nullptr;
  }
  m_pPf = amcl::pf_alloc(pf_param_.m_MinParticles, pf_param_.m_MaxParticles,
                         pf_param_.m_RecoveryAlphaSlow,
                         pf_param_.m_RecoveryAlphaFast,
                         (amcl::pf_init_model_fn_t)uniformPoseGenerator,
                         reinterpret_cast<void *>(m_pMap));
  m_pPf->pop_err = pf_param_.m_KldErr;
  m_pPf->pop_z = pf_param_.m_KldZ;
  // Initialize the filter
  amcl::pf_vector_t pf_init_pose_mean = amcl::pf_vector_zero();
  pf_init_pose_mean.v[0] = m_InitPoseX;
  pf_init_pose_mean.v[1] = m_InitPoseY;
  pf_init_pose_mean.v[2] = m_InitPoseTheta;

  m_AmclPose = pf_init_pose_mean;

  amcl::pf_matrix_t pf_init_pose_cov = amcl::pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = 0.5;
  pf_init_pose_cov.m[1][1] = 0.5;
  pf_init_pose_cov.m[2][2] = 0.15;
  // 如果初始位姿不为0，则进行局部撒粒子，否则全局撒粒子
  if (m_LocationConfig.use_landmark) {
    SLAM_WARN("使用反光柱初始化粒子!!!!!!!!!!!!!\n");
    while (!get_landmark_) usleep(500000);
    SLAM_WARN("开始使用反光柱初始化粒子!!!!!!!!!!!!!\n");
    pf_init_landmark(m_pPf, min_dis_, 0.01, min_theta_, 0.07, landmarks_);
    amcl::pf_sample_set_t *particle_set = amcl::pf_get_current_set(m_pPf);
    int width = p_grid_map_->map_info.miMapWidth;
    int height = p_grid_map_->map_info.miMapHeight;
    cv_map_image_ = cv_init_map_image_.clone();
    int cnt = particle_set->sample_count;
    SLAM_WARN("粒子数量为%d!\n", cnt);
    for (int i = 0; i < cnt; i++) {
      int grid_x, grid_y;
      double map_x = (particle_set->samples + i)->pose.v[0];
      double map_y = (particle_set->samples + i)->pose.v[1];
      double theta = (particle_set->samples + i)->pose.v[2];
      // SLAM_DEBUG("#########map_x %d, map_y %d\n", map_x, map_y);
      grid_x = p_grid_map_->map_info.RealXToMapX(map_x) + width;
      grid_y = p_grid_map_->map_info.RealYToMapY(map_y) + height;
      // SLAM_DEBUG("#########grid_x %d, grid_y %d\n", grid_x, grid_y);
      cv_map_image_.at<cv::Vec3b>(grid_y, grid_x)[0] = 0;
      cv_map_image_.at<cv::Vec3b>(grid_y, grid_x)[1] = 0;
      cv_map_image_.at<cv::Vec3b>(grid_y, grid_x)[2] = 255;
      float end_pos_x = map_x + 0.2 * cos(theta);
      float end_pos_y = map_y + 0.2 * sin(theta);
      int end_grid_x = p_grid_map_->map_info.RealXToMapX(end_pos_x) + width;
      int end_grid_y = p_grid_map_->map_info.RealYToMapY(end_pos_y) + height;
      cv::arrowedLine(cv_map_image_, cv::Point(grid_x, grid_y),
                      cv::Point(end_grid_x, end_grid_y), cv::Scalar(0, 0, 255),
                      3, 8, 0, 0.1);
    }
    for (int i = 0; i < p_grid_map_->datas.size(); i++) {
      if (p_grid_map_->datas[i] == 1) {
        int grid_x, grid_y;
        p_grid_map_->map_info.ArraryIndexToMapIndex(i, &grid_x, &grid_y);
        cv::Point center(grid_x + width, grid_y + height);
        int radius = 5;
        cv::circle(cv_map_image_, center, radius, cv::Scalar(255, 0, 0), -1);
      }
    }
    cv::imwrite("./0.png", cv_map_image_);
    show_cnt_ = 0;

  } else {
    SLAM_WARN("不使用反光柱初始化粒子!!!!!!!!!!!!!\n");
    if ((fabs(m_InitPoseX) > fuzzy_1) || (fabs(m_InitPoseY) > fuzzy_1)) {
      pf_init(m_pPf, pf_init_pose_mean, pf_init_pose_cov);
      // SLAM_INFO("Initializing with uniform distribution");
    } else {
      pf_init_model(m_pPf, (amcl::pf_init_model_fn_t)uniformPoseGenerator,
                    reinterpret_cast<void *>(m_pMap));
      // SLAM_INFO("Global initialisation done!");
    }
  }
  m_PfInit = false;
  // Instantiate the sensor objects
  // Odometry
  if (m_pOdom != nullptr) {
    delete m_pOdom;
    m_pOdom = nullptr;
  }
  m_pOdom = new amcl::AMCLOdom();
  assert(m_pOdom);
  if (m_OdomType == 0)
    m_pOdom->SetModelDiff(pf_param_.m_OdomAlpha1, pf_param_.m_OdomAlpha2,
                          pf_param_.m_OdomAlpha3, pf_param_.m_OdomAlpha4);
  else
    m_pOdom->SetModelOmni(pf_param_.m_OdomAlpha1, pf_param_.m_OdomAlpha2,
                          pf_param_.m_OdomAlpha3, pf_param_.m_OdomAlpha4,
                          pf_param_.m_OdomAlpha5);
  // Laser
  if (m_pLaser != nullptr) {
    delete m_pLaser;
    m_pLaser = nullptr;
  }
  m_pLaser = new amcl::AMCLLaser(pf_param_.m_LaserMaxBeams, m_pMap);
  assert(m_pLaser);
  SLAM_INFO(
      "Initializing likelihood field model; this can take some time on large "
      "maps...");
  SLAM_INFO("get map_data_file_path: %s",
            m_LocationConfig.map_data_file_path.c_str());
  SLAM_INFO("debug: get use_weighted_areas: %d",
            m_LocationConfig.use_weighted_areas);
  m_pLaser->SetWeightedAreasMapFilePath(m_LocationConfig.map_data_file_path);
  m_pLaser->SetUseWeightedAreas(m_LocationConfig.use_weighted_areas);
  m_pLaser->SetUseLandmark(m_LocationConfig.use_landmark);
  m_pLaser->SetModelLikelihoodField(
      pf_param_.m_LaserZHit, pf_param_.m_LaserZRand, pf_param_.m_LaserSigmaHit,
      pf_param_.m_LaserLikelihoodMaxDist);
  m_pLaser->SetGridWeight(m_LocationConfig.grid_weight);
  m_pLaser->SetLandmarkWeight(m_LocationConfig.landmark_weight);
  SLAM_INFO("Done initializing likelihood field model.");
  return 1;
}

/**
 * @brief 参数初始化，在构造函数中调用
 *
 */
void GridLocation::_paramInitial() {
  pf_param_.m_LaserMinRange = m_LocationConfig.laser_min_range;
  pf_param_.m_LaserMaxRange = m_LocationConfig.laser_max_range;
  pf_param_.m_LaserModelType = amcl::LASER_MODEL_LIKELIHOOD_FIELD;

  m_RadarPosition.mfX = m_LocationConfig.sensor_mount.radar_position_x;
  m_RadarPosition.mfY = m_LocationConfig.sensor_mount.radar_position_y;
  m_RadarPosition.mfTheta = m_LocationConfig.sensor_mount.radar_position_theta;
  if (m_LocationConfig.odom_type == OdomType::DiffWheelModel ||
      m_LocationConfig.odom_type == OdomType::DoubleSteerModel)
    m_OdomType = m_LocationConfig.odom_type;
  else
    m_OdomType = 0;

  m_ResampleCount = 0;

  m_PfInit = false;
  m_LocateStep = 0;

  finish_location_ = false;
}

void GridLocation::_publishLadarInfoForTest(const RadarSensoryInfo &raw_ladar) {
  RadarSensoryInfo ladar_info = raw_ladar;
  Position ladar_pos = current_posotion_ * m_RadarPosition;
  // 雷达帧位姿信息
  ladar_info.mstruRadarHeaderData.mfXPos = ladar_pos.mfX;
  ladar_info.mstruRadarHeaderData.mfYPos = ladar_pos.mfY;
  ladar_info.mstruRadarHeaderData.mfTheta = ladar_pos.mfTheta;
  ladar_info.mstruRadarHeaderData.mcPosValid = is_get_amcl_pos_;
  // 显示粒子信息
  amcl::pf_sample_set_t *set;
  amcl::pf_sample_t *smaple_pose;
  set = m_pPf->sets + m_pPf->current_set;
  for (int i = 0; i < set->sample_count; i++) {
    smaple_pose = set->samples + i;
    pcl::PointXYZI pcl_pose;
    pcl_pose.x = smaple_pose->pose.v[0];
    pcl_pose.y = smaple_pose->pose.v[1];
    pcl_pose.z = smaple_pose->pose.v[2];  // 角度
    pcl_pose.intensity = smaple_pose->weight;
    ladar_info.mstruMultiLayerData.push_back(pcl_pose);
  }

  display_ladar_->DisplayLadarMessage(ladar_info);
}

gomros::message::RadarSensoryInfo GridLocation::GetLaserData() {
  pthread_mutex_lock(&particle_mutex_);
  while (!ready) {
    pthread_cond_wait(&cond, &particle_mutex_);
  }
  pthread_mutex_unlock(&particle_mutex_);
  return radar_data_;
}
gomros::message::RadarSensoryInfo GridLocation::GetParticleData() {
  pthread_mutex_lock(&particle_mutex_);
  while (!ready) {
    pthread_cond_wait(&cond, &particle_mutex_);
  }
  pthread_mutex_unlock(&particle_mutex_);
  return particle_data_;
}
gomros::message::Position GridLocation::GetCurrentPose() {
  SLAM_ERROR("等待线程锁释放\n");
  pthread_mutex_lock(&particle_mutex_);
  while (!ready) {
    pthread_cond_wait(&cond, &particle_mutex_);
  }
  // 此时条件满足
  ready = false;  // 重置条件
  pthread_mutex_unlock(&particle_mutex_);
  SLAM_ERROR("线程锁释放\n");
  return cur_pos_;
}

cv::Mat GridLocation::GetImage() {
  SLAM_ERROR("等待线程锁释放\n");
  pthread_mutex_lock(&particle_mutex_);
  while (!ready) {
    pthread_cond_wait(&cond, &particle_mutex_);
  }
  ready = false;  // 重置条件
  pthread_mutex_unlock(&particle_mutex_);
  SLAM_ERROR("线程锁释放\n");
  return cv_map_image_;
}

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
