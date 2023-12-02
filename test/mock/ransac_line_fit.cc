/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: Do not edit
 * @LastEditors: renjy
 * @LastEditTime: 2023-06-29 11:07:40
 */

#include <utility>
#include <algorithm>
#include <map>
#include <vector>

#include "mock/ransac_line_fit.h"
#include "common/logger.h"
#include "include/mapping_and_location_math.h"
namespace LineFit {

MapFit::MapFit(const std::string& file_dir, const std::string& save_file) {
  save_file_ = save_file;
  _loadMapFile(file_dir);
}

void MapFit::_loadMapFile(const std::string& file_dir) {
  std::string char_string;
  std::ifstream infile(file_dir);
  if (!infile.is_open()) {
    SLAM_WARN("open file is %s fail", file_dir.c_str());
    return;
  }
  SLAM_INFO("open file is %s", file_dir.c_str());
  std::getline(infile, char_string);
  const char* map_to_parse = char_string.c_str();
  Json::Reader reader;
  Json::Value map_json;
  if (!reader.parse(map_to_parse, map_json)) {
    SLAM_ERROR("load slam grid map error!!!!!!!!!!!!");
    return;
  }
  _oneLineAveTheta(map_json);
  // _ransacLineAndTheta(map_json);
  // _deletqrinfo(map_json);
}

void MapFit::_oneLineAveTheta(const Json::Value& map_json) {
  Json::Value my_map_json = map_json;
  std::map<int, QRInfo> qr_search;
  for (int i = 0; i < my_map_json["qr_pos_list"].size(); i++) {
    QRInfo qr_info;
    qr_info.id = my_map_json["qr_pos_list"][i]["tag_num"].asInt();
    qr_info.x = my_map_json["qr_pos_list"][i]["x"].asDouble();
    qr_info.y = my_map_json["qr_pos_list"][i]["y"].asDouble();
    qr_info.theta = my_map_json["qr_pos_list"][i]["theta"].asDouble();
    // if ((qr_info.id >=311 && qr_info.id <= 327) || qr_info.id == 201) {
    //   qr_info.x += 0.11;
    // }
    qr_search.insert(std::make_pair(qr_info.id, qr_info));
  }
  std::map<int, float> qr_thetas_ave;
  std::map<int, float> thetas;
  bool get_standard_theta = false;
  float standard_theta = 0.0;
  for (int i = 0; i < my_map_json["qr_pos_list"].size(); i++) {
    if (thetas.empty()) {
      thetas.insert(
        std::make_pair(my_map_json["qr_pos_list"][i]["tag_num"].asInt(),
                      my_map_json["qr_pos_list"][i]["theta"].asDouble()));
      continue;
    }
    if (fabs(my_map_json["qr_pos_list"][i]["theta"].asDouble() -
            thetas.begin()->second) < 0.5) {
      thetas.insert(std::make_pair(
                    my_map_json["qr_pos_list"][i]["tag_num"].asInt(),
                    my_map_json["qr_pos_list"][i]["theta"].asDouble()));
    } else {
      float theta_sum = 0.0;
      float theta_max = -FLT_MAX;
      float theta_min = FLT_MAX;
      for (auto itter = thetas.begin(); itter != thetas.end(); itter++) {
        if (itter->second > theta_max) {
          theta_max = itter->second;
        }
        if (itter->second < theta_min) {
          theta_min = itter->second;
        }
        theta_sum += itter->second;
      }
      // theta_sum = theta_sum - theta_max - theta_min;
      theta_sum = theta_sum / (thetas.size());
      if (!get_standard_theta) {
        get_standard_theta = true;
        standard_theta = theta_sum;
      }
      float delta_theta = theta_sum - standard_theta;
      int correction = delta_theta / (0.5 * M_PI - 0.25);
      theta_sum =
        SLAMMath::NormalizePITheta(0.5 * correction * M_PI + standard_theta);
      for (auto itter = thetas.begin(); itter != thetas.end(); itter++) {
        itter->second = theta_sum;
        qr_thetas_ave.insert(std::make_pair(itter->first, itter->second));
      }
      thetas.clear();
      thetas.insert(std::make_pair(
        my_map_json["qr_pos_list"][i]["tag_num"].asInt(),
        my_map_json["qr_pos_list"][i]["theta"].asDouble()));
    }
  }
  if (!thetas.empty()) {
    float theta_sum = 0.0;
    float theta_max = -FLT_MAX;
    float theta_min = FLT_MAX;
    for (auto itter = thetas.begin(); itter != thetas.end(); itter++) {
      if (itter->second > theta_max) {
        theta_max = itter->second;
      }
      if (itter->second < theta_min) {
        theta_min = itter->second;
      }
      theta_sum += itter->second;
    }
    // theta_sum = theta_sum - theta_max - theta_min;
    theta_sum = theta_sum / (thetas.size());
    if (!get_standard_theta) {
      get_standard_theta = true;
      standard_theta = theta_sum;
    }
    float delta_theta = theta_sum - standard_theta;
    int correction = delta_theta / (0.5 * M_PI - 0.25);
    theta_sum =
      SLAMMath::NormalizePITheta(0.5 * correction * M_PI + standard_theta);
    for (auto itter = thetas.begin(); itter != thetas.end(); itter++) {
      itter->second = theta_sum;
      qr_thetas_ave.insert(std::make_pair(itter->first, itter->second));
    }
    thetas.clear();
  }
  std::string qr_map_id_filr = "/media/sf_share/1111111111/corr_id.txt";
  std::ifstream qr_map_corr_id;
  qr_map_corr_id.open(qr_map_id_filr.c_str(), std::ifstream::in);
  if (!qr_map_corr_id.is_open()) {
    SLAM_ERROR("can not open, please check this file %s",
      qr_map_id_filr.c_str());
    return;
  }
  std::string line_read;
  // y = kx + b;
  int line_id = 0;
  while (getline(qr_map_corr_id, line_read)) {
    int id_1, id_2, start_pos, end_pos;
    std::istringstream iss(line_read);
    if (!(iss >> start_pos >> end_pos >> id_1 >> id_2)) {
      break;
    }
    line_id++;
    double k, b;
    if (fabs(qr_search[start_pos].x - qr_search[end_pos].x) >
          fabs(qr_search[start_pos].y - qr_search[end_pos].y)) {
      k = (qr_search[start_pos].y - qr_search[end_pos].y) /
          (qr_search[start_pos].x - qr_search[end_pos].x);
      b = qr_search[start_pos].y - k * qr_search[start_pos].x;
    } else {
       k = (qr_search[start_pos].x - qr_search[end_pos].x) /
           (qr_search[start_pos].y - qr_search[end_pos].y);
       b = qr_search[start_pos].x - k * qr_search[start_pos].y;
    }

    std::vector<QRInfo> qrs;
    for (auto iter = qr_search.begin(); iter != qr_search.end(); iter++) {
      if (iter->first <= std::max(id_1, id_2) &&
          iter->first >= std::min(id_1, id_2)) {
        iter->second.line_id = line_id;
        qrs.push_back(iter->second);
      }
    }
    for (int i = 0; i < qrs.size(); i++) {
      int xxx = 0;
      if (fabs(qr_search[start_pos].x - qr_search[end_pos].x) >
          fabs(qr_search[start_pos].y - qr_search[end_pos].y)) {
        qr_search[qrs[i].id].y = k * qrs[i].x + b;
        float theta = SLAMMath::NormalizePITheta(atan(k));
        float line_theta = -SLAMMath::NormalizePITheta(atan(1.0f / k));
        // if (qr_thetas_ave[qrs[i].id] < 0) {
        //     line_theta = -1.0 * line_theta;
        // }
        SLAM_WARN("line theta %f, qr_search %f %f\n",
                  theta, qr_thetas_ave[qrs[i].id], line_theta);
      } else {
        qr_search[qrs[i].id].x = k * qrs[i].y + b;
        float theta = SLAMMath::NormalizePITheta(atan(1 / k));
        float line_theta = -SLAMMath::NormalizePITheta(atan(k));
        // if (qr_thetas_ave[qrs[i].id] < 0) {
        //     line_theta = -1.0 * line_theta;
        // }
        SLAM_WARN("line theta %f, qr_search %f %f\n",
                  theta, qr_thetas_ave[qrs[i].id], line_theta);
      }
    }
  }

  int qr_count = 0;
  Json::Value qr_point;
  for (int i = 0; i < my_map_json["qr_pos_list"].size(); i++) {
    Json::Value qr_info;
    qr_info["x"] =
      qr_search[my_map_json["qr_pos_list"][i]["tag_num"].asInt()].x;
    // my_map_json["qr_pos_list"][i]["x"].asDouble();
    qr_info["y"] =
      qr_search[my_map_json["qr_pos_list"][i]["tag_num"].asInt()].y;
    // my_map_json["qr_pos_list"][i]["y"].asDouble();
    qr_info["theta"] =
      qr_thetas_ave[my_map_json["qr_pos_list"][i]["tag_num"].asInt()];
    qr_info["tag_num"] = my_map_json["qr_pos_list"][i]["tag_num"].asInt();
    qr_point[qr_count] = qr_info;
    qr_count++;
  }
  my_map_json["qr_pos_list"].clear();
  my_map_json["qr_pos_list"] = qr_point;

  char new_qr_file_name[1024];
  snprintf(new_qr_file_name, 1024, "./new_qr_map.txt");  // NOLINT
  std::ofstream new_qroutfile(new_qr_file_name);
  if (!new_qroutfile.is_open()) {
    std::cerr << " error error!! " << std::endl;
    return;
  }

  for (int i = 0; i < my_map_json["qr_pos_list"].size(); i++) {
    QRInfo qr_info;
    qr_info.id = my_map_json["qr_pos_list"][i]["tag_num"].asInt();
    qr_info.x = my_map_json["qr_pos_list"][i]["x"].asDouble();
    qr_info.y = my_map_json["qr_pos_list"][i]["y"].asDouble();
    qr_info.theta = my_map_json["qr_pos_list"][i]["theta"].asDouble();
    new_qroutfile << qr_info.id << " " << qr_info.x << " "
                  << qr_info.y << " "<< qr_info.theta << std::endl;
  }
  std::string map_second_name = save_file_ + "now_map.smap";
  SLAM_INFO("save map!!!!!!!!!!!!!!!!!!!!!");
  internal_common::Save(map_second_name.c_str(), my_map_json);
}

void MapFit::_ransacLineAndTheta(const Json::Value& map_json) {
  RanscanFit ranscan_fit;
  std::string qr_map_id_filr = "/media/sf_share/1111111111/corr_id.txt";
  std::ifstream qr_map_corr_id;
  qr_map_corr_id.open(qr_map_id_filr.c_str(), std::ifstream::in);
  if (!qr_map_corr_id.is_open()) {
    SLAM_ERROR("can not open, please check this file %s",
                qr_map_id_filr.c_str());
    return;
  }
  std::map<int, QRInfo> qr_search;
  int line_id = 0;
  // TODO(r) 还没写完
  for (int i = 0; i < map_json["qr_pos_list"].size(); i++) {
    QRInfo qr_info;
    qr_info.id = map_json["qr_pos_list"][i]["tag_num"].asInt();
    qr_info.x = map_json["qr_pos_list"][i]["x"].asDouble();
    qr_info.y = map_json["qr_pos_list"][i]["y"].asDouble();
    qr_info.theta = map_json["qr_pos_list"][i]["theta"].asDouble();
    if (qr_search.empty()) {
      qr_info.line_id = line_id;
      qr_search.insert(std::make_pair(qr_info.id, qr_info));
    } else {
      bool is_have_line = false;
      for (auto iter = qr_search.begin(); iter != qr_search.end(); iter++) {
        bool check_dis = fabs(iter->second.x - qr_info.x) < 0.3 ||
                         fabs(iter->second.y - qr_info.y) < 0.3;
        bool check_theta =
          fabs(SLAMMath::NormalizePITheta(iter->second.theta - qr_info.theta))
          < 0.3;
        if (check_dis && check_theta && iter->second.line_id != -1) {
           is_have_line = true;
           qr_info.line_id = iter->second.line_id;
           qr_search.insert(std::make_pair(qr_info.id, qr_info));
        }
      }
      if (!is_have_line) {
        line_id++;
        qr_info.line_id = line_id;
        qr_search.insert(std::make_pair(qr_info.id, qr_info));
      }
    }
  }





  std::string line_read;
  // y = kx + b;
  // 按照顺序 筛选qr
  while (getline(qr_map_corr_id, line_read)) {
    int id_1, id_2;
    std::istringstream iss(line_read);
    if (!(iss >> id_1 >> id_2)) {
      break;
    }
    if (!qr_search.count(id_1) || !qr_search.count(id_2)) {
      SLAM_ERROR("there is no %d or %d", id_1, id_2);
      continue;
    }
    if (id_1 == id_2) continue;
    int max_id = std::max(id_1, id_2);
    int min_id = std::min(id_1, id_2);
    std::vector<FitNum> qr_fit_num;
    _getFitQr(min_id, max_id, map_json, &qr_fit_num);
    ranscan_fit.SetData(qr_fit_num);
    double k, b;
    ranscan_fit.GetLineParam(&k, &b);
    bool is_use_x = fabs(qr_search[id_1].x - qr_search[id_2].x) >
                    fabs(qr_search[id_1].y - qr_search[id_2].y);
    for (auto iter = qr_fit_num.begin(); iter != qr_fit_num.end(); iter++) {
      if (is_use_x) {
        qr_search[iter->id].y = k * qr_search[iter->id].x + b;
      } else {
        qr_search[iter->id].x = (qr_search[iter->id].y - b) / k;
      }
    }
  }
  Json::Value my_map_json = map_json;
  int qr_count = 0;
  Json::Value qr_point;
  for (int i = 0; i < my_map_json["qr_pos_list"].size(); i++) {
    Json::Value qr_info;
    qr_info["x"] =
      qr_search[my_map_json["qr_pos_list"][i]["tag_num"].asInt()].x;
    qr_info["y"] =
      qr_search[my_map_json["qr_pos_list"][i]["tag_num"].asInt()].y;

    qr_info["theta"] =
      qr_info["theta"] = my_map_json["qr_pos_list"][i]["theta"].asDouble();
    qr_info["tag_num"] = my_map_json["qr_pos_list"][i]["tag_num"].asInt();
    qr_point[qr_count] = qr_info;
    qr_count++;
  }
  my_map_json["qr_pos_list"].clear();
  my_map_json["qr_pos_list"] = qr_point;

  std::string map_second_name = save_file_ + "ransac.smap";
  internal_common::Save(map_second_name.c_str(), my_map_json);
}

void MapFit::_deletqrinfo(const Json::Value& map_json) {
  Json::Value my_map_json = map_json;
  int qr_count = 0;
  Json::Value qr_point;
  for (int i = 0; i < my_map_json["qr_pos_list"].size(); i++) {
    Json::Value qr_info;
    if (my_map_json["qr_pos_list"][i]["tag_num"].asInt() != 201) continue;
    qr_info["x"] = my_map_json["qr_pos_list"][i]["x"].asDouble();
     qr_info["y"] = my_map_json["qr_pos_list"][i]["y"].asDouble();
    qr_info["theta"] = my_map_json["qr_pos_list"][i]["theta"].asDouble();
    qr_info["tag_num"] = my_map_json["qr_pos_list"][i]["tag_num"].asInt();
    qr_point[qr_count] = qr_info;
    qr_count++;
  }
  my_map_json["qr_pos_list"].clear();
  my_map_json["qr_pos_list"] = qr_point;
  std::string map_second_name = save_file_ + "1111111111.smap";
  internal_common::Save(map_second_name.c_str(), my_map_json);
}


void MapFit::_getFitQr(int min_id, int max_id,
  const Json::Value& map_json, std::vector<FitNum>* fit_qr) {
  for (int i = 0; i < map_json["qr_pos_list"].size(); i++) {
    int id = map_json["qr_pos_list"][i]["tag_num"].asInt();
    if (id > max_id || id < min_id) continue;
    FitNum fit_qr_info;
    fit_qr_info.id = id;
    fit_qr_info.mx = map_json["qr_pos_list"][i]["x"].asDouble();
    fit_qr_info.my = map_json["qr_pos_list"][i]["y"].asDouble();
    fit_qr_info.theta = map_json["qr_pos_list"][i]["theta"].asDouble();
    fit_qr->push_back(fit_qr_info);
  }
}



void RanscanFit::SetData(const std::vector<FitNum>& points) {
  _fitLine(points, &a_, &b_, &c_);
}

void RanscanFit::GetLineParam(double* k, double* b) {
  *k = -a_/b_;
  *b = -c_/b_;
}

void RanscanFit::_fitLine(
  const std::vector<FitNum>& ptSet, double* a, double* b,
  double* c) {
  double residual_error = 3;
  int sample_count = 0;
  int N = 300;
  double res = 0;
  bool stop_loop = false;
  int maximum = 0;

  std::vector<bool> inlierFlag = std::vector<bool>(ptSet.size(), false);
  std::vector<double> resids_(ptSet.size(), 3);
  srand((unsigned int)time(NULL));
  std::vector<int> ptsID;

  for (unsigned int i = 0; i < ptSet.size(); i++)
    ptsID.push_back(i);
  while (N > sample_count && !stop_loop) {
    std::vector<bool> inlierstemp;
    std::vector<double> residualstemp;
    std::vector<int> ptss;
    int inlier_count = 0;
    if (!_getSample(ptsID, &ptss)) {
      stop_loop = true;
      continue;
    }

    std::vector<FitNum> pt_sam;
    pt_sam.push_back(ptSet[ptss[0]]);
    pt_sam.push_back(ptSet[ptss[1]]);

    if (!_verifyComposition(pt_sam)) {
      ++sample_count;
      continue;
    }
    _calcLinePara(pt_sam, a, b, c, &res);
    for (unsigned int i = 0; i < ptSet.size(); i++) {
      FitNum pt = ptSet[i];
      double resid_ = fabs(pt.mx * (*a) + pt.my * (*b) + (*c));
      residualstemp.push_back(resid_);
      inlierstemp.push_back(false);
      if (resid_ < residual_error) {
        ++inlier_count;
        inlierstemp[i] = true;
      }
    }
    if (inlier_count >= maximum) {
      maximum = inlier_count;
      resids_ = residualstemp;
      inlierFlag = inlierstemp;
    }
    if (inlier_count == 0) {
      N = 500;
    } else {
      double epsilon =
        1.0 - inlier_count / (1.0f * static_cast<int>(ptSet.size()));
      double p = 0.99;
      double s = 2.0;
      N = static_cast<int>(log(1.0 - p) / log(1.0 - pow((1.0 - epsilon), s)));
    }
    ++sample_count;
  }
  std::vector<FitNum> pset;
  for (unsigned int i = 0; i < ptSet.size(); i++) {
    if (inlierFlag[i])
      pset.push_back(ptSet[i]);
  }
  _calcLinePara(pset, a, b, c, &res);
}

bool RanscanFit::_getSample(std::vector<int> set, std::vector<int>* sset) {
  int i[2];
  if (set.size() > 2) {
    do {
      for (int n = 0; n < 2; n++) {
        double x = rand() / (1.0f * RAND_MAX);  // NOLINT
        i[n] = static_cast<int>(x * (set.size() - 1));
      }
    } while (!(i[1] != i[0]));
    for (int n = 0; n < 2; n++) {
      sset->push_back(i[n]);
    }
  } else {
    return false;
  }
  return true;
}


bool RanscanFit::_verifyComposition(const std::vector<FitNum> pts) {
  FitNum pt1 = pts[0];
  FitNum pt2 = pts[1];
  if (abs(pt1.mx - pt2.mx) < 5 && abs(pt1.my - pt2.my) < 5)
    return false;

  return true;
}

void RanscanFit::_calcLinePara(std::vector<FitNum> pts, double* a,
  double* b, double* c, double* res) {
  *res = 0;
  cv::Vec4f line;
  std::vector<cv::Point2f> ptsF;
  for (unsigned int i = 0; i < pts.size(); i++) {
    cv::Point2f cv_point;
    cv_point.x = pts[i].mx;
    cv_point.y = pts[i].my;
    ptsF.push_back(cv_point);
  }

  // cv::fitLine(ptsF, line, CV_DIST_L2, 0, 1e-2, 1e-2);

  *a = static_cast<double>(line[1]);
  *b = static_cast<double>(-line[0]);
  *c = static_cast<double>(line[0] * line[3] - line[1] * line[2]);

  for (unsigned int i = 0; i < pts.size(); i++) {
    double resid_ = fabs(pts[i].mx * (*a) + pts[i].my * (*b) + (*c));
    *res += resid_;
  }
  *res /= pts.size();
}



}  // namespace LineFit
