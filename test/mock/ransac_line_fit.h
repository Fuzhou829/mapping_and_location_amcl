/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2023-01-10 10:02:47
 * @LastEditTime: 2023-06-29 07:26:24
 */
#pragma once
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>


#include "common/tool.h"

namespace LineFit {

struct FitNum {
  int id;
  bool is_in_line_flag;
  double mx;
  double my;
  double theta;
  FitNum() {}
  FitNum(int num_id, double x, double y, double t) {
    this->id = num_id;
    mx = x;
    my = y;
    theta = t;
  }
};

struct QRInfo {
  int id;
  double x;
  double y;
  double theta;
  int line_id;  // 从属于那一条线
  QRInfo() {
    line_id = -1;
  }
};


class RanscanFit {
 public:
  RanscanFit() {}
  ~RanscanFit() {}

  void SetData(const std::vector<FitNum>& points);
  void GetLineParam(double* k, double* b);

 private:
  void _fitLine(const std::vector<FitNum>& ptSet, double* a, double* b,
                double* c);
  bool _getSample(std::vector<int> set, std::vector<int>* sset);
  bool _verifyComposition(const std::vector<FitNum> pts);
  void _calcLinePara(std::vector<FitNum> pts, double* a,
                     double* b, double* c, double* res);
  double a_, b_, c_;
};


class MapFit {
 public:
  MapFit(const std::string& file_dir, const std::string& save_file);
  ~MapFit() {}
 private:
  void _loadMapFile(const std::string& file_dir);
  void _oneLineAveTheta(const Json::Value& map_json);
  void _ransacLineAndTheta(const Json::Value& map_json);
  void _deletqrinfo(const Json::Value& map_json);
  void _getFitQr(int min_id, int max_id, const Json::Value& map_json,
                 std::vector<FitNum>* fit_qr);

 private:
  std::string save_file_;
};





}  // namespace LineFit




