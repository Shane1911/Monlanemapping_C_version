/***
 * @Author: shane
 * @Date: 2024-04-15 14:35:24
 * @LastEditTime: 2024-04-15 14:38:23
 * @FilePath: /lpm/common/lanefeature.h
 * @Description: start all over again!!!
 * @
 * @Copyright (c) 2024 by ${Shane1911}, All Rights Reserved.
 */
#include "lanefeature.h"

namespace lmap {
namespace src {
Lanefeature::Lanefeature() {}
Lanefeature::~Lanefeature() {}

void Lanefeature::Initial(const std::string& config_path) {
  // initial the yanl file.
  YAML::Node config_para = YAML::LoadFile(config_path);
  limit_range_ = config_para["range_area"].as<std::vector<double>>();
  ctrl_points_chrod_ = config_para["ctrl_points_chord"].as<double>();
  downsample_ = config_para["downsample"].as<double>();
  ctrl_pts_ptr_ = std::make_shared<CtrlPoints>();
  id_ = -1;
  initial_flag_ = true;
  return;
}

bool Lanefeature::UpdateLanepoints(const Laneinfo& cur_laneinfo) {
  /*
  Condition:
  1.cur_laneinfo has been limited.
  2.lanepoints has been transformed.
   */
  if (!initial_flag_ || cur_laneinfo.lane_points.empty()) {
    return false;
  }
  lane_info_ = cur_laneinfo;
  UpdateCtrlPoints();
  return !ctrl_pts_ptr_->GetContralPoints().empty();
}

bool Lanefeature::GetSmoothedlaneByCtrlpts(
    std::vector<Eigen::Vector3d>* smooth_points) {
  if (ctrl_pts_ptr_->GetContralPoints().size() < 2 || !initial_flag_) {
    LOG(INFO) << "cann't get smooth spline!!!";
    return false;
  }
  std::shared_ptr<CatmullRomSplineList> bspline_ptr_ =
      std::make_shared<CatmullRomSplineList>(tau_,
                                             ctrl_pts_ptr_->GetContralPoints());
  int num = int(ctrl_points_chrod_ / downsample_) + 1;
  std::vector<Eigen::Vector3d> spline_points;
  bspline_ptr_->GetPointList(num, &spline_points);
  if (spline_points.empty()) {
    return false;
  }
  // TODO:(XC) can modified
  lane_info_.lane_points = spline_points;
  *smooth_points = spline_points;
  return true;
}

void Lanefeature::UpdateCtrlPoints() {
  if (nullptr == ctrl_pts_ptr_ || lane_info_.lane_points.empty()) {
    return;
  }
  ctrl_pts_ptr_->UpdatedCtrlpts(lane_info_.lane_points);
  return;
}

}  // namespace src
}  // namespace lmap
