#include "catmullrom_splinelists.h"

namespace lmap {
namespace spline {
CatmullRomSplineList::CatmullRomSplineList(
    const double& tau, const std::vector<Eigen::Vector3d>& ctrlpoints) {
  if (ctrlpoints.size() < 4) {
    ExtendPoints(ctrlpoints);
  }
  tau_ = tau;
  ctrl_points_ = ctrlpoints;
  catmullspline_ptr_ = std::make_shared<CatmullRomSpline>();
  UpdateMat(tau);
  initial_falg_ = true;
  return;
}

CatmullRomSplineList::~CatmullRomSplineList() {}

void CatmullRomSplineList::GetPointList(const int& num,
                                        std::vector<Eigen::Vector3d>* points) {
  if (!initial_falg_) {
    return;
  }
  // get the smooth points
  for (auto point_ptr = ctrl_points_.begin();
       point_ptr != ctrl_points_.end() - 3; point_ptr++) {
    std::vector<Eigen::Vector3d> tmp_points;
    tmp_points.insert(tmp_points.end(), point_ptr, point_ptr + 4);
    catmullspline_ptr_->Init(tau_, tmp_points);
    std::vector<double> knots_tmp;
    std::vector<Eigen::Vector3d> add_points_tmp;
    if (point_ptr != ctrl_points_.end() - 4) {
      catmullspline_ptr_->GetPoints(num, &knots_tmp, &add_points_tmp);
      points->insert(points->end(), add_points_tmp.begin(),
                     add_points_tmp.end() - 1);
    } else {
      catmullspline_ptr_->GetPoints(num, &knots_tmp, &add_points_tmp);
      points->insert(points->end(), add_points_tmp.begin(),
                     add_points_tmp.end());
    }
  }
  return;
}

bool CatmullRomSplineList::ExtendPoints(
    const std::vector<Eigen::Vector3d>& ctrlpoints) {
  if (ctrlpoints.size() == 3) {
    Eigen::Vector3d add_point = 2 * ctrlpoints[2] - ctrlpoints[1];
    ctrl_points_ = ctrlpoints;
    ctrl_points_.emplace_back(add_point);
  } else if (ctrlpoints.size() == 2) {
    Eigen::Vector3d add_point_back = 2 * ctrlpoints[1] - ctrlpoints[0];
    ctrl_points_ = ctrlpoints;
    ctrl_points_.insert(ctrl_points_.begin(),
                        2 * ctrlpoints[0] - ctrlpoints[1]);
    ctrl_points_.emplace_back(add_point_back);
  } else {
    return false;
  }
  return true;
}

void CatmullRomSplineList::UpdateMat(const double& tau) {
  // updating the mat of M
  mat_para_ << 0, 1, 0, 0, -tau, 0, tau, 0, 2.0 * tau, tau - 3.0,
      3.0 - 2.0 * tau, -tau, -tau, 2.0 - tau, tau - 2.0, tau;
  return;
}

}  // namespace spline
}  // namespace lmap
