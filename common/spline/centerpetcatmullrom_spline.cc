#include "centerpetcatmullrom_spline.h"

namespace lmap {
namespace spline {

CenterpetcatmullromSpline::CenterpetcatmullromSpline() {}
CenterpetcatmullromSpline::~CenterpetcatmullromSpline() {}

void CenterpetcatmullromSpline::InitSet(
    const double &alpha, const std::vector<Eigen::Vector3d> &ctrlpoints) {
  ctrl_points_ = ctrlpoints;
  alpha_ = alpha;
  num_ctrlpoits_ = ctrlpoints.size();
  paramenters_init_flag_ = true;
  return;
}

void CenterpetcatmullromSpline::GetPoints(
    const int &n_points, std::vector<Eigen::Vector3d> *res_points) {
  if (!paramenters_init_flag_) {
    return;
  }
  for (auto ctrlpoint_ptr = ctrl_points_.begin();
       ctrlpoint_ptr != ctrl_points_.end() - 3; ctrlpoint_ptr++) {
    std::vector<Eigen::Vector3d> tmp_points;
    tmp_points.assign(ctrlpoint_ptr, ctrlpoint_ptr + 4);
    auto add_points = GetpointsPart(tmp_points, n_points);
    if (add_points.empty()) {
      continue;
    }
    res_points->insert(res_points->end(), add_points.begin(), add_points.end());
  }
  return;
}

std::vector<Eigen::Vector3d> CenterpetcatmullromSpline::GetpointsPart(
    const std::vector<Eigen::Vector3d> &fourctrlpoints, const int &n_points) {
  std::vector<Eigen::Vector3d> res;
  if (fourctrlpoints.size() != 4) {
    return res;
  }
  double t0 = 0.0;
  double t1 = GetTimeInsert(fourctrlpoints[0], fourctrlpoints[1], t0);
  double t2 = GetTimeInsert(fourctrlpoints[0], fourctrlpoints[1], t1);
  double t3 = GetTimeInsert(fourctrlpoints[0], fourctrlpoints[1], t2);
  for (double t = t1; t <= t2; t += (t2 - t1) / n_points) {
    auto a1 = (t1 - t) / (t1 - t0) * fourctrlpoints[0] +
              (t - t0) / (t1 - t0) * fourctrlpoints[1];
    auto a2 = (t2 - t) / (t2 - t1) * fourctrlpoints[1] +
              (t - t1) / (t2 - t1) * fourctrlpoints[2];
    auto a3 = (t3 - t) / (t3 - t2) * fourctrlpoints[2] +
              (t - t2) / (t3 - t2) * fourctrlpoints[3];
    auto b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2;
    auto b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3;
    res.push_back((t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2);
  }
  return res;
}

double CenterpetcatmullromSpline::GetTimeInsert(const Eigen::Vector3d &point_i,
                                                const Eigen::Vector3d &point_j,
                                                const double &ts) {
  return pow((point_i - point_j).norm(), alpha_) + ts;
}

}  // namespace spline
}  // namespace lmap
