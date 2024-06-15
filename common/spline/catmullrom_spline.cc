#include "catmullrom_spline.h"

namespace lmap {
namespace spline {
CatmullRomSpline::CatmullRomSpline() {}
// CatmullRomSpline::CatmullRomSpline(
//     const int& tau, const std::vector<Eigen::Vector3d>& ctrlpoints)
//     : tau_(tau), ctrl_points_(ctrlpoints){};
CatmullRomSpline::~CatmullRomSpline() {}

void CatmullRomSpline::Init(const double& tau,
                            const std::vector<Eigen::Vector3d>& ctrlpoints) {
  if (ctrlpoints.size() != 4) {
    return;
  }
  tau_ = tau;
  //
  ctrl_points_ = ctrlpoints;
  // ctrl_points_ = ctrl_points_.empty() ? ctrlpoints : ctrl_points_;
  ctrl_points_mat_.resize(4, 3);
  int i = 0;
  for (const auto& point : ctrlpoints) {
    ctrl_points_mat_.row(i) = point.transpose();
    i++;
  }
  UpdateMat(tau);
  initial_falg_ = true;
  return;
}

void CatmullRomSpline::GetPoints(const int& num, std::vector<double>* knots_u,
                                 std::vector<Eigen::Vector3d>* points) {
  if (!initial_falg_) {
    return;
  }
  Eigen::MatrixXd u_exetend;
  u_exetend.resize(num, 4);
  u_exetend.setZero();
  int index_u = 0;
  double knot = 0.0;
  for (index_u = 0; index_u < num; index_u++) {
    knots_u->push_back(knot);
    u_exetend.block<1, 4>(index_u, 0) << 1, knot, pow(knot, 2), pow(knot, 3);
    knot += 1 / (double(num) + 1e-6);
  }
  index_u = 0;
  auto mat_points = u_exetend * mat_para_ * ctrl_points_mat_;
  for (index_u = 0; index_u < num; ++index_u) {
    points->push_back(mat_points.row(index_u));
  }
  return;
}

void CatmullRomSpline::GetPoint(const double& u, Eigen::Vector4d* coeff,
                                Eigen::Vector3d* point) {
  Eigen::Vector4d vec_u(1, u, pow(u, 2), pow(u, 3));
  Eigen::Matrix<double, 1, 4> vec_coeff = vec_u.transpose() * mat_para_;
  *coeff = vec_coeff.transpose();
  *point = Eigen::Vector3d::Zero();
  int index_vec = 0;
  for (const auto& p : ctrl_points_) {
    *point += vec_coeff[index_vec++] * p;
  }
  return;
}

void CatmullRomSpline::GetDerviative(const double& u,
                                     Eigen::Vector3d* derivate) {
  Eigen::Vector4d vec_u(1, u, pow(u, 2), pow(u, 3));
  Eigen::Matrix<double, 1, 3> der_vec =
      vec_u.transpose() * mat_para_ * ctrl_points_mat_;
  *derivate = der_vec.transpose().normalized();
  return;
}

void CatmullRomSpline::UpdateMat(const double& tau) {
  // updating the mat of M
  mat_para_ << 0, 1, 0, 0, -tau, 0, tau, 0, 2.0 * tau, tau - 3.0,
      3.0 - 2.0 * tau, -tau, -tau, 2.0 - tau, tau - 2.0, tau;
  return;
}

}  // namespace spline
}  // namespace lmap
