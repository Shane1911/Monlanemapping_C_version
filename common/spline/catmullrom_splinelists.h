#pragma once
#include <memory>
#include <string>

#include "catmullrom_spline.h"
using namespace std;

class CatmullRomSpline;

namespace lmap {
namespace spline {
class CatmullRomSplineList {
 public:
  CatmullRomSplineList(const double& tau,
                       const std::vector<Eigen::Vector3d>& ctrlpoints);
  ~CatmullRomSplineList();

  void UpdateMat(const double& tau);

  void GetPointList(const int& num, std::vector<Eigen::Vector3d>* points);

 private:
  bool ExtendPoints(const std::vector<Eigen::Vector3d>& ctrlpoints);

  // para
  double tau_;  // default value
  bool initial_falg_ = false;
  std::shared_ptr<CatmullRomSpline> catmullspline_ptr_ = nullptr;
  std::vector<Eigen::Vector3d> ctrl_points_;
  Eigen::MatrixXd ctrl_points_mat_;
  Eigen::Matrix4d mat_para_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace spline
}  // namespace lmap
