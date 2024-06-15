/***
 * @Author: shane
 * @Date: 2024-04-07 17:19:15
 * @LastEditTime: 2024-04-10 11:28:09
 * @FilePath: /lpm/common/spline/catmullrom_spline.h
 * @Description: start all over again!!!
 * @
 * @Copyright (c) 2024 by ${Shane1911}, All Rights Reserved.
 */
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
using namespace std;
using namespace Eigen;

namespace lmap {
namespace spline {
class CatmullRomSpline {
 public:
  CatmullRomSpline();
  // explicit CatmullRomSpline(const int& tau,
  //                           const std::vector<Eigen::Vector3d>& ctrlpoints);
  ~CatmullRomSpline();

  void Init(const double& tau, const std::vector<Eigen::Vector3d>& ctrlpoints);

  void UpdateMat(const double& tau);

  void GetPoints(const int& num, std::vector<double>* knots_u,
                 std::vector<Eigen::Vector3d>* points);

  void GetDerviative(const double& u, Eigen::Vector3d* derivate);

  void GetPoint(const double& u, Eigen::Vector4d* coeff,
                Eigen::Vector3d* point);

  inline Eigen::Matrix4d GetMat() const { return mat_para_; }

 private:
  // paras
  double tau_;  // default value
  bool initial_falg_ = false;
  std::vector<Eigen::Vector3d> ctrl_points_;
  Eigen::MatrixXd ctrl_points_mat_;
  Eigen::Matrix4d mat_para_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace spline
}  // namespace lmap
