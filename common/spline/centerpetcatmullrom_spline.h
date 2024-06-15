#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <string>
#include <vector>
using namespace std;
using namespace Eigen;

namespace lmap {
namespace spline {
class CenterpetcatmullromSpline {
 public:
  CenterpetcatmullromSpline();
  ~CenterpetcatmullromSpline();

  void InitSet(const double &alpha,
               const std::vector<Eigen::Vector3d> &ctrlpoints);

  void GetPoints(const int &n_poitns, std::vector<Eigen::Vector3d> *res_points);

 private:
  std::vector<Eigen::Vector3d> GetpointsPart(
      const std::vector<Eigen::Vector3d> &fourctrlpoints, const int &n_points);

  double GetTimeInsert(const Eigen::Vector3d &point_i,
                       const Eigen::Vector3d &point_j, const double &ts);

  // parameenters
  double alpha_ = 0.5;
  std::vector<Eigen::Vector3d> ctrl_points_;
  int num_ctrlpoits_;
  bool paramenters_init_flag_ = false;
};

}  // namespace spline
}  // namespace lmap
