#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>

#include "catmullrom_spline.h"
#include "common_include.h"
#include "eigen_defs.h"
#include "io_data.h"
#include "kdtree.h"

using namespace lmap::spline;
using namespace std;
using namespace Kdtree;

typedef std::shared_ptr<Kdtree::KdTree> lm_kdtree_ptr;

namespace lmap {
namespace common {

class CtrlPoints {
 public:
  CtrlPoints();
  ~CtrlPoints();

  /***
   * @description:
   * @return {*}
   */
  bool UpdatedCtrlpts(const std::vector<Eigen::Vector3d>& lanepoints);

  // bool GenerateFeature(const std::vector<Eigen::Vector3d>& lanepoints);

  // bool UpdateFeature(const std::vector<Eigen::Vector3d>& pre_contralpts,
  //                    const std::vector<Eigen::Vector3d>& lanepoints);

  /***
   * @description:
   * @return {*}
   */
  bool QuerryPoints(const int& laneid, const Eigen::Vector3d& search_point,
                    double* u, std::vector<CtrNode>* ctrl_nodes);

  inline std::vector<Eigen::Vector3d> GetContralPoints() {
    return contral_points_;
  }

  /***
   * @description:
   * @return {*}
   */
  inline void UpdatePointByIndex(const int& index,
                                 const Eigen::Vector3d& point) {
    if (index >= contral_points_.size()) {
      return;
    }
    contral_points_[index] = point;
    return;
  }

 private:
  /***
   * @description:
   * @return {*}
   */
  bool FitLaneline(const std::vector<Eigen::Vector3d>& lanepoints);

  /***
   * @description:
   * @return {*}
   */
  void SelectContralPoints(const std::vector<Eigen::Vector3d>& lanepoints);

  /***
   * @description:
   * @return {*}
   */
  std::vector<Eigen::Vector3d> FilterLanepoints(
      const std::vector<Eigen::Vector3d>& lanepoints);

  /***
   * @description:
   * @return {*}
   */
  bool FindBoderPoint(const Eigen::Vector3d& initial_point,
                      std::vector<Eigen::Vector3d>* lanepoints,
                      Eigen::Vector3d* innder_point,
                      Eigen::Vector3d* outer_point);

  /***
   * @description:
   * @param {bool&} head_or_tile
   * @param {Vector3d*} boder_point
   * @return {*}
   */
  void AddContralPoint(const bool& head_or_tile, Eigen::Vector3d* boder_point);

  /***
   * @description:
   * @return {*}
   */
  bool GetQuery(const Eigen::Vector3d& start_point,
                const Eigen::Vector3d& end_point, Eigen::Vector3d* boder_point);

  /***
   * @description:
   * @param {Vector3d&} boder_point
   * @param {int&} index
   * @return {*}
   */
  void GetNextNode(const Eigen::Vector3d& boder_point, const int& index);

  /***
   * @description:
   * @return {*}
   */
  Eigen::Vector3d GetNeareastPointOncircle(const Eigen::Vector3d& boder_point,
                                           const Eigen::Vector3d& center_point);

  /***
   * @description:
   * @return {*}
   */
  bool GetParameterzation(const Eigen::Vector3d& query_point,
                          const KdNodeIndexs& idxs, double* error, double* u);

  /***
   * @description:
   * @return {*}
   */
  bool InitialCtrlPtsKdtree();

  /***
   * @description:
   * @return {*}
   */
  void PointsToKdtreeNode(const std::vector<Eigen::Vector3d>& ctrlpoints,
                          Kdtree::KdNodeVector* node);
  // paramenters of laneline fitting
  Eigen::VectorXd fity_coefficents_;  // coef: a0--->an
  Eigen::VectorXd fitz_coefficents_;
  Eigen::Matrix3d line_rot_;
  Eigen::Vector3d initial_point_;
  std::vector<Eigen::Vector3d> contral_points_;
  // TODO: get form yaml
  double ctrl_points_chrod_ = 3.0;
  // using the kdtree complish the points search for query
  bool initial_kdtree_flag_ = false;
  lm_kdtree_ptr ctrpts_kdtree_ = nullptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace common
}  // namespace lmap
