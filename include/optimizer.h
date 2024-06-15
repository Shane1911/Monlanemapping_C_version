/***
 * @Author: shane
 * @Date: 2024-04-15 14:35:24
 * @LastEditTime: 2024-04-15 14:38:23
 * @FilePath: /lpm/common/lanefeature.h
 * @Description: start all over again!!!
 * @
 * @Copyright (c) 2024 by ${Shane1911}, All Rights Reserved.
 */
#pragma once
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "catmullrom_spline.h"
#include "common_include.h"
#include "io_data.h"

class CatmullRomSpline;

using namespace std;
using namespace gtsam;
using symbol_shorthand::X;
using namespace lmap::common;
using namespace lmap::spline;

namespace lmap {
namespace src {

struct PairHash {
  template <class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2>& p) const {
    auto h1 = std::hash<T1>{}(p.first);
    auto h2 = std::hash<T2>{}(p.second);
    return h1 ^ h2;
  }
};

class Optimizer {
 public:
  Optimizer();
  ~Optimizer();

  /***
   * @description:
   * @return {*}
   */
  bool Initial();

  /***
   * @description:
   * @return {*}
   */
  bool Optimiztion(const std::vector<OptNode>& all_opt_nodes);

  /***
   * @description:
   * @param {int&} index
   * @return {*}
   */
  inline Eigen::Vector3d GetpointByKeyindex(const int& index) {
    Eigen::Vector3d res;
    if (key_maps_ctrpt_index_.find(index) == key_maps_ctrpt_index_.end()) {
      return res;
    }
    res = opt_result_.at<gtsam::Vector3>(X(index));
    return res;
  }

  /***
   * @description:
   * @return {*}
   */
  inline std::unordered_map<int, std::pair<int, int>> GetResultsMaps() {
    return key_maps_ctrpt_index_;
  }

 private:
  /***
   * @description:
   * @return {*}
   */
  bool InitialValue(const std::vector<OptNode>& all_opt_nodes);

  /***
   * @description:
   * @return {*}
   */
  bool GetErrorEdge();

  /***
   * @description:
   * @return {*}
   */
  bool GetNeighborEdge();

  // optimizer paramenters
  NonlinearFactorGraph graph_;
  Values initialEstimate_;
  Values opt_result_;
  std::shared_ptr<LevenbergMarquardtOptimizer> optimizer_ptr_;
  std::vector<OptNode> input_nodes_;
  std::unordered_map<std::pair<int, int>, int, PairHash> grap_nodes_map_;
  std::unordered_map<int, std::vector<int>> lanes_maps_ctrlptsindex_;
  /*
  key_maps_ctrpt_index_
  This maps is using for key index -----> (lane_index, ctrlpts_index).

   */
  std::unordered_map<int, std::pair<int, int>> key_maps_ctrpt_index_;
};

}  // namespace src
}  // namespace lmap
