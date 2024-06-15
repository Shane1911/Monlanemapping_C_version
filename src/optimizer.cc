/***
 * @Author: shane
 * @Date: 2024-04-15 14:35:24
 * @LastEditTime: 2024-04-15 14:38:23
 * @FilePath: /lpm/common/lanefeature.h
 * @Description: start all over again!!!
 * @
 * @Copyright (c) 2024 by ${Shane1911}, All Rights Reserved.
 */
#include "optimizer.h"

namespace lmap {
namespace src {

class MultiFactor
    : public NoiseModelFactor4<Vector3, Vector3, Vector3, Vector3> {
 private:
  Eigen::Vector3d land_maker_;
  double u_;

 public:
  // Provide access to Matrix& version of evaluateError:
  using NoiseModelFactor4<Vector3, Vector3, Vector3, Vector3>::evaluateError;
  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<MultiFactor> shared_ptr;

  MultiFactor(Key i, Key j, Key k, Key l, Eigen::Vector3d pose, double u,
              const SharedNoiseModel& model)
      : NoiseModelFactor4<Vector3, Vector3, Vector3, Vector3>(model, i, j, k,
                                                              l),
        land_maker_(pose),
        u_(u) {}
  ~MultiFactor() override {}
  // initial func
  Vector evaluateError(const Vector3& p1, const Vector3& p2, const Vector3& p3,
                       const Vector3& p4, OptionalMatrixType H1,
                       OptionalMatrixType H2, OptionalMatrixType H3,
                       OptionalMatrixType H4) const override {
    // get the error and the Matrix of Hession.
    std::vector<Eigen::Vector3d> ctrl_pts = {p1, p2, p3, p4};
    std::shared_ptr<lmap::spline::CatmullRomSpline> spline_ptr =
        std::make_shared<lmap::spline::CatmullRomSpline>();
    spline_ptr->Init(0.5, ctrl_pts);
    Eigen::Vector3d error_point;
    Eigen::Vector4d coeff;
    spline_ptr->GetPoint(u_, &coeff, &error_point);
    Eigen::Vector3d vec_err = error_point - land_maker_;
    if (H1) {
      (*H1) = coeff[0] * gtsam::Matrix3::Ones();
    }
    if (H2) {
      (*H2) = coeff[1] * gtsam::Matrix3::Ones();
    }
    if (H3) {
      (*H3) = coeff[2] * gtsam::Matrix3::Ones();
    }
    if (H4) {
      (*H4) = coeff[3] * gtsam::Matrix3::Ones();
    }
    Vector3 error(vec_err[0], vec_err[1], vec_err[2]);
    return error;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new MultiFactor(*this)));
  }
};

Optimizer::Optimizer() {}
Optimizer::~Optimizer() {}

bool Optimizer::Initial() {
  // for get the opter's paramenters
  return true;
}

bool Optimizer::InitialValue(const std::vector<OptNode>& all_opt_nodes) {
  if (all_opt_nodes.empty()) {
    LOG(INFO) << "ERROR: the input nodes is empty!!!";
    return false;
  }
  input_nodes_.clear();
  input_nodes_ = all_opt_nodes;
  int node_nums = 0, lane_index = 0;
  for (auto& ctrl_nodes : input_nodes_) {
    int ctrlpt_index = 0;
    for (auto& node : ctrl_nodes.ctrlnodes) {
      int laneid = node.lane_id;
      int ctrlptid = node.ctrl_points_index;
      if (grap_nodes_map_.find(std::make_pair(laneid, ctrlptid)) ==
          grap_nodes_map_.end()) {
        node.status++;
        initialEstimate_.insert(X(node_nums), node.point);
        node.key_index = node_nums;
        grap_nodes_map_[std::make_pair(laneid, ctrlptid)] = node_nums;
        auto& vec_ctrl_indexs = lanes_maps_ctrlptsindex_[laneid];
        vec_ctrl_indexs.push_back(ctrlptid);
        key_maps_ctrpt_index_[node_nums] =
            std::make_pair(lane_index, ctrlpt_index);
        node_nums++;
      } else {
        // TODO: not this point status
        int key_index = grap_nodes_map_[std::make_pair(laneid, ctrlptid)];
        std::pair node_index = key_maps_ctrpt_index_[key_index];
        input_nodes_[node_index.first].ctrlnodes[node_index.second].status++;
      }
      ctrlpt_index++;
    }
    lane_index++;
  }
  return !input_nodes_.empty();
}

bool Optimizer::Optimiztion(const std::vector<OptNode>& all_opt_nodes) {
  /*
  optimize the ctrlpts:
  1. set the optier initial value
  2. set the error edge
   */
  if (!InitialValue(all_opt_nodes)) {
    LOG(INFO) << "set the initial value failed!!!";
    return false;
  }
  LOG(INFO) << "set the initial value successed!!!";
  // TODO: need opt the key nodes in graph
  if (!GetErrorEdge()) {
    LOG(INFO) << "set the error edge falied!!!";
    return false;
  }

  if (!GetNeighborEdge()) {
    LOG(INFO) << "set the NeighborEdge falied!!!";
    return false;
  }
  LOG(INFO) << "complish the graph building";
  optimizer_ptr_ =
      std::make_shared<LevenbergMarquardtOptimizer>(graph_, initialEstimate_);
  opt_result_ = optimizer_ptr_->optimize();
  LOG(INFO) << "result's error: " << graph_.error(opt_result_) << "\n";
  return true;
}

bool Optimizer::GetErrorEdge() {
  // TODO: get the errot between points with the ctrlpts
  /*
  1. Error edge: noise and the cost-fuc
   */
  if (input_nodes_.empty()) {
    LOG(INFO) << " ERROR: the optnodes is empty!!!";
    return false;
  }
  for (const auto& ctrl_nodes : input_nodes_) {
    bool add_flag = true;
    auto erorr_edge_noise = noiseModel::Diagonal::Sigmas(
        Vector3(ctrl_nodes.noise_v, ctrl_nodes.noise_v, ctrl_nodes.noise_v));
    std::vector<int> node_key_index;
    // ...
  }
  return true;
}

bool Optimizer::GetNeighborEdge() {
  if (lanes_maps_ctrlptsindex_.empty() || key_maps_ctrpt_index_.empty()) {
    LOG(INFO) << "ERROR: input is empty!!!";
    return false;
  }
  for (const auto& [laneid, ctrlpts_indexs] : lanes_maps_ctrlptsindex_) {
    if (ctrlpts_indexs.size() < 2) {
      continue;
    }
    for (int i = 1; i < int(ctrlpts_indexs.size()); i++) {
      // ...
    }
  }
  return true;
}

}  // namespace src
}  // namespace lmap
