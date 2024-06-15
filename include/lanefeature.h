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
#include <memory>
#include <string>
#include <vector>

#include "catmullrom_splinelists.h"
#include "common_include.h"
#include "ctrl_points.h"
#include "eigen_defs.h"
#include "io_data.h"

using namespace std;
using namespace lmap::spline;
using namespace lmap::common;

// class CatmullRomSplineList;
// class CtrlPoints;

namespace lmap {
namespace src {

class Lanefeature {
 public:
  Lanefeature();
  ~Lanefeature();

  /***
   * @description:
   * @return {*}
   */
  void Initial(const std::string& config_path);

  /***
   * @description:
   * @return {*}
   */
  bool UpdateLanepoints(const Laneinfo& cur_laneinfo);

  /***
   * @description:
   * @return {*}
   */
  inline void Visit() {
    life_span_ = life_span_ == 0 ? 20 : life_span_;
    return;
  }

  /***
   * @description:
   * @return {*}
   */
  inline void Notvisit() {
    life_span_--;
    return;
  }

  /***
   * @description:
   * @return {*}
   */
  inline int GetLifespan() { return life_span_; }

  /***
   * @description:
   * @return {*}
   */
  inline std::shared_ptr<CtrlPoints> GetCtrlptsPtr() { return ctrl_pts_ptr_; }

  /***
   * @description:
   * @return {*}
   */
  bool GetSmoothedlaneByCtrlpts(std::vector<Eigen::Vector3d>* smooth_points);

  /***
   * @description:
   * @return {*}
   */
  inline Laneinfo GetLaneInfo() { return lane_info_; }
  /***
   * @description:
   * @param {int&} id
   * @return {*}
   */
  inline void SetId(const int& id) { id_ = id; }

  /***
   * @description:
   * @return {*}
   */
  inline std::vector<Eigen::Vector3d> GetCtrlpoints() {
    return ctrl_pts_ptr_->GetContralPoints();
  }

 private:
  /***
   * @description:
   * @return {*}
   */
  void UpdateCtrlPoints();

  // m-datas
  int id_;
  Laneinfo lane_info_;
  // spline parenters
  double tau_ = 0.5;
  bool initial_flag_ = false;
  int life_span_ = 0;

  // initial from the yaml file
  double ctrl_points_chrod_;
  double downsample_;
  std::vector<double> limit_range_;
  // for generatrating ctrl_points
  std::shared_ptr<CtrlPoints> ctrl_pts_ptr_ = nullptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace src
}  // namespace lmap
