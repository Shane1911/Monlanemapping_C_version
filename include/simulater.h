/***
 * @Author: shane
 * @Date: 2024-04-22 10:56:48
 * @LastEditTime: 2024-04-22 10:58:51
 * @FilePath: /lpm/src/simulater.h
 * @Description: start all over again!!!
 * @
 * @Copyright (c) 2024 by ${Shane1911}, All Rights Reserved.
 */
#pragma once

#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "associate.h"
#include "common_include.h"
#include "config_gflags.h"
#include "file_flow.h"
#include "io_data.h"
#include "lanefeature.h"
#include "optimizer.h"

class Lanefeature;
class Associate;
class Optimizer;

using namespace std;
using namespace lmap::common;

namespace lmap {
namespace src {
/***
 * @description: Run this model(simulate in data flow)
 */
class Simulater {
 public:
  Simulater();
  ~Simulater();

  /***
   * @description:
   * @return {*}
   */
  bool DataBuffer();

  /***
   * @description:
   * @return {*}
   */
  void RunDataflow();

  /***
   * @description:
   * @return {*}
   */
  bool InitialProcess();

  /***
   * @description:
   * @return {*}
   */
  bool TransformToWorldFrame(const Pose& cur_pose,
                             std::vector<Eigen::Vector3d>* res);
  /***
   * @description:
   * @return {*}
   */
  bool LimitLanepointsInRange(std::vector<Eigen::Vector3d>* res);

  /***
   * @description:
   * @return {*}
   */
  inline std::vector<FrameMsg> GetRawData() { return all_frames_; }

 private:
  /***
   * @description:
   * @return {*}
   */
  void Process(const FrameMsg& msg);

  /***
   * @description:
   * @return {*}
   */
  bool AssociateLanes(const FrameMsg& msg);

  /***
   * @description:
   * @return {*}
   */
  bool OptimizeByGtsam(const std::map<int, int>& match_res);

  /***
   * @description:
   * @return {*}
   */
  bool UpdatingMapLanes();

  /***
   * @description:
   * @return {*}
   */
  std::vector<Laneinfo> GetMapLanesinfo();

  /***
   * @description:
   * @return {*}
   */
  std::vector<Laneinfo> GetCurLanesinfo(const FrameMsg& msg);

  /***
   * @description:
   * @param {Vector3d&} point
   * @return {*}
   */
  inline bool PoitInRange(const Eigen::Vector3d& point) {
    if (abs(point.y()) > 10.0 || point.x() < 3.0 || point.x() > 50.0) {
      return false;
    }
    return true;
  }

  /***
   * @description:
   * @return {*}
   */
  std::vector<double> CalculateKnnthd(const Laneinfo& curlaneinfo);

  /***
   * @description:
   * @return {*}
   */
  void AddLaneIntoMap(const Laneinfo& laneinfo);

  /***
   * @description:
   * @return {*}
   */
  std::string PrintLanepoints(const std::vector<Eigen::Vector3d>& lanepoints);

  /***
   * @description:
   * @return {*}
   */
  void CalculateNoise(const std::vector<Eigen::Vector3d>& lanepoints,
                      std::vector<double>* noise);

  /***
   * @description:
   * @return {*}
   */
  bool UpdateOptCtrlpts(const std::vector<OptNode>& optnodes);

  /***
   * @description:
   * @return {*}
   */
  bool SmoothOptCtrlpts();

 private:
  file_flow file_os_;
  std::vector<FrameMsg> all_frames_;
  std::map<int, int> match_res_;
  // save for map and cur_frames
  std::unordered_map<int, std::shared_ptr<Lanefeature>> lanes_map_buffer_;
  // the index in maplanes vector maps the lanes' id
  std::vector<double> line_std_;
  std::map<int, int> lm_indextoid_;
  std::vector<std::vector<double>> cur_frame_knn_thd_;

  std::vector<Laneinfo> cur_frame_lanes_;
  std::vector<Laneinfo> lanes_from_map_;
  Pose cur_pose_;
  std::shared_ptr<Associate> matcher_ptr_ = nullptr;
  std::shared_ptr<Optimizer> opter_ptr_ = nullptr;
  int lane_id_ = 0;
  // noise patramenters
  std::vector<double> line_range_area_;
  std::vector<double> noise_bound_;
};

}  // namespace src
}  // namespace lmap
