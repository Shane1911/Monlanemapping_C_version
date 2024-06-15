/***
 * @Author: shane
 * @Date: 2023-07-17 09:09:58
 * @LastEditTime: 2023-07-27 14:30:14
 * @FilePath: /evaluator/common/io_data.h
 * @Description: start all over again!!!
 * @
 * @Copyright (c) 2023 by ${Shane1911}, All Rights Reserved.
 */
#pragma once
#include <json/json.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>
using namespace std;

namespace lmap {
namespace common {
// define frame info
typedef struct {
  Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
} Pose;

typedef struct {
  int lane_id;
  int ctrl_points_index;
  int status = 0;
  int key_index;
  Eigen::Vector3d point;
} CtrNode;

typedef struct {
  double u;
  std::vector<CtrNode> ctrlnodes;
  Eigen::Vector3d pt_w;
  double noise_v;
} OptNode;

typedef struct {
  std::vector<Eigen::Vector3d> lane_points;
  int catergory;
  std::vector<double> visiblity;
  int trick_id;
} Laneinfo;

typedef struct {
  int frame_sequence;
  double timestamp;
  Pose T_pose_w;
  Pose Extrinsic;
  // the lanespoints of predict.
  std::vector<Laneinfo> lane_gt;
} FrameMsg;

inline void JsonToPose(const Json::Value& pose_info, Pose* pose_ptr) {
  std::vector<double> pose;
  if (pose_info.size() == 0) {
    return;
  }
  for (int i = 0; i < int(pose_info.size()); i++) {
    for_each(pose_info[i].begin(), pose_info[i].end(),
             [&pose](const Json::Value& value) {
               double va = value.asDouble();
               pose.push_back(va);
             });
  }
  auto pf = Eigen::Map<Eigen::MatrixXd>(pose.data(), 4, 4).transpose();
  pose_ptr->rot = pf.block<3, 3>(0, 0);
  pose_ptr->t = pf.block<3, 1>(0, 3);
  return;
}

inline void JsonToLane(const Json::Value& lane_info,
                       std::vector<Laneinfo>* lane_ptr) {
  for (auto single_lane : lane_info) {
    Laneinfo cur_lane;
    cur_lane.trick_id = single_lane["track_id"].asDouble();
    cur_lane.catergory = single_lane["category"].asDouble();
    std::for_each(single_lane["xyz"].begin(), single_lane["xyz"].end(),
                  [&cur_lane](Json::Value& value) {
                    Eigen::Vector3d points;
                    points << value[0].asDouble(), value[1].asDouble(),
                        value[2].asDouble();
                    cur_lane.lane_points.push_back(points);
                  });
    std::for_each(single_lane["visibility"].begin(),
                  single_lane["visibility"].end(),
                  [&cur_lane](Json::Value& value) {
                    cur_lane.visiblity.push_back(value.asDouble());
                  });
    lane_ptr->push_back(cur_lane);
  }
  return;
}

}  // namespace common
}  // namespace lmap
