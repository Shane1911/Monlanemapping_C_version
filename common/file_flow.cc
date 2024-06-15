/*
 * @Author: shane
 * @Date: 2023-08-11 11:37:35
 * @LastEditTime: 2024-04-22 13:38:49
 * @FilePath: /LMAP/common/file_flow.cc
 * @Description: start all over again!!!
 *
 * Copyright (c) 2023 by ${Shane1911}, All Rights Reserved.
 */
#include "file_flow.h"

#include <iomanip>
namespace lmap {
namespace common {

file_flow::file_flow() {}
file_flow::~file_flow() {}

void file_flow::Init(const string& config_path) {
  YAML::Node config_para = YAML::LoadFile(config_path);
  ori_path_ = config_para["raw_data_path"].as<string>();
  flag_save_result_ = config_para["save_pred_result"].as<bool>();
  string pre_traj_path = config_para["ctrl_points_path"].as<string>();
  string pre_traj_path_ext = config_para["lane_points_path"].as<string>();
  if (flag_save_result_) {
    save_data_flow_ext_.open(pre_traj_path_ext, ios::out | ios::trunc);
  }
  if (flag_save_result_) {
    save_data_flow_.open(pre_traj_path, ios::out | ios::trunc);
  }
}

void file_flow::LoadData() {
  // load the json data.
  if (ReadFromJson()) {
    LOG(INFO) << "Complish the data load!!!";
    return;
  }
  LOG(INFO) << "ERROR in data loading!!!";
  return;
}

void file_flow::SaveDataToCsv(const EigenVector3dVec& pre_trajs) {
  save_data_flow_ << std::fixed << setprecision(6);
  for (const auto& point : pre_trajs) {
    save_data_flow_ << point.x() << "," << point.y() << "," << point.z() << " ";
  }
  save_data_flow_ << endl;
}

void file_flow::SaveDataToCsv(const std::vector<Eigen::Vector3d>& pre_trajs) {
  save_data_flow_ext_ << std::fixed << setprecision(6);
  for (const auto& point : pre_trajs) {
    save_data_flow_ext_ << point.x() << "," << point.y() << "," << point.z()
                        << " ";
  }
  save_data_flow_ext_ << endl;
}

bool file_flow::ParsingMsg(const Json::Value& buffer_msg) {
  all_farmes_info_.clear();
  Json::Value::Members mem = buffer_msg.getMemberNames();
  for (int i = 1; i <= static_cast<int>(mem.size()); i++) {
    FrameMsg cuf_frame_info;
    auto cur_msg = buffer_msg[std::to_string(i)];
    cuf_frame_info.frame_sequence = i;
    cuf_frame_info.timestamp = cur_msg["timestamp"].asDouble();
    JsonToPose(cur_msg["gt_pose"], &cuf_frame_info.T_pose_w);
    JsonToPose(cur_msg["Extrinsic"], &cuf_frame_info.Extrinsic);
    JsonToLane(cur_msg["lanes_predict"], &cuf_frame_info.lane_gt);
    all_farmes_info_.push_back(cuf_frame_info);
  }
  return !all_farmes_info_.empty();
}

bool file_flow::ReadFromJson() {
  // get the data from the json file.
  Json::Reader reader;
  Json::Value msg_buffer;
  ifstream srcfile(ori_path_, ios::binary);
  if (!srcfile.is_open()) {
    LOG(INFO) << "Fail to open: " + ori_path_;
    return false;
  }
  if (reader.parse(srcfile, msg_buffer)) {
    return ParsingMsg(msg_buffer);
  }
  return false;
}

}  // namespace common
}  // namespace lmap
