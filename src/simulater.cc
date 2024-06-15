/***
 * @Author: shane
 * @Date: 2024-04-15 14:35:24
 * @LastEditTime: 2024-04-15 14:38:23
 * @FilePath: /lpm/common/lanefeature.h
 * @Description: start all over again!!!
 * @
 * @Copyright (c) 2024 by ${Shane1911}, All Rights Reserved.
 */
#include "simulater.h"

namespace {
template <typename T>
T Clip(const T& max_v, const T& min_v, const T& v) {
  T v_max = max(max_v, min_v);
  T v_min = min(max_v, min_v);
  return max(v_min, min(v_max, v));
}

}  // namespace

namespace lmap {
namespace src {

Simulater::Simulater() {}
Simulater::~Simulater() {}

bool Simulater::DataBuffer() {
  file_os_.Init(FLAGS_config_file_path);
  file_os_.LoadData();
  all_frames_ = file_os_.GetRawData();
  return !all_frames_.empty();
}

void Simulater::RunDataflow() {
  // checking the rawdata
  if (all_frames_.empty()) {
    LOG(INFO) << "ERROR: raw data is empty!!!";
    return;
  }
  // process the model
  for_each(all_frames_.begin(), all_frames_.end(),
           [this](const FrameMsg& cur_msg) { this->Process(cur_msg); });
  return;
}

bool Simulater::InitialProcess() {
  // pre initial the matcher
  matcher_ptr_ = std::make_shared<Associate>();
  matcher_ptr_->InitParas(FLAGS_config_file_path);
  YAML::Node config_para = YAML::LoadFile(FLAGS_config_file_path);
  line_std_ = config_para["associate"]["knn_std"].as<std::vector<double>>();
  // the noise paramenters from the yaml
  line_range_area_ = config_para["range_area"].as<std::vector<double>>();
  noise_bound_ = config_para["noise"].as<std::vector<double>>();
  return true;
}

void Simulater::Process(const FrameMsg& msg) {
  // process frames
  LOG(INFO) << "=========================== Frame index: " << msg.frame_sequence
            << "\n";
  // complish associate
  // std::map<int, int> match_res;
  match_res_.clear();
  if (!AssociateLanes(msg)) {
    LOG(INFO) << "ERROR: mathcer lanes failed!!!";
    return;
  }
  for (const auto [ld_index, lm_index] : match_res_) {
    LOG(INFO) << "match res: " << ld_index << " , " << lm_index;
  }
  // updating mal lanes
  if (!UpdatingMapLanes()) {
    LOG(INFO) << "ERROR: updating map lanes failed!!!";
    return;
  }
  // TODO(xc):optimize the ctrl-points
  /*
  optimize the cur-lanepoints and the ctrl-pts of map
  MUST change: smooth after optimization
   */
  if (!OptimizeByGtsam(match_res_)) {
    LOG(INFO) << "cannot optimize!!!";
  }
  SmoothOptCtrlpts();
  LOG(INFO) << "map size: " << lanes_map_buffer_.size();
  LOG(INFO) << "End Frame index==========";
  return;
}

bool Simulater::AssociateLanes(const FrameMsg& msg) {
  if (nullptr == matcher_ptr_) {
    LOG(INFO) << "ERROR: the matcher is not initial!!!";
    return false;
  }
  // get the info
  lanes_from_map_.clear();
  lanes_from_map_ = GetMapLanesinfo();
  LOG(INFO) << "lanes_from_map_.size: " << lanes_from_map_.size();
  cur_frame_lanes_.clear();
  cur_frame_lanes_ = GetCurLanesinfo(msg);
  LOG(INFO) << "cur_frame_lanes_.size: " << cur_frame_lanes_.size();
  if (lanes_from_map_.empty()) {
    lm_indextoid_.clear();
    LOG(INFO) << "lanes in map is empty";
    for (int i = 0; i < int(cur_frame_lanes_.size()); i++) {
      match_res_[i] = i;
      lm_indextoid_[i] = i;
    }
    return true;
  }
  matcher_ptr_->SetLandmark(lanes_from_map_);
  matcher_ptr_->SetDecction(cur_frame_lanes_, cur_frame_knn_thd_);
  std::map<int, int> res;
  matcher_ptr_->Association(&res);
  match_res_ = res;
  return true;
}

bool Simulater::OptimizeByGtsam(const std::map<int, int>& match_res) {
  if (lanes_map_buffer_.empty() || cur_frame_lanes_.empty()) {
    LOG(INFO) << "ERROR: no input node to be optimized!!!";
    return false;
  }
  // TODO:(need change here) selected the nodes form the map lanes ctrl-points
  opter_ptr_ = std::make_shared<Optimizer>();
  std::vector<OptNode> cur_frame_nodes;
  for (int i = 0; i < int(cur_frame_lanes_.size()); i++) {
    int lm_index = match_res.at(i);
    int lm_id = lm_indextoid_[lm_index];
    if (lanes_map_buffer_.find(lm_id) == lanes_map_buffer_.end()) {
      continue;
    }
    auto cur_ctr_kdtree = lanes_map_buffer_[lm_id]->GetCtrlptsPtr();
    std::vector<double> line_noise;
    CalculateNoise(cur_frame_lanes_[i].lane_points, &line_noise);
    int index_linepoints = 0;
    // ...
  }
  // set the initial value in gragp
  if (!opter_ptr_->Optimiztion(cur_frame_nodes)) {
    return false;
  }
  // updating the optimized resluts
  UpdateOptCtrlpts(cur_frame_nodes);
  return true;
}

bool Simulater::UpdateOptCtrlpts(const std::vector<OptNode>& optnodes) {
  // updating the optimized points
  if (optnodes.empty()) {
    return false;
  }
  auto key_index_maps = opter_ptr_->GetResultsMaps();
  for (const auto& [key_index, index_pair] : key_index_maps) {
    auto& node = optnodes[index_pair.first].ctrlnodes[index_pair.second];
    Eigen::Vector3d point = opter_ptr_->GetpointByKeyindex(key_index);
    auto ctrpts_ptr = lanes_map_buffer_[node.lane_id]->GetCtrlptsPtr();
    ctrpts_ptr->UpdatePointByIndex(node.ctrl_points_index, point);
  }
  return true;
}

bool Simulater::SmoothOptCtrlpts() {
  if (lanes_map_buffer_.empty()) {
    return false;
  }
  for (const auto& [index, lanesinfo_ptr] : lanes_map_buffer_) {
    std::vector<Eigen::Vector3d> smooth_points;
    lanesinfo_ptr->GetSmoothedlaneByCtrlpts(&smooth_points);
  }
  // TODO: get the cur_lanes_info
  for (int i = 0; i < cur_frame_lanes_.size(); i++) {
    int lm_index = match_res_.at(i);
    int lm_id = lm_indextoid_[lm_index];
    if (lanes_map_buffer_.find(lm_id) == lanes_map_buffer_.end()) {
      continue;
    }
    auto ctrl_pts = lanes_map_buffer_[lm_id]->GetCtrlpoints();
    auto raw_points = cur_frame_lanes_[i].lane_points;
    auto line_pts = lanes_map_buffer_[lm_id]->GetLaneInfo().lane_points;
  }
  return true;
}

// TODO(XC): must remove the part of smooth lanepoints.
bool Simulater::UpdatingMapLanes() {
  /*
  Using the result of matcher, updating the maplanes:
  1...
   */
  if (match_res_.empty()) {
    LOG(INFO) << "ERROR: the match result is empty!!!";
    return false;
  }
  LOG(INFO) << "UpdatingMapLanes";
  // updating the map lanes
  std::set<int> visited_id;
  for (int i = 0; i < int(cur_frame_lanes_.size()); i++) {
    if (match_res_.find(i) == match_res_.end()) {
      LOG(INFO) << "add new lane in map";
      AddLaneIntoMap(cur_frame_lanes_[i]);
      match_res_[i] = lane_id_ + 1;
      continue;
    }
    int lm_index = match_res_.at(i);
    int lm_id = lm_indextoid_[lm_index];
    visited_id.insert(lm_id);
    if (lanes_map_buffer_.find(lm_id) != lanes_map_buffer_.end()) {
      lanes_map_buffer_[lm_id]->UpdateLanepoints(cur_frame_lanes_[i]);
    } else {
      AddLaneIntoMap(cur_frame_lanes_[i]);
      match_res_[i] = lane_id_ + 1;
    }
  }
  // remove no-using maplanes
  for (auto iter = lanes_map_buffer_.begin();
       iter != lanes_map_buffer_.end();) {
    if ((iter->second)->GetLifespan() <= 1) {
      iter = lanes_map_buffer_.erase(iter);
    } else {
      ++iter;
    }
  }
  return true;
}

void Simulater::AddLaneIntoMap(const Laneinfo& laneinfo) {
  std::shared_ptr<Lanefeature> new_lanefeature =
      std::make_shared<Lanefeature>();
  new_lanefeature->Initial(FLAGS_config_file_path);
  if (!new_lanefeature->UpdateLanepoints(laneinfo)) {
    LOG(INFO) << "ERROR: updating the lanepoints failed!!!";
    return;
  }
  new_lanefeature->SetId(lane_id_);
  auto& add_feature = lanes_map_buffer_[lane_id_];
  add_feature = new_lanefeature;
  lane_id_++;
  LOG(INFO) << "AddLaneIntoMap: id-" << lane_id_ - 1;
  return;
}

std::vector<Laneinfo> Simulater::GetMapLanesinfo() {
  std::vector<Laneinfo> res;
  lm_indextoid_.clear();
  for (auto& [id, lane_feature_ptr] : lanes_map_buffer_) {
    Laneinfo tmp_laneinfo = lane_feature_ptr->GetLaneInfo();
    res.emplace_back(tmp_laneinfo);
    lm_indextoid_[res.size() - 1] = id;
  }
  return res;
}

std::vector<Laneinfo> Simulater::GetCurLanesinfo(const FrameMsg& msg) {
  /*
  process the framinfo, mainly processing lanepoints:
  1. limiting the range of lanepoints
  2. transform the points to word-frame
  3. calculate the knn-thd
   */
  std::vector<Laneinfo> res;
  std::vector<std::vector<double>> tmp_knnthd;
  cur_pose_ = msg.T_pose_w;
  for (auto laneinfo : msg.lane_gt) {
    // cut off lanepoints
    LimitLanepointsInRange(&(laneinfo.lane_points));
    // get the knn thd
    std::vector<double> lane_thd = CalculateKnnthd(laneinfo);
    tmp_knnthd.emplace_back(lane_thd);
    // transform to word-frames
    TransformToWorldFrame(cur_pose_, &(laneinfo.lane_points));
    res.push_back(laneinfo);
  }
  cur_frame_knn_thd_.swap(tmp_knnthd);
  return res;
}

void Simulater::CalculateNoise(const std::vector<Eigen::Vector3d>& lanepoints,
                               std::vector<double>* noise) {
  // TODO: get the paramenters form the yaml.
  double max_range = pow(line_range_area_[1], 2) + pow(line_range_area_[3], 2);
  for (const auto& point : lanepoints) {
    double ratio = sqrt((pow(point.x(), 2) + pow(point.y(), 2)) / max_range);
    ratio = Clip(noise_bound_[1], noise_bound_[0], ratio);
    double tmp_noise =
        noise_bound_[0] + ratio * (noise_bound_[1] - noise_bound_[0]);
    noise->emplace_back(tmp_noise);
  }
  return;
}

std::vector<double> Simulater::CalculateKnnthd(const Laneinfo& curlaneinfo) {
  std::vector<double> res_thd;
  for (auto point : curlaneinfo.lane_points) {
    double pt_range = point.norm();
    double upper_thd = 2 * pt_range * sin(line_std_[0] * DEG2RAD) +
                       2 * line_std_[1] + 2 * line_std_[2];
    upper_thd = upper_thd > 1.0 ? 1.0 : upper_thd;
    res_thd.push_back(upper_thd);
  }
  return res_thd;
}

bool Simulater::TransformToWorldFrame(const Pose& cur_pose,
                                      std::vector<Eigen::Vector3d>* res) {
  // the lanepoints from body-frames to wold-frames
  if (res->empty()) {
    return false;
  }
  std::vector<Eigen::Vector3d> new_points;
  for (auto point : (*res)) {
    Eigen::Vector3d transed_pose = cur_pose.rot * point + cur_pose.t;
    new_points.emplace_back(transed_pose);
  }
  res->swap(new_points);
  return true;
}

bool Simulater::LimitLanepointsInRange(std::vector<Eigen::Vector3d>* res) {
  if (res->empty()) {
    return false;
  }
  std::vector<Eigen::Vector3d> new_points;
  for (auto point : (*res)) {
    if (PoitInRange(point)) {
      new_points.push_back(point);
    }
  }
  res->swap(new_points);
  return true;
}

std::string Simulater::PrintLanepoints(
    const std::vector<Eigen::Vector3d>& lanepoints) {
  std::string ss;
  for (auto& point : lanepoints) {
    ss += std::to_string(point.transpose()[0]) + "," +
          std::to_string(point.transpose()[1]) + "," +
          std::to_string(point.transpose()[2]) + " ";
  }
  return ss;
}

}  // namespace src
}  // namespace lmap
