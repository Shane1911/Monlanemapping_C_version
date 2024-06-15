/***
 * @Author: shane
 * @Date: 2024-04-15 14:35:24
 * @LastEditTime: 2024-04-15 14:38:23
 * @FilePath: /lpm/common/lanefeature.h
 * @Description: start all over again!!!
 * @
 * @Copyright (c) 2024 by ${Shane1911}, All Rights Reserved.
 */
#include "associate.h"

namespace lmap {
namespace src {

namespace {
double DistanceCalculate(const Kdtree::KdNodeVector& res_point,
                         const std::vector<double>& detect_point) {
  double res = 1e6;
  if (res_point.size() != 1) {
    return res;
  }
  res = sqrt(pow(detect_point[0] - res_point[0].point[0], 2) +
             pow(detect_point[1] - res_point[0].point[1], 2) +
             pow(detect_point[2] - res_point[0].point[2], 2));
  return res;
}

double PointToLine(const Eigen::Vector3d& line_a,
                   const Eigen::Vector3d& line_b1,
                   const Eigen::Vector3d& line_b2) {
  double k_b;
  if ((line_b1 - line_b2).norm() < 1e-6) {
    k_b = 1e6;
  } else {
    k_b = (line_b1[1] - line_b2[1]) / (line_b1[0] - line_b2[0] + 1e-6);
  }
  double tmp = (k_b * line_a[0] - line_a[1] + line_b1[1] - k_b * line_b1[1]);
  double res = tmp / sqrt(pow(k_b, 2) + 1);
  return res;
}

}  // namespace

Associate::Associate() {}
Associate::~Associate() {}

void Associate::InitParas(const std::string& config_file) {
  // get the paramenters form the yaml.
  YAML::Node config_para = YAML::LoadFile(config_file);
  min_match_ratio_ = config_para["associate"]["min_match_ratio"].as<double>();
  use_consistency_ = config_para["associate"]["use_consistency"].as<bool>();
  return;
}

void Associate::SetLandmark(const std::vector<Laneinfo>& maplaneinfo) {
  // initial the map paras.
  if (maplaneinfo.empty()) {
    return;
  }
  lm_kdtrees_.clear();
  num_map_ = maplaneinfo.size();
  map_laneinfo_ = maplaneinfo;
  for (auto single_lane : maplaneinfo) {
    lm_kdtree_ptr cur_kdtree;
    Kdtree::KdNodeVector nodes;
    PointsToKdtreeNode(single_lane.lane_points, &nodes);
    if (nodes.empty()) {
      LOG(INFO) << "In SetLandmark, node is empty!!!";
      continue;
    }
    cur_kdtree = std::make_shared<Kdtree::KdTree>(&nodes);
    lm_kdtrees_.push_back(cur_kdtree);
  }
  return;
}

void Associate::SetDecction(const std::vector<Laneinfo>& curlaneinfo,
                            const std::vector<std::vector<double>>& knn_thd) {
  // set the current laneinfo.
  if (curlaneinfo.empty()) {
    return;
  }
  num_curlane_ = curlaneinfo.size();
  cur_laneinfo_ = curlaneinfo;
  cur_knn_thd_ = knn_thd;
  return;
}

void Associate::Association(std::map<int, int>* match_res) {
  // checking the input data.
  if (num_map_ == 0 || num_curlane_ == 0) {
    return;
  }
  // get the associate matrix. at ij, the default value is 0.
  auto Mat_score = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>();
  Mat_score.resize(num_map_, num_curlane_);
  Mat_score.setZero();
  for (size_t j = 0; j < num_curlane_; j++) {
    for (size_t i = 0; i < num_map_; i++) {
      if (map_laneinfo_[i].catergory != cur_laneinfo_[j].catergory) {
        continue;
      }
      double score = 0.0;
      double ideal_value = 0.0;
      if (!GetDistMatchList(i, j, &score, &ideal_value)) {
        continue;
      }
      Mat_score(i, j) = score < ideal_value ? 1 / (score + 1e-6) : 0.0;
    }
  }
  // slove the Matrix of A
  auto Mat_C = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>();
  Mat_C.resize(num_map_, num_curlane_);
  Mat_C.setOnes();
  if (use_consistency_ && num_curlane_ > 2 && num_curlane_ > 2) {
    // get the consistancy.
    Mat_C = GetConsistencyMatrix(Mat_score);
  }
  Eigen::MatrixXd A = Mat_score.cwiseProduct(Mat_C);
  // // slove matrix
  // LOG(INFO) << "score mat: \n" << Mat_score;
  // LOG(INFO) << "A: \n" << A;
  Hungarin matcher;
  matcher.Init(A);
  std::map<int, int> solve_res;
  matcher.Slove(&solve_res);
  (*match_res) = solve_res;
  return;
}

Eigen::MatrixXd Associate::GetConsistencyMatrix(
    const Eigen::MatrixXd& Mat_score) {
  Eigen::MatrixXd Mat_consis =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>();
  Mat_consis.resize(num_map_, num_curlane_);
  Mat_consis.setZero();
  std::vector<std::pair<int, int>> index_Mat_score;
  for (size_t i = 0; i < num_map_; i++) {
    for (size_t j = 0; j < num_curlane_; j++) {
      if (Mat_score(i, j) > 0.0) {
        index_Mat_score.push_back(std::make_pair(i, j));
      }
    }
  }
  double weight = 1.0;
  // TODO(XC): the bug in index_Mat_score
  for (const auto& [i, j] : index_Mat_score) {
    for (const auto& [k, l] : index_Mat_score) {
      if (i == k && j == l) {
        Mat_consis(i, j) += weight;
      } else {
        // TODO: remove this part
        int size_j = int(0.5 * cur_laneinfo_[j].lane_points.size());
        int size_l = cur_laneinfo_[l].lane_points.size();
        int size_s = int(0.5 * map_laneinfo_[i].lane_points.size());
        int index_mid = size_j > size_s ? size_s : size_j;
        int index_head = size_l > int(map_laneinfo_[k].lane_points.size())
                             ? int(map_laneinfo_[k].lane_points.size())
                             : size_l;
        double dist_a =
            PointToLine(*(map_laneinfo_[i].lane_points.end() - index_mid),
                        *(map_laneinfo_[k].lane_points.end() - index_head),
                        *(map_laneinfo_[k].lane_points.end()));
        double dist_b =
            PointToLine(*(cur_laneinfo_[j].lane_points.begin() + size_j),
                        *(cur_laneinfo_[l].lane_points.begin()),
                        *(cur_laneinfo_[l].lane_points.end()));
        if (dist_a > 0 && dist_b > 0) {
          Mat_consis(i, j) += weight / (1 + abs(dist_a - dist_b));
        }
      }
    }
  }
  if (Mat_consis.maxCoeff() > 0.0) {
    Mat_consis = (1 / Mat_consis.maxCoeff()) * Mat_consis;
    Mat_consis = Mat_consis.array().square();
  } else {
    Mat_consis.setOnes();
  }

  return Mat_consis;
}

bool Associate::GetDistMatchList(const int& i, const int& j, double* score,
                                 double* ideal_value) {
  // j the index of detection (lane from the cur laneinfo)
  // i the index of lanemap info (lane from the lanemap)
  if (cur_laneinfo_.empty() || cur_knn_thd_.empty() || lm_kdtrees_.empty()) {
    return false;
  }
  std::vector<double> cur_thd = cur_knn_thd_[j];
  int point_index = 0;
  double mean_dist_match = 0.0;
  double mean_knn_thd = 0.0;
  int len_dist_match = 0;
  for (const auto& point : cur_laneinfo_[j].lane_points) {
    point_index++;
    // find the closest point in map_lane_points.
    Kdtree::KdNodeVector result;
    std::vector<double> detect_point({point[0], point[1], point[2]});
    lm_kdtrees_[i]->k_nearest_neighbors(detect_point, 1, &result);
    double dist_match = DistanceCalculate(result, detect_point);
    mean_knn_thd =
        (mean_knn_thd * (point_index - 1) + cur_knn_thd_[j][point_index - 1]) /
        point_index;
    if (dist_match > cur_knn_thd_[j][point_index - 1]) {
      continue;
    }
    len_dist_match++;
    mean_dist_match =
        (mean_dist_match * (len_dist_match - 1) + dist_match) / len_dist_match;
  }
  if (len_dist_match == 0) {
    return false;
  }
  *score = mean_dist_match * sqrt(point_index / (len_dist_match + 1e-6));
  *ideal_value = mean_knn_thd * sqrt(1 / (min_match_ratio_ + 1e-6));
  return true;
}

void Associate::PointsToKdtreeNode(
    const std::vector<Eigen::Vector3d>& linepoints,
    Kdtree::KdNodeVector* node) {
  // transform the vec3d to kdtree-node.
  int test_index = 8;
  for (auto point : linepoints) {
    std::vector<double> tmp_points(&point[0], point.data() + 3);
    node->push_back(Kdtree::KdNode(tmp_points, test_index));
  }
  return;
}

}  // namespace src
}  // namespace lmap
