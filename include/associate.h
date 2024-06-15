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
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <vector>

#include "common_include.h"
#include "hungarin.h"
#include "io_data.h"
#include "kdtree.h"

#define DEG2RAD 0.01745329

using namespace std;
using namespace Kdtree;
using namespace lmap::common;
typedef std::shared_ptr<Kdtree::KdTree> lm_kdtree_ptr;

namespace lmap {
namespace src {
class Associate {
 private:
  // paras form the config.
  int demension_;
  bool use_consistency_;
  double min_match_ratio_;
  // map paras.
  size_t num_map_ = 0;
  std::vector<Laneinfo> map_laneinfo_;
  std::vector<lm_kdtree_ptr> lm_kdtrees_;

  // cur paras.
  size_t num_curlane_ = 0;
  std::vector<Laneinfo> cur_laneinfo_;
  std::vector<std::vector<double>> cur_knn_thd_;

  /***
   * @description:
   * @return {*}
   */
  void PointsToKdtreeNode(const std::vector<Eigen::Vector3d>& linepoints,
                          Kdtree::KdNodeVector* node);

  /***
   * @description:
   * @param {int&} i
   * @param {int&} j
   * @return {*}
   */
  bool GetDistMatchList(const int& i, const int& j, double* score,
                        double* ideal_value);

  /***
   * @description:
   * @return {*}
   */
  Eigen::MatrixXd GetConsistencyMatrix(const Eigen::MatrixXd& Mat_score);

 public:
  Associate();
  ~Associate();

  /***
   * @description:
   * @param {string&} config_file
   * @return {*}
   */
  void InitParas(const std::string& config_file);

  /***
   * @description:
   * @return {*}
   */
  void SetLandmark(const std::vector<Laneinfo>& map_laneinfo);

  /***
   * @description:
   * @return {*}
   */
  void SetDecction(const std::vector<Laneinfo>& curlaneinfo,
                   const std::vector<std::vector<double>>& knn_thd);

  /***
   * @description:
   * @return {*}
   */
  void Association(std::map<int, int>* match_res);
};

}  // namespace src
}  // namespace lmap
