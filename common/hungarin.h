/***
 * @Author: shane
 * @Date: 2024-04-03 15:04:43
 * @LastEditTime: 2024-04-03 15:10:34
 * @FilePath: /lpm/src/hungarin.h
 * @Description: start all over again!!!
 * @
 * @Copyright (c) 2024 by ${Shane1911}, All Rights Reserved.
 */
#pragma once
#include <memory.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include "common_include.h"

using namespace std;
#define MAX_num 1e6

namespace lmap {
namespace common {

class Hungarin {
  // TODO: need to be modified.
 private:
  std::vector<std::vector<double>> weight_;
  int N_;
  int weight_n_ = 0;
  int max_match_ = 0;
  std::vector<double> lx_, ly_;
  std::vector<int> xy_, yx_;
  std::vector<bool> S_, T_;
  std::vector<double> salck_;
  std::vector<int> slackx_;
  std::vector<int> prev_;
  bool initial_paras_ = false;

  /***
   * @description:
   * @return {*}
   */
  void Augment();

  /***
   * @description:
   * @return {*}
   */
  void AddToTree(const int& x, const int& prevx);

  /***
   * @description:
   * @return {*}
   */
  void UpdateLabels();

 public:
  Hungarin();
  ~Hungarin();

  /***
   * @description:
   * @return {*}
   */
  void Init(const std::vector<std::vector<double>>& weight);

  /***
   * @description:
   * @return {*}
   */
  void Init(const Eigen::MatrixXd& weight);

  /***
   * @description:
   * @return {*}
   */
  void Slove(std::map<int, int>* assign_res);
};

}  // namespace common
}  // namespace lmap
