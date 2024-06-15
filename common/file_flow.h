/***
 * @Author: shane
 * @Date: 2024-03-28 14:06:45
 * @LastEditTime: 2024-04-01 10:00:26
 * @FilePath: /lpm/common/file_flow.h
 * @Description: start all over again!!!
 * @
 * @Copyright (c) 2024 by ${Shane1911}, All Rights Reserved.
 */
#pragma once
#include <glog/logging.h>
#include <json/json.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "eigen_defs.h"
#include "io_data.h"
using namespace std;

namespace lmap {
namespace common {
class file_flow {
 private:
  /***
   * @description:
   * @return {*}
   */
  bool ReadFromJson();

  /***
   * @description:
   * @param {Value&} buffer_msg
   * @return {*}
   */
  bool ParsingMsg(const Json::Value& buffer_msg);

  inline void CloseDataflow() {
    if (flag_save_result_) {
      save_data_flow_.close();
      save_data_flow_ext_.close();
    }
  };

 public:
  file_flow();
  ~file_flow();

  void Init(const string& config_path);

  void LoadData();

  /***
   * @description:
   * @return {*}
   */
  void SaveDataToCsv(const EigenVector3dVec& pre_trajs);

  /***
   * @description:
   * @return {*}
   */
  void SaveDataToCsv(const std::vector<Eigen::Vector3d>& pre_trajs);

  /***
   * @description:
   * @return {*}
   */
  inline std::vector<FrameMsg> GetRawData() { return all_farmes_info_; }

  inline int GetDataSize() { return all_farmes_info_.size(); }

  string ori_path_ = "";
  std::vector<FrameMsg> all_farmes_info_;
  ofstream save_data_flow_;
  ofstream save_data_flow_ext_;
  bool flag_save_result_ = false;
};

}  // namespace common
}  // namespace lmap
