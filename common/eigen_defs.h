/***
 * @Author: shane
 * @Date: 2023-07-17 09:09:58
 * @LastEditTime: 2023-07-24 15:38:38
 * @FilePath: /evaluator/common/eigen_defs.h
 * @Description: start all over again!!!
 * @
 * @Copyright (c) 2023 by ${Shane1911}, All Rights Reserved.
 */
#pragma once

#include <functional>
#include <map>
#include <utility>
#include <vector>

#include "Eigen/Geometry"

namespace lmap {
namespace common {
template <class EigenType>
using EigenVector = std::vector<EigenType, Eigen::aligned_allocator<EigenType>>;

template <typename T, class EigenType>
using EigenMap =
    std::map<T, EigenType, std::less<T>,
             Eigen::aligned_allocator<std::pair<const T, EigenType>>>;

using EigenVector3dVec = EigenVector<Eigen::Vector3d>;
using EigenAffine3dVec = EigenVector<Eigen::Affine3d>;

}  // namespace common
}  // namespace lmap
