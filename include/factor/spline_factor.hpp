// River: A Tightly-Coupled Radar-Inertial Velocity Estimator Based on Continuous-Time Optimization
// Copyright 2024, the School of Geodesy and Geomatics (SGG), Wuhan University, China
// https://github.com/Unsigned-Long/River.git
//
// Author: Shuolong Chen (shlchen@whu.edu.cn)
// GitHub: https://github.com/Unsigned-Long
//  ORCID: 0000-0002-5283-9057
//
// Purpose: See .h/.hpp file.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * The names of its contributors can not be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef RIVER_SPLINE_FACTOR_HPP
#define RIVER_SPLINE_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace ns_river {
    template<int Order>
    struct RdSplineFactor {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        double timestamp;
        Eigen::Vector3d value;

        double weight;
        double velDtInv;

        std::pair<std::size_t, double> velIU;
        std::size_t VEL_OFFSET;

    public:
        RdSplineFactor(const SplineMetaType &velMeta, double timestamp, Eigen::Vector3d value, double weight)
                : timestamp(timestamp), value(std::move(value)), weight(weight),
                  velDtInv(1.0 / velMeta.segments.front().dt) {
            // compute knots indexes
            velMeta.template ComputeSplineIndex(timestamp, velIU.first, velIU.second);

            // compute knots offset in 'parBlocks'
            VEL_OFFSET = velIU.first;
        }

        static auto
        Create(const SplineMetaType &velMeta, double timestamp, const Eigen::Vector3d &value, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<RdSplineFactor>(
                    new RdSplineFactor(velMeta, timestamp, value, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(RdSplineFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ VEL | ... | VEL ]
         */
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            Eigen::Vector3<T> LIN_VEL_CurToRefInRef;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    parBlocks + VEL_OFFSET, velIU.second, velDtInv, &LIN_VEL_CurToRefInRef
            );

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = LIN_VEL_CurToRefInRef - value.cast<T>();
            residuals = T(weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    template<int Order>
    struct So3SplineFactor {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        double timestamp;
        Sophus::SO3d value;

        double weight;
        double so3DtInv;

        std::pair<std::size_t, double> so3IU;

        std::size_t SO3_OFFSET;

    public:
        So3SplineFactor(const SplineMetaType &so3Meta, double timestamp, const Sophus::SO3d &value, double weight)
                : timestamp(timestamp), value(value), weight(weight), so3DtInv(1.0 / so3Meta.segments.front().dt) {
            // compute knots indexes
            so3Meta.template ComputeSplineIndex(timestamp, so3IU.first, so3IU.second);

            // compute knots offset in 'parBlocks'
            SO3_OFFSET = so3IU.first;
        }

        static auto Create(const SplineMetaType &so3Meta, double timestamp, const Sophus::SO3d &value, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<So3SplineFactor>(
                    new So3SplineFactor(so3Meta, timestamp, value, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(So3SplineFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 ]
         */
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            // compute rotation from {current frame} to {reference frame}
            Sophus::SO3<T> SO3_CurToRef;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET, so3IU.second, so3DtInv, &SO3_CurToRef
            );

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = (SO3_CurToRef.inverse() * value).log();
            residuals = T(weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //RIVER_SPLINE_FACTOR_HPP
