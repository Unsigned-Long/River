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

#ifndef RIVER_RADAR_FACTOR_HPP
#define RIVER_RADAR_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "core/calib_param_manager.h"
#include "sensor/imu.h"

namespace ns_river {
    template<int Order>
    struct RadarFactor {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        const CalibParamManager &parMagr;
        RadarTarget::Ptr target;

        double weight;
        double so3DtInv, velDtInv;

        // compute knots indexes
        std::pair<std::size_t, double> so3IU, velIU;
        std::size_t SO3_OFFSET, VEL_OFFSET;

    public:
        RadarFactor(const CalibParamManager &calibParMagr, RadarTarget::Ptr radarTar, const SplineMetaType &so3Meta,
                    const SplineMetaType &velMeta, double weight)
                : parMagr(calibParMagr), target(std::move(radarTar)), weight(weight),
                  so3DtInv(1.0 / so3Meta.segments.front().dt), velDtInv(1.0 / velMeta.segments.front().dt) {
            // compute knots indexes
            so3Meta.template ComputeSplineIndex(target->GetTimestamp(), so3IU.first, so3IU.second);
            velMeta.template ComputeSplineIndex(target->GetTimestamp(), velIU.first, velIU.second);

            // compute knots offset in 'parBlocks'
            SO3_OFFSET = so3IU.first;
            VEL_OFFSET = so3Meta.NumParameters() + velIU.first;
        }

        static auto Create(const CalibParamManager &calibParMagr, const RadarTarget::Ptr &radarTar,
                           const SplineMetaType &so3Meta, const SplineMetaType &velMeta, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<RadarFactor>(
                    new RadarFactor(calibParMagr, radarTar, so3Meta, velMeta, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(RadarFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | VEL | ... | VEL ]
         */
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            // compute rotation from {current frame} to {reference frame}
            Sophus::SO3<T> SO3_CurToRef;
            // compute angular velocity of {current frame} with respect to {reference frame} expressed in {current frame}
            Sophus::SO3Tangent<T> ANG_VEL_CurToRefInCur;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET, so3IU.second, so3DtInv, &SO3_CurToRef, &ANG_VEL_CurToRefInCur
            );
            Eigen::Vector3<T> ANG_VEL_CurToRefInRef = SO3_CurToRef * ANG_VEL_CurToRefInCur;

            // compute linear velocity of {current frame} with respect to {reference frame} expressed in {reference frame}
            // note that the zero derivative of velocity spline is exactly the linear velocity
            Eigen::Vector3<T> LIN_VEL_CurToRefInRef;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    parBlocks + VEL_OFFSET, velIU.second, velDtInv, &LIN_VEL_CurToRefInRef
            );

            Eigen::Vector3<T> LIN_VEL_RtoRefInRef =
                    -Sophus::SO3<T>::hat(SO3_CurToRef * parMagr.POS_RinB) * ANG_VEL_CurToRefInRef +
                    LIN_VEL_CurToRefInRef;

            T v1 = -target->GetTargetXYZ().cast<T>().dot(
                    parMagr.SO3_RtoB.matrix().transpose() * SO3_CurToRef.matrix().transpose() * LIN_VEL_RtoRefInRef
            );

            T v2 = static_cast<T>(target->GetRadialVelocity());

            Eigen::Map<Eigen::Vector1<T>> residuals(sResiduals);
            residuals(0, 0) = T(weight) * (target->GetInvRange() * v1 - v2);

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}
#endif //RIVER_RADAR_FACTOR_HPP
