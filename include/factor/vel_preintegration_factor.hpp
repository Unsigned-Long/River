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

#ifndef RIVER_VEL_PREINTEGRATION_FACTOR_HPP
#define RIVER_VEL_PREINTEGRATION_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace ns_river {
    template<int Order>
    struct VelPreintegrationFactor {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        Eigen::Vector3d alpha;
        double dt;
        double weight;
        double velDtInv;

        std::pair<std::size_t, double> vel1IU, vel2IU;
        std::size_t VEL1_OFFSET, VEL2_OFFSET, GRAVITY_OFFSET;

    public:
        VelPreintegrationFactor(const SplineMetaType &velMeta, Eigen::Vector3d alpha,
                                double t1, double t2, double weight)
                : alpha(std::move(alpha)), dt(t2 - t1), weight(weight), velDtInv(1.0 / velMeta.segments.front().dt) {
            // compute knots indexes
            velMeta.template ComputeSplineIndex(t1, vel1IU.first, vel1IU.second);
            velMeta.template ComputeSplineIndex(t2, vel2IU.first, vel2IU.second);

            // compute knots offset in 'parBlocks'
            VEL1_OFFSET = vel1IU.first;
            VEL2_OFFSET = vel2IU.first;
            GRAVITY_OFFSET = velMeta.NumParameters();
        }

        static auto Create(const SplineMetaType &velMeta, const Eigen::Vector3d &alpha,
                           double t1, double t2, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<VelPreintegrationFactor>(
                    new VelPreintegrationFactor(velMeta, alpha, t1, t2, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(VelPreintegrationFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ VEL | ... | VEL | GRAVITY ]
         */
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            // compute linear acceleration of {current frame} with
            // respect to {reference frame} expressed in {reference frame}
            // note that the first derivative of velocity spline is exactly the linear acceleration
            Eigen::Vector3<T> LIN_VEL_B1ToRefInRef, LIN_VEL_B2ToRefInRef;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    parBlocks + VEL1_OFFSET, vel1IU.second, velDtInv, &LIN_VEL_B1ToRefInRef
            );
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    parBlocks + VEL2_OFFSET, vel2IU.second, velDtInv, &LIN_VEL_B2ToRefInRef
            );


            Eigen::Map<const Eigen::Vector3<T>> GRAVITY_IN_REF(parBlocks[GRAVITY_OFFSET]);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = LIN_VEL_B2ToRefInRef - LIN_VEL_B1ToRefInRef - alpha.cast<T>() - GRAVITY_IN_REF * dt;
            residuals = T(weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //RIVER_VEL_PREINTEGRATION_FACTOR_HPP
