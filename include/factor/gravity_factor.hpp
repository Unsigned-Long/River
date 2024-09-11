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

#ifndef RIVER_GRAVITY_FACTOR_HPP
#define RIVER_GRAVITY_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace ns_river {
    struct GravityFactor {
    private:
        const double dt;
        Eigen::Vector3d deltaVel;
        Eigen::Vector3d velPIM;
        double weight;

    public:
        GravityFactor(const double dt, Eigen::Vector3d deltaVel, Eigen::Vector3d velPim, double weight)
                : dt(dt), deltaVel(std::move(deltaVel)), velPIM(std::move(velPim)), weight(weight) {}

        static auto
        Create(const double dt, const Eigen::Vector3d &deltaVel, const Eigen::Vector3d &velPim, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<GravityFactor>(
                    new GravityFactor(dt, deltaVel, velPim, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(GravityFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ GRAVITY ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            Eigen::Map<const Eigen::Vector3<T>> gravity(sKnots[0]);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = deltaVel - gravity * dt - velPIM;
            residuals = T(weight) * residuals;

            return true;
        }
    };
}
#endif //RIVER_GRAVITY_FACTOR_HPP
