// Copyright (c) 2023. Created on 10/21/23 8:45 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

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
