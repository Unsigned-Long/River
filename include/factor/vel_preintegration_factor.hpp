// Copyright (c) 2023. Created on 12/5/23 1:16 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

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
