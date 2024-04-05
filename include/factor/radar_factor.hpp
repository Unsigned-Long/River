// Copyright (c) 2023. Created on 10/31/23 12:50 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

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
