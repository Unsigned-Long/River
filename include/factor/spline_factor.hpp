// Copyright (c) 2023. Created on 11/23/23 10:49 AM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

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
