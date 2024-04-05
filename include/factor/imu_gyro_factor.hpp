// Copyright (c) 2023. Created on 10/31/23 12:49 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIVER_IMU_GYRO_FACTOR_HPP
#define RIVER_IMU_GYRO_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace ns_river {
    template<int Order>
    struct IMUGyroFactor {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        IMUFrame::Ptr frame;

        double weight;
        double so3DtInv, bgDtInv;

        // array offset
        std::pair<std::size_t, double> so3IU, bgIU;
        std::size_t SO3_OFFSET, BG_OFFSET;

    public:
        IMUGyroFactor(const SplineMetaType &so3Meta, const SplineMetaType &bgMeta, IMUFrame::Ptr imuFrame,
                      double weight)
                : frame(std::move(imuFrame)), weight(weight), so3DtInv(1.0 / so3Meta.segments.front().dt),
                  bgDtInv(1.0 / bgMeta.segments.front().dt) {
            // compute knots indexes
            so3Meta.template ComputeSplineIndex(frame->GetTimestamp(), so3IU.first, so3IU.second);
            bgMeta.template ComputeSplineIndex(frame->GetTimestamp(), bgIU.first, bgIU.second);

            // compute knots offset in 'parBlocks'
            SO3_OFFSET = so3IU.first;
            BG_OFFSET = so3Meta.NumParameters() + bgIU.first;
        }

        static auto
        Create(const SplineMetaType &so3Meta, const SplineMetaType &bgMeta, const IMUFrame::Ptr &frame, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<IMUGyroFactor>(
                    new IMUGyroFactor(so3Meta, bgMeta, frame, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(IMUGyroFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | BG | ... | BG ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            Sophus::SO3Tangent<T> ANG_VEL_CurToRefInCur;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    sKnots + SO3_OFFSET, so3IU.second, so3DtInv, nullptr, &ANG_VEL_CurToRefInCur
            );

            Eigen::Vector3<T> BG;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    sKnots + BG_OFFSET, bgIU.second, bgDtInv, &BG
            );

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = ANG_VEL_CurToRefInCur + BG - frame->GetGyro().cast<T>();
            residuals = T(weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    template<int Order>
    struct IMUGyroFactorWithConstBias {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        IMUFrame::Ptr frame;

        double weight;
        double so3DtInv;

        // array offset
        std::pair<std::size_t, double> so3IU;
        std::size_t SO3_OFFSET, BG_OFFSET;

    public:
        IMUGyroFactorWithConstBias(const SplineMetaType &so3Meta, IMUFrame::Ptr imuFrame, double weight)
                : frame(std::move(imuFrame)), weight(weight), so3DtInv(1.0 / so3Meta.segments.front().dt) {
            // compute knots indexes
            so3Meta.template ComputeSplineIndex(frame->GetTimestamp(), so3IU.first, so3IU.second);

            // compute knots offset in 'parBlocks'
            SO3_OFFSET = so3IU.first;
            BG_OFFSET = so3Meta.NumParameters();
        }

        static auto
        Create(const SplineMetaType &so3Meta, const IMUFrame::Ptr &frame, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<IMUGyroFactorWithConstBias>(
                    new IMUGyroFactorWithConstBias(so3Meta, frame, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(IMUGyroFactorWithConstBias).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | BG ]
         */
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            Sophus::SO3Tangent<T> ANG_VEL_CurToRefInCur;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET, so3IU.second, so3DtInv, nullptr, &ANG_VEL_CurToRefInCur
            );

            Eigen::Map<const Eigen::Vector3<T>> BG(parBlocks[BG_OFFSET]);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = ANG_VEL_CurToRefInCur + BG - frame->GetGyro().cast<T>();
            residuals = T(weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //RIVER_IMU_GYRO_FACTOR_HPP
