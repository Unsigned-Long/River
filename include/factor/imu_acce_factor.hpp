// Copyright (c) 2023. Created on 10/31/23 12:49 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIVER_IMU_ACCE_FACTOR_HPP
#define RIVER_IMU_ACCE_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace ns_river {
    template<int Order>
    struct IMUAcceFactor {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        IMUFrame::Ptr frame;

        double weight;
        double so3DtInv, velDtInv, baDtInv;

        std::pair<std::size_t, double> so3IU, velIU, baIU;

        std::size_t SO3_OFFSET, VEL_OFFSET, BA_OFFSET, GRAVITY_OFFSET;
    public:
        IMUAcceFactor(const SplineMetaType &so3Meta, const SplineMetaType &velMeta, const SplineMetaType &baMeta,
                      IMUFrame::Ptr imuFrame, double weight)
                : frame(std::move(imuFrame)), weight(weight), so3DtInv(1.0 / so3Meta.segments.front().dt),
                  velDtInv(1.0 / velMeta.segments.front().dt), baDtInv(1.0 / baMeta.segments.front().dt) {
            // compute knots indexes
            so3Meta.template ComputeSplineIndex(frame->GetTimestamp(), so3IU.first, so3IU.second);
            velMeta.template ComputeSplineIndex(frame->GetTimestamp(), velIU.first, velIU.second);
            baMeta.template ComputeSplineIndex(frame->GetTimestamp(), baIU.first, baIU.second);

            // compute knots offset in 'parBlocks'
            SO3_OFFSET = so3IU.first;
            VEL_OFFSET = so3Meta.NumParameters() + velIU.first;
            BA_OFFSET = so3Meta.NumParameters() + velMeta.NumParameters() + baIU.first;
            GRAVITY_OFFSET = so3Meta.NumParameters() + velMeta.NumParameters() + baMeta.NumParameters();
        }

        static auto Create(const SplineMetaType &so3Meta, const SplineMetaType &velMeta, const SplineMetaType &baMeta,
                           const IMUFrame::Ptr &frame, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<IMUAcceFactor>(
                    new IMUAcceFactor(so3Meta, velMeta, baMeta, frame, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(IMUAcceFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | VEL | ... | VEL | BA | ... | BA | GRAVITY ]
         */
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            // compute rotation from {current frame} to {reference frame}
            Sophus::SO3<T> SO3_CurToRef;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET, so3IU.second, so3DtInv, &SO3_CurToRef
            );

            // compute linear acceleration of {current frame} with
            // respect to {reference frame} expressed in {reference frame}
            // note that the first derivative of velocity spline is exactly the linear acceleration
            Eigen::Vector3<T> LIN_ACCE_CurToRefInRef;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 1>(
                    parBlocks + VEL_OFFSET, velIU.second, velDtInv, &LIN_ACCE_CurToRefInRef
            );

            // compute the bias of accelerator
            Eigen::Vector3<T> BA;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    parBlocks + BA_OFFSET, baIU.second, baDtInv, &BA
            );

            Eigen::Map<const Eigen::Vector3<T>> GRAVITY_IN_REF(parBlocks[GRAVITY_OFFSET]);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = SO3_CurToRef.inverse() * (LIN_ACCE_CurToRefInRef - GRAVITY_IN_REF) + BA
                        - frame->GetAcce().cast<T>();
            residuals = T(weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    template<int Order>
    struct IMUAcceFactorWithConstBias {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        IMUFrame::Ptr frame;

        double weight;
        double so3DtInv, velDtInv;

        std::pair<std::size_t, double> so3IU, velIU;

        std::size_t SO3_OFFSET, VEL_OFFSET, BA_OFFSET, GRAVITY_OFFSET;

    public:
        IMUAcceFactorWithConstBias(const SplineMetaType &so3Meta, const SplineMetaType &velMeta,
                                   IMUFrame::Ptr imuFrame, double weight)
                : frame(std::move(imuFrame)), weight(weight), so3DtInv(1.0 / so3Meta.segments.front().dt),
                  velDtInv(1.0 / velMeta.segments.front().dt) {
            // compute knots indexes
            so3Meta.template ComputeSplineIndex(frame->GetTimestamp(), so3IU.first, so3IU.second);
            velMeta.template ComputeSplineIndex(frame->GetTimestamp(), velIU.first, velIU.second);

            // compute knots offset in 'parBlocks'
            SO3_OFFSET = so3IU.first;
            VEL_OFFSET = so3Meta.NumParameters() + velIU.first;
            BA_OFFSET = so3Meta.NumParameters() + velMeta.NumParameters();
            GRAVITY_OFFSET = BA_OFFSET + 1;
        }

        static auto Create(const SplineMetaType &so3Meta, const SplineMetaType &velMeta, const IMUFrame::Ptr &frame,
                           double weight) {
            return new ceres::DynamicAutoDiffCostFunction<IMUAcceFactorWithConstBias>(
                    new IMUAcceFactorWithConstBias(so3Meta, velMeta, frame, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(IMUAcceFactorWithConstBias).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | VEL | ... | VEL | BA | GRAVITY ]
         */
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            // compute rotation from {current frame} to {reference frame}
            Sophus::SO3<T> SO3_CurToRef;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET, so3IU.second, so3DtInv, &SO3_CurToRef
            );

            // compute linear acceleration of {current frame} with
            // respect to {reference frame} expressed in {reference frame}
            // note that the first derivative of velocity spline is exactly the linear acceleration
            Eigen::Vector3<T> LIN_ACCE_CurToRefInRef;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 1>(
                    parBlocks + VEL_OFFSET, velIU.second, velDtInv, &LIN_ACCE_CurToRefInRef
            );

            Eigen::Map<const Eigen::Vector3<T>> BA(parBlocks[BA_OFFSET]);
            Eigen::Map<const Eigen::Vector3<T>> GRAVITY_IN_REF(parBlocks[GRAVITY_OFFSET]);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = SO3_CurToRef.inverse() * (LIN_ACCE_CurToRefInRef - GRAVITY_IN_REF) + BA
                        - frame->GetAcce().cast<T>();
            residuals = T(weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}

#endif //RIVER_IMU_ACCE_FACTOR_HPP
