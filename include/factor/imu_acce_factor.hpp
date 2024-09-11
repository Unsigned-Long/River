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
