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
