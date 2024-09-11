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

#ifndef RIVER_BIAS_FACTOR_HPP
#define RIVER_BIAS_FACTOR_HPP

namespace ns_river {

    struct BiasFactor {
    private:
        Eigen::Vector3d biasPriori;
        Eigen::Matrix3d weight;

    public:
        BiasFactor(Eigen::Vector3d biasPriori, const Eigen::Matrix3d &var) : biasPriori(std::move(biasPriori)) {
            weight = Eigen::LLT<Eigen::Matrix3d>(var.inverse()).matrixL().transpose();
        }

        static auto Create(const Eigen::Vector3d &biasPriori, const Eigen::Matrix3d &var) {
            return new ceres::DynamicAutoDiffCostFunction<BiasFactor>(new BiasFactor(biasPriori, var));
        }

        static std::size_t TypeHashCode() {
            return typeid(BiasFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ bias ]
         */
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            Eigen::Map<const Eigen::Vector3<T>> bias(parBlocks[0]);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = bias - biasPriori;
            residuals = weight * residuals;

            return true;
        }
    };
}

#endif //RIVER_BIAS_FACTOR_HPP
