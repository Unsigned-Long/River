// Copyright (c) 2023. Created on 11/10/23 7:35 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

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
