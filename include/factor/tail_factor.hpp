// Copyright (c) 2023. Created on 11/11/23 7:31 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIVER_TAIL_FACTOR_HPP
#define RIVER_TAIL_FACTOR_HPP

namespace ns_river {
    struct RdTailFactor {
    private:
        double weight;

    public:
        explicit RdTailFactor(double weight) : weight(weight) {}

        static auto Create(double weight) {
            return new ceres::DynamicAutoDiffCostFunction<RdTailFactor>(new RdTailFactor(weight));
        }

        static std::size_t TypeHashCode() {
            return typeid(RdTailFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ VEL | VEL | VEL ]
         */
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            Eigen::Map<const Eigen::Vector3<T>> firKnot(parBlocks[0]);
            Eigen::Map<const Eigen::Vector3<T>> sedKnot(parBlocks[1]);
            Eigen::Map<const Eigen::Vector3<T>> thdKnot(parBlocks[2]);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = weight * (firKnot - 2.0 * sedKnot + thdKnot);

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct So3TailFactor {
    private:
        double weight;

    public:
        explicit So3TailFactor(double weight) : weight(weight) {}

        static auto Create(double weight) {
            return new ceres::DynamicAutoDiffCostFunction<So3TailFactor>(new So3TailFactor(weight));
        }

        static std::size_t TypeHashCode() {
            return typeid(So3TailFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | SO3 | SO3 ]
         */
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            Eigen::Map<Sophus::SO3<T> const> const SO3_firToW(parBlocks[0]);
            Eigen::Map<Sophus::SO3<T> const> const SO3_sedToW(parBlocks[1]);
            Eigen::Map<Sophus::SO3<T> const> const SO3_thdToW(parBlocks[2]);

            Sophus::SO3<T> SO3_firToSed = SO3_sedToW.inverse() * SO3_firToW;
            Sophus::SO3<T> SO3_SedToThd = SO3_thdToW.inverse() * SO3_sedToW;

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = weight * (SO3_SedToThd.inverse() * SO3_firToSed).log();

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //RIVER_TAIL_FACTOR_HPP
