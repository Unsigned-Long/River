// Copyright (c) 2023. Created on 11/10/23 6:35 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIVER_BIAS_FILTER_H
#define RIVER_BIAS_FILTER_H

#include <ostream>
#include "ctraj/core/spline_bundle.h"
#include <cereal/types/list.hpp>

namespace ns_river {
    class BiasFilter {
    public:
        using Ptr = std::shared_ptr<BiasFilter>;

        struct StatePack {
            double time{};
            Eigen::Vector3d state;
            Eigen::Matrix3d var;

            StatePack(double time, Eigen::Vector3d state, const Eigen::Vector3d &varMat);

            StatePack();

            friend std::ostream &operator<<(std::ostream &os, const StatePack &pack);

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(time), CEREAL_NVP(state), CEREAL_NVP(var));
            }
        };

    private:
        StatePack curState;
        const double sigma2;

        std::list<StatePack> stateRecords;

    public:
        BiasFilter(StatePack init, double randomWalk);

        static Ptr Create(const StatePack &init, double randomWalk);

        [[nodiscard]] StatePack Prediction(double t) const;

        [[nodiscard]] const StatePack &GetCurState() const;

        [[nodiscard]] const std::list<StatePack> &GetStateRecords() const;

        void Update(const StatePack &mes);

        void UpdateByEstimator(const StatePack &est);
    };
}


#endif //RIVER_BIAS_FILTER_H
