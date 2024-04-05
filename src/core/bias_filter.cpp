// Copyright (c) 2023. Created on 11/10/23 6:35 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#include "core/bias_filter.h"
#include <utility>

namespace ns_river {

    BiasFilter::StatePack::StatePack(double time, Eigen::Vector3d state, const Eigen::Vector3d &varVec)
            : time(time), state(std::move(state)) {
        this->var.setZero();
        this->var.diagonal() = varVec;
    }

    BiasFilter::StatePack::StatePack() = default;

    std::ostream &operator<<(std::ostream &os, const BiasFilter::StatePack &pack) {
        os << "time: " << pack.time
           << " state: " << pack.state.transpose()
           << " var: " << pack.var.diagonal().transpose();
        return os;
    }

    BiasFilter::BiasFilter(BiasFilter::StatePack init, double randomWalk)
            : curState(std::move(init)), sigma2(randomWalk * randomWalk) {
        stateRecords.push_back(curState);
    }

    BiasFilter::StatePack BiasFilter::Prediction(double t) const {
        StatePack predState;
        predState.time = t;

        // state propagation
        predState.state = curState.state;

        // covariance propagation
        predState.var = curState.var + (t - curState.time) * sigma2 * Eigen::Matrix3d::Identity();
        return predState;
    }

    const BiasFilter::StatePack &BiasFilter::GetCurState() const {
        return curState;
    }

    void BiasFilter::Update(const BiasFilter::StatePack &mes) {
        // prediction
        auto pred = Prediction(mes.time);

        // update
        Eigen::Matrix3d KMat = pred.var * (pred.var + mes.var).inverse();
        curState.time = mes.time;

        // state update
        curState.state = pred.state + KMat * (mes.state - pred.state);

        // covariance update
        Eigen::Matrix3d IKMat = (Eigen::Matrix3d::Identity() - KMat);
        curState.var = IKMat * pred.var * IKMat.transpose() + KMat * mes.var * KMat.transpose();

        stateRecords.push_back(curState);
    }

    BiasFilter::Ptr BiasFilter::Create(const BiasFilter::StatePack &init, double randomWalk) {
        return std::make_shared<BiasFilter>(init, randomWalk);
    }

    void BiasFilter::UpdateByEstimator(const BiasFilter::StatePack &est) {
        this->curState = est;
        stateRecords.push_back(curState);
    }

    const std::list<BiasFilter::StatePack> &BiasFilter::GetStateRecords() const {
        return stateRecords;
    }
}