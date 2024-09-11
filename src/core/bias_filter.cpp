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