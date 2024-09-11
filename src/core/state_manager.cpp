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

#include "core/state_manager.h"

#include <utility>
#include "core/estimator.h"
#include "spdlog/stopwatch.h"
#include "view/view_util.h"

namespace ns_river {

    // -------------------
    // static member field
    // -------------------
    std::mutex StateManager::StatesMutex = {};

    StateManager::StateManager(DataManager::Ptr dataMagr, Configor::Ptr configor)
            : dataMagr(std::move(dataMagr)), configor(std::move(configor)), splines(nullptr),
              gravity(std::make_shared<Eigen::Vector3d>(0.0, 0.0, -this->configor->prior.GravityNorm)),
              ba(std::make_shared<Eigen::Vector3d>(0.0, 0.0, 0.0)),
              bg(std::make_shared<Eigen::Vector3d>(0.0, 0.0, 0.0)), margInfo(nullptr) {}

    StateManager::Ptr StateManager::Create(const DataManager::Ptr &dataMagr, const Configor::Ptr &configor) {
        return std::make_shared<StateManager>(dataMagr, configor);
    }

    void StateManager::Run() {
        spdlog::info("'StateManager::Run' has been booted, thread id: {}.", RIVER_TO_STR(std::this_thread::get_id()));

        ros::Rate rate(configor->preference.IncrementalOptRate);

        while (ros::ok()) {
            auto status = RiverStatus::GetStatusPackSafely();

            if (RiverStatus::IsWith(RiverStatus::StateManager::Status::ShouldQuit, status.StateMagr)) {
                spdlog::warn("'StateManager::Run' quits normally.");
                break;
            }

            if (!RiverStatus::IsWith(RiverStatus::StateManager::Status::HasInitialized, status.StateMagr)) {
                // if this system has not been initialized
                if (RiverStatus::IsWith(RiverStatus::DataManager::Status::RadarTarAryForInitIsReady, status.DataMagr)) {
                    // if this system is ready for initialization
                    {
                        // 'StateManager' has known this message
                        LOCK_RIVER_STATUS
                        RiverStatus::DataManager::CurStatus ^= RiverStatus::DataManager::Status::RadarTarAryForInitIsReady;
                    }
                    spdlog::stopwatch sw;
                    auto valid = TryPerformInitialization();
                    if (valid) { spdlog::info("total time elapsed in initialization: {} (s).", sw); }
                } else {
                    spdlog::info("waiting for radar target arrays to perform initialization...");
                }
            } else {
                spdlog::stopwatch sw;
                // perform incremental optimization
                auto valid = IncrementalOptimization(status);
                if (valid) {
                    spdlog::info("total time elapsed in current incremental optimization: {} (s).", sw);
                }
            }

            rate.sleep();
            ros::spinOnce();
        }
    }

    bool StateManager::TryPerformInitialization() {
        auto radarTarAryForInit = dataMagr->GetRadarTarAryForInitSafely();

        // get start and end time of radar target arrays clocked by the IMU
        double sTimeRadar = radarTarAryForInit.front()->GetTargets().front()->GetTimestamp();
        double eTimeRadar = radarTarAryForInit.back()->GetTargets().back()->GetTimestamp();

        auto imuDataSeq = dataMagr->ExtractIMUDataPieceSafely(sTimeRadar, eTimeRadar);

        // ----------------
        // static condition
        // ----------------
        auto [acceMean, acceVar] = DataManager::AcceMeanVar(imuDataSeq);
        bool stationary = std::abs(acceMean.norm() - configor->prior.GravityNorm) < 0.01
                          && acceVar.diagonal().norm() < 0.01;
        if (stationary) {
            spdlog::warn(
                    "Motion excitation is insufficient!!! Gravity-removed acceleration average: {:.6f}, variance: {:.6f}"
                    ", try to move fast!!!", std::abs(acceMean.norm() - configor->prior.GravityNorm),
                    acceVar.diagonal().norm()
            );
            return false;
        }

        double sTime = std::max(sTimeRadar, imuDataSeq.front()->GetTimestamp());
        double eTime = std::min(eTimeRadar, imuDataSeq.back()->GetTimestamp());

        spdlog::info(
                "'initialization': start time: {:.6f}, end time: {:.6f}, duration: {:.6f}",
                sTime, eTime, eTime - sTime
        );

        if (Configor::Preference::DEBUG_MODE) {
            spdlog::info(
                    "'radarTarAryForInit' size: {}, start time: {:.6f}, end time: {:.6f}, duration: {:.6f}",
                    radarTarAryForInit.size(), sTimeRadar, eTimeRadar, sTimeRadar - eTimeRadar
            );
            spdlog::info(
                    "involved IMU frame size: {}, start time: {:.6f}, end time: {:.6f}, duration: {:.6f}",
                    imuDataSeq.size(), imuDataSeq.front()->GetTimestamp(), imuDataSeq.back()->GetTimestamp(),
                    imuDataSeq.back()->GetTimestamp() - imuDataSeq.front()->GetTimestamp()
            );
        }

        // create splines
        splines = CreateSplines(sTime, eTime);
        if (Configor::Preference::DEBUG_MODE) { ShowSplineStatus(); }

        // init filters
        InitializeBiasFilters(sTime);

        // ---------------------
        // initialize so3 spline
        // ---------------------
        spdlog::stopwatch sw;
        InitializeSO3Spline(imuDataSeq);
        spdlog::info("initialized rotation spline time elapsed: {} (s).", sw);
        // ViewUtil::ShowSO3Spline(splines->GetSo3Spline(Configor::Preference::SO3Spline), 0.01, sTime, eTime);

        // -------------------------
        // initialize gravity vector
        // -------------------------
        spdlog::stopwatch sw2;
        // the minimum eigen value could represent a good or bad initialization (gravity observability)
        InitializeGravity(radarTarAryForInit, imuDataSeq);

        // observability condition is good, the gravity has been initialized
        spdlog::info(
                "initialized gravity vector: 'gx': {:.6f}, 'gy': {:.6f}, 'gz': {:.6f}",
                (*gravity)(0), (*gravity)(1), (*gravity)(2)
        );
        // ViewUtil::ShowSO3SplineWithGravity(
        //         splines->GetSo3Spline(Configor::Preference::SO3Spline), *gravity, 0.01, sTime, eTime
        // );
        // splines->Save(configor->dataStream.OutputPath + "/splines_so3_init.json");

        // -----------------------
        // recover velocity spline
        // -----------------------
        spdlog::stopwatch sw3;
        auto estimator = InitializeVelSpline(radarTarAryForInit, imuDataSeq);
        spdlog::info("initialize velocity spline time elapsed: {} (s)", sw3);


        spdlog::info(
                "refined gravity vector: 'gx': {:.6f}, 'gy': {:.6f}, 'gz': {:.6f}",
                (*gravity)(0), (*gravity)(1), (*gravity)(2)
        );

        // ------------------
        // update bias filter
        // ------------------
        UpdateBiasFilters(estimator, eTime);

        // -----------------------
        // align states to gravity
        // -----------------------
        AlignInitializedStates();

        spdlog::info(
                "aligned gravity vector: 'gx': {:.6f}, 'gy': {:.6f}, 'gz': {:.6f}",
                (*gravity)(0), (*gravity)(1), (*gravity)(2)
        );

        // ---------------------------------
        // marginalization in initialization
        // ---------------------------------
        spdlog::stopwatch sw4;
        // attention: we need perform marginalization after state alignment,
        // as the old states would be stored in the 'MarginalizationInfo'
        MarginalizationInInit(estimator);
        spdlog::info("marginalization in initialization time elapsed: {} (s)", sw4);

        // splines->Save(configor->dataStream.OutputPath + "/init_splines.json");
        // margInfo->Save(configor->dataStream.OutputPath + "/init_marg_info.json");
        // estimator->ShowKnotStatus();
        // ViewUtil::ShowSO3VelSplineWithGravity(
        //         splines->GetSo3Spline(Configor::Preference::SO3Spline),
        //         splines->GetRdSpline(Configor::Preference::VelSpline), *gravity, 0.01, sTime, eTime
        // );
        // ros::shutdown();

        LOCK_RIVER_STATUS
        RiverStatus::StateManager::CurStatus |= RiverStatus::StateManager::Status::HasInitialized;
        RiverStatus::StateManager::CurStatus |= RiverStatus::StateManager::Status::NewStateNeedToDraw;
        RiverStatus::StateManager::CurStatus |= RiverStatus::StateManager::Status::NewStateNeedToPublish;
        RiverStatus::StateManager::ValidStateEndTime = eTime;

        return true;
    }

    StateManager::SplineBundleType::Ptr StateManager::CreateSplines(double sTime, double eTime) const {
        // create four splines
        auto so3SplineInfo = ns_ctraj::SplineInfo(
                Configor::Preference::SO3Spline, ns_ctraj::SplineType::So3Spline,
                sTime, eTime, configor->prior.SO3SplineKnotDist
        );
        auto velSplineInfo = ns_ctraj::SplineInfo(
                Configor::Preference::VelSpline, ns_ctraj::SplineType::RdSpline,
                sTime, eTime, configor->prior.VelSplineKnotDist
        );
        return SplineBundleType::Create({so3SplineInfo, velSplineInfo});
    }

    void StateManager::InitializeSO3Spline(const std::list<IMUFrame::Ptr> &imuData) {
        // add gyro factors and fit so3 spline
        auto estimator = Estimator::Create(configor, splines, gravity, ba, bg);
        for (const auto &frame: imuData) {
            estimator->AddGyroMeasurementWithConstBias(frame, RiverOpt::OPT_SO3, configor->prior.GyroWeight);
        }

        // make this problem fun rank
        estimator->SetParameterBlockConstant(
                splines->GetSo3Spline(Configor::Preference::SO3Spline).KnotsFront().data()
        );

        auto sum = estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));

        if (Configor::Preference::DEBUG_MODE) {
            spdlog::info("here is the summary of 'InitializeSO3Spline':\n{}\n", sum.BriefReport());
        }
    }

    void StateManager::InitializeGravity(const std::list<RadarTargetArray::Ptr> &radarTarAryVec,
                                         const std::list<IMUFrame::Ptr> &imuData) {
        // obtain extrinsics
        const auto &SO3_RtoB = configor->dataStream.CalibParam.SO3_RtoB;
        const Eigen::Vector3d &POS_RinB = configor->dataStream.CalibParam.POS_RinB;

        // obtain the fitted so3 spline
        const auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);

        // timestamp, imu velocity computed from radar measurements
        std::vector<std::pair<double, Eigen::Vector3d>> LIN_VEL_BtoB0inB0_VEC;

        // compute the velocity of imu (with respect to the reference frame expressed in the reference frame)
        for (const auto &tarAry: radarTarAryVec) {
            double t = tarAry->GetTimestamp();

            if (t < so3Spline.MinTime() || t >= so3Spline.MaxTime()) { continue; }

            auto SO3_BtoB0 = so3Spline.Evaluate(t);
            Eigen::Vector3d ANG_VEL_BtoB0inB0 = SO3_BtoB0 * so3Spline.VelocityBody(t);

            Sophus::SO3d SO3_RtoB0 = SO3_BtoB0 * SO3_RtoB;
            Eigen::Vector3d LIN_VEL_RtoB0InB0 = tarAry->RadarVelocityFromStaticTargetArray(SO3_RtoB0);

            Eigen::Vector3d LIN_VEL_BtoB0inB0 =
                    LIN_VEL_RtoB0InB0 + Sophus::SO3d::hat(SO3_BtoB0 * POS_RinB) * ANG_VEL_BtoB0inB0;

            LIN_VEL_BtoB0inB0_VEC.emplace_back(t, LIN_VEL_BtoB0inB0);
        }

        auto estimator = Estimator::Create(configor, splines, gravity, ba, bg);
        for (int i = 0; i < static_cast<int>(LIN_VEL_BtoB0inB0_VEC.size()) - 1; ++i) {
            int j = i + 1;
            const auto &[ti, vi] = LIN_VEL_BtoB0inB0_VEC.at(i);
            const auto &[tj, vj] = LIN_VEL_BtoB0inB0_VEC.at(j);

            auto [sIter, eIter] = ExtractRange(imuData, ti, tj);
            std::vector<std::pair<double, Eigen::Vector3d>> velData;
            for (auto iter = sIter; iter != eIter; ++iter) {
                const auto &frame = *iter;
                double t = frame->GetTimestamp();
                velData.emplace_back(t, so3Spline.Evaluate(t) * frame->GetAcce());
            }
            Eigen::Vector3d velPIM = TrapIntegrationOnce(velData);
            estimator->AddVelPIMForGravityRecovery(tj - ti, vj - vi, velPIM, RiverOpt::OPT_GRAVITY, 1.0);
        }
        auto sum = estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));

        if (Configor::Preference::DEBUG_MODE) {
            spdlog::info("here is the summary of 'InitializeGravity':\n{}\n", sum.BriefReport());
        }

        // covariance check
        // ceres::Covariance::Options opt;
        // ceres::Covariance covariance(opt);
        // covariance.Compute({std::make_pair(gravity->data(), gravity->data())}, estimator.get());
        // Eigen::Matrix<double, 2, 2, Eigen::RowMajor> cov;
        // covariance.GetCovarianceBlockInTangentSpace(gravity->data(), gravity->data(), cov.data());
        // spdlog::error("sigma in gravity recovery: {:.6f}", cov.diagonal().norm());
        // return cov.diagonal().maxCoeff();

        // observability check
        // Eigen::MatrixXd HMat = estimator->GetHessianMatrix();
        // Eigen::JacobiSVD<Eigen::MatrixXd> svd(HMat, Eigen::ComputeFullU | Eigen::ComputeFullV);
        // spdlog::error(
        //         "max eigen value: {:.6f}, min eigen value: {:.6f}", svd.singularValues()(0), svd.singularValues()(1)
        // );
    }

    Estimator::Ptr StateManager::InitializeVelSpline(const std::list<RadarTargetArray::Ptr> &radarTarAryVec,
                                                     const std::list<IMUFrame::Ptr> &imuData) {
        // ----------------------------------------------------------------------------------------------
        // optimize velocity b-spline using velocity preintegration from linear acceleration measurements
        // ----------------------------------------------------------------------------------------------
        {
            auto estimator = Estimator::Create(configor, splines, gravity, ba, bg);
            auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
            auto iter1 = imuData.cbegin();
            for (auto iter2 = std::next(iter1); iter2 != imuData.cend(); ++iter2) {
                double t1 = (*iter1)->GetTimestamp(), t2 = (*iter2)->GetTimestamp();
                if (t2 - t1 < 0.01) { continue; }
                // velocity preintegration
                std::vector<std::pair<double, Eigen::Vector3d>> velData;
                for (auto iter = iter1; iter != std::next(iter2); ++iter) {
                    const auto &frame = *iter;
                    double t = frame->GetTimestamp();
                    velData.emplace_back(t, so3Spline.Evaluate(t) * frame->GetAcce());
                }
                estimator->AddVelPreintegrationConstraint(TrapIntegrationOnce(velData), t1, t2, RiverOpt::OPT_VEL, 1.0);
            }
            // maintain observability of the last few control points
            estimator->AddVelSplineTailConstraint(RiverOpt::OPT_VEL, 1000.0);

            // solving
            estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));
        }

        // add gyro factors and fit so3 spline
        auto estimator = Estimator::Create(configor, splines, gravity, ba, bg);

        // optimization option in initialization
        RiverOpt option = RiverOpt::OPT_SO3 | RiverOpt::OPT_VEL | RiverOpt::OPT_GRAVITY;

        // acce and gyro factors
        for (const auto &frame: imuData) {
            estimator->AddGyroMeasurementWithConstBias(frame, option, configor->prior.GyroWeight);

            estimator->AddAcceMeasurementWithConstBias(frame, option, configor->prior.AcceWeight);
        }

        // radar factors
        for (const auto &ary: radarTarAryVec) {
            for (const auto &tar: ary->GetTargets()) {
                estimator->AddRadarMeasurement(tar, option, configor->prior.RadarWeight);
            }
        }

        // add a tail constraint to handle the poor observability of the last knot
        auto velTailIdVec = estimator->AddVelSplineTailConstraint(option, 1000.0);
        auto so3TailIdVec = estimator->AddSo3SplineTailConstraint(option, 1000.0);

        // make this problem fun rank
        estimator->SetParameterBlockConstant(
                splines->GetSo3Spline(Configor::Preference::SO3Spline).KnotsFront().data()
        );

        auto sum = estimator->Solve(Estimator::DefaultSolverOptions(4, Configor::Preference::DEBUG_MODE, false));

        option |= RiverOpt::OPT_BG | RiverOpt::OPT_BA;
        sum = estimator->Solve(Estimator::DefaultSolverOptions(4, Configor::Preference::DEBUG_MODE, false));

        if (Configor::Preference::DEBUG_MODE) {
            spdlog::info("here is the summary of 'InitializeVelSpline':\n{}\n", sum.BriefReport());
        }

        for (const auto &id: velTailIdVec) { estimator->RemoveResidualBlock(id); }
        for (const auto &id: so3TailIdVec) { estimator->RemoveResidualBlock(id); }

        return estimator;
    }

    void StateManager::InitializeBiasFilters(double sTime) {
        // ShowSplineStatus();
        {
            // -----------------
            // acceleration bias
            // -----------------
            // the covariance of the initial state is set to 0.01 m/s^2
            BiasFilter::StatePack initState(
                    sTime, *ba, Eigen::Vector3d::Ones() * 0.0001 * 0.0001
            );
            baFilter = BiasFilter::Create(initState, configor->prior.AcceBiasRandomWalk);
            spdlog::info("initial state of ba filter: {}", RIVER_TO_STR(baFilter->GetCurState()));
        }

        {
            // --------------
            // gyroscope bias
            // --------------
            // the covariance of the initial state is set to 0.001 rad/s
            BiasFilter::StatePack initState(
                    sTime, *bg, Eigen::Vector3d::Ones() * 0.00001 * 0.00001
            );
            bgFilter = BiasFilter::Create(initState, configor->prior.GyroBiasRandomWalk);
            spdlog::info("initial state of bg filter: {}", RIVER_TO_STR(bgFilter->GetCurState()));
        }
    }

    void StateManager::AlignInitializedStates() {
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
        // current gravity, velocities, and rotations are expressed in the reference frame
        // align them to the world frame whose negative z axis is aligned with the gravity vector
        auto SO3_RefToW = ObtainAlignedWtoRef(so3Spline.Evaluate(so3Spline.MinTime()), *gravity).inverse();
        *gravity = SO3_RefToW * *gravity;
        for (int i = 0; i < static_cast<int>(so3Spline.GetKnots().size()); ++i) {
            so3Spline.GetKnot(i) = SO3_RefToW * so3Spline.GetKnot(i);
        }
        for (int i = 0; i < static_cast<int>(velSpline.GetKnots().size()); ++i) {
            velSpline.GetKnot(i) = SO3_RefToW * velSpline.GetKnot(i);
        }
    }

    void StateManager::MarginalizationInInit(const Estimator::Ptr &estimator) {
        // perform marginalization
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
        int so3KnotSize = static_cast<int>(so3Spline.GetKnots().size());
        int velKnotSize = static_cast<int>(velSpline.GetKnots().size());

        std::set<double *> margParBlocks{gravity->data()};
        /**
         * example, order: 3, knot size: 9, knots from '0'-th to '8'-th
         * | start time for data piece                | end time for data piece
         * [  (O)   (O)   (O)   (O)   (O)   (O)   (O)   (O)   (O) ]
         *     |-----------------|                       | max time for order-4 spline
         *              | the knots which would be marginalized, i.e., from '0'-th to '(kSize - 2 * order)'-th
         *                                                       i.e., from '0'-th to '3'-th
         *  In this way, there would be 5 kept knots.
         */
        for (int i = 0; i < so3KnotSize; ++i) {
            if (i <= so3KnotSize - 2 * Configor::Prior::SplineOrder) {
                margParBlocks.insert(so3Spline.GetKnot(i).data());
            } else {
                lastKeepSo3KnotAdd.insert({i, so3Spline.GetKnot(i).data()});
            }
        }
        for (int i = 0; i < velKnotSize; ++i) {
            if (i <= velKnotSize - 2 * Configor::Prior::SplineOrder) {
                margParBlocks.insert(velSpline.GetKnot(i).data());
            } else {
                lastKeepVelKnotAdd.insert({i, velSpline.GetKnot(i).data()});
            }
        }

        if (Configor::Preference::DEBUG_MODE) {
            spdlog::info("rotation knots: {}, velocity knots: {}", so3KnotSize, velKnotSize);
            spdlog::info(
                    "marginalize rotation knots from 0-th to {}-th", so3KnotSize - Configor::Prior::SplineOrder - 1
            );
            spdlog::info(
                    "marginalize velocity knots from 0-th to {}-th", velKnotSize - Configor::Prior::SplineOrder - 1
            );
        }

        margInfo = ns_ctraj::MarginalizationInfo::Create(estimator.get(), margParBlocks, {}, 2);
    }

    // --------------------------------
    //    top: initialization
    // --------------------------------
    // bottom: incremental optimization
    // --------------------------------

    void StateManager::PreOptimization(const std::list<IMUFrame::Ptr> &imuData) {
        auto estimator = Estimator::Create(configor, splines, gravity, ba, bg);

        // --------------------------
        // optimize rotation b-spline
        // --------------------------
        for (const auto &frame: imuData) {
            estimator->AddGyroMeasurementWithConstBias(frame, RiverOpt::OPT_SO3, configor->prior.GyroWeight);
        }
        // maintain observability of the last few control points
        estimator->AddSo3SplineTailConstraint(RiverOpt::OPT_SO3, 1000.0);

        // solving
        estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));

        ns_ctraj::MarginalizationFactor::AddToProblem(estimator.get(), margInfo, 1.0);

        // ----------------------------------------------------------------------------------------------
        // optimize velocity b-spline using velocity preintegration from linear acceleration measurements
        // ----------------------------------------------------------------------------------------------
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        auto iter1 = imuData.cbegin();
        for (auto iter2 = std::next(iter1); iter2 != imuData.cend(); ++iter2) {
            double t1 = (*iter1)->GetTimestamp(), t2 = (*iter2)->GetTimestamp();
            if (t2 - t1 < 0.01) { continue; }
            // velocity preintegration
            std::vector<std::pair<double, Eigen::Vector3d>> velData;
            for (auto iter = iter1; iter != std::next(iter2); ++iter) {
                const auto &frame = *iter;
                double t = frame->GetTimestamp();
                velData.emplace_back(t, so3Spline.Evaluate(t) * frame->GetAcce());
            }
            estimator->AddVelPreintegrationConstraint(TrapIntegrationOnce(velData), t1, t2, RiverOpt::OPT_VEL, 1.0);
        }
        // maintain observability of the last few control points
        estimator->AddVelSplineTailConstraint(RiverOpt::OPT_VEL, 1000.0);

        // solving
        estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));
    }

    bool StateManager::IncrementalOptimization(const RiverStatus::StatusPack &status) {
        // set start time as the end time of last optimization
        const double sTime = status.ValidStateEndTime, eTime = dataMagr->GetEndTimeSafely();

        if (eTime - sTime < 1.0 / configor->preference.IncrementalOptRate) { return false; }

        spdlog::info(
                "'incremental optimization': start time: {:.6f}, end time: {:.6f}, duration: {:.6f}",
                sTime, eTime, eTime - sTime
        );

        // ------------
        // extract data
        // ------------

        auto imuData = dataMagr->ExtractIMUDataPieceSafely(sTime, eTime);
        auto radarData = dataMagr->ExtractRadarDataPieceSafely(sTime, eTime);

        if (Configor::Preference::DEBUG_MODE) {
            spdlog::info(
                    "'imuData' size: {}, start time: {:.6f}, end time: {:.6f}, duration: {:.6f}",
                    imuData.size(), imuData.front()->GetTimestamp(), imuData.back()->GetTimestamp(),
                    imuData.back()->GetTimestamp() - imuData.front()->GetTimestamp()
            );
            spdlog::info(
                    "'radarData' size: {}, start time: {:.6f}, end time: {:.6f}, duration: {:.6f}",
                    radarData.size(), radarData.front()->GetTimestamp(), radarData.back()->GetTimestamp(),
                    radarData.back()->GetTimestamp() - radarData.front()->GetTimestamp()
            );
        }

        // --------------
        // quit condition
        // --------------
        const double dt = eTime - sTime;
        const double imuFreq = static_cast<double>(imuData.size()) / dt;
        const double radarFreq = static_cast<double>(radarData.size()) / dt;
        if (Configor::Preference::DEBUG_MODE) {
            spdlog::info("imu frequency: {:.6f} hz, radar frequency: {:.6f} hz.", imuFreq, radarFreq);
        }
        if (imuFreq < 50 || radarFreq < 10) {
            LOCK_RIVER_STATUS
            RiverStatus::StateManager::CurStatus |= RiverStatus::StateManager::Status::ShouldQuit;
            return false;
        }

        // ----------------
        // static condition
        // ----------------
        LOCK_STATES
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);

        auto [acceMean, acceVar] = DataManager::AcceMeanVar(imuData);
        double velocity = velSpline.Evaluate(status.ValidStateEndTime - 1E-3).norm();
        // spdlog::info(
        //         "Gravity-removed acceleration average: {:.6f}, variance: {:.6f},  velocity: {:.6f}",
        //         std::abs(acceMean.norm() - configor->prior.GravityNorm), acceVar.diagonal().norm(), velocity
        // );
        bool stationary = std::abs(acceMean.norm() - configor->prior.GravityNorm) < 0.01
                          && acceVar.diagonal().norm() < 0.001 && velocity < 0.1;
        if (stationary) {
            spdlog::warn(
                    "Motion excitation is insufficient!!! Gravity-removed acceleration average: {:.6f}, variance: {:.6f}"
                    ", try to move fast!!!", std::abs(acceMean.norm() - configor->prior.GravityNorm),
                    acceVar.diagonal().norm()
            );

            // todo: when the body is static, not just quit simply
            LOCK_RIVER_STATUS
            RiverStatus::StateManager::CurStatus ^= RiverStatus::StateManager::Status::HasInitialized;
            return false;
        }

        // --------------
        // extend splines
        // --------------
        int so3KnotOldSize = static_cast<int>(so3Spline.GetKnots().size());
        int velKnotOldSize = static_cast<int>(velSpline.GetKnots().size());

        if (Configor::Preference::DEBUG_MODE) { ShowSplineStatus(); }

        // ViewUtil::ShowSO3VelSplineWithGravity(
        //         so3Spline, velSpline, *gravity, 0.01, so3Spline.MinTime(), sTime
        // );

        // ------------------
        // flat extrapolation
        // ------------------
        // ns_ctraj::SplineBundle<Configor::Prior::SplineOrder>::ExtendKnotsTo(
        //         so3Spline, eTime + 1E-6, so3Spline.GetKnots().back()
        // );
        // ns_ctraj::SplineBundle<Configor::Prior::SplineOrder>::ExtendKnotsTo(
        //         velSpline, eTime + 1E-6, velSpline.GetKnots().back()
        // );

        // --------------------
        // linear extrapolation
        // --------------------
        LinearExtendKnotTo(so3Spline, eTime + 1E-6);
        LinearExtendKnotTo(velSpline, eTime + 1E-6);

        // obtain the updated address
        std::map<double *, double *> updatedLastKeepKnotAdd;
        for (const auto &[idx, add]: lastKeepSo3KnotAdd) {
            updatedLastKeepKnotAdd.insert({add, so3Spline.GetKnot(idx).data()});
        }
        for (const auto &[idx, add]: lastKeepVelKnotAdd) {
            updatedLastKeepKnotAdd.insert({add, velSpline.GetKnot(idx).data()});
        }
        margInfo->ShiftKeepParBlockAddress(updatedLastKeepKnotAdd);

        if (Configor::Preference::DEBUG_MODE) {
            int parShiftCount = 0;
            for (const auto &[oldAdd, newAdd]: updatedLastKeepKnotAdd) { if (oldAdd != newAdd) { ++parShiftCount; }}
            spdlog::error(
                    "size of 'updatedLastKeepKnotAdd': {}, count of shifted parameter blocks: {}.",
                    updatedLastKeepKnotAdd.size(), parShiftCount
            );
            // just for debug
            // if (parShiftCount != 0) { ros::shutdown(); }
            ShowSplineStatus();
        }

        // ViewUtil::ShowSO3VelSplineWithGravity(
        //         so3Spline, velSpline, *gravity, 0.01, so3Spline.MinTime(), eTime
        // );

        // ------------------------
        // incremental optimization
        // ------------------------

        PreOptimization(imuData);

        // add gyro factors and fit so3 spline
        auto estimator = Estimator::Create(configor, splines, gravity, ba, bg);

        // marginalization factor
        ns_ctraj::MarginalizationFactor::AddToProblem(estimator.get(), margInfo, 1.0);

        // optimization options for three kinds of factors
        RiverOpt option = RiverOpt::OPT_SO3 | RiverOpt::OPT_VEL | RiverOpt::OPT_BA | RiverOpt::OPT_BG;

        // acce and gyro factors
        for (const auto &frame: imuData) {
            estimator->AddGyroMeasurementWithConstBias(frame, option, configor->prior.GyroWeight);

            estimator->AddAcceMeasurementWithConstBias(frame, option, configor->prior.AcceWeight);
        }

        auto bgPrioriId = estimator->AddGyroBiasPriori(bgFilter->Prediction(eTime), option);

        auto baPrioriId = estimator->AddAcceBiasPriori(baFilter->Prediction(eTime), option);

        // radar factors
        for (const auto &tar: radarData) {
            estimator->AddRadarMeasurement(tar, option, configor->prior.RadarWeight);
        }

        // ---------------------------------------------------
        // handle the poor observability of the last few knots
        // ---------------------------------------------------
        auto velTailIdVec = estimator->AddVelSplineTailConstraint(option, 1000.0);
        auto so3TailIdVec = estimator->AddSo3SplineTailConstraint(option, 1000.0);

        auto sum = estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));

        if (Configor::Preference::DEBUG_MODE) {
            spdlog::info("here is the summary of 'IncrementalOptimization':\n{}\n", sum.BriefReport());
        }

        // ------------------
        // update bias filter
        // ------------------
        // estimator->ShowKnotStatus();
        UpdateBiasFilters(estimator, eTime);

        // -----------------------
        // perform marginalization
        // -----------------------

        int so3KnotSize = static_cast<int>(so3Spline.GetKnots().size());
        int velKnotSize = static_cast<int>(velSpline.GetKnots().size());
        std::set<double *> margParBlocks{gravity->data()};
        lastKeepSo3KnotAdd.clear(), lastKeepVelKnotAdd.clear();
        /**
         * example, order: 3
         *                 |-------> the knots which would be marginalized this time
         * [  (O)   (O)   (O)   (O)   (O) ... ]
         *     |-----| the knots which had been marginalized last time.
         */
        for (int i = std::max(0, so3KnotOldSize - 2 * Configor::Prior::SplineOrder + 1); i < so3KnotSize; ++i) {
            if (i <= so3KnotSize - 2 * Configor::Prior::SplineOrder) {
                margParBlocks.insert(so3Spline.GetKnot(i).data());
            } else {
                lastKeepSo3KnotAdd.insert({i, so3Spline.GetKnot(i).data()});
            }
        }
        for (int i = std::max(0, velKnotOldSize - 2 * Configor::Prior::SplineOrder + 1); i < velKnotSize; ++i) {
            if (i <= velKnotSize - 2 * Configor::Prior::SplineOrder) {
                margParBlocks.insert(velSpline.GetKnot(i).data());
            } else {
                lastKeepVelKnotAdd.insert({i, velSpline.GetKnot(i).data()});
            }
        }

        if (Configor::Preference::DEBUG_MODE) {
            spdlog::info("old rotation knots: {}, current rotation knots: {}", so3KnotOldSize, so3KnotSize);
            spdlog::info("old velocity knots: {}, current velocity knots: {}", velKnotOldSize, velKnotSize);
            spdlog::info(
                    "marginalize rotation knots from {}-th to {}-th",
                    so3KnotOldSize - 2 * Configor::Prior::SplineOrder + 1,
                    so3KnotSize - 2 * Configor::Prior::SplineOrder
            );
            spdlog::info(
                    "marginalize velocity knots from {}-th to {}-th",
                    velKnotOldSize - 2 * Configor::Prior::SplineOrder + 1,
                    velKnotSize - 2 * Configor::Prior::SplineOrder
            );
        }

        // remove bias prior factors before perform 'Marginalization', i.e., remove the prior info for bias
        // as they are treated as random walk process, not constants. In other words, their prior in the next optimization
        // is from the filter, not the marginalization factor
        estimator->RemoveResidualBlock(bgPrioriId);
        estimator->RemoveResidualBlock(baPrioriId);
        // remove linear extrapolation factor as they are not come from real-sensor measurements
        for (const auto &id: velTailIdVec) { estimator->RemoveResidualBlock(id); }
        for (const auto &id: so3TailIdVec) { estimator->RemoveResidualBlock(id); }

        margInfo = ns_ctraj::MarginalizationInfo::Create(estimator.get(), margParBlocks, {}, 1);

        // static int optCount = 0;
        // if (optCount == 0) {
        //     splines->Save(configor->dataStream.OutputPath + "/opt_splines.json");
        //     margInfo->Save(configor->dataStream.OutputPath + "/opt_marg_info.json");
        //     estimator->ShowKnotStatus();
        //     ViewUtil::ShowSO3VelSplineWithGravity(
        //             so3Spline, velSpline, *gravity, 0.01, so3Spline.MinTime(), eTime
        //     );
        //     ros::shutdown();
        // } else { ++optCount; }

        LOCK_RIVER_STATUS
        RiverStatus::StateManager::CurStatus |= RiverStatus::StateManager::Status::NewStateNeedToDraw;
        RiverStatus::StateManager::CurStatus |= RiverStatus::StateManager::Status::NewStateNeedToPublish;
        RiverStatus::StateManager::ValidStateEndTime = eTime;

        return true;
    }

    std::optional<StateManager::StatePack> StateManager::GetStatePackSafely(double t) const {
        LOCK_STATES
        if (splines == nullptr) { return {}; }

        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        if (!splines->TimeInRange(t, so3Spline)) { return {}; }

        auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
        if (!splines->TimeInRange(t, velSpline)) { return {}; }

        StatePack pack;
        pack.timestamp = t;
        pack.ba = *ba;
        pack.bg = *bg;
        pack.gravity = *gravity;
        pack.SO3_CurToRef = so3Spline.Evaluate(t);
        pack.LIN_VEL_CurToRefInRef = velSpline.Evaluate(t);

        return pack;
    }

    std::vector<std::optional<StateManager::StatePack>>
    StateManager::GetStatePackSafely(const std::vector<double> &times) const {
        std::vector<std::optional<StateManager::StatePack>> packs(times.size());
        LOCK_STATES
        if (splines == nullptr) { return packs; }
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);

        for (int i = 0; i < static_cast<int>(times.size()); ++i) {
            double t = times.at(i);
            if (!splines->TimeInRange(t, so3Spline) || !splines->TimeInRange(t, velSpline)) {
                packs.at(i) = {};
            } else {
                StatePack pack;
                pack.timestamp = t;
                pack.ba = *ba;
                pack.bg = *bg;
                pack.gravity = *gravity;
                pack.SO3_CurToRef = so3Spline.Evaluate(t);
                pack.LIN_VEL_CurToRefInRef = velSpline.Evaluate(t);

                packs.at(i) = pack;
            }
        }
        return packs;
    }

    const StateManager::SplineBundleType::Ptr &StateManager::GetSplines() const {
        return splines;
    }

    void StateManager::ShowSplineStatus() const {
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
        spdlog::info(
                "[Rotation Spline] min: '{:.6f}', max: '{:.6f}', dt: '{:.6f}', size: '{}'",
                so3Spline.MinTime(), so3Spline.MaxTime(), so3Spline.GetTimeInterval(), so3Spline.GetKnots().size()
        );
        spdlog::info(
                "[Velocity Spline] min: '{:.6f}', max: '{:.6f}', dt: '{:.6f}', size: '{}'",
                velSpline.MinTime(), velSpline.MaxTime(), velSpline.GetTimeInterval(), velSpline.GetKnots().size()
        );
    }

    void StateManager::UpdateBiasFilters(const Estimator::Ptr &estimator, double eTime) {
        auto baCovOpt = ObtainVarMatFromEstimator<3, 3>({ba->data(), ba->data()}, estimator);
        if (baCovOpt != std::nullopt) {
            Eigen::Vector3d baCov = baCovOpt->diagonal();
            baFilter->UpdateByEstimator(BiasFilter::StatePack(eTime, *ba, baCov));
            spdlog::info("current state of ba filter: {}", RIVER_TO_STR(baFilter->GetCurState()));
        }

        auto bgCovOpt = ObtainVarMatFromEstimator<3, 3>({bg->data(), bg->data()}, estimator);
        if (bgCovOpt != std::nullopt) {
            Eigen::Vector3d bgCov = bgCovOpt->diagonal();
            bgFilter->UpdateByEstimator(BiasFilter::StatePack(eTime, *bg, bgCov));
            spdlog::info("current state of bg filter: {}", RIVER_TO_STR(bgFilter->GetCurState()));
        }
    }

    const BiasFilter::Ptr &StateManager::GetBaFilter() const {
        return baFilter;
    }

    const BiasFilter::Ptr &StateManager::GetBgFilter() const {
        return bgFilter;
    }

    void StateManager::LinearExtendKnotTo(SplineBundleType::RdSplineType &spline, double t) {
        Eigen::Vector3d delta = spline.GetKnots().back() - spline.GetKnots().at(spline.GetKnots().size() - 2);
        while ((spline.GetKnots().size() < SplineBundleType::N) || (spline.MaxTime() < t)) {
            spline.KnotsPushBack(spline.GetKnots().back() + delta);
        }
    }

    void StateManager::LinearExtendKnotTo(SplineBundleType::So3SplineType &spline, double t) {
        Sophus::SO3d delta =
                spline.GetKnots().at(spline.GetKnots().size() - 2).inverse() * spline.GetKnots().back();
        while ((spline.GetKnots().size() < SplineBundleType::N) || (spline.MaxTime() < t)) {
            spline.KnotsPushBack(spline.GetKnots().back() * delta);
        }
    }

    StateManager::StatePack::StatePack(double timestamp, const Sophus::SO3d &so3CurToRef,
                                       Eigen::Vector3d linVelCurToRefInRef, Eigen::Vector3d gravity,
                                       Eigen::Vector3d ba, Eigen::Vector3d bg)
            : timestamp(timestamp), SO3_CurToRef(so3CurToRef), LIN_VEL_CurToRefInRef(std::move(linVelCurToRefInRef)),
              gravity(std::move(gravity)), ba(std::move(ba)), bg(std::move(bg)) {}

    Eigen::Vector3d StateManager::StatePack::LIN_VEL_CurToRefInCur() const {
        return SO3_CurToRef.inverse() * LIN_VEL_CurToRefInRef;
    }

    StateManager::StatePack::StatePack() = default;
}