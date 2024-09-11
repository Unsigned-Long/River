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

#include "core/river.h"
#include "cereal/types/utility.hpp"
#include "river/RiverState.h"

namespace ns_river {

    River::River(const Configor::Ptr &configor)
            : handler(new ros::NodeHandle), configor(configor),
              dataMagr(DataManager::Create(handler, configor)),
              stateMagr(StateManager::Create(dataMagr, configor)),
              viewer(Viewer::Create(configor, stateMagr)),
              stateMagrThread(std::make_shared<std::thread>(&StateManager::Run, stateMagr)),
              viewerThread(std::make_shared<std::thread>(&Viewer::RunViewer, viewer)) {
        /**
         * 1. Once the 'dataMagr' is created, two ros::Subscriber, i.e., 'imuSuber' and 'radarSuber' would
         *    start waiting receive ros message from imu and radar topics;
         * 2. The 'StateManager::Run' runs on another thread once 'stateMagrThread' is created;
         * 3. The 'Viewer::RunViewer' runs on another thread once 'viewerThread' is created;
         */
        statePublisher = this->handler->advertise<river::RiverState>(Configor::Preference::PublishTopic, 100);
    }

    River::Ptr River::Create(const Configor::Ptr &configor) {
        return std::make_shared<River>(configor);
    }

    void River::Run() {
        spdlog::info("'River::Run' has been booted, thread id: {}.", RIVER_TO_STR(std::this_thread::get_id()));

        // auto lastTimeShowDataStatus = ros::Time::now().toSec();
        ros::Rate rate(configor->preference.IncrementalOptRate);

        while (ros::ok()) {
            auto status = RiverStatus::GetStatusPackSafely();

            if (RiverStatus::IsWith(RiverStatus::StateManager::Status::ShouldQuit, status.StateMagr)) {
                spdlog::warn("'River::Run' quits normally.");
                break;
            }

            // ----------------------------------
            // print data status every one second
            // ----------------------------------

            // if (Configor::Preference::DEBUG_MODE && ros::Time::now().toSec() - lastTimeShowDataStatus > 1.0) {
            //     dataMagr->ShowDataStatus();
            //     lastTimeShowDataStatus = ros::Time::now().toSec();
            // }

            // -------------------
            // publish river state
            // -------------------
            PublishRiverState(status);

            // --------------
            // erase old data
            // --------------
            if (RiverStatus::IsWith(RiverStatus::StateManager::Status::HasInitialized, status.StateMagr)) {
                // if you want to evaluate the consistency by comparing inertial quantities, comment out this
                dataMagr->EraseOldDataPieceSafely(status.ValidStateEndTime);
            }

            rate.sleep();
            ros::spinOnce();
        }
        stateMagrThread->join();
        viewerThread->join();
    }

    void River::Save() {
        const std::string &tarDir = configor->dataStream.OutputPath + "/river_output";
        if (!std::filesystem::exists(tarDir)) {
            if (!std::filesystem::create_directories(tarDir)) {
                throw Status(Status::Flag::WARNING, fmt::format(
                        "the output path for data, i.e., '{}', dose not exist and create failed!", tarDir)
                );
            }
        } else {
            std::filesystem::remove_all(tarDir);
            std::filesystem::create_directories(tarDir);
        }

        // splines
        auto &splines = this->stateMagr->GetSplines();
        auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        const double epoch = configor->preference.OutputResultsWithTimeAligned ? 0.0 : *dataMagr->GetRiverTimeEpoch();
        const double velST = velSpline.MinTime(), velET = velSpline.MaxTime();
        const double so3ST = so3Spline.MinTime(), so3ET = so3Spline.MaxTime();

        velSpline.SetStartTime(velST + epoch);
        so3Spline.SetStartTime(so3ST + epoch);

        splines->Save(tarDir + "/splines.json");

        velSpline.SetStartTime(velST);
        so3Spline.SetStartTime(so3ST);

        // velocity
        const double st = std::max(velST, so3ST), et = std::min(velET, so3ET);

        std::vector<std::pair<double, Eigen::Vector3d>> velocityInW, velocityInB;
        std::vector<std::pair<double, Sophus::SO3d>> quatBtoW;
        for (double t = st; t < et;) {
            Eigen::Vector3d LIN_VEL_BtoWinW = velSpline.Evaluate(t);
            velocityInW.emplace_back(t + epoch, LIN_VEL_BtoWinW);

            auto SO3_CurToW = so3Spline.Evaluate(t);
            quatBtoW.emplace_back(t + epoch, SO3_CurToW);

            Eigen::Vector3d LIN_VEL_BtoWinB = SO3_CurToW.inverse() * LIN_VEL_BtoWinW;
            velocityInB.emplace_back(t + epoch, LIN_VEL_BtoWinB);

            t += 0.01;
        }

        {
            std::ofstream file(tarDir + "/velocity.json", std::ios::out);
            cereal::JSONOutputArchive ar(file);
            ar(
                    cereal::make_nvp("velocity_in_world", velocityInW),
                    cereal::make_nvp("velocity_in_body", velocityInB)
            );
        }

        // rotation
        {
            std::ofstream file(tarDir + "/rotation.json", std::ios::out);
            cereal::JSONOutputArchive ar(file);
            ar(cereal::make_nvp("quat_body_to_world", quatBtoW));
        }

        // acce and gyro bias
        {
            std::ofstream file(tarDir + "/bias.json", std::ios::out);
            cereal::JSONOutputArchive ar(file);

            auto acceRecords = stateMagr->GetBaFilter()->GetStateRecords();
            for (auto &item: acceRecords) { item.time += epoch; }

            auto gyroRecords = stateMagr->GetBgFilter()->GetStateRecords();
            for (auto &item: gyroRecords) { item.time += epoch; }

            ar(cereal::make_nvp("acce_bias", acceRecords), cereal::make_nvp("gyro_bias", gyroRecords));
        }

        // imu measurements
        {
            // std::list<ns_river::IMUFrame> rawMes, estMes, diff;
            // for (const auto &item: dataMagr->GetIMUDataSeq()) {
            //     if (!splines->TimeInRange(item->GetTimestamp(), velSpline)) { continue; }
            //     if (!splines->TimeInRange(item->GetTimestamp(), so3Spline)) { continue; }

            //     rawMes.push_back(*item);
            //     rawMes.back().SetTimestamp(rawMes.back().GetTimestamp() + epoch);

            //     double t = item->GetTimestamp();
            //     auto state = stateMagr->GetStatePackSafely(t);

            //     Eigen::Vector3d LIN_VEL_IN_B =
            //             so3Spline.Evaluate(t).inverse() * (velSpline.Evaluate<1>(t) - state->gravity) + state->ba;
            //     Eigen::Vector3d ANG_VEL_IN_B = so3Spline.VelocityBody(t) + state->bg;
            //     estMes.emplace_back(t + epoch, ANG_VEL_IN_B, LIN_VEL_IN_B);

            //     diff.emplace_back(
            //             t + epoch, rawMes.back().GetGyro() - estMes.back().GetGyro(),
            //             rawMes.back().GetAcce() - estMes.back().GetAcce()
            //     );
            // }

            // std::ofstream file(tarDir + "/inertial.json", std::ios::out);
            // cereal::JSONOutputArchive ar(file);
            // ar(cereal::make_nvp("raw_inertial", rawMes), cereal::make_nvp("est_inertial", estMes),
            //    cereal::make_nvp("inertial_diff", diff));
        }

        spdlog::info("outputs of 'River' have been saved to dir '{}'.", tarDir);
    }

    void River::PublishRiverState(const RiverStatus::StatusPack &status) {
        if (!RiverStatus::IsWith(RiverStatus::StateManager::Status::NewStateNeedToPublish, status.StateMagr)) {
            return;
        }

        std::optional<StateManager::StatePack> state = this->stateMagr->GetStatePackSafely(
                status.ValidStateEndTime - Configor::Preference::StatePublishDelay
        );
        if (state == std::nullopt) { return; }

        river::RiverState riverState;
        riverState.header.stamp = ros::Time(state->timestamp);
        riverState.header.frame_id = "world";

        riverState.velocity_norm = static_cast<float>(state->LIN_VEL_CurToRefInRef.norm());
        riverState.acce_bias_norm = static_cast<float>(state->ba.norm());
        riverState.gyro_bias_norm = static_cast<float>(state->bg.norm());

        riverState.velocity_in_world.x = state->LIN_VEL_CurToRefInRef(0);
        riverState.velocity_in_world.y = state->LIN_VEL_CurToRefInRef(1);
        riverState.velocity_in_world.z = state->LIN_VEL_CurToRefInRef(2);

        Eigen::Vector3d LIN_VEL_CurToRefInCur = state->LIN_VEL_CurToRefInCur();

        riverState.velocity_in_body.x = LIN_VEL_CurToRefInCur(0);
        riverState.velocity_in_body.y = LIN_VEL_CurToRefInCur(1);
        riverState.velocity_in_body.z = LIN_VEL_CurToRefInCur(2);

        riverState.quaternion.x = state->SO3_CurToRef.unit_quaternion().x();
        riverState.quaternion.y = state->SO3_CurToRef.unit_quaternion().y();
        riverState.quaternion.z = state->SO3_CurToRef.unit_quaternion().z();
        riverState.quaternion.w = state->SO3_CurToRef.unit_quaternion().w();

        statePublisher.publish(riverState);

        {
            LOCK_RIVER_STATUS
            RiverStatus::StateManager::CurStatus ^= RiverStatus::StateManager::Status::NewStateNeedToPublish;
        }
    }
}