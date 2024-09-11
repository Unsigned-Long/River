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

#ifndef RIVER_STATE_MANAGER_H
#define RIVER_STATE_MANAGER_H

#include <utility>
#include "core/data_manager.h"
#include "core/estimator.h"
#include "ctraj/factor/marginalization_factor.h"
#include "core/status.h"
#include "core/bias_filter.h"

namespace ns_river {
#define LOCK_STATES std::unique_lock<std::mutex> statesLock(StateManager::StatesMutex);

    class StateManager {
    public:
        using Ptr = std::shared_ptr<StateManager>;
        using SplineBundleType = ns_ctraj::SplineBundle<Configor::Prior::SplineOrder>;

        struct StatePack {
        public:
            double timestamp{};
            Sophus::SO3d SO3_CurToRef;
            Eigen::Vector3d LIN_VEL_CurToRefInRef;
            Eigen::Vector3d gravity;
            Eigen::Vector3d ba;
            Eigen::Vector3d bg;

            StatePack(double timestamp, const Sophus::SO3d &so3CurToRef, Eigen::Vector3d linVelCurToRefInRef,
                      Eigen::Vector3d gravity, Eigen::Vector3d ba, Eigen::Vector3d bg);

            StatePack();

            [[nodiscard]] Eigen::Vector3d LIN_VEL_CurToRefInCur() const;

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(
                        CEREAL_NVP(timestamp), CEREAL_NVP(SO3_CurToRef),
                        CEREAL_NVP(LIN_VEL_CurToRefInRef), CEREAL_NVP(gravity),
                        CEREAL_NVP(ba), CEREAL_NVP(bg)
                );
            }
        };

    private:
        DataManager::Ptr dataMagr;
        Configor::Ptr configor;

        // spline bundles that keep four b-splines, i.e., the rotation spline, velocity spline,
        // and bias splines of accelerator and gyroscope
        SplineBundleType::Ptr splines;
        // the gravity represented in the world frame
        std::shared_ptr<Eigen::Vector3d> gravity;
        // the biases of acceleration and gyroscope
        std::shared_ptr<Eigen::Vector3d> ba, bg;

        ns_ctraj::MarginalizationInfo::Ptr margInfo;

        BiasFilter::Ptr baFilter;
        BiasFilter::Ptr bgFilter;

        std::map<int, double *> lastKeepSo3KnotAdd;
        std::map<int, double *> lastKeepVelKnotAdd;

    public:
        // mutexes employed in multi-thread framework
        static std::mutex StatesMutex;

    public:
        StateManager(DataManager::Ptr dataMagr, Configor::Ptr configor);

        static Ptr Create(const DataManager::Ptr &dataMagr, const Configor::Ptr &configor);

        void Run();

        [[nodiscard]] std::optional<StatePack> GetStatePackSafely(double t) const;

        [[nodiscard]] std::vector<std::optional<StatePack>> GetStatePackSafely(const std::vector<double> &times) const;

        [[nodiscard]] const SplineBundleType::Ptr &GetSplines() const;

        [[nodiscard]] const BiasFilter::Ptr &GetBaFilter() const;

        [[nodiscard]] const BiasFilter::Ptr &GetBgFilter() const;

    protected:
        // --------------
        // initialization
        // --------------

        bool TryPerformInitialization();

        [[nodiscard]] inline SplineBundleType::Ptr CreateSplines(double sTime, double eTime) const;

        void InitializeSO3Spline(const std::list<IMUFrame::Ptr> &imuData);

        void InitializeGravity(const std::list<RadarTargetArray::Ptr> &radarTarAryVec,
                               const std::list<IMUFrame::Ptr> &imuData);

        Estimator::Ptr InitializeVelSpline(const std::list<RadarTargetArray::Ptr> &radarTarAryVec,
                                           const std::list<IMUFrame::Ptr> &imuData);

        void AlignInitializedStates();

        void MarginalizationInInit(const Estimator::Ptr &estimator);

        void InitializeBiasFilters(double sTime);

        void UpdateBiasFilters(const Estimator::Ptr &estimator, double eTime);

        // ---------
        // front end
        // ---------
        bool IncrementalOptimization(const RiverStatus::StatusPack &status);

        void PreOptimization(const std::list<IMUFrame::Ptr> &imuData);

        static auto ExtractRange(const std::list<IMUFrame::Ptr> &data, double st, double et) {
            auto sIter = std::find_if(data.begin(), data.end(), [st](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() > st;
            });
            auto eIter = std::find_if(data.rbegin(), data.rend(), [et](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() < et;
            }).base();
            return std::pair(sIter, eIter);
        }

        void ShowSplineStatus() const;

        // -----------------------------
        // small help template functions
        // -----------------------------

        template<class SplineType>
        void MarginalizeKnotsToSet(std::set<double *> &blocks, int oldSize, int newSize, SplineType &spline) {
            /**
             * example, order: 3, knot size: 9, knots from '0'-th to '8'-th
             * | start time for data piece                | end time for data piece
             * [  (O)   (O)   (O)   (O)   (O)   (O)   (O)   (O)   (O) ]
             *     |-----------------|                       | max time for order-4 spline
             *              | the knots which would be marginalized, i.e., from '0'-th to '(kSize - 2 * order)'-th
             *                                                       i.e., from '0'-th to '3'-th
             *  In this way, there would be 5 kept knots.
             */
            for (int i = std::max(0, oldSize - 2 * Configor::Prior::SplineOrder + 1);
                 i <= newSize - 2 * Configor::Prior::SplineOrder; ++i) {
                blocks.insert(spline.GetKnot(i).data());
            }
        }

        template<int Dime1, int Dime2>
        inline std::optional<Eigen::Matrix<double, Dime1, Dime2, Eigen::RowMajor>>
        ObtainVarMatFromEstimator(const std::pair<const double *, const double *> &par, const Estimator::Ptr &est) {
            // compute the covariance of a parameter pair based on the estimator
            ceres::Covariance covariance({});
            auto res = covariance.Compute({par}, est.get());
            if (res) {
                Eigen::Matrix<double, Dime1, Dime2, Eigen::RowMajor> cov;
                covariance.GetCovarianceBlock(par.first, par.second, cov.data());
                return cov;
            } else {
                return {};
            }
        }

        static void LinearExtendKnotTo(SplineBundleType::RdSplineType &spline, double t);

        static void LinearExtendKnotTo(SplineBundleType::So3SplineType &spline, double t);
    };
}


#endif //RIVER_STATE_MANAGER_H
