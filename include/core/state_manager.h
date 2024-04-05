// Copyright (c) 2023. Created on 10/20/23 6:55 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIVER_STATE_MANAGER_H
#define RIVER_STATE_MANAGER_H

#include <utility>
#include "core/data_manager.h"
#include "core/estimator.h"
#include "ctraj/factors/marginalization_factor.h"
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

            Eigen::Vector3d LIN_VEL_CurToRefInCur() const;

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
