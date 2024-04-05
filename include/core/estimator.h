// Copyright (c) 2023. Created on 10/21/23 1:37 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIVER_ESTIMATOR_H
#define RIVER_ESTIMATOR_H

#include "ctraj/core/spline_bundle.h"
#include "config/configor.h"
#include "ctraj/core/trajectory_estimator.h"
#include "sensor/imu.h"
#include "sensor/radar.h"
#include "core/bias_filter.h"

namespace ns_river {
    using namespace magic_enum::bitwise_operators;

    struct RiverOptOption {
        enum class Option : std::uint32_t {
            /**
             * @brief options
             */
            NONE = 1 << 0,
            OPT_SO3 = 1 << 1,
            OPT_VEL = 1 << 2,
            OPT_BA = 1 << 3,
            OPT_BG = 1 << 4,
            OPT_GRAVITY = 1 << 5,
            ALL = OPT_SO3 | OPT_VEL | OPT_BA | OPT_BG | OPT_GRAVITY
        };

        static bool IsOptionWith(Option desired, Option curOption) {
            return (desired == (desired & curOption));
        }
    };

    using RiverOpt = RiverOptOption::Option;

    class Estimator : public ceres::Problem {
    public:
        using Ptr = std::shared_ptr<Estimator>;
        using SplineBundleType = ns_ctraj::SplineBundle<Configor::Prior::SplineOrder>;
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        Configor::Ptr configor;
        SplineBundleType::Ptr splines;
        std::shared_ptr<Eigen::Vector3d> gravity;
        std::shared_ptr<Eigen::Vector3d> ba, bg;

        // manifolds
        static std::shared_ptr<ceres::EigenQuaternionManifold> QUATER_MANIFOLD;
        static std::shared_ptr<ceres::SphereManifold<3>> GRAVITY_MANIFOLD;

        // involved knots recoder: [spline, knot, count]
        std::map<long, std::map<std::size_t, int>> knotRecoder;

    public:
        Estimator(Configor::Ptr configor, SplineBundleType::Ptr splines,
                  const std::shared_ptr<Eigen::Vector3d> &gravity, const std::shared_ptr<Eigen::Vector3d> &ba,
                  const std::shared_ptr<Eigen::Vector3d> &bg);

        static Ptr Create(const Configor::Ptr &configor, const SplineBundleType::Ptr &splines,
                          const std::shared_ptr<Eigen::Vector3d> &gravity, const std::shared_ptr<Eigen::Vector3d> &ba,
                          const std::shared_ptr<Eigen::Vector3d> &bg);

        static ceres::Problem::Options DefaultProblemOptions();

        static ceres::Solver::Options
        DefaultSolverOptions(int threadNum = -1, bool toStdout = true, bool useCUDA = false);

        ceres::Solver::Summary Solve(const ceres::Solver::Options &options = Estimator::DefaultSolverOptions());

        Eigen::MatrixXd GetHessianMatrix();

        void ShowKnotStatus() const;

    public:
        void AddGyroMeasurement(const IMUFrame::Ptr &frame, RiverOpt option, double weight);

        void AddGyroMeasurementWithConstBias(const IMUFrame::Ptr &frame, RiverOpt option, double weight);

        void AddVelPIMForGravityRecovery(double dt, const Eigen::Vector3d &deltaVel, const Eigen::Vector3d &velPim,
                                         RiverOpt option, double weight);

        void AddAcceMeasurement(const IMUFrame::Ptr &frame, RiverOpt option, double weight);

        void AddAcceMeasurementWithConstBias(const IMUFrame::Ptr &frame, RiverOpt option, double weight);

        void AddRadarMeasurement(const RadarTarget::Ptr &radarTar, RiverOpt option, double weight);

        ceres::ResidualBlockId AddAcceBiasPriori(const BiasFilter::StatePack &priori, RiverOpt option);

        ceres::ResidualBlockId AddGyroBiasPriori(const BiasFilter::StatePack &priori, RiverOpt option);

        std::vector<ceres::ResidualBlockId> AddVelSplineTailConstraint(RiverOpt option, double weight);

        std::vector<ceres::ResidualBlockId> AddSo3SplineTailConstraint(RiverOpt option, double weight);

        void AddVelocityConstraint(double time, const Eigen::Vector3d &value, RiverOpt option, double weight);

        void AddRotationConstraint(double time, const Sophus::SO3d &value, RiverOpt option, double weight);

        ceres::ResidualBlockId AddVelPreintegrationConstraint(const Eigen::Vector3d &alpha, double t1, double t2,
                                                              RiverOpt option, double weight);

    protected:
        void AddSo3KnotsData(std::vector<double *> &paramBlockVec, const SplineBundleType::So3SplineType &spline,
                             const SplineMetaType &splineMeta, bool setToConst);

        void AddRdKnotsData(std::vector<double *> &paramBlockVec, const SplineBundleType::RdSplineType &spline,
                            const SplineMetaType &splineMeta, bool setToConst);

        static Eigen::MatrixXd CRSMatrix2EigenMatrix(ceres::CRSMatrix *jacobian_crs_matrix);
    };
}

#endif //RIVER_ESTIMATOR_H
