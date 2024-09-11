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

#include <utility>
#include "core/estimator.h"
#include "factor/imu_gyro_factor.hpp"
#include "factor/gravity_factor.hpp"
#include "factor/imu_acce_factor.hpp"
#include "factor/radar_factor.hpp"
#include "factor/bias_factor.hpp"
#include "factor/tail_factor.hpp"
#include "factor/spline_factor.hpp"
#include "factor/vel_preintegration_factor.hpp"

namespace ns_river {

    std::shared_ptr<ceres::EigenQuaternionManifold> Estimator::QUATER_MANIFOLD(new ceres::EigenQuaternionManifold());
    std::shared_ptr<ceres::SphereManifold<3>> Estimator::GRAVITY_MANIFOLD(new ceres::SphereManifold<3>());

    ceres::Problem::Options Estimator::DefaultProblemOptions() {
        return ns_ctraj::TrajectoryEstimator<Configor::Prior::SplineOrder>::DefaultProblemOptions();
    }

    ceres::Solver::Options Estimator::DefaultSolverOptions(int threadNum, bool toStdout, bool useCUDA) {
        auto defaultSolverOptions = ns_ctraj::TrajectoryEstimator<Configor::Prior::SplineOrder>::DefaultSolverOptions(
                threadNum, toStdout, useCUDA
        );
        if (!useCUDA) {
            defaultSolverOptions.linear_solver_type = ceres::DENSE_SCHUR;
        }
        defaultSolverOptions.trust_region_strategy_type = ceres::DOGLEG;
        return defaultSolverOptions;
    }

    Estimator::Estimator(Configor::Ptr configor, SplineBundleType::Ptr splines,
                         const std::shared_ptr<Eigen::Vector3d> &gravity, const std::shared_ptr<Eigen::Vector3d> &ba,
                         const std::shared_ptr<Eigen::Vector3d> &bg)
            : ceres::Problem(DefaultProblemOptions()), configor(std::move(configor)),
              splines(std::move(splines)), gravity(gravity), ba(ba), bg(bg) {}

    Estimator::Ptr Estimator::Create(const Configor::Ptr &configor, const SplineBundleType::Ptr &splines,
                                     const std::shared_ptr<Eigen::Vector3d> &gravity,
                                     const std::shared_ptr<Eigen::Vector3d> &ba,
                                     const std::shared_ptr<Eigen::Vector3d> &bg) {
        return std::make_shared<Estimator>(configor, splines, gravity, ba, bg);
    }

    /**
     * param blocks:
     * [ SO3 | ... | SO3 | BG | ... | BG ]
     */
    void Estimator::AddGyroMeasurement(const IMUFrame::Ptr &frame, RiverOpt option, double weight) {
        auto time = frame->GetTimestamp();

        if (!splines->TimeInRangeForSo3(time, Configor::Preference::SO3Spline)) {
            // if this frame is not in range
            return;
        }
        if (!splines->TimeInRangeForRd(time, Configor::Preference::BgSpline)) {
            // if this frame is not in range
            return;
        }

        // prepare metas for splines
        SplineMetaType so3Meta, bgMeta;
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta);
        splines->CalculateRdSplineMeta(Configor::Preference::BgSpline, {{time, time}}, bgMeta);

        auto costFunc = IMUGyroFactor<Configor::Prior::SplineOrder>::Create(so3Meta, bgMeta, frame, weight);

        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }
        for (int i = 0; i < static_cast<int>(bgMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(so3Meta.NumParameters() + bgMeta.NumParameters());

        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                !RiverOptOption::IsOptionWith(RiverOpt::OPT_SO3, option)
        );
        AddRdKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::BgSpline), bgMeta,
                !RiverOptOption::IsOptionWith(RiverOpt::OPT_BG, option)
        );

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
    }

    ceres::Solver::Summary Estimator::Solve(const ceres::Solver::Options &options) {
        ceres::Solver::Summary summary;
        ceres::Solve(options, this, &summary);
        return summary;
    }

    void Estimator::AddRdKnotsData(std::vector<double *> &paramBlockVec,
                                   const Estimator::SplineBundleType::RdSplineType &spline,
                                   const Estimator::SplineMetaType &splineMeta, bool setToConst) {
        // for each segment
        for (const auto &seg: splineMeta.segments) {
            // the factor 'seg.dt * 0.5' is the treatment for numerical accuracy
            auto idxMaster = spline.ComputeTIndex(seg.t0 + seg.dt * 0.5).second;

            // from the first control point to the last control point
            for (std::size_t i = idxMaster; i < idxMaster + seg.NumParameters(); ++i) {
                auto *data = const_cast<double *>(spline.GetKnot(static_cast<int>(i)).data());

                this->AddParameterBlock(data, 3);

                paramBlockVec.push_back(data);
                // set this param block to be constant
                if (setToConst) { this->SetParameterBlockConstant(data); }

                // knot recoder
                knotRecoder[reinterpret_cast<long>(&spline)][i]++;
            }
        }
    }

    void Estimator::AddSo3KnotsData(std::vector<double *> &paramBlockVec,
                                    const Estimator::SplineBundleType::So3SplineType &spline,
                                    const Estimator::SplineMetaType &splineMeta, bool setToConst) {
        // for each segment
        for (const auto &seg: splineMeta.segments) {
            // the factor 'seg.dt * 0.5' is the treatment for numerical accuracy
            auto idxMaster = spline.ComputeTIndex(seg.t0 + seg.dt * 0.5).second;

            // from the first control point to the last control point
            for (std::size_t i = idxMaster; i < idxMaster + seg.NumParameters(); ++i) {
                auto *data = const_cast<double *>(spline.GetKnot(static_cast<int>(i)).data());
                // the local parameterization is very important!!!
                this->AddParameterBlock(data, 4, QUATER_MANIFOLD.get());

                paramBlockVec.push_back(data);
                // set this param block to be constant
                if (setToConst) { this->SetParameterBlockConstant(data); }

                // knot recoder
                knotRecoder[reinterpret_cast<long>(&spline)][i]++;
            }
        }
    }

    /**
     * param blocks:
     * [ GRAVITY ]
     */
    void Estimator::AddVelPIMForGravityRecovery(const double dt, const Eigen::Vector3d &deltaVel,
                                                const Eigen::Vector3d &velPim, RiverOpt option, double weight) {
        auto costFunc = GravityFactor::Create(dt, deltaVel, velPim, weight);

        // gravity
        costFunc->AddParameterBlock(3);

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.push_back(gravity->data());

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
        this->SetManifold(gravity->data(), GRAVITY_MANIFOLD.get());

        if (!RiverOptOption::IsOptionWith(RiverOpt::OPT_GRAVITY, option)) {
            this->SetParameterBlockConstant(gravity->data());
        }
    }

    Eigen::MatrixXd Estimator::CRSMatrix2EigenMatrix(ceres::CRSMatrix *jacobian_crs_matrix) {
        Eigen::MatrixXd J(jacobian_crs_matrix->num_rows, jacobian_crs_matrix->num_cols);
        J.setZero();

        std::vector<int> jacobian_crs_matrix_rows, jacobian_crs_matrix_cols;
        std::vector<double> jacobian_crs_matrix_values;
        jacobian_crs_matrix_rows = jacobian_crs_matrix->rows;
        jacobian_crs_matrix_cols = jacobian_crs_matrix->cols;
        jacobian_crs_matrix_values = jacobian_crs_matrix->values;

        int cur_index_in_cols_and_values = 0;
        // rows is a num_rows + 1 sized array
        int row_size = static_cast<int>(jacobian_crs_matrix_rows.size()) - 1;
        // outer loop traverse rows, inner loop traverse cols and values
        for (int row_index = 0; row_index < row_size; ++row_index) {
            while (cur_index_in_cols_and_values < jacobian_crs_matrix_rows[row_index + 1]) {
                J(row_index, jacobian_crs_matrix_cols[cur_index_in_cols_and_values]) =
                        jacobian_crs_matrix_values[cur_index_in_cols_and_values];
                cur_index_in_cols_and_values++;
            }
        }
        return J;
    }

    Eigen::MatrixXd Estimator::GetHessianMatrix() {
        ceres::Problem::EvaluateOptions EvalOpts;
        ceres::CRSMatrix jacobian_crs_matrix;
        this->Evaluate(EvalOpts, nullptr, nullptr, nullptr, &jacobian_crs_matrix);
        Eigen::MatrixXd J = CRSMatrix2EigenMatrix(&jacobian_crs_matrix);
        Eigen::MatrixXd H = J.transpose() * J;
        return H;
    }

    /**
     * param blocks:
     * [ SO3 | ... | SO3 | VEL | ... | VEL | BA | ... | BA | GRAVITY ]
     */
    void Estimator::AddAcceMeasurement(const IMUFrame::Ptr &frame, RiverOpt option, double weight) {
        auto time = frame->GetTimestamp();

        if (!splines->TimeInRangeForSo3(time, Configor::Preference::SO3Spline)) {
            // if this frame is not in range
            return;
        }
        if (!splines->TimeInRangeForRd(time, Configor::Preference::VelSpline)) {
            // if this frame is not in range
            return;
        }
        if (!splines->TimeInRangeForRd(time, Configor::Preference::BaSpline)) {
            // if this frame is not in range
            return;
        }

        // prepare metas for splines
        SplineMetaType so3Meta, velMeta, baMeta;
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta);
        splines->CalculateRdSplineMeta(Configor::Preference::VelSpline, {{time, time}}, velMeta);
        splines->CalculateRdSplineMeta(Configor::Preference::BaSpline, {{time, time}}, baMeta);

        auto costFunc = IMUAcceFactor<Configor::Prior::SplineOrder>::Create(so3Meta, velMeta, baMeta, frame, weight);

        // knots of so3 spline
        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }
        // knots of vel spline
        for (int i = 0; i < static_cast<int>(velMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }
        // knots of ba spline
        for (int i = 0; i < static_cast<int>(baMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }
        // gravity
        costFunc->AddParameterBlock(3);

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(so3Meta.NumParameters() + velMeta.NumParameters() + baMeta.NumParameters() + 1);

        // knots of so3 spline
        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                !RiverOptOption::IsOptionWith(RiverOpt::OPT_SO3, option)
        );
        // knots of vel spline
        AddRdKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::VelSpline), velMeta,
                !RiverOptOption::IsOptionWith(RiverOpt::OPT_VEL, option)
        );
        // knots of ba spline
        AddRdKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::BaSpline), baMeta,
                !RiverOptOption::IsOptionWith(RiverOpt::OPT_BA, option)
        );
        // gravity
        paramBlockVec.push_back(gravity->data());

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
        this->SetManifold(gravity->data(), GRAVITY_MANIFOLD.get());

        if (!RiverOptOption::IsOptionWith(RiverOpt::OPT_GRAVITY, option)) {
            this->SetParameterBlockConstant(gravity->data());
        }
    }

    /**
     * param blocks:
     * [ SO3 | ... | SO3 | VEL | ... | VEL ]
     */
    void Estimator::AddRadarMeasurement(const RadarTarget::Ptr &radarTar, RiverOpt option, double weight) {
        auto time = radarTar->GetTimestamp();

        if (!splines->TimeInRangeForSo3(time, Configor::Preference::SO3Spline)) {
            // if this frame is not in range
            return;
        }
        if (!splines->TimeInRangeForRd(time, Configor::Preference::VelSpline)) {
            // if this frame is not in range
            return;
        }

        // prepare metas for splines
        SplineMetaType so3Meta, velMeta;
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta);
        splines->CalculateRdSplineMeta(Configor::Preference::VelSpline, {{time, time}}, velMeta);

        auto costFunc = RadarFactor<Configor::Prior::SplineOrder>::Create(
                configor->dataStream.CalibParam, radarTar, so3Meta, velMeta, weight
        );

        // knots of so3 spline
        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }
        // knots of vel spline
        for (int i = 0; i < static_cast<int>(velMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }

        costFunc->SetNumResiduals(1);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(so3Meta.NumParameters() + velMeta.NumParameters());

        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                !RiverOptOption::IsOptionWith(RiverOpt::OPT_SO3, option)
        );
        AddRdKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::VelSpline), velMeta,
                !RiverOptOption::IsOptionWith(RiverOpt::OPT_VEL, option)
        );

        this->AddResidualBlock(
                costFunc, new ceres::CauchyLoss(configor->prior.CauchyLossForRadarFactor * weight), paramBlockVec
        );
    }

    /**
     * param blocks:
     * [ SO3 | ... | SO3 | BG ]
     */
    void Estimator::AddGyroMeasurementWithConstBias(const IMUFrame::Ptr &frame, RiverOpt option, double weight) {
        auto time = frame->GetTimestamp();

        if (!splines->TimeInRangeForSo3(time, Configor::Preference::SO3Spline)) {
            // if this frame is not in range
            return;
        }

        // prepare metas for splines
        SplineMetaType so3Meta;
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta);

        auto costFunc = IMUGyroFactorWithConstBias<Configor::Prior::SplineOrder>::Create(so3Meta, frame, weight);

        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }
        // BIAS OF GYRO
        costFunc->AddParameterBlock(3);

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(so3Meta.NumParameters() + 1);

        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                !RiverOptOption::IsOptionWith(RiverOpt::OPT_SO3, option)
        );
        paramBlockVec.push_back(bg->data());

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);

        if (!RiverOptOption::IsOptionWith(RiverOpt::OPT_BG, option)) {
            this->SetParameterBlockConstant(bg->data());
        }
    }

    /**
     * param blocks:
     * [ SO3 | ... | SO3 | VEL | ... | VEL | BA | GRAVITY ]
     */
    void Estimator::AddAcceMeasurementWithConstBias(const IMUFrame::Ptr &frame, RiverOpt option, double weight) {
        auto time = frame->GetTimestamp();

        if (!splines->TimeInRangeForSo3(time, Configor::Preference::SO3Spline)) {
            // if this frame is not in range
            return;
        }
        if (!splines->TimeInRangeForRd(time, Configor::Preference::VelSpline)) {
            // if this frame is not in range
            return;
        }

        // prepare metas for splines
        SplineMetaType so3Meta, velMeta;
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta);
        splines->CalculateRdSplineMeta(Configor::Preference::VelSpline, {{time, time}}, velMeta);

        auto costFunc = IMUAcceFactorWithConstBias<Configor::Prior::SplineOrder>::Create(
                so3Meta, velMeta, frame, weight
        );

        // knots of so3 spline
        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }
        // knots of vel spline
        for (int i = 0; i < static_cast<int>(velMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }
        // BIAS OF ACCE
        costFunc->AddParameterBlock(3);
        // gravity
        costFunc->AddParameterBlock(3);

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(so3Meta.NumParameters() + velMeta.NumParameters() + 2);

        // knots of so3 spline
        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                !RiverOptOption::IsOptionWith(RiverOpt::OPT_SO3, option)
        );
        // knots of vel spline
        AddRdKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::VelSpline), velMeta,
                !RiverOptOption::IsOptionWith(RiverOpt::OPT_VEL, option)
        );
        // BIAS OF ACCE
        paramBlockVec.push_back(ba->data());
        // gravity
        paramBlockVec.push_back(gravity->data());

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
        this->SetManifold(gravity->data(), GRAVITY_MANIFOLD.get());

        if (!RiverOptOption::IsOptionWith(RiverOpt::OPT_BA, option)) {
            this->SetParameterBlockConstant(ba->data());
        }
        if (!RiverOptOption::IsOptionWith(RiverOpt::OPT_GRAVITY, option)) {
            this->SetParameterBlockConstant(gravity->data());
        }
    }

    void Estimator::ShowKnotStatus() const {
        for (const auto &[splineAddress, knotInfo]: knotRecoder) {
            std::stringstream stream;
            stream << "spline: " << splineAddress << ", ";
            for (const auto &[knotId, count]: knotInfo) {
                stream << '[' << knotId << ": " << count << "] ";
            }
            spdlog::info("{}", stream.str());
        }
    }

    ceres::ResidualBlockId Estimator::AddAcceBiasPriori(const BiasFilter::StatePack &priori, RiverOpt option) {
        auto costFunc = BiasFactor::Create(priori.state, priori.var);
        costFunc->AddParameterBlock(3);
        costFunc->SetNumResiduals(3);
        auto id = this->AddResidualBlock(costFunc, nullptr, ba->data());

        if (!RiverOptOption::IsOptionWith(RiverOpt::OPT_BA, option)) {
            this->SetParameterBlockConstant(ba->data());
        }
        return id;
    }

    ceres::ResidualBlockId Estimator::AddGyroBiasPriori(const BiasFilter::StatePack &priori, RiverOpt option) {
        auto costFunc = BiasFactor::Create(priori.state, priori.var);
        costFunc->AddParameterBlock(3);
        costFunc->SetNumResiduals(3);
        auto id = this->AddResidualBlock(costFunc, nullptr, bg->data());

        if (!RiverOptOption::IsOptionWith(RiverOpt::OPT_BG, option)) {
            this->SetParameterBlockConstant(bg->data());
        }
        return id;
    }

    /**
     * param blocks:
     * [ example for three order spline: VEL | VEL | VEL ]
     */
    std::vector<ceres::ResidualBlockId> Estimator::AddVelSplineTailConstraint(RiverOpt option, double weight) {
        auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
        std::vector<ceres::ResidualBlockId> idVec;
        for (int j = 0; j < Configor::Prior::SplineOrder - 2; ++j) {
            auto costFunc = RdTailFactor::Create(weight);
            costFunc->AddParameterBlock(3);
            costFunc->AddParameterBlock(3);
            costFunc->AddParameterBlock(3);
            costFunc->SetNumResiduals(3);

            // organize the param block vector
            std::vector<double *> paramBlockVec(3);
            for (int i = 0; i < 3; ++i) {
                paramBlockVec.at(i) = velSpline.GetKnot(
                        j + i + static_cast<int>(velSpline.GetKnots().size()) - Configor::Prior::SplineOrder
                ).data();
            }

            auto id = this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
            idVec.push_back(id);

            if (!RiverOptOption::IsOptionWith(RiverOpt::OPT_VEL, option)) {
                for (auto &knot: paramBlockVec) { this->SetParameterBlockConstant(knot); }
            }
        }
        return idVec;
    }

    /**
     * param blocks:
     * [ SO3 | SO3 | SO3 ]
     */
    std::vector<ceres::ResidualBlockId> Estimator::AddSo3SplineTailConstraint(RiverOpt option, double weight) {
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        std::vector<ceres::ResidualBlockId> idVec;
        for (int j = 0; j < Configor::Prior::SplineOrder - 2; ++j) {
            auto costFunc = So3TailFactor::Create(weight);
            costFunc->AddParameterBlock(4);
            costFunc->AddParameterBlock(4);
            costFunc->AddParameterBlock(4);
            costFunc->SetNumResiduals(3);

            // organize the param block vector
            std::vector<double *> paramBlockVec(3);
            for (int i = 0; i < 3; ++i) {
                paramBlockVec.at(i) = so3Spline.GetKnot(
                        j + i + static_cast<int>(so3Spline.GetKnots().size()) - Configor::Prior::SplineOrder
                ).data();
            }

            auto id = this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
            idVec.push_back(id);

            for (const auto &item: paramBlockVec) { this->SetManifold(item, QUATER_MANIFOLD.get()); }

            if (!RiverOptOption::IsOptionWith(RiverOpt::OPT_SO3, option)) {
                for (auto &knot: paramBlockVec) { this->SetParameterBlockConstant(knot); }
            }
        }
        return idVec;
    }

    /**
     * param blocks:
     * [ VEL | ... | VEL ]
     */
    void Estimator::AddVelocityConstraint(double time, const Eigen::Vector3d &value, RiverOpt option, double weight) {
        if (!splines->TimeInRangeForRd(time, Configor::Preference::VelSpline)) {
            // if this frame is not in range
            return;
        }
        // prepare metas for splines
        SplineMetaType velMeta;
        splines->CalculateRdSplineMeta(Configor::Preference::VelSpline, {{time, time}}, velMeta);

        auto costFunc = RdSplineFactor<Configor::Prior::SplineOrder>::Create(velMeta, time, value, weight);

        // knots of vel spline
        for (int i = 0; i < static_cast<int>(velMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(velMeta.NumParameters());

        // knots of vel spline
        AddRdKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::VelSpline), velMeta,
                !RiverOptOption::IsOptionWith(RiverOpt::OPT_VEL, option)
        );

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
    }

    void Estimator::AddRotationConstraint(double time, const Sophus::SO3d &value, RiverOpt option, double weight) {
        if (!splines->TimeInRangeForSo3(time, Configor::Preference::SO3Spline)) {
            // if this frame is not in range
            return;
        }
        // prepare metas for splines
        SplineMetaType so3Meta;
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta);

        auto costFunc = So3SplineFactor<Configor::Prior::SplineOrder>::Create(so3Meta, time, value, weight);

        // knots of so3 spline
        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(so3Meta.NumParameters());

        // knots of so3 spline
        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                !RiverOptOption::IsOptionWith(RiverOpt::OPT_SO3, option)
        );

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
    }

    /**
     * param blocks:
     * [ VEL | ... | VEL | GRAVITY ]
     */
    ceres::ResidualBlockId Estimator::AddVelPreintegrationConstraint(const Eigen::Vector3d &alpha, double t1, double t2,
                                                                     RiverOpt option, double weight) {
        if (!splines->TimeInRangeForRd(t1, Configor::Preference::VelSpline) ||
            !splines->TimeInRangeForRd(t2, Configor::Preference::VelSpline)) {
            // if this frame is not in range
            return nullptr;
        }

        // prepare metas for splines
        SplineMetaType velMeta;
        splines->CalculateRdSplineMeta(Configor::Preference::VelSpline, {{t1, t1},
                                                                         {t2, t2}}, velMeta);

        auto costFunc = VelPreintegrationFactor<Configor::Prior::SplineOrder>::Create(velMeta, alpha, t1, t2, weight);

        // knots of vel spline
        for (int i = 0; i < static_cast<int>(velMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }

        // gravity
        costFunc->AddParameterBlock(3);

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(velMeta.NumParameters() + 1);

        // knots of vel spline
        AddRdKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::VelSpline), velMeta,
                !RiverOptOption::IsOptionWith(RiverOpt::OPT_VEL, option)
        );
        // gravity
        paramBlockVec.push_back(gravity->data());

        auto id = this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
        this->SetManifold(gravity->data(), GRAVITY_MANIFOLD.get());

        if (!RiverOptOption::IsOptionWith(RiverOpt::OPT_GRAVITY, option)) {
            this->SetParameterBlockConstant(gravity->data());
        }
        return id;
    }
}