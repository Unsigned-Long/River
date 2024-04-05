// Copyright (c) 2023. Created on 11/8/23 7:11 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIVER_SIMULATION_HPP
#define RIVER_SIMULATION_HPP

#include <utility>
#include "ctraj/core/simu_trajectory.h"
#include "core/calib_param_manager.h"
#include "ti_mmwave_rospkg/RadarScan.h"

namespace ns_ctraj {
    template<int Order>
    class RiverMotion : public SimuTrajectory<Order> {
    public:
        using Parent = SimuTrajectory<Order>;

    protected:
        Posed _lastState;
        std::uniform_real_distribution<double> _randStride, _randAngle;
        double _staDur, _motionDur;
        std::default_random_engine _engine;

    public:
        explicit RiverMotion(double maxStride, double maxAngleDeg, double staDur, double motionDur, double hz)
                : Parent(0.0, 2.0 * staDur + motionDur, hz), _lastState(Sophus::SO3d(), Eigen::Vector3d::Zero(), 0.0),
                  _randStride(-maxStride, maxStride),
                  _randAngle(-maxAngleDeg / 180.0 * M_PI, maxAngleDeg / 180.0 * M_PI), _staDur(staDur),
                  _motionDur(motionDur),
                  _engine(std::chrono::steady_clock::now().time_since_epoch().count()) { this->SimulateTrajectory(); }

    protected:
        Posed GenPoseSequenceAtTime(double t) override {
            if (t < _staDur || t > _staDur + _motionDur) {
                _lastState.timeStamp = t;
                return _lastState;
            }

            Eigen::Vector3d deltaTrans = Eigen::Vector3d(
                    _randStride(_engine), _randStride(_engine), _randStride(_engine)
            );

            auto rot1 = Eigen::AngleAxisd(_randAngle(_engine), Eigen::Vector3d(0.0, 0.0, 1.0));
            auto rot2 = Eigen::AngleAxisd(_randAngle(_engine), Eigen::Vector3d(0.0, 1.0, 0.0));
            auto rot3 = Eigen::AngleAxisd(_randAngle(_engine), Eigen::Vector3d(1.0, 0.0, 0.0));
            auto deltaRot = Sophus::SO3d((rot3 * rot2 * rot1).matrix());

            _lastState.timeStamp = t;
            _lastState.t = _lastState.t + deltaTrans;
            _lastState.so3 = deltaRot * _lastState.so3;

            return _lastState;
        }
    };
}

namespace ns_river {
    class Simulator {
    protected:
        const std::string _savePath;
        CalibParamManager::Ptr _parMagr;
        ns_ctraj::RiverMotion<4> _traj;
        Eigen::Vector3d _gravity;
    public:

        explicit Simulator(std::string savePath, CalibParamManager::Ptr calibParamManager,
                           const double staDur, double motionDur)
                : _savePath(std::move(savePath)), _parMagr(std::move(calibParamManager)),
                  _traj(ns_ctraj::RiverMotion<4>(0.5, 50.0, staDur, motionDur, 5)), _gravity(0.0, 0.0, -9.8) {}

        void Simulate(int imuHZ, int radarHZ, double acceNoise, double gyroNoise, double radarNoise,
                      double acceBias, double gyroBias) {
            auto bag = std::make_unique<rosbag::Bag>();
            bag->open(_savePath + "/radar_imu.bag", rosbag::BagMode::Write);
            static std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());

            SimulateIMU(bag, "/simu_imu/frame", imuHZ, acceNoise, gyroNoise, acceBias, gyroBias);
            SimulateRadar(bag, "/simu_radar/scan", radarHZ, radarNoise);
        }

        [[nodiscard]]  ns_ctraj::RiverMotion<4> &GetTraj() {
            return _traj;
        }

    protected:
        void SimulateIMU(const std::unique_ptr<rosbag::Bag> &bag, const std::string &imuTopic, int hz,
                         double acceNoise, double gyroNoise, double acceBias, double gyroBias) {
            // imu data
            auto imuMes = this->_traj.GetTrajectory()->ComputeIMUMeasurement(_gravity, 1.0 / hz);

            std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
            std::normal_distribution<double> aNoise(0.0, acceNoise), gNoise(0.0, gyroNoise);
            // write imu msgs
            spdlog::info("insert imu messages for topic '{}', hz: '{}'", imuTopic, hz);
            for (const auto &frame: imuMes) {
                if (frame->GetTimestamp() < ros::TIME_MIN.toSec()) { continue; }

                sensor_msgs::Imu imuMsg;
                imuMsg.header.stamp = ros::Time(frame->GetTimestamp());
                imuMsg.header.frame_id = "imu";

                imuMsg.angular_velocity.x = frame->GetGyro()(0) + gNoise(engine) + gyroBias;
                imuMsg.angular_velocity.y = frame->GetGyro()(1) + gNoise(engine) + gyroBias;
                imuMsg.angular_velocity.z = frame->GetGyro()(2) + gNoise(engine) + gyroBias;

                imuMsg.linear_acceleration.x = frame->GetAcce()(0) + aNoise(engine) + acceBias;
                imuMsg.linear_acceleration.y = frame->GetAcce()(1) + aNoise(engine) + acceBias;
                imuMsg.linear_acceleration.z = frame->GetAcce()(2) + aNoise(engine) + acceBias;

                bag->write(imuTopic, imuMsg.header.stamp, imuMsg);
            }
        }

        void SimulateRadar(const std::unique_ptr<rosbag::Bag> &bag, const std::string &radarTopic, int hz,
                           double radarNoise) {
            // radar data
            std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
            std::uniform_real_distribution<double> uTarPos;
            // simu targets
            std::vector<Eigen::Vector3d> targets;
            for (int i = 0; i < 100; ++i) {
                Eigen::Vector3d target(uTarPos(engine), uTarPos(engine), uTarPos(engine));
                targets.push_back(target);
            }
            std::uniform_int_distribution<std::size_t> uTarIdx(0UL, targets.size() - 1UL);
            std::normal_distribution<double> noise(0.0, radarNoise);

            std::vector<RadarTarget::Ptr> array;
            for (const auto &item: _traj.GetTrajectory()->Sampling(1.0 / static_cast<double>(hz))) {
                double timeByIMU = item.timeStamp;
                auto obv = RadarTarget::Create(
                        timeByIMU - _parMagr->TIME_OFFSET_RtoB,
                        // attention: here is 'timeByIMU', not 'timeByIMU - TIME_OFFSET_RtoB'
                        _traj.GetTrajectory()->RadarStaticMeasurement(
                                timeByIMU, targets.at(uTarIdx(engine)), _parMagr->SE3_RtoB()
                        ) + Eigen::Vector4d{0.0, 0.0, 0.0, noise(engine)}
                );
                array.push_back(obv);
            }

            // write radar msgs
            spdlog::info("insert radar messages for topic '{}', hz: '{}'", radarTopic, hz);
            for (const auto &tar: array) {
                if (tar->GetTimestamp() < ros::TIME_MIN.toSec()) { continue; }

                ti_mmwave_rospkg::RadarScan tarMsg;

                tarMsg.header.stamp = ros::Time(tar->GetTimestamp());
                tarMsg.header.frame_id = "radar";

                tarMsg.x = static_cast<float>(tar->GetTargetXYZ()(0));
                tarMsg.y = static_cast<float>(tar->GetTargetXYZ()(1));
                tarMsg.z = static_cast<float>(tar->GetTargetXYZ()(2));
                tarMsg.velocity = static_cast<float>(tar->GetRadialVelocity());
                bag->write(radarTopic, tarMsg.header.stamp, tarMsg);
            }
        }
    };
}

#endif //RIVER_SIMULATION_HPP
