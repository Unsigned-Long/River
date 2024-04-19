// Copyright (c) 2023. Created on 11/8/23 7:11 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#include "ros/ros.h"
#include "thread"
#include "core/status.h"
#include "core/river.h"
#include "view/view_util.h"
#include "tiny-viewer/core/multi_viewer.h"
#include "nofree/simulation.hpp"
#include "cereal/types/utility.hpp"

// config the 'spdlog' log pattern
void ConfigSpdlog() {
    // [log type]-[thread]-[time] message
    spdlog::set_pattern("%^[%L]%$-[%t]-[%H:%M:%S.%e] %v");

    // set log level
    spdlog::set_level(spdlog::level::debug);
}

void PrintLibInfo() {
    // ns_pretab::PrettyTable tab;
    // tab.addRowGrids(0, 1, 0, 2, ns_pretab::Align::CENTER, "");
    // tab.addGrid(1, 0, "RIs-Calib");
    // tab.addGrid(1, 1, "https://github.com/Unsigned-Long/RIs-Calib.git");
    // tab.addGrid(2, 0, "Author");
    // tab.addGrid(2, 1, "Shuolong Chen");
    // tab.addGrid(3, 0, "E-Mail");
    // tab.addGrid(3, 1, "shlchen@whu.edu.cn");
    // std::cout << tab << std::endl;
    std::cout << "+---------------------------------------------------------+\n"
                 "|     _|_|_|    _|_|_|  _|      _|  _|_|_|_|  _|_|_|      |\n"
                 "|     _|    _|    _|    _|      _|  _|        _|    _|    |\n"
                 "|     _|_|_|      _|    _|      _|  _|_|_|    _|_|_|      |\n"
                 "|     _|    _|    _|      _|  _|    _|        _|    _|    |\n"
                 "|     _|    _|  _|_|_|      _|      _|_|_|_|  _|    _|    |\n"
                 "+--------+------------------------------------------------+\n"
                 "| River  | https://github.com/Unsigned-Long/river.git     |\n"
                 "+--------+------------------------------------------------+\n"
                 "| Author | Shuolong Chen                                  |\n"
                 "+--------+------------------------------------------------+\n"
                 "| E-Mail | shlchen@whu.edu.cn                             |\n"
                 "+--------+------------------------------------------------+" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

template<class Type>
Type GetParamFromROS(const std::string &param) {
    Type par;
    if (!ros::param::get(param, par)) {
        throw ns_river::Status(
                ns_river::Status::Flag::CRITICAL,
                fmt::format("the ros param couldn't obtained from '{}'.", param)
        );
    }
    return par;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "river_simu_node");
    try {
        ConfigSpdlog();

        PrintLibInfo();

        auto outputPath = GetParamFromROS<std::string>("/river_simu_node/output_path");
        spdlog::info("config output path: '{}'", outputPath);

        if (!std::filesystem::exists(outputPath)) {
            if (!std::filesystem::create_directories(outputPath)) {
                throw ns_river::Status(
                        ns_river::Status::Flag::CRITICAL, "the 'output_path' is not valid."
                );
            }
        }

        auto staDur = GetParamFromROS<double>("/river_simu_node/stationary_duration");
        spdlog::info("stationary duration: '{:.6f}'", staDur);

        auto motionDur = GetParamFromROS<double>("/river_simu_node/motion_duration");
        spdlog::info("motion duration: '{:.6f}'", motionDur);

        auto extriTempParams = GetParamFromROS<std::string>("/river_simu_node/extri_temp_params");
        spdlog::info("spatiotemporal parameters: '{}'", extriTempParams);

        auto items = ns_river::SplitString(extriTempParams, ';');
        if (items.size() != 8) {
            throw ns_river::Status(
                    ns_river::Status::Flag::CRITICAL,
                    "wrong spatiotemporal parameters are set, they are should be [qx;qy;qz;qw;px;py;pz;tm]."
            );
        }

        auto calibParam = ns_river::CalibParamManager::Create();
        calibParam->SO3_RtoB = Sophus::SO3d(Eigen::Quaterniond(
                std::stod(items[3]), std::stod(items[0]), std::stod(items[1]), std::stod(items[2])
        ));
        calibParam->POS_RinB = Eigen::Vector3d(std::stod(items[4]), std::stod(items[5]), std::stod(items[6]));
        calibParam->TIME_OFFSET_RtoB = std::stod(items[7]);
        calibParam->ShowParamStatus();

        auto frequency = GetParamFromROS<std::string>("/river_simu_node/frequency");
        spdlog::info("frequency: '{}'", frequency);

        auto freq = ns_river::SplitString(frequency, ';');
        if (freq.size() != 2) {
            throw ns_river::Status(
                    ns_river::Status::Flag::CRITICAL,
                    "wrong frequency are set, they are should be [imu;radar]."
            );
        }
        int imuFreq = std::stoi(freq[0]);
        int radarFreq = std::stoi(freq[1]);
        spdlog::info("imu frequency: '{}'", imuFreq);
        spdlog::info("radar frequency: '{}'", radarFreq);

        auto noise = GetParamFromROS<std::string>("/river_simu_node/noise");
        auto n = ns_river::SplitString(noise, ';');
        if (n.size() != 3) {
            throw ns_river::Status(
                    ns_river::Status::Flag::CRITICAL,
                    "wrong noises are set, they are should be [acce;gyro;radar]."
            );
        }
        double acceNoise = std::stod(n[0]), gyroNoise = std::stod(n[1]), radarNoise = std::stod(n[2]);
        spdlog::info("accelerator noise: '{}'", acceNoise);
        spdlog::info("gyroscope noise: '{}'", gyroNoise);
        spdlog::info("radar noise: '{}'", radarNoise);

        auto bias = GetParamFromROS<std::string>("/river_simu_node/bias");
        auto b = ns_river::SplitString(bias, ';');
        if (b.size() != 2) {
            throw ns_river::Status(
                    ns_river::Status::Flag::CRITICAL,
                    "wrong biases are set, they are should be [acce;gyro]."
            );
        }
        double acceBias = std::stod(b[0]), gyroBias = std::stod(b[1]);
        spdlog::info("accelerator bias: '{}'", acceBias);
        spdlog::info("gyroscope bias: '{}'", gyroBias);

        auto simulator = ns_river::Simulator(outputPath, calibParam, staDur, motionDur);

        simulator.Simulate(imuFreq, radarFreq, acceNoise, gyroNoise, radarNoise, acceBias, gyroBias);

        simulator.GetTraj().GetTrajectory()->Save(outputPath + "/trajectory.json");

        std::vector<std::pair<double, Eigen::Vector3d>> velocityInW, velocityInB;
        std::vector<std::pair<double, Sophus::SO3d>> quatBtoW;
        auto traj = simulator.GetTraj().GetTrajectory();

        for (double t = traj->MinTime(); t < traj->MaxTime();) {
            Eigen::Vector3d LIN_VEL_BtoWinW = traj->LinearVeloInRef(t);
            velocityInW.emplace_back(t, LIN_VEL_BtoWinW);

            auto SO3_CurToW = traj->Pose(t).so3();
            quatBtoW.emplace_back(t, SO3_CurToW);

            Eigen::Vector3d LIN_VEL_BtoWinB = SO3_CurToW.inverse() * LIN_VEL_BtoWinW;
            velocityInB.emplace_back(t, LIN_VEL_BtoWinB);

            t += 0.01;
        }
        // velocity
        {
            std::ofstream file(outputPath + "/velocity.json", std::ios::out);
            cereal::JSONOutputArchive ar(file);
            ar(
                    cereal::make_nvp("velocity_in_world", velocityInW),
                    cereal::make_nvp("velocity_in_body", velocityInB)
            );
        }

        // rotation
        {
            std::ofstream file(outputPath + "/rotation.json", std::ios::out);
            cereal::JSONOutputArchive ar(file);
            ar(cereal::make_nvp("quat_body_to_world", quatBtoW));
        }

        simulator.GetTraj().VisualizationDynamic(outputPath);

    } catch (const ns_river::Status &status) {
        // if error happened, print it
        switch (status.flag) {
            case ns_river::Status::Flag::FINE:
                // this case usually won't happen
                spdlog::info(status.what);
                break;
            case ns_river::Status::Flag::WARNING:
                spdlog::warn(status.what);
                break;
            case ns_river::Status::Flag::ERROR:
                spdlog::error(status.what);
                break;
            case ns_river::Status::Flag::CRITICAL:
                spdlog::critical(status.what);
                break;
        }
    } catch (const std::exception &e) {
        // an unknown exception not thrown by this program
        spdlog::critical(e.what());
    }
    ros::shutdown();
    return 0;
}