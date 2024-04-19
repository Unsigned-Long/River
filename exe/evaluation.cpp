// Copyright (c) 2023. Created on 12/15/23 3:09 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.


#include "ros/ros.h"
#include "thread"
#include "core/status.h"
#include "core/river.h"
#include "view/view_util.h"
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "river_eval_node");
    try {
        ConfigSpdlog();

        PrintLibInfo();

        // -----------------------------------------------------
        // extract pose from trajectory based on input reference
        // -----------------------------------------------------
#if 0
        const std::string trajectory = "/home/csl/ros_ws/river/src/river/output/simu-low-dynamic/simulation/trajectory.json";
        const std::string velocityRef = "/home/csl/ros_ws/river/src/river/output/simu-low-dynamic/river_output/velocity.json";
        const std::string rotationRef = "/home/csl/ros_ws/river/src/river/output/simu-low-dynamic/river_output/rotation.json";

        // trajectory
        auto traj = ns_ctraj::Trajectory<4>::Load(trajectory);

        // velocity
        {
            std::vector<std::pair<double, Eigen::Vector3d>> inVelocityInW, inVelocityInB;
            {
                std::ifstream ifile(velocityRef, std::ios::in);
                cereal::JSONInputArchive ar(ifile);
                ar(cereal::make_nvp("velocity_in_world", inVelocityInW),
                   cereal::make_nvp("velocity_in_body", inVelocityInB));
            }

            std::vector<std::pair<double, Eigen::Vector3d>> outVelocityInW, outVelocityInB;
            for (const auto &item: inVelocityInW) {
                Eigen::Vector3d LIN_VEL_BtoWinW = traj->LinearVeloInRef(item.first);
                outVelocityInW.emplace_back(item.first, LIN_VEL_BtoWinW);
            }
            for (const auto &item: inVelocityInB) {
                Eigen::Vector3d LIN_VEL_BtoWinW = traj->LinearVeloInRef(item.first);
                auto SO3_CurToW = traj->Pose(item.first).so3();
                Eigen::Vector3d LIN_VEL_BtoWinB = SO3_CurToW.inverse() * LIN_VEL_BtoWinW;
                outVelocityInB.emplace_back(item.first, LIN_VEL_BtoWinB);
            }

            {
                auto filename = velocityRef.substr(0, velocityRef.find_last_of('/')) + "/velocity_gt.json";
                std::ofstream ofile(filename, std::ios::out);
                cereal::JSONOutputArchive ar(ofile);
                ar(cereal::make_nvp("velocity_in_world", outVelocityInW),
                   cereal::make_nvp("velocity_in_body", outVelocityInB));
            }
        }

        // rotation
        {
            std::vector<std::pair<double, Sophus::SO3d>> inQuatBtoW;
            {
                std::ifstream ifile(rotationRef, std::ios::in);
                cereal::JSONInputArchive ar(ifile);
                ar(cereal::make_nvp("quat_body_to_world", inQuatBtoW));
            }
            std::vector<std::pair<double, Sophus::SO3d>> outQuatBtoW;
            for (const auto &item: inQuatBtoW) {
                auto SO3_CurToW = traj->Pose(item.first).so3();
                outQuatBtoW.emplace_back(item.first, SO3_CurToW);
            }
            {
                auto filename = rotationRef.substr(0, velocityRef.find_last_of('/')) + "/rotation_gt.json";
                std::ofstream ofile(filename, std::ios::out);
                cereal::JSONOutputArchive ar(ofile);
                ar(cereal::make_nvp("quat_body_to_world", outQuatBtoW));
            }
        }
#endif
        // -------------------------------------------------------------------------------
        // compute the difference of the velocity estimates from two radar-inertial suites
        // -------------------------------------------------------------------------------
#if 1
        std::vector<std::string> ws = {
                "/home/csl/ros_ws/river/src/river/dataset/xrio/workshop/river_output_center",
                "/home/csl/ros_ws/river/src/river/dataset/xrio/workshop/river_output_left",
                "/home/csl/ros_ws/river/src/river/dataset/xrio/workshop/river_output_right",
        };
        using Traj = ns_ctraj::SplineBundle<ns_river::Configor::Prior::SplineOrder>;
        std::vector<Traj::Ptr> trajVec;
        std::vector<double> stVec, etVec;
        for (const auto &path: ws) {
            trajVec.push_back(Traj::Load(path + "/splines.json"));
            stVec.push_back(trajVec.back()->GetRdSpline(ns_river::Configor::Preference::VelSpline).MinTime());
            etVec.push_back(trajVec.back()->GetRdSpline(ns_river::Configor::Preference::VelSpline).MaxTime());
        }

        double st = *std::max_element(stVec.begin(), stVec.end());
        double et = *std::min_element(etVec.begin(), etVec.end());
        auto Alignment = [st, et](const Traj::Ptr &traj, const std::string &output) {
            const auto &velSpline = traj->GetRdSpline(ns_river::Configor::Preference::VelSpline);
            const auto &so3Spline = traj->GetSo3Spline(ns_river::Configor::Preference::SO3Spline);

            std::vector<std::pair<double, Eigen::Vector3d>> velocityInW, velocityInB;
            for (double t = st; t < et;) {
                Eigen::Vector3d LIN_VEL_BtoWinW = velSpline.Evaluate(t);
                velocityInW.emplace_back(t, LIN_VEL_BtoWinW);

                auto SO3_CurToW = so3Spline.Evaluate(t);

                Eigen::Vector3d LIN_VEL_BtoWinB = SO3_CurToW.inverse() * LIN_VEL_BtoWinW;
                velocityInB.emplace_back(t, LIN_VEL_BtoWinB);

                t += 0.01;
            }

            std::ofstream file(output, std::ios::out);
            cereal::JSONOutputArchive ar(file);
            ar(
                    cereal::make_nvp("velocity_in_world", velocityInW),
                    cereal::make_nvp("velocity_in_body", velocityInB)
            );
        };
        for (int i = 0; i < static_cast<int>(ws.size()); ++i) {
            const auto &filename = ws.at(i) + "/velocity_aligned.json";
            Alignment(trajVec.at(i), filename);
        }

#endif
        // ----------------------------------------------------------------------
        // compute ground truth of linear velocities using the trajectory (x-RIO)
        // ----------------------------------------------------------------------
#if  0
        std::string trajFilename = "/home/csl/dataset/x-rio/multi_radar_inertial_datasets_jgn_2022/lab_floor_groundtruth.csv";
        std::ifstream file(trajFilename, std::ios::in);
        std::vector<ns_ctraj::Posed> poseVec;
        std::string line;
        while (std::getline(file, line)) {
            auto strVec = ns_river::SplitString(line, ' ');
            double t = stod(strVec.at(0));
            double x = stod(strVec.at(1));
            double y = stod(strVec.at(2));
            double z = stod(strVec.at(3));
            double qx = stod(strVec.at(4));
            double qy = stod(strVec.at(5));
            double qz = stod(strVec.at(6));
            double qw = stod(strVec.at(7));
            // std::cout << std::fixed << std::setprecision(3)
            //           << t << ',' << x << ',' << y << ',' << z << ','
            //           << qx << ',' << qy << ',' << qz << ',' << qw << std::endl;
            // std::cin.get();
            poseVec.push_back(ns_ctraj::Posed(Sophus::SO3d(Eigen::Quaterniond(qw, qx, qy, qz)), {x, y, z}, t));
        }
        file.close();

        // fitting
        const double minTime = poseVec.front().timeStamp;
        const double maxTime = poseVec.back().timeStamp;
        const double hz = static_cast<double>(poseVec.size()) / (maxTime - minTime);
        auto traj = ns_ctraj::Trajectory<4>::Create(1.0 / hz, minTime, maxTime);
        auto estimator = ns_ctraj::TrajectoryEstimator<4>::Create(traj);
        for (const auto &pose: poseVec) {
            estimator->AddSE3Measurement(pose, ns_ctraj::OptimizationOption::ALL, 1.0, 1.0);
        }
        auto sum = estimator->Solve();
        std::cout << sum.BriefReport() << std::endl;
        ns_ctraj::Viewer viewer;
        traj->Visualization(viewer);
        // here, we find that the groundtruth is poor (abnormal oscillation), so do not continue such evaluation
        viewer.RunInSingleThread();
#endif

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