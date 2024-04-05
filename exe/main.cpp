// Copyright (c) 2023. Created on 10/19/23 6:04 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#include "ros/ros.h"
#include "thread"
#include "core/status.h"
#include "core/river.h"
#include "view/view_util.h"

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
                 "+---------+-----------------------------------------------+\n"
                 "| River  | https://github.com/Unsigned-Long/river.git     |\n"
                 "+--------+------------------------------------------------+\n"
                 "| Author | Shuolong Chen                                  |\n"
                 "+--------+------------------------------------------------+\n"
                 "| E-Mail | shlchen@whu.edu.cn                             |\n"
                 "+---------+------------------------------------------------+" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "river_prog_node");
    try {
        ConfigSpdlog();

        PrintLibInfo();

        // auto configor = ns_river::Configor();
        // configor.Save("/home/csl/ros_ws/river/src/river/configor/configor.yaml");
        // std::cin.get();

        // obtain the path of config file
        std::string configPath;
        if (!ros::param::get("/river_prog_node/config_path", configPath)) {
            throw ns_river::Status(
                    ns_river::Status::Flag::CRITICAL,
                    "the configure path couldn't obtained from ros param '/river_prog_node/config_path'."
            );
        }
        spdlog::info("loading configure from json file '{}'...", configPath);

        // load the configure file
        auto configor = ns_river::Configor::Load(configPath);
        configor->PrintMainFields();

        // config the output path of viewer (just for debug)
        ns_river::ViewUtil::DataOutputPath = configor->dataStream.OutputPath;

        // create the 'River'
        auto river = ns_river::River::Create(configor);

        // perform solving
        river->Run();

        // save results
        river->Save();

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