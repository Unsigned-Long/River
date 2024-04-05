// Copyright (c) 2023. Created on 10/19/23 7:51 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIVER_RIVER_H
#define RIVER_RIVER_H

#include "config/configor.h"
#include "ros/ros.h"
#include "core/state_manager.h"
#include "view/viewer.h"

namespace ns_river {
    class River {
    public:
        using Ptr = std::shared_ptr<River>;

    private:
        // ros-related members
        ros::NodeHandlePtr handler;
        Configor::Ptr configor;

        ros::Publisher statePublisher;

        DataManager::Ptr dataMagr;
        StateManager::Ptr stateMagr;

        Viewer::Ptr viewer;

        std::shared_ptr<std::thread> stateMagrThread;
        std::shared_ptr<std::thread> viewerThread;

    public:
        explicit River(const Configor::Ptr &configor);

        static Ptr Create(const Configor::Ptr &configor);

        void Run();

        void Save();

    protected:
        void PublishRiverState(const RiverStatus::StatusPack &status);
    };
}

#endif //RIVER_RIVER_H
