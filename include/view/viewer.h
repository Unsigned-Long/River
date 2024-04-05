// Copyright (c) 2023. Created on 11/9/23 4:09 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIVER_VIEWER_H
#define RIVER_VIEWER_H

#include "tiny-viewer/core/multi_viewer.h"
#include "config/configor.h"
#include "core/state_manager.h"

namespace ns_river {
    class Viewer : protected ns_viewer::MultiViewer {
    public:
        using Ptr = std::shared_ptr<Viewer>;

    private:
        Configor::Ptr configor;
        StateManager::Ptr stateMagr;

        std::map<std::string, std::vector<std::size_t>> lastEntities;

        std::vector<std::pair<Eigen::Vector3d, ns_viewer::Colour>> virMarkers;

        static std::string VR_WIN_NAME;
        static std::string SPLINE_WIN_NAME;

    public:
        Viewer(const ns_viewer::MultiViewerConfigor &viewerConfigor, Configor::Ptr configor,
               StateManager::Ptr stateManager);

        static Ptr Create(const Configor::Ptr &configor, const StateManager::Ptr &stateManager);

        void RunViewer();

    protected:
        void InitializeViewer(const RiverStatus::StatusPack &status);

        void UpdateSplineWindow(const RiverStatus::StatusPack &status);

        void UpdateVRWindow(const RiverStatus::StatusPack &status);

        void InitVirtualMarkers(int n, double r);

        bool InitSceneFromObj(const std::string &plyFilename, int n);
    };
}


#endif //RIVER_VIEWER_H
