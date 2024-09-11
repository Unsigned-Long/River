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
