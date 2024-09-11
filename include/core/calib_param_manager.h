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

#ifndef RIVER_CALIB_PARAM_MANAGER_H
#define RIVER_CALIB_PARAM_MANAGER_H

#include "ctraj/view/traj_viewer.h"
#include "cereal/types/map.hpp"
#include "spdlog/spdlog.h"
#include "util/cereal_archive_helper.hpp"
#include "ctraj/utils/sophus_utils.hpp"

namespace ns_river {
    class CalibParamManager {
    public:
        using Ptr = std::shared_ptr<CalibParamManager>;

    public:
        // trans radian angle to degree angle
        constexpr static double RAD_TO_DEG = 180.0 / M_PI;
        // trans degree angle to radian angle
        constexpr static double DEG_TO_RAD = M_PI / 180.0;

    public:
        // extrinsics
        Sophus::SO3d SO3_RtoB;
        Eigen::Vector3d POS_RinB;

        // time offset
        double TIME_OFFSET_RtoB;

    public:

        // the constructor
        explicit CalibParamManager();

        // the creator
        static CalibParamManager::Ptr Create();

        // save the parameters to file using cereal library
        void
        Save(const std::string &filename, CerealArchiveType::Enum archiveType = CerealArchiveType::Enum::YAML) const;

        // load the parameters from file using cereal library
        static CalibParamManager::Ptr
        Load(const std::string &filename, CerealArchiveType::Enum archiveType = CerealArchiveType::Enum::YAML);

        // print the parameters in the console
        void ShowParamStatus();

        void VisualizationSensors(ns_viewer::Viewer &viewer) const;

        // lie algebra vector space se3
        [[nodiscard]] Sophus::SE3d SE3_RtoB() const;

        [[nodiscard]] Eigen::Quaterniond Q_RtoB() const;

        // the euler angles [radian and degree format]
        [[nodiscard]] Eigen::Vector3d EULER_RtoB_RAD() const;

        [[nodiscard]] Eigen::Vector3d EULER_RtoB_DEG() const;

    public:
        // Serialization
        template<class Archive>
        void serialize(Archive &archive) {
            archive(CEREAL_NVP(SO3_RtoB), CEREAL_NVP(POS_RinB), CEREAL_NVP(TIME_OFFSET_RtoB));
        }

    };
}


#endif //RIVER_CALIB_PARAM_MANAGER_H
