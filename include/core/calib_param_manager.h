// Copyright (c) 2023. Created on 10/19/23 6:19 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

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
