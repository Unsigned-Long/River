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

#include "core/calib_param_manager.h"
#include "spdlog/fmt/bundled/color.h"
#include "util/utils.hpp"
#include "tiny-viewer/object/imu.h"
#include "tiny-viewer/object/radar.h"
#include "tiny-viewer/entity/line.h"

namespace ns_river {

    CalibParamManager::CalibParamManager() : SO3_RtoB(), POS_RinB(Eigen::Vector3d::Zero()), TIME_OFFSET_RtoB(0.0) {}


    CalibParamManager::Ptr CalibParamManager::Create() {
        return std::make_shared<CalibParamManager>();
    }

    void CalibParamManager::Save(const std::string &filename, CerealArchiveType::Enum archiveType) const {
        std::ofstream file(filename, std::ios::out);
        auto ar = GetOutputArchiveVariant(file, archiveType);
        SerializeByOutputArchiveVariant(ar, archiveType, cereal::make_nvp("CalibParam", *this));
    }

    CalibParamManager::Ptr CalibParamManager::Load(const std::string &filename, CerealArchiveType::Enum archiveType) {
        auto calibParamManager = CalibParamManager::Create();
        std::ifstream file(filename, std::ios::in);
        auto ar = GetInputArchiveVariant(file, archiveType);
        SerializeByInputArchiveVariant(ar, archiveType, cereal::make_nvp("CalibParam", *calibParamManager));
        return calibParamManager;
    }


    void CalibParamManager::ShowParamStatus() {
        std::stringstream stream;
#define ITEM(name) fmt::format(fmt::emphasis::bold | fmt::fg(fmt::color::green), name)
#define PARAM(name) fmt::format(fmt::emphasis::bold, name)
#define STREAM_PACK(obj) stream << "-- " << obj << std::endl;

        constexpr std::size_t n = 70;

        STREAM_PACK(std::string(25, '-'))
        STREAM_PACK(ITEM("calibration parameters") << " --")
        STREAM_PACK(std::string(n, '-'))

        // -------------------------
        STREAM_PACK(ITEM("EXTRI"))
        // -------------------------
        STREAM_PACK("")

        const auto EULER_RtoB = EULER_RtoB_DEG();
        STREAM_PACK(PARAM("EULER_RtoB: ") << FormatValueVector<double>(
                {"Xr", "Yp", "Zy"}, {EULER_RtoB(0), EULER_RtoB(1), EULER_RtoB(2)}))

        STREAM_PACK(PARAM("  POS_RinB: ") << FormatValueVector<double>(
                {"Px", "Py", "Pz"}, {POS_RinB(0), POS_RinB(1), POS_RinB(2)}))

        STREAM_PACK("")
        STREAM_PACK(std::string(n, '-'))
        // ----------------------------
        STREAM_PACK(ITEM("TEMPORAL"))
        // ----------------------------
        STREAM_PACK("")

        STREAM_PACK(fmt::format("{}: {:+011.6f} (s)", PARAM("TIME_OFFSET_RtoB"), TIME_OFFSET_RtoB))

        STREAM_PACK("")
        STREAM_PACK(std::string(n, '-'))

        spdlog::info("the detail calibration parameters are below: \n{}", stream.str());

#undef ITEM
#undef PARAM
    }

    void CalibParamManager::VisualizationSensors(ns_viewer::Viewer &viewer) const {
        auto SE3_Body = Sophus::SE3f();
        viewer.AddEntity(
                ns_viewer::IMU::Create(ns_viewer::Posef(SE3_Body.so3().matrix(), SE3_Body.translation()), 0.1)
        );

        auto SE3_RtoB = this->SE3_RtoB().cast<float>();
        auto radar = ns_viewer::Radar::Create(
                ns_viewer::Posef(SE3_RtoB.so3().matrix(), SE3_RtoB.translation()), 0.1, ns_viewer::Colour::Blue()
        );
        auto line = ns_viewer::Line::Create(
                Eigen::Vector3f::Zero(), SE3_RtoB.translation().cast<float>(), ns_viewer::Colour::Black()
        );
        viewer.AddEntity({radar, line});
    }

    Sophus::SE3d CalibParamManager::SE3_RtoB() const {
        return {SO3_RtoB, POS_RinB};
    }

    Eigen::Quaterniond CalibParamManager::Q_RtoB() const {
        return SO3_RtoB.unit_quaternion();
    }

    Eigen::Vector3d CalibParamManager::EULER_RtoB_RAD() const {
        return Q_RtoB().toRotationMatrix().eulerAngles(2, 1, 0);
    }

    Eigen::Vector3d CalibParamManager::EULER_RtoB_DEG() const {
        auto euler = EULER_RtoB_RAD();
        for (int i = 0; i != 3; ++i) { euler(i) *= CalibParamManager::RAD_TO_DEG; }
        return euler;
    }


}
