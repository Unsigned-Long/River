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

#include "config/configor.h"

namespace ns_river {
    // -------------------
    // static member field
    // -------------------
    std::string Configor::Preference::SO3Spline = "so3";
    std::string Configor::Preference::VelSpline = "vel";
    std::string Configor::Preference::BaSpline = "ba";
    std::string Configor::Preference::BgSpline = "bg";

    std::string Configor::Preference::PublishTopic = "/river/state";
    double Configor::Preference::StatePublishDelay = 0.05;

    bool Configor::Preference::DEBUG_MODE = false;

    Configor::Configor() = default;

    Configor::Ptr Configor::Create() {
        return std::make_shared<Configor>();
    }

    Configor::Ptr Configor::Load(const std::string &filename, CerealArchiveType::Enum archiveType) {
        auto configor = Configor::Create();
        std::ifstream file(filename, std::ios::in);
        auto ar = GetInputArchiveVariant(file, archiveType);
        SerializeByInputArchiveVariant(ar, archiveType, cereal::make_nvp("Configor", *configor));
        return configor;
    }

    void Configor::Save(const std::string &filename, CerealArchiveType::Enum archiveType) {
        std::ofstream file(filename, std::ios::out);
        auto ar = GetOutputArchiveVariant(file, archiveType);
        SerializeByOutputArchiveVariant(ar, archiveType, cereal::make_nvp("Configor", *this));
    }

    void Configor::PrintMainFields() {
#define DESC_FIELD(field) #field, field
#define DESC_FORMAT "\n{:>35}: {}"
        spdlog::info(
                "main fields of configor:"
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT

                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT

                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT,
                DESC_FIELD(dataStream.IMUTopic),
                DESC_FIELD(dataStream.IMUMsgType),
                DESC_FIELD(dataStream.RadarTopic),
                DESC_FIELD(dataStream.RadarMsgType),
                DESC_FIELD(dataStream.OutputPath),

                DESC_FIELD(prior.SplineOrder),
                DESC_FIELD(prior.GravityNorm),
                DESC_FIELD(prior.SO3SplineKnotDist),
                DESC_FIELD(prior.VelSplineKnotDist),
                DESC_FIELD(prior.AcceWeight),
                DESC_FIELD(prior.AcceBiasRandomWalk),
                DESC_FIELD(prior.GyroWeight),
                DESC_FIELD(prior.GyroBiasRandomWalk),
                DESC_FIELD(prior.RadarWeight),
                DESC_FIELD(prior.CauchyLossForRadarFactor),

                DESC_FIELD(preference.IMUMsgQueueSize),
                DESC_FIELD(preference.RadarMsgQueueSize),
                DESC_FIELD(preference.IncrementalOptRate),
                DESC_FIELD(preference.ObjFileForDisplay),
                DESC_FIELD(preference.OutputResultsWithTimeAligned)
        );

        dataStream.CalibParam.ShowParamStatus();

#undef DESC_FIELD
#undef DESC_FORMAT
    }
}
