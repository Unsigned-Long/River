// Copyright (c) 2023. Created on 10/19/23 6:20 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

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
