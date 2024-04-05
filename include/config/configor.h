// Copyright (c) 2023. Created on 10/19/23 6:20 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIVER_CONFIGOR_H
#define RIVER_CONFIGOR_H

#include "cereal/archives/json.hpp"
#include "cereal/cereal.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/set.hpp"
#include "util/utils.hpp"
#include "util/enum_cast.hpp"
#include "core/calib_param_manager.h"
#include "sensor/imu_data_loader.h"
#include "sensor/radar_data_loader.h"

namespace ns_river {
    struct Configor {
    public:
        using Ptr = std::shared_ptr<Configor>;

    public:
        struct DataStream {
            std::string IMUTopic;
            std::string IMUMsgType;

            std::string RadarTopic;
            std::string RadarMsgType;

            CalibParamManager CalibParam;

            std::string OutputPath;

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(IMUTopic), CEREAL_NVP(IMUMsgType),
                   CEREAL_NVP(RadarTopic), CEREAL_NVP(RadarMsgType),
                   CEREAL_NVP(CalibParam), CEREAL_NVP(OutputPath));
            }
        } dataStream;

        struct Prior {
            static constexpr int SplineOrder = 3;

            double GravityNorm;

            double SO3SplineKnotDist;
            double VelSplineKnotDist;

            double AcceWeight;
            double AcceBiasRandomWalk;
            double GyroWeight;
            double GyroBiasRandomWalk;
            double RadarWeight;

            double CauchyLossForRadarFactor;

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(
                        CEREAL_NVP(GravityNorm),
                        CEREAL_NVP(SO3SplineKnotDist), CEREAL_NVP(VelSplineKnotDist),
                        CEREAL_NVP(AcceWeight), CEREAL_NVP(AcceBiasRandomWalk),
                        CEREAL_NVP(GyroWeight), CEREAL_NVP(GyroBiasRandomWalk),
                        CEREAL_NVP(RadarWeight),
                        CEREAL_NVP(CauchyLossForRadarFactor)
                );
            }
        } prior{};

        struct Preference {
            /**
             * when the mode is 'DEBUG_MODE', then:
             * 1. the solving information from ceres would be output on the console
             * 2.
             */
            static bool DEBUG_MODE;

            static std::string SO3Spline;
            static std::string VelSpline;

            // these two splines are not used currently
            static std::string BaSpline;
            static std::string BgSpline;

            static std::string PublishTopic;

            static double StatePublishDelay;

            std::uint32_t IMUMsgQueueSize;
            std::uint32_t RadarMsgQueueSize;
            std::uint32_t IncrementalOptRate;

            std::string ObjFileForDisplay;

            bool OutputResultsWithTimeAligned;

            bool VisualizeVelocityInBody;

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(
                        CEREAL_NVP(IMUMsgQueueSize), CEREAL_NVP(RadarMsgQueueSize),
                        CEREAL_NVP(IncrementalOptRate), CEREAL_NVP(ObjFileForDisplay),
                        CEREAL_NVP(OutputResultsWithTimeAligned), CEREAL_NVP(VisualizeVelocityInBody)
                );
            }
        } preference{};

    public:
        Configor();

        static Ptr Create();

        // load configure information from the xml file
        static Configor::Ptr
        Load(const std::string &filename, CerealArchiveType::Enum archiveType = CerealArchiveType::Enum::YAML);

        // load configure information from the xml file
        void Save(const std::string &filename, CerealArchiveType::Enum archiveType = CerealArchiveType::Enum::YAML);

        // print the main fields
        void PrintMainFields();

    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(
                    cereal::make_nvp("DataStream", dataStream),
                    cereal::make_nvp("Prior", prior),
                    cereal::make_nvp("Preference", preference)
            );
        }
    };
}


#endif //RIVER_CONFIGOR_H
