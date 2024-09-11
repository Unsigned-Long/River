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
