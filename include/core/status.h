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

#ifndef RIVER_STATUS_H
#define RIVER_STATUS_H

#include "exception"
#include "string"
#include "util/enum_cast.hpp"
#include "mutex"

namespace ns_river {
    struct Status : std::exception {
        enum class Flag {
            FINE, WARNING, ERROR, CRITICAL
        };
    public:
        Flag flag;
        std::string what;

        Status(Flag flag, std::string what) : flag(flag), what(std::move(what)) {}

        Status() : flag(Flag::FINE), what() {}
    };

    using namespace magic_enum::bitwise_operators;

#define LOCK_RIVER_STATUS std::unique_lock<std::mutex> statusLock(RiverStatus::StatusMutex);

    class RiverStatus {
    public:
        struct StateManager {
            enum class Status : std::uint32_t {
                /**
                 * @brief options
                 */
                NONE = 1 << 0,
                HasInitialized = 1 << 1,
                ShouldQuit = 1 << 2,
                NewStateNeedToDraw = 1 << 3,
                NewStateNeedToPublish = 1 << 4,
            };

            static Status CurStatus;

            static double ValidStateEndTime;
        };

        struct DataManager {
            enum class Status : std::uint32_t {
                /**
                 * @brief options
                 */
                NONE = 1 << 0,
                RadarTarAryForInitIsReady = 1 << 1,
            };

            static Status CurStatus;
        };

        struct StatusPack {
        public:
            StateManager::Status StateMagr;
            double ValidStateEndTime;

            DataManager::Status DataMagr;

        public:
            StatusPack(StateManager::Status stateMagr, double ValidStateEndTime, DataManager::Status dataMagr);
        };

    public:
        static std::mutex StatusMutex;

    public:
        template<class EnumType>
        static bool IsWith(EnumType desired, EnumType current) {
            return (desired == (desired & current));
        }

        static StatusPack GetStatusPackSafely();

    };
}

#endif //RIVER_STATUS_H
