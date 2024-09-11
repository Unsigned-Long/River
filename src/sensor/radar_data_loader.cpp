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

#include "sensor/radar_data_loader.h"
#include "util/enum_cast.hpp"
#include "core/status.h"
#include "pcl_conversions/pcl_conversions.h"

namespace ns_river {

    RadarDataUnpacker::Ptr RadarDataUnpacker::Create() {
        return std::make_shared<RadarDataUnpacker>();
    }

    RadarTargetArray::Ptr RadarDataUnpacker::Unpack(const ti_mmwave_rospkg::RadarScanConstPtr &msg) {
        auto target = RadarTarget::Create(
                msg->header.stamp.toSec(), {msg->x, msg->y, msg->z}, msg->velocity
        );

        return RadarTargetArray::Create(target->GetTimestamp(), {target});
    }

    RadarTargetArray::Ptr RadarDataUnpacker::Unpack(const ti_mmwave_rospkg::RadarScanCustomConstPtr &msg) {
        auto target = RadarTarget::Create(
                msg->header.stamp.toSec(), {msg->x, msg->y, msg->z}, msg->velocity
        );

        return RadarTargetArray::Create(target->GetTimestamp(), {target});
    }

    RadarTargetArray::Ptr RadarDataUnpacker::Unpack(const sensor_msgs::PointCloud2ConstPtr &msg) {
        // for (const auto &item: msg->fields) { std::cout << item.name << ' '; }

        XRIORadarPOSVCloud radarTargets;
        pcl::fromROSMsg(*msg, radarTargets);

        std::vector<RadarTarget::Ptr> targets;
        targets.reserve(radarTargets.size());

        for (const auto &tar: radarTargets) {
            if (std::isnan(tar.x) || std::isnan(tar.y) || std::isnan(tar.z) ||
                std::isnan(tar.v_doppler_mps)) { continue; }

            targets.push_back(
                    RadarTarget::Create(msg->header.stamp.toSec(), {tar.x, tar.y, tar.z}, tar.v_doppler_mps)
            );
        }

        return RadarTargetArray::Create(msg->header.stamp.toSec(), targets);
    }
}