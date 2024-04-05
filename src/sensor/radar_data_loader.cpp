// Copyright (c) 2023. Created on 7/7/23 1:31 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

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