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

#ifndef RIVER_RADAR_DATA_LOADER_H
#define RIVER_RADAR_DATA_LOADER_H

#include "rosbag/message_instance.h"
#include "sensor/radar.h"
#include "ti_mmwave_rospkg/RadarScan.h"
#include "ti_mmwave_rospkg/RadarScanCustom.h"
#include "sensor_msgs/PointCloud2.h"

struct EIGEN_ALIGN16 XRIORadarTarget {
    PCL_ADD_POINT4D;   // quad-word XYZ
    float snr_db;         // CFAR cell to side noise ratio in [dB]
    float noise_db;       // CFAR noise level of the side of the detected cell in [dB]
    float v_doppler_mps;  // Doppler's velocity in [m/s]

    PCL_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
        XRIORadarTarget,
        (float, x, x)(float, y, y)(float, z, z)(float, snr_db, snr_db)
                (float, noise_db, noise_db)(float, v_doppler_mps, v_doppler_mps)
)
using XRIORadarPOSVCloud = pcl::PointCloud<XRIORadarTarget>;

namespace ns_river {
    enum class RadarMsgType {
        AWR1843BOOST_RAW,
        AWR1843BOOST_CUSTOM,
        POINTCLOUD2_XRIO,
    };

    class RadarDataUnpacker {
    public:
        using Ptr = std::shared_ptr<RadarDataUnpacker>;

    public:
        explicit RadarDataUnpacker() = default;

        static RadarDataUnpacker::Ptr Create();

        static RadarTargetArray::Ptr Unpack(const ti_mmwave_rospkg::RadarScanConstPtr &msg);

        static RadarTargetArray::Ptr Unpack(const ti_mmwave_rospkg::RadarScanCustomConstPtr &msg);

        static RadarTargetArray::Ptr Unpack(const sensor_msgs::PointCloud2ConstPtr &msg);
    };
}

#endif //RIVER_RADAR_DATA_LOADER_H
