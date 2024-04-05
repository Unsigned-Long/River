// Copyright (c) 2023. Created on 7/7/23 1:29 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

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
