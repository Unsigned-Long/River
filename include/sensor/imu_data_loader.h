// Copyright (c) 2023. Created on 9/20/23 8:06 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIVER_IMU_DATA_LOADER_H
#define RIVER_IMU_DATA_LOADER_H

#include "sensor_msgs/Imu.h"
#include "sbg_driver/SbgImuData.h"
#include "rosbag/message_instance.h"
#include "sensor/imu.h"

namespace ns_river {
    enum class IMUMsgType {
        SENSOR_IMU,
        SBG_IMU
    };

    class IMUDataUnpacker {
    public:
        using Ptr = std::shared_ptr<IMUDataUnpacker>;

    public:
        explicit IMUDataUnpacker() = default;

        static IMUDataUnpacker::Ptr Create();

        static IMUFrame::Ptr Unpack(const sensor_msgs::Imu::ConstPtr &msg);

        static IMUFrame::Ptr Unpack(const sbg_driver::SbgImuData::ConstPtr &msg);
    };

}


#endif //RIVER_IMU_DATA_LOADER_H
