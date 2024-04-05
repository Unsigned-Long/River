// Copyright (c) 2023. Created on 9/20/23 8:06 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#include "sensor/imu_data_loader.h"
#include "util/enum_cast.hpp"
#include "core/status.h"
#include "spdlog/fmt/fmt.h"

namespace ns_river {


    IMUDataUnpacker::Ptr IMUDataUnpacker::Create() {
        return std::make_shared<IMUDataUnpacker>();
    }

    IMUFrame::Ptr IMUDataUnpacker::Unpack(const sensor_msgs::Imu::ConstPtr &msg) {

        auto acce = Eigen::Vector3d(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z
        );
        auto gyro = Eigen::Vector3d(
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z
        );

        return IMUFrame::Create(msg->header.stamp.toSec(), gyro, acce);
    }

    IMUFrame::Ptr IMUDataUnpacker::Unpack(const sbg_driver::SbgImuData_<std::allocator<void>>::ConstPtr &msg) {

        auto acce = Eigen::Vector3d(
                msg->accel.x,
                msg->accel.y,
                msg->accel.z
        );
        auto gyro = Eigen::Vector3d(
                msg->gyro.x,
                msg->gyro.y,
                msg->gyro.z
        );

        return IMUFrame::Create(msg->header.stamp.toSec(), gyro, acce);
    }
}