// Copyright (c) 2023. Created on 10/19/23 7:55 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#include "core/data_manager.h"

namespace ns_river {
    // -------------------
    // static member field
    // -------------------
    std::mutex DataManager::IMUDataSeqMutex = {};
    std::mutex DataManager::RadarDataSeqMutex = {};
    std::mutex DataManager::RadarInitFramesMutex = {};

    DataManager::DataManager(const ros::NodeHandlePtr &handler, const ns_river::Configor::Ptr &configor)
            : handler(handler), configor(configor), RIVER_TIME_EPOCH(std::optional<double>()) {
        spdlog::info("'DataManager' has been booted, thread id: {}.", RIVER_TO_STR(std::this_thread::get_id()));
        // -------------------------------------------------------
        // create imu message subscriber based on the message type
        // -------------------------------------------------------
        IMUMsgType imuMsgType;
        try {
            imuMsgType = EnumCast::stringToEnum<IMUMsgType>(configor->dataStream.IMUMsgType);
        } catch (...) {
            throw Status(
                    Status::Flag::WARNING,
                    fmt::format(
                            "Unsupported IMU Type: '{}'. "
                            "Currently supported IMU types are: \n"
                            "(1) SENSOR_IMU: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html\n"
                            "(2)    SBG_IMU: https://github.com/SBG-Systems/sbg_ros_driver.git\n"
                            "...\n"
                            "If you need to use other IMU types, "
                            "please 'Issues' us on the profile of the github repository.",
                            configor->dataStream.IMUMsgType
                    )
            );
        }
        /**
         * create subscriber of imu data
         * @topic configor->dataStream.IMUTopic
         * @queue_size configor->preference.IMUMsgQueueSize
         * @callback HandleIMUMessage(...)
         */
        switch (imuMsgType) {
            case IMUMsgType::SENSOR_IMU:
                imuSuber = handler->subscribe<sensor_msgs::Imu>(
                        configor->dataStream.IMUTopic, configor->preference.IMUMsgQueueSize,
                        [this](auto &&PH1) {
                            HandleIMUMessage<sensor_msgs::Imu>(std::forward<decltype(PH1)>(PH1));
                        }
                );
                break;
            case IMUMsgType::SBG_IMU:
                imuSuber = handler->subscribe<sbg_driver::SbgImuData>(
                        configor->dataStream.IMUTopic, configor->preference.IMUMsgQueueSize,
                        [this](auto &&PH1) {
                            HandleIMUMessage<sbg_driver::SbgImuData>(std::forward<decltype(PH1)>(PH1));
                        }
                );
                break;
        }
        // ---------------------------------------------------------
        // create radar message subscriber based on the message type
        // ---------------------------------------------------------
        RadarMsgType radarMsgType;
        try {
            radarMsgType = EnumCast::stringToEnum<RadarMsgType>(configor->dataStream.RadarMsgType);
        } catch (...) {
            throw Status(
                    Status::Flag::WARNING,
                    fmt::format(
                            "Unsupported Radar Type: '{}'. "
                            "Currently supported radar types are: \n"
                            "(1)    AWR1843BOOST_RAW: https://github.com/Unsigned-Long/ti_mmwave_rospkg.git\n"
                            "(2) AWR1843BOOST_CUSTOM: https://github.com/Unsigned-Long/ti_mmwave_rospkg.git\n"
                            "(3)    POINTCLOUD2_POSV: 'sensor_msgs/PointCloud2' with point format: [x, y, z, velocity]\n"
                            "(4)   POINTCLOUD2_POSIV: 'sensor_msgs/PointCloud2' with point format: [x, y, z, intensity, velocity]\n"
                            "...\n"
                            "If you need to use other radar types, "
                            "please 'Issues' us on the profile of the github repository.",
                            configor->dataStream.RadarMsgType
                    )
            );
        }
        /**
         * create subscriber of radar data
         * @topic configor->dataStream.RadarTopic
         * @queue_size configor->preference.RadarMsgQueueSize
         * @callback HandleRadarMessage(...)
         */
        switch (radarMsgType) {
            case RadarMsgType::AWR1843BOOST_RAW:
                radarSuber = handler->subscribe<ti_mmwave_rospkg::RadarScan>(
                        configor->dataStream.RadarTopic, configor->preference.RadarMsgQueueSize,
                        [this](auto &&PH1) {
                            HandleRadarMessage<ti_mmwave_rospkg::RadarScan>(std::forward<decltype(PH1)>(PH1));
                        }
                );
                break;
            case RadarMsgType::AWR1843BOOST_CUSTOM:
                radarSuber = handler->subscribe<ti_mmwave_rospkg::RadarScanCustom>(
                        configor->dataStream.RadarTopic, configor->preference.RadarMsgQueueSize,
                        [this](auto &&PH1) {
                            HandleRadarMessage<ti_mmwave_rospkg::RadarScanCustom>(std::forward<decltype(PH1)>(PH1));
                        }
                );
                break;
            case RadarMsgType::POINTCLOUD2_XRIO:
                radarSuber = handler->subscribe<sensor_msgs::PointCloud2>(
                        configor->dataStream.RadarTopic, configor->preference.RadarMsgQueueSize,
                        [this](auto &&PH1) {
                            HandleRadarMessage<sensor_msgs::PointCloud2>(std::forward<decltype(PH1)>(PH1));
                        }
                );
                break;
        }
    }

    DataManager::Ptr DataManager::Create(const ros::NodeHandlePtr &handler, const Configor::Ptr &configor) {
        return std::make_shared<DataManager>(handler, configor);
    }

    void DataManager::ShowDataStatus() const {
        std::size_t s = 0;
        double st = 0.0, et = 0.0;
        {
            // lock imu data and obtain its size, start time, and end time quickly
            LOCK_IMU_DATA_SEQ
            if (!imuDataSeq.empty()) {
                s = imuDataSeq.size();
                st = imuDataSeq.front()->GetTimestamp();
                et = imuDataSeq.back()->GetTimestamp();
            }
        }
        spdlog::info("imu data size: {:02}, time span from '{:.6f}' to '{:.6f}'", s, st, et);

        s = 0, st = 0.0, et = 0.0;
        {
            // lock radar data and obtain its size, start time, and end time quickly
            LOCK_RADAR_DATA_SEQ
            if (!radarDataSeq.empty()) {
                s = radarDataSeq.size();
                st = radarDataSeq.front()->GetTimestamp();
                et = radarDataSeq.back()->GetTimestamp();
            }
        }
        spdlog::info("radar data size: {:02}, time span from '{:.6f}' to '{:.6f}'", s, st, et);
    }

    void DataManager::OrganizeRadarTarAryForInit(const RadarTargetArray::Ptr &rawTarAry) {
        // if this system has not been initialization, construct radar rawTarAry arrays
        static std::vector<RadarTarget::Ptr> tarAry = {};

        tarAry.insert(tarAry.end(), rawTarAry->GetTargets().cbegin(), rawTarAry->GetTargets().cend());

        static double scanHeadTime = tarAry.front()->GetTimestamp();

        if (tarAry.back()->GetTimestamp() - scanHeadTime < 0.1) { return; }

        // if time span is larger than 0.1 (s), try to organize these targets as a radar frame
        if (tarAry.size() < 3) {
            // this radar frame is invalid, clear current status
            tarAry.clear();

            LOCK_RADAR_INIT_FRAMES
            radarTarAryForInit.clear();
            return;
        }

        // this radar frame is valid, compute average time
        double avgTime = 0.0;
        for (const auto &item: tarAry) { avgTime += item->GetTimestamp(); }
        avgTime /= static_cast<double>(tarAry.size());

        {
            LOCK_RADAR_INIT_FRAMES
            radarTarAryForInit.push_back(RadarTargetArray::Create(avgTime, tarAry));

            if (radarTarAryForInit.size() > 10) {
                // keep data that lasting for 1.0 (s), i.e., ten frames, each lasting 0.1 (s)
                radarTarAryForInit.pop_front();

                LOCK_RIVER_STATUS
                RiverStatus::DataManager::CurStatus |= RiverStatus::DataManager::Status::RadarTarAryForInitIsReady;
            }
        }

        // clear current status
        scanHeadTime = tarAry.back()->GetTimestamp();
        tarAry.clear();
    }

    std::list<RadarTargetArray::Ptr> DataManager::GetRadarTarAryForInitSafely() const {
        LOCK_RADAR_INIT_FRAMES
        return radarTarAryForInit;
    }

    std::list<IMUFrame::Ptr> DataManager::ExtractIMUDataPieceSafely(double start, double end) {
        LOCK_IMU_DATA_SEQ
        std::list<IMUFrame::Ptr> dataSeq;
        auto sIter = std::find_if(imuDataSeq.rbegin(), imuDataSeq.rend(), [start](const IMUFrame::Ptr &frame) {
            return frame->GetTimestamp() < start;
        }).base();
        auto eIter = std::find_if(imuDataSeq.rbegin(), imuDataSeq.rend(), [end](const IMUFrame::Ptr &frame) {
            return frame->GetTimestamp() < end;
        }).base();
        std::copy(sIter, eIter, std::back_inserter(dataSeq));
        return dataSeq;
    }

    std::list<IMUFrame::Ptr> DataManager::ExtractIMUDataPieceSafely(double start) {
        LOCK_IMU_DATA_SEQ
        std::list<IMUFrame::Ptr> dataSeq;
        auto sIter = std::find_if(imuDataSeq.rbegin(), imuDataSeq.rend(), [start](const IMUFrame::Ptr &frame) {
            return frame->GetTimestamp() < start;
        }).base();
        std::copy(sIter, imuDataSeq.end(), std::back_inserter(dataSeq));
        return dataSeq;
    }

    std::list<RadarTarget::Ptr> DataManager::ExtractRadarDataPieceSafely(double start, double end) {
        LOCK_RADAR_DATA_SEQ
        std::list<RadarTarget::Ptr> tarSeq;
        auto sIter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [start](const RadarTarget::Ptr &tar) {
            return tar->GetTimestamp() < start;
        }).base();
        auto eIter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [end](const RadarTarget::Ptr &tar) {
            return tar->GetTimestamp() < end;
        }).base();
        std::copy(sIter, eIter, std::back_inserter(tarSeq));
        return tarSeq;
    }

    std::list<RadarTarget::Ptr> DataManager::ExtractRadarDataPieceSafely(double start) {
        LOCK_RADAR_DATA_SEQ
        std::list<RadarTarget::Ptr> tarSeq;
        auto sIter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [start](const RadarTarget::Ptr &tar) {
            return tar->GetTimestamp() < start;
        }).base();
        std::copy(sIter, radarDataSeq.end(), std::back_inserter(tarSeq));
        return tarSeq;
    }

    double DataManager::GetEndTimeSafely() const {
        double m1, m2;
        {
            LOCK_IMU_DATA_SEQ
            m1 = imuDataSeq.back()->GetTimestamp();
        }
        {
            LOCK_RADAR_DATA_SEQ
            m2 = radarDataSeq.back()->GetTimestamp();
        }
        return std::min(m1, m2);
    }

    const std::optional<double> &DataManager::GetRiverTimeEpoch() const {
        return RIVER_TIME_EPOCH;
    }

    double DataManager::GetIMUEndTimeSafely() const {
        LOCK_IMU_DATA_SEQ
        return imuDataSeq.back()->GetTimestamp();
    }

    double DataManager::GetRadarEndTimeSafely() const {
        LOCK_RADAR_DATA_SEQ
        return radarDataSeq.back()->GetTimestamp();
    }

    std::pair<Eigen::Vector3d, Eigen::Matrix3d> DataManager::AcceMeanVar(const std::list<IMUFrame::Ptr> &data) {
        if (data.empty()) { return {Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()}; }

        Eigen::MatrixXd matrix(data.size(), 3);
        int i = 0;
        for (const auto &v: data) { matrix.row(i++) = v->GetAcce(); }

        Eigen::Vector3d mean = matrix.colwise().mean();
        Eigen::Matrix3d var = ((matrix.rowwise() - matrix.colwise().mean()).transpose() *
                               (matrix.rowwise() - matrix.colwise().mean())) / static_cast<double>(data.size() - 1);

        return {mean, var};
    }

    std::pair<Eigen::Vector3d, Eigen::Matrix3d> DataManager::GyroMeanVar(const std::list<IMUFrame::Ptr> &data) {
        if (data.empty()) { return {Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()}; }

        Eigen::MatrixXd matrix(data.size(), 3);
        int i = 0;
        for (const auto &v: data) { matrix.row(i++) = v->GetGyro(); }

        Eigen::Vector3d mean = matrix.colwise().mean();
        Eigen::Matrix3d var = ((matrix.rowwise() - matrix.colwise().mean()).transpose() *
                               (matrix.rowwise() - matrix.colwise().mean())) / static_cast<double>(data.size() - 1);

        return {mean, var};
    }

    void DataManager::EraseOldDataPieceSafely(double time) {
        {
            LOCK_IMU_DATA_SEQ
            auto iter = std::find_if(imuDataSeq.rbegin(), imuDataSeq.rend(), [time](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() < time;
            }).base();
            imuDataSeq.erase(imuDataSeq.begin(), iter);
        }
        {
            LOCK_RADAR_DATA_SEQ
            auto iter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [time](const RadarTarget::Ptr &tar) {
                return tar->GetTimestamp() < time;
            }).base();
            radarDataSeq.erase(radarDataSeq.begin(), iter);
        }
    }

    const std::list<IMUFrame::Ptr> &DataManager::GetIMUDataSeq() const {
        return imuDataSeq;
    }

    const std::list<RadarTarget::Ptr> &DataManager::GetRadarDataSeq() const {
        return radarDataSeq;
    }

}

