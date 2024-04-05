// Copyright (c) 2023. Created on 10/19/23 7:55 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIVER_DATA_MANAGER_H
#define RIVER_DATA_MANAGER_H

#include "config/configor.h"
#include "sensor/imu_data_loader.h"
#include "sensor/radar_data_loader.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "core/status.h"

namespace ns_river {
    // unique locks for mutexes
#define LOCK_IMU_DATA_SEQ std::unique_lock<std::mutex> imuDataLock(DataManager::IMUDataSeqMutex);
#define LOCK_RADAR_DATA_SEQ std::unique_lock<std::mutex> radarDataLock(DataManager::RadarDataSeqMutex);
#define LOCK_RADAR_INIT_FRAMES std::unique_lock<std::mutex> radarInitFramesLock(DataManager::RadarInitFramesMutex);

    class DataManager {
    public:
        using Ptr = std::shared_ptr<DataManager>;

    private:
        // ros node handler used for creating subscribers, i.e., 'imuSuber' and 'radarSuber'
        ros::NodeHandlePtr handler;
        Configor::Ptr configor;

        // ros subscribers to receive imu and radar data
        ros::Subscriber imuSuber;
        ros::Subscriber radarSuber;

        // containers to store sensor data
        std::list<IMUFrame::Ptr> imuDataSeq;
        std::list<RadarTarget::Ptr> radarDataSeq;

        // the radar target array for initialization, size: 10
        std::list<RadarTargetArray::Ptr> radarTarAryForInit;

        std::optional<double> RIVER_TIME_EPOCH;

    public:
        // mutexes employed in multi-thread framework
        static std::mutex IMUDataSeqMutex;
        static std::mutex RadarDataSeqMutex;
        static std::mutex RadarInitFramesMutex;

    public:
        explicit DataManager(const ros::NodeHandlePtr &handler, const Configor::Ptr &configor);

        static Ptr Create(const ros::NodeHandlePtr &handler, const Configor::Ptr &configor);

        [[nodiscard]] const std::optional<double> &GetRiverTimeEpoch() const;

        [[nodiscard]] double GetEndTimeSafely() const;

        [[nodiscard]] double GetIMUEndTimeSafely() const;

        [[nodiscard]] double GetRadarEndTimeSafely() const;

        const std::list<IMUFrame::Ptr> &GetIMUDataSeq() const;

        const std::list<RadarTarget::Ptr> &GetRadarDataSeq() const;

        void ShowDataStatus() const;

        [[nodiscard]] std::list<RadarTargetArray::Ptr> GetRadarTarAryForInitSafely() const;

        std::list<IMUFrame::Ptr> ExtractIMUDataPieceSafely(double start, double end);

        std::list<IMUFrame::Ptr> ExtractIMUDataPieceSafely(double start);

        std::list<RadarTarget::Ptr> ExtractRadarDataPieceSafely(double start, double end);

        std::list<RadarTarget::Ptr> ExtractRadarDataPieceSafely(double start);

        static std::pair<Eigen::Vector3d, Eigen::Matrix3d> AcceMeanVar(const std::list<IMUFrame::Ptr> &data);

        static std::pair<Eigen::Vector3d, Eigen::Matrix3d> GyroMeanVar(const std::list<IMUFrame::Ptr> &data);

        void EraseOldDataPieceSafely(double time);

    protected:
        template<class IMUMsgType>
        void HandleIMUMessage(const typename IMUMsgType::ConstPtr &msg) {
            auto frame = IMUDataUnpacker::Unpack(msg);
            // try to initialize the time epoch of River
            if (RIVER_TIME_EPOCH == std::nullopt) {
                RIVER_TIME_EPOCH = frame->GetTimestamp();
                spdlog::info("time epoch of River initialized by imu frame: {:.6f}", *RIVER_TIME_EPOCH);
            }
            frame->SetTimestamp(frame->GetTimestamp() - *RIVER_TIME_EPOCH);
            {
                LOCK_IMU_DATA_SEQ
                imuDataSeq.push_back(frame);
            }
        }

        template<class RadarMsgType>
        void HandleRadarMessage(const typename RadarMsgType::ConstPtr &msg) {
            auto targetAry = RadarDataUnpacker::Unpack(msg);
            // try to initialize the time epoch of River
            if (RIVER_TIME_EPOCH == std::nullopt) {
                RIVER_TIME_EPOCH = targetAry->GetTimestamp();
                spdlog::info("time epoch of River initialized by radar targetAry: {:.6f}", *RIVER_TIME_EPOCH);
            }
            // aligned the time of targetAry (i.e., radar time) to IMU time
            targetAry->SetTimestamp(
                    targetAry->GetTimestamp() - *RIVER_TIME_EPOCH + configor->dataStream.CalibParam.TIME_OFFSET_RtoB
            );
            for (auto &tar: targetAry->GetTargets()) {
                tar->SetTimestamp(
                        tar->GetTimestamp() - *RIVER_TIME_EPOCH + configor->dataStream.CalibParam.TIME_OFFSET_RtoB
                );
            }
            {
                LOCK_RADAR_DATA_SEQ
                radarDataSeq.insert(
                        radarDataSeq.end(), targetAry->GetTargets().cbegin(), targetAry->GetTargets().cend()
                );
            }
            auto status = RiverStatus::GetStatusPackSafely();
            if (!RiverStatus::IsWith(RiverStatus::StateManager::Status::HasInitialized, status.StateMagr)) {
                OrganizeRadarTarAryForInit(targetAry);
            } else {
                {
                    LOCK_RADAR_INIT_FRAMES
                    radarTarAryForInit.clear();
                }
                if (RiverStatus::IsWith(RiverStatus::DataManager::Status::RadarTarAryForInitIsReady, status.DataMagr)) {
                    LOCK_RIVER_STATUS
                    RiverStatus::DataManager::CurStatus ^= RiverStatus::DataManager::Status::RadarTarAryForInitIsReady;
                }
            }
        }

        inline void OrganizeRadarTarAryForInit(const RadarTargetArray::Ptr &rawTarAry);
    };

}


#endif //RIVER_DATA_MANAGER_H
