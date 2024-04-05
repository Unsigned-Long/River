// Copyright (c) 2023. Created on 10/21/23 12:28 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.


#include "core/status.h"

namespace ns_river {
    // ------------------------
    // static initialized filed
    // ------------------------
    std::mutex RiverStatus::StatusMutex = {};

    RiverStatus::StateManager::Status RiverStatus::StateManager::CurStatus = RiverStatus::StateManager::Status::NONE;
    double RiverStatus::StateManager::ValidStateEndTime = -1.0;

    RiverStatus::DataManager::Status RiverStatus::DataManager::CurStatus = RiverStatus::DataManager::Status::NONE;

    RiverStatus::StatusPack::StatusPack(RiverStatus::StateManager::Status stateMagr, double ValidStateEndTime,
                                        RiverStatus::DataManager::Status dataMagr)
            : StateMagr(stateMagr), ValidStateEndTime(ValidStateEndTime), DataMagr(dataMagr) {}

    RiverStatus::StatusPack RiverStatus::GetStatusPackSafely() {
        LOCK_RIVER_STATUS
        return {StateManager::CurStatus, StateManager::ValidStateEndTime, DataManager::CurStatus};
    }
}