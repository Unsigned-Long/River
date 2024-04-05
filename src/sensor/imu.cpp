// Copyright (c) 2023. Created on 7/7/23 1:31 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#include "sensor/imu.h"

namespace ns_river {
    IMUFrameArray::IMUFrameArray(double timestamp, const std::vector<IMUFrame::Ptr> &frames)
            : _timestamp(timestamp), _frames(frames) {}

    IMUFrameArray::Ptr IMUFrameArray::Create(double timestamp, const std::vector<IMUFrame::Ptr> &frames) {
        return std::make_shared<IMUFrameArray>(timestamp, frames);
    }

    double IMUFrameArray::GetTimestamp() const {
        return _timestamp;
    }

    void IMUFrameArray::SetTimestamp(double timestamp) {
        _timestamp = timestamp;
    }

    const std::vector<IMUFrame::Ptr> &IMUFrameArray::GetFrames() const {
        return _frames;
    }

    bool IMUFrameArray::SaveFramesArraysToDisk(const std::string &filename,
                                               const std::vector<IMUFrameArray::Ptr> &arrays,
                                               int precision) {
        std::ofstream file(filename);
        file << std::fixed << std::setprecision(precision);
        cereal::JSONOutputArchive ar(file);
        ar(cereal::make_nvp("imu_arrays", arrays));
        return true;
    }
}
