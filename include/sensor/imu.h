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

#ifndef RIVER_IMU_H
#define RIVER_IMU_H

#include "ctraj/core/imu.h"

namespace ns_river {
    using IMUFrame = ns_ctraj::IMUFrame;

    struct IMUFrameArray {
    public:
        using Ptr = std::shared_ptr<IMUFrameArray>;

    private:
        // the timestamp of this array
        double _timestamp;
        std::vector<IMUFrame::Ptr> _frames;

    public:
        explicit IMUFrameArray(double timestamp = INVALID_TIME_STAMP, const std::vector<IMUFrame::Ptr> &frames = {});

        static IMUFrameArray::Ptr
        Create(double timestamp = INVALID_TIME_STAMP, const std::vector<IMUFrame::Ptr> &frames = {});

        [[nodiscard]] double GetTimestamp() const;

        void SetTimestamp(double timestamp);

        [[nodiscard]] const std::vector<IMUFrame::Ptr> &GetFrames() const;

        // save radar frames sequence to disk
        static bool SaveFramesArraysToDisk(const std::string &filename,
                                           const std::vector<IMUFrameArray::Ptr> &arrays,
                                           int precision = 10);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(cereal::make_nvp("timestamp", _timestamp), cereal::make_nvp("frames", _frames));
        }
    };
}


#endif //RIVER_IMU_H
