// Copyright (c) 2023. Created on 7/7/23 1:31 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#include "sensor/radar.h"

#include <utility>

namespace ns_river {
    // -----------
    // RadarTarget
    // -----------

    [[nodiscard]] double RadarTarget::GetTimestamp() const {
        return _timestamp;
    }

    void RadarTarget::SetTimestamp(double timestamp) {
        _timestamp = timestamp;
    }

    const Eigen::Vector3d &RadarTarget::GetTargetXYZ() const {
        return _target;
    }

    Eigen::Vector3d RadarTarget::GetTargetRTP() const {
        return ns_ctraj::XYZtoRTP<double>(_target);
    }

    double RadarTarget::GetRadialVelocity() const {
        return _radialVel;
    }

    RadarTarget::Ptr RadarTarget::Create(double timestamp, const Eigen::Vector4d &rawMes) {
        return std::make_shared<RadarTarget>(timestamp, rawMes);
    }

    RadarTarget::RadarTarget(double timestamp, const Eigen::Vector4d &rawMes)
            : _timestamp(timestamp), _target(ns_ctraj::RTPtoXYZ<double>({rawMes(0), rawMes(1), rawMes(2)})),
              _radialVel(rawMes(3)), _range(rawMes(0)), _invRange(1.0 / rawMes(0)) {}

    RadarTarget::Ptr RadarTarget::Create(double timestamp, const Eigen::Vector3d &target, double radialVel) {
        return std::make_shared<RadarTarget>(timestamp, target, radialVel);
    }

    RadarTarget::RadarTarget(double timestamp, Eigen::Vector3d target, double radialVel) :
            _timestamp(timestamp), _target(std::move(target)), _radialVel(radialVel),
            _range(_target.norm()), _invRange(1.0 / _target.norm()) {}

    double RadarTarget::GetRange() const {
        return _range;
    }

    double RadarTarget::GetInvRange() const {
        return _invRange;
    }

    // ----------------
    // RadarTargetArray
    // ----------------

    RadarTargetArray::RadarTargetArray(double timestamp, const std::vector<RadarTarget::Ptr> &targets)
            : _timestamp(timestamp), _targets(targets) {}

    RadarTargetArray::Ptr RadarTargetArray::Create(double timestamp, const std::vector<RadarTarget::Ptr> &targets) {
        return std::make_shared<RadarTargetArray>(timestamp, targets);
    }

    double RadarTargetArray::GetTimestamp() const {
        return _timestamp;
    }

    const std::vector<RadarTarget::Ptr> &RadarTargetArray::GetTargets() const {
        return _targets;
    }

    void RadarTargetArray::SetTimestamp(double timestamp) {
        _timestamp = timestamp;
    }

    bool RadarTargetArray::SaveTargetArraysToDisk(const std::string &filename,
                                                  const std::vector<RadarTargetArray::Ptr> &arrays,
                                                  int precision) {
        std::ofstream file(filename);
        file << std::fixed << std::setprecision(precision);
        cereal::JSONOutputArchive ar(file);
        ar(cereal::make_nvp("radar_arrays", arrays));
        return true;
    }

    Eigen::Vector3d RadarTargetArray::RadarVelocityFromStaticTargetArray(const Sophus::SO3d &SO3_RtoB0) const {
        Eigen::Matrix3d ROT_B0toR = SO3_RtoB0.inverse().matrix();
        Eigen::VectorXd lVec(_targets.size());
        Eigen::MatrixXd BMat(_targets.size(), 3);
        for (int i = 0; i < static_cast<int>(_targets.size()); ++i) {
            const auto &tar = _targets.at(i);
            lVec(i) = tar->GetRadialVelocity() * tar->GetTargetXYZ().norm();
            BMat.block<1, 3>(i, 0) = -tar->GetTargetXYZ().transpose() * ROT_B0toR;
        }
        Eigen::Vector3d xVec = (BMat.transpose() * BMat).inverse() * BMat.transpose() * lVec;
        return xVec;
    }

}
