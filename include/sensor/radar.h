// Copyright (c) 2023. Created on 7/7/23 1:29 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIVER_RADAR_H
#define RIVER_RADAR_H

#include "memory"
#include "util/utils.hpp"
#include "ctraj/utils/macros.hpp"
#include "ctraj/utils/utils.hpp"

namespace ns_river {

    struct RadarTarget {
    public:
        using Ptr = std::shared_ptr<RadarTarget>;

    private:
        // the timestamp of this frame
        double _timestamp;

        Eigen::Vector3d _target;
        double _radialVel;

        double _range, _invRange;

    public:
        /**
         * @attention rawMes: [ range | theta | phi | target radial vel with respect to radar in frame {R} ]
         */
        explicit RadarTarget(double timestamp = INVALID_TIME_STAMP,
                             const Eigen::Vector4d &rawMes = Eigen::Vector4d::Zero());

        /**
         * @attention rawMes: [ xyz | target radial vel with respect to radar in frame {R} ]
         */
        explicit RadarTarget(double timestamp, Eigen::Vector3d target, double radialVel);

        /**
         * @attention rawMes: [ range | theta | phi | target radial vel with respect to radar in frame {R} ]
         */
        static RadarTarget::Ptr Create(double timestamp = INVALID_TIME_STAMP,
                                       const Eigen::Vector4d &rawMes = Eigen::Vector4d::Zero());

        /**
         * @attention rawMes: [ xyz | target radial vel with respect to radar in frame {R} ]
         */
        static RadarTarget::Ptr Create(double timestamp, const Eigen::Vector3d &target, double radialVel);

        // access
        [[nodiscard]] double GetTimestamp() const;

        void SetTimestamp(double timestamp);

        [[nodiscard]] const Eigen::Vector3d &GetTargetXYZ() const;

        [[nodiscard]]  Eigen::Vector3d GetTargetRTP() const;

        [[nodiscard]]  double GetRadialVelocity() const;

        [[nodiscard]] double GetRange() const;

        [[nodiscard]] double GetInvRange() const;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(
                    cereal::make_nvp("timestamp", _timestamp),
                    cereal::make_nvp("target", _target),
                    cereal::make_nvp("radial_vel", _radialVel)
            );
        }
    };

    struct RadarTargetArray {
    public:
        using Ptr = std::shared_ptr<RadarTargetArray>;

    private:
        // the timestamp of this array
        double _timestamp;
        std::vector<RadarTarget::Ptr> _targets;

    public:
        explicit RadarTargetArray(double timestamp = INVALID_TIME_STAMP,
                                  const std::vector<RadarTarget::Ptr> &targets = {});

        static RadarTargetArray::Ptr
        Create(double timestamp = INVALID_TIME_STAMP, const std::vector<RadarTarget::Ptr> &targets = {});

        [[nodiscard]] double GetTimestamp() const;

        void SetTimestamp(double timestamp);

        [[nodiscard]] const std::vector<RadarTarget::Ptr> &GetTargets() const;

        // save radar frames sequence to disk
        static bool SaveTargetArraysToDisk(const std::string &filename,
                                           const std::vector<RadarTargetArray::Ptr> &arrays,
                                           int precision = 10);

        Eigen::Vector3d RadarVelocityFromStaticTargetArray(const Sophus::SO3d &SO3_RtoB0) const;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(cereal::make_nvp("timestamp", _timestamp), cereal::make_nvp("targets", _targets));
        }
    };
}


#endif //RIVER_RADAR_H
