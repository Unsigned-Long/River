// Copyright (c) 2023. Created on 10/21/23 7:32 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIVER_VIEW_UTIL_H
#define RIVER_VIEW_UTIL_H

#include "tiny-viewer/core/viewer.h"
#include "ctraj/core/spline_bundle.h"
#include "config/configor.h"

namespace ns_river {
    struct ViewUtil {
    public:
        using SplineBundleType = ns_ctraj::SplineBundle<Configor::Prior::SplineOrder>;

        static std::string DataOutputPath;

    public:
        static void ShowSO3Spline(const SplineBundleType::So3SplineType &spline, double dt, double st, double et);

        static void ShowSO3SplineWithGravity(const SplineBundleType::So3SplineType &spline,
                                             const Eigen::Vector3d &gravity, double dt, double st, double et);

        static void ShowSO3VelSplineWithGravity(const SplineBundleType::So3SplineType &so3Spline,
                                                const SplineBundleType::RdSplineType &velSpline,
                                                const Eigen::Vector3d &gravity, double dt, double st, double et);
    };
}

#endif //RIVER_VIEW_UTIL_H
