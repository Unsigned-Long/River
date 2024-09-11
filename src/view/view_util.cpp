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

#include "view/view_util.h"
#include "tiny-viewer/entity/arrow.h"
#include "tiny-viewer/object/landmark.h"
#include "tiny-viewer/entity/line.h"
#include "tiny-viewer/entity/coordinate.h"

namespace ns_river {
    std::string ViewUtil::DataOutputPath = {};

    void ViewUtil::ShowSO3Spline(const ViewUtil::SplineBundleType::So3SplineType &spline,
                                 double dt, double st, double et) {
        auto config = ns_viewer::ViewerConfigor().WithWinName("SO3 Spline");
        config.output.dataOutputPath = DataOutputPath;
        ns_viewer::Viewer viewer(config);
        for (double t = st; t < et;) {
            auto pose = ns_viewer::Posed(spline.Evaluate(t).matrix(), {(t - spline.MinTime()) * 5.0, 0.0, 0.0}, t);
            viewer.AddEntity(ns_viewer::Coordinate::Create(pose.cast<float>()));
            t += dt;
        }
        viewer.RunInSingleThread();
    }

    void ViewUtil::ShowSO3SplineWithGravity(const ViewUtil::SplineBundleType::So3SplineType &spline,
                                            const Eigen::Vector3d &gravity, double dt, double st, double et) {
        auto config = ns_viewer::ViewerConfigor().WithWinName("SO3 Spline With Gravity");
        config.output.dataOutputPath = DataOutputPath;
        ns_viewer::Viewer viewer(config);
        for (double t = st; t < et;) {
            auto pose = ns_viewer::Posed(spline.Evaluate(t).matrix(), {(t - spline.MinTime()) * 5.0, 0.0, 0.0}, t);
            viewer.AddEntity(ns_viewer::Coordinate::Create(pose.cast<float>()));
            t += dt;
        }
        viewer.AddEntity(ns_viewer::Arrow::Create(
                gravity.cast<float>().normalized(), {0.0, 0.0, 0.0}, ns_viewer::Colour::Blue(), 4.0f
        ));
        viewer.RunInSingleThread();
    }

    void ViewUtil::ShowSO3VelSplineWithGravity(const ViewUtil::SplineBundleType::So3SplineType &so3Spline,
                                               const ViewUtil::SplineBundleType::RdSplineType &velSpline,
                                               const Eigen::Vector3d &gravity, double dt, double st, double et) {
        auto config = ns_viewer::ViewerConfigor().WithWinName("SO3 & Vel Splines With Gravity");
        config.output.dataOutputPath = DataOutputPath;
        ns_viewer::Viewer viewer(config);
        for (double t = st; t < et;) {
            auto pose = ns_viewer::Posed(so3Spline.Evaluate(t).matrix(), velSpline.Evaluate(t), t);
            viewer.AddEntity(ns_viewer::Coordinate::Create(pose.cast<float>(), 0.05f));
            t += dt;
        }
        const auto &knots = velSpline.GetKnots();
        for (int i = 0; i < static_cast<int>(knots.size()) - 1; ++i) {
            int j = i + 1;
            const Eigen::Vector3d &ki = knots.at(i);
            const Eigen::Vector3d &kj = knots.at(j);
            viewer.AddEntity(
                    ns_viewer::Landmark::Create(ki.cast<float>(), 0.01f, ns_viewer::Colour::Black())
            );
            viewer.AddEntity(
                    ns_viewer::Line::Create(ki.cast<float>(), kj.cast<float>(), ns_viewer::Colour::Black())
            );
        }
        viewer.AddEntity(
                ns_viewer::Landmark::Create(knots.back().cast<float>(), 0.01f, ns_viewer::Colour::Black())
        );

        viewer.AddEntity(ns_viewer::Arrow::Create(
                gravity.cast<float>().normalized(), {0.0, 0.0, 0.0}, ns_viewer::Colour::Blue(), 4.0f
        ));
        viewer.RunInSingleThread();
    }
}