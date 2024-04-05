// Copyright (c) 2023. Created on 10/21/23 7:33 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#include "view/view_util.h"

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