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

#include "view/viewer.h"
#include <utility>
#include "ros/ros.h"
#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include "util/sampling.hpp"
#include <vtkOBJReader.h>
#include "tiny-viewer/object/imu.h"
#include "tiny-viewer/object/landmark.h"
#include "tiny-viewer/entity/arrow.h"
#include "tiny-viewer/entity/line.h"

namespace ns_river {
    std::string Viewer::VR_WIN_NAME = "VR_WIN_NAME";
    std::string Viewer::SPLINE_WIN_NAME = "SPLINE_WIN_NAME";

    Viewer::Viewer(const ns_viewer::MultiViewerConfigor &viewerConfigor, Configor::Ptr configor,
                   StateManager::Ptr stateManager)
            : ns_viewer::MultiViewer(viewerConfigor), configor(std::move(configor)), stateMagr(std::move(stateManager)),
              lastEntities() {
        lastEntities.insert({VR_WIN_NAME, {}});
        lastEntities.insert({SPLINE_WIN_NAME, {}});

        // if load ply file fails, then create virtual markers on a ball
        if (!InitSceneFromObj(this->configor->preference.ObjFileForDisplay, 800)) {
            InitVirtualMarkers(500, 10.0);
        }
    }

    Viewer::Ptr Viewer::Create(const Configor::Ptr &configor, const StateManager::Ptr &stateManager) {
        ns_viewer::MultiViewerConfigor viewerConfigor({VR_WIN_NAME, SPLINE_WIN_NAME}, "River");
        // config the output path
        viewerConfigor.output.dataOutputPath = configor->dataStream.OutputPath;
        viewerConfigor.camera.at(VR_WIN_NAME).fx = 300;
        viewerConfigor.camera.at(VR_WIN_NAME).fy = 300;
        viewerConfigor.camera.at(SPLINE_WIN_NAME).fx = 300;
        viewerConfigor.camera.at(SPLINE_WIN_NAME).fy = 300;
        return std::make_shared<Viewer>(viewerConfigor, configor, stateManager);
    }

    void Viewer::RunViewer() {
        this->RunInMultiThread();
        // waiting for the viewer active
        // if (!this->WaitForActive(5000)) {
        //     spdlog::warn("the viewer is still inactive after waiting for 5000 (ms), try to quit 'river'...");
        //     LOCK_RIVER_STATUS
        //     RiverStatus::StateManager::CurStatus |= RiverStatus::StateManager::Status::ShouldQuit;
        // }
        while (!this->IsActive()) {
            spdlog::warn("waiting multi-thread tiny-viewer active...");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        spdlog::info("'Viewer::RunViewer' has been booted, thread id: {}.", RIVER_TO_STR(std::this_thread::get_id()));

        ros::Rate rate(configor->preference.IncrementalOptRate);
        bool hasNotInit = true;

        while (ros::ok() && this->IsActive()) {
            auto status = RiverStatus::GetStatusPackSafely();
            if (!RiverStatus::IsWith(RiverStatus::StateManager::Status::NewStateNeedToDraw, status.StateMagr)) {
                continue;
            }
            if (hasNotInit) {
                InitializeViewer(status);
                hasNotInit = false;
            }

            // update the vr window
            UpdateVRWindow(status);

            // update the right window
            UpdateSplineWindow(status);

            {
                LOCK_RIVER_STATUS
                RiverStatus::StateManager::CurStatus ^= RiverStatus::StateManager::Status::NewStateNeedToDraw;
            }
            // sleep and spin once
            rate.sleep();
            ros::spinOnce();
        }

        spdlog::warn("'Viewer::RunViewer' quits normally.");

        LOCK_RIVER_STATUS
        RiverStatus::StateManager::CurStatus |= RiverStatus::StateManager::Status::ShouldQuit;
    }

    void Viewer::InitializeViewer(const RiverStatus::StatusPack &status) {
        // initialize the camera view for spline window
        const auto &c = _configor.camera.at(SPLINE_WIN_NAME);
        auto renderState = pangolin::OpenGlRenderState(
                pangolin::ProjectionMatrix(c.width, c.height, c.fx, c.fy, c.cx, c.cy, c.near, c.far),
                pangolin::ModelViewLookAt(2.0f, 2.0f, 2.0f, ExpandStdVec3(c.initViewPoint), pangolin::AxisZ));
        this->SetCamView(renderState, SPLINE_WIN_NAME);

        // draw the gravity
        auto gravity = stateMagr->GetStatePackSafely(
                status.ValidStateEndTime - Configor::Preference::StatePublishDelay
        )->gravity;
        auto gravityEntity = ns_viewer::Arrow::Create(
                gravity.normalized().cast<float>(), {0.0, 0.0, 0.0}, ns_viewer::Colour::Blue(), 4.0f
        );
        this->AddEntity(gravityEntity, VR_WIN_NAME);
        this->AddEntity(gravityEntity, SPLINE_WIN_NAME);

        // draw markers
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(new pcl::PointCloud<pcl::PointXYZRGB>());
        points->resize(virMarkers.size());
        for (int i = 0; i < static_cast<int>(virMarkers.size()); ++i) {
            const auto &m = virMarkers.at(i);

            points->at(i).x = static_cast<float>(m.first(0));
            points->at(i).y = static_cast<float>(m.first(1));
            points->at(i).z = static_cast<float>(m.first(2));
            points->at(i).r = static_cast<std::uint32_t>(m.second.r * 255.0f);
            points->at(i).g = static_cast<std::uint32_t>(m.second.g * 255.0f);
            points->at(i).b = static_cast<std::uint32_t>(m.second.b * 255.0f);
        }
        this->AddEntity(ns_viewer::Cloud<pcl::PointXYZRGB>::Create(points, 15.0f), VR_WIN_NAME);

    }

    void Viewer::UpdateSplineWindow(const RiverStatus::StatusPack &status) {
        // remove old entities in the spline window
        this->RemoveEntity(lastEntities.at(SPLINE_WIN_NAME), SPLINE_WIN_NAME);

        // visualization for two seconds
        const double eTime = status.ValidStateEndTime - Configor::Preference::StatePublishDelay, sTime = eTime - 2.0;

        std::vector<ns_viewer::Entity::Ptr> curEntities;

        // -----------
        // coordinates
        // -----------
        const double dt = 0.01;
        std::vector<double> times;
        times.reserve(static_cast<std::size_t>((eTime - sTime) / dt + 1));
        for (double t = sTime; t < eTime;) {
            times.push_back(t);
            t += dt;
        }
        // for each obtained states
        auto statePacks = stateMagr->GetStatePackSafely(times);
        for (const auto &pack: statePacks) {
            if (pack == std::nullopt) { continue; }
            ns_viewer::Posef pose;
            if (configor->preference.VisualizeVelocityInBody) {
                // visualization in-body linear velocity rather than world-frame ones
                pose = ns_viewer::Posed(pack->SO3_CurToRef.matrix(), pack->LIN_VEL_CurToRefInCur()).cast<float>();
            } else {
                pose = ns_viewer::Posed(pack->SO3_CurToRef.matrix(), pack->LIN_VEL_CurToRefInRef).cast<float>();
            }
            // time span from sTime to eTime, size from 0.01f to 0.15f
            double s = 0.01 + 0.14 * (pack->timestamp - sTime) / (eTime - sTime);
            curEntities.push_back(ns_viewer::Coordinate::Create(pose, static_cast<float>(s)));
        }

        // --------------
        // velocity knots
        // --------------

        if (configor->preference.VisualizeVelocityInBody) {
            LOCK_STATES
            const auto &velSpline = stateMagr->GetSplines()->GetRdSpline(Configor::Preference::VelSpline);
            const auto &knots = velSpline.GetKnots();
            auto knotSize = knots.size();
            int num = std::max(0, static_cast<int>(
                    static_cast<double>(knotSize) - 20.0 / configor->prior.VelSplineKnotDist)
            );

            const auto &so3Spline = stateMagr->GetSplines()->GetSo3Spline(Configor::Preference::SO3Spline);
            std::vector<Sophus::SO3d> so3Vec(knotSize);
            std::size_t lastKnotIdx = 0;
            for (std::size_t i = num; i < knotSize - 1; ++i) {
                double t = velSpline.MinTime() + velSpline.GetTimeInterval() * (double) i;
                if (so3Spline.TimeStampInRange(t)) {
                    so3Vec.at(i) = so3Spline.Evaluate(t).inverse();
                    lastKnotIdx = i;
                }
            }

            for (std::size_t i = num; i < lastKnotIdx; ++i) {
                // in-body knots
                const Eigen::Vector3d &ki = so3Vec.at(i) * knots.at(i);
                const Eigen::Vector3d &kj = so3Vec.at(i + 1) * knots.at(i + 1);
                // lines between knots
                curEntities.push_back(
                        ns_viewer::Line::Create(ki.cast<float>(), kj.cast<float>(), ns_viewer::Colour::Black())
                );
                // knots
                curEntities.push_back(
                        ns_viewer::Landmark::Create(ki.cast<float>(), 0.01f, ns_viewer::Colour::Black())
                );
            }
        } else {
            LOCK_STATES
            const auto &knots = stateMagr->GetSplines()->GetRdSpline(Configor::Preference::VelSpline).GetKnots();
            auto knotSize = knots.size();
            int num = std::max(0, static_cast<int>(
                    static_cast<double>(knotSize) - 20.0 / configor->prior.VelSplineKnotDist)
            );
            for (std::size_t i = num; i < knotSize - 1; ++i) {
                const auto &ki = knots.at(i);
                const auto &kj = knots.at(i + 1);
                // lines between knots
                curEntities.push_back(
                        ns_viewer::Line::Create(ki.cast<float>(), kj.cast<float>(), ns_viewer::Colour::Black())
                );
                // knots
                curEntities.push_back(
                        ns_viewer::Landmark::Create(ki.cast<float>(), 0.01f, ns_viewer::Colour::Black())
                );
            }
            // the last knot
            curEntities.push_back(
                    ns_viewer::Landmark::Create(knots.back().cast<float>(), 0.01f, ns_viewer::Colour::Black())
            );
        }


        // record the indexes for entities-removing in the next time
        auto &curEntityIds = lastEntities.at(SPLINE_WIN_NAME);
        curEntityIds.clear();
        curEntityIds.resize(curEntities.size());
        for (int i = 0; i < static_cast<int>(curEntities.size()); ++i) {
            curEntityIds.at(i) = curEntities.at(i)->GetId();
        }

        this->AddEntity(curEntities, SPLINE_WIN_NAME);
    }

    void Viewer::UpdateVRWindow(const RiverStatus::StatusPack &status) {
        // remove old entities in the spline window
        this->RemoveEntity(lastEntities.at(VR_WIN_NAME), VR_WIN_NAME);

        std::vector<ns_viewer::Entity::Ptr> curEntities;

        // ---------------
        // draw body frame
        // ---------------
        auto newState = stateMagr->GetStatePackSafely(
                status.ValidStateEndTime - Configor::Preference::StatePublishDelay
        );
        const auto &body = newState->SO3_CurToRef;
        auto pose = ns_viewer::Posed(body.matrix(), {0.0, 0.0, 0.0}).cast<float>();
        curEntities.push_back(ns_viewer::IMU::Create(pose, 0.08f));

        // ---------------------
        // follow the body frame
        // ---------------------
        // x -> z, -y -> x, -z -> y
        Eigen::Matrix3f camRot;
        camRot.col(2) = pose.rotation.col(0);
        camRot.col(0) = -pose.rotation.col(1);
        camRot.col(1) = -pose.rotation.col(2);
        Eigen::Vector3f bias = -camRot.col(2) * 0.5f;
        this->SetCamView(ns_viewer::Posef(camRot, bias), VR_WIN_NAME);

        // ---------------------------
        // draw velocities for markers
        // ---------------------------
        Eigen::Vector3d ANG_VEL_CurToWinW, LIN_VEL_CurToWinW;
        {
            LOCK_STATES
            const auto &splines = stateMagr->GetSplines();

            const auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
            ANG_VEL_CurToWinW = newState->SO3_CurToRef * so3Spline.VelocityBody(
                    status.ValidStateEndTime - Configor::Preference::StatePublishDelay
            );

            const auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
            LIN_VEL_CurToWinW = velSpline.Evaluate(status.ValidStateEndTime - Configor::Preference::StatePublishDelay);
        }
        auto SO3_WtoCur = newState->SO3_CurToRef.inverse();
        for (const auto &[p, c]: virMarkers) {
            Eigen::Vector3d pInBody = SO3_WtoCur * p;
            Eigen::Vector3d v = Sophus::SO3d::hat(newState->SO3_CurToRef * pInBody) * ANG_VEL_CurToWinW;
            // Eigen::Vector3d pVelInBody = SO3_WtoCur * (v - LIN_VEL_CurToWinW);
            Eigen::Vector3d pVelInW = (v - LIN_VEL_CurToWinW);

            curEntities.push_back(
                    ns_viewer::Line::Create(p.cast<float>(), (p + pVelInW * 0.2f).cast<float>(), c, 3.0f)
            );
        }

        // record the indexes for entities-removing in the next time
        auto &curEntityIds = lastEntities.at(VR_WIN_NAME);
        curEntityIds.clear();
        curEntityIds.resize(curEntities.size());
        for (int i = 0; i < static_cast<int>(curEntities.size()); ++i) {
            curEntityIds.at(i) = curEntities.at(i)->GetId();
        }

        this->AddEntity(curEntities, VR_WIN_NAME);
    }

    void Viewer::InitVirtualMarkers(int n, double r) {
        const double phi = (std::sqrt(5.0) - 1.0) * 0.5;
        virMarkers = std::vector<std::pair<Eigen::Vector3d, ns_viewer::Colour>>(n);
        for (int i = 1; i < n + 1; ++i) {
            auto z = (2.0 * i - 1.0) / n - 1;
            auto x = std::sqrt(1.0 - z * z) * std::cos(2.0 * M_PI * i * phi);
            auto y = std::sqrt(1.0 - z * z) * std::sin(2.0 * M_PI * i * phi);
            Eigen::Vector3d(x * r, y * r, z * r);
            virMarkers.at(i - 1) = {Eigen::Vector3d(x * r, y * r, z * r), ns_viewer::Entity::GetUniqueColour()};
        }
    }

    bool Viewer::InitSceneFromObj(const std::string &plyFilename, int n) {
        if (!std::filesystem::exists(plyFilename)) {
            return false;
        }

        std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<float> grayUniformDist(0.0f, 1.0f);

        // Create a PLY reader
        vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
        reader->SetFileName(plyFilename.c_str());
        reader->Update();

        // Get the output polydata
        vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
        polydata->ShallowCopy(reader->GetOutput());

        // add point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pointCloud->resize(polydata->GetNumberOfPoints());

        vtkSmartPointer<vtkPoints> points = polydata->GetPoints();
        auto virMarkersTemp = std::vector<std::pair<Eigen::Vector3d, ns_viewer::Colour>>(polydata->GetNumberOfPoints());
        for (vtkIdType i = 0; i < points->GetNumberOfPoints(); i++) {
            double point[3];
            points->GetPoint(i, point);
            pointCloud->at(i).x = static_cast<float>(point[0]);
            pointCloud->at(i).y = static_cast<float>(point[1]);
            pointCloud->at(i).z = static_cast<float>(point[2]);

            virMarkersTemp.at(i).first = Eigen::Vector3d(ExpandAryVec3(point));
            float gray = grayUniformDist(engine);
            virMarkersTemp.at(i).second = ns_viewer::Colour(gray, gray, gray, 0.8f);
        }
        this->RemoveEntity(VR_WIN_NAME);
        this->AddEntity(ns_viewer::Coordinate::Create(ns_viewer::Posef()), VR_WIN_NAME);
        this->AddEntity(
                ns_viewer::Cloud<pcl::PointXYZ>::Create(pointCloud, 10.0f, ns_viewer::Colour::White()), VR_WIN_NAME
        );

        // create virtual markers
        virMarkers = SamplingWoutReplace2(
                engine, virMarkersTemp, std::min(n, static_cast<int>(pointCloud->size()))
        );

        // add polygons
        // vtkSmartPointer<vtkCellArray> polygons = polydata->GetPolys();
        // vtkIdType ptNum, *ptIdAry;
        // for (polygons->InitTraversal(); polygons->GetNextCell(ptNum, ptIdAry);) {
        //     std::vector<Eigen::Vector3f> verts(ptNum);
        //     for (vtkIdType i = 0; i < ptNum; i++) {
        //         double point[3];
        //         points->GetPoint(ptIdAry[i], point);
        //         verts.at(i) = Eigen::Vector3d(ExpandAryVec3(point)).cast<float>();
        //     }
        //     float gray = grayUniformDist(engine);
        //     this->AddEntity(
        //             ns_viewer::Polygon::Create(verts, true, ns_viewer::Colour(gray, gray, gray, 0.8f)), VR_WIN_NAME
        //     );
        // }

        // add obj data
        this->AddObjEntity(plyFilename, VR_WIN_NAME);
        return true;
    }
}