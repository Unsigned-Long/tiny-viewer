// Tiny-Viewer: Tiny But Powerful Graphic Entity And Object Visualization
// Copyright 2024, the School of Geodesy and Geomatics (SGG), Wuhan University, China
// https://github.com/Unsigned-Long/tiny-viewer.git
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

#include "tiny-viewer/core/viewer.h"
#include "pcl/io/pcd_io.h"
#include "tiny-viewer/core/multi_viewer.h"
#include "pangolin/handler/handler.h"
#include "tiny-viewer/entity/arrow.h"
#include "tiny-viewer/entity/cone.h"
#include "tiny-viewer/entity/coordinate.h"
#include "tiny-viewer/entity/cube.h"
#include "tiny-viewer/entity/cylinder.h"
#include "tiny-viewer/entity/line.h"
#include "tiny-viewer/entity/point_cloud.hpp"
#include "tiny-viewer/entity/polygon.h"
#include "tiny-viewer/entity/path.h"
#include "tiny-viewer/object/camera.h"
#include "tiny-viewer/object/imu.h"
#include "tiny-viewer/object/lidar.h"
#include "tiny-viewer/object/plane.h"
#include "tiny-viewer/object/surfel.h"
#include "tiny-viewer/object/aligned_cloud.hpp"
#include "tiny-viewer/object/radar.h"
#include "tiny-viewer/object/landmark.h"
#include "tiny-viewer/core/pose.hpp"
#include "tiny-viewer/entity/circle.h"

void TEST_ENTITIES() {
    try {
        using namespace ns_viewer;
        ViewerConfigor().SaveConfigure("/home/csl/CppWorks/artwork/tiny-viewer/config/config.json");

        // viewer
        Viewer viewer("/home/csl/CppWorks/artwork/tiny-viewer/config/config.json");

        viewer.GetConfigor().callBacks.insert(
            {'a', [&viewer]() {
                 viewer.AddEntity(Path::Create(
                     "M 3 3 3 l 0 0 -1 s 0 0 -0.5 0.5 0 0 0.5 0 0 0 0 0.5 l 0 0 1 "
                     "m 0.25 0 0 l 0 0 -1 s 0 0 -0.5 0.5 0 0 l 0.25 0 0 "
                     "m 0.5 0 0 s -0.5 0 0 0 0 0.5 0 0 0.5 0.5 0 0 0.5 0 0 0 0 -0.5 0 0 -0.5 -0.5 "
                     "0 0 "
                     "m 0.5 0 0.75 l 0 0 -0.75 s 0 0 0.5 0 0 0.5 0.4 0 0 0.4 0 0 0 0 -0.5 0 0 -0.5 "
                     "m 1 0 0.5 s 0 0 -0.5 -0.5 0 0 -0.5 0 0 0 0 0.5 0 0 0.5 0.5 0 0 0.5 0 0 0 0 "
                     "-0.5 "
                     "s 0 0 -0.75 0 0 -0.5 -0.25 0 0 -0.5 0 0 0 0 0.25 "
                     "m 1 0 1.25 s 0 0 0.5 0.5 0 0 0.5 0 0 0 0 -0.5 -0.5 0 -0.5 -0.5 0 -0.25 l "
                     "0.75 0 0"));
             }});

        // add entities
        viewer.AddEntity(
            Line::Create({1.0, 1.0, 1.0}, {3.0, 4.0, 5.0}, Colour::Red().WithAlpha(0.3f)));

        viewer.AddEntity(Cube::Create(Posef::Random(3.0f), false, Colour::Blue().WithAlpha(0.3f)));
        viewer.AddEntity(Cube::Create(Posef::Random(3.0f), true));

        viewer.AddEntity(Plane::Create(Posef::Random(2.0f), 0.5f, 0.3f, false,
                                       Entity::GetUniqueColour().WithAlpha(0.3f)));
        viewer.AddEntity(Plane::Create(Posef::Random(1.0f), 0.5f, 0.3f, true));

        viewer.AddEntity(Coordinate::Create(Posef::Random(5.0), 0.5f));

        viewer.AddEntity(Cylinder::Create(Posef::Random(3.0f), 1.0, 0.5f));

        viewer.AddEntity(Cloud<pcl::PointXYZRGBA>::Random(1.0f, 200, {1.0f, 2.0f, 3.0f}));
        viewer.AddEntity(Cloud<pcl::PointXYZRGB>::Random(1.0f, 200, {3.0f, 3.0f, 3.0f}));
        viewer.AddEntity(Cloud<pcl::PointXYZ>::Random(0.5f, 200, {3.0f, 2.0f, 1.0f}));
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile("/home/csl/CppWorks/artwork/tiny-viewer/data/scan.pcd", *cloud);
        viewer.AddEntity(Cloud<pcl::PointXYZI>::Create(cloud, 2.0f));
        viewer.AddEntity(AlignedCloud<pcl::PointXYZI>::Random(2.0f, 2000, {-3.0f, 2.0f, -3.0f},
                                                              {0.0f, 0.0f, -1.0f}));

        viewer.AddEntity(IMU::Create(Posef::Random(3.0f)));
        viewer.AddEntity(IMU::Create(Posef::Random(3.0f)));

        viewer.AddEntity(Camera::Create(Posef::Random(3.0f)));
        viewer.AddEntity(CubeCamera::Create(Posef::Random(3.0f)));

        std::vector<Entity::Ptr> entities;
        for (int i = 0; i < 5; ++i) {
            entities.push_back(LiDAR::Create(Posef::Random(5.0f)));
            entities.push_back(LivoxLiDAR::Create(Posef::Random(5.0f)));
        }
        auto ids = viewer.AddEntity(entities);

        viewer.AddEntity(Arrow::Create({4.0f, 2.0f, 2.0f}, {4.0f, 2.0f, 5.0f}));

        viewer.AddEntity(Cone::Create(Posef::Random(5.0f), 0.3f, M_PI_4));

        viewer.AddEntity(Polygon::Create({{0.0f, 0.0f, 2.0f},
                                          {0.0f, 1.0f, 3.0f},
                                          {1.0f, 2.0f, 2.0f},
                                          {2.0f, 3.0f, 1.0f},
                                          {3.0f, 1.0f, 2.0f}},
                                         false, Colour::Black().WithAlpha(0.2f)));

        for (int i = 0; i < 10; ++i) {
            viewer.AddEntity(Surfel::Random(5.0f));
            viewer.AddEntity(Circle::Create(Posef::Random(5.0), 0.5f, true, true,
                                            Entity::GetUniqueColour(), pangolin::AxisZ));
        }

        viewer.AddEntity(Radar::Create(Posef::Random(5.0f)));

        viewer.AddEntity(Cloud<Landmark>::Random(5.0, 20, {4, 5, 4}));
        // show (multi thread)
        viewer.RunInMultiThread();

        // std::cout << "press any key to set cam view from file." << std::endl;
        // std::cin.get();
        // viewer.SetCamView("/home/csl/CppWorks/artwork/tiny-viewer/output/1683795147461499635.cam");

    } catch (const std::exception &e) {
        std::cout << e.what() << std::endl;
    }
}

void TEST_VIEWER() {
    try {
        {
            ns_viewer::Viewer::Create(ns_viewer::ViewerConfigor().WithWinName("win 1"))
                ->RunInSingleThread();
        }
        {
            ns_viewer::Viewer::Create(ns_viewer::ViewerConfigor().WithWinName("win 2"))
                ->RunInSingleThread();
        }
    } catch (const std::exception &e) {
        std::cout << e.what() << std::endl;
    }
}

void TEST_CAM_VIEW() {
    auto viewer = ns_viewer::Viewer();

    Eigen::Matrix3f rotMat;
    rotMat.col(0) = Eigen::Vector3f{1, 0, 0};
    rotMat.col(1) = Eigen::Vector3f{0, 0, -1};
    rotMat.col(2) = Eigen::Vector3f{0, 1, 0};

    Eigen::Vector3f pVec(0, -2, 1);

    auto a1 = Eigen::AngleAxisf(10.0 * M_PI / 180.0, Eigen::Vector3f(1, 0, 0));
    auto a2 = Eigen::AngleAxisf(10.0 * M_PI / 180.0, Eigen::Vector3f(0, 1, 0));
    auto a3 = Eigen::AngleAxisf(10.0 * M_PI / 180.0, Eigen::Vector3f(0, 0, 1));

    auto pose = ns_viewer::Posef((a3 * a2 * a1).matrix() * rotMat, pVec);

    viewer.AddEntity(ns_viewer::Camera::Create(pose, ns_viewer::Colour::Red()));
    viewer.RunInMultiThread();
    pose.translation -= pose.rotation.col(2) * 0.5f;

    std::cin.get();

    std::cout << "set camera pose" << std::endl;
    viewer.SetCamView(pose);
}

void TEST_MULTI_VIEWER() {
    try {
        using namespace ns_viewer;
        const std::string win1 = "win1", win2 = "win2";
        MultiViewerConfigor config({win1, win2});
        config.SaveConfigure("/home/csl/CppWorks/artwork/tiny-viewer/config/config.json");

        // viewer
        MultiViewer viewer("/home/csl/CppWorks/artwork/tiny-viewer/config/config.json");

        // add entities
        viewer.AddEntity(
            Line::Create({1.0, 1.0, 1.0}, {3.0, 4.0, 5.0}, Colour::Red().WithAlpha(0.3f)), win1);

        viewer.AddEntity(Cube::Create(Posef::Random(3.0f), false, Colour::Blue().WithAlpha(0.3f)),
                         win1);
        viewer.AddEntity(Cube::Create(Posef::Random(3.0f), true), win1);

        viewer.AddEntity(Plane::Create(Posef::Random(2.0f), 0.5f, 0.3f, false,
                                       Entity::GetUniqueColour().WithAlpha(0.3f)),
                         win1);
        viewer.AddEntity(Plane::Create(Posef::Random(1.0f), 0.5f, 0.3f, true), win1);

        viewer.AddEntity(Coordinate::Create(Posef::Random(5.0), 0.5f), win1);

        viewer.AddEntity(Cylinder::Create(Posef::Random(3.0f), 1.0, 0.5f), win1);

        viewer.AddEntity(Cloud<pcl::PointXYZRGBA>::Random(1.0f, 200, {1.0f, 2.0f, 3.0f}), win1);
        viewer.AddEntity(Cloud<pcl::PointXYZRGB>::Random(1.0f, 200, {3.0f, 3.0f, 3.0f}), win1);
        viewer.AddEntity(Cloud<pcl::PointXYZ>::Random(0.5f, 200, {3.0f, 2.0f, 1.0f}), win1);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile("/home/csl/CppWorks/artwork/tiny-viewer/data/scan.pcd", *cloud);
        viewer.AddEntity(Cloud<pcl::PointXYZI>::Create(cloud, 2.0f), win1);
        viewer.AddEntity(AlignedCloud<pcl::PointXYZI>::Random(2.0f, 2000, {-3.0f, 2.0f, -3.0f},
                                                              {0.0f, 0.0f, -1.0f}),
                         win1);

        viewer.AddEntity(IMU::Create(Posef::Random(3.0f)), win2);
        viewer.AddEntity(IMU::Create(Posef::Random(3.0f)), win2);

        viewer.AddEntity(Camera::Create(Posef::Random(3.0f)), win2);
        viewer.AddEntity(CubeCamera::Create(Posef::Random(3.0f)), win2);

        std::vector<Entity::Ptr> entities;
        for (int i = 0; i < 5; ++i) {
            entities.push_back(LiDAR::Create(Posef::Random(5.0f)));
        }
        auto ids = viewer.AddEntity(entities, win2);

        viewer.AddEntity(Arrow::Create({4.0f, 2.0f, 2.0f}, {4.0f, 2.0f, 5.0f}), win2);

        viewer.AddEntity(Cone::Create(Posef::Random(5.0f), 0.3f, M_PI_4), win2);

        viewer.AddEntity(Polygon::Create({{0.0f, 0.0f, 2.0f},
                                          {0.0f, 1.0f, 3.0f},
                                          {1.0f, 2.0f, 2.0f},
                                          {2.0f, 3.0f, 1.0f},
                                          {3.0f, 1.0f, 2.0f}},
                                         false, Colour::Black().WithAlpha(0.2f)),
                         win2);

        for (int i = 0; i < 10; ++i) {
            viewer.AddEntity(Surfel::Random(5.0f), win2);
        }

        viewer.AddEntity(Radar::Create(Posef::Random(5.0f)), win2);

        viewer.AddEntity(
            Path::Create(
                "M 3 3 3 l 0 0 -1 s 0 0 -0.5 0.5 0 0 0.5 0 0 0 0 0.5 l 0 0 1 "
                "m 0.25 0 0 l 0 0 -1 s 0 0 -0.5 0.5 0 0 l 0.25 0 0 "
                "m 0.5 0 0 s -0.5 0 0 0 0 0.5 0 0 0.5 0.5 0 0 0.5 0 0 0 0 -0.5 0 0 -0.5 -0.5 0 0 "
                "m 0.5 0 0.75 l 0 0 -0.75 s 0 0 0.5 0 0 0.5 0.4 0 0 0.4 0 0 0 0 -0.5 0 0 -0.5 "
                "m 1 0 0.5 s 0 0 -0.5 -0.5 0 0 -0.5 0 0 0 0 0.5 0 0 0.5 0.5 0 0 0.5 0 0 0 0 -0.5 "
                "s 0 0 -0.75 0 0 -0.5 -0.25 0 0 -0.5 0 0 0 0 0.25 "
                "m 1 0 1.25 s 0 0 0.5 0.5 0 0 0.5 0 0 0 0 -0.5 -0.5 0 -0.5 -0.5 0 -0.25 l 0.75 0 "
                "0"),
            win2);

        viewer.AddEntity(Cloud<Landmark>::Random(5.0, 20, {4, 5, 4}), win2);

        // show (multi thread)
        viewer.RunInMultiThread();

        // std::cout << "press any key to set cam view from file." << std::endl;
        // std::cin.get();
        // viewer.SetCamView("/home/csl/CppWorks/artwork/tiny-viewer/output/1683795147461499635.cam");

    } catch (const std::exception &e) {
        std::cout << e.what() << std::endl;
    }
}

int main(int argc, char **argv) {
    TEST_ENTITIES();
    // TEST_VIEWER();
    // TEST_CAM_VIEW();
    // TEST_MULTI_VIEWER();
    return 0;
}