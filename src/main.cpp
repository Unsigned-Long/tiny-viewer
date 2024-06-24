//
// Created by csl on 10/16/22.
//
#include "tiny-viewer/core/viewer.h"
#include "pcl/io/pcd_io.h"
#include "tiny-viewer/core/multi_viewer.h"
#include "pangolin/handler/handler.h"

void TEST_ENTITIES() {
    try {
        using namespace ns_viewer;
        ViewerConfigor().SaveConfigure("/home/csl/CppWorks/artwork/tiny-viewer/config/config.json");

        // viewer
        Viewer viewer("/home/csl/CppWorks/artwork/tiny-viewer/config/config.json");

        viewer.GetConfigor().callBacks.insert({'a', [&viewer]() {
            viewer.AddEntity(Path::Create(
                    "M 3 3 3 l 0 0 -1 s 0 0 -0.5 0.5 0 0 0.5 0 0 0 0 0.5 l 0 0 1 "
                    "m 0.25 0 0 l 0 0 -1 s 0 0 -0.5 0.5 0 0 l 0.25 0 0 "
                    "m 0.5 0 0 s -0.5 0 0 0 0 0.5 0 0 0.5 0.5 0 0 0.5 0 0 0 0 -0.5 0 0 -0.5 -0.5 0 0 "
                    "m 0.5 0 0.75 l 0 0 -0.75 s 0 0 0.5 0 0 0.5 0.4 0 0 0.4 0 0 0 0 -0.5 0 0 -0.5 "
                    "m 1 0 0.5 s 0 0 -0.5 -0.5 0 0 -0.5 0 0 0 0 0.5 0 0 0.5 0.5 0 0 0.5 0 0 0 0 -0.5 "
                    "s 0 0 -0.75 0 0 -0.5 -0.25 0 0 -0.5 0 0 0 0 0.25 "
                    "m 1 0 1.25 s 0 0 0.5 0.5 0 0 0.5 0 0 0 0 -0.5 -0.5 0 -0.5 -0.5 0 -0.25 l 0.75 0 0"));
        }});

        // add entities
        viewer.AddEntity(Line::Create({1.0, 1.0, 1.0}, {3.0, 4.0, 5.0}, Colour::Red().WithAlpha(0.3f)));

        viewer.AddEntity(Cube::Create(Posef::Random(3.0f), false, Colour::Blue().WithAlpha(0.3f)));
        viewer.AddEntity(Cube::Create(Posef::Random(3.0f), true));

        viewer.AddEntity(
                Plane::Create(Posef::Random(2.0f), 0.5f, 0.3f, false, Entity::GetUniqueColour().WithAlpha(0.3f))
        );
        viewer.AddEntity(Plane::Create(Posef::Random(1.0f), 0.5f, 0.3f, true));

        viewer.AddEntity(Coordinate::Create(Posef::Random(5.0), 0.5f));

        viewer.AddEntity(Cylinder::Create(Posef::Random(3.0f), 1.0, 0.5f));

        viewer.AddEntity(Cloud<pcl::PointXYZRGBA>::Random(1.0f, 200, {1.0f, 2.0f, 3.0f}));
        viewer.AddEntity(Cloud<pcl::PointXYZRGB>::Random(1.0f, 200, {3.0f, 3.0f, 3.0f}));
        viewer.AddEntity(Cloud<pcl::PointXYZ>::Random(0.5f, 200, {3.0f, 2.0f, 1.0f}));
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile("/home/csl/CppWorks/artwork/tiny-viewer/data/scan.pcd", *cloud);
        viewer.AddEntity(Cloud<pcl::PointXYZI>::Create(cloud, 2.0f));
        viewer.AddEntity(AlignedCloud<pcl::PointXYZI>::Random(2.0f, 2000, {-3.0f, 2.0f, -3.0f}, {0.0f, 0.0f, -1.0f}));

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
                                          {3.0f, 1.0f, 2.0f}}, false, Colour::Black().WithAlpha(0.2f)));

        for (int i = 0; i < 10; ++i) {
            viewer.AddEntity(Surfel::Random(5.0f));
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
        { ns_viewer::Viewer::Create(ns_viewer::ViewerConfigor().WithWinName("win 1"))->RunInSingleThread(); }
        { ns_viewer::Viewer::Create(ns_viewer::ViewerConfigor().WithWinName("win 2"))->RunInSingleThread(); }
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
        viewer.AddEntity(Line::Create({1.0, 1.0, 1.0}, {3.0, 4.0, 5.0}, Colour::Red().WithAlpha(0.3f)), win1);

        viewer.AddEntity(Cube::Create(Posef::Random(3.0f), false, Colour::Blue().WithAlpha(0.3f)), win1);
        viewer.AddEntity(Cube::Create(Posef::Random(3.0f), true), win1);

        viewer.AddEntity(
                Plane::Create(Posef::Random(2.0f), 0.5f, 0.3f, false, Entity::GetUniqueColour().WithAlpha(0.3f)), win1
        );
        viewer.AddEntity(Plane::Create(Posef::Random(1.0f), 0.5f, 0.3f, true), win1);

        viewer.AddEntity(Coordinate::Create(Posef::Random(5.0), 0.5f), win1);

        viewer.AddEntity(Cylinder::Create(Posef::Random(3.0f), 1.0, 0.5f), win1);

        viewer.AddEntity(Cloud<pcl::PointXYZRGBA>::Random(1.0f, 200, {1.0f, 2.0f, 3.0f}), win1);
        viewer.AddEntity(Cloud<pcl::PointXYZRGB>::Random(1.0f, 200, {3.0f, 3.0f, 3.0f}), win1);
        viewer.AddEntity(Cloud<pcl::PointXYZ>::Random(0.5f, 200, {3.0f, 2.0f, 1.0f}), win1);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile("/home/csl/CppWorks/artwork/tiny-viewer/data/scan.pcd", *cloud);
        viewer.AddEntity(Cloud<pcl::PointXYZI>::Create(cloud, 2.0f), win1);
        viewer.AddEntity(AlignedCloud<pcl::PointXYZI>::Random(2.0f, 2000, {-3.0f, 2.0f, -3.0f}, {0.0f, 0.0f, -1.0f}),
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
                                          {3.0f, 1.0f, 2.0f}}, false, Colour::Black().WithAlpha(0.2f)), win2);

        for (int i = 0; i < 10; ++i) {
            viewer.AddEntity(Surfel::Random(5.0f), win2);
        }

        viewer.AddEntity(Radar::Create(Posef::Random(5.0f)), win2);

        viewer.AddEntity(Path::Create(
                "M 3 3 3 l 0 0 -1 s 0 0 -0.5 0.5 0 0 0.5 0 0 0 0 0.5 l 0 0 1 "
                "m 0.25 0 0 l 0 0 -1 s 0 0 -0.5 0.5 0 0 l 0.25 0 0 "
                "m 0.5 0 0 s -0.5 0 0 0 0 0.5 0 0 0.5 0.5 0 0 0.5 0 0 0 0 -0.5 0 0 -0.5 -0.5 0 0 "
                "m 0.5 0 0.75 l 0 0 -0.75 s 0 0 0.5 0 0 0.5 0.4 0 0 0.4 0 0 0 0 -0.5 0 0 -0.5 "
                "m 1 0 0.5 s 0 0 -0.5 -0.5 0 0 -0.5 0 0 0 0 0.5 0 0 0.5 0.5 0 0 0.5 0 0 0 0 -0.5 "
                "s 0 0 -0.75 0 0 -0.5 -0.25 0 0 -0.5 0 0 0 0 0.25 "
                "m 1 0 1.25 s 0 0 0.5 0.5 0 0 0.5 0 0 0 0 -0.5 -0.5 0 -0.5 -0.5 0 -0.25 l 0.75 0 0"), win2);

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
    TEST_VIEWER();
    // TEST_CAM_VIEW();
    TEST_MULTI_VIEWER();
    return 0;
}