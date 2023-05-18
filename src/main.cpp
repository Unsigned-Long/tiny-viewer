//
// Created by csl on 10/16/22.
//
#include "tiny-viewer/core/viewer.h"
#include "pcl/io/pcd_io.h"
#include <vtkLookupTable.h>

int main(int argc, char **argv) {
    try {
        using namespace ns_viewer;
        // {
        //     ViewerConfigor().SaveConfigure("/home/csl/CppWorks/artwork/tiny-viewer/config/config.json");
        //     std::cin.get();
        // }
        // {
        //     auto viewer = Viewer::Load("/home/csl/CppWorks/artwork/tiny-viewer/output/1683794904023740133.view");
        //     viewer->RunInMultiThread();
        //     return 0;
        // }
        // viewer
        Viewer viewer("/home/csl/CppWorks/artwork/tiny-viewer/config/config.json");

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

        // show (multi thread)
        viewer.RunInMultiThread();

        // std::cout << "press any key to set cam view from file." << std::endl;
        // std::cin.get();
        // viewer.SetCamView("/home/csl/CppWorks/artwork/tiny-viewer/output/1683795147461499635.cam");

    } catch (const std::exception &e) {
        std::cout << e.what() << std::endl;
    }
    return 0;
}