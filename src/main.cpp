//
// Created by csl on 10/16/22.
//
#include "tiny-viewer/core/viewer.h"

int main(int argc, char **argv) {
    try {
        using namespace ns_viewer;
        // viewer
        Viewer viewer("/home/csl/CppWorks/artwork/tiny-viewer/config/config.json");

        // add entities
        viewer.AddEntity(Line::Create({1.0, 1.0, 1.0}, {3.0, 4.0, 5.0}, Colour::Red().WithAlpha(0.3f)));
        viewer.AddEntity(Coordinate::Create(Posef::Random(5.0), 0.5f));
        viewer.AddEntity(Cube::Create(Posef::Random(3.0f), false, Colour::Blue().WithAlpha(0.3f)));
        viewer.AddEntity(Cube::Create(Posef::Random(3.0f), true));
        viewer.AddEntity(PosColorCloud::Random(1.0f, 200, {1.0f, 2.0f, 3.0f}));
        viewer.AddEntity(PosCloud::Random(0.5f, 200, {3.0f, 2.0f, 1.0f}));
        viewer.AddEntity(IMU::Create(Posef::Random(3.0f)));
        viewer.AddEntity(IMU::Create(Posef::Random(3.0f)));
        viewer.AddEntity(Camera::Create(Posef::Random(3.0f)));
        viewer.AddEntity(LiDAR::Create(Posef::Random(1.0f)));
        std::vector<Entity::Ptr> entities;
        for (int i = 0; i < 5; ++i) {
            entities.push_back(LiDAR::Create(Posef::Random(5.0f)));
        }
        auto ids = viewer.AddEntity(entities);
        // show (multi thread)
        viewer.RunInMultiThread();
        // access
        std::cout << "hello, world!" << std::endl;
        // std::cin.get();
        // std::cout << "hello, world!" << std::endl;
        // viewer.RemoveEntity(ids);
        // std::cin.get();
        // std::cout << "hello, world!" << std::endl;
        // viewer.RemoveEntity();

    } catch (const std::exception &e) {
        std::cout << e.what() << std::endl;
    }
    return 0;
}