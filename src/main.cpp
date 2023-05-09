//
// Created by csl on 10/16/22.
//
#include "tiny-viewer/core/viewer.h"

int main(int argc, char **argv) {
    try {
        using namespace ns_viewer;
        // configor
        ViewerConfigor configor;
        configor.ScreenShotSaveDir = "../scene-shot";

        // viewer
        Viewer viewer(configor);

        // add entities
        viewer.AddLine(Line({1.0, 1.0, 1.0}, {3.0, 4.0, 5.0}, Colour::Red().WithAlpha(0.3f)));
        viewer.AddCoordinate(Coordinate(Posef::Random(5.0), 0.5f));
        viewer.AddCube(Cube(Posef::Random(3.0f), false, Colour::Blue().WithAlpha(0.3f)));
        viewer.AddCube(Cube(Posef::Random(3.0f), true));

        // show (multi thread)
        viewer.RunInMultiThread();

        // access
        std::cout << "hello, world!" << std::endl;

    } catch (const std::exception &e) {
        std::cout << e.what() << std::endl;
    }
    return 0;
}