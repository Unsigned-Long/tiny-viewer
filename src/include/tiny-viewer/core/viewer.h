//
// Created by csl on 10/22/22.
//

#ifndef TINY_VIEWER_VIEWER_H
#define TINY_VIEWER_VIEWER_H

#include "thread"
#include "memory"
#include "utility"
#include "tiny-viewer/entity/line.h"
#include "tiny-viewer/entity/coordinate.h"
#include "iostream"
#include "tiny-viewer/entity/cube.h"
#include "tiny-viewer/entity/point_cloud.h"
#include "tiny-viewer/entity/imu.h"

namespace ns_viewer {

    struct ViewerConfigor {
    public:
        std::string ScreenShotSaveDir;
        Colour BackGroundColor = Colour::White();
        std::string WinName = "Tiny Viewer";

        // draw setting
    };

    class Viewer {
    public:
        using Ptr = std::shared_ptr<Viewer>;

    protected:
        ViewerConfigor _configor;
        std::shared_ptr<std::thread> _thread;

    protected:
        std::unordered_map<std::size_t, Line> _lines;
        std::unordered_map<std::size_t, Coordinate> _coords;
        std::unordered_map<std::size_t, Cube> _cubes;
        std::unordered_map<std::size_t, PosColorCloud> _posColorClouds;
        std::unordered_map<std::size_t, PosCloud> _posClouds;
        std::unordered_map<std::size_t, IMU> _imus;

    public:

        explicit Viewer(ViewerConfigor configor = ViewerConfigor());

        static Ptr Create(const ViewerConfigor &configor = ViewerConfigor());

        virtual ~Viewer();

        void RunInSingleThread();

        void RunInMultiThread();

        void AddLine(const Line &l);

        void AddCoordinate(const Coordinate &coordinate);

        void AddCube(const Cube &cube);

        void AddPointCloud(const PosColorCloud &cloud);

        void AddPointCloud(const PosCloud &cloud);

        void AddIMU(const IMU &imu);

    protected:

        void InitViewer();

        void Run();

        void KeyBoardCallBack() const;
    };
}

#endif //TINY_VIEWER_VIEWER_H
