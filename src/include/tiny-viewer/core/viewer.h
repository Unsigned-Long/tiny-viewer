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
#include "tiny-viewer/entity/camera.h"
#include "tiny-viewer/entity/lidar.h"

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
        std::unordered_map<std::size_t, Entity::Ptr> _entities;

    public:

        explicit Viewer(ViewerConfigor configor = ViewerConfigor());

        static Ptr Create(const ViewerConfigor &configor = ViewerConfigor());

        virtual ~Viewer();

        void RunInSingleThread();

        void RunInMultiThread();

        void AddEntity(const Entity::Ptr &entity);

    protected:

        void InitViewer();

        void Run();

        void KeyBoardCallBack() const;
    };
}

#endif //TINY_VIEWER_VIEWER_H
