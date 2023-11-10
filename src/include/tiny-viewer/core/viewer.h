//
// Created by csl on 10/22/22.
//

#ifndef TINY_VIEWER_VIEWER_H
#define TINY_VIEWER_VIEWER_H

#include "iostream"
#include "memory"
#include "mutex"
#include "pangolin/gl/opengl_render_state.h"
#include "thread"
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
#include "tiny-viewer/core/viewer_configor.h"
#include "utility"

namespace ns_viewer {

#define LOCKER_VIEWER std::unique_lock<std::mutex> viewerLock(Viewer::MUTEX);

    class Viewer {
    public:
        using Ptr = std::shared_ptr<Viewer>;

    protected:
        ViewerConfigor _configor;
        std::shared_ptr<std::thread> _thread;

    public:
        static std::mutex MUTEX;

    protected:
        std::unordered_map<std::size_t, Entity::Ptr> _entities;
        pangolin::OpenGlRenderState _camView;
        bool _isActive;

    public:

        explicit Viewer(ViewerConfigor configor = ViewerConfigor());

        explicit Viewer(const std::string &configPath);

        static Ptr Create(const ViewerConfigor &configor = ViewerConfigor());

        static Ptr Create(const std::string &configPath);

        // used for load viewer from file
        explicit Viewer(char) : _thread(nullptr), _isActive(false) {}

        virtual ~Viewer();

        void RunInSingleThread();

        void RunInMultiThread();

        std::size_t AddEntity(const Entity::Ptr &entity);

        std::vector<std::size_t> AddEntity(const std::vector<Entity::Ptr> &entities);

        bool RemoveEntity(std::size_t id);

        bool RemoveEntity(const std::vector<std::size_t> &ids);

        bool RemoveEntity();

        void SetCamView(const pangolin::OpenGlRenderState &camView);

        void SetCamView(const std::string &filename);

        void SetCamView(Posef T_CamToWorld);

        void Save(const std::string &filename, bool binaryMode = true) const;

        static Ptr Load(const std::string &filename, bool binaryMode = true);

        ViewerConfigor &GetConfigor();

        bool IsActive() const;

    protected:

        void InitViewer(bool initCamViewFromConfigor);

        void Run();

        void SaveScreenShotCallBack() const;

        void SaveCameraCallBack() const;

        void SaveViewerCallBack() const;

        void VideoRecordCallBack() const;

    public:

        template<class Archive>
        void serialize(Archive &archive) {
            archive(
                    cereal::make_nvp("configor", _configor),
                    cereal::make_nvp("entities", _entities),
                    cereal::make_nvp("camera_view", _camView)
            );
        }
    };
}

#endif //TINY_VIEWER_VIEWER_H
