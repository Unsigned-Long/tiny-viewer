//
// Created by csl on 10/22/22.
//

#ifndef TINY_VIEWER_MULTI_VIEWER_H
#define TINY_VIEWER_MULTI_VIEWER_H

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
#include "tiny-viewer/core/viewer_configor.h"
#include "tiny-viewer/object/landmark.h"

namespace ns_viewer {

#define LOCKER_MULTI_VIEWER std::unique_lock<std::mutex> viewerLock(MultiViewer::MUTEX);

    class MultiViewer {
    public:
        using Ptr = std::shared_ptr<MultiViewer>;

    protected:
        MultiViewerConfigor _configor;
        std::shared_ptr<std::thread> _thread;

    public:
        static std::mutex MUTEX;

    protected:
        // sub window name, entities
        std::unordered_map<std::string, std::unordered_map<std::size_t, Entity::Ptr>> _entities;
        std::unordered_map<std::string, pangolin::OpenGlRenderState> _camView;

    public:

        explicit MultiViewer(MultiViewerConfigor configor);

        explicit MultiViewer(const std::string &configPath);

        static Ptr Create(const MultiViewerConfigor &configor);

        static Ptr Create(const std::string &configPath);

        // used for load viewer from file
        explicit MultiViewer(char) : _configor({}), _thread(nullptr) {}

        virtual ~MultiViewer();

        void RunInSingleThread();

        void RunInMultiThread();

        std::size_t AddEntity(const Entity::Ptr &entity, const std::string &subWinName);

        std::vector<std::size_t> AddEntity(const std::vector<Entity::Ptr> &entities, const std::string &subWinName);

        bool RemoveEntity(std::size_t id, const std::string &subWinName);

        bool RemoveEntity(const std::vector<std::size_t> &ids, const std::string &subWinName);

        bool RemoveEntity(const std::string &subWinName);

        bool RemoveEntity();

        void SetCamView(const pangolin::OpenGlRenderState &camView, const std::string &subWinName);

        void SetCamView(const std::string &filename, const std::string &subWinName);

        void SetCamView(Posef T_CamToWorld, const std::string &subWinName);

        void Save(const std::string &filename, bool binaryMode = true) const;

        static Ptr Load(const std::string &filename, bool binaryMode = true);

        MultiViewerConfigor &GetConfigor();

    protected:

        void InitMultiViewer(bool initCamViewFromConfigor);

        void Run();

        void SaveScreenShotCallBack() const;

        void SaveCameraCallBack() const;

        void SaveMultiViewerCallBack() const;

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

#endif //TINY_VIEWER_MULTI_VIEWER_H
