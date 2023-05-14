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
#include "tiny-viewer/object/camera.h"
#include "tiny-viewer/object/imu.h"
#include "tiny-viewer/object/lidar.h"
#include "tiny-viewer/object/surfel.h"
#include "utility"

namespace ns_viewer {

    struct ViewerConfigor {
    public:
        struct {
            std::string Name = "Tiny Viewer";
            std::string DataOutputPath;
            Colour BackGroundColor = Colour::White();
            int Width = 640 * 2;
            int height = 480 * 2;

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(
                        cereal::make_nvp("Name", Name),
                        cereal::make_nvp("DataOutputPath", DataOutputPath),
                        cereal::make_nvp("BackGroundColor", BackGroundColor),
                        cereal::make_nvp("Width", Width),
                        cereal::make_nvp("height", height)
                );
            }
        } Window;

        struct {
            int Width = 640, Height = 480;
            double Fx = 420, Fy = 420;
            double Cx = 320, Cy = 240;
            double Near = 0.01, Far = 100;

            std::vector<float> InitPos = {6.0f, 6.0f, 6.0f};
            std::vector<float> InitViewPoint = {0.0f, 0.0f, 0.0f};

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(
                        cereal::make_nvp("Width", Width),
                        cereal::make_nvp("Height", Height),
                        cereal::make_nvp("Fx", Fx),
                        cereal::make_nvp("Fy", Fy),
                        cereal::make_nvp("Cx", Cx),
                        cereal::make_nvp("Cy", Cy),
                        cereal::make_nvp("Near", Near),
                        cereal::make_nvp("Far", Far),
                        cereal::make_nvp("InitPos", InitPos),
                        cereal::make_nvp("InitViewPoint", InitViewPoint)
                );
            }
        } Camera;

        struct {
            int CellCount = 10;
            float CellSize = 1.0f;
            // 0: xy, 1: yz, 2: zx
            int PlaneId = 0;

            Colour Color = Colour::Black().WithAlpha(0.3f);

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(
                        cereal::make_nvp("CellCount", CellCount),
                        cereal::make_nvp("CellSize", CellSize),
                        cereal::make_nvp("PlaneId", PlaneId),
                        cereal::make_nvp("Color", Color)
                );
            }
        } Grid;

    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(
                    cereal::make_nvp("Window", Window),
                    cereal::make_nvp("Camera", Camera),
                    cereal::make_nvp("Grid", Grid)
            );
        }

    public:
        explicit ViewerConfigor(const std::string &winName = "Tiny Viewer");

        // load configure information from the json file
        static ViewerConfigor LoadConfigure(const std::string &filename);

        // load configure information from the json file
        bool SaveConfigure(const std::string &filename);

        ViewerConfigor &WithWinName(const std::string &winName);

        ViewerConfigor &WithScreenShotSaveDir(const std::string &dir);
    };

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

    public:

        explicit Viewer(ViewerConfigor configor = ViewerConfigor());

        explicit Viewer(const std::string &configPath);

        static Ptr Create(const ViewerConfigor &configor = ViewerConfigor());

        // used for load viewer from file
        explicit Viewer(char) : _thread(nullptr) {}

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

        void Save(const std::string &filename, bool binaryMode = true) const;

        static Ptr Load(const std::string &filename, bool binaryMode = true);

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
