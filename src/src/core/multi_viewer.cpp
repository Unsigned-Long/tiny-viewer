//
// Created by csl on 10/22/22.
//

#include "tiny-viewer/core/multi_viewer.h"
#include "filesystem"
#include "pangolin/display/display.h"
#include "pangolin/display/view.h"
#include "pangolin/handler/handler.h"
#include "pangolin/gl/gldraw.h"

namespace ns_viewer {

    // ------
    // MultiViewer
    // ------

    std::mutex MultiViewer::MUTEX = {};

    // --------------
    // public methods
    // --------------
    MultiViewer::MultiViewer(const std::set<std::string> &subWinNames, ViewerConfigor configor)
            : _configor(std::move(configor)), _thread(nullptr), _subWinNames(subWinNames) { InitMultiViewer(true); }

    MultiViewer::Ptr MultiViewer::Create(const std::set<std::string> &subWinNames, const ViewerConfigor &configor) {
        return std::make_shared<MultiViewer>(subWinNames, configor);
    }

    MultiViewer::Ptr
    MultiViewer::Create(const std::string &mainWinName, const std::set<std::string> &subWinNames, bool showGrid,
                        bool showIdentityCoord) {
        ViewerConfigor configor(mainWinName);
        configor.Grid.ShowGrid = showGrid;
        configor.Grid.ShowIdentityCoord = showIdentityCoord;
        return std::make_shared<MultiViewer>(subWinNames, configor);
    }

    MultiViewer::MultiViewer(const std::set<std::string> &subWinNames, const std::string &configPath)
            : MultiViewer(subWinNames, ViewerConfigor::LoadConfigure(configPath)) {}

    MultiViewer::~MultiViewer() {
        if (_thread != nullptr) {
            _thread->join();
        }
        pangolin::DestroyWindow(_configor.Window.Name);
    }

    void MultiViewer::RunInSingleThread() {
        Run();
    }

    void MultiViewer::RunInMultiThread() {
        this->_thread = std::make_shared<std::thread>([this]() { Run(); });
    }

    // -----------------
    // protected methods
    // -----------------
    void MultiViewer::InitMultiViewer(bool initCamViewFromConfigor) {
        // create a window and bind its context to the main thread
        pangolin::CreateWindowAndBind(_configor.Window.Name, _configor.Window.Width, _configor.Window.Height);

        // unset the current context from the main thread
        pangolin::GetBoundWindow()->RemoveCurrent();

        if (initCamViewFromConfigor) {
            // Define Projection and initial ModelView matrix
            const auto &c = _configor.Camera;
            for (const auto &name: _subWinNames) {
                _camView.insert({name, pangolin::OpenGlRenderState(
                        pangolin::ProjectionMatrix(c.Width, c.Height, c.Fx, c.Fy, c.Cx, c.Cy, c.Near, c.Far),
                        pangolin::ModelViewLookAt(
                                ExpandStdVec3(_configor.Camera.InitPos),
                                ExpandStdVec3(_configor.Camera.InitViewPoint), pangolin::AxisZ
                        ))});
                _entities.insert({name, {}});
            }

            if (_configor.Grid.ShowIdentityCoord) {
                for (const auto &name: _subWinNames) { AddEntity(Coordinate::Create(Posef()), name); }
            }

            if (_configor.Grid.ShowGrid) {
                Eigen::Vector3f v1, v2;
                switch (_configor.Grid.PlanePos % 3) {
                    case 0:
                        v1 = {1.0f, 0.0f, 0.0f};
                        v2 = {0.0f, 1.0f, 0.0f};
                        break;
                    case 1:
                        v1 = {0.0f, 1.0f, 0.0f};
                        v2 = {0.0f, 0.0f, 1.0f};
                        break;
                    case 2:
                        v1 = {0.0f, 0.0f, 1.0f};
                        v2 = {1.0f, 0.0f, 0.0f};
                        break;
                    default:
                        v1 = {1.0f, 0.0f, 0.0f};
                        v2 = {0.0f, 1.0f, 0.0f};
                }
                v1 *= _configor.Grid.CellSize;
                v2 *= _configor.Grid.CellSize;
                Eigen::Vector3f s1 = -v1 * _configor.Grid.CellCount * 0.5;
                Eigen::Vector3f s2 = -v2 * _configor.Grid.CellCount * 0.5;
                Eigen::Vector3f s3 = s1 + s2;

                for (const auto &name: _subWinNames) {
                    for (int i = 0; i < _configor.Grid.CellCount + 1; ++i) {
                        Eigen::Vector3f p1 = s3 + i * v1;
                        Eigen::Vector3f p2 = p1 + _configor.Grid.CellCount * v2;
                        AddEntity(Line::Create(p1, p2, DefaultLineSize, _configor.Grid.Color), name);
                    }

                    for (int i = 0; i < _configor.Grid.CellCount + 1; ++i) {
                        Eigen::Vector3f p1 = s3 + i * v2;
                        Eigen::Vector3f p2 = p1 + _configor.Grid.CellCount * v1;
                        AddEntity(Line::Create(p1, p2, DefaultLineSize, _configor.Grid.Color), name);
                    }
                }

            }
        }
    }

    void MultiViewer::Run() {

        // fetch the context and bind it to this thread
        pangolin::BindToContext(_configor.Window.Name);

        // we manually need to restore the properties of the context
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // Create Interactive View in window
        std::unordered_map<std::string, pangolin::View &> d_cam;

        auto &display = pangolin::Display("multi").SetBounds(0.0, 1.0, 0.0, 1.0).SetLayout(pangolin::LayoutEqual);

        for (const auto &name: _subWinNames) {
            pangolin::View &cam = pangolin::Display(name)
                    .SetAspect(static_cast<double>(_configor.Camera.Width) / _configor.Camera.Height)
                    .SetHandler(new pangolin::Handler3D(_camView.at(name)));
            d_cam.insert({name, cam});
            display.AddDisplay(cam);
        }

        if (std::filesystem::exists(_configor.Output.DataOutputPath)) {

            pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 's', [this] { SaveScreenShotCallBack(); });
            pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'c', [this] { SaveCameraCallBack(); });
            pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'v', [this] { SaveMultiViewerCallBack(); });
            pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'r', [this] { VideoRecordCallBack(); });

            std::cout << "\033[92m\033[3m[MultiViewer] "
                         "press [ctrl+'s'] to save the current scene, "
                         "[ctrl+'c'] for camera view, "
                         "and [ctrl+'v'] for total viewer.\033[0m" << std::endl;
        }

        while (!pangolin::ShouldQuit()) {

            // Clear screen and activate view to render into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glClearColor(
                    _configor.Window.BackGroundColor.r, _configor.Window.BackGroundColor.g,
                    _configor.Window.BackGroundColor.b, _configor.Window.BackGroundColor.a
            );

            // -------
            // drawing
            // -------
            {
                LOCKER_MULTI_VIEWER
                for (const auto &[name, entities]: _entities) {
                    d_cam.at(name).Activate(_camView.at(name));
                    for (const auto &[id, entity]: entities) { entity->Draw(); }
                }
            }
            // -----------
            // end drawing
            // -----------

            // Swap frames and Process Events
            pangolin::FinishFrame();
        }

        // unset the current context from the main thread
        pangolin::GetBoundWindow()->RemoveCurrent();
    }

    void MultiViewer::SaveScreenShotCallBack() const {
        std::int64_t curTimeStamp = std::chrono::system_clock::now().time_since_epoch().count();
        const std::string filename = _configor.Output.DataOutputPath + "/" + std::to_string(curTimeStamp) + ".png";
        pangolin::SaveWindowOnRender(filename);
        std::cout << "\033[92m\033[3m[MultiViewer] the scene shot is saved to path: '"
                  << filename << "\033[0m" << std::endl;
    }

    void MultiViewer::SaveCameraCallBack() const {
        std::int64_t curTimeStamp = std::chrono::system_clock::now().time_since_epoch().count();
        const std::string filename = _configor.Output.DataOutputPath + "/" + std::to_string(curTimeStamp) + ".cam";

        std::ofstream file(filename);
        cereal::JSONOutputArchive ar(file);
        ar(cereal::make_nvp("cam_view", this->_camView));
        std::cout << "\033[92m\033[3m[MultiViewer] the camera view is saved to path: '"
                  << filename << "\033[0m" << std::endl;
    }

    void MultiViewer::SaveMultiViewerCallBack() const {
        std::int64_t curTimeStamp = std::chrono::system_clock::now().time_since_epoch().count();
        const std::string filename = _configor.Output.DataOutputPath + "/" + std::to_string(curTimeStamp) + ".view";
        this->Save(filename, true);
        std::cout << "\033[92m\033[3m[MultiViewer] the viewer is saved to path: '"
                  << filename << "\033[0m" << std::endl;
    }

    // ----------------
    // process Entities
    // ----------------
    std::size_t MultiViewer::AddEntity(const Entity::Ptr &entity, const std::string &subWinName) {
        LOCKER_MULTI_VIEWER
        _entities.at(subWinName).insert({entity->GetId(), entity});
        return entity->GetId();
    }

    bool MultiViewer::RemoveEntity(std::size_t id, const std::string &subWinName) {
        LOCKER_MULTI_VIEWER
        return _entities.at(subWinName).erase(id) == 1;
    }

    std::vector<std::size_t>
    MultiViewer::AddEntity(const std::vector<Entity::Ptr> &entities, const std::string &subWinName) {
        LOCKER_MULTI_VIEWER
        std::vector<std::size_t> ids(entities.size());
        for (int i = 0; i < static_cast<int>(entities.size()); ++i) {
            const Entity::Ptr &entity = entities.at(i);
            _entities.at(subWinName).insert({entity->GetId(), entity});
            ids.at(i) = entity->GetId();
        }
        return ids;
    }

    bool MultiViewer::RemoveEntity(const std::vector<std::size_t> &ids, const std::string &subWinName) {
        LOCKER_MULTI_VIEWER
        bool b = false;
        for (const auto &id: ids) {
            b = (_entities.at(subWinName).erase(id) == 1) && b;
        }
        return b;
    }

    bool MultiViewer::RemoveEntity() {
        LOCKER_MULTI_VIEWER
        _entities.clear();
        return true;
    }

    void MultiViewer::Save(const std::string &filename, bool binaryMode) const {
        std::ofstream file(filename);
        if (binaryMode) {
            cereal::BinaryOutputArchive ar(file);
            LOCKER_MULTI_VIEWER
            ar(*this);
        } else {
            cereal::JSONOutputArchive ar(file);
            LOCKER_MULTI_VIEWER
            ar(*this);
        }
    }

    MultiViewer::Ptr MultiViewer::Load(const std::string &filename, bool binaryMode) {
        auto viewer = std::make_shared<MultiViewer>('2');
        std::ifstream file(filename);
        if (binaryMode) {
            cereal::BinaryInputArchive ar(file);
            ar(*viewer);
        } else {
            cereal::JSONInputArchive ar(file);
            ar(*viewer);
        }
        viewer->InitMultiViewer(false);
        return viewer;
    }

    void MultiViewer::SetCamView(const pangolin::OpenGlRenderState &camView, const std::string &subWinName) {
        this->_camView.at(subWinName).SetModelViewMatrix(camView.GetModelViewMatrix());
        this->_camView.at(subWinName).SetProjectionMatrix(camView.GetProjectionMatrix());
    }

    void MultiViewer::SetCamView(const std::string &filename, const std::string &subWinName) {
        std::ifstream file(filename);
        cereal::JSONInputArchive ar(file);
        pangolin::OpenGlRenderState camView;
        ar(camView);
        SetCamView(camView, subWinName);
    }

    void MultiViewer::VideoRecordCallBack() const {
        // todo: record video
    }

    ViewerConfigor &MultiViewer::GetConfigor() {
        return _configor;
    }

    void MultiViewer::SetCamView(Posef T_CamToWorld, const std::string &subWinName) {
        Eigen::Vector3f vp(T_CamToWorld.translation + T_CamToWorld.rotation.col(2));
        this->_camView.at(subWinName).SetModelViewMatrix(pangolin::ModelViewLookAt(
                ExpandVec3(T_CamToWorld.translation), ExpandVec3(vp), pangolin::AxisZ
        ));
    }
}
