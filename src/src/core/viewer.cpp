//
// Created by csl on 10/22/22.
//

#include "tiny-viewer/core/viewer.h"
#include "filesystem"
#include "pangolin/display/display.h"
#include "pangolin/display/view.h"
#include "pangolin/handler/handler.h"
#include "pangolin/gl/gldraw.h"

namespace ns_viewer {

    // --------------
    // ViewerConfigor
    // --------------
    ViewerConfigor::ViewerConfigor() = default;

    ViewerConfigor ViewerConfigor::LoadConfigure(const std::string &filename) {
        std::ifstream file(filename);
        cereal::JSONInputArchive archive(file);
        ViewerConfigor configor;
        archive(cereal::make_nvp("Configor", configor));
        return configor;
    }

    bool ViewerConfigor::SaveConfigure(const std::string &filename) {
        std::ofstream file(filename);
        cereal::JSONOutputArchive archive(file);
        archive(cereal::make_nvp("Configor", *this));
        return true;
    }

    // ------
    // Viewer
    // ------

    std::mutex Viewer::MUTEX = {};

    // --------------
    // public methods
    // --------------
    Viewer::Viewer(ViewerConfigor configor)
            : _thread(nullptr), _configor(std::move(configor)) { InitViewer(); }

    Viewer::Ptr Viewer::Create(const ViewerConfigor &configor) {
        return std::make_shared<Viewer>(configor);
    }

    Viewer::Viewer(const std::string &configPath) : Viewer(ViewerConfigor::LoadConfigure(configPath)) {}

    Viewer::~Viewer() {
        if (_thread != nullptr) {
            _thread->join();
        }
    }

    void Viewer::RunInSingleThread() {
        if (std::filesystem::exists(_configor.Window.ScreenShotSaveDir)) {
            std::cout
                    << "\033[92m\033[3m[Viewer] adjust the camera and press 's' key to save the current scene.\033[0m"
                    << std::endl;
        }
        Run();
    }

    void Viewer::RunInMultiThread() {
        if (std::filesystem::exists(_configor.Window.ScreenShotSaveDir)) {
            std::cout
                    << "\033[92m\033[3m[Viewer] adjust the camera and press 's' key to save the current scene.\033[0m"
                    << std::endl;
        }
        this->_thread = std::make_shared<std::thread>([this]() { Run(); });
    }

    // -----------------
    // protected methods
    // -----------------
    void Viewer::InitViewer() {
        // create a window and bind its context to the main thread
        pangolin::CreateWindowAndBind(_configor.Window.Name, _configor.Window.Width, _configor.Window.height);

        // unset the current context from the main thread
        pangolin::GetBoundWindow()->RemoveCurrent();

        AddEntity(Coordinate::Create(Posef()));
        Eigen::Vector3f v1, v2;
        switch (_configor.Grid.PlaneId % 3) {
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

        for (int i = 0; i < _configor.Grid.CellCount + 1; ++i) {
            Eigen::Vector3f p1 = s3 + i * v1;
            Eigen::Vector3f p2 = p1 + _configor.Grid.CellCount * v2;
            AddEntity(Line::Create(p1, p2, DefaultLineSize, _configor.Grid.Color));
        }

        for (int i = 0; i < _configor.Grid.CellCount + 1; ++i) {
            Eigen::Vector3f p1 = s3 + i * v2;
            Eigen::Vector3f p2 = p1 + _configor.Grid.CellCount * v1;
            AddEntity(Line::Create(p1, p2, DefaultLineSize, _configor.Grid.Color));
        }
    }

    void Viewer::Run() {
        // fetch the context and bind it to this thread
        pangolin::BindToContext(_configor.Window.Name);

        // we manually need to restore the properties of the context
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // Define Projection and initial ModelView matrix
        const auto &c = _configor.Camera;
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(c.Width, c.Height, c.Fx, c.Fy, c.Cx, c.Cy, c.Near, c.Far),
                pangolin::ModelViewLookAt(
                        ExpandStdVec3(_configor.Camera.InitPos),
                        ExpandStdVec3(_configor.Camera.InitViewPoint), pangolin::AxisZ
                )
        );

        // Create Interactive View in window
        pangolin::Handler3D handler(s_cam);
        pangolin::View &d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -static_cast<double>(_configor.Camera.Width) / _configor.Camera.Height)
                .SetHandler(&handler);

        pangolin::RegisterKeyPressCallback('s', [this] { KeyBoardCallBack(); });

        while (!pangolin::ShouldQuit()) {

            // Clear screen and activate view to render into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);
            glClearColor(
                    _configor.Window.BackGroundColor.r, _configor.Window.BackGroundColor.g,
                    _configor.Window.BackGroundColor.b, _configor.Window.BackGroundColor.a
            );

            // -------
            // drawing
            // -------
            {
                LOCKER_VIEWER
                for (const auto &item: _entities) { item.second->Draw(); }
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

    void Viewer::KeyBoardCallBack() const {
        std::int64_t curTimeStamp = std::chrono::system_clock::now().time_since_epoch().count();
        const std::string filename = _configor.Window.ScreenShotSaveDir + "/" + std::to_string(curTimeStamp) + ".png";
        pangolin::SaveWindowOnRender(filename);
        std::cout << "\033[92m\033[3m[Viewer] the scene shot is saved to path: '"
                  << filename << "\033[0m" << std::endl;
    }

    // ----------------
    // process Entities
    // ----------------
    std::size_t Viewer::AddEntity(const Entity::Ptr &entity) {
        LOCKER_VIEWER
        _entities.insert({entity->GetId(), entity});
        return entity->GetId();
    }

    bool Viewer::RemoveEntity(std::size_t id) {
        LOCKER_VIEWER
        return _entities.erase(id) == 1;
    }

    std::vector<std::size_t> Viewer::AddEntity(const std::vector<Entity::Ptr> &entities) {
        LOCKER_VIEWER
        std::vector<std::size_t> ids(entities.size());
        for (int i = 0; i < entities.size(); ++i) {
            const Entity::Ptr &entity = entities.at(i);
            _entities.insert({entity->GetId(), entity});
            ids.at(i) = entity->GetId();
        }
        return ids;
    }

    bool Viewer::RemoveEntity(const std::vector<std::size_t> &ids) {
        LOCKER_VIEWER
        bool b = false;
        for (const auto &id: ids) {
            b = (_entities.erase(id) == 1) && b;
        }
        return b;
    }

    bool Viewer::RemoveEntity() {
        LOCKER_VIEWER
        _entities.clear();
        return true;
    }

    void Viewer::Save(const std::string &filename, bool binaryMode) const {
        std::ofstream file(filename);
        if (binaryMode) {
            cereal::BinaryOutputArchive ar(file);
            LOCKER_VIEWER
            ar(*this);
        } else {
            cereal::JSONOutputArchive ar(file);
            LOCKER_VIEWER
            ar(*this);
        }
    }

    Viewer::Ptr Viewer::Load(const std::string &filename, bool binaryMode) {
        auto viewer = std::make_shared<Viewer>('2');
        std::ifstream file(filename);
        if (binaryMode) {
            cereal::BinaryInputArchive ar(file);
            ar(*viewer);
        } else {
            cereal::JSONInputArchive ar(file);
            ar(*viewer);
        }
        viewer->InitViewer();
        return viewer;
    }
}
