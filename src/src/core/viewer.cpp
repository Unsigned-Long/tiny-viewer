//
// Created by csl on 10/22/22.
//

#include "tiny-viewer/core/viewer.h"
#include "filesystem"
#include "pangolin/display/display.h"
#include "pangolin/display/view.h"
#include "pangolin/handler/handler.h"
#include "pangolin/gl/gldraw.h"
#include "tiny-viewer/core/shader.h"
#include "tiny-viewer/core/rendertree.h"

namespace ns_viewer {

    // ------
    // Viewer
    // ------

    std::mutex Viewer::MUTEX = {};

    // --------------
    // public methods
    // --------------
    Viewer::Viewer(ViewerConfigor configor)
            : _configor(std::move(configor)), _thread(nullptr), _isActive(false) { InitViewer(true); }

    Viewer::Ptr Viewer::Create(const ViewerConfigor &configor) {
        return std::make_shared<Viewer>(configor);
    }

    Viewer::Ptr Viewer::Create(const std::string &configPath) {
        return std::make_shared<Viewer>(configPath);
    }

    Viewer::Viewer(const std::string &configPath) : Viewer(ViewerConfigor::LoadConfigure(configPath)) {}

    Viewer::~Viewer() {
        if (_thread != nullptr) {
            _thread->join();
        }
    }

    void Viewer::RunInSingleThread() {
        _isActive = true;
        Run();
    }

    void Viewer::RunInMultiThread() {
        _isActive = true;
        this->_thread = std::make_shared<std::thread>([this]() { Run(); });
    }

    // -----------------
    // protected methods
    // -----------------
    void Viewer::InitViewer(bool initCamViewFromConfigor) {
        // create a window and bind its context to the main thread
        pangolin::CreateWindowAndBind(_configor.window.name, _configor.window.width, _configor.window.height);

        // unset the current context from the main thread
        pangolin::GetBoundWindow()->RemoveCurrent();

        if (initCamViewFromConfigor) {
            // Define Projection and initial ModelView matrix
            const auto &c = _configor.camera;
            _camView = pangolin::OpenGlRenderState(
                    pangolin::ProjectionMatrix(c.width, c.height, c.fx, c.fy, c.cx, c.cy, c.near, c.far),
                    pangolin::ModelViewLookAt(
                            ExpandStdVec3(_configor.camera.initPos),
                            ExpandStdVec3(_configor.camera.initViewPoint), pangolin::AxisZ
                    )
            );

            if (_configor.grid.showIdentityCoord) {
                AddEntity(Coordinate::Create(Posef()));
            }

            if (_configor.grid.showGrid) {
                Eigen::Vector3f v1, v2;
                switch (_configor.grid.planePos % 3) {
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
                v1 *= _configor.grid.cellSize;
                v2 *= _configor.grid.cellSize;
                Eigen::Vector3f s1 = -v1 * _configor.grid.cellCount * 0.5;
                Eigen::Vector3f s2 = -v2 * _configor.grid.cellCount * 0.5;
                Eigen::Vector3f s3 = s1 + s2;

                for (int i = 0; i < _configor.grid.cellCount + 1; ++i) {
                    Eigen::Vector3f p1 = s3 + i * v1;
                    Eigen::Vector3f p2 = p1 + _configor.grid.cellCount * v2;
                    AddEntity(Line::Create(p1, p2, DefaultLineSize, _configor.grid.color));
                }

                for (int i = 0; i < _configor.grid.cellCount + 1; ++i) {
                    Eigen::Vector3f p1 = s3 + i * v2;
                    Eigen::Vector3f p2 = p1 + _configor.grid.cellCount * v1;
                    AddEntity(Line::Create(p1, p2, DefaultLineSize, _configor.grid.color));
                }
            }
        }
    }

    void Viewer::Run() {
        // fetch the context and bind it to this thread
        pangolin::BindToContext(_configor.window.name);

        // we manually need to restore the properties of the context
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // Create Interactive View in window
        pangolin::Handler3D handler(_camView);
        pangolin::View &d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -static_cast<double>(_configor.camera.width) / _configor.camera.height)
                .SetHandler(&handler);

        if (std::filesystem::exists(_configor.output.dataOutputPath)) {

            pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 's', [this] { SaveScreenShotCallBack(); });
            pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'c', [this] { SaveCameraCallBack(); });
            pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'v', [this] { SaveViewerCallBack(); });
            pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'r', [this] { VideoRecordCallBack(); });

            std::cout << "\033[92m\033[3m[Viewer] "
                         "press [ctrl+'s'] to save the current scene, "
                         "[ctrl+'c'] for camera view, "
                         "and [ctrl+'v'] for total viewer.\033[0m" << std::endl;
        }

        bool scvrHasBeenRegistered = std::filesystem::exists(_configor.output.dataOutputPath);

        for (const auto &[key, func]: _configor.callBacks) {
            if (key == pangolin::PANGO_CTRL + 's' && scvrHasBeenRegistered) {
                throw std::runtime_error("pangolin::PANGO_CTRL + 's' has been registered as 'SaveScreenShotCallBack'!");
            }
            if (key == pangolin::PANGO_CTRL + 'c' && scvrHasBeenRegistered) {
                throw std::runtime_error("pangolin::PANGO_CTRL + 'c' has been registered as 'SaveCameraCallBack'!");
            }
            if (key == pangolin::PANGO_CTRL + 'v' && scvrHasBeenRegistered) {
                throw std::runtime_error("pangolin::PANGO_CTRL + 'v' has been registered as 'SaveViewerCallBack'!");
            }
            if (key == pangolin::PANGO_CTRL + 'r' && scvrHasBeenRegistered) {
                throw std::runtime_error("pangolin::PANGO_CTRL + 'r' has been registered as 'VideoRecordCallBack'!");
            }
            pangolin::RegisterKeyPressCallback(key, func);
        }

        std::map<std::size_t, RenderNode> roots;
        std::map<std::size_t, Eigen::AlignedBox3f> totalAABBs;
        std::map<std::size_t, std::vector<std::shared_ptr<GlGeomRenderable>>> renderables;

        const std::string ModeNames[] = {
                "SHOW_UV", "SHOW_TEXTURE", "SHOW_COLOR", "SHOW_NORMAL", "SHOW_MATCAP", "SHOW_VERTEX"
        };
        pangolin::GlSlProgram defProg;
        defProg.ClearShaders();
        std::map<std::string, std::string> progDefines;
        for (int i = 0; i < (int) ObjRenderMode::NUM_MODES - 1; ++i) {
            progDefines[ModeNames[i]] = std::to_string((int) this->_configor.render == i);
        }
        defProg.AddShader(pangolin::GlSlAnnotatedShader, pangolin::default_model_shader, progDefines);
        defProg.Link();

        while (!pangolin::ShouldQuit()) {

            // Clear screen and activate view to render into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(_camView);
            glClearColor(
                    _configor.window.backGroundColor.r, _configor.window.backGroundColor.g,
                    _configor.window.backGroundColor.b, _configor.window.backGroundColor.a
            );

            // -------
            // drawing
            // -------
            {
                LOCKER_VIEWER
                for (const auto &item: _entities) { item.second->Draw(); }

                for (const auto &[id, geo]: _geometry) {
                    auto aabb = pangolin::GetAxisAlignedBox(geo);
                    totalAABBs[id].extend(aabb);
                    auto renderable = std::make_shared<GlGeomRenderable>(pangolin::ToGlGeometry(geo), aabb);
                    renderables[id].push_back(renderable);
                    RenderNode::Edge edge = {
                            std::make_shared<SpinTransform>(pangolin::AxisDirection::AxisNone), {renderable, {}}
                    };
                    roots[id].edges.emplace_back(std::move(edge));

                    if (d_cam.IsShown()) {
                        d_cam.Activate();

                        defProg.Bind();
                        render_tree(defProg, roots[id], _camView.GetProjectionMatrix(),
                                    _camView.GetModelViewMatrix(), nullptr);
                        defProg.Unbind();

                        _camView.Apply();
                    }
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

        pangolin::DestroyWindow(_configor.window.name);

        {
            LOCKER_VIEWER
            _isActive = false;
        }
    }

    void Viewer::SaveScreenShotCallBack() const {
        std::int64_t curTimeStamp = std::chrono::system_clock::now().time_since_epoch().count();
        const std::string filename = _configor.output.dataOutputPath + "/" + std::to_string(curTimeStamp) + ".png";
        pangolin::SaveWindowOnRender(filename);
        std::cout << "\033[92m\033[3m[Viewer] the scene shot is saved to path: '"
                  << filename << "\033[0m" << std::endl;
    }

    void Viewer::SaveCameraCallBack() const {
        std::int64_t curTimeStamp = std::chrono::system_clock::now().time_since_epoch().count();
        const std::string filename = _configor.output.dataOutputPath + "/" + std::to_string(curTimeStamp) + ".cam";

        std::ofstream file(filename);
        cereal::JSONOutputArchive ar(file);
        ar(cereal::make_nvp("cam_view", this->_camView));
        std::cout << "\033[92m\033[3m[Viewer] the camera view is saved to path: '"
                  << filename << "\033[0m" << std::endl;
    }

    void Viewer::SaveViewerCallBack() const {
        std::int64_t curTimeStamp = std::chrono::system_clock::now().time_since_epoch().count();
        const std::string filename = _configor.output.dataOutputPath + "/" + std::to_string(curTimeStamp) + ".view";
        this->Save(filename, true);
        std::cout << "\033[92m\033[3m[Viewer] the viewer is saved to path: '"
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
        for (int i = 0; i < static_cast<int>(entities.size()); ++i) {
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
        viewer->InitViewer(false);
        return viewer;
    }

    void Viewer::SetCamView(const pangolin::OpenGlRenderState &camView) {
        this->_camView.SetModelViewMatrix(camView.GetModelViewMatrix());
        this->_camView.SetProjectionMatrix(camView.GetProjectionMatrix());
    }

    void Viewer::SetCamView(const std::string &filename) {
        std::ifstream file(filename);
        cereal::JSONInputArchive ar(file);
        pangolin::OpenGlRenderState camView;
        ar(camView);
        SetCamView(camView);
    }

    void Viewer::VideoRecordCallBack() const {
        // todo: record video
    }

    ViewerConfigor &Viewer::GetConfigor() {
        return _configor;
    }

    void Viewer::SetCamView(Posef T_CamToWorld) {
        Eigen::Vector3f vp(T_CamToWorld.translation + T_CamToWorld.rotation.col(2));
        this->_camView.SetModelViewMatrix(pangolin::ModelViewLookAt(
                ExpandVec3(T_CamToWorld.translation), ExpandVec3(vp), ExpandVec3(-T_CamToWorld.rotation.col(1))
        ));
    }

    bool Viewer::IsActive() const {
        return _isActive;
    }

    std::size_t Viewer::AddObjEntity(const std::string &filename) {
        LOCKER_VIEWER
        static std::size_t id = 0;
        this->_geometry.insert({++id, pangolin::LoadGeometry(filename)});
        return id;
    }

    void Viewer::RemoveObjEntity(std::size_t id) {
        LOCKER_VIEWER
        this->_geometry.erase(id);
    }
}
