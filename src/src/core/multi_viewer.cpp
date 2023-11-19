//
// Created by csl on 10/22/22.
//

#include "tiny-viewer/core/multi_viewer.h"
#include "filesystem"
#include "pangolin/display/display.h"
#include "pangolin/display/view.h"
#include "pangolin/handler/handler.h"
#include "pangolin/gl/gldraw.h"
#include "tiny-viewer/core/shader.h"
#include "tiny-viewer/core/rendertree.h"

namespace ns_viewer {

    // -----------
    // MultiViewer
    // -----------

    std::mutex MultiViewer::MUTEX = {};

    // --------------
    // public methods
    // --------------
    MultiViewer::MultiViewer(MultiViewerConfigor configor)
            : _configor(std::move(configor)), _thread(nullptr), _isActive(false) { InitMultiViewer(true); }

    MultiViewer::Ptr MultiViewer::Create(const MultiViewerConfigor &configor) {
        return std::make_shared<MultiViewer>(configor);
    }

    MultiViewer::Ptr MultiViewer::Create(const std::string &configPath) {
        return std::make_shared<MultiViewer>(configPath);
    }

    MultiViewer::MultiViewer(const std::string &configPath)
            : MultiViewer(MultiViewerConfigor::LoadConfigure(configPath)) {}

    MultiViewer::~MultiViewer() {
        if (_thread != nullptr) {
            _thread->join();
        }
    }

    void MultiViewer::RunInSingleThread() {
        _isActive = true;
        Run();
    }

    void MultiViewer::RunInMultiThread() {
        _isActive = true;
        this->_thread = std::make_shared<std::thread>([this]() { Run(); });
    }

    // -----------------
    // protected methods
    // -----------------
    void MultiViewer::InitMultiViewer(bool initCamViewFromConfigor) {
        // create a window and bind its context to the main thread
        pangolin::CreateWindowAndBind(_configor.window.name, _configor.window.width, _configor.window.height);

        // unset the current context from the main thread
        pangolin::GetBoundWindow()->RemoveCurrent();

        if (initCamViewFromConfigor) {
            for (const auto &name: _configor.subWinNames) {

                // Define Projection and initial ModelView matrix
                const auto &c = _configor.camera.at(name);
                _camView.insert({name, pangolin::OpenGlRenderState(
                        pangolin::ProjectionMatrix(c.width, c.height, c.fx, c.fy, c.cx, c.cy, c.near, c.far),
                        pangolin::ModelViewLookAt(
                                ExpandStdVec3(c.initPos), ExpandStdVec3(c.initViewPoint), pangolin::AxisZ
                        ))});
                _entities.insert({name, std::unordered_map<std::size_t, Entity::Ptr>{}});
                geometry.insert({name, std::unordered_map<std::size_t, pangolin::Geometry>{}});

                const auto &grid = _configor.grid.at(name);
                if (grid.showIdentityCoord) {
                    AddEntity(Coordinate::Create(Posef()), name);
                }

                if (grid.showGrid) {
                    Eigen::Vector3f v1, v2;
                    switch (grid.planePos % 3) {
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
                    v1 *= grid.cellSize;
                    v2 *= grid.cellSize;
                    Eigen::Vector3f s1 = -v1 * grid.cellCount * 0.5;
                    Eigen::Vector3f s2 = -v2 * grid.cellCount * 0.5;
                    Eigen::Vector3f s3 = s1 + s2;

                    for (int i = 0; i < grid.cellCount + 1; ++i) {
                        Eigen::Vector3f p1 = s3 + i * v1;
                        Eigen::Vector3f p2 = p1 + grid.cellCount * v2;
                        AddEntity(Line::Create(p1, p2, DefaultLineSize, grid.color), name);
                    }

                    for (int i = 0; i < grid.cellCount + 1; ++i) {
                        Eigen::Vector3f p1 = s3 + i * v2;
                        Eigen::Vector3f p2 = p1 + grid.cellCount * v1;
                        AddEntity(Line::Create(p1, p2, DefaultLineSize, grid.color), name);
                    }
                }

            }
        }
    }

    void MultiViewer::Run() {
        // fetch the context and bind it to this thread
        pangolin::BindToContext(_configor.window.name);

        // we manually need to restore the properties of the context
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // Create Interactive View in window
        std::unordered_map<std::string, pangolin::View &> d_cam;

        auto &display = pangolin::Display("multi").SetBounds(0.0, 1.0, 0.0, 1.0).SetLayout(pangolin::LayoutEqual);

        for (const auto &name: _configor.subWinNames) {
            const auto &c = _configor.camera.at(name);
            pangolin::View &cam = pangolin::Display(name)
                    .SetAspect(static_cast<double>(c.width) / c.height)
                    .SetHandler(new pangolin::Handler3D(_camView.at(name)));
            d_cam.insert({name, cam});
            display.AddDisplay(cam);
        }

        if (std::filesystem::exists(_configor.output.dataOutputPath)) {

            pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 's', [this] { SaveScreenShotCallBack(); });
            pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'c', [this] { SaveCameraCallBack(); });
            pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'v', [this] { SaveMultiViewerCallBack(); });
            pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'r', [this] { VideoRecordCallBack(); });

            std::cout << "\033[92m\033[3m[MultiViewer] "
                         "press [ctrl+'s'] to save the current scene, "
                         "[ctrl+'c'] for camera view, "
                         "and [ctrl+'v'] for total viewer.\033[0m" << std::endl;
        }

        std::map<std::string, std::map<std::size_t, RenderNode>> roots;
        std::map<std::string, std::map<std::size_t, Eigen::AlignedBox3f>> totalAABBs;
        std::map<std::string, std::map<std::size_t, std::vector<std::shared_ptr<GlGeomRenderable>>>> renderables;

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

            glClearColor(
                    _configor.window.backGroundColor.r, _configor.window.backGroundColor.g,
                    _configor.window.backGroundColor.b, _configor.window.backGroundColor.a
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
                for (const auto &[name, item]: geometry) {
                    d_cam.at(name).Activate(_camView.at(name));
                    for (const auto &[id, geo]: item) {
                        auto aabb = pangolin::GetAxisAlignedBox(geo);
                        totalAABBs[name][id].extend(aabb);
                        auto renderable = std::make_shared<GlGeomRenderable>(pangolin::ToGlGeometry(geo), aabb);
                        renderables[name][id].push_back(renderable);
                        RenderNode::Edge edge = {
                                std::make_shared<SpinTransform>(pangolin::AxisDirection::AxisNone), {renderable, {}}
                        };
                        roots[name][id].edges.emplace_back(std::move(edge));

                        if (d_cam.at(name).IsShown()) {
                            d_cam.at(name).Activate();

                            defProg.Bind();
                            render_tree(defProg, roots[name][id], _camView.at(name).GetProjectionMatrix(),
                                        _camView.at(name).GetModelViewMatrix(), nullptr);
                            defProg.Unbind();

                            _camView.at(name).Apply();
                        }
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
            LOCKER_MULTI_VIEWER
            _isActive = false;
        }
    }

    void MultiViewer::SaveScreenShotCallBack() const {
        std::int64_t curTimeStamp = std::chrono::system_clock::now().time_since_epoch().count();
        const std::string filename = _configor.output.dataOutputPath + "/" + std::to_string(curTimeStamp) + ".png";
        pangolin::SaveWindowOnRender(filename);
        std::cout << "\033[92m\033[3m[MultiViewer] the scene shot is saved to path: '"
                  << filename << "\033[0m" << std::endl;
    }

    void MultiViewer::SaveCameraCallBack() const {
        std::int64_t curTimeStamp = std::chrono::system_clock::now().time_since_epoch().count();
        const std::string filename = _configor.output.dataOutputPath + "/" + std::to_string(curTimeStamp) + ".cam";

        std::ofstream file(filename);
        cereal::JSONOutputArchive ar(file);
        ar(cereal::make_nvp("cam_view", this->_camView));
        std::cout << "\033[92m\033[3m[MultiViewer] the camera view is saved to path: '"
                  << filename << "\033[0m" << std::endl;
    }

    void MultiViewer::SaveMultiViewerCallBack() const {
        std::int64_t curTimeStamp = std::chrono::system_clock::now().time_since_epoch().count();
        const std::string filename = _configor.output.dataOutputPath + "/" + std::to_string(curTimeStamp) + ".view";
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

    bool MultiViewer::RemoveEntity(const std::string &subWinName) {
        LOCKER_MULTI_VIEWER
        _entities.at(subWinName).clear();
        return true;
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

    MultiViewerConfigor &MultiViewer::GetConfigor() {
        return _configor;
    }

    void MultiViewer::SetCamView(Posef T_CamToWorld, const std::string &subWinName) {
        Eigen::Vector3f vp(T_CamToWorld.translation + T_CamToWorld.rotation.col(2));
        this->_camView.at(subWinName).SetModelViewMatrix(pangolin::ModelViewLookAt(
                ExpandVec3(T_CamToWorld.translation), ExpandVec3(vp), ExpandVec3(-T_CamToWorld.rotation.col(1))
        ));
    }

    bool MultiViewer::IsActive() const {
        return _isActive;
    }

    std::size_t MultiViewer::AddObjEntity(const std::string &filename, const std::string &subWinName) {
        LOCKER_MULTI_VIEWER
        static std::size_t id = 0;
        this->geometry.at(subWinName).insert({++id, pangolin::LoadGeometry(filename)});
        return id;
    }

    void MultiViewer::RemoveObjEntity(std::size_t id, const std::string &subWinName) {
        LOCKER_MULTI_VIEWER
        this->geometry.at(subWinName).erase(id);
    }

}
