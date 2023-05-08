//
// Created by csl on 10/22/22.
//

#include "tiny-viewer/viewer.h"
#include "filesystem"
#include "pangolin/display/display.h"
#include "pangolin/display/view.h"
#include "pangolin/handler/handler.h"
#include "pangolin/gl/gl.h"
#include "pangolin/gl/gldraw.h"

namespace ns_viewer {

    // --------------
    // public methods
    // --------------
    Viewer::Viewer(ViewerConfigor configor)
            : _thread(nullptr), _configor(std::move(configor)) { InitViewer(); }

    Viewer::Ptr Viewer::Create(const ViewerConfigor &configor) {
        return std::make_shared<Viewer>(configor);
    }

    Viewer::~Viewer() {
        if (_thread != nullptr) {
            _thread->join();
        }
    }

    void Viewer::RunInSingleThread() {
        if (std::filesystem::exists(_configor.ScreenShotSaveDir)) {
            std::cout
                    << "\033[92m\033[3m[Viewer] adjust the camera and press 's' key to save the current scene.\033[0m"
                    << std::endl;
        }
        Run();
    }

    void Viewer::RunInMultiThread() {
        if (std::filesystem::exists(_configor.ScreenShotSaveDir)) {
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
        pangolin::CreateWindowAndBind(_configor.WinName, 640 * 3, 480 * 3);

        // enable depth
        glEnable(GL_DEPTH_TEST);

        // unset the current context from the main thread
        pangolin::GetBoundWindow()->RemoveCurrent();

        AddCoordinate(Coordinate(Posef()));
    }

    void Viewer::Run() {
        // fetch the context and bind it to this thread
        pangolin::BindToContext(_configor.WinName);

        // we manually need to restore the properties of the context
        glEnable(GL_DEPTH_TEST);

        // Define Projection and initial ModelView matrix
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
                pangolin::ModelViewLookAt(2, 2, 2, 0, 0, 0, pangolin::AxisZ)
        );

        // Create Interactive View in window
        pangolin::Handler3D handler(s_cam);
        pangolin::View &d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
                .SetHandler(&handler);

        pangolin::RegisterKeyPressCallback('s', [this] { KeyBoardCallBack(); });

        while (!pangolin::ShouldQuit()) {

            // Clear screen and activate view to render into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);
            glClearColor(
                    _configor.BackGroundColor.r, _configor.BackGroundColor.g,
                    _configor.BackGroundColor.b, _configor.BackGroundColor.a
            );

            // -------
            // drawing
            // -------
            for (const auto &item: _lines) { item.second.Draw(); }
            for (const auto &item: _coords) { item.second.Draw(); }
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
        const std::string filename = _configor.ScreenShotSaveDir + "/" + std::to_string(curTimeStamp) + ".png";
        pangolin::SaveWindowOnRender(filename);
        std::cout << "\033[92m\033[3m[Viewer] the scene shot is saved to path: '"
                  << filename << "\033[0m" << std::endl;
    }

    // ------------
    // Add Entities
    // ------------
    void Viewer::AddLine(const Line &l) {
        InsertEntityPair(_lines, l);
    }

    void Viewer::AddCoordinate(const Coordinate &coordinate) {
        InsertEntityPair(_coords, coordinate);
    }
}
