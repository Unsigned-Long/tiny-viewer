#ifndef TINY_VIEWER_VIEWER_CONFIGOR_H
#define TINY_VIEWER_VIEWER_CONFIGOR_H

#include "cereal/cereal.hpp"
#include "tiny-viewer/entity/utils.h"
#include "pangolin/display/view.h"

namespace ns_viewer {
    // GlSl Graphics shader program for display
    enum class ObjRenderMode {
        UV = 0, TEX, COLOR, NORMAL, MATCAP, VERTEX, NUM_MODES
    };

    struct Window {
        std::string name = "Tiny Viewer";
        Colour backGroundColor = Colour::White();
        int width = 640 * 2;
        int height = 480 * 2;
        pangolin::Layout layout = pangolin::LayoutEqual;

    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(
                    CEREAL_NVP(name), CEREAL_NVP(backGroundColor),
                    CEREAL_NVP(width), CEREAL_NVP(height), CEREAL_NVP(layout)
            );
        }
    };

    struct Output {
        std::string dataOutputPath;

    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(CEREAL_NVP(dataOutputPath));
        }
    };

    struct ConfigCamera {
        int width = 640, height = 480;
        double fx = 420, fy = 420;
        double cx = 320, cy = 240;
        double near = 0.01, far = 100;

        std::vector<float> initPos = {6.0f, 6.0f, 6.0f};
        std::vector<float> initViewPoint = {0.0f, 0.0f, 0.0f};

    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(
                    CEREAL_NVP(width), CEREAL_NVP(height),
                    CEREAL_NVP(fx), CEREAL_NVP(fy),
                    CEREAL_NVP(cx), CEREAL_NVP(cy),
                    CEREAL_NVP(near), CEREAL_NVP(far),
                    CEREAL_NVP(initPos), CEREAL_NVP(initViewPoint)
            );
        }
    };

    struct Grid {
        bool showGrid = true;
        bool showIdentityCoord = true;

        int cellCount = 10;
        float cellSize = 1.0f;
        // 0: xy, 1: yz, 2: zx
        int planePos = 0;

        Colour color = Colour::Black().WithAlpha(0.3f);

    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(
                    CEREAL_NVP(showGrid), CEREAL_NVP(showIdentityCoord),
                    CEREAL_NVP(cellCount), CEREAL_NVP(cellSize),
                    CEREAL_NVP(planePos), CEREAL_NVP(color)
            );
        }
    };

    struct ViewerConfigor {
    public:
        Window window;

        Output output;

        ConfigCamera camera;

        Grid grid;

        ObjRenderMode render;

        std::map<int, std::function<void(void)>> callBacks;
    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(CEREAL_NVP(window), CEREAL_NVP(output), CEREAL_NVP(camera), CEREAL_NVP(grid), CEREAL_NVP(render));
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

    struct MultiViewerConfigor {
    public:
        Window window;

        Output output;

        std::unordered_map<std::string, ConfigCamera> camera;

        std::unordered_map<std::string, Grid> grid;

        std::vector<std::string> subWinNames;

        ObjRenderMode render;

        std::map<int, std::function<void(void)>> callBacks;
    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(CEREAL_NVP(window), CEREAL_NVP(output), CEREAL_NVP(camera),
               CEREAL_NVP(grid), CEREAL_NVP(subWinNames), CEREAL_NVP(render));
        }

    public:
        explicit MultiViewerConfigor(const std::vector<std::string> &subWinNames,
                                     const std::string &winName = "Tiny Viewer");

        // load configure information from the json file
        static MultiViewerConfigor LoadConfigure(const std::string &filename);

        // load configure information from the json file
        bool SaveConfigure(const std::string &filename);

        MultiViewerConfigor &WithWinName(const std::string &winName);

        MultiViewerConfigor &WithScreenShotSaveDir(const std::string &dir);
    };
}


#endif //TINY_VIEWER_VIEWER_CONFIGOR_H
