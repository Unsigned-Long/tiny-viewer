// Tiny-Viewer: Tiny But Powerful Graphic Entity And Object Visualization
// Copyright 2024, the School of Geodesy and Geomatics (SGG), Wuhan University, China
// https://github.com/Unsigned-Long/tiny-viewer.git
//
// Author: Shuolong Chen (shlchen@whu.edu.cn)
// GitHub: https://github.com/Unsigned-Long
//  ORCID: 0000-0002-5283-9057
//
// Purpose: See .h/.hpp file.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * The names of its contributors can not be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef TINY_VIEWER_VIEWER_CONFIGOR_H
#define TINY_VIEWER_VIEWER_CONFIGOR_H

#include "cereal/cereal.hpp"
#include "tiny-viewer/entity/utils.h"
#include "pangolin/display/view.h"

namespace ns_viewer {
// GlSl Graphics shader program for display
enum class ObjRenderMode { UV = 0, TEX, COLOR, NORMAL, MATCAP, VERTEX, NUM_MODES };

struct Window {
    std::string name = "Tiny Viewer";
    Colour backGroundColor = Colour::White();
    int width = 640 * 2;
    int height = 480 * 2;
    pangolin::Layout layout = pangolin::LayoutEqual;

public:
    template <class Archive>
    void serialize(Archive &ar) {
        ar(CEREAL_NVP(name), CEREAL_NVP(backGroundColor), CEREAL_NVP(width), CEREAL_NVP(height),
           CEREAL_NVP(layout));
    }
};

struct Output {
    std::string dataOutputPath;

public:
    template <class Archive>
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
    template <class Archive>
    void serialize(Archive &ar) {
        ar(CEREAL_NVP(width), CEREAL_NVP(height), CEREAL_NVP(fx), CEREAL_NVP(fy), CEREAL_NVP(cx),
           CEREAL_NVP(cy), CEREAL_NVP(near), CEREAL_NVP(far), CEREAL_NVP(initPos),
           CEREAL_NVP(initViewPoint));
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
    template <class Archive>
    void serialize(Archive &ar) {
        ar(CEREAL_NVP(showGrid), CEREAL_NVP(showIdentityCoord), CEREAL_NVP(cellCount),
           CEREAL_NVP(cellSize), CEREAL_NVP(planePos), CEREAL_NVP(color));
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
    template <class Archive>
    void serialize(Archive &ar) {
        ar(CEREAL_NVP(window), CEREAL_NVP(output), CEREAL_NVP(camera), CEREAL_NVP(grid),
           CEREAL_NVP(render));
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
    template <class Archive>
    void serialize(Archive &ar) {
        ar(CEREAL_NVP(window), CEREAL_NVP(output), CEREAL_NVP(camera), CEREAL_NVP(grid),
           CEREAL_NVP(subWinNames), CEREAL_NVP(render));
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
}  // namespace ns_viewer

#endif  // TINY_VIEWER_VIEWER_CONFIGOR_H
