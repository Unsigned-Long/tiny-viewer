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

#include "tiny-viewer/core/viewer_configor.h"

namespace ns_viewer {

// --------------
// ViewerConfigor
// --------------
ViewerConfigor::ViewerConfigor(const std::string &winName)
    : render(ObjRenderMode::NORMAL) {
    window.name = winName;
}

ViewerConfigor ViewerConfigor::LoadConfigure(const std::string &filename) {
    std::ifstream file(filename);
    ViewerConfigor configor;
    cereal::JSONInputArchive archive(file);
    archive(cereal::make_nvp("Configor", configor));
    return configor;
}

bool ViewerConfigor::SaveConfigure(const std::string &filename) {
    std::ofstream file(filename);
    cereal::JSONOutputArchive archive(file);
    archive(cereal::make_nvp("Configor", *this));
    return true;
}

ViewerConfigor &ViewerConfigor::WithWinName(const std::string &winName) {
    window.name = winName;
    return *this;
}

ViewerConfigor &ViewerConfigor::WithScreenShotSaveDir(const std::string &dir) {
    output.dataOutputPath = dir;
    return *this;
}

// -------------------
// MultiViewerConfigor
// -------------------

MultiViewerConfigor::MultiViewerConfigor(const std::vector<std::string> &subWinNames,
                                         const std::string &winName)
    : subWinNames(subWinNames),
      render(ObjRenderMode::NORMAL) {
    window.name = winName;
    for (const auto &name : this->subWinNames) {
        camera.insert({name, {}});
        grid.insert({name, {}});
    }
    window.width = static_cast<int>(window.width * 2.0 * 0.8);
    window.height = static_cast<int>(window.height * 0.8);
}

MultiViewerConfigor MultiViewerConfigor::LoadConfigure(const std::string &filename) {
    std::ifstream file(filename);
    MultiViewerConfigor configor({});
    cereal::JSONInputArchive archive(file);
    archive(cereal::make_nvp("Configor", configor));
    return configor;
}

bool MultiViewerConfigor::SaveConfigure(const std::string &filename) {
    std::ofstream file(filename);
    cereal::JSONOutputArchive archive(file);
    archive(cereal::make_nvp("Configor", *this));
    return true;
}

MultiViewerConfigor &MultiViewerConfigor::WithWinName(const std::string &winName) {
    window.name = winName;
    return *this;
}

MultiViewerConfigor &MultiViewerConfigor::WithScreenShotSaveDir(const std::string &dir) {
    output.dataOutputPath = dir;
    return *this;
}

}  // namespace ns_viewer
