#include "tiny-viewer/core/viewer_configor.h"

namespace ns_viewer {

    // --------------
    // ViewerConfigor
    // --------------
    ViewerConfigor::ViewerConfigor(const std::string &winName) { window.name = winName; }

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

    MultiViewerConfigor::MultiViewerConfigor(const std::vector<std::string> &subWinNames, const std::string &winName)
            : subWinNames(subWinNames) {
        window.name = winName;
        for (const auto &name: this->subWinNames) {
            camera.insert({name, {}});
            grid.insert({name, {}});
        }
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

}
