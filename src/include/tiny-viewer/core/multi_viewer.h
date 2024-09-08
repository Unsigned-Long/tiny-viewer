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

#ifndef TINY_VIEWER_MULTI_VIEWER_H
#define TINY_VIEWER_MULTI_VIEWER_H

#include "iostream"
#include "memory"
#include "mutex"
#include "pangolin/gl/opengl_render_state.h"
#include "thread"
#include "tiny-viewer/core/viewer_configor.h"
#include <pangolin/geometry/glgeometry.h>

namespace ns_viewer {

#define LOCKER_MULTI_VIEWER std::unique_lock<std::mutex> viewerLock(MultiViewer::MUTEX);

struct Entity;
using EntityPtr = std::shared_ptr<Entity>;
template <class ScalarType>
struct Pose;
using Posef = Pose<float>;

class MultiViewer {
public:
    using Ptr = std::shared_ptr<MultiViewer>;

protected:
    MultiViewerConfigor _configor;
    std::shared_ptr<std::thread> _thread;

public:
    static std::mutex MUTEX;

protected:
    // sub window name, entities
    std::unordered_map<std::string, std::unordered_map<std::size_t, EntityPtr>> _entities;
    std::unordered_map<std::string, pangolin::OpenGlRenderState> _camView;
    bool _isActive;

    std::unordered_map<std::string, std::unordered_map<std::size_t, pangolin::Geometry>>
        _geometries;

public:
    explicit MultiViewer(MultiViewerConfigor configor);

    explicit MultiViewer(const std::string &configPath);

    static Ptr Create(const MultiViewerConfigor &configor);

    static Ptr Create(const std::string &configPath);

    // used for load viewer from file
    explicit MultiViewer(char)
        : _configor({}),
          _thread(nullptr),
          _isActive(false) {}

    virtual ~MultiViewer();

    void RunInSingleThread();

    void RunInMultiThread();

    std::size_t AddEntity(const EntityPtr &entity, const std::string &subWinName);

    std::size_t AddObjEntity(const std::string &filename, const std::string &subWinName);

    void RemoveObjEntity(std::size_t id, const std::string &subWinName);

    std::vector<std::size_t> AddEntity(const std::vector<EntityPtr> &entities,
                                       const std::string &subWinName);

    bool RemoveEntity(std::size_t id, const std::string &subWinName);

    bool RemoveEntity(const std::vector<std::size_t> &ids, const std::string &subWinName);

    bool RemoveEntity(const std::string &subWinName);

    bool RemoveEntity();

    void SetCamView(const pangolin::OpenGlRenderState &camView, const std::string &subWinName);

    void SetCamView(const std::string &filename, const std::string &subWinName);

    void SetCamView(Posef T_CamToWorld, const std::string &subWinName);

    void Save(const std::string &filename, bool binaryMode = true) const;

    static Ptr Load(const std::string &filename, bool binaryMode = true);

    MultiViewerConfigor &GetConfigor();

    bool IsActive() const;

    bool WaitForActive(double waitTimeMs) const;

protected:
    void InitMultiViewer(bool initCamViewFromConfigor);

    void Run();

    void SaveScreenShotCallBack() const;

    void SaveCameraCallBack() const;

    void SaveMultiViewerCallBack() const;

    void VideoRecordCallBack() const;

public:
    template <class Archive>
    void serialize(Archive &archive) {
        archive(cereal::make_nvp("configor", _configor), cereal::make_nvp("entities", _entities),
                cereal::make_nvp("camera_view", _camView));
    }
};
}  // namespace ns_viewer

#endif  // TINY_VIEWER_MULTI_VIEWER_H
