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

#ifndef TINY_VIEWER_VIEWER_H
#define TINY_VIEWER_VIEWER_H

#include "iostream"
#include "memory"
#include "mutex"
#include "pangolin/gl/opengl_render_state.h"
#include "thread"
#include "tiny-viewer/entity/arrow.h"
#include "tiny-viewer/entity/cone.h"
#include "tiny-viewer/entity/coordinate.h"
#include "tiny-viewer/entity/cube.h"
#include "tiny-viewer/entity/cylinder.h"
#include "tiny-viewer/entity/line.h"
#include "tiny-viewer/entity/point_cloud.hpp"
#include "tiny-viewer/entity/polygon.h"
#include "tiny-viewer/entity/path.h"
#include "tiny-viewer/object/camera.h"
#include "tiny-viewer/object/imu.h"
#include "tiny-viewer/object/lidar.h"
#include "tiny-viewer/object/plane.h"
#include "tiny-viewer/object/surfel.h"
#include "tiny-viewer/object/aligned_cloud.hpp"
#include "tiny-viewer/object/radar.h"
#include "tiny-viewer/object/landmark.h"
#include "tiny-viewer/core/viewer_configor.h"
#include "utility"
#include <pangolin/geometry/glgeometry.h>

namespace ns_viewer {

#define LOCKER_VIEWER std::unique_lock<std::mutex> viewerLock(Viewer::MUTEX);

class Viewer {
public:
    using Ptr = std::shared_ptr<Viewer>;

protected:
    ViewerConfigor _configor;
    std::shared_ptr<std::thread> _thread;

public:
    static std::mutex MUTEX;

protected:
    std::unordered_map<std::size_t, Entity::Ptr> _entities;
    pangolin::OpenGlRenderState _camView;
    bool _isActive;
    pangolin::Viewport _viewport;

    std::unordered_map<std::size_t, pangolin::Geometry> _geometry;

public:
    explicit Viewer(ViewerConfigor configor = ViewerConfigor());

    explicit Viewer(const std::string &configPath);

    static Ptr Create(const ViewerConfigor &configor = ViewerConfigor());

    static Ptr Create(const std::string &configPath);

    // used for load viewer from file
    explicit Viewer(char)
        : _thread(nullptr),
          _isActive(false) {}

    virtual ~Viewer();

    void RunInSingleThread();

    void RunInMultiThread();

    std::size_t AddEntity(const Entity::Ptr &entity);

    std::size_t AddObjEntity(const std::string &filename);

    void RemoveObjEntity(std::size_t id);

    std::vector<std::size_t> AddEntity(const std::vector<Entity::Ptr> &entities);

    bool RemoveEntity(std::size_t id);

    bool RemoveEntity(const std::vector<std::size_t> &ids);

    bool RemoveEntity();

    void SetCamView(const pangolin::OpenGlRenderState &camView);

    void SetCamView(const std::string &filename);

    void SetCamView(Posef T_CamToWorld);

    void Save(const std::string &filename, bool binaryMode = true) const;

    static Ptr Load(const std::string &filename, bool binaryMode = true);

    ViewerConfigor &GetConfigor();

    bool IsActive() const;

    // negative number means waiting forever
    bool WaitForActive(double waitTimeMs = -1.0) const;

    const pangolin::Viewport &GetViewport() const;

protected:
    void InitViewer(bool initCamViewFromConfigor);

    void Run();

    void SaveScreenShotCallBack() const;

    void SaveCameraCallBack() const;

    void SaveViewerCallBack() const;

    void VideoRecordCallBack() const;

public:
    template <class Archive>
    void serialize(Archive &archive) {
        archive(cereal::make_nvp("configor", _configor), cereal::make_nvp("entities", _entities),
                cereal::make_nvp("camera_view", _camView));
    }
};
}  // namespace ns_viewer

#endif  // TINY_VIEWER_VIEWER_H
