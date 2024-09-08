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

#ifndef TINY_VIEWER_CUBE_H
#define TINY_VIEWER_CUBE_H

#include "entity.h"

namespace ns_viewer {
template <typename>
struct Pose;
using Posef = Pose<float>;

struct Cube : public Entity {
public:
    using Ptr = std::shared_ptr<Cube>;

protected:
    Eigen::Vector3f v1;
    Eigen::Vector3f v2;
    Eigen::Vector3f v3;
    Eigen::Vector3f v4;

    Eigen::Vector3f v5;
    Eigen::Vector3f v6;
    Eigen::Vector3f v7;
    Eigen::Vector3f v8;

    bool lineMode{};
    Colour color;

public:
    Cube(const Posef &pose,
         bool lineMode,
         float xWidth = DefaultCubeSize,
         float yWidth = DefaultCubeSize,
         float zWidth = DefaultCubeSize,
         const Colour &color = GetUniqueColour());

    Cube(const Posef &pose,
         bool lineMode,
         const Colour &color,
         float xWidth = DefaultCubeSize,
         float yWidth = DefaultCubeSize,
         float zWidth = DefaultCubeSize);

    static Ptr Create(const Posef &pose,
                      bool lineMode,
                      float xWidth = DefaultCubeSize,
                      float yWidth = DefaultCubeSize,
                      float zWidth = DefaultCubeSize,
                      const Colour &color = GetUniqueColour());

    static Ptr Create(const Posef &pose,
                      bool lineMode,
                      const Colour &color,
                      float xWidth = DefaultCubeSize,
                      float yWidth = DefaultCubeSize,
                      float zWidth = DefaultCubeSize);

    [[nodiscard]] Eigen::Vector3f GetCenter() const;

    [[nodiscard]] std::array<Eigen::Vector3f, 8> GetVertices() const;

    ~Cube() override;

    void Draw() const override;

    Cube() = default;

public:
    template <class Archive>
    void serialize(Archive &archive) {
        Entity::serialize(archive);
        archive(CEREAL_NVP(v1), CEREAL_NVP(v2), CEREAL_NVP(v3), CEREAL_NVP(v4), CEREAL_NVP(v5),
                CEREAL_NVP(v6), CEREAL_NVP(v7), CEREAL_NVP(v8), CEREAL_NVP(color),
                CEREAL_NVP(lineMode));
    }
};
}  // namespace ns_viewer

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Cube, "Cube")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Cube)

#endif  // TINY_VIEWER_CUBE_H
