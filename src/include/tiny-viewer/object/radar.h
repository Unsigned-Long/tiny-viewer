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

#ifndef TINY_VIEWER_RADAR_H
#define TINY_VIEWER_RADAR_H

#include "tiny-viewer/entity/entity.h"
#include "tiny-viewer/entity/coordinate.h"

namespace ns_viewer {

struct Radar : public Entity {
public:
    using Ptr = std::shared_ptr<Radar>;

protected:
    Coordinate coord;

    std::array<Eigen::Vector3f, 8> tVerts;
    Eigen::Vector3f cenVert;
    std::array<Eigen::Vector3f, 4> bVerts;

    Colour color;

public:
    explicit Radar(const Posef &pose,
                   float size = DefaultRadarSize,
                   const Colour &color = Colour(0.2f, 0.2f, 0.2f));

    explicit Radar(const Posef &pose, const Colour &color, float size = DefaultRadarSize);

    static Ptr Create(const Posef &pose,
                      float size = DefaultRadarSize,
                      const Colour &color = Colour(0.2f, 0.2f, 0.2f));

    static Ptr Create(const Posef &pose, const Colour &color, float size = DefaultRadarSize);

    ~Radar() override = default;

    void Draw() const override;

    Radar() = default;

public:
    template <class Archive>
    void serialize(Archive &archive) {
        Entity::serialize(archive);
        archive(CEREAL_NVP(coord), CEREAL_NVP(tVerts), CEREAL_NVP(cenVert), CEREAL_NVP(bVerts),
                CEREAL_NVP(color));
    }
};
}  // namespace ns_viewer

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Radar, "Radar")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Radar)

#endif  // TINY_VIEWER_RADAR_H
