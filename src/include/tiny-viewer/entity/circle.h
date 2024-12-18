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

#ifndef TINY_VIEWER_CIRCLE_H
#define TINY_VIEWER_CIRCLE_H

#include "tiny-viewer/entity/coordinate.h"
#include "tiny-viewer/entity/entity.h"
#include "tiny-viewer/entity/polygon.h"

namespace ns_viewer {
template <typename>
struct Pose;
using Posef = Pose<float>;

struct Circle : public Entity {
public:
    using Ptr = std::shared_ptr<Circle>;

protected:
    Coordinate coord;
    Polygon poly;
    bool drawCoord{};

public:
    explicit Circle(const Posef &pose,
                    float radius,
                    bool lineMode = true,
                    bool drawCoord = true,
                    const Colour &color = GetUniqueColour(),
                    pangolin::AxisDirection nAxis = pangolin::AxisZ,
                    int ptCount = 16);

    static Ptr Create(const Posef &pose,
                      float radius,
                      bool lineMode = true,
                      bool drawCoord = true,
                      const Colour &color = GetUniqueColour(),
                      pangolin::AxisDirection nAxis = pangolin::AxisZ,
                      int ptCount = 16);

    ~Circle() override;

    void Draw() const override;

    Circle() = default;

public:
    template <class Archive>
    void serialize(Archive &archive) {
        Entity::serialize(archive);
        archive(CEREAL_NVP(coord), CEREAL_NVP(poly));
    }
};
}  // namespace ns_viewer

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Circle, "Circle")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Circle)

#endif  // TINY_VIEWER_CIRCLE_H
