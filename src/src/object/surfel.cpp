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

#include "tiny-viewer/object/surfel.h"
#include "pangolin/gl/gldraw.h"
#include "tiny-viewer/core/utils.hpp"
#include "tiny-viewer/core/pose.hpp"
#include "random"

namespace ns_viewer {

Surfel::Surfel(const Vector4f &plane,
               const Cube &cube,
               const bool lineMode,
               bool drawCube,
               const Colour &color)
    : Entity(),
      drawCube(drawCube),
      cube(cube) {
    auto v = cube.GetVertices();
    Eigen::Vector3f norm = plane.block<3, 1>(0, 0);
    float dist = plane(3, 0);

    std::vector<Eigen::Vector3f> verts;
    verts.reserve(10);

#define INTERSECT_HELP(i, j)                                         \
    if (auto p = LinePlaneIntersection(v[i], v[j], norm, dist); p) { \
        verts.push_back(*p);                                         \
    }

    INTERSECT_HELP(0, 1)
    INTERSECT_HELP(2, 3)
    INTERSECT_HELP(4, 5)
    INTERSECT_HELP(6, 7)

    INTERSECT_HELP(0, 2)
    INTERSECT_HELP(1, 3)
    INTERSECT_HELP(4, 6)
    INTERSECT_HELP(5, 7)

    INTERSECT_HELP(0, 4)
    INTERSECT_HELP(1, 5)
    INTERSECT_HELP(2, 6)
    INTERSECT_HELP(3, 7)

    Eigen::Vector3f cen = cube.GetCenter();
    std::pair<Eigen::Vector3f, Eigen::Vector3f> axis = TangentBasis(norm);

    std::sort(verts.begin(), verts.end(),
              [&cen, &axis](const Eigen::Vector3f &v1, const Eigen::Vector3f &v2) {
                  Eigen::Vector3f d1 = v1 - cen, d2 = v2 - cen;
                  float theta1 = std::atan2(d1.dot(axis.second), d1.dot(axis.first));
                  float theta2 = std::atan2(d2.dot(axis.second), d2.dot(axis.first));
                  return theta1 < theta2;
              });

#undef INTERSECT_HELP
    polygon = Polygon(verts, lineMode, color);
}

Surfel::~Surfel() = default;

void Surfel::Draw() const {
    if (drawCube) {
        cube.Draw();
    }
    polygon.Draw();
}

Surfel::Ptr Surfel::Create(
    const Vector4f &plane, const Cube &cube, bool lineMode, bool drawCube, const Colour &color) {
    return std::make_shared<Surfel>(plane, cube, lineMode, drawCube, color);
}

Surfel::Ptr Surfel::Random(float bound) {
    std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<float> u1(0.5f, 1.0f);
    std::uniform_real_distribution<float> u2(-bound, bound);
    Eigen::Vector3f norm = Eigen::Vector3f(u2(engine), u2(engine), u2(engine)).normalized();
    auto pose = Posef::Random(bound);
    return Surfel::Create({ExpandVec3(norm), -pose.translation.dot(norm)},
                          Cube(pose, true, u1(engine), u1(engine), u1(engine)), false);
}
}  // namespace ns_viewer