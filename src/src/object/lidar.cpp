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

#include "tiny-viewer/object/lidar.h"

namespace ns_viewer {

LiDAR::LiDAR(const Posef &pose, float size, const Colour &color)
    : Entity(),
      coord(pose, size),
      cylinder(pose, size * 2.0f, size, color) {}

LiDAR::LiDAR(const Posef &pose, const Colour &color, float size)
    : LiDAR(pose, size, color) {}

LiDAR::Ptr LiDAR::Create(const Posef &pose, const Colour &color, float size) {
    return std::make_shared<LiDAR>(pose, size, color);
}

LiDAR::Ptr LiDAR::Create(const Posef &pose, float size, const Colour &color) {
    return std::make_shared<LiDAR>(pose, size, color);
}

LiDAR::~LiDAR() = default;

void LiDAR::Draw() const {
    coord.Draw();
    cylinder.Draw();
}

LivoxLiDAR::LivoxLiDAR(const Posef &pose, float size, const Colour &color)
    : Entity(),
      coord(pose, size),
      cube(pose, true, size * 2.0f, size, size, color) {
    const auto &vertices = cube.GetVertices();
    Eigen::Vector3f v1 = vertices.at(0) + 0.25f * (vertices.at(3) - vertices.at(0));
    Eigen::Vector3f v2 = vertices.at(3) + 0.25f * (vertices.at(0) - vertices.at(3));
    Eigen::Vector3f v3 = vertices.at(1) + 0.25f * (vertices.at(2) - vertices.at(1));
    Eigen::Vector3f v4 = vertices.at(2) + 0.25f * (vertices.at(1) - vertices.at(2));

    lines.at(0) = Line(v1, vertices.at(0), color);
    lines.at(1) = Line(v2, vertices.at(3), color);
    lines.at(2) = Line(v3, vertices.at(1), color);
    lines.at(3) = Line(v4, vertices.at(2), color);

    lines.at(4) = Line(v1, v3, color);
    lines.at(5) = Line(v1, v4, color);
    lines.at(6) = Line(v2, v3, color);
    lines.at(7) = Line(v2, v4, color);

    lines.at(8) = Line(vertices.at(4), vertices.at(7), color);
    lines.at(9) = Line(vertices.at(6), vertices.at(5), color);
}

LivoxLiDAR::LivoxLiDAR(const Posef &pose, const Colour &color, float size)
    : LivoxLiDAR(pose, size, color) {}

LivoxLiDAR::Ptr LivoxLiDAR::Create(const Posef &pose, const Colour &color, float size) {
    return std::make_shared<LivoxLiDAR>(pose, size, color);
}

LivoxLiDAR::Ptr LivoxLiDAR::Create(const Posef &pose, float size, const Colour &color) {
    return std::make_shared<LivoxLiDAR>(pose, size, color);
}

LivoxLiDAR::~LivoxLiDAR() = default;

void LivoxLiDAR::Draw() const {
    coord.Draw();
    cube.Draw();
    for (const auto &line : lines) {
        line.Draw();
    }
}
}  // namespace ns_viewer