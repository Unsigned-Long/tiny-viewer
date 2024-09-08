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

#include "tiny-viewer/object/landmark.h"
#include "pangolin/gl/gldraw.h"
#include "tiny-viewer/core/pose.hpp"

namespace ns_viewer {

Landmark::Landmark(const Vector3f &p, float size, const Colour &color)
    : color(color),
      size(size) {
    constexpr float scales[7][3] = {
        {1.0f, 0.0f, 0.0f},      {0.0f, 1.0f, 0.0f},     {0.0f, 0.0f, 1.0f},
        {0.58f, 0.58f, 0.58f},   {-0.58f, 0.58f, 0.58f}, {0.58f, 0.58f, -0.58f},
        {-0.58f, 0.58f, -0.58f},
    };

    for (int i = 0; i < static_cast<int>(verts.size()); i += 2) {
        Eigen::Vector3f delta =
            Eigen::Vector3f{scales[i / 2][0], scales[i / 2][1], scales[i / 2][2]} * size;
        verts.at(i + 0) = p + delta, verts.at(i + 1) = p - delta;
    }
}

Landmark::Landmark(const Vector3f &p, const Colour &color, float size)
    : Landmark(p, size, color) {}

Landmark::Ptr Landmark::Create(const Vector3f &p, float size, const Colour &color) {
    return std::make_shared<Landmark>(p, size, color);
}

Landmark::Ptr Landmark::Create(const Vector3f &p, const Colour &color, float size) {
    return std::make_shared<Landmark>(p, color, size);
}

void Landmark::Draw() const {
    glColor4f(ExpandColor(color));
    glLineWidth(DefaultLineSize);
    for (int i = 0; i < static_cast<int>(verts.size()); i += 2) {
        pangolin::glDrawLine(ExpandVec3(verts.at(i + 0)), ExpandVec3(verts.at(i + 1)));
    }
}
}  // namespace ns_viewer