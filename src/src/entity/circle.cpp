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

#include "tiny-viewer/entity/circle.h"
#include <tiny-viewer/core/pose.hpp>
#include "pangolin/gl/gldraw.h"
#include "pangolin/gl/gl.h"

namespace ns_viewer {
Circle::Circle(const Posef& pose,
               float radius,
               bool lineMode,
               bool drawCoord,
               const Colour& color,
               pangolin::AxisDirection nAxis,
               int ptCount)
    : coord(pose, radius),
      drawCoord(drawCoord) {
    std::pair<Eigen::Vector3f, Eigen::Vector3f> basis;
    if (nAxis == pangolin::AxisX || nAxis == pangolin::AxisNegX) {
        basis.first = pose.rotation.col(1), basis.second = pose.rotation.col(2);
    } else if (nAxis == pangolin::AxisY || nAxis == pangolin::AxisNegY) {
        basis.first = pose.rotation.col(2), basis.second = pose.rotation.col(0);
    } else {
        basis.first = pose.rotation.col(0), basis.second = pose.rotation.col(1);
    }
    basis.first *= radius, basis.second *= radius;

    std::vector<Eigen::Vector3f> verts(ptCount);
    const float deltaAng = M_PI * 2.0f / static_cast<double>(verts.size());

    for (int i = 0; i < static_cast<int>(verts.size()); ++i) {
        float ang = deltaAng * static_cast<float>(i);
        float c = std::cos(ang);
        float s = std::sin(ang);
        verts.at(i) = pose.translation + c * basis.first + s * basis.second;
    }
    poly = Polygon(verts, lineMode, color);
}

Circle::Ptr Circle::Create(const Posef& pose,
                           float radius,
                           bool lineMode,
                           bool drawCoord,
                           const Colour& color,
                           pangolin::AxisDirection nAxis,
                           int ptCount) {
    return std::make_shared<Circle>(pose, radius, lineMode, drawCoord, color, nAxis, ptCount);
}

Circle::~Circle() = default;

void Circle::Draw() const {
    if (drawCoord) {
        coord.Draw();
    }
    poly.Draw();
}
}  // namespace ns_viewer