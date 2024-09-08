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

#include "tiny-viewer/entity/cone.h"
#include "pangolin/gl/gldraw.h"
#include "tiny-viewer/core/pose.hpp"

namespace ns_viewer {

Cone::Cone(const Posef &pose, float height, float angle, const Colour &color)
    : Entity(),
      color(color) {
    bp = pose.translation;
    Eigen::Vector3f x = pose.rotation.col(0);
    Eigen::Vector3f z = pose.rotation.col(2);
    tp = bp + z * height;

    const float deltaAng = M_PI * 2.0f / 8.0f;
    const float r = height * std::sin(angle);

    for (int i = 0; i < 8; ++i) {
        float ang = deltaAng * static_cast<float>(i);
        Vector3f rv = Eigen::AngleAxisf(ang, z) * x * r;
        verts.at(i) = bp + rv;
    }
}

Cone::~Cone() = default;

void Cone::Draw() const {
    glColor4f(ExpandColor(color));
    glLineWidth(DefaultLineSize);
    for (int i = 0; i < 8; ++i) {
        const Eigen::Vector3f &vj = verts.at((i + 1) % 8);
        pangolin::glDrawLine(ExpandVec3(verts.at(i)), ExpandVec3(tp));
        pangolin::glDrawLine(ExpandVec3(verts.at(i)), ExpandVec3(bp));
        pangolin::glDrawLine(ExpandVec3(verts.at(i)), ExpandVec3(vj));
    }

    glPointSize(DefaultPointSize);
    glBegin(GL_POINTS);
    glVertex3f(ExpandVec3(tp));
    glVertex3f(ExpandVec3(bp));
    for (int i = 0; i < 8; ++i) {
        glVertex3f(ExpandVec3(verts.at(i)));
    }
    glEnd();
}

Cone::Ptr Cone::Create(const Posef &pose, float height, float angle, const Colour &color) {
    return std::make_shared<Cone>(pose, height, angle, color);
}

}  // namespace ns_viewer