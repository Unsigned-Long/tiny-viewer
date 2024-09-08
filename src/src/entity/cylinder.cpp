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

#include "tiny-viewer/entity/cylinder.h"
#include "pangolin/gl/gldraw.h"
#include "pangolin/gl/gl.h"
#include "tiny-viewer/core/pose.hpp"

namespace ns_viewer {
Cylinder::Cylinder(const Posef &pose, float height, float radius, const Colour &color)
    : Entity(),
      color(color) {
    Eigen::Vector3f p = pose.translation;
    Eigen::Vector3f x = pose.rotation.col(0) * radius;
    Eigen::Vector3f y = pose.rotation.col(1) * radius;
    Eigen::Vector3f z = pose.rotation.col(2) * height * 0.5f;
    const float deltaAng = M_PI * 2.0f / 8.0f;

    for (int i = 0; i < 8; ++i) {
        float ang = deltaAng * static_cast<float>(i);
        float c = std::cos(ang);
        float s = std::sin(ang);
        tops.at(i) = p + c * x + s * y + z;
        bottoms.at(i) = p + c * x + s * y - z;
    }
}

Cylinder::Ptr Cylinder::Create(const Posef &pose, float height, float radius, const Colour &color) {
    return std::make_shared<Cylinder>(pose, height, radius, color);
}

Cylinder::~Cylinder() = default;

void Cylinder::Draw() const {
    glColor4f(ExpandColor(color));
    glLineWidth(DefaultLineSize);

    for (int i = 0; i < 8; ++i) {
        int j = (i + 1) % 8;
        const Eigen::Vector3f &tvi = tops.at(i);
        const Eigen::Vector3f &tvj = tops.at(j);
        pangolin::glDrawLine(ExpandVec3(tvi), ExpandVec3(tvj));

        const Eigen::Vector3f &bvi = bottoms.at(i);
        const Eigen::Vector3f &bvj = bottoms.at(j);
        pangolin::glDrawLine(ExpandVec3(bvi), ExpandVec3(bvj));

        pangolin::glDrawLine(ExpandVec3(tvi), ExpandVec3(bvi));
    }
    glPointSize(DefaultPointSize);
    glBegin(GL_POINTS);
    for (int i = 0; i < 8; ++i) {
        const Eigen::Vector3f &tvi = tops.at(i);
        const Eigen::Vector3f &bvi = bottoms.at(i);
        glVertex3f(ExpandVec3(tvi));
        glVertex3f(ExpandVec3(bvi));
    }
    glEnd();
}
}  // namespace ns_viewer