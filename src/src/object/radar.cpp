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

#include "tiny-viewer/object/radar.h"
#include "pangolin/gl/gldraw.h"
#include "tiny-viewer/core/pose.hpp"

namespace ns_viewer {

Radar::Radar(const Posef &pose, float size, const Colour &color)
    : coord(pose, size),
      color(color) {
    cenVert = pose.translation;
    Eigen::Vector3f x = pose.rotation.col(0);
    Eigen::Vector3f y = pose.rotation.col(1);
    Eigen::Vector3f z = pose.rotation.col(2);

    {
        Eigen::Vector3f ep = cenVert + x * size;
        const float deltaAng = M_PI * 2.0f / 8.0f;
        const auto r = static_cast<float>(size * std::tan(M_PI / 5.0));

        for (int i = 0; i < 8; ++i) {
            float ang = deltaAng * static_cast<float>(i);
            Eigen::Vector3f rv = y * r * std::cos(ang) + z * r * std::sin(ang);
            tVerts.at(i) = ep + rv;
        }
    }
    {
        Eigen::Vector3f ep = cenVert - z * size;
        const float deltaAng = M_PI * 2.0f / 4.0f;
        const auto r = static_cast<float>(size * std::tan(M_PI / 8.0));
        for (int i = 0; i < 4; ++i) {
            float ang = deltaAng * static_cast<float>(i);
            Eigen::Vector3f rv = x * r * std::cos(ang) + y * r * std::sin(ang);
            bVerts.at(i) = ep + rv;
        }
    }
}

void Radar::Draw() const {
    coord.Draw();

    glColor4f(ExpandColor(color));
    glLineWidth(DefaultLineSize);
    for (int i = 0; i < 8; ++i) {
        const Eigen::Vector3f &vj = tVerts.at((i + 1) % 8);
        pangolin::glDrawLine(ExpandVec3(tVerts.at(i)), ExpandVec3(cenVert));
        pangolin::glDrawLine(ExpandVec3(tVerts.at(i)), ExpandVec3(vj));
    }
    for (int i = 0; i < 4; ++i) {
        const Eigen::Vector3f &vj = bVerts.at((i + 1) % 4);
        pangolin::glDrawLine(ExpandVec3(bVerts.at(i)), ExpandVec3(cenVert));
        pangolin::glDrawLine(ExpandVec3(bVerts.at(i)), ExpandVec3(vj));
    }

    glPointSize(DefaultPointSize);
    glBegin(GL_POINTS);
    glVertex3f(ExpandVec3(cenVert));
    for (int i = 0; i < 8; ++i) {
        glVertex3f(ExpandVec3(tVerts.at(i)));
    }
    for (int i = 0; i < 4; ++i) {
        glVertex3f(ExpandVec3(bVerts.at(i)));
    }
    glEnd();
}

Radar::Ptr Radar::Create(const Posef &pose, float size, const Colour &color) {
    return std::make_shared<Radar>(pose, size, color);
}

Radar::Ptr Radar::Create(const Posef &pose, const Colour &color, float size) {
    return std::make_shared<Radar>(pose, color, size);
}

Radar::Radar(const Posef &pose, const Colour &color, float size)
    : Radar(pose, size, color) {}
}  // namespace ns_viewer