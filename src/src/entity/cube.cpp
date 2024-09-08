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

#include "tiny-viewer/entity/cube.h"
#include "pangolin/gl/gldraw.h"

namespace ns_viewer {

Cube::Cube(
    const Posef &pose, bool lineMode, float xWidth, float yWidth, float zWidth, const Colour &color)
    : Entity(),
      lineMode(lineMode),
      color(color) {
    Eigen::Vector3f p = pose.translation;
    Eigen::Vector3f x = pose.rotation.col(0) * xWidth * 0.5f;
    Eigen::Vector3f y = pose.rotation.col(1) * yWidth * 0.5f;
    Eigen::Vector3f z = pose.rotation.col(2) * zWidth * 0.5f;

    v1 = p + x + y + z;
    v2 = p + x - y + z;
    v3 = p + x + y - z;
    v4 = p + x - y - z;

    v5 = p - x + y + z;
    v6 = p - x - y + z;
    v7 = p - x + y - z;
    v8 = p - x - y - z;
}

Cube::Cube(
    const Posef &pose, bool lineMode, const Colour &color, float xWidth, float yWidth, float zWidth)
    : Cube(pose, lineMode, xWidth, yWidth, zWidth, color) {}

Cube::~Cube() = default;

void Cube::Draw() const {
    glColor4f(ExpandColor(color));

    if (lineMode) {
        glLineWidth(DefaultLineSize);
        pangolin::glDrawLine(ExpandVec3(v1), ExpandVec3(v2));
        pangolin::glDrawLine(ExpandVec3(v3), ExpandVec3(v4));
        pangolin::glDrawLine(ExpandVec3(v1), ExpandVec3(v3));
        pangolin::glDrawLine(ExpandVec3(v2), ExpandVec3(v4));

        pangolin::glDrawLine(ExpandVec3(v5), ExpandVec3(v6));
        pangolin::glDrawLine(ExpandVec3(v7), ExpandVec3(v8));
        pangolin::glDrawLine(ExpandVec3(v5), ExpandVec3(v7));
        pangolin::glDrawLine(ExpandVec3(v6), ExpandVec3(v8));

        pangolin::glDrawLine(ExpandVec3(v1), ExpandVec3(v5));
        pangolin::glDrawLine(ExpandVec3(v2), ExpandVec3(v6));
        pangolin::glDrawLine(ExpandVec3(v3), ExpandVec3(v7));
        pangolin::glDrawLine(ExpandVec3(v4), ExpandVec3(v8));

    } else {
        const GLfloat verts[] = {
            ExpandVec3(v1), ExpandVec3(v2), ExpandVec3(v3), ExpandVec3(v4),  // FRONT
            ExpandVec3(v5), ExpandVec3(v6), ExpandVec3(v7), ExpandVec3(v8),  // BACK
            ExpandVec3(v1), ExpandVec3(v3), ExpandVec3(v5), ExpandVec3(v7),  // LEFT
            ExpandVec3(v2), ExpandVec3(v4), ExpandVec3(v6), ExpandVec3(v8),  // RIGHT
            ExpandVec3(v1), ExpandVec3(v2), ExpandVec3(v5), ExpandVec3(v6),  // TOP
            ExpandVec3(v3), ExpandVec3(v4), ExpandVec3(v7), ExpandVec3(v8)   // BOTTOM
        };

        glVertexPointer(3, GL_FLOAT, 0, verts);
        glEnableClientState(GL_VERTEX_ARRAY);

        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glDrawArrays(GL_TRIANGLE_STRIP, 4, 4);

        glDrawArrays(GL_TRIANGLE_STRIP, 8, 4);
        glDrawArrays(GL_TRIANGLE_STRIP, 12, 4);

        glDrawArrays(GL_TRIANGLE_STRIP, 16, 4);
        glDrawArrays(GL_TRIANGLE_STRIP, 20, 4);

        glDisableClientState(GL_VERTEX_ARRAY);
    }

    glPointSize(DefaultPointSize);
    glBegin(GL_POINTS);
    glVertex3f(ExpandVec3(v1));
    glVertex3f(ExpandVec3(v2));
    glVertex3f(ExpandVec3(v3));
    glVertex3f(ExpandVec3(v4));
    glVertex3f(ExpandVec3(v5));
    glVertex3f(ExpandVec3(v6));
    glVertex3f(ExpandVec3(v7));
    glVertex3f(ExpandVec3(v8));
    glEnd();
}

Cube::Ptr Cube::Create(const Posef &pose,
                       bool lineMode,
                       float xWidth,
                       float yWidth,
                       float zWidth,
                       const Colour &color) {
    return std::make_shared<Cube>(pose, lineMode, xWidth, yWidth, zWidth, color);
}

Cube::Ptr Cube::Create(const Posef &pose,
                       bool lineMode,
                       const Colour &color,
                       float xWidth,
                       float yWidth,
                       float zWidth) {
    return std::make_shared<Cube>(pose, lineMode, xWidth, yWidth, zWidth, color);
}

std::array<Eigen::Vector3f, 8> Cube::GetVertices() const {
    return {v1, v2, v3, v4, v5, v6, v7, v8};
}

Eigen::Vector3f Cube::GetCenter() const { return (v1 + v2 + v7 + v8) * 0.25f; }
}  // namespace ns_viewer