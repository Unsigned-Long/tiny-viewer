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

#include "pangolin/gl/gldraw.h"
#include "tiny-viewer/object/plane.h"

namespace ns_viewer {

Plane::Plane(const Posef &pose,
             float mainWidth,
             float subWidth,
             bool lineMode,
             const Colour &color,
             pangolin::AxisDirection mainAxis,
             pangolin::AxisDirection subAxis)
    : Entity(),
      lineMode(lineMode),
      color(color),
      coord(pose, 0.5f * std::min(mainWidth, subWidth)) {
    Eigen::Vector3f p = pose.translation;

    Eigen::Vector3f ma = 0.5f * mainWidth * pose.rotation *
                         Eigen::Vector3f(ExpandAryVec3(pangolin::AxisDirectionVector[mainAxis]));
    Eigen::Vector3f sa = 0.5f * subWidth * pose.rotation *
                         Eigen::Vector3f(ExpandAryVec3(pangolin::AxisDirectionVector[subAxis]));

    v1 = p + ma + sa;
    v2 = p + ma - sa;
    v3 = p - ma - sa;
    v4 = p - ma + sa;
}

Plane::Plane(const Vector4f &plane,
             float mainWidth,
             float subWidth,
             bool lineMode,
             const Colour &color,
             pangolin::AxisDirection mainAxis,
             pangolin::AxisDirection subAxis)
    : Plane(
          [&plane]() {
              Eigen::Vector3f zAxis = plane.block<3, 1>(0, 0);
              std::pair<Eigen::Vector3f, Eigen::Vector3f> axis = TangentBasis(zAxis);
              Eigen::Matrix3f rotMat;
              rotMat.col(0) = axis.first;
              rotMat.col(1) = axis.second;
              rotMat.col(2) = zAxis;
              return Posef(rotMat, -plane(3, 0) * zAxis);
          }(),
          mainWidth,
          subWidth,
          lineMode,
          color,
          mainAxis,
          subAxis) {}

Plane::Ptr Plane::Create(const Posef &pose,
                         float mainWidth,
                         float subWidth,
                         bool lineMode,
                         const Colour &color,
                         pangolin::AxisDirection mainAxis,
                         pangolin::AxisDirection subAxis) {
    return std::make_shared<Plane>(pose, mainWidth, subWidth, lineMode, color, mainAxis, subAxis);
}

Plane::Ptr Plane::Create(const Vector4f &plane,
                         float mainWidth,
                         float subWidth,
                         bool lineMode,
                         const Colour &color,
                         pangolin::AxisDirection mainAxis,
                         pangolin::AxisDirection subAxis) {
    return std::make_shared<Plane>(plane, mainWidth, subWidth, lineMode, color, mainAxis, subAxis);
}

Plane::~Plane() = default;

void Plane::Draw() const {
    coord.Draw();
    glColor4f(ExpandColor(color));
    if (lineMode) {
        glLineWidth(DefaultLineSize);
        pangolin::glDrawLine(ExpandVec3(v1), ExpandVec3(v2));
        pangolin::glDrawLine(ExpandVec3(v2), ExpandVec3(v3));
        pangolin::glDrawLine(ExpandVec3(v3), ExpandVec3(v4));
        pangolin::glDrawLine(ExpandVec3(v4), ExpandVec3(v1));
    } else {
        const GLfloat verts[] = {
            ExpandVec3(v1), ExpandVec3(v2), ExpandVec3(v3),
            ExpandVec3(v1), ExpandVec3(v3), ExpandVec3(v4),
        };

        glVertexPointer(3, GL_FLOAT, 0, verts);
        glEnableClientState(GL_VERTEX_ARRAY);
        glDrawArrays(GL_TRIANGLES, 0, 6);

        glDisableClientState(GL_VERTEX_ARRAY);
    }
    glPointSize(DefaultPointSize);
    glBegin(GL_POINTS);
    glVertex3f(ExpandVec3(v1));
    glVertex3f(ExpandVec3(v2));
    glVertex3f(ExpandVec3(v3));
    glVertex3f(ExpandVec3(v4));
    glEnd();
}
}  // namespace ns_viewer