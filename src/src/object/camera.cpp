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

#include "tiny-viewer/object/camera.h"
#include "pangolin/gl/gldraw.h"
#include "tiny-viewer/core/pose.hpp"

namespace ns_viewer {

//-------
// camera
// ------
Camera::Camera(const Posef &pose, float size, const Colour &color)
    : Entity(),
      coord(pose, size),
      color(color) {
    v0 = pose.translation;
    Eigen::Vector3f x = pose.rotation.col(0) * size;
    Eigen::Vector3f y = pose.rotation.col(1) * size;
    Eigen::Vector3f z = pose.rotation.col(2) * size;

    v1 = v0 - x - y * 0.75f + z;
    v2 = v0 + x - y * 0.75f + z;
    v3 = v0 - x + y * 0.75f + z;
    v4 = v0 + x + y * 0.75f + z;
}

Camera::Camera(const Posef &pose, const Colour &color, float size)
    : Camera(pose, size, color) {}

Camera::~Camera() = default;

Camera::Ptr Camera::Create(const Posef &pose, float size, const Colour &color) {
    return std::make_shared<Camera>(pose, size, color);
}

Camera::Ptr Camera::Create(const Posef &pose, const Colour &color, float size) {
    return std::make_shared<Camera>(pose, size, color);
}

void Camera::Draw() const {
    coord.Draw();
    glColor4f(ExpandColor(color));
    glLineWidth(DefaultLineSize);
    pangolin::glDrawLine(ExpandVec3(v0), ExpandVec3(v1));
    pangolin::glDrawLine(ExpandVec3(v0), ExpandVec3(v2));
    pangolin::glDrawLine(ExpandVec3(v0), ExpandVec3(v3));
    pangolin::glDrawLine(ExpandVec3(v0), ExpandVec3(v4));
    pangolin::glDrawLine(ExpandVec3(v1), ExpandVec3(v2));
    pangolin::glDrawLine(ExpandVec3(v3), ExpandVec3(v4));
    pangolin::glDrawLine(ExpandVec3(v1), ExpandVec3(v3));
    pangolin::glDrawLine(ExpandVec3(v2), ExpandVec3(v4));
    glPointSize(DefaultPointSize);
    glBegin(GL_POINTS);
    glVertex3f(ExpandVec3(v0));
    glVertex3f(ExpandVec3(v1));
    glVertex3f(ExpandVec3(v2));
    glVertex3f(ExpandVec3(v3));
    glVertex3f(ExpandVec3(v4));
    glEnd();
}

//-------
// camera
// ------
CubeCamera::CubeCamera(const Posef &pose, float size, const Colour &color)
    : Entity(),
      coord(pose, size),
      color(color),
      cube(pose, true) {
    v0 = pose.translation;
    Eigen::Vector3f x = pose.rotation.col(0) * size;
    Eigen::Vector3f y = pose.rotation.col(1) * size;
    Eigen::Vector3f z = pose.rotation.col(2) * size;

    v1 = v0 - x * 0.6f - y * 0.6f + z;
    v2 = v0 + x * 0.6f - y * 0.6f + z;
    v3 = v0 - x * 0.6f + y * 0.6f + z;
    v4 = v0 + x * 0.6f + y * 0.6f + z;

    cube = Cube(Posef(pose.rotation, v0 - z * 0.75f), true, size * 1.2f, size * 1.2f, size * 1.5f,
                color);
}

CubeCamera::CubeCamera(const Posef &pose, const Colour &color, float size)
    : CubeCamera(pose, size, color) {}

CubeCamera::~CubeCamera() = default;

CubeCamera::Ptr CubeCamera::Create(const Posef &pose, float size, const Colour &color) {
    return std::make_shared<CubeCamera>(pose, size, color);
}

CubeCamera::Ptr CubeCamera::Create(const Posef &pose, const Colour &color, float size) {
    return std::make_shared<CubeCamera>(pose, size, color);
}

void CubeCamera::Draw() const {
    coord.Draw();
    cube.Draw();
    glColor4f(ExpandColor(color));
    glLineWidth(DefaultLineSize);
    pangolin::glDrawLine(ExpandVec3(v0), ExpandVec3(v1));
    pangolin::glDrawLine(ExpandVec3(v0), ExpandVec3(v2));
    pangolin::glDrawLine(ExpandVec3(v0), ExpandVec3(v3));
    pangolin::glDrawLine(ExpandVec3(v0), ExpandVec3(v4));
    pangolin::glDrawLine(ExpandVec3(v1), ExpandVec3(v2));
    pangolin::glDrawLine(ExpandVec3(v3), ExpandVec3(v4));
    pangolin::glDrawLine(ExpandVec3(v1), ExpandVec3(v3));
    pangolin::glDrawLine(ExpandVec3(v2), ExpandVec3(v4));
    glPointSize(DefaultPointSize);
    glBegin(GL_POINTS);
    glVertex3f(ExpandVec3(v0));
    glVertex3f(ExpandVec3(v1));
    glVertex3f(ExpandVec3(v2));
    glVertex3f(ExpandVec3(v3));
    glVertex3f(ExpandVec3(v4));
    glEnd();
}
}  // namespace ns_viewer