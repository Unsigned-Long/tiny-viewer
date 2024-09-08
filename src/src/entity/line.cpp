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

#include <utility>
#include "tiny-viewer/entity/line.h"
#include "pangolin/gl/gldraw.h"

namespace ns_viewer {

Line::Line(Eigen::Vector3f sp, Eigen::Vector3f ep, float size, const Colour &color)
    : Entity(),
      sp(std::move(sp)),
      ep(std::move(ep)),
      size(size),
      color(color) {}

Line::Line(Eigen::Vector3f sp, Eigen::Vector3f ep, const Colour &color, float size)
    : Line(std::move(sp), std::move(ep), size, color) {}

Line::~Line() = default;

void Line::Draw() const {
    glColor4f(ExpandColor(color));
    glLineWidth(size);
    pangolin::glDrawLine(ExpandVec3(sp), ExpandVec3(ep));
    glPointSize(size * 2.0f);
    glBegin(GL_POINTS);
    glVertex3f(ExpandVec3(sp));
    glVertex3f(ExpandVec3(ep));
    glEnd();
}

std::shared_ptr<Line> Line::Create(const Vector3f &sp,
                                   const Vector3f &ep,
                                   const Colour &color,
                                   float size) {
    return std::make_shared<Line>(sp, ep, size, color);
}

std::shared_ptr<Line> Line::Create(const Vector3f &sp,
                                   const Vector3f &ep,
                                   float size,
                                   const Colour &color) {
    return std::make_shared<Line>(sp, ep, size, color);
}
}  // namespace ns_viewer