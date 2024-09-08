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

#ifndef TINY_VIEWER_PATH_H
#define TINY_VIEWER_PATH_H

#include "entity.h"
#include "line.h"

namespace ns_viewer {
struct Bezier {
public:
    static std::vector<Eigen::Vector3f> Solve(const std::vector<Eigen::Vector3f> &controlPoints,
                                              std::size_t num);

protected:
    static Eigen::Vector3f Solve(float t,
                                 const std::vector<Eigen::Vector3f> &controlPoints,
                                 std::size_t beg,
                                 std::size_t end);
};

/**
 * M = moveto, L = lineto, S = smooth curveto, Z = closepath
 * Uppercase means absolute position, lowercase means relative position
 */
struct Path : public Entity {
public:
    using Ptr = std::shared_ptr<Path>;

protected:
    std::vector<Line> lines;

public:
    explicit Path(const std::string &svgCode,
                  float size = DefaultLineSize,
                  const Colour &color = GetUniqueColour());

    explicit Path(const std::string &svgCode, const Colour &color, float size = DefaultLineSize);

    static Ptr Create(const std::string &svgCode,
                      float size = DefaultLineSize,
                      const Colour &color = GetUniqueColour());

    static Ptr Create(const std::string &svgCode,
                      const Colour &color,
                      float size = DefaultLineSize);

    ~Path() override = default;

    void Draw() const override;

    Path() = default;

protected:
    static std::vector<Line> ParseSVGCode(const std::string &svgCode,
                                          float size,
                                          const Colour &color);

    static bool IsSVGPathControlCode(const std::string &str);

    static float CurveLength(const std::vector<Eigen::Vector3f> &pts);

public:
    template <class Archive>
    void serialize(Archive &archive) {
        Entity::serialize(archive);
        archive(CEREAL_NVP(lines));
    }
};
}  // namespace ns_viewer

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Path, "Path")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Path)

#endif  // TINY_VIEWER_PATH_H
