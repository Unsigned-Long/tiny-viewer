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

#ifndef SLAM_SCENE_VIEWER_POSE_H
#define SLAM_SCENE_VIEWER_POSE_H

#include "utils.hpp"
#include "Eigen/Geometry"
#include "random"
#include "chrono"

namespace ns_viewer {
template <class ScalarType>
struct Pose {
public:
    using Scale = ScalarType;
    using Rotation = Matrix<Scale, 3, 3>;
    using Translation = Vector3<Scale>;
    using Transform = Matrix<Scale, 4, 4>;

public:
    double timeStamp;
    Rotation rotation;
    Translation translation;

    explicit Pose(const Transform &transform, double timeStamp = INVALID_TIME_STAMP)
        : timeStamp(timeStamp),
          rotation(transform.topLeftCorner(3, 3)),
          translation(transform.topRightCorner(3, 1)) {}

    explicit Pose(const Rotation &rotation = Rotation::Identity(),
                  const Translation &translation = Translation::Zero(),
                  double timeStamp = INVALID_TIME_STAMP)
        : timeStamp(timeStamp),
          rotation(rotation),
          translation(translation) {}

    static Pose Random(ScalarType bound) {
        std::default_random_engine engine(
            std::chrono::steady_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<ScalarType> ut(-bound, bound);
        std::uniform_real_distribution<ScalarType> ur(-M_PI, M_PI);
        Translation t(ut(engine), ut(engine), ut(engine));
        Eigen::AngleAxis<ScalarType> a1(ur(engine), Translation(0.0, 0.0, 1.0));
        Eigen::AngleAxis<ScalarType> a2(ur(engine), Translation(0.0, 1.0, 0.0));
        Eigen::AngleAxis<ScalarType> a3(ur(engine), Translation(1.0, 0.0, 0.0));
        Rotation r = (a3 * a2 * a1).toRotationMatrix();
        return Pose(r, t, INVALID_TIME_STAMP);
    }

    Transform matrix44() const {
        Transform transform = Transform::Identity();
        transform.topLeftCorner(3, 3) = rotation;
        transform.topRightCorner(3, 1) = translation;
        return transform;
    }

    Matrix34<Scale> matrix34() const {
        ns_viewer::Matrix34<Scale> transform = ns_viewer::Matrix34<Scale>::Identity();
        transform.topLeftCorner(3, 3) = rotation;
        transform.topRightCorner(3, 1) = translation;
        return transform;
    }

    Vector3<Scale> trans(const Vector3<Scale> &p) const { return rotation * p + translation; }

    Eigen::Quaternion<Scale> quaternion() const { return Eigen::Quaternion<Scale>(rotation); }

    Pose inverse() const {
        return Pose(rotation.inverse(), -rotation.inverse() * translation, timeStamp);
    }

    template <class TarScale>
    Pose<TarScale> cast() const {
        return Pose<TarScale>(rotation.template cast<TarScale>(),
                              translation.template cast<TarScale>());
    }
};

using Posed = Pose<double>;
using Posef = Pose<float>;
}  // namespace ns_viewer

#endif  // SLAM_SCENE_VIEWER_POSE_H
