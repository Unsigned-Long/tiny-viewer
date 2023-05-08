//
// Created by csl on 10/3/22.
//

#ifndef SLAM_SCENE_VIEWER_POSE_H
#define SLAM_SCENE_VIEWER_POSE_H

#include "tiny-viewer/utils.hpp"
#include "Eigen/Geometry"
#include "random"
#include "chrono"

namespace ns_viewer {
    template<class ScalarType>
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

        explicit Pose(const Transform &transform,
                      double timeStamp = INVALID_TIME_STAMP)
                : timeStamp(timeStamp),
                  rotation(transform.topLeftCorner(3, 3)),
                  translation(transform.topRightCorner(3, 1)) {}

        explicit Pose(const Rotation &rotation = Rotation::Identity(),
                      const Translation &translation = Translation::Zero(),
                      double timeStamp = INVALID_TIME_STAMP)
                : timeStamp(timeStamp), rotation(rotation), translation(translation) {}

        static Pose Random(ScalarType bound) {
            std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
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

        Vector3<Scale> trans(const Vector3<Scale> &p) const {
            return rotation * p + translation;
        }

        Eigen::Quaternion<Scale> quaternion() const {
            return Eigen::Quaternion<Scale>(rotation);
        }

        Pose inverse() const {
            return Pose(rotation.inverse(), -rotation.inverse() * translation, timeStamp);
        }

        template<class TarScale>
        Pose<TarScale> cast() const {
            return Pose<TarScale>(rotation.template cast<TarScale>(), translation.template cast<TarScale>());
        }

    };

    using Posed = Pose<double>;
    using Posef = Pose<float>;
}

#endif //SLAM_SCENE_VIEWER_POSE_H
