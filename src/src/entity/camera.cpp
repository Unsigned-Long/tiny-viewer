//
// Created by csl on 5/9/23.
//

#include "tiny-viewer/entity/camera.h"
#include "pangolin/gl/gldraw.h"

namespace ns_viewer {

    Camera::Camera(const Posef &pose, float size, const Colour &color)
            : Entity(), _coord(pose, size * 0.5f), _color(color) {
        _v0 = pose.translation;
        Eigen::Vector3f x = pose.rotation.col(0) * size * 0.5f;
        Eigen::Vector3f y = pose.rotation.col(1) * size * 0.5f;
        Eigen::Vector3f z = pose.rotation.col(2) * size * 0.5f;

        _v1 = _v0 - x - y * 0.75f + z;
        _v2 = _v0 + x - y * 0.75f + z;
        _v3 = _v0 - x + y * 0.75f + z;
        _v4 = _v0 + x + y * 0.75f + z;
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
        _coord.Draw();
        glColor4f(ExpandColor(_color));
        glLineWidth(DefaultLineSize);
        pangolin::glDrawLine(ExpandVec3(_v0), ExpandVec3(_v1));
        pangolin::glDrawLine(ExpandVec3(_v0), ExpandVec3(_v2));
        pangolin::glDrawLine(ExpandVec3(_v0), ExpandVec3(_v3));
        pangolin::glDrawLine(ExpandVec3(_v0), ExpandVec3(_v4));
        pangolin::glDrawLine(ExpandVec3(_v1), ExpandVec3(_v2));
        pangolin::glDrawLine(ExpandVec3(_v3), ExpandVec3(_v4));
        pangolin::glDrawLine(ExpandVec3(_v1), ExpandVec3(_v3));
        pangolin::glDrawLine(ExpandVec3(_v2), ExpandVec3(_v4));
    }
}