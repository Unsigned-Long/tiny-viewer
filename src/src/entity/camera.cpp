//
// Created by csl on 5/9/23.
//

#include "tiny-viewer/entity/camera.h"
#include "pangolin/gl/gldraw.h"

namespace ns_viewer {

    Camera::Camera(const Posef &pose, float size, const Colour &color)
            : Entity(), coord(pose, size), color(color) {
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
}