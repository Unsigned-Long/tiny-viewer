//
// Created by csl on 5/11/23.
//

#include "tiny-viewer/entity/cylinder.h"
#include "pangolin/gl/gldraw.h"
#include "pangolin/gl/gl.h"

namespace ns_viewer {
    Cylinder::Cylinder(const Posef &pose, float height, float radius, const Colour &color)
            : Entity(), color(color) {
        Eigen::Vector3f p = pose.translation;
        Eigen::Vector3f x = pose.rotation.col(0) * radius;
        Eigen::Vector3f y = pose.rotation.col(1) * radius;
        Eigen::Vector3f z = pose.rotation.col(2) * height * 0.5f;
        const float deltaAng = M_PI * 2.0f / 8.0f;

        for (int i = 0; i < 8; ++i) {
            float ang = deltaAng * static_cast<float>(i);
            float c = std::cos(ang);
            float s = std::sin(ang);
            tops.at(i) = p + c * x + s * y + z;
            bottoms.at(i) = p + c * x + s * y - z;
        }
    }

    Cylinder::Ptr Cylinder::Create(const Posef &pose, float height, float radius, const Colour &color) {
        return std::make_shared<Cylinder>(pose, height, radius, color);
    }

    Cylinder::~Cylinder() = default;

    void Cylinder::Draw() const {
        glColor4f(ExpandColor(color));
        glLineWidth(DefaultLineSize);

        for (int i = 0; i < 8; ++i) {
            int j = (i + 1) % 8;
            const Eigen::Vector3f &tvi = tops.at(i);
            const Eigen::Vector3f &tvj = tops.at(j);
            pangolin::glDrawLine(ExpandVec3(tvi), ExpandVec3(tvj));

            const Eigen::Vector3f &bvi = bottoms.at(i);
            const Eigen::Vector3f &bvj = bottoms.at(j);
            pangolin::glDrawLine(ExpandVec3(bvi), ExpandVec3(bvj));

            pangolin::glDrawLine(ExpandVec3(tvi), ExpandVec3(bvi));
        }
        glPointSize(DefaultPointSize);
        glBegin(GL_POINTS);
        for (int i = 0; i < 8; ++i) {
            const Eigen::Vector3f &tvi = tops.at(i);
            const Eigen::Vector3f &bvi = bottoms.at(i);
            glVertex3f(ExpandVec3(tvi));
            glVertex3f(ExpandVec3(bvi));
        }
        glEnd();
    }
}