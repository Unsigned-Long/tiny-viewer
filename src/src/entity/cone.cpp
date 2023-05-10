//
// Created by csl on 5/10/23.
//

#include <utility>
#include "tiny-viewer/entity/cone.h"
#include "pangolin/gl/gldraw.h"

namespace ns_viewer {

    Cone::Cone(const Posef &pose, float height, float angle, const Colour &color)
            : Entity(), color(color) {
        bp = pose.translation;
        Eigen::Vector3f x = pose.rotation.col(0);
        Eigen::Vector3f z = pose.rotation.col(2);
        tp = bp + z * height;

        const float deltaAng = M_PI * 2.0f / 8.0f;
        const float r = height * std::sin(angle);

        for (int i = 0; i < 8; ++i) {
            float ang = deltaAng * static_cast<float>(i);
            Vector3f rv = Eigen::AngleAxisf(ang, z) * x * r;
            verts.at(i) = bp + rv;
        }
    }

    Cone::~Cone() = default;

    void Cone::Draw() const {
        glColor4f(ExpandColor(color));
        glLineWidth(DefaultLineSize);
        for (int i = 0; i < 8; ++i) {
            const Eigen::Vector3f &vj = verts.at((i + 1) % 8);
            pangolin::glDrawLine(ExpandVec3(verts.at(i)), ExpandVec3(tp));
            pangolin::glDrawLine(ExpandVec3(verts.at(i)), ExpandVec3(bp));
            pangolin::glDrawLine(ExpandVec3(verts.at(i)), ExpandVec3(vj));
        }

        glPointSize(DefaultPointSize);
        glBegin(GL_POINTS);
        glVertex3f(ExpandVec3(tp));
        glVertex3f(ExpandVec3(bp));
        for (int i = 0; i < 8; ++i) {
            glVertex3f(ExpandVec3(verts.at(i)));
        }
        glEnd();
    }

    Cone::Ptr Cone::Create(const Posef &pose, float height, float angle, const Colour &color) {
        return std::make_shared<Cone>(pose, height, angle, color);
    }

}