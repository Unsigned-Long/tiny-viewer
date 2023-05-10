//
// Created by csl on 5/10/23.
//

#include "tiny-viewer/entity/arrow.h"
#include "pangolin/gl/gldraw.h"

namespace ns_viewer {
    Arrow::Arrow(Eigen::Vector3f sp, Eigen::Vector3f ep, float size, const Colour &color)
            : Entity(), sp(std::move(sp)), ep(std::move(ep)), size(size), color(color) {
        Eigen::Vector3f vec = this->ep - this->sp;
        float len = vec.norm();
        Eigen::Vector3f dir = len * 0.05f * vec.normalized();
        Eigen::Vector3f r = len * 0.025f * TangentBasis(dir).first;
        mp = this->sp + dir;

        const float deltaAng = M_PI * 2.0f / 4.0f;

        for (int i = 0; i < 4; ++i) {
            float ang = deltaAng * static_cast<float>(i);
            Vector3f rv = Eigen::AngleAxisf(ang, dir.normalized()) * r;
            verts.at(i) = this->sp + dir + rv;
        }
    }

    Arrow::Arrow(Eigen::Vector3f sp, Eigen::Vector3f ep, const Colour &color, float size)
            : Arrow(std::move(sp), std::move(ep), size, color) {}

    Arrow::~Arrow() = default;

    void Arrow::Draw() const {
        glColor4f(ExpandColor(color));
        glLineWidth(size);
        pangolin::glDrawLine(ExpandVec3(sp), ExpandVec3(ep));
        for (int i = 0; i < 4; ++i) {
            pangolin::glDrawLine(ExpandVec3(verts.at(i)), ExpandVec3(sp));
            pangolin::glDrawLine(ExpandVec3(verts.at(i)), ExpandVec3(mp));
        }

        glPointSize(size * 2.0f);
        glBegin(GL_POINTS);
        glVertex3f(ExpandVec3(sp));
        glVertex3f(ExpandVec3(ep));
        glVertex3f(ExpandVec3(mp));
        for (int i = 0; i < 4; ++i) {
            glVertex3f(ExpandVec3(verts.at(i)));
        }
        glEnd();
    }

    std::shared_ptr<Arrow> Arrow::Create(const Vector3f &sp, const Vector3f &ep, const Colour &color, float size) {
        return std::make_shared<Arrow>(sp, ep, size, color);
    }

    std::shared_ptr<Arrow> Arrow::Create(const Vector3f &sp, const Vector3f &ep, float size, const Colour &color) {
        return std::make_shared<Arrow>(sp, ep, size, color);
    }
}