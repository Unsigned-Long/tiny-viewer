//
// Created by csl on 6/12/23.
//

#include "tiny-viewer/object/radar.h"
#include "pangolin/gl/gldraw.h"

namespace ns_viewer {

    Radar::Radar(const Posef &pose, float size, const Colour &color)
            : coord(pose, size), color(color) {
        cenVert = pose.translation;
        Eigen::Vector3f x = pose.rotation.col(0);
        Eigen::Vector3f y = pose.rotation.col(1);
        Eigen::Vector3f z = pose.rotation.col(2);

        {
            Eigen::Vector3f ep = cenVert + x * size;
            const float deltaAng = M_PI * 2.0f / 8.0f;
            const auto r = static_cast<float>(size * std::tan(M_PI / 5.0));

            for (int i = 0; i < 8; ++i) {
                float ang = deltaAng * static_cast<float>(i);
                Eigen::Vector3f rv = y * r * std::cos(ang) + z * r * std::sin(ang);
                tVerts.at(i) = ep + rv;
            }
        }
        {
            Eigen::Vector3f ep = cenVert - z * size;
            const float deltaAng = M_PI * 2.0f / 4.0f;
            const auto r = static_cast<float>(size * std::tan(M_PI / 8.0));
            for (int i = 0; i < 4; ++i) {
                float ang = deltaAng * static_cast<float>(i);
                Eigen::Vector3f rv = x * r * std::cos(ang) + y * r * std::sin(ang);
                bVerts.at(i) = ep + rv;
            }
        }
    }

    void Radar::Draw() const {
        coord.Draw();

        glColor4f(ExpandColor(color));
        glLineWidth(DefaultLineSize);
        for (int i = 0; i < 8; ++i) {
            const Eigen::Vector3f &vj = tVerts.at((i + 1) % 8);
            pangolin::glDrawLine(ExpandVec3(tVerts.at(i)), ExpandVec3(cenVert));
            pangolin::glDrawLine(ExpandVec3(tVerts.at(i)), ExpandVec3(vj));
        }
        for (int i = 0; i < 4; ++i) {
            const Eigen::Vector3f &vj = bVerts.at((i + 1) % 4);
            pangolin::glDrawLine(ExpandVec3(bVerts.at(i)), ExpandVec3(cenVert));
            pangolin::glDrawLine(ExpandVec3(bVerts.at(i)), ExpandVec3(vj));
        }

        glPointSize(DefaultPointSize);
        glBegin(GL_POINTS);
        glVertex3f(ExpandVec3(cenVert));
        for (int i = 0; i < 8; ++i) { glVertex3f(ExpandVec3(tVerts.at(i))); }
        for (int i = 0; i < 4; ++i) { glVertex3f(ExpandVec3(bVerts.at(i))); }
        glEnd();
    }

    Radar::Ptr Radar::Create(const Posef &pose, float size, const Colour &color) {
        return std::make_shared<Radar>(pose, size, color);
    }

    Radar::Ptr Radar::Create(const Posef &pose, const Colour &color, float size) {
        return std::make_shared<Radar>(pose, color, size);
    }

    Radar::Radar(const Posef &pose, const Colour &color, float size)
            : Radar(pose, size, color) {}
}