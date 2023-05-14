//
// Created by csl on 5/14/23.
//

#include "tiny-viewer/entity/polygon.h"
#include "pangolin/gl/gldraw.h"

namespace ns_viewer {

    Polygon::Polygon(const std::vector<Eigen::Vector3f> &verts, bool lineMode, const Colour &color)
            : Entity(), verts(verts), lineMode(lineMode), color(color) {}

    Polygon::~Polygon() = default;

    Polygon::Ptr Polygon::Create(const std::vector<Eigen::Vector3f> &verts, bool lineMode, const Colour &color) {
        return std::make_shared<Polygon>(verts, lineMode, color);
    }

    void Polygon::Draw() const {
        glColor4f(ExpandColor(color));
        if (lineMode) {
            glPointSize(DefaultPointSize);
            glBegin(GL_POINTS);
            for (const auto &v: verts) { glVertex3f(ExpandVec3(v)); }
            glEnd();
            glLineWidth(DefaultLineSize);
            glBegin(GL_LINE_LOOP);
            for (const auto &v: verts) { glVertex3f(ExpandVec3(v)); }
            glEnd();
        } else {
            glBegin(GL_POLYGON);
            for (const auto &v: verts) { glVertex3f(ExpandVec3(v)); }
            glEnd();
        }
    }
}