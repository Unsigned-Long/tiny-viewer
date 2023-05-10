//
// Created by csl on 5/10/23.
//

#include "pangolin/gl/gldraw.h"
#include "tiny-viewer/object/surfel.h"

namespace ns_viewer {

    Surfel::Surfel(const Posef &pose, float mainWidth, float subWidth, bool lineMode, const Colour &color,
                   pangolin::AxisDirection mainAxis, pangolin::AxisDirection subAxis)
            : Entity(), lineMode(lineMode), color(color), coord(pose, 0.5f * std::min(mainWidth, subWidth)) {
        Eigen::Vector3f p = pose.translation;

        Eigen::Vector3f ma = 0.5f * mainWidth * pose.rotation *
                             Eigen::Vector3f(ExpandAryVec3(pangolin::AxisDirectionVector[mainAxis]));
        Eigen::Vector3f sa = 0.5f * subWidth * pose.rotation *
                             Eigen::Vector3f(ExpandAryVec3(pangolin::AxisDirectionVector[subAxis]));

        v1 = p + ma + sa;
        v2 = p + ma - sa;
        v3 = p - ma - sa;
        v4 = p - ma + sa;
    }

    Surfel::Ptr Surfel::Create(const Posef &pose, float mainWidth, float subWidth, bool lineMode, const Colour &color,
                               pangolin::AxisDirection mainAxis, pangolin::AxisDirection subAxis) {
        return std::make_shared<Surfel>(pose, mainWidth, subWidth, lineMode, color, mainAxis, subAxis);
    }

    Surfel::~Surfel() = default;

    void Surfel::Draw() const {
        coord.Draw();
        glColor4f(ExpandColor(color));
        if (lineMode) {
            glLineWidth(DefaultLineSize);
            pangolin::glDrawLine(ExpandVec3(v1), ExpandVec3(v2));
            pangolin::glDrawLine(ExpandVec3(v2), ExpandVec3(v3));
            pangolin::glDrawLine(ExpandVec3(v3), ExpandVec3(v4));
            pangolin::glDrawLine(ExpandVec3(v4), ExpandVec3(v1));
        } else {
            const GLfloat verts[] = {
                    ExpandVec3(v1), ExpandVec3(v2), ExpandVec3(v3),
                    ExpandVec3(v1), ExpandVec3(v3), ExpandVec3(v4),
            };

            glVertexPointer(3, GL_FLOAT, 0, verts);
            glEnableClientState(GL_VERTEX_ARRAY);
            glDrawArrays(GL_TRIANGLES, 0, 6);

            glDisableClientState(GL_VERTEX_ARRAY);
        }
        glPointSize(DefaultPointSize);
        glBegin(GL_POINTS);
        glVertex3f(ExpandVec3(v1));
        glVertex3f(ExpandVec3(v2));
        glVertex3f(ExpandVec3(v3));
        glVertex3f(ExpandVec3(v4));
        glEnd();
    }
}