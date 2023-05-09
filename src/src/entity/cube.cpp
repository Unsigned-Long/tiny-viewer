//
// Created by csl on 5/9/23.
//

#include "tiny-viewer/entity/cube.h"
#include "pangolin/gl/gldraw.h"

namespace ns_viewer {

    Cube::Cube(const Posef &pose, bool lineMode, float xWidth, float yWidth, float zWidth, const Colour &color)
            : Entity(), _lineMode(lineMode), _color(color) {
        Eigen::Vector3f p = pose.translation;
        Eigen::Vector3f x = pose.rotation.col(0) * xWidth * 0.5f;
        Eigen::Vector3f y = pose.rotation.col(1) * yWidth * 0.5f;
        Eigen::Vector3f z = pose.rotation.col(2) * zWidth * 0.5f;

        v1 = p + x + y + z;
        v2 = p + x - y + z;
        v3 = p + x + y - z;
        v4 = p + x - y - z;

        v5 = p - x + y + z;
        v6 = p - x - y + z;
        v7 = p - x + y - z;
        v8 = p - x - y - z;
    }

    Cube::Cube(const Posef &pose, bool lineMode, const Colour &color, float xWidth, float yWidth, float zWidth)
            : Cube(pose, lineMode, xWidth, yWidth, zWidth, color) {}

    Cube::~Cube() = default;

    void Cube::Draw() const {
        glColor4f(ExpandColor(_color));

        if (_lineMode) {
            glLineWidth(DefaultLineSize);
            pangolin::glDrawLine(ExpandVec3(v1), ExpandVec3(v2));
            pangolin::glDrawLine(ExpandVec3(v3), ExpandVec3(v4));
            pangolin::glDrawLine(ExpandVec3(v1), ExpandVec3(v3));
            pangolin::glDrawLine(ExpandVec3(v2), ExpandVec3(v4));

            pangolin::glDrawLine(ExpandVec3(v5), ExpandVec3(v6));
            pangolin::glDrawLine(ExpandVec3(v7), ExpandVec3(v8));
            pangolin::glDrawLine(ExpandVec3(v5), ExpandVec3(v7));
            pangolin::glDrawLine(ExpandVec3(v6), ExpandVec3(v8));

            pangolin::glDrawLine(ExpandVec3(v1), ExpandVec3(v5));
            pangolin::glDrawLine(ExpandVec3(v2), ExpandVec3(v6));
            pangolin::glDrawLine(ExpandVec3(v3), ExpandVec3(v7));
            pangolin::glDrawLine(ExpandVec3(v4), ExpandVec3(v8));

        } else {

            const GLfloat verts[] = {
                    ExpandVec3(v1), ExpandVec3(v2), ExpandVec3(v3), ExpandVec3(v4),  // FRONT
                    ExpandVec3(v5), ExpandVec3(v6), ExpandVec3(v7), ExpandVec3(v8),  // BACK
                    ExpandVec3(v1), ExpandVec3(v3), ExpandVec3(v5), ExpandVec3(v7),  // LEFT
                    ExpandVec3(v2), ExpandVec3(v4), ExpandVec3(v6), ExpandVec3(v8),  // RIGHT
                    ExpandVec3(v1), ExpandVec3(v2), ExpandVec3(v5), ExpandVec3(v6),  // TOP
                    ExpandVec3(v3), ExpandVec3(v4), ExpandVec3(v7), ExpandVec3(v8)   // BOTTOM
            };

            glVertexPointer(3, GL_FLOAT, 0, verts);
            glEnableClientState(GL_VERTEX_ARRAY);

            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
            glDrawArrays(GL_TRIANGLE_STRIP, 4, 4);

            glDrawArrays(GL_TRIANGLE_STRIP, 8, 4);
            glDrawArrays(GL_TRIANGLE_STRIP, 12, 4);

            glDrawArrays(GL_TRIANGLE_STRIP, 16, 4);
            glDrawArrays(GL_TRIANGLE_STRIP, 20, 4);

            glDisableClientState(GL_VERTEX_ARRAY);
        }
    }
}