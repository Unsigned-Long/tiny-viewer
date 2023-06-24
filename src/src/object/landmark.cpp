//
// Created by csl on 6/24/23.
//

#include "tiny-viewer/object/landmark.h"
#include "pangolin/gl/gldraw.h"

namespace ns_viewer {

    Landmark::Landmark(const Vector3f &p, float size, const Colour &color)
            : color(color), size(size) {
        constexpr float scales[7][3] = {
                {1.0f,   0.0f,  0.0f},
                {0.0f,   1.0f,  0.0f},
                {0.0f,   0.0f,  1.0f},
                {0.58f,  0.58f, 0.58f},
                {-0.58f, 0.58f, 0.58f},
                {0.58f,  0.58f, -0.58f},
                {-0.58f, 0.58f, -0.58f},
        };

        for (int i = 0; i < verts.size(); i += 2) {
            Eigen::Vector3f delta = Eigen::Vector3f{scales[i / 2][0], scales[i / 2][1], scales[i / 2][2]} * size;
            verts.at(i + 0) = p + delta, verts.at(i + 1) = p - delta;
        }
    }

    Landmark::Landmark(const Vector3f &p, const Colour &color, float size)
            : Landmark(p, size, color) {}

    Landmark::Ptr Landmark::Create(const Vector3f &p, float size, const Colour &color) {
        return std::make_shared<Landmark>(p, size, color);
    }

    Landmark::Ptr Landmark::Create(const Vector3f &p, const Colour &color, float size) {
        return std::make_shared<Landmark>(p, color, size);
    }

    void Landmark::Draw() const {
        glColor4f(ExpandColor(color));
        glLineWidth(DefaultLineSize);
        for (int i = 0; i < verts.size(); i += 2) {
            pangolin::glDrawLine(ExpandVec3(verts.at(i + 0)), ExpandVec3(verts.at(i + 1)));
        }
    }
}