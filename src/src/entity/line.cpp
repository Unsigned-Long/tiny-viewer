//
// Created by csl on 5/8/23.
//
#include <utility>
#include "tiny-viewer/entity/line.h"
#include "pangolin/gl/gldraw.h"

namespace ns_viewer {

    Line::Line(Eigen::Vector3f sp, Eigen::Vector3f ep, float size, const Colour &color)
            : Entity(), _sp(std::move(sp)), _ep(std::move(ep)), _size(size), _color(color) {}

    Line::~Line() = default;

    void Line::Draw() const {
        glColor4f(ExpandColor(_color));
        glLineWidth(_size);
        pangolin::glDrawLine(ExpandVec3(_sp), ExpandVec3(_ep));
    }
}