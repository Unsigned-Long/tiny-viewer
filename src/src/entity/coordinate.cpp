//
// Created by csl on 5/8/23.
//

#include "tiny-viewer/entity/coordinate.h"
#include "pangolin/gl/gldraw.h"

namespace ns_viewer {
    Coordinate::Coordinate(const ns_viewer::Posef &pose, float size)
            : Entity(), _pose(pose.matrix44()), _size(size) {}

    void Coordinate::Draw() const {
        glLineWidth(5.0f);
        pangolin::glDrawAxis(_pose, _size);
    }

    Coordinate::~Coordinate() = default;
}

