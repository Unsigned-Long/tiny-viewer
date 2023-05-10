//
// Created by csl on 5/8/23.
//

#include "tiny-viewer/entity/coordinate.h"
#include "pangolin/gl/gldraw.h"

namespace ns_viewer {
    Coordinate::Coordinate(const ns_viewer::Posef &pose, float size)
            : Entity(), pose(pose.matrix44()), size(size) {}

    void Coordinate::Draw() const {
        glLineWidth(5.0f);
        pangolin::glDrawAxis(pose, size);
    }

    Coordinate::Ptr Coordinate::Create(const Posef &pose, float size) {
        return std::make_shared<Coordinate>(pose, size);
    }

    Coordinate::~Coordinate() = default;
}

