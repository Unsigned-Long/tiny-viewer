//
// Created by csl on 5/9/23.
//

#include "tiny-viewer/object/lidar.h"

namespace ns_viewer {

    LiDAR::LiDAR(const Posef &pose, float size, const Colour &color)
            : Entity(), coord(pose, size), cylinder(pose, size * 2.0f, size, color) {}

    LiDAR::LiDAR(const Posef &pose, const Colour &color, float size) : LiDAR(pose, size, color) {}

    LiDAR::Ptr LiDAR::Create(const Posef &pose, const Colour &color, float size) {
        return std::make_shared<LiDAR>(pose, size, color);
    }

    LiDAR::Ptr LiDAR::Create(const Posef &pose, float size, const Colour &color) {
        return std::make_shared<LiDAR>(pose, size, color);
    }

    LiDAR::~LiDAR() = default;

    void LiDAR::Draw() const {
        coord.Draw();
        cylinder.Draw();
    }
}