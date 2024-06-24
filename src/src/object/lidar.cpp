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

    LivoxLiDAR::LivoxLiDAR(const Posef &pose, float size, const Colour &color)
            : Entity(), coord(pose, size), cube(pose, true, size * 2.0f, size, size, color) {
        const auto &vertices = cube.GetVertices();
        Eigen::Vector3f v1 = vertices.at(0) + 0.25f * (vertices.at(3) - vertices.at(0));
        Eigen::Vector3f v2 = vertices.at(3) + 0.25f * (vertices.at(0) - vertices.at(3));
        Eigen::Vector3f v3 = vertices.at(1) + 0.25f * (vertices.at(2) - vertices.at(1));
        Eigen::Vector3f v4 = vertices.at(2) + 0.25f * (vertices.at(1) - vertices.at(2));

        lines.at(0) = Line(v1, vertices.at(0), color);
        lines.at(1) = Line(v2, vertices.at(3), color);
        lines.at(2) = Line(v3, vertices.at(1), color);
        lines.at(3) = Line(v4, vertices.at(2), color);

        lines.at(4) = Line(v1, v3, color);
        lines.at(5) = Line(v1, v4, color);
        lines.at(6) = Line(v2, v3, color);
        lines.at(7) = Line(v2, v4, color);

        lines.at(8) = Line(vertices.at(4), vertices.at(7), color);
        lines.at(9) = Line(vertices.at(6), vertices.at(5), color);
    }

    LivoxLiDAR::LivoxLiDAR(const Posef &pose, const Colour &color, float size) : LivoxLiDAR(pose, size, color) {}

    LivoxLiDAR::Ptr LivoxLiDAR::Create(const Posef &pose, const Colour &color, float size) {
        return std::make_shared<LivoxLiDAR>(pose, size, color);
    }

    LivoxLiDAR::Ptr LivoxLiDAR::Create(const Posef &pose, float size, const Colour &color) {
        return std::make_shared<LivoxLiDAR>(pose, size, color);
    }

    LivoxLiDAR::~LivoxLiDAR() = default;

    void LivoxLiDAR::Draw() const {
        coord.Draw();
        cube.Draw();
        for (const auto &line: lines) { line.Draw(); }
    }
}