//
// Created by csl on 5/9/23.
//

#include "tiny-viewer/entity/lidar.h"
#include "pangolin/gl/gldraw.h"
#include "pangolin/gl/gl.h"

namespace ns_viewer {

    LiDAR::LiDAR(const Posef &pose, float size, const Colour &color)
            : Entity(), _coord(pose, size), _color(color) {
        Eigen::Vector3f p = pose.translation;
        Eigen::Vector3f x = pose.rotation.col(0) * size;
        Eigen::Vector3f y = pose.rotation.col(1) * size;
        Eigen::Vector3f z = pose.rotation.col(2) * size;
        const float deltaAng = M_PI * 2.0f / 8.0f;

        for (int i = 0; i < 8; ++i) {
            float ang = deltaAng * static_cast<float>(i);
            float c = std::cos(ang);
            float s = std::sin(ang);
            _tops.at(i) = p + c * x + s * y + z;
            _bottoms.at(i) = p + c * x + s * y - z;
        }
    }

    LiDAR::LiDAR(const Posef &pose, const Colour &color, float size) : LiDAR(pose, size, color) {}

    LiDAR::Ptr LiDAR::Create(const Posef &pose, const Colour &color, float size) {
        return std::make_shared<LiDAR>(pose, size, color);
    }

    LiDAR::Ptr LiDAR::Create(const Posef &pose, float size, const Colour &color) {
        return std::make_shared<LiDAR>(pose, size, color);
    }

    LiDAR::~LiDAR() = default;

    void LiDAR::Draw() const {
        _coord.Draw();
        glColor4f(ExpandColor(_color));
        glLineWidth(DefaultLineSize);

        for (int i = 0; i < 8; ++i) {
            int j = (i + 1) % 8;
            Eigen::Vector3f tvi = _tops.at(i);
            Eigen::Vector3f tvj = _tops.at(j);
            pangolin::glDrawLine(ExpandVec3(tvi), ExpandVec3(tvj));

            Eigen::Vector3f bvi = _bottoms.at(i);
            Eigen::Vector3f bvj = _bottoms.at(j);
            pangolin::glDrawLine(ExpandVec3(bvi), ExpandVec3(bvj));

            pangolin::glDrawLine(ExpandVec3(tvi), ExpandVec3(bvi));
        }
    }
}