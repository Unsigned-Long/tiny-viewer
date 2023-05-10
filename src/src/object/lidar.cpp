//
// Created by csl on 5/9/23.
//

#include "tiny-viewer/object/lidar.h"
#include "pangolin/gl/gldraw.h"
#include "pangolin/gl/gl.h"

namespace ns_viewer {

    LiDAR::LiDAR(const Posef &pose, float size, const Colour &color)
            : Entity(), coord(pose, size), color(color) {
        Eigen::Vector3f p = pose.translation;
        Eigen::Vector3f x = pose.rotation.col(0) * size;
        Eigen::Vector3f y = pose.rotation.col(1) * size;
        Eigen::Vector3f z = pose.rotation.col(2) * size;
        const float deltaAng = M_PI * 2.0f / 8.0f;

        for (int i = 0; i < 8; ++i) {
            float ang = deltaAng * static_cast<float>(i);
            float c = std::cos(ang);
            float s = std::sin(ang);
            tops.at(i) = p + c * x + s * y + z;
            bottoms.at(i) = p + c * x + s * y - z;
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
        coord.Draw();
        glColor4f(ExpandColor(color));
        glLineWidth(DefaultLineSize);

        for (int i = 0; i < 8; ++i) {
            int j = (i + 1) % 8;
            const Eigen::Vector3f &tvi = tops.at(i);
            const Eigen::Vector3f &tvj = tops.at(j);
            pangolin::glDrawLine(ExpandVec3(tvi), ExpandVec3(tvj));

            const Eigen::Vector3f &bvi = bottoms.at(i);
            const Eigen::Vector3f &bvj = bottoms.at(j);
            pangolin::glDrawLine(ExpandVec3(bvi), ExpandVec3(bvj));

            pangolin::glDrawLine(ExpandVec3(tvi), ExpandVec3(bvi));
        }
        glPointSize(DefaultPointSize);
        glBegin(GL_POINTS);
        for (int i = 0; i < 8; ++i) {
            const Eigen::Vector3f &tvi = tops.at(i);
            const Eigen::Vector3f &bvi = bottoms.at(i);
            glVertex3f(ExpandVec3(tvi));
            glVertex3f(ExpandVec3(bvi));
        }
        glEnd();
    }
}