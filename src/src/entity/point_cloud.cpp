//
// Created by csl on 5/9/23.
//

#include <utility>
#include "tiny-viewer/entity/point_cloud.h"
#include "tiny-viewer/entity/line.h"
#include <pangolin/gl/gl.h>

namespace ns_viewer {
    // -------------
    // PosColorCloud
    // -------------
    PosColorCloud::PosColorCloud(PointCloudPtr cloud, float size)
            : Entity(), _cloud(std::move(cloud)), _size(size) {}

    PosColorCloud::~PosColorCloud() = default;

    void PosColorCloud::Draw() const {
        glPointSize(_size);
        glBegin(GL_POINTS);
        for (const auto &p: _cloud->points) {
            glColor4f(ExpandPCLColor(p));
            glVertex3f(ExpandPCLPointXYZ(p));
        }
        glEnd();
    }

    PosColorCloud PosColorCloud::Random(float bound, std::size_t count, const Eigen::Vector3f &center) {
        std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<float> u(-bound, bound);
        PointCloudPtr cloud(new PointCloud);
        cloud->resize(count);
        for (int i = 0; i < count; ++i) {
            pcl::PointXYZRGBA &p = cloud->at(i);
            // x, y, z
            p.x = u(engine) + center(0);
            p.y = u(engine) + center(1);
            p.z = u(engine) + center(2);
            // r, g, b, a
            auto color = GetUniqueColour();
            p.r = static_cast<std::uint8_t>(color.r * 255.0f);
            p.g = static_cast<std::uint8_t>(color.g * 255.0f);
            p.b = static_cast<std::uint8_t>(color.b * 255.0f);
            p.a = static_cast<std::uint8_t>(color.a * 255.0f);

        }
        return PosColorCloud(cloud);
    }

    // --------
    // PosCloud
    // --------
    PosCloud::PosCloud(PosCloud::PointCloudPtr cloud, float size, const Colour &colour)
            : Entity(), _cloud(std::move(cloud)), _color(colour), _size(size) {}

    PosCloud::~PosCloud() = default;

    void PosCloud::Draw() const {
        glPointSize(_size);
        glColor4f(ExpandColor(_color));
        glBegin(GL_POINTS);
        for (const auto &p: _cloud->points) {
            glVertex3f(ExpandPCLPointXYZ(p));
        }
        glEnd();
    }

    PosCloud PosCloud::Random(float bound, std::size_t count, const Vector3f &center) {
        std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<float> u(-bound, bound);
        PointCloudPtr cloud(new PointCloud);
        cloud->resize(count);
        for (int i = 0; i < count; ++i) {
            pcl::PointXYZ &p = cloud->at(i);
            // x, y, z
            p.x = u(engine) + center(0);
            p.y = u(engine) + center(1);
            p.z = u(engine) + center(2);
        }
        return PosCloud(cloud);
    }
}


