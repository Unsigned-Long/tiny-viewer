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
            : Entity(), cloud(std::move(cloud)), size(size) {}

    PosColorCloud::~PosColorCloud() = default;

    void PosColorCloud::Draw() const {
        glPointSize(size);
        glBegin(GL_POINTS);
        for (const auto &p: cloud->points) {
            glColor4f(ExpandPCLColor(p));
            glVertex3f(ExpandPCLPointXYZ(p));
        }
        glEnd();
    }

    PosColorCloud::Ptr PosColorCloud::Random(float bound, std::size_t count, const Eigen::Vector3f &center) {
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
        return PosColorCloud::Create(cloud);
    }

    PosColorCloud::Ptr PosColorCloud::Create(const PosColorCloud::PointCloudPtr &cloud, float size) {
        return std::make_shared<PosColorCloud>(cloud, size);
    }

    // --------
    // PosCloud
    // --------
    PosCloud::PosCloud(PosCloud::PointCloudPtr cloud, float size, const Colour &colour)
            : Entity(), cloud(std::move(cloud)), color(colour), size(size) {}

    PosCloud::~PosCloud() = default;

    void PosCloud::Draw() const {
        glPointSize(size);
        glColor4f(ExpandColor(color));
        glBegin(GL_POINTS);
        for (const auto &p: cloud->points) {
            glVertex3f(ExpandPCLPointXYZ(p));
        }
        glEnd();
    }

    PosCloud::Ptr PosCloud::Random(float bound, std::size_t count, const Vector3f &center) {
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
        return PosCloud::Create(cloud);
    }

    PosCloud::Ptr PosCloud::Create(const PosCloud::PointCloudPtr &cloud, float size, const Colour &colour) {
        return std::make_shared<PosCloud>(cloud, size, colour);
    }
}


