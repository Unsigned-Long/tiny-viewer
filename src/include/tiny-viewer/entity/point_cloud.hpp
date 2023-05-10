//
// Created by csl on 5/9/23.
//

#ifndef TINY_VIEWER_POINT_CLOUD_H
#define TINY_VIEWER_POINT_CLOUD_H

#include "entity.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pangolin/gl/gl.h>

namespace ns_viewer {

    template<typename PointType>
    struct Cloud : public Entity {
    public:
        using Ptr = std::shared_ptr<Cloud>;

    protected:
        using PointCloud = pcl::PointCloud<PointType>;
        using PointCloudPtr = typename PointCloud::Ptr;

        PointCloudPtr cloud;

        float size;
        Colour color;
    public:
        explicit Cloud(PointCloudPtr cloud, float size = DefaultPointSize, const Colour &colour = GetUniqueColour())
                : Entity(), cloud(std::move(cloud)), color(colour), size(size) {}

        static Ptr
        Create(const PointCloudPtr &cloud, float size = DefaultPointSize, const Colour &colour = GetUniqueColour()) {
            return std::make_shared<Cloud>(cloud, size, colour);
        }

        static Cloud::Ptr Random(float bound, std::size_t count, const Eigen::Vector3f &center) {
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
            return Cloud::Create(cloud);
        }

        ~Cloud() override = default;

        void Draw() const override {
            glPointSize(size);
            glColor4f(ExpandColor(color));
            glBegin(GL_POINTS);
            for (const auto &p: cloud->points) {
                glVertex3f(ExpandPCLPointXYZ(p));
            }
            glEnd();
        }
    };

    template<>
    struct Cloud<pcl::PointXYZRGBA> : public Entity {
    public:
        using Ptr = std::shared_ptr<Cloud>;

    protected:
        using PointCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
        using PointCloudPtr = PointCloud::Ptr;

        PointCloudPtr cloud;

        float size;
    public:
        explicit Cloud(PointCloudPtr cloud, float size = DefaultPointSize)
                : Entity(), cloud(std::move(cloud)), size(size) {}

        static Ptr Create(const PointCloudPtr &cloud, float size = DefaultPointSize) {
            return std::make_shared<Cloud>(cloud, size);
        }

        static Cloud::Ptr Random(float bound, std::size_t count, const Eigen::Vector3f &center) {
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
            return Cloud::Create(cloud);
        }

        ~Cloud() override = default;

        void Draw() const override {
            glPointSize(size);
            glBegin(GL_POINTS);
            for (const auto &p: cloud->points) {
                glColor4f(ExpandPCLColor(p));
                glVertex3f(ExpandPCLPointXYZ(p));
            }
            glEnd();
        }
    };
}

#endif //TINY_VIEWER_POINT_CLOUD_H
