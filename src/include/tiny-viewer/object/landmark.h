//
// Created by csl on 6/24/23.
//

#ifndef TINY_VIEWER_LANDMARK_H
#define TINY_VIEWER_LANDMARK_H

#include "tiny-viewer/entity/entity.h"
#include "tiny-viewer/entity/point_cloud.hpp"

namespace ns_viewer {
    struct Landmark : public Entity {
    public:
        using Ptr = std::shared_ptr<Landmark>;

    protected:
        std::array<Eigen::Vector3f, 7 * 2> verts;
        Colour color;
        float size{};

    public:
        explicit Landmark(const Eigen::Vector3f &p, float size = DefaultLandmarkSize,
                          const Colour &color = GetUniqueColour());

        Landmark(const Eigen::Vector3f &p, const Colour &color, float size = DefaultLandmarkSize);

        static Ptr Create(const Eigen::Vector3f &p, float size = DefaultLandmarkSize,
                          const Colour &color = GetUniqueColour());

        static Ptr Create(const Eigen::Vector3f &p, const Colour &color, float size = DefaultLandmarkSize);

        ~Landmark() override = default;

        void Draw() const override;

        Landmark() = default;

    public:

        template<class Archive>
        void serialize(Archive &archive) {
            Entity::serialize(archive);
            archive(CEREAL_NVP(verts), CEREAL_NVP(color), CEREAL_NVP(size));
        }
    };

    template<>
    struct Cloud<Landmark> : public Entity {
    public:
        using Ptr = std::shared_ptr<Cloud>;

    protected:
        using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
        using PointCloudPtr = PointCloud::Ptr;

        using RGBPointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
        using RGBPointCloudPtr = RGBPointCloud::Ptr;

        using RGBAPointCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
        using RGBAPointCloudPtr = RGBAPointCloud::Ptr;

        std::vector<Landmark> landmarks;
    public:
        explicit Cloud(const PointCloudPtr &cloud, float size = DefaultLandmarkSize)
                : Entity() {
            landmarks.resize(cloud->size());
            for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
                const auto &p = cloud->at(i);
                landmarks.at(i) = Landmark({ExpandPCLPointXYZ(p)}, size);
            }
        }

        explicit Cloud(const RGBPointCloudPtr &cloud, float size = DefaultLandmarkSize)
                : Entity() {
            landmarks.resize(cloud->size());
            for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
                const auto &p = cloud->at(i);
                landmarks.at(i) = Landmark({ExpandPCLPointXYZ(p)}, size, Colour(ExpandPCLColor(p)));
            }
        }

        explicit Cloud(const RGBAPointCloudPtr &cloud, float size = DefaultLandmarkSize)
                : Entity() {
            landmarks.resize(cloud->size());
            for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
                const auto &p = cloud->at(i);
                landmarks.at(i) = Landmark({ExpandPCLPointXYZ(p)}, size, Colour(ExpandPCLColor(p)));
            }
        }

        static Ptr Create(const PointCloudPtr &cloud, float size = DefaultLandmarkSize) {
            return std::make_shared<Cloud>(cloud, size);
        }

        static Cloud::Ptr Random(float bound, std::size_t count, const Eigen::Vector3f &center) {
            std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
            std::uniform_real_distribution<float> u(-bound, bound);
            PointCloudPtr cloud(new PointCloud);
            cloud->resize(count);
            for (int i = 0; i < static_cast<int>(count); ++i) {
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
            for (const auto &lm: landmarks) { lm.Draw(); }
        }

        Cloud() = default;

    public:

        template<class Archive>
        void serialize(Archive &archive) {
            Entity::serialize(archive);
            archive(cereal::make_nvp("landmarks", landmarks));
        }
    };
}


CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Landmark, "Landmark")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Landmark)

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Cloud<ns_viewer::Landmark>, "Cloud::Landmark")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Cloud<ns_viewer::Landmark>)
#endif //TINY_VIEWER_LANDMARK_H
