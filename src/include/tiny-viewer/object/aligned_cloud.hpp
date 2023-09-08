//
// Created by csl on 5/18/23.
//

#ifndef TINY_VIEWER_ALIGNED_CLOUD_HPP
#define TINY_VIEWER_ALIGNED_CLOUD_HPP

#include "tiny-viewer/entity/point_cloud.hpp"

namespace ns_viewer {
    template<typename PointType>
    struct AlignedCloud : public Entity {
    public:
        using Ptr = std::shared_ptr<AlignedCloud>;

    protected:
        using PointCloud = pcl::PointCloud<PointType>;
        using PointCloudPtr = typename PointCloud::Ptr;

        Cloud<pcl::PointXYZI> cloud;
    public:

        explicit AlignedCloud(const PointCloudPtr &inputCloud, const Eigen::Vector3f &dir,
                              float size = DefaultPointSize, IntensityMode mode = IntensityMode::PCL_VISUALIZER_LUT_HSV)
                : Entity(), cloud(ColorizeCloudByDirection(inputCloud, dir), size, mode) {}

        static Ptr Create(const PointCloudPtr &inputCloud, const Eigen::Vector3f &dir,
                          float size = DefaultPointSize, IntensityMode mode = IntensityMode::PCL_VISUALIZER_LUT_HSV) {
            return std::make_shared<AlignedCloud>(inputCloud, dir, size, mode);
        }

        ~AlignedCloud() override = default;

        void Draw() const override {
            cloud.Draw();
        }

        AlignedCloud() = default;

        static auto
        Random(float bound, std::size_t count, const Eigen::Vector3f &center, const Eigen::Vector3f &dir) {
            std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
            std::uniform_real_distribution<float> u(-bound, bound);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            cloud->resize(count);
            for (int i = 0; i < static_cast<int>(count); ++i) {
                auto &p = cloud->at(i);
                // x, y, z
                p.x = u(engine) + center(0);
                p.y = u(engine) + center(1);
                p.z = u(engine) + center(2);
            }
            return AlignedCloud<pcl::PointXYZ>::Create(cloud, dir);
        }

    public:

        template<class Archive>
        void serialize(Archive &archive) {
            Entity::serialize(archive);
            archive(cereal::make_nvp("data", cloud));
        }

    protected:
        static pcl::PointCloud<pcl::PointXYZI>::Ptr
        ColorizeCloudByDirection(const typename pcl::PointCloud<PointType>::Ptr &iCloud, const Eigen::Vector3f &dir) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr oCloud(new pcl::PointCloud<pcl::PointXYZI>);
            oCloud->resize(iCloud->size());
            for (int i = 0; i < static_cast<int>(iCloud->size()); ++i) {
                const auto &ip = iCloud->at(i);
                auto &op = oCloud->at(i);
                op.x = ip.x, op.y = ip.y, op.z = ip.z;
                op.intensity = -Eigen::Vector3f(ip.x, ip.y, ip.z).dot(dir);
            }
            return oCloud;
        }
    };
}

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::AlignedCloud<pcl::PointXYZ>, "AlignedCloud::PointXYZ")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::AlignedCloud<pcl::PointXYZ>)

#endif //TINY_VIEWER_ALIGNED_CLOUD_HPP
