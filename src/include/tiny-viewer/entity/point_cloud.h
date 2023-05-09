//
// Created by csl on 5/9/23.
//

#ifndef TINY_VIEWER_POINT_CLOUD_H
#define TINY_VIEWER_POINT_CLOUD_H

#include "entity.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace ns_viewer {
    struct PosColorCloud : public Entity {
    public:
        using Ptr = std::shared_ptr<PosColorCloud>;

    protected:
        using PointCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
        using PointCloudPtr = PointCloud::Ptr;

        PointCloudPtr _cloud;

        float _size;
    public:
        explicit PosColorCloud(PointCloudPtr cloud, float size = DefaultPointSize);

        static Ptr Create(const PointCloudPtr &cloud, float size = DefaultPointSize);

        static PosColorCloud::Ptr Random(float bound, std::size_t count, const Eigen::Vector3f &center);

        ~PosColorCloud() override;

        void Draw() const override;
    };

    struct PosCloud : public Entity {
    public:
        using Ptr = std::shared_ptr<PosCloud>;

    protected:
        using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
        using PointCloudPtr = PointCloud::Ptr;

        PointCloudPtr _cloud;

        float _size;
        Colour _color;
    public:
        explicit PosCloud(PointCloudPtr cloud, float size = DefaultPointSize,
                          const Colour &colour = GetUniqueColour());

        static Ptr Create(const PointCloudPtr &cloud, float size = DefaultPointSize,
                          const Colour &colour = GetUniqueColour());


        static PosCloud::Ptr Random(float bound, std::size_t count, const Eigen::Vector3f &center);

        ~PosCloud() override;

        void Draw() const override;
    };
}

#endif //TINY_VIEWER_POINT_CLOUD_H
