//
// Created by csl on 5/9/23.
//

#ifndef TINY_VIEWER_LIDAR_H
#define TINY_VIEWER_LIDAR_H

#include "tiny-viewer/entity/entity.h"
#include "tiny-viewer/entity/coordinate.h"

namespace ns_viewer {
    struct LiDAR : public Entity {
    public:
        using Ptr = std::shared_ptr<LiDAR>;
    protected:
        Coordinate coord;

        std::array<Eigen::Vector3f, 8> tops;
        std::array<Eigen::Vector3f, 8> bottoms;

        Colour color;

    public:
        explicit LiDAR(const Posef &pose, float size = DefaultLiDARSize, const Colour &color = Colour::Blue());

        explicit LiDAR(const Posef &pose, const Colour &color, float size = DefaultLiDARSize);

        Ptr static Create(const Posef &pose, float size = DefaultLiDARSize, const Colour &color = Colour::Blue());

        Ptr static Create(const Posef &pose, const Colour &color, float size = DefaultLiDARSize);

        ~LiDAR() override;

        void Draw() const override;
    };
}

#endif //TINY_VIEWER_LIDAR_H
