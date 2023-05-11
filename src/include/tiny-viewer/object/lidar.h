//
// Created by csl on 5/9/23.
//

#ifndef TINY_VIEWER_LIDAR_H
#define TINY_VIEWER_LIDAR_H

#include "tiny-viewer/entity/entity.h"
#include "tiny-viewer/entity/coordinate.h"
#include "tiny-viewer/entity/cylinder.h"

namespace ns_viewer {
    struct LiDAR : public Entity {
    public:
        using Ptr = std::shared_ptr<LiDAR>;
    protected:
        Coordinate coord;

        Cylinder cylinder;

    public:
        explicit LiDAR(const Posef &pose, float size = DefaultLiDARSize, const Colour &color = Colour::Blue());

        explicit LiDAR(const Posef &pose, const Colour &color, float size = DefaultLiDARSize);

        Ptr static Create(const Posef &pose, float size = DefaultLiDARSize, const Colour &color = Colour::Blue());

        Ptr static Create(const Posef &pose, const Colour &color, float size = DefaultLiDARSize);

        ~LiDAR() override;

        void Draw() const override;

        LiDAR() = default;

    public:

        template<class Archive>
        void serialize(Archive &archive) {
            Entity::serialize(archive);
            archive(CEREAL_NVP(coord), CEREAL_NVP(cylinder));
        }
    };
}

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::LiDAR, "LiDAR")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::LiDAR)

#endif //TINY_VIEWER_LIDAR_H
