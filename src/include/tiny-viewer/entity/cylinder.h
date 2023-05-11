//
// Created by csl on 5/11/23.
//

#ifndef TINY_VIEWER_CYLINDER_H
#define TINY_VIEWER_CYLINDER_H

#include "tiny-viewer/entity/entity.h"
#include "tiny-viewer/entity/coordinate.h"

namespace ns_viewer {
    struct Cylinder : public Entity {
    public:
        using Ptr = std::shared_ptr<Cylinder>;
    protected:
        std::array<Eigen::Vector3f, 8> tops;
        std::array<Eigen::Vector3f, 8> bottoms;

        Colour color;
    public:
        explicit Cylinder(const Posef &pose, float height, float radius, const Colour &color = GetUniqueColour());

        Ptr static Create(const Posef &pose, float height, float radius, const Colour &color = GetUniqueColour());

        ~Cylinder() override;

        void Draw() const override;

        Cylinder() = default;

    public:

        template<class Archive>
        void serialize(Archive &archive) {
            Entity::serialize(archive);
            archive(CEREAL_NVP(tops), CEREAL_NVP(bottoms), CEREAL_NVP(color));
        }
    };
}

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Cylinder, "Cylinder")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Cylinder)

#endif //TINY_VIEWER_CYLINDER_H
