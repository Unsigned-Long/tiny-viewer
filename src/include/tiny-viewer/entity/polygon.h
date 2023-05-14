//
// Created by csl on 5/14/23.
//

#ifndef TINY_VIEWER_POLYGON_H
#define TINY_VIEWER_POLYGON_H


#include "entity.h"

namespace ns_viewer {
    struct Polygon : public Entity {
    public:
        using Ptr = std::shared_ptr<Polygon>;

    protected:
        std::vector<Eigen::Vector3f> verts;

        Colour color;
        bool lineMode{};

    public:
        Polygon(const std::vector<Eigen::Vector3f> &verts, bool lineMode, const Colour &color = GetUniqueColour());

        static Ptr
        Create(const std::vector<Eigen::Vector3f> &verts, bool lineMode, const Colour &color = GetUniqueColour());

        ~Polygon() override;

        void Draw() const override;

        Polygon() = default;

    public:

        template<class Archive>
        void serialize(Archive &archive) {
            Entity::serialize(archive);
            archive(CEREAL_NVP(verts), CEREAL_NVP(color), CEREAL_NVP(lineMode));
        }
    };
}

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Polygon, "Polygon")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Polygon)

#endif //TINY_VIEWER_POLYGON_H
