//
// Created by csl on 5/14/23.
//

#ifndef TINY_VIEWER_SURFEL_H
#define TINY_VIEWER_SURFEL_H

#include "tiny-viewer/entity/entity.h"
#include "tiny-viewer/entity/cube.h"
#include "tiny-viewer/entity/polygon.h"

namespace ns_viewer {
    struct Surfel : public Entity {
    public:
        using Ptr = std::shared_ptr<Surfel>;

    protected:

        Polygon polygon;
        bool drawCube{};
        Cube cube;

    public:
        Surfel(const Eigen::Vector4f &plane, const Cube &cube, bool lineMode, bool drawCube = true,
               const Colour &color = GetUniqueColour());

        static Ptr Create(const Eigen::Vector4f &plane, const Cube &cube, bool lineMode, bool drawCube = true,
                          const Colour &color = GetUniqueColour());

        ~Surfel() override;

        void Draw() const override;

        static Ptr Random(float bound);

        Surfel() = default;

    public:

        template<class Archive>
        void serialize(Archive &archive) {
            Entity::serialize(archive);
            archive(CEREAL_NVP(polygon), CEREAL_NVP(cube), CEREAL_NVP(drawCube));
        }
    };
}


CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Surfel, "Surfel")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Surfel)

#endif //TINY_VIEWER_SURFEL_H
