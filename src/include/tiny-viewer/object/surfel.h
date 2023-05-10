//
// Created by csl on 5/10/23.
//

#ifndef TINY_VIEWER_SURFEL_H
#define TINY_VIEWER_SURFEL_H

#include "tiny-viewer/entity/entity.h"
#include "pangolin/gl/opengl_render_state.h"
#include "tiny-viewer/entity/coordinate.h"

namespace ns_viewer {
    struct Surfel : public Entity {
    public:
        using Ptr = std::shared_ptr<Surfel>;

    protected:
        Eigen::Vector3f v1;
        Eigen::Vector3f v2;
        Eigen::Vector3f v3;
        Eigen::Vector3f v4;

        Colour color;
        bool lineMode{};

        Coordinate coord;

    public:
        Surfel(const Posef &pose, float mainWidth, float subWidth, bool lineMode,
               const Colour &color = GetUniqueColour(), pangolin::AxisDirection mainAxis = pangolin::AxisX,
               pangolin::AxisDirection subAxis = pangolin::AxisY);

        static Ptr Create(const Posef &pose, float mainWidth, float subWidth, bool lineMode,
                          const Colour &color = GetUniqueColour(), pangolin::AxisDirection mainAxis = pangolin::AxisX,
                          pangolin::AxisDirection subAxis = pangolin::AxisY);

        ~Surfel() override;

        void Draw() const override;

        Surfel() = default;

    public:

        template<class Archive>
        void serialize(Archive &archive) {
            Entity::serialize(archive);
            archive(
                    CEREAL_NVP(v1), CEREAL_NVP(v2), CEREAL_NVP(v3), CEREAL_NVP(v4),
                    CEREAL_NVP(color), CEREAL_NVP(lineMode), CEREAL_NVP(coord)
            );
        }
    };
}

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Surfel, "Surfel")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Surfel)
#endif //TINY_VIEWER_SURFEL_H
