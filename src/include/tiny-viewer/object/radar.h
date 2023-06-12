//
// Created by csl on 6/12/23.
//

#ifndef TINY_VIEWER_RADAR_H
#define TINY_VIEWER_RADAR_H


#include "tiny-viewer/entity/entity.h"
#include "tiny-viewer/entity/coordinate.h"

namespace ns_viewer {

    struct Radar : public Entity {
    public:
        using Ptr = std::shared_ptr<Radar>;

    protected:
        Coordinate coord;

        std::array<Eigen::Vector3f, 8> tVerts;
        Eigen::Vector3f cenVert;
        std::array<Eigen::Vector3f, 8> bVerts;

        Colour color;

    public:

        explicit Radar(const Posef &pose, float size = DefaultRadarSize,
                       const Colour &color = Colour(0.2f, 0.2f, 0.2f));

        explicit Radar(const Posef &pose, const Colour &color, float size = DefaultRadarSize);

        static Ptr
        Create(const Posef &pose, float size = DefaultRadarSize, const Colour &color = Colour(0.2f, 0.2f, 0.2f));

        static Ptr Create(const Posef &pose, const Colour &color, float size = DefaultRadarSize);

        ~Radar() override = default;

        void Draw() const override;

        Radar() = default;

    public:

        template<class Archive>
        void serialize(Archive &archive) {
            Entity::serialize(archive);
            archive(CEREAL_NVP(coord), CEREAL_NVP(tVerts), CEREAL_NVP(cenVert), CEREAL_NVP(bVerts), CEREAL_NVP(color));
        }
    };
}

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Radar, "Radar")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Radar)

#endif //TINY_VIEWER_RADAR_H
