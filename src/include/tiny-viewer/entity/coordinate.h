//
// Created by csl on 5/8/23.
//

#ifndef TINY_VIEWER_COORDINATE_H
#define TINY_VIEWER_COORDINATE_H

#include "tiny-viewer/entity/entity.h"

namespace ns_viewer {

    struct Coordinate : public Entity {
    public:
        using Ptr = std::shared_ptr<Coordinate>;

    protected:
        float size{};
        Eigen::Matrix4f pose;

    public:

        explicit Coordinate(const ns_viewer::Posef &pose, float size = DefaultCoordSize);

        static Ptr Create(const ns_viewer::Posef &pose, float size = DefaultCoordSize);

        ~Coordinate() override;

        void Draw() const override;

        Coordinate() = default;

    public:

        template<class Archive>
        void serialize(Archive &archive) {
            Entity::serialize(archive);
            archive(CEREAL_NVP(size), CEREAL_NVP(pose));
        }
    };
}
CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Coordinate, "Coordinate")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Coordinate)

#endif //TINY_VIEWER_COORDINATE_H
