//
// Created by csl on 5/9/23.
//

#ifndef TINY_VIEWER_IMU_H
#define TINY_VIEWER_IMU_H

#include "tiny-viewer/entity/entity.h"
#include "tiny-viewer/entity/coordinate.h"
#include "tiny-viewer/entity/cube.h"

namespace ns_viewer {

    struct IMU : public Entity {
    public:
        using Ptr = std::shared_ptr<IMU>;

    protected:
        Coordinate coord;
        Cube cube;

    public:
        explicit IMU(const Posef &pose, float size = DefaultIMUSize, const Colour &colour = Colour::Red());

        explicit IMU(const Posef &pose, const Colour &colour, float size = DefaultIMUSize);

        static Ptr Create(const Posef &pose, float size = DefaultIMUSize, const Colour &colour = Colour::Red());

        static Ptr Create(const Posef &pose, const Colour &colour, float size = DefaultIMUSize);

        ~IMU() override;

        void Draw() const override;

        IMU() = default;

    public:

        template<class Archive>
        void serialize(Archive &archive) {
            Entity::serialize(archive);
            archive(CEREAL_NVP(coord), CEREAL_NVP(cube));
        }
    };
}

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::IMU, "IMU")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::IMU)

#endif //TINY_VIEWER_IMU_H
