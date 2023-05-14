//
// Created by csl on 5/9/23.
//

#ifndef TINY_VIEWER_CUBE_H
#define TINY_VIEWER_CUBE_H

#include "entity.h"

namespace ns_viewer {
    struct Cube : public Entity {
    public:
        using Ptr = std::shared_ptr<Cube>;

    protected:
        Eigen::Vector3f v1;
        Eigen::Vector3f v2;
        Eigen::Vector3f v3;
        Eigen::Vector3f v4;

        Eigen::Vector3f v5;
        Eigen::Vector3f v6;
        Eigen::Vector3f v7;
        Eigen::Vector3f v8;

        Colour color;
        bool lineMode{};

    public:
        Cube(const Posef &pose, bool lineMode, float xWidth = DefaultCubeSize,
             float yWidth = DefaultCubeSize, float zWidth = DefaultCubeSize, const Colour &color = GetUniqueColour());

        Cube(const Posef &pose, bool lineMode, const Colour &color, float xWidth = DefaultCubeSize,
             float yWidth = DefaultCubeSize, float zWidth = DefaultCubeSize);

        static Ptr Create(const Posef &pose, bool lineMode, float xWidth = DefaultCubeSize,
                          float yWidth = DefaultCubeSize, float zWidth = DefaultCubeSize,
                          const Colour &color = GetUniqueColour());

        static Ptr Create(const Posef &pose, bool lineMode, const Colour &color, float xWidth = DefaultCubeSize,
                          float yWidth = DefaultCubeSize, float zWidth = DefaultCubeSize);

        [[nodiscard]] Eigen::Vector3f GetCenter() const;

        [[nodiscard]] std::array<Eigen::Vector3f, 8> GetVertices() const;

        ~Cube() override;

        void Draw() const override;

        Cube() = default;

    public:

        template<class Archive>
        void serialize(Archive &archive) {
            Entity::serialize(archive);
            archive(
                    CEREAL_NVP(v1), CEREAL_NVP(v2), CEREAL_NVP(v3), CEREAL_NVP(v4),
                    CEREAL_NVP(v5), CEREAL_NVP(v6), CEREAL_NVP(v7), CEREAL_NVP(v8),
                    CEREAL_NVP(color),
                    CEREAL_NVP(lineMode)
            );
        }
    };
}

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Cube, "Cube")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Cube)

#endif //TINY_VIEWER_CUBE_H
