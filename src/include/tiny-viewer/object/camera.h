//
// Created by csl on 5/9/23.
//

#ifndef TINY_VIEWER_CAMERA_H
#define TINY_VIEWER_CAMERA_H

#include "tiny-viewer/entity/entity.h"
#include "tiny-viewer/entity/coordinate.h"
#include "tiny-viewer/entity/cube.h"

namespace ns_viewer {

    struct Camera : public Entity {
    public:
        using Ptr = std::shared_ptr<Camera>;

    protected:
        Coordinate coord;

        Eigen::Vector3f v0;
        Eigen::Vector3f v1;
        Eigen::Vector3f v2;
        Eigen::Vector3f v3;
        Eigen::Vector3f v4;

        Colour color;

    public:
        explicit Camera(const Posef &pose, float size = DefaultCameraSize, const Colour &color = Colour::Green());

        explicit Camera(const Posef &pose, const Colour &color, float size = DefaultCameraSize);

        static Ptr Create(const Posef &pose, float size = DefaultCameraSize, const Colour &color = Colour::Green());

        static Ptr Create(const Posef &pose, const Colour &color, float size = DefaultCameraSize);

        ~Camera() override;

        void Draw() const override;

        Camera() = default;

    public:

        template<class Archive>
        void serialize(Archive &archive) {
            Entity::serialize(archive);
            archive(
                    CEREAL_NVP(coord),
                    CEREAL_NVP(v0), CEREAL_NVP(v1), CEREAL_NVP(v2), CEREAL_NVP(v3), CEREAL_NVP(v4),
                    CEREAL_NVP(color)
            );
        }
    };

    struct CubeCamera : public Entity {
    public:
        using Ptr = std::shared_ptr<CubeCamera>;

    protected:
        Coordinate coord;

        Eigen::Vector3f v0;
        Eigen::Vector3f v1;
        Eigen::Vector3f v2;
        Eigen::Vector3f v3;
        Eigen::Vector3f v4;

        Colour color;
        Cube cube;

    public:
        explicit CubeCamera(const Posef &pose, float size = DefaultCameraSize, const Colour &color = Colour::Green());

        explicit CubeCamera(const Posef &pose, const Colour &color, float size = DefaultCameraSize);

        static Ptr Create(const Posef &pose, float size = DefaultCameraSize, const Colour &color = Colour::Green());

        static Ptr Create(const Posef &pose, const Colour &color, float size = DefaultCameraSize);

        ~CubeCamera() override;

        void Draw() const override;

        CubeCamera() = default;

    public:

        template<class Archive>
        void serialize(Archive &archive) {
            Entity::serialize(archive);
            archive(
                    CEREAL_NVP(coord),
                    CEREAL_NVP(v0), CEREAL_NVP(v1), CEREAL_NVP(v2), CEREAL_NVP(v3), CEREAL_NVP(v4),
                    CEREAL_NVP(cube), CEREAL_NVP(color)
            );
        }
    };
}
CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Camera, "camera")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Camera)

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::CubeCamera, "CubeCamera")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::CubeCamera)

#endif //TINY_VIEWER_CAMERA_H
