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

        Cube cube;

        Colour color;

    public:
        explicit CubeCamera(const Posef &pose, float size = DefaultCameraSize, const Colour &color = Colour::Green());

        explicit CubeCamera(const Posef &pose, const Colour &color, float size = DefaultCameraSize);

        static Ptr Create(const Posef &pose, float size = DefaultCameraSize, const Colour &color = Colour::Green());

        static Ptr Create(const Posef &pose, const Colour &color, float size = DefaultCameraSize);

        ~CubeCamera() override;

        void Draw() const override;
    };
}


#endif //TINY_VIEWER_CAMERA_H
