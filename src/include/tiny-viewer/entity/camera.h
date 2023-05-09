//
// Created by csl on 5/9/23.
//

#ifndef TINY_VIEWER_CAMERA_H
#define TINY_VIEWER_CAMERA_H

#include "entity.h"
#include "coordinate.h"

namespace ns_viewer {

    struct Camera : public Entity {
    public:
        using Ptr = std::shared_ptr<Camera>;

    protected:
        Coordinate _coord;

        Eigen::Vector3f _v0;
        Eigen::Vector3f _v1;
        Eigen::Vector3f _v2;
        Eigen::Vector3f _v3;
        Eigen::Vector3f _v4;

        Colour _color;

    public:
        explicit Camera(const Posef &pose, float size = DefaultCameraSize, const Colour &color = Colour::Green());

        explicit Camera(const Posef &pose, const Colour &color, float size = DefaultCameraSize);

        static Ptr Create(const Posef &pose, float size = DefaultCameraSize, const Colour &color = Colour::Green());

        static Ptr Create(const Posef &pose, const Colour &color, float size = DefaultCameraSize);

        ~Camera() override;

        void Draw() const override;
    };
}


#endif //TINY_VIEWER_CAMERA_H
