//
// Created by csl on 5/9/23.
//

#ifndef TINY_VIEWER_CUBE_H
#define TINY_VIEWER_CUBE_H

#include "entity.h"

namespace ns_viewer {
    struct Cube : public Entity {
    protected:
        Eigen::Vector3f v1;
        Eigen::Vector3f v2;
        Eigen::Vector3f v3;
        Eigen::Vector3f v4;

        Eigen::Vector3f v5;
        Eigen::Vector3f v6;
        Eigen::Vector3f v7;
        Eigen::Vector3f v8;


        Colour _color;
        bool _lineMode;

    public:
        Cube(const Posef &pose, bool lineMode, float xWidth = DefaultCubeSize,
             float yWidth = DefaultCubeSize, float zWidth = DefaultCubeSize, const Colour &color = GetUniqueColour());

        Cube(const Posef &pose, bool lineMode, const Colour &color, float xWidth = DefaultCubeSize,
             float yWidth = DefaultCubeSize, float zWidth = DefaultCubeSize);

        ~Cube() override;

        void Draw() const override;
    };
}


#endif //TINY_VIEWER_CUBE_H
