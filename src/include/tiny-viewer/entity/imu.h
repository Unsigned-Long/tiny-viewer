//
// Created by csl on 5/9/23.
//

#ifndef TINY_VIEWER_IMU_H
#define TINY_VIEWER_IMU_H

#include "entity.h"
#include "coordinate.h"
#include "cube.h"

namespace ns_viewer {

    struct IMU : public Entity {
    protected:
        Coordinate _coord;
        Cube _cube;

    public:
        explicit IMU(const Posef &pose, float size = DefaultIMUSize);

        ~IMU() override;

        void Draw() const override;
    };
}


#endif //TINY_VIEWER_IMU_H
