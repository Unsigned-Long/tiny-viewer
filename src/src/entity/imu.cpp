//
// Created by csl on 5/9/23.
//

#include "tiny-viewer/entity/imu.h"

namespace ns_viewer {

    IMU::IMU(const Posef &pose, float size)
            : Entity(), _coord(pose, size * 0.5f), _cube(pose, true, size, size, size) {}

    IMU::~IMU() = default;

    void IMU::Draw() const {
        _cube.Draw();
        _coord.Draw();
    }
}