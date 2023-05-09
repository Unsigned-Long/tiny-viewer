//
// Created by csl on 5/9/23.
//

#include "tiny-viewer/entity/imu.h"

namespace ns_viewer {

    IMU::IMU(const Posef &pose, float size, const Colour &colour)
            : Entity(), _coord(pose, size * 0.5f), _cube(pose, true, size, size, size, colour) {}

    IMU::~IMU() = default;

    IMU::Ptr IMU::Create(const Posef &pose, float size, const Colour &colour) {
        return std::make_shared<IMU>(pose, size, colour);
    }

    IMU::IMU(const Posef &pose, const Colour &colour, float size) :
            IMU(pose, size, colour) {}

    IMU::Ptr IMU::Create(const Posef &pose, const Colour &colour, float size) {
        return std::make_shared<IMU>(pose, size, colour);
    }

    void IMU::Draw() const {
        _cube.Draw();
        _coord.Draw();
    }
}