//
// Created by csl on 5/9/23.
//

#include "tiny-viewer/object/imu.h"

namespace ns_viewer {

    IMU::IMU(const Posef &pose, float size, const Colour &colour)
            : Entity(), coord(pose, size), cube(pose, true, size * 2.0f, size * 2.0f, size * 2.0f, colour) {}

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
        cube.Draw();
        coord.Draw();
    }
}