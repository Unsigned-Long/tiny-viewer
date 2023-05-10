//
// Created by csl on 5/8/23.
//

#include "tiny-viewer/entity/entity.h"

namespace ns_viewer {

    ColourWheel Entity::COLOR_WHEEL = ColourWheel(1.0f);
    std::size_t Entity::COUNT = 0;

    Entity::Entity() : id(GenUniqueName()) {}

    Entity::~Entity() = default;

    std::size_t Entity::GenUniqueName() {
        return COUNT++;
    }

    Colour Entity::GetUniqueColour() {
        return COLOR_WHEEL.GetUniqueColour();
    }

    const std::size_t &Entity::GetId() const {
        return id;
    }
}
