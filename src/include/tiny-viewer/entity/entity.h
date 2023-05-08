//
// Created by csl on 5/8/23.
//

#ifndef TINY_VIEWER_ENTITY_H
#define TINY_VIEWER_ENTITY_H

#include "tiny-viewer/pose.hpp"
#include "string"
#include "set"
#include "tiny-viewer/colour.hpp"
#include "tiny-viewer/macro.hpp"

namespace ns_viewer {

    struct Entity {
    private:
        std::size_t _id;
        static std::size_t COUNT;
        static ColourWheel COLOR_WHEEL;

    public:
        explicit Entity();

        virtual ~Entity();

        virtual void Draw() const = 0;

        static Colour GetUniqueColour();

        [[nodiscard]] const std::size_t &GetId() const;

    private:
        static std::size_t GenUniqueName();
    };
}


#endif //TINY_VIEWER_ENTITY_H
