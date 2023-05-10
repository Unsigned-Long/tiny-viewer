//
// Created by csl on 5/8/23.
//

#ifndef TINY_VIEWER_ENTITY_H
#define TINY_VIEWER_ENTITY_H

#include "tiny-viewer/core/pose.hpp"
#include "string"
#include <memory>
#include "set"
#include "utils.h"
#include "cereal/cereal.hpp"

namespace ns_viewer {

    struct Entity {
    public:
        using Ptr = std::shared_ptr<Entity>;

    private:
        std::size_t id;
        static std::size_t counter;
        static ColourWheel COLOR_WHEEL;

    public:
        explicit Entity();

        virtual ~Entity();

        virtual void Draw() const = 0;

        static Colour GetUniqueColour();

        [[nodiscard]] const std::size_t &GetId() const;

    private:
        static std::size_t GenUniqueName();

    public:
        template<class Archive>
        void serialize(Archive &archive) {
            std::size_t counterBackup = counter;
            archive(CEREAL_NVP(id), CEREAL_NVP(counter));
            if (counter < counterBackup) { counter = counterBackup; }
        }
    };
}


#endif //TINY_VIEWER_ENTITY_H
