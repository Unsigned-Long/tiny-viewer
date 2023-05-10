//
// Created by csl on 5/10/23.
//

#ifndef TINY_VIEWER_CONE_H
#define TINY_VIEWER_CONE_H

#include "entity.h"
#include "coordinate.h"

namespace ns_viewer {
    struct Cone : public Entity {
    public:
        using Ptr = std::shared_ptr<Cone>;

    protected:
        Eigen::Vector3f tp;
        Eigen::Vector3f bp;

        std::array<Eigen::Vector3f, 8> verts;

        Colour color;
    public:
        Cone(const Posef &pose, float height, float angle, const Colour &color = GetUniqueColour());

        static Ptr Create(const Posef &pose, float height, float angle, const Colour &color = GetUniqueColour());

        ~Cone() override;

        void Draw() const override;
    };
}

#endif //TINY_VIEWER_CONE_H
