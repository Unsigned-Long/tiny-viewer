//
// Created by csl on 5/8/23.
//

#ifndef TINY_VIEWER_LINE_H
#define TINY_VIEWER_LINE_H

#include "entity.h"

namespace ns_viewer {

    struct Line : public Entity {
    protected:
        Eigen::Vector3f _sp;
        Eigen::Vector3f _ep;

        Colour _color;
        float _size;

    public:

        Line(Eigen::Vector3f sp, Eigen::Vector3f ep, float size = DefaultLineSize,
             const Colour &color = GetUniqueColour());

        Line(Eigen::Vector3f sp, Eigen::Vector3f ep, const Colour &color, float size = DefaultLineSize);

        ~Line() override;

        void Draw() const override;

    };

}

#endif //TINY_VIEWER_LINE_H
