//
// Created by csl on 5/8/23.
//

#ifndef TINY_VIEWER_COORDINATE_H
#define TINY_VIEWER_COORDINATE_H

#include "tiny-viewer/entity/entity.h"

namespace ns_viewer {

    struct Coordinate : public Entity {
    public:
        using Ptr = std::shared_ptr<Coordinate>;

    protected:
        float _size;
        Eigen::Matrix4f _pose;

    public:

        explicit Coordinate(const ns_viewer::Posef &pose, float size = DefaultCoordSize);

        static Ptr Create(const ns_viewer::Posef &pose, float size = DefaultCoordSize);

        ~Coordinate() override;

        void Draw() const override;

    };

}


#endif //TINY_VIEWER_COORDINATE_H
