//
// Created by csl on 5/10/23.
//

#ifndef TINY_VIEWER_SURFEL_H
#define TINY_VIEWER_SURFEL_H

#include "entity.h"
#include "pangolin/gl/opengl_render_state.h"
#include "coordinate.h"

namespace ns_viewer {
    struct Surfel : public Entity {
    public:
        using Ptr = std::shared_ptr<Surfel>;

    protected:
        Eigen::Vector3f v1;
        Eigen::Vector3f v2;
        Eigen::Vector3f v3;
        Eigen::Vector3f v4;

        Colour color;
        bool lineMode;

        Coordinate coord;

    public:
        Surfel(const Posef &pose, float mainWidth, float subWidth, bool lineMode,
               const Colour &color = GetUniqueColour(), pangolin::AxisDirection mainAxis = pangolin::AxisX,
               pangolin::AxisDirection subAxis = pangolin::AxisY);

        static Ptr Create(const Posef &pose, float mainWidth, float subWidth, bool lineMode,
                          const Colour &color = GetUniqueColour(), pangolin::AxisDirection mainAxis = pangolin::AxisX,
                          pangolin::AxisDirection subAxis = pangolin::AxisY);

        ~Surfel() override;

        void Draw() const override;
    };
}

#endif //TINY_VIEWER_SURFEL_H
