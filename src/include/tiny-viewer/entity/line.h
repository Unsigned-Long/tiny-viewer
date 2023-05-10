//
// Created by csl on 5/8/23.
//

#ifndef TINY_VIEWER_LINE_H
#define TINY_VIEWER_LINE_H

#include "entity.h"

namespace ns_viewer {

    struct Line : public Entity {
    public:
        using Ptr = std::shared_ptr<Line>;

    protected:
        Eigen::Vector3f sp;
        Eigen::Vector3f ep;

        Colour color;
        float size{};

    public:

        Line(Eigen::Vector3f sp, Eigen::Vector3f ep, float size = DefaultLineSize,
             const Colour &color = GetUniqueColour());

        Line(Eigen::Vector3f sp, Eigen::Vector3f ep, const Colour &color, float size = DefaultLineSize);

        static Ptr Create(const Eigen::Vector3f &sp, const Eigen::Vector3f &ep, float size = DefaultLineSize,
                          const Colour &color = GetUniqueColour());

        static Ptr Create(const Eigen::Vector3f &sp, const Eigen::Vector3f &ep,
                          const Colour &color, float size = DefaultLineSize);

        ~Line() override;

        void Draw() const override;

        Line() = default;

    public:

        template<class Archive>
        void serialize(Archive &archive) {
            Entity::serialize(archive);
            archive(CEREAL_NVP(sp), CEREAL_NVP(ep), CEREAL_NVP(color), CEREAL_NVP(size));
        }
    };
}

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Line, "Line")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Line)

#endif //TINY_VIEWER_LINE_H
