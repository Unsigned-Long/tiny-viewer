//
// Created by csl on 5/10/23.
//

#ifndef TINY_VIEWER_ARROW_H
#define TINY_VIEWER_ARROW_H

#include "entity.h"

namespace ns_viewer {
    struct Arrow : public Entity {
    public:
        using Ptr = std::shared_ptr<Arrow>;

    protected:
        Eigen::Vector3f sp;
        Eigen::Vector3f ep;
        Eigen::Vector3f mp;

        std::array<Eigen::Vector3f, 4> verts;

        float size{};
        Colour color;

    public:

        Arrow(Eigen::Vector3f sp, Eigen::Vector3f ep, float size = DefaultLineSize,
              const Colour &color = GetUniqueColour());

        Arrow(Eigen::Vector3f sp, Eigen::Vector3f ep, const Colour &color, float size = DefaultLineSize);

        static Ptr Create(const Eigen::Vector3f &sp, const Eigen::Vector3f &ep, float size = DefaultLineSize,
                          const Colour &color = GetUniqueColour());

        static Ptr Create(const Eigen::Vector3f &sp, const Eigen::Vector3f &ep,
                          const Colour &color, float size = DefaultLineSize);

        ~Arrow() override;

        void Draw() const override;

        Arrow() = default;

    public:

        template<class Archive>
        void serialize(Archive &archive) {
            Entity::serialize(archive);
            archive(
                    CEREAL_NVP(sp), CEREAL_NVP(ep), CEREAL_NVP(mp),
                    CEREAL_NVP(verts), CEREAL_NVP(color), CEREAL_NVP(size)
            );
        }
    };
}
CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Arrow, "Arrow")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Arrow)

#endif //TINY_VIEWER_ARROW_H
