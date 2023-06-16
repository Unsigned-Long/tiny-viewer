//
// Created by csl on 6/15/23.
//

#ifndef TINY_VIEWER_PATH_H
#define TINY_VIEWER_PATH_H

#include "entity.h"
#include "line.h"

namespace ns_viewer {
    struct Bezier {
    public:
        static std::vector<Eigen::Vector3f> Solve(const std::vector<Eigen::Vector3f> &controlPoints, std::size_t num);

    protected:
        static Eigen::Vector3f
        Solve(float t, const std::vector<Eigen::Vector3f> &controlPoints, std::size_t beg, std::size_t end);
    };

    /**
     * M = moveto, L = lineto, S = smooth curveto, Z = closepath
     * Uppercase means absolute position, lowercase means relative position
     */
    struct Path : public Entity {
    public:
        using Ptr = std::shared_ptr<Path>;

    protected:
        std::vector<Line> lines;

    public:
        explicit Path(const std::string &svgCode, float size = DefaultLineSize,
                      const Colour &color = GetUniqueColour());

        explicit Path(const std::string &svgCode, const Colour &color, float size = DefaultLineSize);

        static Ptr Create(const std::string &svgCode, float size = DefaultLineSize,
                          const Colour &color = GetUniqueColour());

        static Ptr Create(const std::string &svgCode, const Colour &color, float size = DefaultLineSize);

        ~Path() override = default;

        void Draw() const override;

        Path() = default;

    protected:
        static std::vector<Line> ParseSVGCode(const std::string &svgCode, float size, const Colour &color);

        static bool IsSVGPathControlCode(const std::string &str);

        static float CurveLength(const std::vector<Eigen::Vector3f> &pts);

    public:
        template<class Archive>
        void serialize(Archive &archive) {
            Entity::serialize(archive);
            archive(CEREAL_NVP(lines));
        }
    };
}

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Path, "Path")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Path)

#endif //TINY_VIEWER_PATH_H
