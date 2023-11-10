//
// Created by csl on 6/15/23.
//

#include "tiny-viewer/entity/path.h"

namespace ns_viewer {

    std::vector<Eigen::Vector3f> Bezier::Solve(const std::vector<Eigen::Vector3f> &controlPoints, std::size_t num) {
        float t = 0.0f, delta = 1.0f / static_cast<float>(num - 1);
        std::vector<Eigen::Vector3f> result(num);

        for (int i = 0; i < static_cast<int>(num); ++i) {
            result[i] = Solve(t, controlPoints, 0, controlPoints.size());
            t += delta;
        }
        return result;
    }

    Eigen::Vector3f
    Bezier::Solve(float t, const std::vector<Eigen::Vector3f> &controlPoints, std::size_t beg, std::size_t end) {
        if (end - beg == 1) {
            return controlPoints[beg];
        } else {
            auto p1 = Solve(t, controlPoints, beg, end - 1);
            p1(0) *= 1 - t;
            p1(1) *= 1 - t;
            p1(2) *= 1 - t;
            auto p2 = Solve(t, controlPoints, beg + 1, end);
            p2(0) *= t;
            p2(1) *= t;
            p2(2) *= t;
            return {p1(0) + p2(0), p1(1) + p2(1), p1(2) + p2(2)};
        }
    }

    Path::Path(const std::string &svgCode, float size, const Colour &color)
            : lines(ParseSVGCode(svgCode, size, color)) {}

    Path::Path(const std::string &svgCode, const Colour &color, float size)
            : Path(svgCode, size, color) {}

    void Path::Draw() const {
        for (const auto &line: lines) { line.Draw(); }
    }

    Path::Ptr Path::Create(const std::string &svgCode, float size, const Colour &color) {
        return std::make_shared<Path>(svgCode, size, color);
    }

    Path::Ptr Path::Create(const std::string &svgCode, const Colour &color, float size) {
        return std::make_shared<Path>(svgCode, color, size);
    }

    std::vector<Line> Path::ParseSVGCode(const std::string &svgCode, float size, const Colour &color) {
        auto codes = StringSplit(svgCode, ' ', true);
        std::vector<std::pair<char, std::vector<float>>> data;
        for (int i = 0; i < static_cast<int>(codes.size());) {
            const auto &code = codes.at(i);
            if (IsSVGPathControlCode(code)) {
                // control code
                data.push_back({code.front(), {}});
                int j = i + 1;
                for (; j < static_cast<int>(codes.size()); ++j) {
                    auto nextCode = codes.at(j);
                    if (IsSVGPathControlCode(nextCode)) { break; }
                    data.back().second.push_back(std::stof(nextCode));
                }
                i = j;
            }
        }
        if (data.front().first != 'M' && data.front().first != 'm') { return {}; }

        std::vector<Line> lines;
        Eigen::Vector3f firPoint = Eigen::Vector3f::Zero(), lastPoint = Eigen::Vector3f::Zero();
        for (const auto &[code, vec]: data) {
            switch (code) {
                case 'M': {
                    Eigen::Vector3f curPoint = {vec.at(0), vec.at(1), vec.at(2)};
                    lastPoint = curPoint;
                    firPoint = curPoint;
                    break;
                }
                case 'm': {
                    Eigen::Vector3f curPoint = lastPoint + Eigen::Vector3f{vec.at(0), vec.at(1), vec.at(2)};
                    lastPoint = curPoint;
                    firPoint = curPoint;
                    break;
                }
                case 'L': {
                    Eigen::Vector3f curPoint = {vec.at(0), vec.at(1), vec.at(2)};
                    lines.emplace_back(lastPoint, curPoint, size, color);
                    lastPoint = curPoint;
                    break;
                }
                case 'l': {
                    Eigen::Vector3f curPoint = lastPoint + Eigen::Vector3f{vec.at(0), vec.at(1), vec.at(2)};
                    lines.emplace_back(lastPoint, curPoint, size, color);
                    lastPoint = curPoint;
                    break;
                }
                case 'S': {
                    std::vector<Eigen::Vector3f> cps(1 + vec.size() / 3);
                    cps.front() = lastPoint;
                    for (int i = 0; i < static_cast<int>(vec.size()); ++i) { cps.at(1 + i / 3)(i % 3) = vec.at(i); }
                    auto pts = Bezier::Solve(cps, 2 + static_cast<std::size_t>(CurveLength(cps) / 0.25f));
                    for (int i = 0; i < static_cast<int>(pts.size()) - 1; ++i) {
                        lines.emplace_back(pts.at(i), pts.at(i + 1), size, color);
                    }
                    lastPoint = cps.back();
                    break;
                }
                case 's': {
                    std::vector<Eigen::Vector3f> cps(1 + vec.size() / 3);
                    cps.front() = lastPoint;
                    for (int i = 0; i < static_cast<int>(vec.size()); ++i) { cps.at(1 + i / 3)(i % 3) = vec.at(i); }
                    for (int i = 1; i < static_cast<int>(cps.size()); ++i) { cps.at(i) += cps.at(i - 1); }

                    auto pts = Bezier::Solve(cps, 2 + static_cast<std::size_t>(CurveLength(cps) / 0.25f));
                    for (int i = 0; i < static_cast<int>(pts.size()) - 1; ++i) {
                        lines.emplace_back(pts.at(i), pts.at(i + 1), size, color);
                    }
                    lastPoint = cps.back();
                    break;
                }
                case 'Z':
                case 'z': {
                    lines.emplace_back(lastPoint, firPoint, size, color);
                    break;
                }
                default:
                    break;
            }
        }

        return lines;
    }

    bool Path::IsSVGPathControlCode(const std::string &str) {
        if (str.size() != 1) { return false; }
        auto code = str.front();
        if (code == 'M' || code == 'm' ||
            code == 'L' || code == 'l' ||
            code == 'Z' || code == 'z' ||
            code == 'S' || code == 's') {
            return true;
        } else {
            return false;
        }
    }

    float Path::CurveLength(const std::vector<Eigen::Vector3f> &pts) {
        float len = 0.0f;
        for (int i = 0; i < static_cast<int>(pts.size()) - 1; ++i) {
            const Eigen::Vector3f &p1 = pts.at(i);
            const Eigen::Vector3f &p2 = pts.at(i + 1);
            len += (p1 - p2).norm();
        }
        return len;
    }
}