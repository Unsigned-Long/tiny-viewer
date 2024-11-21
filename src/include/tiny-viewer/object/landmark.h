// Tiny-Viewer: Tiny But Powerful Graphic Entity And Object Visualization
// Copyright 2024, the School of Geodesy and Geomatics (SGG), Wuhan University, China
// https://github.com/Unsigned-Long/tiny-viewer.git
//
// Author: Shuolong Chen (shlchen@whu.edu.cn)
// GitHub: https://github.com/Unsigned-Long
//  ORCID: 0000-0002-5283-9057
//
// Purpose: See .h/.hpp file.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * The names of its contributors can not be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef TINY_VIEWER_LANDMARK_H
#define TINY_VIEWER_LANDMARK_H

#include "tiny-viewer/entity/entity.h"
#include "tiny-viewer/entity/point_cloud.hpp"
#include "chrono"

namespace ns_viewer {
struct Landmark : public Entity {
public:
    using Ptr = std::shared_ptr<Landmark>;

protected:
    std::array<Eigen::Vector3f, 7 * 2> verts;
    Colour color;
    float size{};

public:
    explicit Landmark(const Eigen::Vector3f &p,
                      float size = DefaultLandmarkSize,
                      const Colour &color = GetUniqueColour());

    Landmark(const Eigen::Vector3f &p, const Colour &color, float size = DefaultLandmarkSize);

    static Ptr Create(const Eigen::Vector3f &p,
                      float size = DefaultLandmarkSize,
                      const Colour &color = GetUniqueColour());

    static Ptr Create(const Eigen::Vector3f &p,
                      const Colour &color,
                      float size = DefaultLandmarkSize);

    ~Landmark() override = default;

    void Draw() const override;

    Landmark() = default;

public:
    template <class Archive>
    void serialize(Archive &archive) {
        Entity::serialize(archive);
        archive(CEREAL_NVP(verts), CEREAL_NVP(color), CEREAL_NVP(size));
    }
};

template <>
struct Cloud<Landmark> : public Entity {
public:
    using Ptr = std::shared_ptr<Cloud>;

protected:
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using PointCloudPtr = PointCloud::Ptr;

    using RGBPointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
    using RGBPointCloudPtr = RGBPointCloud::Ptr;

    using RGBAPointCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
    using RGBAPointCloudPtr = RGBAPointCloud::Ptr;

    std::vector<Landmark> landmarks;

public:
    explicit Cloud(const PointCloudPtr &cloud, float size = DefaultLandmarkSize)
        : Entity() {
        landmarks.resize(cloud->size());
        for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
            const auto &p = cloud->at(i);
            landmarks.at(i) = Landmark({ExpandPCLPointXYZ(p)}, size);
        }
    }

    explicit Cloud(const RGBPointCloudPtr &cloud, float size = DefaultLandmarkSize)
        : Entity() {
        landmarks.resize(cloud->size());
        for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
            const auto &p = cloud->at(i);
            landmarks.at(i) = Landmark({ExpandPCLPointXYZ(p)}, size, Colour(ExpandPCLColor(p)));
        }
    }

    explicit Cloud(const RGBAPointCloudPtr &cloud, float size = DefaultLandmarkSize)
        : Entity() {
        landmarks.resize(cloud->size());
        for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
            const auto &p = cloud->at(i);
            landmarks.at(i) = Landmark({ExpandPCLPointXYZ(p)}, size, Colour(ExpandPCLColor(p)));
        }
    }

    static Ptr Create(const PointCloudPtr &cloud, float size = DefaultLandmarkSize) {
        return std::make_shared<Cloud>(cloud, size);
    }

    static Cloud::Ptr Random(float bound, std::size_t count, const Eigen::Vector3f &center) {
        std::default_random_engine engine(
            std::chrono::steady_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<float> u(-bound, bound);
        PointCloudPtr cloud(new PointCloud);
        cloud->resize(count);
        for (int i = 0; i < static_cast<int>(count); ++i) {
            pcl::PointXYZ &p = cloud->at(i);
            // x, y, z
            p.x = u(engine) + center(0);
            p.y = u(engine) + center(1);
            p.z = u(engine) + center(2);
        }
        return Cloud::Create(cloud);
    }

    ~Cloud() override = default;

    void Draw() const override {
        for (const auto &lm : landmarks) {
            lm.Draw();
        }
    }

    Cloud() = default;

public:
    template <class Archive>
    void serialize(Archive &archive) {
        Entity::serialize(archive);
        archive(cereal::make_nvp("landmarks", landmarks));
    }
};
}  // namespace ns_viewer

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Landmark, "Landmark")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Landmark)

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Cloud<ns_viewer::Landmark>, "Cloud::Landmark")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Cloud<ns_viewer::Landmark>)
#endif  // TINY_VIEWER_LANDMARK_H
