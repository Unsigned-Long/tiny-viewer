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

#ifndef TINY_VIEWER_POINT_CLOUD_H
#define TINY_VIEWER_POINT_CLOUD_H

#include "entity.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pangolin/gl/gl.h"
#include "chrono"
#include "random"

namespace ns_viewer {

template <typename PointType>
struct Cloud : public Entity {
public:
    using Ptr = std::shared_ptr<Cloud>;

protected:
    using PointCloud = pcl::PointCloud<PointType>;
    using PointCloudPtr = typename PointCloud::Ptr;

    PointCloudPtr cloud;

    Colour color;
    float size;

public:
    explicit Cloud(PointCloudPtr cloud,
                   float size = DefaultPointSize,
                   const Colour &colour = GetUniqueColour())
        : Entity(),
          cloud(std::move(cloud)),
          color(colour),
          size(size) {}

    static Ptr Create(const PointCloudPtr &cloud,
                      float size = DefaultPointSize,
                      const Colour &colour = GetUniqueColour()) {
        return std::make_shared<Cloud>(cloud, size, colour);
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
        glPointSize(size);
        glColor4f(ExpandColor(color));
        glBegin(GL_POINTS);
        for (const auto &p : cloud->points) {
            glVertex3f(ExpandPCLPointXYZ(p));
        }
        glEnd();
    }

    Cloud()
        : cloud(new PointCloud) {}

    [[nodiscard]] const PointCloudPtr &GetCloud() const { return cloud; }

public:
    template <class Archive>
    void serialize(Archive &archive) {
        Entity::serialize(archive);
        archive(cereal::make_nvp("data", *cloud), CEREAL_NVP(size), CEREAL_NVP(color));
    }
};

template <>
struct Cloud<pcl::PointXYZRGB> : public Entity {
public:
    using Ptr = std::shared_ptr<Cloud>;

protected:
    using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
    using PointCloudPtr = PointCloud::Ptr;

    PointCloudPtr cloud;

    float size{};

public:
    explicit Cloud(PointCloudPtr cloud, float size = DefaultPointSize)
        : Entity(),
          cloud(std::move(cloud)),
          size(size) {}

    static Ptr Create(const PointCloudPtr &cloud, float size = DefaultPointSize) {
        return std::make_shared<Cloud>(cloud, size);
    }

    static Cloud::Ptr Random(float bound, std::size_t count, const Eigen::Vector3f &center) {
        std::default_random_engine engine(
            std::chrono::steady_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<float> u(-bound, bound);
        PointCloudPtr cloud(new PointCloud);
        cloud->resize(count);
        for (int i = 0; i < static_cast<int>(count); ++i) {
            pcl::PointXYZRGB &p = cloud->at(i);
            // x, y, z
            p.x = u(engine) + center(0);
            p.y = u(engine) + center(1);
            p.z = u(engine) + center(2);
            // r, g, b, a
            auto color = GetUniqueColour();
            p.r = static_cast<std::uint8_t>(color.r * 255.0f);
            p.g = static_cast<std::uint8_t>(color.g * 255.0f);
            p.b = static_cast<std::uint8_t>(color.b * 255.0f);
        }
        return Cloud::Create(cloud);
    }

    ~Cloud() override = default;

    void Draw() const override {
        glPointSize(size);
        glBegin(GL_POINTS);
        for (const auto &p : cloud->points) {
            glColor4f(ExpandPCLColor(p));
            glVertex3f(ExpandPCLPointXYZ(p));
        }
        glEnd();
    }

    Cloud()
        : cloud(new PointCloud) {}

    [[nodiscard]] const PointCloudPtr &GetCloud() const { return cloud; }

public:
    template <class Archive>
    void serialize(Archive &archive) {
        Entity::serialize(archive);
        archive(cereal::make_nvp("data", *cloud), CEREAL_NVP(size));
    }
};

template <>
struct Cloud<pcl::PointXYZRGBA> : public Entity {
public:
    using Ptr = std::shared_ptr<Cloud>;

protected:
    using PointCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
    using PointCloudPtr = PointCloud::Ptr;

    PointCloudPtr cloud;

    float size{};

public:
    explicit Cloud(PointCloudPtr cloud, float size = DefaultPointSize)
        : Entity(),
          cloud(std::move(cloud)),
          size(size) {}

    static Ptr Create(const PointCloudPtr &cloud, float size = DefaultPointSize) {
        return std::make_shared<Cloud>(cloud, size);
    }

    static Cloud::Ptr Random(float bound, std::size_t count, const Eigen::Vector3f &center) {
        std::default_random_engine engine(
            std::chrono::steady_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<float> u(-bound, bound);
        std::uniform_real_distribution<float> ua(0.0f, 255.0f);
        PointCloudPtr cloud(new PointCloud);
        cloud->resize(count);
        for (int i = 0; i < static_cast<int>(count); ++i) {
            pcl::PointXYZRGBA &p = cloud->at(i);
            // x, y, z
            p.x = u(engine) + center(0);
            p.y = u(engine) + center(1);
            p.z = u(engine) + center(2);
            // r, g, b, a
            auto color = GetUniqueColour();
            p.r = static_cast<std::uint8_t>(color.r * 255.0f);
            p.g = static_cast<std::uint8_t>(color.g * 255.0f);
            p.b = static_cast<std::uint8_t>(color.b * 255.0f);
            p.a = static_cast<std::uint8_t>(ua(engine));
        }
        return Cloud::Create(cloud);
    }

    ~Cloud() override = default;

    void Draw() const override {
        glPointSize(size);
        glBegin(GL_POINTS);
        for (const auto &p : cloud->points) {
            glColor4f(ExpandPCLColor(p));
            glVertex3f(ExpandPCLPointXYZ(p));
        }
        glEnd();
    }

    Cloud()
        : cloud(new PointCloud) {}

    [[nodiscard]] const PointCloudPtr &GetCloud() const { return cloud; }

public:
    template <class Archive>
    void serialize(Archive &archive) {
        Entity::serialize(archive);
        archive(cereal::make_nvp("data", *cloud), CEREAL_NVP(size));
    }
};

template <>
struct Cloud<pcl::PointXYZI> : public Entity {
public:
    using Ptr = std::shared_ptr<Cloud>;

protected:
    using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
    using ColorPointCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
    using PointCloudPtr = PointCloud::Ptr;
    using ColorPointCloudPtr = ColorPointCloud::Ptr;

    ColorPointCloudPtr cloud;

    float size{};

public:
    explicit Cloud(const PointCloudPtr &inputCloud,
                   float size = DefaultPointSize,
                   IntensityMode mode = IntensityMode::PCL_VISUALIZER_LUT_HSV)
        : Entity(),
          cloud(new ColorPointCloud),
          size(size) {
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> colorHandler(
            inputCloud, "intensity");
        auto colors = colorHandler.getColor();
        double minmax[2];
        colors->GetRange(minmax);
        vtkSmartPointer<vtkLookupTable> table = GetColormapLUT(mode, minmax);
        double rgb[3];
        cloud->resize(inputCloud->size());
        for (int i = 0; i < static_cast<int>(inputCloud->size()); ++i) {
            auto &ip = inputCloud->at(i);
            auto &op = cloud->at(i);
            op.x = ip.x;
            op.y = ip.y;
            op.z = ip.z;
            table->GetColor(ip.intensity, rgb);
            op.r = static_cast<std::uint8_t>(rgb[0] * 255.0f);
            op.g = static_cast<std::uint8_t>(rgb[1] * 255.0f);
            op.b = static_cast<std::uint8_t>(rgb[2] * 255.0f);
            op.a = static_cast<std::uint8_t>(255.0f);
        }
    }

    static Ptr Create(const PointCloudPtr &cloud, float size = DefaultPointSize) {
        return std::make_shared<Cloud>(cloud, size);
    }

    static Cloud::Ptr Random(float bound, std::size_t count, const Eigen::Vector3f &center) {
        std::default_random_engine engine(
            std::chrono::steady_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<float> u(-bound, bound);
        std::uniform_real_distribution<float> ui(0.0, 1.0);
        PointCloudPtr cloud(new PointCloud);
        cloud->resize(count);
        for (int i = 0; i < static_cast<int>(count); ++i) {
            pcl::PointXYZI &p = cloud->at(i);
            // x, y, z
            p.x = u(engine) + center(0);
            p.y = u(engine) + center(1);
            p.z = u(engine) + center(2);
            // intensity
            p.intensity = ui(engine);
        }
        return Cloud::Create(cloud);
    }

    ~Cloud() override = default;

    void Draw() const override {
        glPointSize(size);
        glBegin(GL_POINTS);
        for (const auto &p : cloud->points) {
            glColor4f(ExpandPCLColor(p));
            glVertex3f(ExpandPCLPointXYZ(p));
        }
        glEnd();
    }

    Cloud()
        : cloud(new ColorPointCloud) {}

    [[nodiscard]] const ColorPointCloudPtr &GetCloud() const { return cloud; }

public:
    template <class Archive>
    void serialize(Archive &archive) {
        Entity::serialize(archive);
        archive(cereal::make_nvp("data", *cloud), CEREAL_NVP(size));
    }
};
}  // namespace ns_viewer
CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Cloud<pcl::PointXYZ>, "Cloud::PointXYZ")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Cloud<pcl::PointXYZ>)

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Cloud<pcl::PointXYZI>, "Cloud::PointXYZI")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Cloud<pcl::PointXYZI>)

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Cloud<pcl::PointXYZRGBA>, "Cloud::PointXYZRGBA")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Cloud<pcl::PointXYZRGBA>)

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::Cloud<pcl::PointXYZRGB>, "Cloud::PointXYZRGB")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::Cloud<pcl::PointXYZRGB>)
#endif  // TINY_VIEWER_POINT_CLOUD_H
