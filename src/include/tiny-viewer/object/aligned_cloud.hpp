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

#ifndef TINY_VIEWER_ALIGNED_CLOUD_HPP
#define TINY_VIEWER_ALIGNED_CLOUD_HPP

#include "tiny-viewer/entity/point_cloud.hpp"

namespace ns_viewer {
template <typename PointType>
struct AlignedCloud : public Entity {
public:
    using Ptr = std::shared_ptr<AlignedCloud>;

protected:
    using PointCloud = pcl::PointCloud<PointType>;
    using PointCloudPtr = typename PointCloud::Ptr;

    Cloud<pcl::PointXYZI> cloud;

public:
    explicit AlignedCloud(const PointCloudPtr &inputCloud,
                          const Eigen::Vector3f &dir,
                          float size = DefaultPointSize,
                          IntensityMode mode = IntensityMode::PCL_VISUALIZER_LUT_HSV)
        : Entity(),
          cloud(ColorizeCloudByDirection(inputCloud, dir), size, mode) {}

    static Ptr Create(const PointCloudPtr &inputCloud,
                      const Eigen::Vector3f &dir,
                      float size = DefaultPointSize,
                      IntensityMode mode = IntensityMode::PCL_VISUALIZER_LUT_HSV) {
        return std::make_shared<AlignedCloud>(inputCloud, dir, size, mode);
    }

    ~AlignedCloud() override = default;

    void Draw() const override { cloud.Draw(); }

    AlignedCloud() = default;

    static auto Random(float bound,
                       std::size_t count,
                       const Eigen::Vector3f &center,
                       const Eigen::Vector3f &dir) {
        std::default_random_engine engine(
            std::chrono::steady_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<float> u(-bound, bound);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->resize(count);
        for (int i = 0; i < static_cast<int>(count); ++i) {
            auto &p = cloud->at(i);
            // x, y, z
            p.x = u(engine) + center(0);
            p.y = u(engine) + center(1);
            p.z = u(engine) + center(2);
        }
        return AlignedCloud<pcl::PointXYZ>::Create(cloud, dir);
    }

    [[nodiscard]] const Cloud<pcl::PointXYZI> &GetCloud() const { return cloud; }

public:
    template <class Archive>
    void serialize(Archive &archive) {
        Entity::serialize(archive);
        archive(cereal::make_nvp("data", cloud));
    }

protected:
    static pcl::PointCloud<pcl::PointXYZI>::Ptr ColorizeCloudByDirection(
        const typename pcl::PointCloud<PointType>::Ptr &iCloud, const Eigen::Vector3f &dir) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr oCloud(new pcl::PointCloud<pcl::PointXYZI>);
        oCloud->resize(iCloud->size());
        for (int i = 0; i < static_cast<int>(iCloud->size()); ++i) {
            const auto &ip = iCloud->at(i);
            auto &op = oCloud->at(i);
            op.x = ip.x, op.y = ip.y, op.z = ip.z;
            op.intensity = -Eigen::Vector3f(ip.x, ip.y, ip.z).dot(dir);
        }
        return oCloud;
    }
};
}  // namespace ns_viewer

CEREAL_REGISTER_TYPE_WITH_NAME(ns_viewer::AlignedCloud<pcl::PointXYZ>, "AlignedCloud::PointXYZ")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_viewer::Entity, ns_viewer::AlignedCloud<pcl::PointXYZ>)

#endif  // TINY_VIEWER_ALIGNED_CLOUD_HPP
