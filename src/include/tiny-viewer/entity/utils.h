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

#ifndef TINY_VIEWER_UTILS_H
#define TINY_VIEWER_UTILS_H

#include "pcl/visualization/pcl_visualizer.h"
#include "cereal/cereal.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/array.hpp"
#include "cereal/types/unordered_map.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/archives/xml.hpp"
#include "cereal/archives/binary.hpp"
#include "cereal/types/polymorphic.hpp"
#include "fstream"
#include "pangolin/gl/colour.h"
#include "pangolin/gl/opengl_render_state.h"

namespace ns_viewer {

#define DefaultLineSize (2.0f)
#define DefaultCoordSize (1.0f)
#define DefaultCubeSize (0.5f)
#define DefaultPointSize (4.0f)
#define DefaultIMUSize (0.2f)
#define DefaultCameraSize (0.2f)
#define DefaultRadarSize (0.2f)
#define DefaultLiDARSize (0.2f)
#define DefaultLandmarkSize (0.05f)

#define ExpandVec3(v) v(0), v(1), v(2)
#define ExpandStdVec3(v) v.at(0), v.at(1), v.at(2)
#define ExpandAryVec3(v) v[0], v[1], v[2]
#define ExpandPCLPointXYZ(p) p.x, p.y, p.z
#define ExpandColor(c) c.r, c.g, c.b, c.a
#define ExpandPCLColor(p) p.r * 0.00392, p.g * 0.00392, p.b * 0.00392, p.a * 0.00392

using IntensityMode = pcl::visualization::LookUpTableRepresentationProperties;
using ColourWheel = pangolin::ColourWheel;
using Colour = pangolin::Colour;

vtkSmartPointer<vtkLookupTable> GetColormapLUT(
    pcl::visualization::LookUpTableRepresentationProperties colormapType, double minmax[2]);

std::pair<Eigen::Vector3f, Eigen::Vector3f> TangentBasis(const Eigen::Vector3f &v);

std::optional<Eigen::Vector3f> LinePlaneIntersection(const Eigen::Vector3f &ls,
                                                     const Eigen::Vector3f &le,
                                                     const Eigen::Vector3f &norm,
                                                     float d);

std::vector<std::string> StringSplit(const std::string &str, char splitor, bool ignoreEmpty = true);
}  // namespace ns_viewer

namespace Eigen {
template <class Archive, typename ScaleType, int Rows, int Cols>
void serialize(Archive &archive, Eigen::Matrix<ScaleType, Rows, Cols> &m) {
    for (int i = 0; i < Rows; ++i) {
        for (int j = 0; j < Cols; ++j) {
            archive(cereal::make_nvp('r' + std::to_string(i) + 'c' + std::to_string(j), m(i, j)));
        }
    }
}

template <class Archive, typename ScaleType, int Cols>
void serialize(Archive &archive, Eigen::Matrix<ScaleType, Eigen::Dynamic, Cols> &m) {
    for (int i = 0; i < m.rows(); ++i) {
        for (int j = 0; j < Cols; ++j) {
            archive(cereal::make_nvp('r' + std::to_string(i) + 'c' + std::to_string(j), m(i, j)));
        }
    }
}

template <class Archive, typename ScaleType>
void serialize(Archive &archive, Eigen::Matrix<ScaleType, Eigen::Dynamic, Eigen::Dynamic> &m) {
    for (int i = 0; i < m.rows(); ++i) {
        for (int j = 0; j < m.cols(); ++j) {
            archive(cereal::make_nvp('r' + std::to_string(i) + 'c' + std::to_string(j), m(i, j)));
        }
    }
}

template <class Archive, typename ScaleType>
void serialize(Archive &archive, Eigen::Quaternion<ScaleType> &q) {
    archive(cereal::make_nvp("qx", q.coeffs()[0]), cereal::make_nvp("qy", q.coeffs()[1]),
            cereal::make_nvp("qz", q.coeffs()[2]), cereal::make_nvp("qw", q.coeffs()[3]));
}
}  // namespace Eigen

namespace pangolin {
template <class Archive>
void serialize(Archive &ar, ns_viewer::Colour &color) {
    ar(cereal::make_nvp("red", color.red), cereal::make_nvp("green", color.green),
       cereal::make_nvp("blue", color.blue), cereal::make_nvp("alpha", color.alpha));
}

template <class Archive>
void serialize(Archive &ar, OpenGlMatrix &m) {
    for (double &i : m.m) {
        ar(i);
    }
}

template <class Archive>
void serialize(Archive &ar, OpenGlRenderState &m) {
    ar(cereal::make_nvp("projection_mat", m.GetProjectionMatrix()),
       cereal::make_nvp("model_view_mat", m.GetModelViewMatrix()));
}
}  // namespace pangolin

namespace pcl {
template <class Archive>
void serialize(Archive &ar, PCLHeader &h) {
    ar(cereal::make_nvp("stamp", h.stamp), cereal::make_nvp("frame_id", h.frame_id),
       cereal::make_nvp("seq", h.seq));
}

template <class Archive>
void serialize(Archive &ar, pcl::PointXYZI &p) {
    ar(cereal::make_nvp("x", p.x), cereal::make_nvp("y", p.y), cereal::make_nvp("z", p.z),
       cereal::make_nvp("intensity", p.intensity));
}

template <class Archive>
void serialize(Archive &ar, pcl::PointXYZRGB &p) {
    ar(cereal::make_nvp("x", p.x), cereal::make_nvp("y", p.y), cereal::make_nvp("z", p.z),
       cereal::make_nvp("r", p.r), cereal::make_nvp("g", p.g), cereal::make_nvp("b", p.b));
}

template <class Archive>
void serialize(Archive &ar, pcl::PointXYZRGBA &p) {
    ar(cereal::make_nvp("x", p.x), cereal::make_nvp("y", p.y), cereal::make_nvp("z", p.z),
       cereal::make_nvp("r", p.r), cereal::make_nvp("g", p.g), cereal::make_nvp("b", p.b),
       cereal::make_nvp("a", p.a));
}

template <class Archive>
void serialize(Archive &ar, pcl::PointXYZ &p) {
    ar(cereal::make_nvp("x", p.x), cereal::make_nvp("y", p.y), cereal::make_nvp("z", p.z));
}

template <class Archive, typename PointType>
void serialize(Archive &ar, pcl::PointCloud<PointType> &cloud) {
    ar(cereal::make_nvp("header", cloud.header), cereal::make_nvp("points", cloud.points),
       cereal::make_nvp("width", cloud.width), cereal::make_nvp("height", cloud.height),
       cereal::make_nvp("is_dense", cloud.is_dense),
       cereal::make_nvp("sensor_orientation_", cloud.sensor_orientation_),
       cereal::make_nvp("sensor_origin_", cloud.sensor_origin_));
}
}  // namespace pcl

#endif  // TINY_VIEWER_UTILS_H
