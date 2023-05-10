//
// Created by csl on 5/8/23.
//

#ifndef TINY_VIEWER_UTILS_H
#define TINY_VIEWER_UTILS_H

#include "pcl/visualization/pcl_visualizer.h"
#include "cereal/cereal.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/array.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/types/polymorphic.hpp"
#include "fstream"
#include "pangolin/gl/colour.h"

namespace ns_viewer {

#define DefaultLineSize (2.0f)
#define DefaultCoordSize (1.0f)
#define DefaultCubeSize (0.5f)
#define DefaultPointSize (4.0f)
#define DefaultIMUSize (0.2f)
#define DefaultCameraSize (0.2f)
#define DefaultLiDARSize (0.2f)

#define ExpandVec3(v) v(0), v(1), v(2)
#define ExpandStdVec3(v) v.at(0), v.at(1), v.at(2)
#define ExpandAryVec3(v) v[0], v[1], v[2]
#define ExpandPCLPointXYZ(p) p.x, p.y, p.z
#define ExpandColor(c) c.r, c.g, c.b, c.a
#define ExpandPCLColor(p) p.r * 0.00392, p.g * 0.00392, p.b * 0.00392, p.a * 0.00392

    using IntensityMode = pcl::visualization::LookUpTableRepresentationProperties;
    using ColourWheel = pangolin::ColourWheel;
    using Colour = pangolin::Colour;

    vtkSmartPointer <vtkLookupTable>
    GetColormapLUT(pcl::visualization::LookUpTableRepresentationProperties colormapType, double minmax[2]);

    std::pair<Eigen::Vector3f, Eigen::Vector3f> TangentBasis(const Eigen::Vector3f &v);
}

namespace Eigen {
    template<class Archive, typename ScaleType, int Rows, int Cols>
    void serialize(Archive &archive, Eigen::Matrix<ScaleType, Rows, Cols> &m) {
        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                archive(m(i, j));
            }
        }
    }

    template<class Archive, typename ScaleType>
    void serialize(Archive &archive, Eigen::Quaternion<ScaleType> &q) {
        archive(q.coeffs());
    }
}

namespace pangolin {
    template<class Archive>
    void serialize(Archive &ar, ns_viewer::Colour &color) {
        ar(
                cereal::make_nvp("red", color.red),
                cereal::make_nvp("green", color.green),
                cereal::make_nvp("blue", color.blue),
                cereal::make_nvp("alpha", color.alpha)
        );
    }
}

namespace pcl {
    template<class Archive>
    void serialize(Archive &ar, PCLHeader &h) {
        ar(
                cereal::make_nvp("stamp", h.stamp),
                cereal::make_nvp("frame_id", h.frame_id),
                cereal::make_nvp("seq", h.seq)
        );
    }

    template<class Archive>
    void serialize(Archive &ar, pcl::PointXYZI &p) {
        ar(
                cereal::make_nvp("x", p.x),
                cereal::make_nvp("y", p.y),
                cereal::make_nvp("z", p.z),
                cereal::make_nvp("intensity", p.intensity)
        );
    }

    template<class Archive>
    void serialize(Archive &ar, pcl::PointXYZRGBA &p) {
        ar(
                cereal::make_nvp("x", p.x),
                cereal::make_nvp("y", p.y),
                cereal::make_nvp("z", p.z),
                cereal::make_nvp("r", p.r),
                cereal::make_nvp("g", p.g),
                cereal::make_nvp("b", p.b),
                cereal::make_nvp("a", p.a)
        );
    }

    template<class Archive>
    void serialize(Archive &ar, pcl::PointXYZ &p) {
        ar(
                cereal::make_nvp("x", p.x),
                cereal::make_nvp("y", p.y),
                cereal::make_nvp("z", p.z)
        );
    }

    template<class Archive, typename PointType>
    void serialize(Archive &ar, pcl::PointCloud<PointType> &cloud) {
        ar(
                cereal::make_nvp("header", cloud.header),
                cereal::make_nvp("points", cloud.points),
                cereal::make_nvp("width", cloud.width),
                cereal::make_nvp("height", cloud.height),
                cereal::make_nvp("is_dense", cloud.is_dense),
                cereal::make_nvp("sensor_orientation_", cloud.sensor_orientation_),
                cereal::make_nvp("sensor_origin_", cloud.sensor_origin_)
        );
    }
}

#endif //TINY_VIEWER_UTILS_H
