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

#include "tiny-viewer/entity/utils.h"

namespace ns_viewer {

vtkSmartPointer<vtkLookupTable> GetColormapLUT(
    pcl::visualization::LookUpTableRepresentationProperties colormapType, double *minmax) {
    vtkSmartPointer<vtkLookupTable> table = vtkSmartPointer<vtkLookupTable>::New();
    table->SetTableRange(minmax[0], minmax[1]);

    switch (colormapType) {
        case IntensityMode::PCL_VISUALIZER_LUT_JET: {
            table->SetHueRange(0, 0.667);
            table->SetSaturationRange(1, 1);
            table->SetAlphaRange(1, 1);
            break;
        }
        case IntensityMode::PCL_VISUALIZER_LUT_JET_INVERSE: {
            table->SetHueRange(0.667, 0);
            table->SetSaturationRange(1, 1);
            table->SetAlphaRange(1, 1);
            break;
        }
        case IntensityMode::PCL_VISUALIZER_LUT_HSV: {
            table->SetHueRange(0, 1);
            table->SetSaturationRange(1, 1);
            table->SetAlphaRange(1, 1);
            break;
        }
        case IntensityMode::PCL_VISUALIZER_LUT_HSV_INVERSE: {
            table->SetHueRange(1, 0);
            table->SetSaturationRange(1, 1);
            table->SetAlphaRange(1, 1);
            break;
        }
        case IntensityMode::PCL_VISUALIZER_LUT_GREY: {
            table->SetValueRange(0, 1);
            table->SetHueRange(0, 0);
            table->SetSaturationRange(0, 0);
            table->SetAlphaRange(1, 1);
            break;
        }
        case IntensityMode::PCL_VISUALIZER_LUT_BLUE2RED: {
            table->SetSaturationRange(1, 1);
            table->SetAlphaRange(1, 1);
            table->SetNumberOfTableValues(256);

            double red[3] = {1.0, 0.0, 0.0};
            double white[3] = {1.0, 1.0, 1.0};
            double blue[3] = {0.0, 0.0, 1.0};

            for (std::size_t i = 0; i < 128; i++) {
                double weight = static_cast<double>(i) / 128.0;
                table->SetTableValue(static_cast<vtkIdType>(i),
                                     white[0] * weight + blue[0] * (1 - weight),
                                     white[1] * weight + blue[1] * (1 - weight),
                                     white[2] * weight + blue[2] * (1 - weight));
            }

            for (std::size_t i = 128; i < 256; i++) {
                double weight = (static_cast<double>(i) - 128.0) / 128.0;
                table->SetTableValue(static_cast<vtkIdType>(i),
                                     red[0] * weight + white[0] * (1 - weight),
                                     red[1] * weight + white[1] * (1 - weight),
                                     red[2] * weight + white[2] * (1 - weight));
            }
            break;
        }
        case IntensityMode::PCL_VISUALIZER_LUT_VIRIDIS: {
            table->SetSaturationRange(1, 1);
            table->SetAlphaRange(1, 1);
            table->SetNumberOfTableValues(static_cast<vtkIdType>(pcl::ViridisLUT::size()));
            for (std::size_t i = 0; i < pcl::ViridisLUT::size(); i++) {
                pcl::RGB c = pcl::ViridisLUT::at(i);
                table->SetTableValue(static_cast<vtkIdType>(i), static_cast<double>(c.r) / 255.0,
                                     static_cast<double>(c.g) / 255.0,
                                     static_cast<double>(c.b) / 255.0);
            }
            break;
        }
        default: {
            table->SetHueRange(0, 0.667);
            table->SetSaturationRange(1, 1);
            table->SetAlphaRange(1, 1);
            break;
        }
    }
    table->Build();
    return table;
}

std::pair<Eigen::Vector3f, Eigen::Vector3f> TangentBasis(const Eigen::Vector3f &v) {
    Eigen::Vector3f b, c;
    Eigen::Vector3f a = v.normalized();
    Eigen::Vector3f tmp(0, 0, 1);
    if (a == tmp) tmp << 1, 0, 0;
    b = (tmp - a * (a.transpose() * tmp)).normalized();
    c = a.cross(b);
    return {b, c};
}

std::optional<Eigen::Vector3f> LinePlaneIntersection(const Eigen::Vector3f &ls,
                                                     const Eigen::Vector3f &le,
                                                     const Eigen::Vector3f &norm,
                                                     float d) {
    float ds = norm.dot(ls) + d;
    float de = norm.dot(le) + d;
    if (ds * de > 0.0) {
        // small side
        return {};
    } else if (ds == 0.0) {
        // ls in plane
        return ls;
    } else if (de == 0.0) {
        // le in plane
        return le;
    } else {
        // intersection
        float nds = std::abs(ds), nde = std::abs(de);
        return (nds * le + nde * ls) / (nds + nde);
    }
}

std::vector<std::string> StringSplit(const std::string &str, char splitor, bool ignoreEmpty) {
    std::vector<std::string> vec;
    auto iter = str.cbegin();
    while (true) {
        auto pos = std::find(iter, str.cend(), splitor);
        auto elem = std::string(iter, pos);
        if (!(elem.empty() && ignoreEmpty)) {
            vec.push_back(elem);
        }
        if (pos == str.cend()) {
            break;
        }
        iter = ++pos;
    }
    return vec;
}
}  // namespace ns_viewer