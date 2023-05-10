//
// Created by csl on 5/8/23.
//

#ifndef TINY_VIEWER_UTILS_H
#define TINY_VIEWER_UTILS_H

#include "pcl/visualization/pcl_visualizer.h"

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

    vtkSmartPointer <vtkLookupTable>
    GetColormapLUT(pcl::visualization::LookUpTableRepresentationProperties colormapType, double minmax[2]);

    std::pair<Eigen::Vector3f, Eigen::Vector3f> TangentBasis(const Eigen::Vector3f &v);
}

#endif //TINY_VIEWER_UTILS_H
