//
// Created by csl on 5/8/23.
//

#ifndef TINY_VIEWER_MACRO_HPP
#define TINY_VIEWER_MACRO_HPP

#define DefaultLineSize (2.0f)
#define DefaultCoordSize (1.0f)
#define DefaultCubeSize (0.5f)
#define DefaultPointSize (5.0f)
#define DefaultIMUSize (0.2f)
#define DefaultCameraSize (0.2f)
#define DefaultLiDARSize (0.2f)

#define InsertEntityPair(m, entity) m.insert({entity.GetId(), entity})
#define ExpandVec3(v) v(0), v(1), v(2)
#define ExpandPCLPointXYZ(p) p.x, p.y, p.z
#define ExpandColor(c) c.r, c.g, c.b, c.a
#define ExpandPCLColor(p) p.r * 0.00392, p.g * 0.00392, p.b * 0.00392, p.a * 0.00392

#endif //TINY_VIEWER_MACRO_HPP
