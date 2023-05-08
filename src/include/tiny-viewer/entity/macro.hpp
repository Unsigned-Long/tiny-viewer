//
// Created by csl on 5/8/23.
//

#ifndef TINY_VIEWER_MACRO_HPP
#define TINY_VIEWER_MACRO_HPP

#define DefaultLineSize (2.0f)
#define DefaultCoordSize (1.0f)
#define DefaultPointSize (5.0f)
#define InsertEntityPair(m, entity) m.insert({entity.GetId(), entity})
#define ExpandVec3(v) v(0), v(1), v(2)
#define ExpandColor(c) c.r, c.g, c.b, c.a

#endif //TINY_VIEWER_MACRO_HPP
