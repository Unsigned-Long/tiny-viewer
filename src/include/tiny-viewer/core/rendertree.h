
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

#ifndef TINY_VIEWER_RENDER_TREE_H
#define TINY_VIEWER_RENDER_TREE_H

#include <Eigen/Geometry>

#include <pangolin/scene/tree.h>
#include <pangolin/geometry/glgeometry.h>

struct Renderable {
    virtual ~Renderable() {}

    Renderable()
        : show(true) {}

    virtual void Render(pangolin::GlSlProgram & /*prog*/,
                        const pangolin::GlTexture * /*matcap*/) const {}

    inline virtual Eigen::AlignedBox3f GetAABB() const { return Eigen::AlignedBox3f(); }

    bool show;
};

struct GlGeomRenderable : public Renderable {
    GlGeomRenderable(pangolin::GlGeometry &&glgeom, const Eigen::AlignedBox3f &aabb)
        : glgeom(std::move(glgeom)),
          aabb(aabb) {}

    void Render(pangolin::GlSlProgram &prog, const pangolin::GlTexture *matcap) const override {
        if (show) {
            pangolin::GlDraw(prog, glgeom, matcap);
        }
    }

    Eigen::AlignedBox3f GetAABB() const override { return aabb; }

    pangolin::GlGeometry glgeom;
    Eigen::AlignedBox3f aabb;
};

struct RenderableTransform {
    virtual ~RenderableTransform() {}

    virtual Eigen::Matrix4f GetT_pc() const = 0;
};

struct FixedTransform : public RenderableTransform {
    FixedTransform(Eigen::Matrix4f T_pc = Eigen::Matrix4f::Identity())
        : T_pc(T_pc) {}

    Eigen::Matrix4f GetT_pc() const override { return T_pc; }

    Eigen::Matrix4f T_pc;
};

struct SpinTransform : public RenderableTransform {
    SpinTransform(pangolin::AxisDirection dir)
        : dir(dir),
          start(std::chrono::steady_clock::now()) {}

    Eigen::Matrix4f GetT_pc() const override {
        if (dir != pangolin::AxisNone) {
            const double rad_per_sec = 0.5;
            const double rad =
                rad_per_sec * (std::chrono::steady_clock::now() - start).count() / 1E9;
            const Eigen::Map<const Eigen::Matrix<pangolin::GLprecision, 3, 1>> axis(
                pangolin::AxisDirectionVector[dir]);
            Eigen::AngleAxisf aa(rad, axis.cast<float>());
            Eigen::Matrix4f T_pc = Eigen::Matrix4f::Identity();
            T_pc.block<3, 3>(0, 0) = aa.toRotationMatrix();
            return T_pc;
        } else {
            return Eigen::Matrix4f::Identity();
        }
    }

    pangolin::AxisDirection dir;
    std::chrono::steady_clock::time_point start;
};

using RenderNode =
    pangolin::TreeNode<std::shared_ptr<Renderable>, std::shared_ptr<RenderableTransform>>;

static void render_tree(pangolin::GlSlProgram &prog,
                        RenderNode &node,
                        const pangolin::OpenGlMatrix &K,
                        const pangolin::OpenGlMatrix &T_camera_node,
                        pangolin::GlTexture *matcap) {
    if (node.item) {
        prog.SetUniform("KT_cw", K * T_camera_node);
        prog.SetUniform("T_cam_norm", T_camera_node);
        node.item->Render(prog, matcap);
    }
    for (auto &e : node.edges) {
        render_tree(prog, e.node, K,
                    T_camera_node * (pangolin::OpenGlMatrix)e.parent_child->GetT_pc(), matcap);
    }
}

#endif