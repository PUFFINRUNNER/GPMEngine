#include "GSoftwareRenderer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <allegro5/allegro.h>
#include <allegro5/color.h>

namespace {
    [[nodiscard]] float edgeFunction(float ax, float ay, float bx, float by, float px, float py) noexcept {
        return (px - ax) * (by - ay) - (py - ay) * (bx - ax);
    }

    [[nodiscard]] ALLEGRO_COLOR toAllegroColor(const G::Color& c) noexcept {
        return al_map_rgba_f(c.r, c.g, c.b, c.a);
    }

    [[nodiscard]] G::Color multiplyColor(const G::Color& a, const G::Color& b) noexcept {
        return G::Color(a.r * b.r, a.g * b.g, a.b * b.b, a.a * b.a);
    }
}

namespace G {

    void SoftwareRenderer3D::beginFrame(const Viewport& viewport) {
        m_viewport = viewport;
        ensureDepthBuffer();
        clearDepth();

        m_stats = RenderStats3D{};
    }

    void SoftwareRenderer3D::endFrame() {
    }

    void SoftwareRenderer3D::clear(const Color& color) {
        al_clear_to_color(toAllegroColor(color));
        clearDepth();
    }

    void SoftwareRenderer3D::drawLine3D(const M::Point3D& a,
                                        const M::Point3D& b,
                                        const Color& color,
                                        const Camera3D& camera,
                                        const Viewport& viewport) {
        ScreenPoint3D pa;
        ScreenPoint3D pb;
        if (projectLineToScreen(a, b, camera, viewport, pa, pb)) {
            al_draw_line(pa.x, pa.y, pb.x, pb.y, toAllegroColor(color), 1.0f);
        }
    }

    void SoftwareRenderer3D::drawMesh(const Mesh& mesh,
                                      const M::Transform3D& transform,
                                      const Material3D& material,
                                      const Camera3D& camera,
                                      const Viewport& viewport,
                                      const RenderSettings3D& settings) {
        const float aspect = (viewport.height > 0)
            ? static_cast<float>(viewport.width) / static_cast<float>(viewport.height)
            : 1.0f;

        const M::Matrix4x4 model = transform.matrix();
        const M::Matrix4x4 viewProjection = camera.viewProjectionMatrix(aspect);

        for (const auto& tri : mesh.triangles) {
            const Vertex& va = mesh.vertices[static_cast<std::size_t>(tri.a)];
            const Vertex& vb = mesh.vertices[static_cast<std::size_t>(tri.b)];
            const Vertex& vc = mesh.vertices[static_cast<std::size_t>(tri.c)];

            const ClipVertex ca = transformToClip(va, model, viewProjection);
            const ClipVertex cb = transformToClip(vb, model, viewProjection);
            const ClipVertex cc = transformToClip(vc, model, viewProjection);

            // Minimal first version: skip triangles that cross behind the camera.
            if (ca.position.w <= 0.0f || cb.position.w <= 0.0f || cc.position.w <= 0.0f) {
                continue;
            }

            const ScreenVertex sa = perspectiveDivide(ca, viewport);
            const ScreenVertex sb = perspectiveDivide(cb, viewport);
            const ScreenVertex sc = perspectiveDivide(cc, viewport);

            if (settings.backfaceCulling && isBackFacing(sa, sb, sc)) {
                ++m_stats.trianglesCulled;
                continue;
            }

            rasterizeTriangle(sa, sb, sc, material);
            ++m_stats.trianglesSubmitted;
        }

        ++m_stats.drawCalls;
    }

    void SoftwareRenderer3D::drawObject(const Object3D& object,
                                        const Camera3D& camera,
                                        const Viewport& viewport,
                                        const RenderSettings3D& settings) {
        if (!object.visible || !object.mesh || !object.mesh->valid() || object.mesh->empty()) {
            return;
        }

        drawMesh(*object.mesh, object.transform, object.material, camera, viewport, settings);
        ++m_stats.objectsSubmitted;
    }

    void SoftwareRenderer3D::render(const Scene3D& scene,
                                    const Viewport& viewport,
                                    const RenderSettings3D& settings) {
        beginFrame(viewport);
        clear(scene.clearColor);

        for (const auto& object : scene.objects) {
            drawObject(object, scene.camera, viewport, settings);
        }

        endFrame();
    }

    RenderStats3D SoftwareRenderer3D::stats() const noexcept {
        return m_stats;
    }

    void SoftwareRenderer3D::ensureDepthBuffer() {
        const std::size_t needed = static_cast<std::size_t>(std::max(0, m_viewport.width) * std::max(0, m_viewport.height));
        if (m_depthBuffer.size() != needed) {
            m_depthBuffer.resize(needed, std::numeric_limits<float>::infinity());
        }
    }

    void SoftwareRenderer3D::clearDepth() {
        std::fill(m_depthBuffer.begin(), m_depthBuffer.end(), std::numeric_limits<float>::infinity());
    }

    Color SoftwareRenderer3D::shadePixel(float u, float v, const Material3D& material) const noexcept {
        Color base = material.albedo;

        if (material.diffuseMap && material.diffuseMap->valid()) {
            const Color texel = material.diffuseMap->sample(M::Vector2D(u, v));
            base = multiplyColor(base, texel);
        }

        base.a *= material.opacity;
        return base;
    }

    void SoftwareRenderer3D::rasterizeTriangle(const ScreenVertex& a,
                                               const ScreenVertex& b,
                                               const ScreenVertex& c,
                                               const Material3D& material) {
        const float area = edgeFunction(a.x, a.y, b.x, b.y, c.x, c.y);
        if (std::fabs(area) < M::EPS) {
            return;
        }

        const float minXf = std::floor(std::min({a.x, b.x, c.x}));
        const float maxXf = std::ceil(std::max({a.x, b.x, c.x}));
        const float minYf = std::floor(std::min({a.y, b.y, c.y}));
        const float maxYf = std::ceil(std::max({a.y, b.y, c.y}));

        const int minX = std::max(m_viewport.x, static_cast<int>(minXf));
        const int maxX = std::min(m_viewport.x + m_viewport.width - 1, static_cast<int>(maxXf));
        const int minY = std::max(m_viewport.y, static_cast<int>(minYf));
        const int maxY = std::min(m_viewport.y + m_viewport.height - 1, static_cast<int>(maxYf));

        const float u0w = a.uv.v * a.invW;
        const float v0w = a.uv.w * a.invW;
        const float u1w = b.uv.v * b.invW;
        const float v1w = b.uv.w * b.invW;
        const float u2w = c.uv.v * c.invW;
        const float v2w = c.uv.w * c.invW;

        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                const float px = static_cast<float>(x) + 0.5f;
                const float py = static_cast<float>(y) + 0.5f;

                const float w0 = edgeFunction(b.x, b.y, c.x, c.y, px, py);
                const float w1 = edgeFunction(c.x, c.y, a.x, a.y, px, py);
                const float w2 = edgeFunction(a.x, a.y, b.x, b.y, px, py);

                const bool inside =
                    (area > 0.0f)
                    ? (w0 >= 0.0f && w1 >= 0.0f && w2 >= 0.0f)
                    : (w0 <= 0.0f && w1 <= 0.0f && w2 <= 0.0f);

                if (!inside) {
                    continue;
                }

                const float alpha = w0 / area;
                const float beta  = w1 / area;
                const float gamma = w2 / area;

                const float depth = alpha * a.z + beta * b.z + gamma * c.z;
                const int localX = x - m_viewport.x;
                const int localY = y - m_viewport.y;
                const std::size_t idx = static_cast<std::size_t>(localY * m_viewport.width + localX);

                if (idx >= m_depthBuffer.size()) {
                    continue;
                }

                if (depth >= m_depthBuffer[idx]) {
                    continue;
                }

                const float invW = alpha * a.invW + beta * b.invW + gamma * c.invW;
                if (std::fabs(invW) < M::EPS) {
                    continue;
                }

                const float u = (alpha * u0w + beta * u1w + gamma * u2w) / invW;
                const float v = (alpha * v0w + beta * v1w + gamma * v2w) / invW;

                const Color color = shadePixel(u, v, material);

                m_depthBuffer[idx] = depth;
                al_put_pixel(static_cast<float>(x), static_cast<float>(y), toAllegroColor(color));
            }
        }
    }

} // namespace G