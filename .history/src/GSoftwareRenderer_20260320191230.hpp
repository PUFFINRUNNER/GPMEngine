#ifndef G_SOFTWARE_RENDERER_HPP
#define G_SOFTWARE_RENDERER_HPP

#include <vector>

#include <allegro5/allegro.h>

#include "G.hpp"

namespace G {
    class SoftwareRenderer3D : public IRenderer3D {
    public:
        SoftwareRenderer3D() = default;
        ~SoftwareRenderer3D() override = default;

        void beginFrame(const Viewport& viewport) override;
        void endFrame() override;

        void clear(const Color& color) override;

        void drawLine3D(const M::Point3D& a,
                        const M::Point3D& b,
                        const Color& color,
                        const Camera3D& camera,
                        const Viewport& viewport) override;

        void drawMesh(const Mesh& mesh,
                      const M::Transform3D& transform,
                      const Material3D& material,
                      const Camera3D& camera,
                      const Viewport& viewport,
                      const RenderSettings3D& settings) override;

        void drawObject(const Object3D& object,
                        const Camera3D& camera,
                        const Viewport& viewport,
                        const RenderSettings3D& settings) override;

        void render(const Scene3D& scene,
                    const Viewport& viewport,
                    const RenderSettings3D& settings) override;

        [[nodiscard]] RenderStats3D stats() const noexcept override;

    private:
        Viewport m_viewport;
        std::vector<float> m_depthBuffer;
        RenderStats3D m_stats {};

        void ensureDepthBuffer();
        void clearDepth();

        void rasterizeTriangle(const ScreenVertex& a,
                               const ScreenVertex& b,
                               const ScreenVertex& c,
                               const Material3D& material);

        [[nodiscard]] Color shadePixel(float u, float v, const Material3D& material) const noexcept;
    };

} // namespace G

#endif