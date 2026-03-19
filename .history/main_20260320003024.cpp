#include <cmath>

#include <allegro5/allegro.h>
#include <allegro5/allegro_font.h>
#include <allegro5/allegro_image.h>
#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_ttf.h>
#include <allegro5/color.h>
#include <allegro5/events.h>
#include <allegro5/keyboard.h>
#include <allegro5/mouse.h>
#include <allegro5/system.h>
#include <allegro5/timer.h>

#include "Run.hpp"
#include "G.hpp"
#include "M.hpp"

namespace {
    constexpr float PI = 3.14159265358979323846f;
    constexpr float TAU = 6.28318530717958647692f;

    class DemoApp final : public Run::App {
    public:
        DemoApp() {
            m_cube = G::makeCube(1.5f, 1.5f, 1.5f);
            m_camera.position = M::Point3D(0.0f, 0.0f, 6.0f);
            m_camera.rotation = M::Quaternion::identity();
            m_camera.fovYRadians = PI / 3.0f;
            m_camera.nearPlane = 0.1f;
            m_camera.farPlane = 100.0f;
        }

        void onInit() override {
            m_font = al_create_builtin_font();
        }

        void onShutdown() override {
            if (m_font) {
                al_destroy_font(m_font);
                m_font = nullptr;
            }
        }

        void onUpdate(double dt) override {
            updateCamera(static_cast<float>(dt));
            m_cubeAngle += static_cast<float>(dt) * 0.9f;
            m_squareAngle += static_cast<float>(dt) * 1.6f;
        }

        void onRender(const G::Viewport& viewport, double /*alpha*/) override {
            const ALLEGRO_COLOR white = al_map_rgb(255, 255, 255);

            const M::Quaternion cubeRotation =
                M::Quaternion::fromAxisAngle(M::Vector3D(0.0f, 1.0f, 0.0f), m_cubeAngle) *
                M::Quaternion::fromAxisAngle(M::Vector3D(1.0f, 0.0f, 0.0f), m_cubeAngle * 0.45f);

            M::Transform3D cubeTransform;
            cubeTransform.position = M::Point3D(0.0f, 0.0f, 0.0f);
            cubeTransform.rotation = cubeRotation;
            cubeTransform.scale = M::Vector3D(1.0f, 1.0f, 1.0f);

            const G::Mesh worldCube = m_cube.transformed(cubeTransform);

            for (const auto& tri : worldCube.triangles) {
                const M::Point3D& a = worldCube.vertices[static_cast<std::size_t>(tri.a)].position;
                const M::Point3D& b = worldCube.vertices[static_cast<std::size_t>(tri.b)].position;
                const M::Point3D& c = worldCube.vertices[static_cast<std::size_t>(tri.c)].position;

                drawSegment3D(a, b, viewport, white, 2.0f);
                drawSegment3D(b, c, viewport, white, 2.0f);
                drawSegment3D(c, a, viewport, white, 2.0f);
            }

            drawPoint3D(M::Point3D(0.0f, 0.0f, 0.0f), viewport, al_map_rgb(255, 255, 255), 4.0f);
            drawOverlaySquare2D(viewport);
            drawHelp(viewport);
        }

    private:
        G::Mesh m_cube;
        G::Camera3D m_camera;
        float m_cubeAngle = 0.0f;
        float m_squareAngle = 0.0f;
        ALLEGRO_FONT* m_font = nullptr;

        static bool keyDown(int keycode) {
            ALLEGRO_KEYBOARD_STATE state;
            al_get_keyboard_state(&state);
            return al_key_down(&state, keycode);
        }

        void updateCamera(float dt) {
            const float moveSpeed = 4.0f;
            M::Vector3D move(0.0f, 0.0f, 0.0f);

            const M::Vector3D forward = m_camera.forward();
            const M::Vector3D right = m_camera.right();
            const M::Vector3D up = m_camera.up();

            if (keyDown(ALLEGRO_KEY_W)) move += forward;
            if (keyDown(ALLEGRO_KEY_S)) move -= forward;
            if (keyDown(ALLEGRO_KEY_D)) move += right;
            if (keyDown(ALLEGRO_KEY_A)) move -= right;
            if (keyDown(ALLEGRO_KEY_E)) move += up;
            if (keyDown(ALLEGRO_KEY_Q)) move -= up;

            if (move.magnitudeSqr() > M::EPS) {
                move.normalize();
                m_camera.position += move * (moveSpeed * dt);
            }
        }

        void drawPoint3D(const M::Point3D& worldPoint,
                         const G::Viewport& viewport,
                         ALLEGRO_COLOR color,
                         float radius) const {
            G::ScreenPoint3D p;
            if (G::projectPointToScreen(worldPoint, m_camera, viewport, p)) {
                al_draw_filled_circle(p.x, p.y, radius, color);
            }
        }

        void drawSegment3D(const M::Point3D& a,
                           const M::Point3D& b,
                           const G::Viewport& viewport,
                           ALLEGRO_COLOR color,
                           float thickness) const {
            G::ScreenPoint3D pa;
            G::ScreenPoint3D pb;
            if (G::projectLineToScreen(a, b, m_camera, viewport, pa, pb)) {
                al_draw_line(pa.x, pa.y, pb.x, pb.y, color, thickness);
            }
        }

        void drawOverlaySquare2D(const G::Viewport& viewport) {
            const float margin = 110.0f;
            const float cx = static_cast<float>(viewport.width) - margin;
            const float cy = margin;
            const float half = 36.0f;

            struct Corner2D {
                float x;
                float y;
            };

            Corner2D corners[4] = {
                {-half, -half},
                { half, -half},
                { half,  half},
                {-half,  half}
            };

            const float c = std::cos(m_squareAngle);
            const float s = std::sin(m_squareAngle);

            float px[4];
            float py[4];
            for (int i = 0; i < 4; ++i) {
                const float rx = corners[i].x * c - corners[i].y * s;
                const float ry = corners[i].x * s + corners[i].y * c;
                px[i] = cx + rx;
                py[i] = cy + ry;
            }

            const ALLEGRO_COLOR white = al_map_rgb(255, 255, 255);
            al_draw_line(px[0], py[0], px[1], py[1], white, 2.0f);
            al_draw_line(px[1], py[1], px[2], py[2], white, 2.0f);
            al_draw_line(px[2], py[2], px[3], py[3], white, 2.0f);
            al_draw_line(px[3], py[3], px[0], py[0], white, 2.0f);
        }

        void drawHelp(const G::Viewport& viewport) {
            if (!m_font) {
                return;
            }
            const ALLEGRO_COLOR white = al_map_rgb(255, 255, 255);
            al_draw_text(m_font, white, 12.0f, static_cast<float>(viewport.height - 24), 0,
                         "Move camera: W A S D E Q | 2D square overlay");
        }
    };
}

int main() {
    Run::Config cfg;
    cfg.title = "Rotating Cube";
    cfg.startupResolution = Run::Resolution(1280, 720);
    cfg.targetFps = 144.0;
    cfg.fixedPhysicsHz = 60.0;
    cfg.vsync = true;
    cfg.allowFullscreenToggle = true;
    cfg.showFpsInTitle = true;
    cfg.clearColor = G::Color(0.08f, 0.09f, 0.11f, 1.0f);

    Run::Runtime runtime(cfg);
    DemoApp app;
    runtime.run(app);
    return 0;
}
