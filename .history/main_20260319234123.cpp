#include <algorithm>
#include <cmath>
#include <vector>

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

    struct ProjectedVertex {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        bool valid = false;
    };

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
        }

        void onRender(const G::Viewport& viewport, double /*alpha*/) override {
            const float aspect = (viewport.height > 0)
                ? static_cast<float>(viewport.width) / static_cast<float>(viewport.height)
                : 1.0f;

            const M::Matrix4x4 proj = m_camera.projectionMatrix(aspect);
            const M::Matrix4x4 view = m_camera.viewMatrix();

            const M::Quaternion cubeRotation =
                M::Quaternion::fromAxisAngle(M::Vector3D(0.0f, 1.0f, 0.0f), m_cubeAngle) *
                M::Quaternion::fromAxisAngle(M::Vector3D(1.0f, 0.0f, 0.0f), m_cubeAngle * 0.45f);

            M::Transform3D cubeTransform;
            cubeTransform.position = M::Point3D(0.0f, 0.0f, 0.0f);
            cubeTransform.rotation = cubeRotation;
            cubeTransform.scale = M::Vector3D(1.0f, 1.0f, 1.0f);

            const M::Matrix4x4 model = cubeTransform.matrix();
            const M::Matrix4x4 mvp = proj * view * model;

            std::vector<ProjectedVertex> projected(m_cube.vertices.size());
            for (std::size_t i = 0; i < m_cube.vertices.size(); ++i) {
                const auto& p = m_cube.vertices[i].position;
                const M::Vector4D clip = mvp * M::Vector4D(p.x, p.y, p.z, 1.0f);

                if (std::fabs(clip.w) < M::EPS || clip.w <= 0.0f) {
                    projected[i].valid = false;
                    continue;
                }

                const float invW = 1.0f / clip.w;
                const float ndcX = clip.x * invW;
                const float ndcY = clip.y * invW;
                const float ndcZ = clip.z * invW;

                projected[i].x = static_cast<float>(viewport.x) + (ndcX + 1.0f) * 0.5f * static_cast<float>(viewport.width);
                projected[i].y = static_cast<float>(viewport.y) + (1.0f - (ndcY + 1.0f) * 0.5f) * static_cast<float>(viewport.height);
                projected[i].z = ndcZ;
                projected[i].valid = true;
            }

            struct FaceDraw {
                int a = 0;
                int b = 0;
                int c = 0;
                float depth = 0.0f;
            };

            std::vector<FaceDraw> faces;
            faces.reserve(m_cube.triangles.size());

            for (const auto& tri : m_cube.triangles) {
                const ProjectedVertex& v0 = projected[static_cast<std::size_t>(tri.a)];
                const ProjectedVertex& v1 = projected[static_cast<std::size_t>(tri.b)];
                const ProjectedVertex& v2 = projected[static_cast<std::size_t>(tri.c)];
                if (!v0.valid || !v1.valid || !v2.valid) {
                    continue;
                }

                const float abx = v1.x - v0.x;
                const float aby = v1.y - v0.y;
                const float acx = v2.x - v0.x;
                const float acy = v2.y - v0.y;
                const float cross = abx * acy - aby * acx;
                if (cross >= 0.0f) {
                    continue;
                }

                faces.push_back(FaceDraw{
                    tri.a,
                    tri.b,
                    tri.c,
                    (v0.z + v1.z + v2.z) / 3.0f
                });
            }

            std::sort(faces.begin(), faces.end(), [](const FaceDraw& lhs, const FaceDraw& rhs) {
                return lhs.depth > rhs.depth;
            });

            const ALLEGRO_COLOR white = al_map_rgb(255, 255, 255);
            for (const auto& face : faces) {
                const ProjectedVertex& a = projected[static_cast<std::size_t>(face.a)];
                const ProjectedVertex& b = projected[static_cast<std::size_t>(face.b)];
                const ProjectedVertex& c = projected[static_cast<std::size_t>(face.c)];
                al_draw_triangle(a.x, a.y, b.x, b.y, c.x, c.y, white, 2.0f);
            }

            drawHelp(viewport);
        }

    private:
        G::Mesh m_cube;
        G::Camera3D m_camera;
        float m_cubeAngle = 0.0f;
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

        void drawHelp(const G::Viewport& viewport) {
            if (!m_font) {
                return;
            }
            const ALLEGRO_COLOR white = al_map_rgb(255, 255, 255);
            al_draw_text(m_font, white, 12.0f, static_cast<float>(viewport.height - 24), 0,
                         "Move camera: W A S D E Q");
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
