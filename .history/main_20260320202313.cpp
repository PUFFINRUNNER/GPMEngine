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

#include <iostream>

#include "Run.hpp"
#include "G.hpp"
#include "M.hpp"
#include "P.hpp"
#include "GSoftwareRenderer.hpp"
#include "GAllegroTexture.hpp"
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

namespace {
    constexpr float PI = 3.14159265358979323846f;
    struct BodySnapshot {
        bool valid = false;
        M::Point3D position;
        M::Quaternion rotation;
    };

    struct PhysicsSnapshot {
        BodySnapshot base;
        BodySnapshot cube;
    };
    class DemoApp final : public Run::App {
    public:
        G::SoftwareRenderer3D m_renderer;
        std::shared_ptr<G::AllegroTexture2D> m_texture;
        DemoApp() {
            m_cubeMesh = G::makeCube(1.5f, 1.5f, 1.5f);
            m_platformMesh = G::makeCube(50.0f, 0.5f, 50.0f);

            m_camera.position = M::Point3D(0.0f, 2.0f, 10.0f);
            m_camera.rotation = M::Quaternion::identity();
            m_camera.fovYRadians = PI / 3.0f;
            m_camera.nearPlane = 0.1f;
            m_camera.farPlane = 100.0f;
        }

        void onInit() override {
            m_font = al_create_builtin_font();
            m_texture = G::AllegroTexture2D::loadFromFile("assets/texture.png");
            startPhysicsThread();
        }

        void onShutdown() override {
            stopPhysicsThread();
            if (m_font) {
                al_destroy_font(m_font);
                m_font = nullptr;
            }
        }

        void onFixedUpdate(double /*dt*/) override {}

        void onUpdate(double dt) override {
            updateCamera(static_cast<float>(dt));
            m_squareAngle += static_cast<float>(dt) * 1.6f;
        }

        void onRender(const G::Viewport& viewport, double /*alpha*/) override {
            const G::Color white(1.0f, 1.0f, 1.0f, 1.0f);

            G::RenderSettings3D rs;
            rs.backfaceCulling = true;
            rs.depthTest = true;
            rs.perspectiveCorrectUV = true;
            rs.wireframeOnly = false;

            m_renderer.beginFrame(viewport);
            m_renderer.clear(G::Color(0.08f, 0.09f, 0.11f, 1.0f));

            if (m_baseBody) {
                drawMeshFromBody(m_platformMesh, *m_baseBody, viewport, white, 2.0f);
            }

            if (m_dropBody) {
                G::Object3D object;
                object.mesh = std::make_shared<G::Mesh>(m_cubeMesh);
                object.transform = m_dropBody->transform();
                object.material.albedo = G::Color::white();
                object.material.diffuseMap = m_texture;
                object.material.opacity = 1.0f;
                object.material.unlit = true;

                m_renderer.drawObject(object, m_camera, viewport, rs);

                // Optional: cube wireframe on top of its textured faces, but still depth-tested
                // drawMeshFromBody(m_cubeMesh, *m_dropBody, viewport, white, 2.0f);
            }

            m_renderer.endFrame();

            drawOverlaySquare2D(viewport);
            drawHelp(viewport);
        }

    private:
        G::Mesh m_cubeMesh;
        G::Mesh m_platformMesh;
        G::Camera3D m_camera;
        ALLEGRO_FONT* m_font = nullptr;

        std::thread m_physicsThread;
        std::atomic<bool> m_physicsRunning {false};

        mutable std::mutex m_snapshotMutex;
        PhysicsSnapshot m_snapshot;

        double m_platformAmplitude = 2.0;
        double m_platformSpeed = 0.8;

        float m_squareAngle = 0.0f;

        void setupPhysics() {
            m_world.settings.restingLinearThreshold = 0.35f;
            m_world.settings.restingAngularThreshold = 0.9f;
            m_world.settings.settleAssistTorque = 18.0f;
            m_world.settings.settleSnapDot = 0.9965f;

            m_world.settings.solver.velocityIterations = 10;
            m_world.settings.solver.positionIterations = 2;
            m_world.settings.solver.baumgarte = 0.5f;
            m_world.settings.solver.penetrationSlop = 0.02f;
            m_baseBody = m_world.createBody();
            m_baseBody->type = P::BodyType::Kinematic;
            m_baseBody->position = M::Point3D(0.0f, -2.0f, 0.0f);
            m_baseBody->rotation = M::Quaternion::identity();
            auto baseCollider = std::make_shared<P::BoxCollider>(M::Vector3D(25.0f, 0.25f, 25.0f));
            P::Fixture* baseFixture = m_world.createFixture(m_baseBody, baseCollider);
            baseFixture->material.restitution = 0.1f;
            baseFixture->material.staticFriction = 1.0f;
            baseFixture->material.dynamicFriction = 0.7f;

            m_dropBody = m_world.createBody();
            m_dropBody->type = P::BodyType::Dynamic;
            m_dropBody->setMass(1.0f);
            m_dropBody->rotation = M::Quaternion::identity();
            auto dropCollider = std::make_shared<P::BoxCollider>(M::Vector3D(0.75f, 0.75f, 0.75f));
            P::Fixture* dropFixture = m_world.createFixture(m_dropBody, dropCollider);
            dropFixture->material.restitution = 0.05f;
            dropFixture->material.staticFriction = 1.0f;
            dropFixture->material.dynamicFriction = 0.7f;
        }

        void respawnDropCube() {
            if (!m_dropBody) {
                return;
            }
            m_dropBody->position = M::Point3D(0.0f, 2.5f, 0.0f);
            m_dropBody->rotation = M::Quaternion::identity();
            m_dropBody->linearVelocity = M::Vector3D(0.0f, 0.0f, 0.0f);
            m_dropBody->angularVelocity = M::Vector3D(0.0f, 0.0f, 0.0f);
            m_dropBody->forceAccum = M::Vector3D(0.0f, 0.0f, 0.0f);
            m_dropBody->torqueAccum = M::Vector3D(0.0f, 0.0f, 0.0f);
            m_dropBody->awake = true;
        }

        static bool keyDown(int keycode) {
            ALLEGRO_KEYBOARD_STATE state;
            al_get_keyboard_state(&state);
            return al_key_down(&state, keycode);
        }

        void updateCamera(float dt) {
            const float moveSpeed = 5.0f;
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

        void drawMeshFromBody(const G::Mesh& mesh,
                            const P::RigidBody& body,
                            const G::Viewport& viewport,
                            const G::Color& color,
                            float thickness) {
            const M::Transform3D t = body.transform();

            const G::Mesh worldMesh = mesh.transformed(t);
            for (const auto& tri : worldMesh.triangles) {
                const M::Point3D& a = worldMesh.vertices[static_cast<std::size_t>(tri.a)].position;
                const M::Point3D& b = worldMesh.vertices[static_cast<std::size_t>(tri.b)].position;
                const M::Point3D& c = worldMesh.vertices[static_cast<std::size_t>(tri.c)].position;

                drawSegment3D(a, b, viewport, color, thickness);
                drawSegment3D(b, c, viewport, color, thickness);
                drawSegment3D(c, a, viewport, color, thickness);
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
                        const G::Color& color,
                        float /*thickness*/) {
            m_renderer.drawLine3D(a, b, color, m_camera, viewport);
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
                         "Move camera: W A S D E Q | Physics: moving platform, drop resets every 10s");
        }
    };
}

int main() {
    Run::Config cfg;
    cfg.title = "Rotating Cube + Physics Test";
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
