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
#include "GModel.hpp"
#include "GObjLoader.hpp"

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
        BodySnapshot model;
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
            m_model = G::ObjLoader::load("assets/Cardboardbox.obj");
std::cout << "model parts: " << m_model.parts.size() << std::endl;
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
            const PhysicsSnapshot snapshot = readSnapshot();

            G::RenderSettings3D rs;
            rs.backfaceCulling = true;
            rs.depthTest = true;
            rs.perspectiveCorrectUV = true;
            rs.wireframeOnly = false;

            m_renderer.beginFrame(viewport);
            m_renderer.clear(G::Color(0.08f, 0.09f, 0.11f, 1.0f));

            if (snapshot.base.valid) {
                drawMeshFromSnapshot(m_platformMesh, snapshot.base, viewport, white, 2.0f);
            }

            if (snapshot.model.valid) {

                M::Transform3D t;
                t.position = snapshot.model.position;
                t.rotation = snapshot.model.rotation;
                t.scale = M::Vector3D(1.0f, 1.0f, 1.0f);

                for (const auto& part : m_model.parts) {
                    m_renderer.drawMesh(*part.mesh, t, part.material, m_camera, viewport, rs);
                }
                // for (const auto& part : m_model.parts) {
                //     const G::Mesh worldMesh = part.mesh->transformed(t);
                //     for (const auto& tri : worldMesh.triangles) {
                //         const M::Point3D& a = worldMesh.vertices[static_cast<std::size_t>(tri.a)].position;
                //         const M::Point3D& b = worldMesh.vertices[static_cast<std::size_t>(tri.b)].position;
                //         const M::Point3D& c = worldMesh.vertices[static_cast<std::size_t>(tri.c)].position;

                //         drawSegment3D(a, b, viewport, G::Color::white(), 1.0f);
                //         drawSegment3D(b, c, viewport, G::Color::white(), 1.0f);
                //         drawSegment3D(c, a, viewport, G::Color::white(), 1.0f);
                //     }
                // }
                
            }

            if (snapshot.cube.valid) {
                G::Object3D object;
                object.mesh = std::make_shared<G::Mesh>(m_cubeMesh);
                object.transform.position = snapshot.cube.position;
                object.transform.rotation = snapshot.cube.rotation;
                object.transform.scale = M::Vector3D(1.0f, 1.0f, 1.0f);
                object.material.albedo = G::Color::white();
                object.material.diffuseMap = m_texture;
                object.material.opacity = 1.0f;
                object.material.unlit = true;

                m_renderer.drawObject(object, m_camera, viewport, rs);
                drawMeshFromSnapshot(m_cubeMesh, snapshot.cube, viewport, white, 2.0f);
            }

            m_renderer.endFrame();

            drawOverlaySquare2D(viewport);
            drawHelp(viewport);
        }

    private:
        G::Mesh m_cubeMesh;
        G::Mesh m_platformMesh;
        G::Camera3D m_camera;
        G::Model m_model;
        ALLEGRO_FONT* m_font = nullptr;

        std::thread m_physicsThread;
        std::atomic<bool> m_physicsRunning {false};

        mutable std::mutex m_snapshotMutex;
        PhysicsSnapshot m_snapshot;

        double m_platformAmplitude = 2.0;
        double m_platformSpeed = 0.8;

        float m_squareAngle = 0.0f;

        std::shared_ptr<P::TriangleMeshCollider> makeTriangleMeshColliderFromMesh(const G::Mesh& mesh) {
            auto collider = std::make_shared<P::TriangleMeshCollider>();

            collider->vertices.reserve(mesh.vertices.size());
            for (const auto& v : mesh.vertices) {
                collider->vertices.push_back(v.position);
            }

            collider->faces.reserve(mesh.triangles.size());
            for (const auto& t : mesh.triangles) {
                collider->faces.push_back(P::TriangleMeshCollider::Face{t.a, t.b, t.c});
            }

            return collider;
        }

        std::shared_ptr<P::TriangleMeshCollider> makeTriangleMeshColliderFromModel(const G::Model& model) {
            auto collider = std::make_shared<P::TriangleMeshCollider>();

            int vertexOffset = 0;
            for (const auto& part : model.parts) {
                if (!part.mesh) {
                    continue;
                }

                for (const auto& v : part.mesh->vertices) {
                    collider->vertices.push_back(v.position);
                }

                for (const auto& t : part.mesh->triangles) {
                    collider->faces.push_back(P::TriangleMeshCollider::Face{
                        t.a + vertexOffset,
                        t.b + vertexOffset,
                        t.c + vertexOffset
                    });
                }

                vertexOffset += static_cast<int>(part.mesh->vertices.size());
            }

            return collider;
        }

        void setupPhysicsWorld(P::World& world,
                            P::RigidBody*& baseBody,
                            P::RigidBody*& dropBody,
                            P::RigidBody*& modelBody) {
            baseBody = world.createBody();
            baseBody->type = P::BodyType::Kinematic;
            baseBody->position = M::Point3D(0.0f, -2.0f, 0.0f);
            baseBody->rotation = M::Quaternion::identity();

            auto baseCollider = std::make_shared<P::BoxCollider>(M::Vector3D(25.0f, 0.25f, 25.0f));
            P::Fixture* baseFixture = world.createFixture(baseBody, baseCollider);
            baseFixture->material.restitution = 0.1f;
            baseFixture->material.staticFriction = 1.0f;
            baseFixture->material.dynamicFriction = 0.7f;

            // Imported model as a static triangle-mesh collider
            modelBody = world.createBody();
            modelBody->type = P::BodyType::Static;
            modelBody->position = M::Point3D(0.0f, -1.0f, -4.0f);
            modelBody->rotation = M::Quaternion::identity();

            auto modelCollider = makeTriangleMeshColliderFromModel(m_model);
            P::Fixture* modelFixture = world.createFixture(modelBody, modelCollider);
            modelFixture->material.restitution = 0.05f;
            modelFixture->material.staticFriction = 0.9f;
            modelFixture->material.dynamicFriction = 0.6f;

            dropBody = world.createBody();
            dropBody->type = P::BodyType::Dynamic;
            dropBody->setMass(1.0f);
            dropBody->rotation = M::Quaternion::identity();

            auto dropCollider = std::make_shared<P::BoxCollider>(M::Vector3D(0.75f, 0.75f, 0.75f));
            P::Fixture* dropFixture = world.createFixture(dropBody, dropCollider);
            dropFixture->material.restitution = 0.05f;
            dropFixture->material.staticFriction = 1.0f;
            dropFixture->material.dynamicFriction = 0.7f;
        }

        void respawnDropCube(P::RigidBody* dropBody) {
            if (!dropBody) {
                return;
            }

            dropBody->position = M::Point3D(0.0f, 2.5f, 0.0f);
            dropBody->rotation = M::Quaternion::identity();
            dropBody->linearVelocity = M::Vector3D(0.0f, 0.0f, 0.0f);
            dropBody->angularVelocity = M::Vector3D(0.0f, 0.0f, 0.0f);
            dropBody->forceAccum = M::Vector3D(0.0f, 0.0f, 0.0f);
            dropBody->torqueAccum = M::Vector3D(0.0f, 0.0f, 0.0f);
            dropBody->awake = true;
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

        void drawMeshFromSnapshot(const G::Mesh& mesh,
                                const BodySnapshot& body,
                                const G::Viewport& viewport,
                                const G::Color& color,
                                float thickness) {
            M::Transform3D t;
            t.position = body.position;
            t.rotation = body.rotation;
            t.scale = M::Vector3D(1.0f, 1.0f, 1.0f);

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
        void startPhysicsThread() {
            m_physicsRunning = true;
            m_physicsThread = std::thread([this]() { physicsLoop(); });
        }

        void publishSnapshot(const P::RigidBody* baseBody, const P::RigidBody* dropBody) {
            PhysicsSnapshot next;

            if (baseBody) {
                next.base.valid = true;
                next.base.position = baseBody->position;
                next.base.rotation = baseBody->rotation;
            }

            if (dropBody) {
                next.cube.valid = true;
                next.cube.position = dropBody->position;
                next.cube.rotation = dropBody->rotation;
            }

            // Static imported model snapshot
            next.model.valid = true;
            next.model.position = M::Point3D(0.0f, -1.0f, -4.0f);
            next.model.rotation = M::Quaternion::identity();

            std::lock_guard<std::mutex> lock(m_snapshotMutex);
            m_snapshot = next;
        }

        PhysicsSnapshot readSnapshot() const {
            std::lock_guard<std::mutex> lock(m_snapshotMutex);
            return m_snapshot;
        }

        void stopPhysicsThread() {
            m_physicsRunning = false;
            if (m_physicsThread.joinable()) {
                m_physicsThread.join();
            }
        }

        void physicsLoop() {
            P::World world;

            world.settings.restingLinearThreshold = 0.35f;
            world.settings.restingAngularThreshold = 0.9f;
            world.settings.settleAssistTorque = 18.0f;
            world.settings.settleSnapDot = 0.9965f;

            world.settings.solver.velocityIterations = 10;
            world.settings.solver.positionIterations = 2;
            world.settings.solver.baumgarte = 0.5f;
            world.settings.solver.penetrationSlop = 0.02f;

            P::RigidBody* baseBody = nullptr;
            P::RigidBody* dropBody = nullptr;
            P::RigidBody* modelBody = nullptr;

            setupPhysicsWorld(world, baseBody, dropBody, modelBody);
            respawnDropCube(dropBody);

            double dropTimer = 0.0;
            double platformTime = 0.0;

            using clock = std::chrono::steady_clock;
            auto previous = clock::now();
            constexpr double fixedDt = 1.0 / 60.0;
            double accumulator = 0.0;

            while (m_physicsRunning) {
                const auto now = clock::now();
                const std::chrono::duration<double> frameSpan = now - previous;
                previous = now;

                double frameDt = frameSpan.count();
                if (frameDt > 0.25) {
                    frameDt = 0.25;
                }
                accumulator += frameDt;

                while (accumulator >= fixedDt) {
                    if (baseBody) {
                        platformTime += fixedDt;
                        const float prevX = baseBody->position.x;
                        const float newX = static_cast<float>(m_platformAmplitude * std::sin(platformTime * m_platformSpeed));
                        baseBody->position.x = newX;
                        baseBody->linearVelocity = M::Vector3D(
                            static_cast<float>((newX - prevX) / fixedDt),
                            0.0f,
                            0.0f
                        );
                        baseBody->angularVelocity = M::Vector3D(0.0f, 0.0f, 0.0f);
                    }

                    world.step(static_cast<float>(fixedDt));

                    dropTimer += fixedDt;
                    if (dropTimer >= 10.0) {
                        dropTimer = 0.0;
                        respawnDropCube(dropBody);
                    }

                    publishSnapshot(baseBody, dropBody, modelBody);
                    accumulator -= fixedDt;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
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
