#ifndef G_HPP
#define G_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include "M.hpp"

namespace G {

    // =========================================================
    // Basic rendering data
    // =========================================================

    struct Color {
        float r;
        float g;
        float b;
        float a;

        constexpr Color() noexcept
            : r(1.0f), g(1.0f), b(1.0f), a(1.0f) {}

        constexpr Color(float _r, float _g, float _b, float _a = 1.0f) noexcept
            : r(_r), g(_g), b(_b), a(_a) {}

        [[nodiscard]] static Color white() noexcept;
        [[nodiscard]] static Color black() noexcept;
        [[nodiscard]] static Color red() noexcept;
        [[nodiscard]] static Color green() noexcept;
        [[nodiscard]] static Color blue() noexcept;
        [[nodiscard]] static Color transparent() noexcept;
    };

    struct Viewport {
        int x;
        int y;
        int width;
        int height;

        constexpr Viewport() noexcept
            : x(0), y(0), width(0), height(0) {}

        constexpr Viewport(int _x, int _y, int _width, int _height) noexcept
            : x(_x), y(_y), width(_width), height(_height) {}

        [[nodiscard]] bool isValid() const noexcept;
        [[nodiscard]] float aspect() const noexcept;
    };

    // =========================================================
    // Mesh layer
    // =========================================================

    struct Vertex {
        M::Point3D position;
        M::Vector3D normal;
        M::Vector2D uv;
    };

    struct Triangle {
        int a;
        int b;
        int c;
    };

    class Mesh {
    public:
        std::vector<Vertex> vertices;
        std::vector<Triangle> triangles;

        [[nodiscard]] bool empty() const noexcept;
        void clear();
        [[nodiscard]] bool valid() const noexcept;

        [[nodiscard]] std::size_t vertexCount() const noexcept;
        [[nodiscard]] std::size_t triangleCount() const noexcept;

        [[nodiscard]] M::AABB3D bounds() const noexcept;

        void computeFlatNormals() noexcept;
        void computeSmoothNormals() noexcept;
        void flipWinding() noexcept;
        void invertNormals() noexcept;

        [[nodiscard]] Mesh transformed(const M::Transform3D& transform) const noexcept;
    };

    // =========================================================
    // 2D geometry for rendering
    // =========================================================

    struct Vertex2D {
        M::Point2D position;
        M::Vector2D uv;
        Color color;
    };

    struct LineVertex2D {
        M::Point2D position;
        Color color;
    };

    struct Triangle2DIndexed {
        int a;
        int b;
        int c;
    };

    class Mesh2D {
    public:
        std::vector<Vertex2D> vertices;
        std::vector<Triangle2DIndexed> triangles;

        [[nodiscard]] bool empty() const noexcept;
        void clear();
        [[nodiscard]] bool valid() const noexcept;

        [[nodiscard]] std::size_t vertexCount() const noexcept;
        [[nodiscard]] std::size_t triangleCount() const noexcept;

        [[nodiscard]] M::AABB2D bounds() const noexcept;

        [[nodiscard]] Mesh2D transformed(const M::Transform2D& transform) const noexcept;
    };

    // =========================================================
    // Materials / textures
    // =========================================================

    class Texture2D {
    public:
        Texture2D() = default;
        virtual ~Texture2D() = default;

        [[nodiscard]] virtual int width() const noexcept = 0;
        [[nodiscard]] virtual int height() const noexcept = 0;
        [[nodiscard]] virtual bool valid() const noexcept = 0;
    };

    struct Material2D {
        Color tint;
        std::shared_ptr<Texture2D> texture;
        bool visible;
        bool wireframe;

        constexpr Material2D() noexcept
            : tint(), texture(), visible(true), wireframe(false) {}
    };

    struct Material3D {
        Color albedo;
        std::shared_ptr<Texture2D> diffuseMap;
        std::shared_ptr<Texture2D> normalMap;
        float opacity;
        float roughness;
        float metallic;
        bool visible;
        bool wireframe;
        bool doubleSided;
        bool unlit;

        constexpr Material3D() noexcept
            : albedo(),
              diffuseMap(),
              normalMap(),
              opacity(1.0f),
              roughness(1.0f),
              metallic(0.0f),
              visible(true),
              wireframe(false),
              doubleSided(false),
              unlit(false) {}
    };

    // =========================================================
    // Cameras
    // =========================================================

    class Camera2D {
    public:
        M::Point2D position;
        float rotation;
        float zoom;

        constexpr Camera2D() noexcept
            : position(), rotation(0.0f), zoom(1.0f) {}

        constexpr Camera2D(const M::Point2D& _position, float _rotation, float _zoom) noexcept
            : position(_position), rotation(_rotation), zoom(_zoom) {}

        [[nodiscard]] M::Matrix3x3 viewMatrix() const noexcept;
        [[nodiscard]] M::Matrix3x3 projectionMatrix(const Viewport& viewport) const noexcept;
        [[nodiscard]] M::Point2D worldToScreen(const M::Point2D& worldPoint, const Viewport& viewport) const noexcept;
        [[nodiscard]] M::Point2D screenToWorld(const M::Point2D& screenPoint, const Viewport& viewport) const noexcept;
    };

    class Camera3D {
    public:
        M::Point3D position;
        M::Quaternion rotation;

        float fovYRadians;
        float nearPlane;
        float farPlane;

        constexpr Camera3D() noexcept
            : position(),
              rotation(),
              fovYRadians(1.0471975512f),
              nearPlane(0.1f),
              farPlane(1000.0f) {}

        constexpr Camera3D(const M::Point3D& _position,
                           const M::Quaternion& _rotation,
                           float _fovYRadians,
                           float _nearPlane,
                           float _farPlane) noexcept
            : position(_position),
              rotation(_rotation),
              fovYRadians(_fovYRadians),
              nearPlane(_nearPlane),
              farPlane(_farPlane) {}

        [[nodiscard]] M::Matrix4x4 viewMatrix() const noexcept;
        [[nodiscard]] M::Matrix4x4 projectionMatrix(float aspect) const noexcept;
        [[nodiscard]] M::Matrix4x4 viewProjectionMatrix(float aspect) const noexcept;

        [[nodiscard]] M::Vector3D forward() const noexcept;
        [[nodiscard]] M::Vector3D right() const noexcept;
        [[nodiscard]] M::Vector3D up() const noexcept;
    };

    // =========================================================
    // Lights
    // =========================================================

    enum class LightType {
        Directional,
        Point,
        Spot
    };

    struct Light3D {
        LightType type;
        Color color;
        float intensity;

        M::Point3D position;
        M::Vector3D direction;

        float range;
        float innerConeRadians;
        float outerConeRadians;

        constexpr Light3D() noexcept
            : type(LightType::Directional),
              color(Color::white()),
              intensity(1.0f),
              position(),
              direction(0.0f, -1.0f, 0.0f),
              range(10.0f),
              innerConeRadians(0.5f),
              outerConeRadians(1.0f) {}
    };

    struct AmbientLight {
        Color color;
        float intensity;

        constexpr AmbientLight() noexcept
            : color(Color::white()), intensity(0.1f) {}
    };

    // =========================================================
    // Renderable objects
    // =========================================================

    class Sprite2D {
    public:
        M::Transform2D transform;
        M::Vector2D size;
        Material2D material;
        M::Vector2D uvMin;
        M::Vector2D uvMax;
        float layer;
        bool visible;

        constexpr Sprite2D() noexcept
            : transform(),
              size(1.0f, 1.0f),
              material(),
              uvMin(0.0f, 0.0f),
              uvMax(1.0f, 1.0f),
              layer(0.0f),
              visible(true) {}

        [[nodiscard]] M::AABB2D bounds() const noexcept;
    };

    class Shape2D {
    public:
        M::Transform2D transform;
        std::shared_ptr<Mesh2D> mesh;
        Material2D material;
        float layer;
        bool visible;

        constexpr Shape2D() noexcept
            : transform(),
              mesh(),
              material(),
              layer(0.0f),
              visible(true) {}

        [[nodiscard]] bool valid() const noexcept;
        [[nodiscard]] M::AABB2D bounds() const noexcept;
    };

    class Object3D {
    public:
        M::Transform3D transform;
        std::shared_ptr<Mesh> mesh;
        Material3D material;
        bool visible;
        bool castShadows;
        bool receiveShadows;

        constexpr Object3D() noexcept
            : transform(),
              mesh(),
              material(),
              visible(true),
              castShadows(true),
              receiveShadows(true) {}

        [[nodiscard]] bool valid() const noexcept;
        [[nodiscard]] M::AABB3D localBounds() const noexcept;
        [[nodiscard]] M::AABB3D worldBounds() const noexcept;
    };

    // =========================================================
    // Projected / intermediate render structures
    // =========================================================

    struct ClipVertex {
        M::Vector4D position;
        M::Vector3D normal;
        M::Vector2D uv;
        Color color;
    };

    struct ScreenVertex {
        float x;
        float y;
        float z;
        float invW;
        M::Vector3D normal;
        M::Vector2D uv;
        Color color;
    };

    struct RenderTriangle2D {
        Vertex2D a;
        Vertex2D b;
        Vertex2D c;
        float layer;
    };

    struct RenderLine2D {
        LineVertex2D a;
        LineVertex2D b;
        float layer;
    };

    struct RenderTriangle3D {
        ScreenVertex a;
        ScreenVertex b;
        ScreenVertex c;
        Material3D material;
        float depth;
        std::uint32_t objectId;
    };

    struct RenderCommand2D {
        enum class Type {
            Line,
            Triangle,
            Sprite
        };

        Type type;
        float layer;
    };

    struct RenderCommand3D {
        enum class Type {
            Triangle,
            Line,
            Point
        };

        Type type;
        float depth;
        std::uint32_t objectId;
    };

    // =========================================================
    // Scene containers
    // =========================================================

    class Scene2D {
    public:
        Camera2D camera;
        std::vector<Sprite2D> sprites;
        std::vector<Shape2D> shapes;
        Color clearColor;

        Scene2D() noexcept;
        void clear() noexcept;
    };

    class Scene3D {
    public:
        Camera3D camera;
        AmbientLight ambientLight;
        std::vector<Light3D> lights;
        std::vector<Object3D> objects;
        Color clearColor;

        Scene3D() noexcept;
        void clear() noexcept;
    };

    // =========================================================
    // Render settings / stats
    // =========================================================

    struct RenderSettings2D {
        bool sortByLayer;
        bool cullOutsideViewport;

        constexpr RenderSettings2D() noexcept
            : sortByLayer(true), cullOutsideViewport(true) {}
    };

    struct RenderSettings3D {
        bool backfaceCulling;
        bool depthTest;
        bool perspectiveCorrectUV;
        bool wireframeOnly;
        bool sortTransparentBackToFront;
        bool clipToFrustum;

        constexpr RenderSettings3D() noexcept
            : backfaceCulling(true),
              depthTest(true),
              perspectiveCorrectUV(true),
              wireframeOnly(false),
              sortTransparentBackToFront(true),
              clipToFrustum(true) {}
    };

    struct RenderStats2D {
        std::size_t spritesSubmitted;
        std::size_t shapesSubmitted;
        std::size_t trianglesSubmitted;
        std::size_t linesSubmitted;
    };

    struct RenderStats3D {
        std::size_t objectsSubmitted;
        std::size_t trianglesSubmitted;
        std::size_t trianglesCulled;
        std::size_t trianglesClipped;
        std::size_t drawCalls;
    };

    // =========================================================
    // Renderer interfaces
    // =========================================================

    class IRenderer2D {
    public:
        virtual ~IRenderer2D() = default;

        virtual void beginFrame(const Viewport& viewport) = 0;
        virtual void endFrame() = 0;

        virtual void clear(const Color& color) = 0;

        virtual void drawLine(const M::Point2D& a,
                              const M::Point2D& b,
                              const Color& color) = 0;

        virtual void drawTriangle(const Vertex2D& a,
                                  const Vertex2D& b,
                                  const Vertex2D& c,
                                  const Material2D& material) = 0;

        virtual void drawSprite(const Sprite2D& sprite,
                                const Camera2D& camera,
                                const Viewport& viewport) = 0;

        virtual void drawShape(const Shape2D& shape,
                               const Camera2D& camera,
                               const Viewport& viewport) = 0;

        virtual void render(const Scene2D& scene,
                            const Viewport& viewport,
                            const RenderSettings2D& settings) = 0;

        [[nodiscard]] virtual RenderStats2D stats() const noexcept = 0;
    };

    class IRenderer3D {
    public:
        virtual ~IRenderer3D() = default;

        virtual void beginFrame(const Viewport& viewport) = 0;
        virtual void endFrame() = 0;

        virtual void clear(const Color& color) = 0;

        virtual void drawLine3D(const M::Point3D& a,
                                const M::Point3D& b,
                                const Color& color,
                                const Camera3D& camera,
                                const Viewport& viewport) = 0;

        virtual void drawMesh(const Mesh& mesh,
                              const M::Transform3D& transform,
                              const Material3D& material,
                              const Camera3D& camera,
                              const Viewport& viewport,
                              const RenderSettings3D& settings) = 0;

        virtual void drawObject(const Object3D& object,
                                const Camera3D& camera,
                                const Viewport& viewport,
                                const RenderSettings3D& settings) = 0;

        virtual void render(const Scene3D& scene,
                            const Viewport& viewport,
                            const RenderSettings3D& settings) = 0;

        [[nodiscard]] virtual RenderStats3D stats() const noexcept = 0;
    };

    // =========================================================
    // Projection / pipeline helpers
    // =========================================================

    [[nodiscard]] M::Vector4D toHomogeneousPoint(const M::Point3D& p) noexcept;
    [[nodiscard]] M::Vector4D toHomogeneousVector(const M::Vector3D& v) noexcept;

    [[nodiscard]] M::Point3D fromHomogeneousPoint(const M::Vector4D& v) noexcept;

    [[nodiscard]] ClipVertex transformToClip(const Vertex& vertex,
                                             const M::Matrix4x4& model,
                                             const M::Matrix4x4& viewProjection) noexcept;

    [[nodiscard]] ScreenVertex perspectiveDivide(const ClipVertex& v,
                                                 const Viewport& viewport) noexcept;

    [[nodiscard]] M::Point2D ndcToScreen(const M::Point2D& ndc, const Viewport& viewport) noexcept;
    [[nodiscard]] M::Point2D screenToNdc(const M::Point2D& screen, const Viewport& viewport) noexcept;

    [[nodiscard]] bool isInsideClipVolume(const M::Vector4D& clipPos) noexcept;
    [[nodiscard]] bool isBackFacing(const ScreenVertex& a,
                                    const ScreenVertex& b,
                                    const ScreenVertex& c) noexcept;

    [[nodiscard]] float triangleDepth(const ScreenVertex& a,
                                      const ScreenVertex& b,
                                      const ScreenVertex& c) noexcept;

    // =========================================================
    // Mesh factories
    // =========================================================

    [[nodiscard]] Mesh makeTriangle(const M::Point3D& a,
                                    const M::Point3D& b,
                                    const M::Point3D& c) noexcept;

    [[nodiscard]] Mesh makeQuad(float width, float height) noexcept;
    [[nodiscard]] Mesh makePlane(float width, float depth,
                                 int subdivisionsX = 1,
                                 int subdivisionsZ = 1) noexcept;

    [[nodiscard]] Mesh makeCube(float width, float height, float depth) noexcept;
    [[nodiscard]] Mesh makeCuboid(const M::Vector3D& size) noexcept;
    [[nodiscard]] Mesh makePyramid(float baseSize, float height) noexcept;
    [[nodiscard]] Mesh makeSphere(float radius,
                                  int slices = 16,
                                  int stacks = 16) noexcept;
    [[nodiscard]] Mesh makeCylinder(float radius,
                                    float height,
                                    int slices = 16) noexcept;
    [[nodiscard]] Mesh makeCone(float radius,
                                float height,
                                int slices = 16) noexcept;

    [[nodiscard]] Mesh2D makeTriangle2D(const M::Point2D& a,
                                        const M::Point2D& b,
                                        const M::Point2D& c) noexcept;

    [[nodiscard]] Mesh2D makeQuad2D(float width, float height) noexcept;
    [[nodiscard]] Mesh2D makeRectangle2D(const M::Vector2D& size) noexcept;
    [[nodiscard]] Mesh2D makeCircle2D(float radius,
                                      int segments = 32) noexcept;
    [[nodiscard]] Mesh2D makePolygon2D(const std::vector<M::Point2D>& points) noexcept;

    // =========================================================
    // Mesh utility helpers
    // =========================================================

    [[nodiscard]] M::Vector3D computeFaceNormal(const Mesh& mesh,
                                                const Triangle& triangle) noexcept;

    [[nodiscard]] M::Point3D computeTriangleCentroid(const Mesh& mesh,
                                                     const Triangle& triangle) noexcept;

    [[nodiscard]] M::AABB3D computeBounds(const Mesh& mesh) noexcept;
    [[nodiscard]] M::AABB2D computeBounds(const Mesh2D& mesh) noexcept;

    [[nodiscard]] bool validateMesh(const Mesh& mesh) noexcept;
    [[nodiscard]] bool validateMesh(const Mesh2D& mesh) noexcept;

    void recomputeFlatNormals(Mesh& mesh) noexcept;
    void recomputeSmoothNormals(Mesh& mesh) noexcept;

    // =========================================================
    // Optional asset-loading interface
    // =========================================================

    class IMeshLoader {
    public:
        virtual ~IMeshLoader() = default;
        [[nodiscard]] virtual bool canLoad(const std::string& path) const noexcept = 0;
        [[nodiscard]] virtual Mesh load(const std::string& path) const = 0;
    };

}

#endif