#include "G.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
    [[nodiscard]] constexpr float PI() noexcept {
        return 3.14159265358979323846f;
    }

    [[nodiscard]] constexpr float TAU() noexcept {
        return 6.28318530717958647692f;
    }

    [[nodiscard]] inline float safeInv(float x) noexcept {
        return (std::fabs(x) < M::EPS) ? 0.0f : (1.0f / x);
    }

    [[nodiscard]] inline float clamp01(float x) noexcept {
        return std::max(0.0f, std::min(1.0f, x));
    }

    [[nodiscard]] inline G::Color multiplyColor(const G::Color& c, float k) noexcept {
        return G::Color(c.r * k, c.g * k, c.b * k, c.a);
    }

    [[nodiscard]] inline M::Vector3D transformDirection(const M::Matrix4x4& m, const M::Vector3D& v) noexcept {
        const M::Vector4D hv(v, 0.0f);
        const M::Vector4D out = m * hv;
        return M::Vector3D(out.x, out.y, out.z);
    }

    [[nodiscard]] inline M::Point3D transformPoint(const M::Matrix4x4& m, const M::Point3D& p) noexcept {
        const M::Vector4D hp(p.x, p.y, p.z, 1.0f);
        const M::Vector4D out = m * hp;
        if (std::fabs(out.w) < M::EPS) {
            return M::Point3D(out.x, out.y, out.z);
        }
        return M::Point3D(out.x / out.w, out.y / out.w, out.z / out.w);
    }

    [[nodiscard]] inline M::Point2D transformPoint(const M::Matrix3x3& m, const M::Point2D& p) noexcept {
        const M::Vector3D hp(p.v, p.w, 1.0f);
        const M::Vector3D out = m * hp;
        if (std::fabs(out.z) < M::EPS) {
            return M::Point2D(out.x, out.y);
        }
        return M::Point2D(out.x / out.z, out.y / out.z);
    }

    [[nodiscard]] inline bool validIndex(int idx, std::size_t n) noexcept {
        return idx >= 0 && static_cast<std::size_t>(idx) < n;
    }

    [[nodiscard]] inline M::Matrix4x4 makePerspective(float fovYRadians, float aspect, float nearPlane, float farPlane) noexcept {
        const float f = 1.0f / std::tan(fovYRadians * 0.5f);
        const float nf = 1.0f / (nearPlane - farPlane);
        return M::Matrix4x4(
            f / aspect, 0.0f, 0.0f, 0.0f,
            0.0f, f, 0.0f, 0.0f,
            0.0f, 0.0f, (farPlane + nearPlane) * nf, (2.0f * farPlane * nearPlane) * nf,
            0.0f, 0.0f, -1.0f, 0.0f
        );
    }

    [[nodiscard]] inline M::Matrix3x3 makeOrtho2D(float left, float right, float bottom, float top) noexcept {
        const float rl = right - left;
        const float tb = top - bottom;
        return M::Matrix3x3(
            2.0f / rl, 0.0f, -(right + left) / rl,
            0.0f, 2.0f / tb, -(top + bottom) / tb,
            0.0f, 0.0f, 1.0f
        );
    }

    [[nodiscard]] inline M::Point3D triangleCentroid(const G::Mesh& mesh, const G::Triangle& t) noexcept {
        const auto& a = mesh.vertices[static_cast<std::size_t>(t.a)].position;
        const auto& b = mesh.vertices[static_cast<std::size_t>(t.b)].position;
        const auto& c = mesh.vertices[static_cast<std::size_t>(t.c)].position;
        return M::Point3D(
            (a.x + b.x + c.x) / 3.0f,
            (a.y + b.y + c.y) / 3.0f,
            (a.z + b.z + c.z) / 3.0f
        );
    }
}

namespace G {

// =========================================================
// Color
// =========================================================

Color Color::white() noexcept { return Color(1.0f, 1.0f, 1.0f, 1.0f); }
Color Color::black() noexcept { return Color(0.0f, 0.0f, 0.0f, 1.0f); }
Color Color::red() noexcept { return Color(1.0f, 0.0f, 0.0f, 1.0f); }
Color Color::green() noexcept { return Color(0.0f, 1.0f, 0.0f, 1.0f); }
Color Color::blue() noexcept { return Color(0.0f, 0.0f, 1.0f, 1.0f); }
Color Color::transparent() noexcept { return Color(0.0f, 0.0f, 0.0f, 0.0f); }

// =========================================================
// Viewport
// =========================================================

bool Viewport::isValid() const noexcept {
    return width > 0 && height > 0;
}

float Viewport::aspect() const noexcept {
    if (height <= 0) return 1.0f;
    return static_cast<float>(width) / static_cast<float>(height);
}

// =========================================================
// Mesh
// =========================================================

bool Mesh::empty() const noexcept {
    return vertices.empty() || triangles.empty();
}

void Mesh::clear() {
    vertices.clear();
    triangles.clear();
}

bool Mesh::valid() const noexcept {
    for (const auto& t : triangles) {
        if (!validIndex(t.a, vertices.size()) ||
            !validIndex(t.b, vertices.size()) ||
            !validIndex(t.c, vertices.size())) {
            return false;
        }
    }
    return true;
}

std::size_t Mesh::vertexCount() const noexcept {
    return vertices.size();
}

std::size_t Mesh::triangleCount() const noexcept {
    return triangles.size();
}

M::AABB3D Mesh::bounds() const noexcept {
    return computeBounds(*this);
}

void Mesh::computeFlatNormals() noexcept {
    recomputeFlatNormals(*this);
}

void Mesh::computeSmoothNormals() noexcept {
    recomputeSmoothNormals(*this);
}

void Mesh::flipWinding() noexcept {
    for (auto& t : triangles) {
        std::swap(t.b, t.c);
    }
}

void Mesh::invertNormals() noexcept {
    for (auto& v : vertices) {
        v.normal.negate();
    }
}

Mesh Mesh::transformed(const M::Transform3D& transform) const noexcept {
    Mesh out = *this;
    const M::Matrix4x4 m = transform.matrix();
    for (auto& v : out.vertices) {
        v.position = transformPoint(m, v.position);
        v.normal = transformDirection(m, v.normal).normalized();
    }
    return out;
}

// =========================================================
// Mesh2D
// =========================================================

bool Mesh2D::empty() const noexcept {
    return vertices.empty() || triangles.empty();
}

void Mesh2D::clear() {
    vertices.clear();
    triangles.clear();
}

bool Mesh2D::valid() const noexcept {
    for (const auto& t : triangles) {
        if (!validIndex(t.a, vertices.size()) ||
            !validIndex(t.b, vertices.size()) ||
            !validIndex(t.c, vertices.size())) {
            return false;
        }
    }
    return true;
}

std::size_t Mesh2D::vertexCount() const noexcept {
    return vertices.size();
}

std::size_t Mesh2D::triangleCount() const noexcept {
    return triangles.size();
}

M::AABB2D Mesh2D::bounds() const noexcept {
    return computeBounds(*this);
}

Mesh2D Mesh2D::transformed(const M::Transform2D& transform) const noexcept {
    Mesh2D out = *this;
    const M::Matrix3x3 m = transform.matrix();
    for (auto& v : out.vertices) {
        v.position = ::transformPoint(m, v.position);
    }
    return out;
}

// =========================================================
// Camera2D
// =========================================================

M::Matrix3x3 Camera2D::viewMatrix() const noexcept {
    const M::Matrix3x3 t = M::Matrix3x3::translation(-position.v, -position.w);
    const M::Matrix3x3 r = M::Matrix3x3::rotation(-rotation);
    const float invZoom = (std::fabs(zoom) < M::EPS) ? 1.0f : (1.0f / zoom);
    const M::Matrix3x3 s = M::Matrix3x3::scaling(invZoom, invZoom);
    return s * r * t;
}

M::Matrix3x3 Camera2D::projectionMatrix(const Viewport& viewport) const noexcept {
    const float halfW = static_cast<float>(viewport.width) * 0.5f;
    const float halfH = static_cast<float>(viewport.height) * 0.5f;
    return makeOrtho2D(-halfW, halfW, -halfH, halfH);
}

M::Point2D Camera2D::worldToScreen(const M::Point2D& worldPoint, const Viewport& viewport) const noexcept {
    const M::Point2D viewPt = ::transformPoint(viewMatrix(), worldPoint);
    return M::Point2D(
        viewPt.v + static_cast<float>(viewport.x) + static_cast<float>(viewport.width) * 0.5f,
        static_cast<float>(viewport.y) + static_cast<float>(viewport.height) * 0.5f - viewPt.w
    );
}

M::Point2D Camera2D::screenToWorld(const M::Point2D& screenPoint, const Viewport& viewport) const noexcept {
    const float cx = static_cast<float>(viewport.x) + static_cast<float>(viewport.width) * 0.5f;
    const float cy = static_cast<float>(viewport.y) + static_cast<float>(viewport.height) * 0.5f;
    const M::Point2D local(screenPoint.v - cx, cy - screenPoint.w);
    const M::Matrix3x3 inv = viewMatrix().inverted();
    return ::transformPoint(inv, local);
}

// =========================================================
// Camera3D
// =========================================================

M::Matrix4x4 Camera3D::viewMatrix() const noexcept {
    const M::Matrix4x4 rInv = rotation.conjugated().toMatrix4x4();
    const M::Matrix4x4 tInv = M::Matrix4x4::translation(-position.x, -position.y, -position.z);
    return rInv * tInv;
}

M::Matrix4x4 Camera3D::projectionMatrix(float aspect) const noexcept {
    return makePerspective(fovYRadians, aspect, nearPlane, farPlane);
}

M::Matrix4x4 Camera3D::viewProjectionMatrix(float aspect) const noexcept {
    return projectionMatrix(aspect) * viewMatrix();
}

M::Vector3D Camera3D::forward() const noexcept {
    return rotation.rotatedVector(M::Vector3D(0.0f, 0.0f, -1.0f)).normalized();
}

M::Vector3D Camera3D::right() const noexcept {
    return rotation.rotatedVector(M::Vector3D(1.0f, 0.0f, 0.0f)).normalized();
}

M::Vector3D Camera3D::up() const noexcept {
    return rotation.rotatedVector(M::Vector3D(0.0f, 1.0f, 0.0f)).normalized();
}

// =========================================================
// Sprite2D / Shape2D / Object3D
// =========================================================

M::AABB2D Sprite2D::bounds() const noexcept {
    const M::Vector2D half = size * 0.5f;
    const M::Point2D p1 = transform.appliedTo(M::Point2D(-half.v, -half.w));
    const M::Point2D p2 = transform.appliedTo(M::Point2D( half.v, -half.w));
    const M::Point2D p3 = transform.appliedTo(M::Point2D( half.v,  half.w));
    const M::Point2D p4 = transform.appliedTo(M::Point2D(-half.v,  half.w));

    M::AABB2D box(p1, p1);
    box.expandToInclude(p2);
    box.expandToInclude(p3);
    box.expandToInclude(p4);
    return box;
}

bool Shape2D::valid() const noexcept {
    return mesh && mesh->valid() && !mesh->empty();
}

M::AABB2D Shape2D::bounds() const noexcept {
    if (!mesh) return M::AABB2D();
    return mesh->transformed(transform).bounds();
}

bool Object3D::valid() const noexcept {
    return mesh && mesh->valid() && !mesh->empty();
}

M::AABB3D Object3D::localBounds() const noexcept {
    if (!mesh) return M::AABB3D();
    return mesh->bounds();
}

M::AABB3D Object3D::worldBounds() const noexcept {
    if (!mesh) return M::AABB3D();
    return mesh->transformed(transform).bounds();
}

// =========================================================
// Scene containers
// =========================================================

Scene2D::Scene2D() noexcept
    : camera(), sprites(), shapes(), clearColor(Color::black()) {}

void Scene2D::clear() noexcept {
    sprites.clear();
    shapes.clear();
}

Scene3D::Scene3D() noexcept
    : camera(), ambientLight(), lights(), objects(), clearColor(Color::black()) {}

void Scene3D::clear() noexcept {
    lights.clear();
    objects.clear();
}

// =========================================================
// Projection / pipeline helpers
// =========================================================

M::Vector4D toHomogeneousPoint(const M::Point3D& p) noexcept {
    return M::Vector4D(p.x, p.y, p.z, 1.0f);
}

M::Vector4D toHomogeneousVector(const M::Vector3D& v) noexcept {
    return M::Vector4D(v.x, v.y, v.z, 0.0f);
}

M::Point3D fromHomogeneousPoint(const M::Vector4D& v) noexcept {
    if (std::fabs(v.w) < M::EPS) {
        return M::Point3D(v.x, v.y, v.z);
    }
    return M::Point3D(v.x / v.w, v.y / v.w, v.z / v.w);
}

ClipVertex transformToClip(const Vertex& vertex,
                           const M::Matrix4x4& model,
                           const M::Matrix4x4& viewProjection) noexcept {
    ClipVertex out{};
    const M::Vector4D worldPos = model * toHomogeneousPoint(vertex.position);
    out.position = viewProjection * worldPos;
    out.normal = transformDirection(model, vertex.normal).normalized();
    out.uv = vertex.uv;
    out.color = Color::white();
    return out;
}

ScreenVertex perspectiveDivide(const ClipVertex& v, const Viewport& viewport) noexcept {
    ScreenVertex out{};
    const float invW = (std::fabs(v.position.w) < M::EPS) ? 0.0f : (1.0f / v.position.w);
    const float ndcX = v.position.x * invW;
    const float ndcY = v.position.y * invW;
    const float ndcZ = v.position.z * invW;

    out.x = static_cast<float>(viewport.x) + (ndcX + 1.0f) * 0.5f * static_cast<float>(viewport.width);
    out.y = static_cast<float>(viewport.y) + (1.0f - (ndcY + 1.0f) * 0.5f) * static_cast<float>(viewport.height);
    out.z = ndcZ;
    out.invW = invW;
    out.normal = v.normal;
    out.uv = v.uv;
    out.color = v.color;
    return out;
}

M::Point2D ndcToScreen(const M::Point2D& ndc, const Viewport& viewport) noexcept {
    return M::Point2D(
        static_cast<float>(viewport.x) + (ndc.v + 1.0f) * 0.5f * static_cast<float>(viewport.width),
        static_cast<float>(viewport.y) + (1.0f - (ndc.w + 1.0f) * 0.5f) * static_cast<float>(viewport.height)
    );
}

M::Point2D screenToNdc(const M::Point2D& screen, const Viewport& viewport) noexcept {
    const float x = (screen.v - static_cast<float>(viewport.x)) / static_cast<float>(viewport.width);
    const float y = (screen.w - static_cast<float>(viewport.y)) / static_cast<float>(viewport.height);
    return M::Point2D(x * 2.0f - 1.0f, 1.0f - y * 2.0f);
}

bool isInsideClipVolume(const M::Vector4D& clipPos) noexcept {
    const float w = std::fabs(clipPos.w);
    return clipPos.x >= -w && clipPos.x <= w &&
           clipPos.y >= -w && clipPos.y <= w &&
           clipPos.z >= -w && clipPos.z <= w;
}

bool isBackFacing(const ScreenVertex& a,
                  const ScreenVertex& b,
                  const ScreenVertex& c) noexcept {
    const float abx = b.x - a.x;
    const float aby = b.y - a.y;
    const float acx = c.x - a.x;
    const float acy = c.y - a.y;
    const float cross = abx * acy - aby * acx;
    return cross >= 0.0f;
}

float triangleDepth(const ScreenVertex& a,
                    const ScreenVertex& b,
                    const ScreenVertex& c) noexcept {
    return (a.z + b.z + c.z) / 3.0f;
}

// =========================================================
// Mesh factories 3D
// =========================================================

Mesh makeTriangle(const M::Point3D& a,
                  const M::Point3D& b,
                  const M::Point3D& c) noexcept {
    Mesh mesh;
    const M::Vector3D n = ((b - a).cross(c - a)).normalized();
    mesh.vertices = {
        {a, n, M::Vector2D(0.0f, 0.0f)},
        {b, n, M::Vector2D(1.0f, 0.0f)},
        {c, n, M::Vector2D(0.5f, 1.0f)}
    };
    mesh.triangles = {{0, 1, 2}};
    return mesh;
}

Mesh makeQuad(float width, float height) noexcept {
    Mesh mesh;
    const float hx = width * 0.5f;
    const float hy = height * 0.5f;
    const M::Vector3D n(0.0f, 0.0f, 1.0f);
    mesh.vertices = {
        {M::Point3D(-hx, -hy, 0.0f), n, M::Vector2D(0.0f, 1.0f)},
        {M::Point3D( hx, -hy, 0.0f), n, M::Vector2D(1.0f, 1.0f)},
        {M::Point3D( hx,  hy, 0.0f), n, M::Vector2D(1.0f, 0.0f)},
        {M::Point3D(-hx,  hy, 0.0f), n, M::Vector2D(0.0f, 0.0f)}
    };
    mesh.triangles = {{0, 1, 2}, {0, 2, 3}};
    return mesh;
}

Mesh makePlane(float width, float depth, int subdivisionsX, int subdivisionsZ) noexcept {
    Mesh mesh;
    subdivisionsX = std::max(1, subdivisionsX);
    subdivisionsZ = std::max(1, subdivisionsZ);

    const int vx = subdivisionsX + 1;
    const int vz = subdivisionsZ + 1;
    const float halfW = width * 0.5f;
    const float halfD = depth * 0.5f;

    mesh.vertices.reserve(static_cast<std::size_t>(vx * vz));
    for (int z = 0; z < vz; ++z) {
        const float tz = static_cast<float>(z) / static_cast<float>(subdivisionsZ);
        const float pz = -halfD + tz * depth;
        for (int x = 0; x < vx; ++x) {
            const float tx = static_cast<float>(x) / static_cast<float>(subdivisionsX);
            const float px = -halfW + tx * width;
            mesh.vertices.push_back({
                M::Point3D(px, 0.0f, pz),
                M::Vector3D(0.0f, 1.0f, 0.0f),
                M::Vector2D(tx, tz)
            });
        }
    }

    for (int z = 0; z < subdivisionsZ; ++z) {
        for (int x = 0; x < subdivisionsX; ++x) {
            const int i0 = z * vx + x;
            const int i1 = i0 + 1;
            const int i2 = i0 + vx;
            const int i3 = i2 + 1;
            mesh.triangles.push_back({i0, i2, i1});
            mesh.triangles.push_back({i1, i2, i3});
        }
    }
    return mesh;
}

Mesh makeCube(float width, float height, float depth) noexcept {
    return makeCuboid(M::Vector3D(width, height, depth));
}

Mesh makeCuboid(const M::Vector3D& size) noexcept {
    Mesh mesh;
    const float hx = size.x * 0.5f;
    const float hy = size.y * 0.5f;
    const float hz = size.z * 0.5f;

    auto addFace = [&](const M::Point3D& p0, const M::Point3D& p1,
                       const M::Point3D& p2, const M::Point3D& p3,
                       const M::Vector3D& n) {
        const int base = static_cast<int>(mesh.vertices.size());
        mesh.vertices.push_back({p0, n, M::Vector2D(0.0f, 1.0f)});
        mesh.vertices.push_back({p1, n, M::Vector2D(1.0f, 1.0f)});
        mesh.vertices.push_back({p2, n, M::Vector2D(1.0f, 0.0f)});
        mesh.vertices.push_back({p3, n, M::Vector2D(0.0f, 0.0f)});
        mesh.triangles.push_back({base + 0, base + 1, base + 2});
        mesh.triangles.push_back({base + 0, base + 2, base + 3});
    };

    addFace(M::Point3D(-hx, -hy,  hz), M::Point3D( hx, -hy,  hz), M::Point3D( hx,  hy,  hz), M::Point3D(-hx,  hy,  hz), M::Vector3D(0.0f, 0.0f, 1.0f));
    addFace(M::Point3D( hx, -hy, -hz), M::Point3D(-hx, -hy, -hz), M::Point3D(-hx,  hy, -hz), M::Point3D( hx,  hy, -hz), M::Vector3D(0.0f, 0.0f, -1.0f));
    addFace(M::Point3D(-hx, -hy, -hz), M::Point3D(-hx, -hy,  hz), M::Point3D(-hx,  hy,  hz), M::Point3D(-hx,  hy, -hz), M::Vector3D(-1.0f, 0.0f, 0.0f));
    addFace(M::Point3D( hx, -hy,  hz), M::Point3D( hx, -hy, -hz), M::Point3D( hx,  hy, -hz), M::Point3D( hx,  hy,  hz), M::Vector3D(1.0f, 0.0f, 0.0f));
    addFace(M::Point3D(-hx,  hy,  hz), M::Point3D( hx,  hy,  hz), M::Point3D( hx,  hy, -hz), M::Point3D(-hx,  hy, -hz), M::Vector3D(0.0f, 1.0f, 0.0f));
    addFace(M::Point3D(-hx, -hy, -hz), M::Point3D( hx, -hy, -hz), M::Point3D( hx, -hy,  hz), M::Point3D(-hx, -hy,  hz), M::Vector3D(0.0f, -1.0f, 0.0f));

    return mesh;
}

Mesh makePyramid(float baseSize, float height) noexcept {
    Mesh mesh;
    const float h = baseSize * 0.5f;
    const M::Point3D p0(-h, 0.0f, -h);
    const M::Point3D p1( h, 0.0f, -h);
    const M::Point3D p2( h, 0.0f,  h);
    const M::Point3D p3(-h, 0.0f,  h);
    const M::Point3D top(0.0f, height, 0.0f);

    mesh = makePlane(baseSize, baseSize, 1, 1);

    auto addTri = [&](const M::Point3D& a, const M::Point3D& b, const M::Point3D& c) {
        const M::Vector3D n = ((b - a).cross(c - a)).normalized();
        const int base = static_cast<int>(mesh.vertices.size());
        mesh.vertices.push_back({a, n, M::Vector2D(0.0f, 1.0f)});
        mesh.vertices.push_back({b, n, M::Vector2D(1.0f, 1.0f)});
        mesh.vertices.push_back({c, n, M::Vector2D(0.5f, 0.0f)});
        mesh.triangles.push_back({base + 0, base + 1, base + 2});
    };

    addTri(p0, p1, top);
    addTri(p1, p2, top);
    addTri(p2, p3, top);
    addTri(p3, p0, top);

    return mesh;
}

Mesh makeSphere(float radius, int slices, int stacks) noexcept {
    Mesh mesh;
    slices = std::max(3, slices);
    stacks = std::max(2, stacks);

    for (int stack = 0; stack <= stacks; ++stack) {
        const float v = static_cast<float>(stack) / static_cast<float>(stacks);
        const float phi = PI() * v;
        const float y = std::cos(phi);
        const float r = std::sin(phi);

        for (int slice = 0; slice <= slices; ++slice) {
            const float u = static_cast<float>(slice) / static_cast<float>(slices);
            const float theta = TAU() * u;
            const float x = r * std::cos(theta);
            const float z = r * std::sin(theta);
            const M::Vector3D n(x, y, z);
            mesh.vertices.push_back({
                M::Point3D(x * radius, y * radius, z * radius),
                n.normalized(),
                M::Vector2D(u, v)
            });
        }
    }

    const int row = slices + 1;
    for (int stack = 0; stack < stacks; ++stack) {
        for (int slice = 0; slice < slices; ++slice) {
            const int i0 = stack * row + slice;
            const int i1 = i0 + 1;
            const int i2 = i0 + row;
            const int i3 = i2 + 1;
            mesh.triangles.push_back({i0, i2, i1});
            mesh.triangles.push_back({i1, i2, i3});
        }
    }
    return mesh;
}

Mesh makeCylinder(float radius, float height, int slices) noexcept {
    Mesh mesh;
    slices = std::max(3, slices);
    const float halfH = height * 0.5f;

    for (int i = 0; i <= slices; ++i) {
        const float u = static_cast<float>(i) / static_cast<float>(slices);
        const float a = TAU() * u;
        const float x = std::cos(a);
        const float z = std::sin(a);
        const M::Vector3D n(x, 0.0f, z);
        mesh.vertices.push_back({M::Point3D(x * radius, -halfH, z * radius), n, M::Vector2D(u, 1.0f)});
        mesh.vertices.push_back({M::Point3D(x * radius,  halfH, z * radius), n, M::Vector2D(u, 0.0f)});
    }

    for (int i = 0; i < slices; ++i) {
        const int i0 = i * 2;
        const int i1 = i0 + 1;
        const int i2 = i0 + 2;
        const int i3 = i0 + 3;
        mesh.triangles.push_back({i0, i2, i1});
        mesh.triangles.push_back({i1, i2, i3});
    }

    const int baseTopCenter = static_cast<int>(mesh.vertices.size());
    mesh.vertices.push_back({M::Point3D(0.0f, halfH, 0.0f), M::Vector3D(0.0f, 1.0f, 0.0f), M::Vector2D(0.5f, 0.5f)});
    const int baseBottomCenter = static_cast<int>(mesh.vertices.size());
    mesh.vertices.push_back({M::Point3D(0.0f, -halfH, 0.0f), M::Vector3D(0.0f, -1.0f, 0.0f), M::Vector2D(0.5f, 0.5f)});

    for (int i = 0; i < slices; ++i) {
        const float u0 = static_cast<float>(i) / static_cast<float>(slices);
        const float u1 = static_cast<float>(i + 1) / static_cast<float>(slices);
        const float a0 = TAU() * u0;
        const float a1 = TAU() * u1;
        const M::Point3D p0(std::cos(a0) * radius, halfH, std::sin(a0) * radius);
        const M::Point3D p1(std::cos(a1) * radius, halfH, std::sin(a1) * radius);
        const M::Point3D q0(std::cos(a0) * radius, -halfH, std::sin(a0) * radius);
        const M::Point3D q1(std::cos(a1) * radius, -halfH, std::sin(a1) * radius);

        int t0 = static_cast<int>(mesh.vertices.size());
        mesh.vertices.push_back({p0, M::Vector3D(0.0f, 1.0f, 0.0f), M::Vector2D(0.0f, 0.0f)});
        mesh.vertices.push_back({p1, M::Vector3D(0.0f, 1.0f, 0.0f), M::Vector2D(1.0f, 0.0f)});
        mesh.triangles.push_back({baseTopCenter, t0, t0 + 1});

        int b0 = static_cast<int>(mesh.vertices.size());
        mesh.vertices.push_back({q0, M::Vector3D(0.0f, -1.0f, 0.0f), M::Vector2D(0.0f, 0.0f)});
        mesh.vertices.push_back({q1, M::Vector3D(0.0f, -1.0f, 0.0f), M::Vector2D(1.0f, 0.0f)});
        mesh.triangles.push_back({baseBottomCenter, b0 + 1, b0});
    }

    return mesh;
}

Mesh makeCone(float radius, float height, int slices) noexcept {
    Mesh mesh;
    slices = std::max(3, slices);
    const float halfH = height * 0.5f;
    const M::Point3D apex(0.0f, halfH, 0.0f);

    for (int i = 0; i < slices; ++i) {
        const float u0 = static_cast<float>(i) / static_cast<float>(slices);
        const float u1 = static_cast<float>(i + 1) / static_cast<float>(slices);
        const float a0 = TAU() * u0;
        const float a1 = TAU() * u1;
        const M::Point3D p0(std::cos(a0) * radius, -halfH, std::sin(a0) * radius);
        const M::Point3D p1(std::cos(a1) * radius, -halfH, std::sin(a1) * radius);
        const M::Vector3D n = ((p1 - p0).cross(apex - p0)).normalized();
        const int base = static_cast<int>(mesh.vertices.size());
        mesh.vertices.push_back({p0, n, M::Vector2D(0.0f, 1.0f)});
        mesh.vertices.push_back({p1, n, M::Vector2D(1.0f, 1.0f)});
        mesh.vertices.push_back({apex, n, M::Vector2D(0.5f, 0.0f)});
        mesh.triangles.push_back({base + 0, base + 1, base + 2});
    }

    const int center = static_cast<int>(mesh.vertices.size());
    mesh.vertices.push_back({M::Point3D(0.0f, -halfH, 0.0f), M::Vector3D(0.0f, -1.0f, 0.0f), M::Vector2D(0.5f, 0.5f)});
    for (int i = 0; i < slices; ++i) {
        const float u0 = static_cast<float>(i) / static_cast<float>(slices);
        const float u1 = static_cast<float>(i + 1) / static_cast<float>(slices);
        const float a0 = TAU() * u0;
        const float a1 = TAU() * u1;
        const M::Point3D p0(std::cos(a0) * radius, -halfH, std::sin(a0) * radius);
        const M::Point3D p1(std::cos(a1) * radius, -halfH, std::sin(a1) * radius);
        const int base = static_cast<int>(mesh.vertices.size());
        mesh.vertices.push_back({p0, M::Vector3D(0.0f, -1.0f, 0.0f), M::Vector2D(0.0f, 0.0f)});
        mesh.vertices.push_back({p1, M::Vector3D(0.0f, -1.0f, 0.0f), M::Vector2D(1.0f, 0.0f)});
        mesh.triangles.push_back({center, base + 1, base + 0});
    }

    return mesh;
}

// =========================================================
// Mesh factories 2D
// =========================================================

Mesh2D makeTriangle2D(const M::Point2D& a,
                      const M::Point2D& b,
                      const M::Point2D& c) noexcept {
    Mesh2D mesh;
    mesh.vertices = {
        {a, M::Vector2D(0.0f, 0.0f), Color::white()},
        {b, M::Vector2D(1.0f, 0.0f), Color::white()},
        {c, M::Vector2D(0.5f, 1.0f), Color::white()}
    };
    mesh.triangles = {{0, 1, 2}};
    return mesh;
}

Mesh2D makeQuad2D(float width, float height) noexcept {
    return makeRectangle2D(M::Vector2D(width, height));
}

Mesh2D makeRectangle2D(const M::Vector2D& size) noexcept {
    Mesh2D mesh;
    const float hx = size.v * 0.5f;
    const float hy = size.w * 0.5f;
    mesh.vertices = {
        {M::Point2D(-hx, -hy), M::Vector2D(0.0f, 1.0f), Color::white()},
        {M::Point2D( hx, -hy), M::Vector2D(1.0f, 1.0f), Color::white()},
        {M::Point2D( hx,  hy), M::Vector2D(1.0f, 0.0f), Color::white()},
        {M::Point2D(-hx,  hy), M::Vector2D(0.0f, 0.0f), Color::white()}
    };
    mesh.triangles = {{0, 1, 2}, {0, 2, 3}};
    return mesh;
}

Mesh2D makeCircle2D(float radius, int segments) noexcept {
    Mesh2D mesh;
    segments = std::max(3, segments);
    mesh.vertices.reserve(static_cast<std::size_t>(segments + 1));
    mesh.vertices.push_back({M::Point2D(0.0f, 0.0f), M::Vector2D(0.5f, 0.5f), Color::white()});
    for (int i = 0; i < segments; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(segments);
        const float a = TAU() * t;
        const float x = std::cos(a) * radius;
        const float y = std::sin(a) * radius;
        mesh.vertices.push_back({M::Point2D(x, y), M::Vector2D((x / radius + 1.0f) * 0.5f, (y / radius + 1.0f) * 0.5f), Color::white()});
    }
    for (int i = 0; i < segments; ++i) {
        const int i0 = 0;
        const int i1 = i + 1;
        const int i2 = (i + 1) % segments + 1;
        mesh.triangles.push_back({i0, i1, i2});
    }
    return mesh;
}

Mesh2D makePolygon2D(const std::vector<M::Point2D>& points) noexcept {
    Mesh2D mesh;
    if (points.size() < 3) return mesh;
    mesh.vertices.reserve(points.size());
    for (const auto& p : points) {
        mesh.vertices.push_back({p, M::Vector2D(0.0f, 0.0f), Color::white()});
    }
    for (std::size_t i = 1; i + 1 < points.size(); ++i) {
        mesh.triangles.push_back({0, static_cast<int>(i), static_cast<int>(i + 1)});
    }
    return mesh;
}

// =========================================================
// Mesh utility helpers
// =========================================================

M::Vector3D computeFaceNormal(const Mesh& mesh,
                              const Triangle& triangle) noexcept {
    if (!mesh.valid()) return M::Vector3D();
    const auto& a = mesh.vertices[static_cast<std::size_t>(triangle.a)].position;
    const auto& b = mesh.vertices[static_cast<std::size_t>(triangle.b)].position;
    const auto& c = mesh.vertices[static_cast<std::size_t>(triangle.c)].position;
    return ((b - a).cross(c - a)).normalized();
}

M::Point3D computeTriangleCentroid(const Mesh& mesh,
                                   const Triangle& triangle) noexcept {
    if (!mesh.valid()) return M::Point3D();
    return ::triangleCentroid(mesh, triangle);
}

M::AABB3D computeBounds(const Mesh& mesh) noexcept {
    if (mesh.vertices.empty()) return M::AABB3D();
    M::AABB3D box(mesh.vertices.front().position, mesh.vertices.front().position);
    for (const auto& v : mesh.vertices) {
        box.expandToInclude(v.position);
    }
    return box;
}

M::AABB2D computeBounds(const Mesh2D& mesh) noexcept {
    if (mesh.vertices.empty()) return M::AABB2D();
    M::AABB2D box(mesh.vertices.front().position, mesh.vertices.front().position);
    for (const auto& v : mesh.vertices) {
        box.expandToInclude(v.position);
    }
    return box;
}

bool validateMesh(const Mesh& mesh) noexcept {
    return mesh.valid();
}

bool validateMesh(const Mesh2D& mesh) noexcept {
    return mesh.valid();
}

void recomputeFlatNormals(Mesh& mesh) noexcept {
    if (!mesh.valid()) return;
    for (auto& v : mesh.vertices) {
        v.normal = M::Vector3D();
    }
    for (const auto& t : mesh.triangles) {
        const M::Vector3D n = computeFaceNormal(mesh, t);
        mesh.vertices[static_cast<std::size_t>(t.a)].normal = n;
        mesh.vertices[static_cast<std::size_t>(t.b)].normal = n;
        mesh.vertices[static_cast<std::size_t>(t.c)].normal = n;
    }
}

void recomputeSmoothNormals(Mesh& mesh) noexcept {
    if (!mesh.valid()) return;
    for (auto& v : mesh.vertices) {
        v.normal = M::Vector3D();
    }
    for (const auto& t : mesh.triangles) {
        const auto& a = mesh.vertices[static_cast<std::size_t>(t.a)].position;
        const auto& b = mesh.vertices[static_cast<std::size_t>(t.b)].position;
        const auto& c = mesh.vertices[static_cast<std::size_t>(t.c)].position;
        const M::Vector3D n = (b - a).cross(c - a);
        mesh.vertices[static_cast<std::size_t>(t.a)].normal += n;
        mesh.vertices[static_cast<std::size_t>(t.b)].normal += n;
        mesh.vertices[static_cast<std::size_t>(t.c)].normal += n;
    }
    for (auto& v : mesh.vertices) {
        v.normal.normalize();
    }
}

} // namespace G
