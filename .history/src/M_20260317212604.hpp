#ifndef M_HPP
#define M_HPP

#include <cmath>
#include <limits>

namespace M {
    inline constexpr float EPS = 1e-6f;

    class Vector2D {
    public:
        float v;
        float w;
        constexpr Vector2D(): v(0), w(0) {}
        constexpr Vector2D(float _v, float _w): v(_v), w(_w) {}
        [[nodiscard]] float magnitude() const noexcept;
        [[nodiscard]] float angle() const noexcept;
        [[nodiscard]] bool operator==(const Vector2D &other) const noexcept {
            return std::fabs(v - other.v) < EPS && std::fabs(w - other.w) < EPS;
        }
        [[nodiscard]] bool operator!=(const Vector2D &other) const noexcept {
            return !(*this == other);
        }
        [[nodiscard]] constexpr Vector2D operator+(const Vector2D &other) const noexcept {
            return Vector2D(v + other.v, w + other.w);
        }
        [[nodiscard]] constexpr Vector2D operator-(const Vector2D &other) const noexcept {
            return Vector2D(v - other.v, w - other.w);
        }
        [[nodiscard]] constexpr Vector2D operator-() const noexcept {
            return Vector2D(-v, -w);
        }
        Vector2D &operator+=(const Vector2D &other) noexcept;
        Vector2D &operator-=(const Vector2D &other) noexcept;
        [[nodiscard]] constexpr Vector2D operator*(float x) const noexcept {
            return Vector2D(v * x, w * x);
        }
        [[nodiscard]] Vector2D operator/(float x) const noexcept {
            if (x > -EPS && x < EPS) {
                return Vector2D(std::numeric_limits<float>::quiet_NaN(),
                                std::numeric_limits<float>::quiet_NaN());
            }
            return Vector2D(v / x, w / x);
        }
        Vector2D &operator*=(float x) noexcept;
        Vector2D &operator/=(float x) noexcept;
        [[nodiscard]] Vector2D atMagnitude(float mag) const noexcept;
        Vector2D &setMagnitude(float mag) noexcept;
        [[nodiscard]] float angleBetween(const Vector2D &other) const noexcept;
        [[nodiscard]] Vector2D negated() const noexcept;
        Vector2D &negate() noexcept;
        [[nodiscard]] float magnitudeSqr() const noexcept;
        [[nodiscard]] Vector2D normalized() const noexcept;
        Vector2D &normalize() noexcept;
        [[nodiscard]] float dot(const Vector2D& other) const noexcept;
        [[nodiscard]] float cross(const Vector2D& other) const noexcept;
        [[nodiscard]] Vector2D rotated(float angle) const noexcept;
        Vector2D &rotate(float angle) noexcept;
        [[nodiscard]] Vector2D perpendicular() const noexcept;
        Vector2D &makePerpendicular() noexcept;
        [[nodiscard]] bool isParallel(const Vector2D &other) const noexcept;
        [[nodiscard]] bool isPerpendicular(const Vector2D &other) const noexcept;
    };

    class Point2D {
    public:
        float v;
        float w;
        constexpr Point2D() : v(0), w(0) {}
        constexpr Point2D(float _v, float _w) : v(_v), w(_w) {}
        Point2D& operator=(const Point2D& other) = default;
        [[nodiscard]] bool operator==(const Point2D& other) const noexcept {
            return std::fabs(v - other.v) < EPS && std::fabs(w - other.w) < EPS;
        }
        [[nodiscard]] bool operator!=(const Point2D& other) const noexcept {
            return !(*this == other);
        }
        [[nodiscard]] constexpr Point2D operator+(const Vector2D& other) const noexcept {
            return Point2D(v + other.v, w + other.w);
        }
        [[nodiscard]] constexpr Point2D operator-(const Vector2D& other) const noexcept {
            return Point2D(v - other.v, w - other.w);
        }
        [[nodiscard]] constexpr Vector2D operator-(const Point2D& other) const noexcept {
            return Vector2D(v - other.v, w - other.w);
        }
        Point2D& operator+=(const Vector2D& other) noexcept;
        Point2D& operator-=(const Vector2D& other) noexcept;
        [[nodiscard]] float distanceTo(const Point2D& other) const noexcept;
        [[nodiscard]] float distanceSqrTo(const Point2D& other) const noexcept;
    };

    class Vector3D {
    public:
        float x;
        float y;
        float z;

        constexpr Vector3D() : x(0), y(0), z(0) {}
        constexpr Vector3D(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

        [[nodiscard]] float magnitude() const noexcept;
        [[nodiscard]] float magnitudeSqr() const noexcept;

        [[nodiscard]] bool operator==(const Vector3D& other) const noexcept {
            return std::fabs(x - other.x) < EPS &&
                   std::fabs(y - other.y) < EPS &&
                   std::fabs(z - other.z) < EPS;
        }

        [[nodiscard]] bool operator!=(const Vector3D& other) const noexcept {
            return !(*this == other);
        }

        [[nodiscard]] constexpr Vector3D operator+(const Vector3D& other) const noexcept {
            return Vector3D(x + other.x, y + other.y, z + other.z);
        }

        [[nodiscard]] constexpr Vector3D operator-(const Vector3D& other) const noexcept {
            return Vector3D(x - other.x, y - other.y, z - other.z);
        }

        [[nodiscard]] constexpr Vector3D operator-() const noexcept {
            return Vector3D(-x, -y, -z);
        }

        Vector3D& operator+=(const Vector3D& other) noexcept;
        Vector3D& operator-=(const Vector3D& other) noexcept;

        [[nodiscard]] constexpr Vector3D operator*(float k) const noexcept {
            return Vector3D(x * k, y * k, z * k);
        }

        [[nodiscard]] Vector3D operator/(float k) const noexcept {
            if (k > -EPS && k < EPS) {
                return Vector3D(
                    std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN()
                );
            }
            return Vector3D(x / k, y / k, z / k);
        }

        Vector3D& operator*=(float k) noexcept;
        Vector3D& operator/=(float k) noexcept;

        [[nodiscard]] Vector3D atMagnitude(float mag) const noexcept;
        Vector3D& setMagnitude(float mag) noexcept;

        [[nodiscard]] Vector3D negated() const noexcept;
        Vector3D& negate() noexcept;

        [[nodiscard]] Vector3D normalized() const noexcept;
        Vector3D& normalize() noexcept;

        [[nodiscard]] float dot(const Vector3D& other) const noexcept;
        [[nodiscard]] Vector3D cross(const Vector3D& other) const noexcept;

        [[nodiscard]] bool isParallel(const Vector3D& other) const noexcept;
        [[nodiscard]] bool isPerpendicular(const Vector3D& other) const noexcept;
    };

    class Point3D {
    public:
        float x;
        float y;
        float z;

        constexpr Point3D() : x(0), y(0), z(0) {}
        constexpr Point3D(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

        Point3D& operator=(const Point3D& other) = default;

        [[nodiscard]] bool operator==(const Point3D& other) const noexcept {
            return std::fabs(x - other.x) < EPS &&
                   std::fabs(y - other.y) < EPS &&
                   std::fabs(z - other.z) < EPS;
        }

        [[nodiscard]] bool operator!=(const Point3D& other) const noexcept {
            return !(*this == other);
        }

        [[nodiscard]] constexpr Point3D operator+(const Vector3D& other) const noexcept {
            return Point3D(x + other.x, y + other.y, z + other.z);
        }

        [[nodiscard]] constexpr Point3D operator-(const Vector3D& other) const noexcept {
            return Point3D(x - other.x, y - other.y, z - other.z);
        }

        [[nodiscard]] constexpr Vector3D operator-(const Point3D& other) const noexcept {
            return Vector3D(x - other.x, y - other.y, z - other.z);
        }

        Point3D& operator+=(const Vector3D& other) noexcept;
        Point3D& operator-=(const Vector3D& other) noexcept;

        [[nodiscard]] float distanceTo(const Point3D& other) const noexcept;
        [[nodiscard]] float distanceSqrTo(const Point3D& other) const noexcept;
    };

    // =========================
    // Vector4D
    // =========================

    class Vector4D {
    public:
        float x;
        float y;
        float z;
        float w;

        constexpr Vector4D() : x(0), y(0), z(0), w(0) {}
        constexpr Vector4D(float _x, float _y, float _z, float _w)
            : x(_x), y(_y), z(_z), w(_w) {}
        constexpr Vector4D(const Vector3D& xyz, float _w)
            : x(xyz.x), y(xyz.y), z(xyz.z), w(_w) {}

        [[nodiscard]] float magnitude() const noexcept;
        [[nodiscard]] float magnitudeSqr() const noexcept;

        [[nodiscard]] bool operator==(const Vector4D& other) const noexcept {
            return std::fabs(x - other.x) < EPS &&
                   std::fabs(y - other.y) < EPS &&
                   std::fabs(z - other.z) < EPS &&
                   std::fabs(w - other.w) < EPS;
        }

        [[nodiscard]] bool operator!=(const Vector4D& other) const noexcept {
            return !(*this == other);
        }

        [[nodiscard]] constexpr Vector4D operator+(const Vector4D& other) const noexcept {
            return Vector4D(x + other.x, y + other.y, z + other.z, w + other.w);
        }

        [[nodiscard]] constexpr Vector4D operator-(const Vector4D& other) const noexcept {
            return Vector4D(x - other.x, y - other.y, z - other.z, w - other.w);
        }

        [[nodiscard]] constexpr Vector4D operator-() const noexcept {
            return Vector4D(-x, -y, -z, -w);
        }

        Vector4D& operator+=(const Vector4D& other) noexcept;
        Vector4D& operator-=(const Vector4D& other) noexcept;

        [[nodiscard]] constexpr Vector4D operator*(float k) const noexcept {
            return Vector4D(x * k, y * k, z * k, w * k);
        }

        [[nodiscard]] Vector4D operator/(float k) const noexcept {
            if (k > -EPS && k < EPS) {
                return Vector4D(
                    std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN()
                );
            }
            return Vector4D(x / k, y / k, z / k, w / k);
        }

        Vector4D& operator*=(float k) noexcept;
        Vector4D& operator/=(float k) noexcept;

        [[nodiscard]] Vector4D normalized() const noexcept;
        Vector4D& normalize() noexcept;

        [[nodiscard]] float dot(const Vector4D& other) const noexcept;
        [[nodiscard]] Vector4D atMagnitude(float mag) const noexcept;
        Vector4D& setMagnitude(float mag) noexcept;
    };

    // =========================
    // Matrix3x3
    // Row-major
    // =========================

    class Matrix3x3 {
    public:
        float m[3][3];

        constexpr Matrix3x3()
            : m{{1,0,0},{0,1,0},{0,0,1}} {}

        constexpr Matrix3x3(float m00, float m01, float m02,
                            float m10, float m11, float m12,
                            float m20, float m21, float m22)
            : m{{m00,m01,m02},{m10,m11,m12},{m20,m21,m22}} {}

        [[nodiscard]] static Matrix3x3 identity() noexcept;
        [[nodiscard]] static Matrix3x3 zero() noexcept;
        [[nodiscard]] static Matrix3x3 scaling(float sv, float sw) noexcept;
        [[nodiscard]] static Matrix3x3 rotation(float angle) noexcept;
        [[nodiscard]] static Matrix3x3 translation(float tv, float tw) noexcept;

        [[nodiscard]] bool operator==(const Matrix3x3& other) const noexcept;
        [[nodiscard]] bool operator!=(const Matrix3x3& other) const noexcept;

        [[nodiscard]] constexpr const float* operator[](int row) const noexcept { return m[row]; }
        [[nodiscard]] constexpr float* operator[](int row) noexcept { return m[row]; }

        [[nodiscard]] Matrix3x3 operator+(const Matrix3x3& other) const noexcept;
        [[nodiscard]] Matrix3x3 operator-(const Matrix3x3& other) const noexcept;
        [[nodiscard]] Matrix3x3 operator*(const Matrix3x3& other) const noexcept;
        [[nodiscard]] Matrix3x3 operator*(float k) const noexcept;
        Matrix3x3& operator+=(const Matrix3x3& other) noexcept;
        Matrix3x3& operator-=(const Matrix3x3& other) noexcept;
        Matrix3x3& operator*=(const Matrix3x3& other) noexcept;
        Matrix3x3& operator*=(float k) noexcept;

        [[nodiscard]] Vector3D operator*(const Vector3D& vec) const noexcept;
        [[nodiscard]] float determinant() const noexcept;
        [[nodiscard]] Matrix3x3 transposed() const noexcept;
        Matrix3x3& transpose() noexcept;
        [[nodiscard]] Matrix3x3 inverted() const noexcept;
        Matrix3x3& invert() noexcept;
    };

    // =========================
    // Matrix4x4
    // Row-major
    // =========================

    class Matrix4x4 {
    public:
        float m[4][4];

        constexpr Matrix4x4()
            : m{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}} {}

        constexpr Matrix4x4(float m00, float m01, float m02, float m03,
                            float m10, float m11, float m12, float m13,
                            float m20, float m21, float m22, float m23,
                            float m30, float m31, float m32, float m33)
            : m{{m00,m01,m02,m03},{m10,m11,m12,m13},{m20,m21,m22,m23},{m30,m31,m32,m33}} {}

        [[nodiscard]] static Matrix4x4 identity() noexcept;
        [[nodiscard]] static Matrix4x4 zero() noexcept;

        [[nodiscard]] static Matrix4x4 translation(float tx, float ty, float tz) noexcept;
        [[nodiscard]] static Matrix4x4 scaling(float sx, float sy, float sz) noexcept;
        [[nodiscard]] static Matrix4x4 rotationX(float angle) noexcept;
        [[nodiscard]] static Matrix4x4 rotationY(float angle) noexcept;
        [[nodiscard]] static Matrix4x4 rotationZ(float angle) noexcept;

        [[nodiscard]] bool operator==(const Matrix4x4& other) const noexcept;
        [[nodiscard]] bool operator!=(const Matrix4x4& other) const noexcept;

        [[nodiscard]] constexpr const float* operator[](int row) const noexcept { return m[row]; }
        [[nodiscard]] constexpr float* operator[](int row) noexcept { return m[row]; }

        [[nodiscard]] Matrix4x4 operator+(const Matrix4x4& other) const noexcept;
        [[nodiscard]] Matrix4x4 operator-(const Matrix4x4& other) const noexcept;
        [[nodiscard]] Matrix4x4 operator*(const Matrix4x4& other) const noexcept;
        [[nodiscard]] Matrix4x4 operator*(float k) const noexcept;
        Matrix4x4& operator+=(const Matrix4x4& other) noexcept;
        Matrix4x4& operator-=(const Matrix4x4& other) noexcept;
        Matrix4x4& operator*=(const Matrix4x4& other) noexcept;
        Matrix4x4& operator*=(float k) noexcept;

        [[nodiscard]] Vector4D operator*(const Vector4D& vec) const noexcept;
        [[nodiscard]] float determinant() const noexcept;
        [[nodiscard]] Matrix4x4 transposed() const noexcept;
        Matrix4x4& transpose() noexcept;
        [[nodiscard]] Matrix4x4 inverted() const noexcept;
        Matrix4x4& invert() noexcept;
    };

    // =========================
    // 2D geometry
    // =========================

    class Line2D {
    public:
        Point2D point;
        Vector2D dir;

        constexpr Line2D() : point(), dir(1.0f, 0.0f) {}
        constexpr Line2D(const Point2D& _point, const Vector2D& _dir)
            : point(_point), dir(_dir) {}
        Line2D(const Point2D& a, const Point2D& b) noexcept;
        Line2D(const Point2D& center, float angle) noexcept;

        [[nodiscard]] bool isValid() const noexcept;
        [[nodiscard]] Point2D pointAt(float t) const noexcept;
        [[nodiscard]] bool contains(const Point2D& p) const noexcept;
        [[nodiscard]] bool isParallel(const Line2D& other) const noexcept;
        [[nodiscard]] bool isPerpendicular(const Line2D& other) const noexcept;
        [[nodiscard]] Point2D projected(const Point2D& p) const noexcept;
        [[nodiscard]] float distanceTo(const Point2D& p) const noexcept;
        [[nodiscard]] float distanceSqrTo(const Point2D& p) const noexcept;
    };

    class Ray2D {
    public:
        Point2D origin;
        Vector2D dir;

        constexpr Ray2D() : origin(), dir(1.0f, 0.0f) {}
        constexpr Ray2D(const Point2D& _origin, const Vector2D& _dir)
            : origin(_origin), dir(_dir) {}
        Ray2D(const Point2D& origin_, const Point2D& through) noexcept;
        Ray2D(const Point2D& origin_, float angle) noexcept;

        [[nodiscard]] bool isValid() const noexcept;
        [[nodiscard]] Point2D pointAt(float t) const noexcept;
        [[nodiscard]] bool contains(const Point2D& p) const noexcept;
        [[nodiscard]] Point2D projected(const Point2D& p) const noexcept;
        [[nodiscard]] float distanceTo(const Point2D& p) const noexcept;
        [[nodiscard]] float distanceSqrTo(const Point2D& p) const noexcept;
    };

    class Segment2D {
    public:
        Point2D start;
        Point2D end;

        constexpr Segment2D() : start(), end() {}
        constexpr Segment2D(const Point2D& _start, const Point2D& _end)
            : start(_start), end(_end) {}

        [[nodiscard]] Vector2D direction() const noexcept;
        [[nodiscard]] float length() const noexcept;
        [[nodiscard]] float lengthSqr() const noexcept;
        [[nodiscard]] Point2D midpoint() const noexcept;
        [[nodiscard]] Point2D pointAt(float t) const noexcept;
        [[nodiscard]] bool contains(const Point2D& p) const noexcept;
        [[nodiscard]] Point2D projected(const Point2D& p) const noexcept;
        [[nodiscard]] float distanceTo(const Point2D& p) const noexcept;
        [[nodiscard]] float distanceSqrTo(const Point2D& p) const noexcept;
        [[nodiscard]] bool isDegenerate() const noexcept;
    };

    // =========================
    // 3D geometry
    // =========================

    class Line3D {
    public:
        Point3D point;
        Vector3D dir;

        constexpr Line3D() : point(), dir(1.0f, 0.0f, 0.0f) {}
        constexpr Line3D(const Point3D& _point, const Vector3D& _dir)
            : point(_point), dir(_dir) {}
        Line3D(const Point3D& a, const Point3D& b) noexcept;

        [[nodiscard]] bool isValid() const noexcept;
        [[nodiscard]] Point3D pointAt(float t) const noexcept;
        [[nodiscard]] bool contains(const Point3D& p) const noexcept;
        [[nodiscard]] bool isParallel(const Line3D& other) const noexcept;
        [[nodiscard]] bool isPerpendicular(const Line3D& other) const noexcept;
        [[nodiscard]] Point3D projected(const Point3D& p) const noexcept;
        [[nodiscard]] float distanceTo(const Point3D& p) const noexcept;
        [[nodiscard]] float distanceSqrTo(const Point3D& p) const noexcept;
    };

    class Ray3D {
    public:
        Point3D origin;
        Vector3D dir;

        constexpr Ray3D() : origin(), dir(1.0f, 0.0f, 0.0f) {}
        constexpr Ray3D(const Point3D& _origin, const Vector3D& _dir)
            : origin(_origin), dir(_dir) {}
        Ray3D(const Point3D& origin_, const Point3D& through) noexcept;

        [[nodiscard]] bool isValid() const noexcept;
        [[nodiscard]] Point3D pointAt(float t) const noexcept;
        [[nodiscard]] bool contains(const Point3D& p) const noexcept;
        [[nodiscard]] Point3D projected(const Point3D& p) const noexcept;
        [[nodiscard]] float distanceTo(const Point3D& p) const noexcept;
        [[nodiscard]] float distanceSqrTo(const Point3D& p) const noexcept;
    };

    class Segment3D {
    public:
        Point3D start;
        Point3D end;

        constexpr Segment3D() : start(), end() {}
        constexpr Segment3D(const Point3D& _start, const Point3D& _end)
            : start(_start), end(_end) {}

        [[nodiscard]] Vector3D direction() const noexcept;
        [[nodiscard]] float length() const noexcept;
        [[nodiscard]] float lengthSqr() const noexcept;
        [[nodiscard]] Point3D midpoint() const noexcept;
        [[nodiscard]] Point3D pointAt(float t) const noexcept;
        [[nodiscard]] bool contains(const Point3D& p) const noexcept;
        [[nodiscard]] Point3D projected(const Point3D& p) const noexcept;
        [[nodiscard]] float distanceTo(const Point3D& p) const noexcept;
        [[nodiscard]] float distanceSqrTo(const Point3D& p) const noexcept;
        [[nodiscard]] bool isDegenerate() const noexcept;
    };

    class Plane3D {
    public:
        Vector3D normal;
        float d;

        constexpr Plane3D() : normal(0.0f, 1.0f, 0.0f), d(0.0f) {}
        constexpr Plane3D(const Vector3D& _normal, float _d)
            : normal(_normal), d(_d) {}
        Plane3D(const Vector3D& normal_, const Point3D& point) noexcept;
        Plane3D(const Point3D& a, const Point3D& b, const Point3D& c) noexcept;

        [[nodiscard]] bool isValid() const noexcept;
        [[nodiscard]] float signedDistanceTo(const Point3D& p) const noexcept;
        [[nodiscard]] float distanceTo(const Point3D& p) const noexcept;
        [[nodiscard]] bool contains(const Point3D& p) const noexcept;
        [[nodiscard]] Point3D projected(const Point3D& p) const noexcept;
        [[nodiscard]] Plane3D normalized() const noexcept;
        Plane3D& normalize() noexcept;
    };

        // =========================
    // Utility scalar helpers
    // =========================

    [[nodiscard]] float clamp(float x, float lo, float hi) noexcept;
    [[nodiscard]] float lerp(float a, float b, float t) noexcept;
    [[nodiscard]] float inverseLerp(float a, float b, float value) noexcept;
    [[nodiscard]] float remap(float inA, float inB, float outA, float outB, float value) noexcept;
    [[nodiscard]] float radiansToDegrees(float radians) noexcept;
    [[nodiscard]] float degreesToRadians(float degrees) noexcept;
    [[nodiscard]] bool nearlyEqual(float a, float b, float eps = EPS) noexcept;

    // =========================
    // Transform2D
    // =========================

    class Transform2D {
    public:
        Point2D position;
        float rotation;
        Vector2D scale;

        constexpr Transform2D()
            : position(), rotation(0.0f), scale(1.0f, 1.0f) {}

        constexpr Transform2D(const Point2D& _position, float _rotation, const Vector2D& _scale)
            : position(_position), rotation(_rotation), scale(_scale) {}

        [[nodiscard]] Matrix3x3 matrix() const noexcept;
        [[nodiscard]] Point2D appliedTo(const Point2D& p) const noexcept;
        [[nodiscard]] Vector2D appliedTo(const Vector2D& v) const noexcept;
        [[nodiscard]] Transform2D combinedWith(const Transform2D& other) const noexcept;
    };

    // =========================
    // Quaternion
    // =========================

    class Quaternion {
    public:
        float x;
        float y;
        float z;
        float w;

        constexpr Quaternion() : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {}
        constexpr Quaternion(float _x, float _y, float _z, float _w)
            : x(_x), y(_y), z(_z), w(_w) {}

        [[nodiscard]] float magnitude() const noexcept;
        [[nodiscard]] float magnitudeSqr() const noexcept;
        [[nodiscard]] Quaternion normalized() const noexcept;
        Quaternion& normalize() noexcept;
        [[nodiscard]] Quaternion conjugated() const noexcept;
        Quaternion& conjugate() noexcept;
        [[nodiscard]] Quaternion inverted() const noexcept;
        Quaternion& invert() noexcept;

        [[nodiscard]] bool operator==(const Quaternion& other) const noexcept;
        [[nodiscard]] bool operator!=(const Quaternion& other) const noexcept;

        [[nodiscard]] constexpr Quaternion operator+(const Quaternion& other) const noexcept {
            return Quaternion(x + other.x, y + other.y, z + other.z, w + other.w);
        }

        [[nodiscard]] constexpr Quaternion operator-(const Quaternion& other) const noexcept {
            return Quaternion(x - other.x, y - other.y, z - other.z, w - other.w);
        }

        [[nodiscard]] constexpr Quaternion operator*(float k) const noexcept {
            return Quaternion(x * k, y * k, z * k, w * k);
        }

        [[nodiscard]] Quaternion operator/(float k) const noexcept;
        [[nodiscard]] Quaternion operator*(const Quaternion& other) const noexcept;

        Quaternion& operator+=(const Quaternion& other) noexcept;
        Quaternion& operator-=(const Quaternion& other) noexcept;
        Quaternion& operator*=(float k) noexcept;
        Quaternion& operator/=(float k) noexcept;
        Quaternion& operator*=(const Quaternion& other) noexcept;

        [[nodiscard]] float dot(const Quaternion& other) const noexcept;
        [[nodiscard]] Matrix4x4 toMatrix4x4() const noexcept;
        [[nodiscard]] Vector3D rotatedVector(const Vector3D& v) const noexcept;

        [[nodiscard]] static Quaternion identity() noexcept;
        [[nodiscard]] static Quaternion fromAxisAngle(const Vector3D& axis, float angle) noexcept;
    };

    // =========================
    // Transform3D
    // =========================

    class Transform3D {
    public:
        Point3D position;
        Quaternion rotation;
        Vector3D scale;

        constexpr Transform3D()
            : position(), rotation(), scale(1.0f, 1.0f, 1.0f) {}

        constexpr Transform3D(const Point3D& _position, const Quaternion& _rotation, const Vector3D& _scale)
            : position(_position), rotation(_rotation), scale(_scale) {}

        [[nodiscard]] Matrix4x4 matrix() const noexcept;
        [[nodiscard]] Point3D appliedTo(const Point3D& p) const noexcept;
        [[nodiscard]] Vector3D appliedTo(const Vector3D& v) const noexcept;
        [[nodiscard]] Transform3D combinedWith(const Transform3D& other) const noexcept;
    };

    // =========================
    // Bounds / volumes
    // =========================

    class AABB2D {
    public:
        Point2D min;
        Point2D max;

        constexpr AABB2D() : min(), max() {}
        constexpr AABB2D(const Point2D& _min, const Point2D& _max) : min(_min), max(_max) {}

        [[nodiscard]] bool isValid() const noexcept;
        [[nodiscard]] Point2D center() const noexcept;
        [[nodiscard]] Vector2D size() const noexcept;
        [[nodiscard]] float area() const noexcept;
        [[nodiscard]] bool contains(const Point2D& p) const noexcept;
        [[nodiscard]] bool overlaps(const AABB2D& other) const noexcept;
        [[nodiscard]] Point2D clampedPoint(const Point2D& p) const noexcept;
        [[nodiscard]] float distanceTo(const Point2D& p) const noexcept;
        [[nodiscard]] float distanceSqrTo(const Point2D& p) const noexcept;
        void expandToInclude(const Point2D& p) noexcept;
        void expandToInclude(const AABB2D& other) noexcept;
    };

    class AABB3D {
    public:
        Point3D min;
        Point3D max;

        constexpr AABB3D() : min(), max() {}
        constexpr AABB3D(const Point3D& _min, const Point3D& _max) : min(_min), max(_max) {}

        [[nodiscard]] bool isValid() const noexcept;
        [[nodiscard]] Point3D center() const noexcept;
        [[nodiscard]] Vector3D size() const noexcept;
        [[nodiscard]] float volume() const noexcept;
        [[nodiscard]] bool contains(const Point3D& p) const noexcept;
        [[nodiscard]] bool overlaps(const AABB3D& other) const noexcept;
        [[nodiscard]] Point3D clampedPoint(const Point3D& p) const noexcept;
        [[nodiscard]] float distanceTo(const Point3D& p) const noexcept;
        [[nodiscard]] float distanceSqrTo(const Point3D& p) const noexcept;
        void expandToInclude(const Point3D& p) noexcept;
        void expandToInclude(const AABB3D& other) noexcept;
    };

    class Circle {
    public:
        Point2D center;
        float radius;

        constexpr Circle() : center(), radius(0.0f) {}
        constexpr Circle(const Point2D& _center, float _radius) : center(_center), radius(_radius) {}

        [[nodiscard]] bool isValid() const noexcept;
        [[nodiscard]] float area() const noexcept;
        [[nodiscard]] float circumference() const noexcept;
        [[nodiscard]] bool contains(const Point2D& p) const noexcept;
        [[nodiscard]] bool overlaps(const Circle& other) const noexcept;
        [[nodiscard]] Point2D clampedPoint(const Point2D& p) const noexcept;
        [[nodiscard]] float distanceTo(const Point2D& p) const noexcept;
        [[nodiscard]] float distanceSqrTo(const Point2D& p) const noexcept;
    };

    class Sphere {
    public:
        Point3D center;
        float radius;

        constexpr Sphere() : center(), radius(0.0f) {}
        constexpr Sphere(const Point3D& _center, float _radius) : center(_center), radius(_radius) {}

        [[nodiscard]] bool isValid() const noexcept;
        [[nodiscard]] float volume() const noexcept;
        [[nodiscard]] float surfaceArea() const noexcept;
        [[nodiscard]] bool contains(const Point3D& p) const noexcept;
        [[nodiscard]] bool overlaps(const Sphere& other) const noexcept;
        [[nodiscard]] Point3D clampedPoint(const Point3D& p) const noexcept;
        [[nodiscard]] float distanceTo(const Point3D& p) const noexcept;
        [[nodiscard]] float distanceSqrTo(const Point3D& p) const noexcept;
    };

    // =========================
    // Triangles
    // =========================

    class Triangle2D {
    public:
        Point2D a;
        Point2D b;
        Point2D c;

        constexpr Triangle2D() : a(), b(), c() {}
        constexpr Triangle2D(const Point2D& _a, const Point2D& _b, const Point2D& _c)
            : a(_a), b(_b), c(_c) {}

        [[nodiscard]] float signedArea() const noexcept;
        [[nodiscard]] float area() const noexcept;
        [[nodiscard]] Point2D centroid() const noexcept;
        [[nodiscard]] bool contains(const Point2D& p) const noexcept;
        [[nodiscard]] bool isDegenerate() const noexcept;
    };

    class Triangle3D {
    public:
        Point3D a;
        Point3D b;
        Point3D c;

        constexpr Triangle3D() : a(), b(), c() {}
        constexpr Triangle3D(const Point3D& _a, const Point3D& _b, const Point3D& _c)
            : a(_a), b(_b), c(_c) {}

        [[nodiscard]] Vector3D normal() const noexcept;
        [[nodiscard]] float area() const noexcept;
        [[nodiscard]] Point3D centroid() const noexcept;
        [[nodiscard]] bool isDegenerate() const noexcept;
        [[nodiscard]] Plane3D plane() const noexcept;
    };

    // =========================
    // Hit / query results
    // =========================

    struct RaycastHit2D {
        bool hit;
        float t;
        Point2D point;
        Vector2D normal;
    };

    struct RaycastHit3D {
        bool hit;
        float t;
        Point3D point;
        Vector3D normal;
    };

    // =========================
    // Closest-point helpers
    // =========================

    [[nodiscard]] Point2D closestPoint(const Line2D& line, const Point2D& p) noexcept;
    [[nodiscard]] Point2D closestPoint(const Ray2D& ray, const Point2D& p) noexcept;
    [[nodiscard]] Point2D closestPoint(const Segment2D& segment, const Point2D& p) noexcept;

    [[nodiscard]] Point3D closestPoint(const Line3D& line, const Point3D& p) noexcept;
    [[nodiscard]] Point3D closestPoint(const Ray3D& ray, const Point3D& p) noexcept;
    [[nodiscard]] Point3D closestPoint(const Segment3D& segment, const Point3D& p) noexcept;
    [[nodiscard]] Point3D closestPoint(const Plane3D& plane, const Point3D& p) noexcept;

    // =========================
    // Intersection / overlap tests
    // =========================

    [[nodiscard]] bool intersects(const AABB2D& a, const AABB2D& b) noexcept;
    [[nodiscard]] bool intersects(const AABB3D& a, const AABB3D& b) noexcept;
    [[nodiscard]] bool intersects(const Circle& a, const Circle& b) noexcept;
    [[nodiscard]] bool intersects(const Sphere& a, const Sphere& b) noexcept;

    [[nodiscard]] bool intersects(const Line2D& a, const Line2D& b) noexcept;
    [[nodiscard]] bool intersects(const Ray2D& ray, const Segment2D& segment) noexcept;
    [[nodiscard]] bool intersects(const Segment2D& a, const Segment2D& b) noexcept;

    [[nodiscard]] bool intersects(const Line3D& line, const Plane3D& plane) noexcept;
    [[nodiscard]] bool intersects(const Ray3D& ray, const Plane3D& plane) noexcept;
    [[nodiscard]] bool intersects(const Ray3D& ray, const Sphere& sphere) noexcept;
    [[nodiscard]] bool intersects(const Ray3D& ray, const AABB3D& box) noexcept;

    // =========================
    // Raycast helpers
    // =========================

    [[nodiscard]] RaycastHit2D raycast(const Ray2D& ray, const Segment2D& segment) noexcept;
    [[nodiscard]] RaycastHit3D raycast(const Ray3D& ray, const Plane3D& plane) noexcept;
    [[nodiscard]] RaycastHit3D raycast(const Ray3D& ray, const Sphere& sphere) noexcept;
    [[nodiscard]] RaycastHit3D raycast(const Ray3D& ray, const AABB3D& box) noexcept;
}



#endif