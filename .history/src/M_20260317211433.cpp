#include "M.hpp"

namespace M {

// --- Vector2D ---

float Vector2D::magnitude() const noexcept {
    return std::sqrt(v * v + w * w);
}

float Vector2D::angle() const noexcept {
    if ((*this) == Vector2D(0.0f, 0.0f)) {
        return 0.0f;
    }
    return std::atan2(w, v);
}

Vector2D& Vector2D::operator+=(const Vector2D& other) noexcept {
    v += other.v;
    w += other.w;
    return *this;
}

Vector2D& Vector2D::operator-=(const Vector2D& other) noexcept {
    v -= other.v;
    w -= other.w;
    return *this;
}

Vector2D& Vector2D::operator*=(float x) noexcept {
    v *= x;
    w *= x;
    return *this;
}

Vector2D& Vector2D::operator/=(float x) noexcept {
    if (x > -EPS && x < EPS) {
        v = std::numeric_limits<float>::quiet_NaN();
        w = std::numeric_limits<float>::quiet_NaN();
        return *this;
    }
    v /= x;
    w /= x;
    return *this;
}

Vector2D Vector2D::atMagnitude(float mag) const noexcept {
    const float m = magnitude();
    if (m < EPS) {
        return Vector2D(0.0f, 0.0f);
    }
    return (*this) * (mag / m);
}

Vector2D& Vector2D::setMagnitude(float mag) noexcept {
    const float m = magnitude();
    if (m < EPS) {
        v = 0.0f;
        w = 0.0f;
        return *this;
    }
    const float scale = mag / m;
    v *= scale;
    w *= scale;
    return *this;
}

float Vector2D::angleBetween(const Vector2D& other) const noexcept {
    const float m1 = magnitude();
    const float m2 = other.magnitude();

    if (m1 < EPS || m2 < EPS) {
        return 0.0f;
    }

    float c = dot(other) / (m1 * m2);
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;

    return std::acos(c);
}

Vector2D Vector2D::negated() const noexcept {
    return -(*this);
}

Vector2D& Vector2D::negate() noexcept {
    v = -v;
    w = -w;
    return *this;
}

float Vector2D::magnitudeSqr() const noexcept {
    return v * v + w * w;
}

Vector2D Vector2D::normalized() const noexcept {
    const float m = magnitude();
    if (m < EPS) {
        return Vector2D(0.0f, 0.0f);
    }
    return Vector2D(v / m, w / m);
}

Vector2D& Vector2D::normalize() noexcept {
    const float m = magnitude();
    if (m < EPS) {
        v = 0.0f;
        w = 0.0f;
        return *this;
    }
    v /= m;
    w /= m;
    return *this;
}

float Vector2D::dot(const Vector2D& other) const noexcept {
    return v * other.v + w * other.w;
}

float Vector2D::cross(const Vector2D& other) const noexcept {
    return v * other.w - w * other.v;
}

Vector2D Vector2D::rotated(float ang) const noexcept {
    const float c = std::cos(ang);
    const float s = std::sin(ang);
    return Vector2D(
        v * c - w * s,
        v * s + w * c
    );
}

Vector2D& Vector2D::rotate(float ang) noexcept {
    const float c = std::cos(ang);
    const float s = std::sin(ang);

    const float nv = v * c - w * s;
    const float nw = v * s + w * c;

    v = nv;
    w = nw;
    return *this;
}

Vector2D Vector2D::perpendicular() const noexcept {
    return Vector2D(-w, v);
}

Vector2D& Vector2D::makePerpendicular() noexcept {
    const float nv = -w;
    const float nw = v;
    v = nv;
    w = nw;
    return *this;
}

bool Vector2D::isParallel(const Vector2D& other) const noexcept {
    if (magnitudeSqr() < EPS || other.magnitudeSqr() < EPS) {
        return false;
    }
    return std::fabs(cross(other)) < EPS;
}

bool Vector2D::isPerpendicular(const Vector2D& other) const noexcept {
    if (magnitudeSqr() < EPS || other.magnitudeSqr() < EPS) {
        return false;
    }
    return std::fabs(dot(other)) < EPS;
}

// --- Point2D ---


Point2D& Point2D::operator+=(const Vector2D& other) noexcept {
    v += other.v;
    w += other.w;
    return *this;
}

Point2D& Point2D::operator-=(const Vector2D& other) noexcept {
    v -= other.v;
    w -= other.w;
    return *this;
}

float Point2D::distanceSqrTo(const Point2D& other) const noexcept {
    const float dv = v - other.v;
    const float dw = w - other.w;
    return dv * dv + dw * dw;
}

float Point2D::distanceTo(const Point2D& other) const noexcept {
    return std::sqrt(distanceSqrTo(other));
}

float Vector3D::magnitude() const noexcept {
    return std::sqrt(x * x + y * y + z * z);
}

// --- Vector3D ---

float Vector3D::magnitudeSqr() const noexcept {
    return x * x + y * y + z * z;
}

Vector3D& Vector3D::operator+=(const Vector3D& other) noexcept {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

Vector3D& Vector3D::operator-=(const Vector3D& other) noexcept {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
}

Vector3D& Vector3D::operator*=(float k) noexcept {
    x *= k;
    y *= k;
    z *= k;
    return *this;
}

Vector3D& Vector3D::operator/=(float k) noexcept {
    if (k > -EPS && k < EPS) {
        x = std::numeric_limits<float>::quiet_NaN();
        y = std::numeric_limits<float>::quiet_NaN();
        z = std::numeric_limits<float>::quiet_NaN();
        return *this;
    }
    x /= k;
    y /= k;
    z /= k;
    return *this;
}

Vector3D Vector3D::atMagnitude(float mag) const noexcept {
    const float m = magnitude();
    if (m < EPS) {
        return Vector3D(0.0f, 0.0f, 0.0f);
    }
    return (*this) * (mag / m);
}

Vector3D& Vector3D::setMagnitude(float mag) noexcept {
    const float m = magnitude();
    if (m < EPS) {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        return *this;
    }
    const float scale = mag / m;
    x *= scale;
    y *= scale;
    z *= scale;
    return *this;
}

Vector3D Vector3D::negated() const noexcept {
    return -(*this);
}

Vector3D& Vector3D::negate() noexcept {
    x = -x;
    y = -y;
    z = -z;
    return *this;
}

Vector3D Vector3D::normalized() const noexcept {
    const float m = magnitude();
    if (m < EPS) {
        return Vector3D(0.0f, 0.0f, 0.0f);
    }
    return Vector3D(x / m, y / m, z / m);
}

Vector3D& Vector3D::normalize() noexcept {
    const float m = magnitude();
    if (m < EPS) {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        return *this;
    }
    x /= m;
    y /= m;
    z /= m;
    return *this;
}

float Vector3D::dot(const Vector3D& other) const noexcept {
    return x * other.x + y * other.y + z * other.z;
}

Vector3D Vector3D::cross(const Vector3D& other) const noexcept {
    return Vector3D(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}

bool Vector3D::isParallel(const Vector3D& other) const noexcept {
    if (magnitudeSqr() < EPS || other.magnitudeSqr() < EPS) {
        return false;
    }
    return cross(other).magnitudeSqr() < EPS * EPS;;
}

bool Vector3D::isPerpendicular(const Vector3D& other) const noexcept {
    if (magnitudeSqr() < EPS || other.magnitudeSqr() < EPS) {
        return false;
    }
    return std::fabs(dot(other)) < EPS;
}

// --- Point3D ---

Point3D& Point3D::operator+=(const Vector3D& other) noexcept {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

Point3D& Point3D::operator-=(const Vector3D& other) noexcept {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
}

float Point3D::distanceSqrTo(const Point3D& other) const noexcept {
    const float dx = x - other.x;
    const float dy = y - other.y;
    const float dz = z - other.z;
    return dx * dx + dy * dy + dz * dz;
}

float Point3D::distanceTo(const Point3D& other) const noexcept {
    return std::sqrt(distanceSqrTo(other));
}

// =========================
// Vector4D
// =========================

float Vector4D::magnitude() const noexcept {
    return std::sqrt(x * x + y * y + z * z + w * w);
}

float Vector4D::magnitudeSqr() const noexcept {
    return x * x + y * y + z * z + w * w;
}

Vector4D& Vector4D::operator+=(const Vector4D& other) noexcept {
    x += other.x;
    y += other.y;
    z += other.z;
    w += other.w;
    return *this;
}

Vector4D& Vector4D::operator-=(const Vector4D& other) noexcept {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    w -= other.w;
    return *this;
}

Vector4D& Vector4D::operator*=(float k) noexcept {
    x *= k;
    y *= k;
    z *= k;
    w *= k;
    return *this;
}

Vector4D& Vector4D::operator/=(float k) noexcept {
    if (k > -EPS && k < EPS) {
        x = std::numeric_limits<float>::quiet_NaN();
        y = std::numeric_limits<float>::quiet_NaN();
        z = std::numeric_limits<float>::quiet_NaN();
        w = std::numeric_limits<float>::quiet_NaN();
        return *this;
    }
    x /= k;
    y /= k;
    z /= k;
    w /= k;
    return *this;
}

Vector4D Vector4D::normalized() const noexcept {
    const float m = magnitude();
    if (m < EPS) return Vector4D(0.0f, 0.0f, 0.0f, 0.0f);
    return Vector4D(x / m, y / m, z / m, w / m);
}

Vector4D& Vector4D::normalize() noexcept {
    const float m = magnitude();
    if (m < EPS) {
        x = y = z = w = 0.0f;
        return *this;
    }
    x /= m;
    y /= m;
    z /= m;
    w /= m;
    return *this;
}

float Vector4D::dot(const Vector4D& other) const noexcept {
    return x * other.x + y * other.y + z * other.z + w * other.w;
}

Vector4D Vector4D::atMagnitude(float mag) const noexcept {
    const float m = magnitude();
    if (m < EPS) return Vector4D(0.0f, 0.0f, 0.0f, 0.0f);
    return (*this) * (mag / m);
}

Vector4D& Vector4D::setMagnitude(float mag) noexcept {
    const float m = magnitude();
    if (m < EPS) {
        x = y = z = w = 0.0f;
        return *this;
    }
    const float s = mag / m;
    x *= s;
    y *= s;
    z *= s;
    w *= s;
    return *this;
}

// =========================
// Matrix3x3
// =========================

Matrix3x3 Matrix3x3::identity() noexcept {
    return Matrix3x3();
}

Matrix3x3 Matrix3x3::zero() noexcept {
    return Matrix3x3(
        0, 0, 0,
        0, 0, 0,
        0, 0, 0
    );
}

Matrix3x3 Matrix3x3::scaling(float sv, float sw) noexcept {
    return Matrix3x3(
        sv, 0,  0,
        0,  sw, 0,
        0,  0,  1
    );
}

Matrix3x3 Matrix3x3::rotation(float angle) noexcept {
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    return Matrix3x3(
         c, -s, 0,
         s,  c, 0,
         0,  0, 1
    );
}

Matrix3x3 Matrix3x3::translation(float tv, float tw) noexcept {
    return Matrix3x3(
        1, 0, tv,
        0, 1, tw,
        0, 0, 1
    );
}

bool Matrix3x3::operator==(const Matrix3x3& other) const noexcept {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            if (std::fabs(m[r][c] - other.m[r][c]) >= EPS) return false;
        }
    }
    return true;
}

bool Matrix3x3::operator!=(const Matrix3x3& other) const noexcept {
    return !(*this == other);
}

Matrix3x3 Matrix3x3::operator+(const Matrix3x3& other) const noexcept {
    Matrix3x3 out = zero();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out.m[r][c] = m[r][c] + other.m[r][c];
    return out;
}

Matrix3x3 Matrix3x3::operator-(const Matrix3x3& other) const noexcept {
    Matrix3x3 out = zero();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out.m[r][c] = m[r][c] - other.m[r][c];
    return out;
}

Matrix3x3 Matrix3x3::operator*(const Matrix3x3& other) const noexcept {
    Matrix3x3 out = zero();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            for (int k = 0; k < 3; ++k) {
                out.m[r][c] += m[r][k] * other.m[k][c];
            }
        }
    }
    return out;
}

Matrix3x3 Matrix3x3::operator*(float k) const noexcept {
    Matrix3x3 out = zero();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out.m[r][c] = m[r][c] * k;
    return out;
}

Matrix3x3& Matrix3x3::operator+=(const Matrix3x3& other) noexcept {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            m[r][c] += other.m[r][c];
    return *this;
}

Matrix3x3& Matrix3x3::operator-=(const Matrix3x3& other) noexcept {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            m[r][c] -= other.m[r][c];
    return *this;
}

Matrix3x3& Matrix3x3::operator*=(const Matrix3x3& other) noexcept {
    *this = (*this) * other;
    return *this;
}

Matrix3x3& Matrix3x3::operator*=(float k) noexcept {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            m[r][c] *= k;
    return *this;
}

Vector3D Matrix3x3::operator*(const Vector3D& vec) const noexcept {
    return Vector3D(
        m[0][0] * vec.x + m[0][1] * vec.y + m[0][2] * vec.z,
        m[1][0] * vec.x + m[1][1] * vec.y + m[1][2] * vec.z,
        m[2][0] * vec.x + m[2][1] * vec.y + m[2][2] * vec.z
    );
}

float Matrix3x3::determinant() const noexcept {
    return
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
        m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
        m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

Matrix3x3 Matrix3x3::transposed() const noexcept {
    return Matrix3x3(
        m[0][0], m[1][0], m[2][0],
        m[0][1], m[1][1], m[2][1],
        m[0][2], m[1][2], m[2][2]
    );
}

Matrix3x3& Matrix3x3::transpose() noexcept {
    *this = transposed();
    return *this;
}

Matrix3x3 Matrix3x3::inverted() const noexcept {
    const float det = determinant();
    if (std::fabs(det) < EPS) {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        return Matrix3x3(
            nan, nan, nan,
            nan, nan, nan,
            nan, nan, nan
        );
    }

    const float invDet = 1.0f / det;

    Matrix3x3 out(
        +(m[1][1] * m[2][2] - m[1][2] * m[2][1]),
        -(m[0][1] * m[2][2] - m[0][2] * m[2][1]),
        +(m[0][1] * m[1][2] - m[0][2] * m[1][1]),

        -(m[1][0] * m[2][2] - m[1][2] * m[2][0]),
        +(m[0][0] * m[2][2] - m[0][2] * m[2][0]),
        -(m[0][0] * m[1][2] - m[0][2] * m[1][0]),

        +(m[1][0] * m[2][1] - m[1][1] * m[2][0]),
        -(m[0][0] * m[2][1] - m[0][1] * m[2][0]),
        +(m[0][0] * m[1][1] - m[0][1] * m[1][0])
    );

    out *= invDet;
    return out.transposed();
}

Matrix3x3& Matrix3x3::invert() noexcept {
    *this = inverted();
    return *this;
}

// =========================
// Matrix4x4
// =========================

Matrix4x4 Matrix4x4::identity() noexcept {
    return Matrix4x4();
}

Matrix4x4 Matrix4x4::zero() noexcept {
    return Matrix4x4(
        0,0,0,0,
        0,0,0,0,
        0,0,0,0,
        0,0,0,0
    );
}

Matrix4x4 Matrix4x4::translation(float tx, float ty, float tz) noexcept {
    return Matrix4x4(
        1,0,0,tx,
        0,1,0,ty,
        0,0,1,tz,
        0,0,0,1
    );
}

Matrix4x4 Matrix4x4::scaling(float sx, float sy, float sz) noexcept {
    return Matrix4x4(
        sx,0, 0, 0,
        0, sy,0, 0,
        0, 0, sz,0,
        0, 0, 0, 1
    );
}

Matrix4x4 Matrix4x4::rotationX(float angle) noexcept {
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    return Matrix4x4(
        1,0,0,0,
        0,c,-s,0,
        0,s, c,0,
        0,0,0,1
    );
}

Matrix4x4 Matrix4x4::rotationY(float angle) noexcept {
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    return Matrix4x4(
         c,0,s,0,
         0,1,0,0,
        -s,0,c,0,
         0,0,0,1
    );
}

Matrix4x4 Matrix4x4::rotationZ(float angle) noexcept {
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    return Matrix4x4(
        c,-s,0,0,
        s, c,0,0,
        0, 0,1,0,
        0, 0,0,1
    );
}

bool Matrix4x4::operator==(const Matrix4x4& other) const noexcept {
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (std::fabs(m[r][c] - other.m[r][c]) >= EPS) return false;
        }
    }
    return true;
}

bool Matrix4x4::operator!=(const Matrix4x4& other) const noexcept {
    return !(*this == other);
}

Matrix4x4 Matrix4x4::operator+(const Matrix4x4& other) const noexcept {
    Matrix4x4 out = zero();
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out.m[r][c] = m[r][c] + other.m[r][c];
    return out;
}

Matrix4x4 Matrix4x4::operator-(const Matrix4x4& other) const noexcept {
    Matrix4x4 out = zero();
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out.m[r][c] = m[r][c] - other.m[r][c];
    return out;
}

Matrix4x4 Matrix4x4::operator*(const Matrix4x4& other) const noexcept {
    Matrix4x4 out = zero();
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            for (int k = 0; k < 4; ++k) {
                out.m[r][c] += m[r][k] * other.m[k][c];
            }
        }
    }
    return out;
}

Matrix4x4 Matrix4x4::operator*(float k) const noexcept {
    Matrix4x4 out = zero();
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out.m[r][c] = m[r][c] * k;
    return out;
}

Matrix4x4& Matrix4x4::operator+=(const Matrix4x4& other) noexcept {
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            m[r][c] += other.m[r][c];
    return *this;
}

Matrix4x4& Matrix4x4::operator-=(const Matrix4x4& other) noexcept {
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            m[r][c] -= other.m[r][c];
    return *this;
}

Matrix4x4& Matrix4x4::operator*=(const Matrix4x4& other) noexcept {
    *this = (*this) * other;
    return *this;
}

Matrix4x4& Matrix4x4::operator*=(float k) noexcept {
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            m[r][c] *= k;
    return *this;
}

Vector4D Matrix4x4::operator*(const Vector4D& vec) const noexcept {
    return Vector4D(
        m[0][0] * vec.x + m[0][1] * vec.y + m[0][2] * vec.z + m[0][3] * vec.w,
        m[1][0] * vec.x + m[1][1] * vec.y + m[1][2] * vec.z + m[1][3] * vec.w,
        m[2][0] * vec.x + m[2][1] * vec.y + m[2][2] * vec.z + m[2][3] * vec.w,
        m[3][0] * vec.x + m[3][1] * vec.y + m[3][2] * vec.z + m[3][3] * vec.w
    );
}

static float det3x3_inline(
    float a00, float a01, float a02,
    float a10, float a11, float a12,
    float a20, float a21, float a22
) {
    return
        a00 * (a11 * a22 - a12 * a21) -
        a01 * (a10 * a22 - a12 * a20) +
        a02 * (a10 * a21 - a11 * a20);
}

float Matrix4x4::determinant() const noexcept {
    const float c0 = det3x3_inline(
        m[1][1], m[1][2], m[1][3],
        m[2][1], m[2][2], m[2][3],
        m[3][1], m[3][2], m[3][3]
    );
    const float c1 = det3x3_inline(
        m[1][0], m[1][2], m[1][3],
        m[2][0], m[2][2], m[2][3],
        m[3][0], m[3][2], m[3][3]
    );
    const float c2 = det3x3_inline(
        m[1][0], m[1][1], m[1][3],
        m[2][0], m[2][1], m[2][3],
        m[3][0], m[3][1], m[3][3]
    );
    const float c3 = det3x3_inline(
        m[1][0], m[1][1], m[1][2],
        m[2][0], m[2][1], m[2][2],
        m[3][0], m[3][1], m[3][2]
    );

    return m[0][0] * c0 - m[0][1] * c1 + m[0][2] * c2 - m[0][3] * c3;
}

Matrix4x4 Matrix4x4::transposed() const noexcept {
    return Matrix4x4(
        m[0][0], m[1][0], m[2][0], m[3][0],
        m[0][1], m[1][1], m[2][1], m[3][1],
        m[0][2], m[1][2], m[2][2], m[3][2],
        m[0][3], m[1][3], m[2][3], m[3][3]
    );
}

Matrix4x4& Matrix4x4::transpose() noexcept {
    *this = transposed();
    return *this;
}

Matrix4x4 Matrix4x4::inverted() const noexcept {
    Matrix4x4 a = *this;
    Matrix4x4 inv = identity();

    for (int col = 0; col < 4; ++col) {
        int pivot = col;
        float best = std::fabs(a.m[col][col]);

        for (int row = col + 1; row < 4; ++row) {
            const float v = std::fabs(a.m[row][col]);
            if (v > best) {
                best = v;
                pivot = row;
            }
        }

        if (best < EPS) {
            const float nan = std::numeric_limits<float>::quiet_NaN();
            return Matrix4x4(
                nan, nan, nan, nan,
                nan, nan, nan, nan,
                nan, nan, nan, nan,
                nan, nan, nan, nan
            );
        }

        if (pivot != col) {
            for (int c = 0; c < 4; ++c) {
                std::swap(a.m[col][c], inv.m[col][c]);
                std::swap(a.m[pivot][c], inv.m[pivot][c]);
            }
            for (int c = 0; c < 4; ++c) {
                std::swap(a.m[col][c], a.m[pivot][c]);
                std::swap(inv.m[col][c], inv.m[pivot][c]);
            }
        }

        const float diag = a.m[col][col];
        for (int c = 0; c < 4; ++c) {
            a.m[col][c] /= diag;
            inv.m[col][c] /= diag;
        }

        for (int row = 0; row < 4; ++row) {
            if (row == col) continue;
            const float factor = a.m[row][col];
            for (int c = 0; c < 4; ++c) {
                a.m[row][c] -= factor * a.m[col][c];
                inv.m[row][c] -= factor * inv.m[col][c];
            }
        }
    }

    return inv;
}

Matrix4x4& Matrix4x4::invert() noexcept {
    *this = inverted();
    return *this;
}

// =========================
// Line2D
// =========================

Line2D::Line2D(const Point2D& a, const Point2D& b) noexcept
    : point(a), dir(b - a) {}

Line2D::Line2D(const Point2D& center, float angle) noexcept
    : point(center), dir(std::cos(angle), std::sin(angle)) {}

bool Line2D::isValid() const noexcept {
    return dir.magnitudeSqr() >= EPS;
}

Point2D Line2D::pointAt(float t) const noexcept {
    return point + dir * t;
}

bool Line2D::contains(const Point2D& p) const noexcept {
    if (!isValid()) return false;
    return std::fabs((p - point).cross(dir)) < EPS;
}

bool Line2D::isParallel(const Line2D& other) const noexcept {
    return dir.isParallel(other.dir);
}

bool Line2D::isPerpendicular(const Line2D& other) const noexcept {
    return dir.isPerpendicular(other.dir);
}

Point2D Line2D::projected(const Point2D& p) const noexcept {
    if (!isValid()) return point;
    const Vector2D ap = p - point;
    const float t = ap.dot(dir) / dir.magnitudeSqr();
    return pointAt(t);
}

float Line2D::distanceSqrTo(const Point2D& p) const noexcept {
    return p.distanceSqrTo(projected(p));
}

float Line2D::distanceTo(const Point2D& p) const noexcept {
    return std::sqrt(distanceSqrTo(p));
}

// =========================
// Ray2D
// =========================

Ray2D::Ray2D(const Point2D& origin_, const Point2D& through) noexcept
    : origin(origin_), dir(through - origin_) {}

Ray2D::Ray2D(const Point2D& origin_, float angle) noexcept
    : origin(origin_), dir(std::cos(angle), std::sin(angle)) {}

bool Ray2D::isValid() const noexcept {
    return dir.magnitudeSqr() >= EPS;
}

Point2D Ray2D::pointAt(float t) const noexcept {
    return origin + dir * t;
}

bool Ray2D::contains(const Point2D& p) const noexcept {
    if (!isValid()) return false;
    const Vector2D op = p - origin;
    if (std::fabs(op.cross(dir)) >= EPS) return false;
    return op.dot(dir) >= -EPS;
}

Point2D Ray2D::projected(const Point2D& p) const noexcept {
    if (!isValid()) return origin;
    const Vector2D op = p - origin;
    float t = op.dot(dir) / dir.magnitudeSqr();
    if (t < 0.0f) t = 0.0f;
    return pointAt(t);
}

float Ray2D::distanceSqrTo(const Point2D& p) const noexcept {
    return p.distanceSqrTo(projected(p));
}

float Ray2D::distanceTo(const Point2D& p) const noexcept {
    return std::sqrt(distanceSqrTo(p));
}

// =========================
// Segment2D
// =========================

Vector2D Segment2D::direction() const noexcept {
    return end - start;
}

float Segment2D::lengthSqr() const noexcept {
    return start.distanceSqrTo(end);
}

float Segment2D::length() const noexcept {
    return std::sqrt(lengthSqr());
}

Point2D Segment2D::midpoint() const noexcept {
    return Point2D(
        (start.v + end.v) * 0.5f,
        (start.w + end.w) * 0.5f
    );
}

Point2D Segment2D::pointAt(float t) const noexcept {
    return start + direction() * t;
}

bool Segment2D::isDegenerate() const noexcept {
    return lengthSqr() < EPS;
}

bool Segment2D::contains(const Point2D& p) const noexcept {
    if (isDegenerate()) return p == start;

    const Vector2D d = direction();
    const Vector2D sp = p - start;

    if (std::fabs(sp.cross(d)) >= EPS) return false;

    const float t = sp.dot(d) / d.magnitudeSqr();
    return t >= -EPS && t <= 1.0f + EPS;
}

Point2D Segment2D::projected(const Point2D& p) const noexcept {
    if (isDegenerate()) return start;

    const Vector2D d = direction();
    const Vector2D sp = p - start;
    float t = sp.dot(d) / d.magnitudeSqr();

    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    return pointAt(t);
}

float Segment2D::distanceSqrTo(const Point2D& p) const noexcept {
    return p.distanceSqrTo(projected(p));
}

float Segment2D::distanceTo(const Point2D& p) const noexcept {
    return std::sqrt(distanceSqrTo(p));
}

// =========================
// Line3D
// =========================

Line3D::Line3D(const Point3D& a, const Point3D& b) noexcept
    : point(a), dir(b - a) {}

bool Line3D::isValid() const noexcept {
    return dir.magnitudeSqr() >= EPS;
}

Point3D Line3D::pointAt(float t) const noexcept {
    return point + dir * t;
}

bool Line3D::contains(const Point3D& p) const noexcept {
    if (!isValid()) return false;
    return (p - point).cross(dir).magnitudeSqr() < EPS * EPS;
}

bool Line3D::isParallel(const Line3D& other) const noexcept {
    return dir.isParallel(other.dir);
}

bool Line3D::isPerpendicular(const Line3D& other) const noexcept {
    return dir.isPerpendicular(other.dir);
}

Point3D Line3D::projected(const Point3D& p) const noexcept {
    if (!isValid()) return point;
    const Vector3D ap = p - point;
    const float t = ap.dot(dir) / dir.magnitudeSqr();
    return pointAt(t);
}

float Line3D::distanceSqrTo(const Point3D& p) const noexcept {
    return p.distanceSqrTo(projected(p));
}

float Line3D::distanceTo(const Point3D& p) const noexcept {
    return std::sqrt(distanceSqrTo(p));
}

// =========================
// Ray3D
// =========================

Ray3D::Ray3D(const Point3D& origin_, const Point3D& through) noexcept
    : origin(origin_), dir(through - origin_) {}

bool Ray3D::isValid() const noexcept {
    return dir.magnitudeSqr() >= EPS;
}

Point3D Ray3D::pointAt(float t) const noexcept {
    return origin + dir * t;
}

bool Ray3D::contains(const Point3D& p) const noexcept {
    if (!isValid()) return false;
    const Vector3D op = p - origin;
    if (op.cross(dir).magnitudeSqr() >= EPS * EPS) return false;
    return op.dot(dir) >= -EPS;
}

Point3D Ray3D::projected(const Point3D& p) const noexcept {
    if (!isValid()) return origin;
    const Vector3D op = p - origin;
    float t = op.dot(dir) / dir.magnitudeSqr();
    if (t < 0.0f) t = 0.0f;
    return pointAt(t);
}

float Ray3D::distanceSqrTo(const Point3D& p) const noexcept {
    return p.distanceSqrTo(projected(p));
}

float Ray3D::distanceTo(const Point3D& p) const noexcept {
    return std::sqrt(distanceSqrTo(p));
}

// =========================
// Segment3D
// =========================

Vector3D Segment3D::direction() const noexcept {
    return end - start;
}

float Segment3D::lengthSqr() const noexcept {
    return start.distanceSqrTo(end);
}

float Segment3D::length() const noexcept {
    return std::sqrt(lengthSqr());
}

Point3D Segment3D::midpoint() const noexcept {
    return Point3D(
        (start.x + end.x) * 0.5f,
        (start.y + end.y) * 0.5f,
        (start.z + end.z) * 0.5f
    );
}

Point3D Segment3D::pointAt(float t) const noexcept {
    return start + direction() * t;
}

bool Segment3D::isDegenerate() const noexcept {
    return lengthSqr() < EPS;
}

bool Segment3D::contains(const Point3D& p) const noexcept {
    if (isDegenerate()) return p == start;

    const Vector3D d = direction();
    const Vector3D sp = p - start;

    if (sp.cross(d).magnitudeSqr() >= EPS * EPS) return false;

    const float t = sp.dot(d) / d.magnitudeSqr();
    return t >= -EPS && t <= 1.0f + EPS;
}

Point3D Segment3D::projected(const Point3D& p) const noexcept {
    if (isDegenerate()) return start;

    const Vector3D d = direction();
    const Vector3D sp = p - start;
    float t = sp.dot(d) / d.magnitudeSqr();

    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    return pointAt(t);
}

float Segment3D::distanceSqrTo(const Point3D& p) const noexcept {
    return p.distanceSqrTo(projected(p));
}

float Segment3D::distanceTo(const Point3D& p) const noexcept {
    return std::sqrt(distanceSqrTo(p));
}

// =========================
// Plane3D
// =========================

Plane3D::Plane3D(const Vector3D& normal_, const Point3D& point) noexcept
    : normal(normal_) {
    d = -(normal.x * point.x + normal.y * point.y + normal.z * point.z);
}

Plane3D::Plane3D(const Point3D& a, const Point3D& b, const Point3D& c) noexcept {
    normal = (b - a).cross(c - a);
    d = -(normal.x * a.x + normal.y * a.y + normal.z * a.z);
}

bool Plane3D::isValid() const noexcept {
    return normal.magnitudeSqr() >= EPS;
}

float Plane3D::signedDistanceTo(const Point3D& p) const noexcept {
    const float nmag = normal.magnitude();
    if (nmag < EPS) return std::numeric_limits<float>::quiet_NaN();
    return (normal.x * p.x + normal.y * p.y + normal.z * p.z + d) / nmag;
}

float Plane3D::distanceTo(const Point3D& p) const noexcept {
    return std::fabs(signedDistanceTo(p));
}

bool Plane3D::contains(const Point3D& p) const noexcept {
    if (!isValid()) return false;
    return std::fabs(normal.x * p.x + normal.y * p.y + normal.z * p.z + d) < EPS;
}

Point3D Plane3D::projected(const Point3D& p) const noexcept {
    if (!isValid()) return p;
    const float denom = normal.magnitudeSqr();
    const float t = (normal.x * p.x + normal.y * p.y + normal.z * p.z + d) / denom;
    return Point3D(
        p.x - normal.x * t,
        p.y - normal.y * t,
        p.z - normal.z * t
    );
}

Plane3D Plane3D::normalized() const noexcept {
    const float m = normal.magnitude();
    if (m < EPS) {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        return Plane3D(Vector3D(nan, nan, nan), nan);
    }
    return Plane3D(
        Vector3D(normal.x / m, normal.y / m, normal.z / m),
        d / m
    );
}

Plane3D& Plane3D::normalize() noexcept {
    const float m = normal.magnitude();
    if (m < EPS) {
        normal.x = normal.y = normal.z = std::numeric_limits<float>::quiet_NaN();
        d = std::numeric_limits<float>::quiet_NaN();
        return *this;
    }
    normal /= m;
    d /= m;
    return *this;
}

}