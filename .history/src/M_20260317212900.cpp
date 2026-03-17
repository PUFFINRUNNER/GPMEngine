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

// =========================
// Utility scalar helpers
// =========================

float clamp(float x, float lo, float hi) noexcept {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float lerp(float a, float b, float t) noexcept {
    return a + (b - a) * t;
}

float inverseLerp(float a, float b, float value) noexcept {
    if (std::fabs(b - a) < EPS) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    return (value - a) / (b - a);
}

float remap(float inA, float inB, float outA, float outB, float value) noexcept {
    const float t = inverseLerp(inA, inB, value);
    if (std::isnan(t)) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    return lerp(outA, outB, t);
}

float radiansToDegrees(float radians) noexcept {
    return radians * (180.0f / 3.14159265358979323846f);
}

float degreesToRadians(float degrees) noexcept {
    return degrees * (3.14159265358979323846f / 180.0f);
}

bool nearlyEqual(float a, float b, float eps) noexcept {
    return std::fabs(a - b) < eps;
}

// =========================
// Transform2D
// =========================

Matrix3x3 Transform2D::matrix() const noexcept {
    return Matrix3x3::translation(position.v, position.w) *
           Matrix3x3::rotation(rotation) *
           Matrix3x3::scaling(scale.v, scale.w);
}

Point2D Transform2D::appliedTo(const Point2D& p) const noexcept {
    const Vector3D hp(p.v, p.w, 1.0f);
    const Vector3D out = matrix() * hp;
    return Point2D(out.x, out.y);
}

Vector2D Transform2D::appliedTo(const Vector2D& v) const noexcept {
    const Vector3D hv(v.v, v.w, 0.0f);
    const Vector3D out = matrix() * hv;
    return Vector2D(out.x, out.y);
}

Transform2D Transform2D::combinedWith(const Transform2D& other) const noexcept {
    Transform2D out;

    out.position = appliedTo(other.position);

    out.rotation = rotation + other.rotation;

    out.scale = Vector2D(
        scale.v * other.scale.v,
        scale.w * other.scale.w
    );

    if (std::fabs(out.scale.v) < EPS) out.scale.v = 0.0f;
    if (std::fabs(out.scale.w) < EPS) out.scale.w = 0.0f;

    while (out.rotation > 3.14159265358979323846f) {
        out.rotation -= 2.0f * 3.14159265358979323846f;
    }
    while (out.rotation < -3.14159265358979323846f) {
        out.rotation += 2.0f * 3.14159265358979323846f;
    }

    return out;
}

// =========================
// Quaternion
// =========================

float Quaternion::magnitude() const noexcept {
    return std::sqrt(x * x + y * y + z * z + w * w);
}

float Quaternion::magnitudeSqr() const noexcept {
    return x * x + y * y + z * z + w * w;
}

Quaternion Quaternion::normalized() const noexcept {
    const float m = magnitude();
    if (m < EPS) {
        return Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    }
    return Quaternion(x / m, y / m, z / m, w / m);
}

Quaternion& Quaternion::normalize() noexcept {
    *this = normalized();
    return *this;
}

Quaternion Quaternion::conjugated() const noexcept {
    return Quaternion(-x, -y, -z, w);
}

Quaternion& Quaternion::conjugate() noexcept {
    x = -x;
    y = -y;
    z = -z;
    return *this;
}

Quaternion Quaternion::inverted() const noexcept {
    const float ms = magnitudeSqr();
    if (ms < EPS) {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        return Quaternion(nan, nan, nan, nan);
    }
    return conjugated() / ms;
}

Quaternion& Quaternion::invert() noexcept {
    *this = inverted();
    return *this;
}

bool Quaternion::operator==(const Quaternion& other) const noexcept {
    return std::fabs(x - other.x) < EPS &&
           std::fabs(y - other.y) < EPS &&
           std::fabs(z - other.z) < EPS &&
           std::fabs(w - other.w) < EPS;
}

bool Quaternion::operator!=(const Quaternion& other) const noexcept {
    return !(*this == other);
}

Quaternion Quaternion::operator/(float k) const noexcept {
    if (k > -EPS && k < EPS) {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        return Quaternion(nan, nan, nan, nan);
    }
    return Quaternion(x / k, y / k, z / k, w / k);
}

Quaternion Quaternion::operator*(const Quaternion& other) const noexcept {
    return Quaternion(
        w * other.x + x * other.w + y * other.z - z * other.y,
        w * other.y - x * other.z + y * other.w + z * other.x,
        w * other.z + x * other.y - y * other.x + z * other.w,
        w * other.w - x * other.x - y * other.y - z * other.z
    );
}

Quaternion& Quaternion::operator+=(const Quaternion& other) noexcept {
    x += other.x;
    y += other.y;
    z += other.z;
    w += other.w;
    return *this;
}

Quaternion& Quaternion::operator-=(const Quaternion& other) noexcept {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    w -= other.w;
    return *this;
}

Quaternion& Quaternion::operator*=(float k) noexcept {
    x *= k;
    y *= k;
    z *= k;
    w *= k;
    return *this;
}

Quaternion& Quaternion::operator/=(float k) noexcept {
    *this = (*this) / k;
    return *this;
}

Quaternion& Quaternion::operator*=(const Quaternion& other) noexcept {
    *this = (*this) * other;
    return *this;
}

float Quaternion::dot(const Quaternion& other) const noexcept {
    return x * other.x + y * other.y + z * other.z + w * other.w;
}

Matrix4x4 Quaternion::toMatrix4x4() const noexcept {
    const Quaternion q = normalized();

    const float xx = q.x * q.x;
    const float yy = q.y * q.y;
    const float zz = q.z * q.z;
    const float xy = q.x * q.y;
    const float xz = q.x * q.z;
    const float yz = q.y * q.z;
    const float wx = q.w * q.x;
    const float wy = q.w * q.y;
    const float wz = q.w * q.z;

    return Matrix4x4(
        1.0f - 2.0f * (yy + zz), 2.0f * (xy - wz),         2.0f * (xz + wy),         0.0f,
        2.0f * (xy + wz),         1.0f - 2.0f * (xx + zz), 2.0f * (yz - wx),         0.0f,
        2.0f * (xz - wy),         2.0f * (yz + wx),         1.0f - 2.0f * (xx + yy), 0.0f,
        0.0f,                     0.0f,                     0.0f,                     1.0f
    );
}

Vector3D Quaternion::rotatedVector(const Vector3D& v) const noexcept {
    const Quaternion qv(v.x, v.y, v.z, 0.0f);
    const Quaternion r = (*this) * qv * this->inverted();
    return Vector3D(r.x, r.y, r.z);
}

Quaternion Quaternion::identity() noexcept {
    return Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
}

Quaternion Quaternion::fromAxisAngle(const Vector3D& axis, float angle) noexcept {
    if (axis.magnitudeSqr() < EPS) {
        return identity();
    }

    const Vector3D n = axis.normalized();
    const float half = angle * 0.5f;
    const float s = std::sin(half);
    const float c = std::cos(half);

    return Quaternion(n.x * s, n.y * s, n.z * s, c);
}

// =========================
// Transform3D
// =========================

Matrix4x4 Transform3D::matrix() const noexcept {
    return Matrix4x4::translation(position.x, position.y, position.z) *
           rotation.toMatrix4x4() *
           Matrix4x4::scaling(scale.x, scale.y, scale.z);
}

Point3D Transform3D::appliedTo(const Point3D& p) const noexcept {
    const Vector4D hp(p.x, p.y, p.z, 1.0f);
    const Vector4D out = matrix() * hp;
    if (std::fabs(out.w) < EPS) {
        return Point3D(out.x, out.y, out.z);
    }
    return Point3D(out.x / out.w, out.y / out.w, out.z / out.w);
}

Vector3D Transform3D::appliedTo(const Vector3D& v) const noexcept {
    const Vector4D hv(v.x, v.y, v.z, 0.0f);
    const Vector4D out = matrix() * hv;
    return Vector3D(out.x, out.y, out.z);
}

Transform3D Transform3D::combinedWith(const Transform3D& other) const noexcept {
    Transform3D out;
    out.position = appliedTo(other.position);
    out.rotation = rotation * other.rotation;
    out.scale = Vector3D(
        scale.x * other.scale.x,
        scale.y * other.scale.y,
        scale.z * other.scale.z
    );
    return out;
}

// =========================
// AABB2D
// =========================

bool AABB2D::isValid() const noexcept {
    return min.v <= max.v + EPS && min.w <= max.w + EPS;
}

Point2D AABB2D::center() const noexcept {
    return Point2D((min.v + max.v) * 0.5f, (min.w + max.w) * 0.5f);
}

Vector2D AABB2D::size() const noexcept {
    return Vector2D(max.v - min.v, max.w - min.w);
}

float AABB2D::area() const noexcept {
    if (!isValid()) return 0.0f;
    const Vector2D s = size();
    return s.v * s.w;
}

bool AABB2D::contains(const Point2D& p) const noexcept {
    return p.v >= min.v - EPS && p.v <= max.v + EPS &&
           p.w >= min.w - EPS && p.w <= max.w + EPS;
}

bool AABB2D::overlaps(const AABB2D& other) const noexcept {
    return !(max.v < other.min.v - EPS || other.max.v < min.v - EPS ||
             max.w < other.min.w - EPS || other.max.w < min.w - EPS);
}

Point2D AABB2D::clampedPoint(const Point2D& p) const noexcept {
    return Point2D(
        clamp(p.v, min.v, max.v),
        clamp(p.w, min.w, max.w)
    );
}

float AABB2D::distanceSqrTo(const Point2D& p) const noexcept {
    return p.distanceSqrTo(clampedPoint(p));
}

float AABB2D::distanceTo(const Point2D& p) const noexcept {
    return std::sqrt(distanceSqrTo(p));
}

void AABB2D::expandToInclude(const Point2D& p) noexcept {
    if (p.v < min.v) min.v = p.v;
    if (p.w < min.w) min.w = p.w;
    if (p.v > max.v) max.v = p.v;
    if (p.w > max.w) max.w = p.w;
}

void AABB2D::expandToInclude(const AABB2D& other) noexcept {
    expandToInclude(other.min);
    expandToInclude(other.max);
}

// =========================
// AABB3D
// =========================

bool AABB3D::isValid() const noexcept {
    return min.x <= max.x + EPS &&
           min.y <= max.y + EPS &&
           min.z <= max.z + EPS;
}

Point3D AABB3D::center() const noexcept {
    return Point3D(
        (min.x + max.x) * 0.5f,
        (min.y + max.y) * 0.5f,
        (min.z + max.z) * 0.5f
    );
}

Vector3D AABB3D::size() const noexcept {
    return Vector3D(max.x - min.x, max.y - min.y, max.z - min.z);
}

float AABB3D::volume() const noexcept {
    if (!isValid()) return 0.0f;
    const Vector3D s = size();
    return s.x * s.y * s.z;
}

bool AABB3D::contains(const Point3D& p) const noexcept {
    return p.x >= min.x - EPS && p.x <= max.x + EPS &&
           p.y >= min.y - EPS && p.y <= max.y + EPS &&
           p.z >= min.z - EPS && p.z <= max.z + EPS;
}

bool AABB3D::overlaps(const AABB3D& other) const noexcept {
    return !(max.x < other.min.x - EPS || other.max.x < min.x - EPS ||
             max.y < other.min.y - EPS || other.max.y < min.y - EPS ||
             max.z < other.min.z - EPS || other.max.z < min.z - EPS);
}

Point3D AABB3D::clampedPoint(const Point3D& p) const noexcept {
    return Point3D(
        clamp(p.x, min.x, max.x),
        clamp(p.y, min.y, max.y),
        clamp(p.z, min.z, max.z)
    );
}

float AABB3D::distanceSqrTo(const Point3D& p) const noexcept {
    return p.distanceSqrTo(clampedPoint(p));
}

float AABB3D::distanceTo(const Point3D& p) const noexcept {
    return std::sqrt(distanceSqrTo(p));
}

void AABB3D::expandToInclude(const Point3D& p) noexcept {
    if (p.x < min.x) min.x = p.x;
    if (p.y < min.y) min.y = p.y;
    if (p.z < min.z) min.z = p.z;
    if (p.x > max.x) max.x = p.x;
    if (p.y > max.y) max.y = p.y;
    if (p.z > max.z) max.z = p.z;
}

void AABB3D::expandToInclude(const AABB3D& other) noexcept {
    expandToInclude(other.min);
    expandToInclude(other.max);
}

// =========================
// Circle
// =========================

bool Circle::isValid() const noexcept {
    return radius >= 0.0f;
}

float Circle::area() const noexcept {
    return 3.14159265358979323846f * radius * radius;
}

float Circle::circumference() const noexcept {
    return 2.0f * 3.14159265358979323846f * radius;
}

bool Circle::contains(const Point2D& p) const noexcept {
    return center.distanceSqrTo(p) <= radius * radius + EPS;
}

bool Circle::overlaps(const Circle& other) const noexcept {
    const float r = radius + other.radius;
    return center.distanceSqrTo(other.center) <= r * r + EPS;
}

Point2D Circle::clampedPoint(const Point2D& p) const noexcept {
    const Vector2D d = p - center;
    const float ms = d.magnitudeSqr();
    if (ms <= radius * radius + EPS || ms < EPS) {
        return p;
    }
    return center + d.atMagnitude(radius);
}

float Circle::distanceSqrTo(const Point2D& p) const noexcept {
    const float d = center.distanceTo(p) - radius;
    return d <= 0.0f ? 0.0f : d * d;
}

float Circle::distanceTo(const Point2D& p) const noexcept {
    const float d = center.distanceTo(p) - radius;
    return d <= 0.0f ? 0.0f : d;
}

// =========================
// Sphere
// =========================

bool Sphere::isValid() const noexcept {
    return radius >= 0.0f;
}

float Sphere::volume() const noexcept {
    return (4.0f / 3.0f) * 3.14159265358979323846f * radius * radius * radius;
}

float Sphere::surfaceArea() const noexcept {
    return 4.0f * 3.14159265358979323846f * radius * radius;
}

bool Sphere::contains(const Point3D& p) const noexcept {
    return center.distanceSqrTo(p) <= radius * radius + EPS;
}

bool Sphere::overlaps(const Sphere& other) const noexcept {
    const float r = radius + other.radius;
    return center.distanceSqrTo(other.center) <= r * r + EPS;
}

Point3D Sphere::clampedPoint(const Point3D& p) const noexcept {
    const Vector3D d = p - center;
    const float ms = d.magnitudeSqr();
    if (ms <= radius * radius + EPS || ms < EPS) {
        return p;
    }
    return center + d.atMagnitude(radius);
}

float Sphere::distanceSqrTo(const Point3D& p) const noexcept {
    const float d = center.distanceTo(p) - radius;
    return d <= 0.0f ? 0.0f : d * d;
}

float Sphere::distanceTo(const Point3D& p) const noexcept {
    const float d = center.distanceTo(p) - radius;
    return d <= 0.0f ? 0.0f : d;
}

// =========================
// Triangle2D
// =========================

float Triangle2D::signedArea() const noexcept {
    return 0.5f * ((b - a).cross(c - a));
}

float Triangle2D::area() const noexcept {
    return std::fabs(signedArea());
}

Point2D Triangle2D::centroid() const noexcept {
    return Point2D(
        (a.v + b.v + c.v) / 3.0f,
        (a.w + b.w + c.w) / 3.0f
    );
}

bool Triangle2D::isDegenerate() const noexcept {
    return std::fabs(signedArea()) < EPS;
}

bool Triangle2D::contains(const Point2D& p) const noexcept {
    if (isDegenerate()) return false;

    const float c1 = (b - a).cross(p - a);
    const float c2 = (c - b).cross(p - b);
    const float c3 = (a - c).cross(p - c);

    const bool hasNeg = (c1 < -EPS) || (c2 < -EPS) || (c3 < -EPS);
    const bool hasPos = (c1 > EPS) || (c2 > EPS) || (c3 > EPS);

    return !(hasNeg && hasPos);
}

// =========================
// Triangle3D
// =========================

Vector3D Triangle3D::normal() const noexcept {
    return (b - a).cross(c - a);
}

float Triangle3D::area() const noexcept {
    return 0.5f * normal().magnitude();
}

Point3D Triangle3D::centroid() const noexcept {
    return Point3D(
        (a.x + b.x + c.x) / 3.0f,
        (a.y + b.y + c.y) / 3.0f,
        (a.z + b.z + c.z) / 3.0f
    );
}

bool Triangle3D::isDegenerate() const noexcept {
    return normal().magnitudeSqr() < EPS;
}

Plane3D Triangle3D::plane() const noexcept {
    return Plane3D(a, b, c);
}

// =========================
// Closest-point helpers
// =========================

Point2D closestPoint(const Line2D& line, const Point2D& p) noexcept {
    return line.projected(p);
}

Point2D closestPoint(const Ray2D& ray, const Point2D& p) noexcept {
    return ray.projected(p);
}

Point2D closestPoint(const Segment2D& segment, const Point2D& p) noexcept {
    return segment.projected(p);
}

Point3D closestPoint(const Line3D& line, const Point3D& p) noexcept {
    return line.projected(p);
}

Point3D closestPoint(const Ray3D& ray, const Point3D& p) noexcept {
    return ray.projected(p);
}

Point3D closestPoint(const Segment3D& segment, const Point3D& p) noexcept {
    return segment.projected(p);
}

Point3D closestPoint(const Plane3D& plane, const Point3D& p) noexcept {
    return plane.projected(p);
}

// =========================
// Intersection / overlap tests
// =========================

bool intersects(const AABB2D& a, const AABB2D& b) noexcept {
    return a.overlaps(b);
}

bool intersects(const AABB3D& a, const AABB3D& b) noexcept {
    return a.overlaps(b);
}

bool intersects(const Circle& a, const Circle& b) noexcept {
    return a.overlaps(b);
}

bool intersects(const Sphere& a, const Sphere& b) noexcept {
    return a.overlaps(b);
}

bool intersects(const Line2D& a, const Line2D& b) noexcept {
    if (!a.isValid() || !b.isValid()) return false;
    if (!a.dir.isParallel(b.dir)) return true;
    return a.contains(b.point);
}

bool intersects(const Ray2D& ray, const Segment2D& segment) noexcept {
    return raycast(ray, segment).hit;
}

bool intersects(const Segment2D& a, const Segment2D& b) noexcept {
    const Vector2D r = a.end - a.start;
    const Vector2D s = b.end - b.start;
    const float rxs = r.cross(s);
    const Vector2D qp = b.start - a.start;

    if (std::fabs(rxs) < EPS) {
        if (std::fabs(qp.cross(r)) >= EPS) return false;

        const float rr = r.magnitudeSqr();
        if (rr < EPS) return a.start == b.start;

        const float t0 = qp.dot(r) / rr;
        const float t1 = t0 + s.dot(r) / rr;

        const float lo = t0 < t1 ? t0 : t1;
        const float hi = t0 > t1 ? t0 : t1;
        return !(hi < -EPS || lo > 1.0f + EPS);
    }

    const float t = qp.cross(s) / rxs;
    const float u = qp.cross(r) / rxs;

    return t >= -EPS && t <= 1.0f + EPS &&
           u >= -EPS && u <= 1.0f + EPS;
}

bool intersects(const Line3D& line, const Plane3D& plane) noexcept {
    if (!line.isValid() || !plane.isValid()) return false;
    const float denom = plane.normal.dot(line.dir);
    if (std::fabs(denom) >= EPS) return true;
    return plane.contains(line.point);
}

bool intersects(const Ray3D& ray, const Plane3D& plane) noexcept {
    if (!ray.isValid() || !plane.isValid()) return false;
    const float denom = plane.normal.dot(ray.dir);
    const float num = -(plane.normal.x * ray.origin.x +
                        plane.normal.y * ray.origin.y +
                        plane.normal.z * ray.origin.z + plane.d);

    if (std::fabs(denom) < EPS) {
        return std::fabs(num) < EPS;
    }

    const float t = num / denom;
    return t >= -EPS;
}

bool intersects(const Ray3D& ray, const Sphere& sphere) noexcept {
    return raycast(ray, sphere).hit;
}

bool intersects(const Ray3D& ray, const AABB3D& box) noexcept {
    return raycast(ray, box).hit;
}

// =========================
// Raycast helpers
// =========================

RaycastHit2D raycast(const Ray2D& ray, const Segment2D& segment) noexcept {
    RaycastHit2D hit{};
    hit.hit = false;
    hit.t = std::numeric_limits<float>::quiet_NaN();
    hit.point = Point2D();
    hit.normal = Vector2D();

    if (!ray.isValid() || segment.isDegenerate()) {
        return hit;
    }

    const Vector2D r = ray.dir;
    const Vector2D s = segment.end - segment.start;
    const Vector2D qp = segment.start - ray.origin;
    const float rxs = r.cross(s);

    if (std::fabs(rxs) < EPS) {
        return hit;
    }

    const float t = qp.cross(s) / rxs;
    const float u = qp.cross(r) / rxs;

    if (t < -EPS || u < -EPS || u > 1.0f + EPS) {
        return hit;
    }

    hit.hit = true;
    hit.t = t;
    hit.point = ray.pointAt(t);

    Vector2D n = s.perpendicular();
    if (n.magnitudeSqr() >= EPS) {
        n.normalize();
        if (n.dot(ray.dir) > 0.0f) n.negate();
    }
    hit.normal = n;

    return hit;
}

RaycastHit3D raycast(const Ray3D& ray, const Plane3D& plane) noexcept {
    RaycastHit3D hit{};
    hit.hit = false;
    hit.t = std::numeric_limits<float>::quiet_NaN();
    hit.point = Point3D();
    hit.normal = Vector3D();

    if (!ray.isValid() || !plane.isValid()) {
        return hit;
    }

    const float denom = plane.normal.dot(ray.dir);
    const float num = -(plane.normal.x * ray.origin.x +
                        plane.normal.y * ray.origin.y +
                        plane.normal.z * ray.origin.z + plane.d);

    if (std::fabs(denom) < EPS) {
        return hit;
    }

    const float t = num / denom;
    if (t < -EPS) {
        return hit;
    }

    hit.hit = true;
    hit.t = t;
    hit.point = ray.pointAt(t);
    hit.normal = plane.normal.normalized();
    if (hit.normal.dot(ray.dir) > 0.0f) hit.normal.negate();

    return hit;
}

RaycastHit3D raycast(const Ray3D& ray, const Sphere& sphere) noexcept {
    RaycastHit3D hit{};
    hit.hit = false;
    hit.t = std::numeric_limits<float>::quiet_NaN();
    hit.point = Point3D();
    hit.normal = Vector3D();

    if (!ray.isValid() || !sphere.isValid()) {
        return hit;
    }

    const Vector3D oc = ray.origin - sphere.center;
    const float a = ray.dir.dot(ray.dir);
    const float b = 2.0f * oc.dot(ray.dir);
    const float c = oc.dot(oc) - sphere.radius * sphere.radius;

    const float disc = b * b - 4.0f * a * c;
    if (disc < 0.0f) {
        return hit;
    }

    const float sdisc = std::sqrt(disc);
    float t = (-b - sdisc) / (2.0f * a);
    if (t < 0.0f) {
        t = (-b + sdisc) / (2.0f * a);
    }
    if (t < 0.0f) {
        return hit;
    }

    hit.hit = true;
    hit.t = t;
    hit.point = ray.pointAt(t);
    hit.normal = (hit.point - sphere.center).normalized();
    if (hit.normal.dot(ray.dir) > 0.0f) hit.normal.negate();

    return hit;
}

RaycastHit3D raycast(const Ray3D& ray, const AABB3D& box) noexcept {
    RaycastHit3D hit{};
    hit.hit = false;
    hit.t = std::numeric_limits<float>::quiet_NaN();
    hit.point = Point3D();
    hit.normal = Vector3D();

    if (!ray.isValid() || !box.isValid()) {
        return hit;
    }

    float tmin = 0.0f;
    float tmax = std::numeric_limits<float>::infinity();
    Vector3D nearNormal(0.0f, 0.0f, 0.0f);

    const auto axis_test = [&](float origin, float dir, float minv, float maxv,
                               Vector3D negN, Vector3D posN,
                               float& io_tmin, float& io_tmax, Vector3D& io_nearNormal) -> bool {
        if (std::fabs(dir) < EPS) {
            return origin >= minv - EPS && origin <= maxv + EPS;
        }

        float t1 = (minv - origin) / dir;
        float t2 = (maxv - origin) / dir;
        Vector3D n1 = negN;
        Vector3D n2 = posN;

        if (t1 > t2) {
            const float tmp = t1; t1 = t2; t2 = tmp;
            const Vector3D tn = n1; n1 = n2; n2 = tn;
        }

        if (t1 > io_tmin) {
            io_tmin = t1;
            io_nearNormal = n1;
        }
        if (t2 < io_tmax) {
            io_tmax = t2;
        }

        return io_tmin <= io_tmax + EPS;
    };

    if (!axis_test(ray.origin.x, ray.dir.x, box.min.x, box.max.x,
                   Vector3D(-1,0,0), Vector3D(1,0,0),
                   tmin, tmax, nearNormal)) return hit;

    if (!axis_test(ray.origin.y, ray.dir.y, box.min.y, box.max.y,
                   Vector3D(0,-1,0), Vector3D(0,1,0),
                   tmin, tmax, nearNormal)) return hit;

    if (!axis_test(ray.origin.z, ray.dir.z, box.min.z, box.max.z,
                   Vector3D(0,0,-1), Vector3D(0,0,1),
                   tmin, tmax, nearNormal)) return hit;

    if (tmax < 0.0f) {
        return hit;
    }

    hit.hit = true;
    hit.t = tmin >= 0.0f ? tmin : tmax;
    hit.point = ray.pointAt(hit.t);
    hit.normal = nearNormal;

    return hit;
}

}