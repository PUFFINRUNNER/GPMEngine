#include "P.hpp"

namespace P {

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
    return std::fabs(cross(other)) < EPS;
}

bool Vector2D::isPerpendicular(const Vector2D& other) const noexcept {
    return std::fabs(dot(other)) < EPS;
}

// =========================
// Point2D
// =========================

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

}