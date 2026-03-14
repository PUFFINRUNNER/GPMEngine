#ifndef P_HPP
#define P_HPP

#include <cmath>

namespace P {
    inline constexpr float EPS = 1e-6f;
    class Vector2D {
    public:
        float v;
        float w;
        constexpr Vector2D(): v(0), w(0) {}
        constexpr Vector2D(float _v, float _w): v(_v), w(_w) {}
        [[nodiscard]] float magnitude() const noexcept;    //get magnitude
        [[nodiscard]] float angle() const noexcept;    //get polar angle
        [[nodiscard]] bool operator==(const Vector2D &other) const noexcept {  //check if equal with another vector
            return std::fabs(v - other.v) < EPS && std::fabs(w - other.w) < EPS;
        }
        [[nodiscard]] bool operator!=(const Vector2D &other) const noexcept {  //check if not equal with another vector
            return !(*this == other);
        }
        [[nodiscard]] constexpr Vector2D operator+(const Vector2D &other) const noexcept {   //add other vector
            return Vector2D(v + other.v, w + other.w);
        }
        [[nodiscard]] constexpr Vector2D operator-(const Vector2D &other) const noexcept {   //subtract other vector
            return Vector2D(v - other.v, w - other.w);
        }
        [[nodiscard]] constexpr Vector2D operator-() const noexcept {//instead of writing "negated"
            return Vector2D(-v, -w);
        }
        Vector2D &operator+=(const Vector2D &other) noexcept;    //^
        Vector2D &operator-=(const Vector2D &other) noexcept;    //^
        [[nodiscard]] constexpr Vector2D operator*(float x) const noexcept {  //multiply magintude by x
            return Vector2D(v*x, w*x);
        }
        [[nodiscard]] Vector2D operator/(float x) const noexcept {  //divide magnitude by x
            if (x > -EPS && x < EPS) {
                return Vector2D(std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN());
            }
            return Vector2D(v/x, w/x);
        }
        Vector2D &operator*=(float x) noexcept;   //^
        Vector2D &operator/=(float x) noexcept;   //^
        [[nodiscard]] Vector2D atMagnitude(float mag) const noexcept;   //return a vector that has the same direction as this but with magnitude mag
        Vector2D &setMagnitude(float mag) noexcept;    //set magnitude to mag
        [[nodiscard]] float angleBetween(const Vector2D &other) const noexcept; //get the smallest not negative angle between 2 vectors
        [[nodiscard]] Vector2D negated() const noexcept;  //return a parallel vector in the opposite direction
        Vector2D &negate() noexcept;    //turn vector 180 degrees
        [[nodiscard]] float magnitudeSqr() const noexcept; //return a squared magnitude
        [[nodiscard]] Vector2D normalized() const noexcept;//return a normalized vector with the same direction as this
        Vector2D &normalize() noexcept;   //normalize this vector
        [[nodiscard]] float dot(const Vector2D& other) const noexcept; //dot product
        [[nodiscard]] float cross(const Vector2D& other) const noexcept;   //cross product
        [[nodiscard]] Vector2D rotated(float angle) const noexcept;    //return a rotated vector with the same magnitude
        Vector2D &rotate(float angle) noexcept;   //rotate this vector
        [[nodiscard]] Vector2D perpendicular() const noexcept; //return a perpendicular vector
        Vector2D &makePerpendicular() noexcept;   //make this vector perpendicular to itself
        [[nodiscard]] bool isParallel(const Vector2D &other) const noexcept; //check if this and other are parallel
        [[nodiscard]] bool isPerpendicular(const Vector2D &other) const noexcept; //check if this and other are perpendicular
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
};

#endif