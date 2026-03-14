#ifndef P_HPP
#define P_HPP

namespace P {
    inline constexpr float DEFAULT_EPS = 1e-6f;
    class Vector2D {
    public:
        float v;
        float w;
        constexpr Vector2D(): v(0), w(0) {}
        constexpr Vector2D(float _v, float _w): v(_v), w(_w) {}
        float magnitude() const;    //get magnitude
        float angle() const;    //get polar angle
        bool operator==(const Vector2D &other) const;   //check if equal with another vector
        bool operator!=(const Vector2D &other) const;   //check if not equal with another vector
        constexpr Vector2D operator+(const Vector2D &other) const {   //add other vector
            
            
        constexpr Vector2D operator-(const Vector2D &other) const;    //subtract other vector
        constexpr Vector2D operator-() const; //instead of writing "negated"
        Vector2D &operator+=(const Vector2D &other);    //^
        Vector2D &operator-=(const Vector2D &other);    //^
        constexpr Vector2D operator*(float x) const;   //multiply magintude by x
        constexpr Vector2D operator/(float x) const;   //divide magnitude by x
        Vector2D &operator*=(float x);   //^
        Vector2D &operator/=(float x);   //^
        Vector2D atMagnitude(float mag) const;   //return a vector that has the same direction as this but with magnitude mag
        Vector2D &setMagnitude(float mag);    //set magnitude to mag
        float getAngleBetween(const Vector2D &other) const; //get the smallest not negative angle between 2 vectors
        Vector2D negated() const;  //return a parallel vector in the opposite direction
        Vector2D &negate();    //turn vector 180 degrees
        float magnitudeSqr() const; //return a squared magnitude
        Vector2D normalized() const;//return a normalized vector with the same direction as this
        Vector2D &normalize();   //normalize this vector
        float dot(const Vector2D& other) const; //dot product
        float cross(const Vector2D& other) const;   //cross product
        Vector2D rotated(float angle) const;    //return a rotated vector with the same magnitude
        Vector2D &rotate(float angle);   //rotate this vector
        Vector2D perpendicular() const; //return a perpendicular vector
        Vector2D &makePerpendicular();   //make this vector perpendicular to itself
        bool isParallel(const Vector2D &other) const; //check if this and other are parallel
        bool isPerpendicular(const Vector2D &other) const; //check if this and other are perpendicular
    };

    
    class Point2D {
    public:
        float v;
        float w;
        constexpr Point2D() : v(0), w(0) {}
        constexpr Point2D(float _v, float _w) : v(_v), w(_w) {}
        Point2D& operator=(const Point2D& other) = default;
        bool operator==(const Point2D& other) const;
        bool operator!=(const Point2D& other) const;
        constexpr Point2D operator+(const Vector2D& other) const;
        constexpr Point2D operator-(const Vector2D& other) const;
        constexpr Vector2D operator-(const Point2D& other) const;
        Point2D& operator+=(const Vector2D& other);
        Point2D& operator-=(const Vector2D& other);
        float distanceTo(const Point2D& other) const;
        float distanceSqrTo(const Point2D& other) const;
        ~Point2D() = default;
    };
};

#endif