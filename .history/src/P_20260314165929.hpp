#ifndef P_HPP
#define P_HPP

class Vector2D {
  public:
    float v;
    float w;
    Vector2D(): v(0), w(0) {}
    Vector2D(float _v, float _w): v(_v), w(_w) {}
    float magnitude();
    float polar()
};

class Point2D {
  public:
    float v;
    float w;
    Point2D(): v(0), w(0) {}
    Point2D(float _v, float _w): v(_v), w(_w) {}
    Point2D &operator=(const Point2D &other) = default;
    bool operator==(const Point2D &other) const;
    bool operator<(const Point2D &other) const;
    bool operator>(const Point2D &other) const;
    bool operator!=(const Point2D &other) const;
    Point2D operator+(const Vector2D &other) const;
    Vector2D operator-(const Point2D &other) const;
    Point2D &operator+=(const Vector2D &other);
    ~Point2D() = default;
};

#endif