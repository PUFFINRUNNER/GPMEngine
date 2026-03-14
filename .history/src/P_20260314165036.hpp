#ifndef P_HPP
#define P_HPP

class Point2D {
  public:
    float v;
    float w;
    Point2D(): v(0), w(0) {}
    Point2D(float _v, float _w): v(_v), w(_w) {}
    Point2D &operator=(const Point2D &other) {}
    void operator==(const Point2D &other) const {}
    void operator<(const Point2D &other) const {}
    void operator>(const Point2D &other) const {}
    void operator!=(const Point2D &other) const {}
    Point2D operator+(const Point2D &other) const {}
    Point2D operator-(const Point2D &other) const {}
    Point2D operator*(const Point2D &other) const {}
    Point2D operator/(const Point2D &other) const {}
    Point2D &operator+=(const Point2D &other) {}
    Point2D &operator-=(const Point2D &other) {}
    Point2D &operator*=(const Point2D &other) {}
    Point2D &operator/=(const Point2D &other) {}
};

class Vector2D {
    float v;
    float w;
  public:
    
};

#endif