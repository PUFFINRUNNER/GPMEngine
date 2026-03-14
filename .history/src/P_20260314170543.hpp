#ifndef P_HPP
#define P_HPP

class Vector2D {
  public:
    float v;
    float w;
    Vector2D(): v(0), w(0) {}
    Vector2D(float _v, float _w): v(_v), w(_w) {}
    float magnitude();  //get magnitude
    float polar();      //get polar angle
    bool operator==(const Vector2D &other) const;   //check if equal with another vector
    bool operator<(const Vector2D &other) const;    //check if is shorter than the other vector
    bool operator>(const Vector2D &other) const;    //check if is longer than the other vector
    Vector2D operator+(const Vector2D &other) const;    //add other vector
    Vector2D operator-(const Vector2D &other) const;    //subtract other vector
    Vector2D &operator+=(const Vector2D &other);    
    Vector2D &operator-=(const Vector2D &other);
    Vector2D operator*(const float &other) const;
    Vector2D operator/(const float &other) const;
    Vector2D &operator*=(const float &other);
    Vector2D &operator/=(const float &other);
    Vector2D atMagnitude(float);
    void setMagnitude(float);
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