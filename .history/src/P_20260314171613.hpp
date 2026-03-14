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
    bool operator!=(const Vector2D &other) const;   //check if not equal with another vector
    bool operator<(const Vector2D &other) const;    //check if is shorter than the other vector
    bool operator>(const Vector2D &other) const;    //check if is longer than the other vector
    Vector2D operator+(const Vector2D &other) const;    //add other vector
    Vector2D operator-(const Vector2D &other) const;    //subtract other vector
    Vector2D &operator+=(const Vector2D &other);    //^
    Vector2D &operator-=(const Vector2D &other);    //^
    Vector2D operator*(const float &x) const;   //multiply magintude by x
    Vector2D operator/(const float &x) const;   //divide magnitude by x
    Vector2D &operator*=(const float &x);   //^
    Vector2D &operator/=(const float &x);   //^
    Vector2D atMagnitude(const float &mag) const;   //return a vector that has the same direction as this but with magnitude mag
    Vector2D setMagnitude(const float &mag);    //set magnitude to mag
    float getAngleBetween(const Vector2D &toher) const; //get the smallest not negative angle between 2 vectors
    Vector2D negative() const;  //return a parallel vector in the opposite direction
    Vector2D negativeSelf();    //turn vector 180 degrees
    float magnitudeSqr() const; //return a squared magnitude
    Vector2D normalized() const;//return a normalized vector with the same direction as this
    Vector2D normalize();   //normalize this vector
    float dot(const Vector2D& other) const; //dot product
    Vector2D rotated(float angle) const;    //return a rotated vector with the same magnitude
    Vector2D rotate(float angle);   //rotate this vector
    Vector2D perpendicular() const; //return a perpendicular vector
    
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