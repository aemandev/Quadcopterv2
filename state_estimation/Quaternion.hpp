#include <cstdint>

class Vector
{
public:
  float x;
  float y;
  float z;

  Vector();
  Vector(float x_, float y_, float z_);

  float norm() const;
  float sqrd_norm() const;
  Vector& normalize();
  Vector normalized() const;

  float dot(const Vector& v) const;
  Vector cross(const Vector& v) const;

  Vector operator*(float s) const;
  Vector operator/(float s) const;
  Vector& operator*=(float s);
  Vector& operator/=(float s);
  Vector operator+(const Vector& v) const;
  Vector operator-(const Vector& v) const;
  Vector& operator+=(const Vector& v);
  Vector& operator-=(const Vector& v);
};

inline Vector operator*(float s, const Vector& v)
{
  return v * s;
}
inline Vector operator/(float s, const Vector& v)
{
  return v / s;
}


class Quaternion
{
public:
  float w;
  float x;
  float y;
  float z;

  Quaternion();
  Quaternion(float w_, float x_, float y_, float z_);
  Quaternion(const Vector& u, const Vector& v);
  Quaternion(float roll, float pitch, float yaw);

  Vector rotate(const Vector& v) const;
  Quaternion& normalize();
  Quaternion inverse() const;
  Quaternion& invert();
  Quaternion& from_two_unit_vectors(const Vector& u, const Vector& v);
  Quaternion& from_RPY(float roll, float pitch, float yaw);
  void get_RPY(float* roll, float* pitch, float* yaw) const;

  Vector operator*(const Vector& v) const;
  Quaternion operator*(const Quaternion& q) const;
  Quaternion& operator*=(const Quaternion& q);
  Vector boxminus(const Quaternion& q) const;
//   static Vector log(const Quaternion& q)
//
private:
  float w_;
  float x_;
  float y_;
  float z_;
};

