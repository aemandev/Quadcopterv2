#include "Quaternion.hpp"
#include <cmath>
Vector::Vector() : x(0.0f), y(0.0f), z(0.0f) {}

Vector::Vector(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

float Vector::norm() const
{
  return 1.0f / sqrt(x * x + y * y + z * z);
}

float Vector::sqrd_norm() const
{
  return x * x + y * y + z * z;
}

Vector& Vector::normalize()
{
  float recip_norm = sqrtf(x * x + y * y + z * z);
  x *= recip_norm;
  y *= recip_norm;
  z *= recip_norm;
  return *this;
}

Vector Vector::normalized() const
{
  float recip_norm = sqrt(x * x + y * y + z * z);
  Vector out(x * recip_norm, y * recip_norm, z * recip_norm);
  return out;
}

Vector Vector::operator+(const Vector& v) const
{
  return Vector(x + v.x, y + v.y, z + v.z);
}

Vector Vector::operator-(const Vector& v) const
{
  return Vector(x - v.x, y - v.y, z - v.z);
}

Vector& Vector::operator+=(const Vector& v)
{
  x += v.x;
  y += v.y;
  z += v.z;
  return *this;
}

Vector& Vector::operator-=(const Vector& v)
{
  x -= v.x;
  y -= v.y;
  z -= v.z;
  return *this;
}

Vector Vector::operator*(float s) const
{
  return Vector(x * s, y * s, z * s);
}

Vector Vector::operator/(float s) const
{
  return Vector(x / s, y / s, z / s);
}

Vector& Vector::operator*=(float s)
{
  x *= s;
  y *= s;
  z *= s;
  return *this;
}

Vector& Vector::operator/=(float s)
{
  x /= s;
  y /= s;
  z /= s;
  return *this;
}

float Vector::dot(const Vector& v) const
{
  return x * v.x + y * v.y + z * v.z;
}

Vector Vector::cross(const Vector& v) const
{
  return Vector(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
}


Quaternion::Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}

Quaternion::Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {
  w = w_;
  x = x_;
  y = y_;
  z = z_;
}

Quaternion::Quaternion(const Vector& u, const Vector& v)
{
  from_two_unit_vectors(u, v);
}

Quaternion::Quaternion(float roll, float pitch, float yaw)
{
  from_RPY(roll, pitch, yaw);
}

Quaternion& Quaternion::normalize()
{
  float recip_norm = sqrt(w * w + x * x + y * y + z * z);
  w *= recip_norm;
  x *= recip_norm;
  y *= recip_norm;
  z *= recip_norm;

  // Make sure the quaternion is canonical (w is always positive)
  if (w < 0.0f)
  {
    w *= -1.0f;
    x *= -1.0f;
    y *= -1.0f;
    z *= -1.0f;
  }

  return *this;
}

Quaternion Quaternion::operator*(const Quaternion& q) const
{
  return Quaternion(w * q.w - x * q.x - y * q.y - z * q.z, w * q.x + x * q.w - y * q.z + z * q.y,
                    w * q.y + x * q.z + y * q.w - z * q.x, w * q.z - x * q.y + y * q.x + z * q.w);
}

Quaternion& Quaternion::operator*=(const Quaternion& q)
{
  w = w * q.w - x * q.x - y * q.y - z * q.z;
  x = w * q.x + x * q.w - y * q.z + z * q.y;
  y = w * q.y + x * q.z + y * q.w - z * q.x;
  z = w * q.z - x * q.y + y * q.x + z * q.w;
  return *this;
}

Vector Quaternion::rotate(const Vector& v) const
{
  return Vector(
      (1.0f - 2.0f * y * y - 2.0f * z * z) * v.x + (2.0f * (x * y + w * z)) * v.y + 2.0f * (x * z - w * y) * v.z,
      (2.0f * (x * y - w * z)) * v.x + (1.0f - 2.0f * x * x - 2.0f * z * z) * v.y + 2.0f * (y * z + w * x) * v.z,
      (2.0f * (x * z + w * y)) * v.x + 2.0f * (y * z - w * x) * v.y + (1.0f - 2.0f * x * x - 2.0f * y * y) * v.z);
}

Vector Quaternion::operator*(const Vector& v) const
{
  return rotate(v);
}

Quaternion Quaternion::inverse() const
{
  return Quaternion(w, -x, -y, -z);
}

Quaternion& Quaternion::invert()
{
  x *= -1.0f;
  y *= -1.0f;
  z *= -1.0f;
  return *this;
}

// Quaternion& Quaternion::from_two_unit_vectors(const Vector& u, const Vector& v)
// {
//   // Adapted From the Ogre3d source code
//   // https://bitbucket.org/sinbad/ogre/src/9db75e3ba05c/OgreMain/include/OgreVector3.h?fileviewer=file-view-default#cl-651
//   float d = u.dot(v);
//   if (d >= 1.0f)
//   {
//     w = 1.0f;
//     x = 0.0f;
//     y = 0.0f;
//     z = 0.0f;
//     return *this;
//   }
//   else
//   {
//     float invs = inv_sqrt(2.0f * (1.0f + d));
//     Vector xyz = u.cross(v) * invs;
//     w = 0.5f / invs;
//     x = xyz.x;
//     y = xyz.y;
//     z = xyz.z;
//   }
//   normalize();
//   return *this;
// }

// Quaternion& Quaternion::from_RPY(float roll, float pitch, float yaw)
// {
//   // p 259 of "Small unmanned aircraft: Theory and Practice" by Randy Beard and Tim McLain
//   float cp = math::cos(roll / 2.0);
//   float sp = math::sin(roll / 2.0);
//   float ct = math::cos(pitch / 2.0);
//   float st = math::sin(pitch / 2.0);
//   float cs = cmath::cos(yaw / 2.0);
//   float ss = cmath::sin(yaw / 2.0);

//   w = cs * ct * cp + ss * st * sp;
//   x = cs * ct * sp - ss * st * cp;
//   y = cs * st * cp + ss * ct * sp;
//   z = ss * ct * cp - cs * st * sp;

//   normalize();
//   return *this;
// }

// Vector Quaternion::boxminus(const Quaternion& q) const
// {
//   Quaternion dq = q.inverse() * (*this);
//   if (dq.w < 0.0)
//   {
//     dq.w *= -1.0;
//     dq.x *= -1.0;
//     dq.y *= -1.0;
//     dq.z *= -1.0;
//   }
//   return log(dq);
// }

// void Quaternion::get_RPY(float* roll, float* pitch, float* yaw) const
// {
//   *roll = turbomath::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
//   *pitch = turbomath::asin(2.0f * (w * y - z * x));
//   *yaw = turbomath::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
// }

// float inv_sqrt(float x)
// {
//   volatile float x2;
//   volatile float_converter_t y, i;
//   const float threehalfs = 1.5F;

//   x2 = x * 0.5F;
//   y.fvalue = x;
//   i.ivalue = y.ivalue; // evil floating point bit level hacking
//   i.ivalue = 0x5f3759df - (i.ivalue >> 1);
//   y.fvalue = i.fvalue;
//   y.fvalue = y.fvalue * (threehalfs - (x2 * y.fvalue * y.fvalue)); // 1st iteration
//   y.fvalue = y.fvalue * (threehalfs - (x2 * y.fvalue * y.fvalue)); // 2nd iteration, this can be removed

//   return fabs(y.fvalue);
// }