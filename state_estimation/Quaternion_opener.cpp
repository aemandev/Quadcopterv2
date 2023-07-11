#include "Quaternion.hpp"
#include <iostream>
#include <cmath>

using namespace std;

Quaternion::Quaternion(float w, float x, float y, float z) 
:w_(w), x_(x), y_(y), z_(z) 
{Set(w, x, y, z);}

Quaternion::Quaternion()
:w(1.0f), x(0.0f), y(0.0f), z(0.0f) {
    Set(w, x, y, z);
}

Quaternion::Quaternion(const quat_data_t q_array_wxyz[4]) {
    Set(q_array_wxyz);
}
void Quaternion::Set(float w, float x, float y, float z) {
    w_ = w;
    x_ = x;
    y_ = y;
    z_ = z;
}

void Quaternion::Set(const quat_data_t q_array_wxyz[4]){
  Set(q_array_wxyz[0].q_array_wxyz[0], q_array_wxyz[0].q_array_wxyz[1], q_array_wxyz[0].q_array_wxyz[2], q_array_wxyz[0].q_array_wxyz[3]);
}

Quaternion::Quaternion(const Quaternion &q)
:w_(q.w_), x_(q.x_), y_(q.y_), z_(q.z_) {
    w_ = q.W();
    x_ = q.X();
    y_ = q.Y();
    z_ = q.Z();

}

float Quaternion::W() const {return w_;}
float Quaternion::X() const {return x_;}
float Quaternion::Y() const {return y_;}
float Quaternion::Z() const {return z_;}

// Quaternion::Quaternion(float roll, float pitch, float yaw)
// {
//   from_RPY(roll, pitch, yaw);
// }

Quaternion Quaternion::normalize() const{
    float norm = sqrt(w_*w_ + x_*x_ + y_*y_ + z_*z_);
    float w_norm = w_/norm;
    float x_norm = x_/norm;
    float y_norm = y_/norm;
    float z_norm = z_/norm;
    return Quaternion(w_norm, x_norm, y_norm, z_norm);
}

Quaternion Quaternion::conjugate() const{
    return Quaternion(w_, -x_, -y_, -z_);
}
Quaternion& Quaternion::invert() {
    x *= -1;
    y *= -1;
    z *= -1;
    return *this;
}

Quaternion& Quaternion::from_RPY(float roll, float pitch, float yaw)
{
  // p 259 of "Small unmanned aircraft: Theory and Practice" by Randy Beard and Tim McLain
  float cp = cos(roll / 2.0);
  float sp = sin(roll / 2.0);
  float ct = cos(pitch / 2.0);
  float st = sin(pitch / 2.0);
  float cs = cos(yaw / 2.0);
  float ss = sin(yaw / 2.0);

  w = cs * ct * cp + ss * st * sp;
  x = cs * ct * sp - ss * st * cp;
  y = cs * st * cp + ss * ct * sp;
  z = ss * ct * cp - cs * st * sp;

  normalize();
  return *this;
}

void Quaternion::ToArray(quat_data_t q_array[4]) const {
    q_array[0].q_array_wxyz[0] = w_;
    q_array[0].q_array_wxyz[1] = x_;
    q_array[0].q_array_wxyz[2] = y_;
    q_array[0].q_array_wxyz[3] = z_;
}