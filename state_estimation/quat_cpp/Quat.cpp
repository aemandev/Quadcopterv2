#include "Quat.hpp"
#include <cmath>
#include <iostream>
#include <vector>

Quat::Quat() : w(1.0), x(0), y(0), z(0) {}
Quat::Quat(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}
Quat::Quat(std::vector<float> v, const float &dt)
{
    double a;
    std::vector<double> e;
    a = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) * dt;
    e = {v[0] / (a / dt) * sin(a / 2),
         v[1] / (a / dt) * sin(a / 2),
         v[2] / (a / dt) * sin(a / 2)};
    w = cos(a / 2);
    x = e[0];
    y = e[1];
    z = e[2];

    // Normalize the quaternion
    norm();
}
Quat::Quat(std::vector<float> v)
{
    double a;
    std::vector<double> e;
    a = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    e = {v[0] / a * sin(a / 2),
         v[1] / a * sin(a / 2),
         v[2] / a * sin(a / 2)};
    w = cos(a / 2);
    x = e[0];
    y = e[1];
    z = e[2];
    norm();
}
Quat &Quat::conjugate()
{
    w = w;
    x = -x;
    y = -y;
    z = -z;
    return *this;
}
Quat &Quat::inverse()
{
    float mag = sqrt(w * w + x * x + y * y + z * z);
    w = w / mag;
    x = -x / mag;
    y = -y / mag;
    z = -z / mag;
    return *this;
}
Quat &Quat::norm()
{
    float mag = sqrt(w * w + x * x + y * y + z * z);
    w = w / mag;
    x = x / mag;
    y = y / mag;
    z = z / mag;
    return *this;
}

Quat Quat::operator*(const Quat &q2) const
{
    float w1 = w;
    float x1 = x;
    float y1 = y;
    float z1 = z;
    float w2 = q2.w;
    float x2 = q2.x;
    float y2 = q2.y;
    float z2 = q2.z;
    Quat q3;
    q3.w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    q3.x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    q3.y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    q3.z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
    return q3;
}

Quat Quat::operator+(const Quat &q2) const
{
    Quat q3;
    q3.w = w + q2.w;
    q3.x = x + q2.x;
    q3.y = y + q2.y;
    q3.z = z + q2.z;
    return q3;
}

Quat Quat::operator-(const Quat &q2) const
{
    Quat q3;
    q3.w = w - q2.w;
    q3.x = x - q2.x;
    q3.y = y - q2.y;
    q3.z = z - q2.z;
    return q3;
}

std::vector<float> Quat::rotate_vector(std::vector<float>& v)
{
    Quat v2(0, v[0], v[1], v[2]);
    Quat qOut = this->inverse() * (v2 * *this);
    std::vector<float> vOut = {qOut.x, qOut.y, qOut.z};
    return vOut;
}

std::ostream &operator<<(std::ostream &os, const Quat &quat)
{
    std::vector<float> quatComponents{quat.w, quat.x, quat.y, quat.z};

    os << "[";
    for (int i = 0; i < quatComponents.size(); ++i)
    {
        os << quatComponents[i];
        if (i != quatComponents.size() - 1)
        {
            os << ", ";
        }
    }
    os << "]";

    return os;
}
